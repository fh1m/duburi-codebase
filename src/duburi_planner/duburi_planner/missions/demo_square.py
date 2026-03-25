"""
Demo square mission — drive forward + PID turn 90° × 4 to trace a square.

Purpose: end-to-end smoke test for the planner and a template showing
how to build YASMIN missions for Duburi.

The RC controller has a trapezoidal velocity ramp. Sending a new
movement command while already moving produces a SMOOTH transition
(the ramp handles accel/decel). We do NOT send "stop" between legs
and turns — that would jerk the thrusters to zero and restart.
Only the final TURN_4 sends stop to end the mission cleanly.

═══════════════════════════════════════════════════════════════════════
  HOW TO CUSTOMIZE THIS MISSION (YASMIN mini-guide)
═══════════════════════════════════════════════════════════════════════

  1. CHANGE PARAMETERS — edit the defaults in _setup_demo() or
     override them on the Blackboard before calling build_demo_square():

         blackboard["leg_duration"] = 5.0    # longer legs
         blackboard["turn_angle"]   = 45.0   # octagon instead of square
         blackboard["demo_speed"]   = 60     # faster
         blackboard["turn_settle"]  = 3.0    # more time to complete turn

  2. ADD A STATE — e.g. add a pause between legs:

         from yasmin import CbState
         def pause(bb): ctx = bb["ctx"]; ctx.sleep(2); return "done"
         sm.add_state("PAUSE_1", CbState(["done"], pause),
                      transitions={"done": "TURN_1"})
         # Then change LEG_1's transition: {"leg_done": "PAUSE_1"}

  3. ADD AN OUTCOME — e.g. abort if heading drifts too much:

         class CheckHeadingState(State):
             def __init__(self):
                 super().__init__(outcomes=["ok", "drifted"])
             def execute(self, blackboard):
                 ctx = blackboard["ctx"]
                 if abs(ctx.heading - blackboard["expected"]) > 20:
                     return "drifted"
                 return "ok"

  4. NEST THIS IN A LARGER MISSION — this SM is a sub-SM:

         top = StateMachine(outcomes=["done"])
         top.add_state("ARM", ArmState(), {"armed": "SQUARE", ...})
         top.add_state("SQUARE", build_demo_square(),
                       {"square_done": "done"})

  See the YASMIN docs: https://uleroboticsgroup.github.io/yasmin/4.2.4/
  See the Duburi planner README: src/duburi_planner/README.md
═══════════════════════════════════════════════════════════════════════

Sub-state-machine:
    SETUP → LEG_1 → TURN_1 → LEG_2 → TURN_2 → LEG_3 → TURN_3 → LEG_4 → TURN_4 → (done)

Outcomes:
    "square_done"  — all four legs completed
"""

from __future__ import annotations

from yasmin import StateMachine, State, Blackboard, CbState

from ..bb_utils import bb_get

SQUARE_DONE = "square_done"


class DemoLegState(State):
    """Drive forward for a set duration.

    Does NOT send "stop" at the end — the next state (a turn) will
    send its own command, and the RC ramp handles the smooth transition.
    """

    def __init__(self, leg_number: int) -> None:
        super().__init__(outcomes=["leg_done"])
        self._leg = leg_number

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        duration = bb_get(blackboard, "leg_duration", 3.0)
        speed = bb_get(blackboard, "demo_speed", 40)

        ctx.log(f"DEMO LEG {self._leg} — move_forward "
                f"dur={duration}s spd={speed}")
        ctx.send('move_forward', duration=duration, speed=speed)
        ctx.sleep(duration)
        return "leg_done"


class DemoTurnState(State):
    """PID turn 90° right relative to current heading.

    Does NOT send "stop" before or after — the PID yaw command
    replaces the forward channel via the ramp, and the next leg
    command will smoothly ramp into forward thrust.

    The last turn (is_final=True) sends "stop" to end the mission.
    """

    def __init__(self, turn_number: int, is_final: bool = False) -> None:
        super().__init__(outcomes=["turn_done"])
        self._turn = turn_number
        self._is_final = is_final

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        turn_angle = bb_get(blackboard, "turn_angle", 90.0)
        speed = bb_get(blackboard, "demo_speed", 40)
        settle = bb_get(blackboard, "turn_settle", 2.0)

        current = ctx.heading
        target = (current + turn_angle) % 360

        ctx.log(f"DEMO TURN {self._turn} — PID yaw "
                f"{current:.1f}° → {target:.1f}° (+{turn_angle}°)")
        ctx.send('pid_yaw_to_heading', angle=target, speed=speed)
        ctx.sleep(settle)

        if self._is_final:
            ctx.send('stop')
            ctx.log("DEMO SQUARE — pattern complete, thrusters stopped")

        return "turn_done"


def _setup_demo(blackboard: Blackboard) -> str:
    """Inject demo parameters into the blackboard."""
    ctx = blackboard["ctx"]
    if "leg_duration" not in blackboard:
        blackboard["leg_duration"] = 3.0
    if "turn_angle" not in blackboard:
        blackboard["turn_angle"] = 90.0
    if "demo_speed" not in blackboard:
        blackboard["demo_speed"] = 40
    if "turn_settle" not in blackboard:
        blackboard["turn_settle"] = 2.0

    ctx.log("DEMO SQUARE — starting 4-leg square pattern")
    ctx.log(f"  leg_duration={blackboard['leg_duration']}s  "
            f"turn_angle={blackboard['turn_angle']}°  "
            f"speed={blackboard['demo_speed']}")
    return "ready"


def build_demo_square() -> StateMachine:
    """Construct the demo square sub-state-machine."""
    sm = StateMachine(outcomes=[SQUARE_DONE])

    sm.add_state(
        "SETUP",
        CbState(["ready"], _setup_demo),
        transitions={"ready": "LEG_1"},
    )

    for i in range(1, 5):
        sm.add_state(
            f"LEG_{i}",
            DemoLegState(i),
            transitions={"leg_done": f"TURN_{i}"},
        )

        is_final = (i == 4)
        after_turn = SQUARE_DONE if is_final else f"LEG_{i + 1}"
        sm.add_state(
            f"TURN_{i}",
            DemoTurnState(i, is_final=is_final),
            transitions={"turn_done": after_turn},
        )

    return sm
