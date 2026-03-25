"""
DriveState — move in a direction for a fixed duration.

Generic timed movement state. Used for driving through gates,
approaching targets, or any open-loop timed maneuver.

The RC controller has a trapezoidal velocity ramp, so sending a new
movement command while already moving produces a smooth transition
without jerking. Set stop_after=False when the next state will
immediately send another movement command.

Outcomes:
    "done"  — duration elapsed

Blackboard reads:
    ctx              PlannerContext
    drive_command    str   — command name (default "move_forward")
    drive_duration   float — seconds to drive (default cfg.drive_through_time)
    drive_speed      int   — PWM offset (optional)
    drive_heading    float — hold this heading via go_forward (optional)
    stop_after       bool  — send stop when done (default True)
"""

from __future__ import annotations

from yasmin import State, Blackboard

from ..bb_utils import bb_get

DONE = "done"


class DriveState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[DONE])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        command = bb_get(blackboard, "drive_command", "move_forward")
        duration = bb_get(blackboard, "drive_duration", ctx.cfg.drive_through_time)
        speed = bb_get(blackboard, "drive_speed", ctx.cfg.default_speed)
        heading = bb_get(blackboard, "drive_heading", None)
        stop_after = bb_get(blackboard, "stop_after", True)

        if heading is not None:
            ctx.log(f"DRIVE — go_forward heading={heading:.1f}° "
                    f"dur={duration}s spd={speed}")
            ctx.send('go_forward', angle=heading, duration=duration, speed=speed)
        else:
            ctx.log(f"DRIVE — {command} dur={duration}s spd={speed}")
            ctx.send(command, duration=duration, speed=speed)

        ctx.sleep(duration)

        if stop_after:
            ctx.send('stop')

        ctx.log("DRIVE — complete")
        return DONE
