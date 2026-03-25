"""
SubmergeState — arm, set mode, dive to mission depth, wait to stabilize.

Sends arm + MANUAL mode, then activates PID depth hold. Waits for
depth confirmation via telemetry. If no telemetry is available (desk
testing), continues after settle time so the pipeline can be exercised.

Outcomes:
    "submerged"  — vehicle armed and at target depth (or settle elapsed)
    "failed"     — arm explicitly rejected

Blackboard reads:
    ctx             PlannerContext
    dive_depth      float (optional override, else uses cfg.dive_depth)
"""

from __future__ import annotations

import time

from yasmin import State, Blackboard

from ..bb_utils import bb_get

SUBMERGED = "submerged"
FAILED = "failed"


class SubmergeState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[SUBMERGED, FAILED])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        depth = bb_get(blackboard, "dive_depth", ctx.cfg.dive_depth)

        ctx.log(f"SUBMERGE — arming and diving to {depth:.2f} m")

        # ── Arm ──────────────────────────────────────────────────────
        ctx.send('arm')

        deadline = time.monotonic() + ctx.cfg.arm_settle_time
        while time.monotonic() < deadline:
            ctx.sleep(0.5)
            if ctx.armed:
                ctx.log("SUBMERGE — confirmed armed")
                break
        else:
            if ctx.vehicle_state is None:
                ctx.warn("SUBMERGE — no telemetry (desk mode?) — continuing")
            else:
                ctx.warn("SUBMERGE — not armed after settle — continuing")

        # ── Mode ─────────────────────────────────────────────────────
        ctx.send('set_mode', mode='MANUAL')
        ctx.sleep(0.5)

        # ── Dive ─────────────────────────────────────────────────────
        ctx.send('pid_depth', depth=depth)

        deadline = time.monotonic() + ctx.cfg.feedback_timeout
        has_telemetry = ctx.vehicle_state is not None

        while time.monotonic() < deadline:
            ctx.sleep(0.5)
            if has_telemetry:
                current = abs(ctx.depth)
                if abs(current - depth) < 0.15:
                    ctx.log(f"SUBMERGE — reached {current:.2f} m "
                            f"(target {depth:.2f} m)")
                    ctx.sleep(ctx.cfg.dive_settle_time)
                    return SUBMERGED
            else:
                # No telemetry — just wait the settle time and proceed
                ctx.warn("SUBMERGE — no depth telemetry, waiting settle time")
                ctx.sleep(ctx.cfg.dive_settle_time)
                return SUBMERGED

        ctx.warn(f"SUBMERGE — depth timeout (current={abs(ctx.depth):.2f} m) "
                 f"— proceeding anyway")
        ctx.sleep(ctx.cfg.dive_settle_time)
        return SUBMERGED
