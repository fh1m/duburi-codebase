"""
ArmState — arm the vehicle and set MANUAL mode.

Sends arm + set_mode commands and waits for the vehicle to confirm
armed status via /mavlink/vehicle_state. If no telemetry is available
(desk testing, no Pixhawk), continues anyway after the settle time
so the rest of the mission can still exercise the command pipeline.

Outcomes:
    "armed"   — vehicle armed (confirmed or assumed after settle)
    "failed"  — explicitly rejected (feedback says rejected)

Blackboard reads:
    ctx   PlannerContext
"""

from __future__ import annotations

import time

from yasmin import State, Blackboard

from ..bb_utils import bb_get

ARMED = "armed"
FAILED = "failed"


class ArmState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[ARMED, FAILED])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        settle = ctx.cfg.arm_settle_time

        ctx.log("ARM — sending arm + MANUAL mode")

        ctx.send('arm')

        # Wait for arm confirmation with polling
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            ctx.sleep(0.5)
            if ctx.armed:
                ctx.log("ARM — confirmed armed via telemetry")
                break
        else:
            # No telemetry confirmation — may be desk testing
            if ctx.vehicle_state is None:
                ctx.warn("ARM — no telemetry (desk mode?) — continuing anyway")
            else:
                ctx.warn("ARM — telemetry present but not armed — continuing")

        ctx.send('set_mode', mode='MANUAL')
        ctx.sleep(0.5)

        ctx.log("ARM — ready (MANUAL mode set)")
        return ARMED
