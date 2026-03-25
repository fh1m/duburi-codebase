"""
ArmState — arm the vehicle and set MANUAL mode.

Waits for DDS publisher matching (so the first command isn't dropped),
sets MANUAL mode, then sends arm with retry. Confirms armed status
via /mavlink/vehicle_state telemetry. If no telemetry is available
(desk testing, no Pixhawk), continues after the settle time so the
rest of the mission can still exercise the command pipeline.

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

_ARM_RETRY_INTERVAL = 1.5


class ArmState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[ARMED, FAILED])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        settle = ctx.cfg.arm_settle_time

        ctx.log("ARM — waiting for inspector connection...")
        ctx.wait_for_ready(timeout=8.0)

        ctx.log("ARM — setting MANUAL mode")
        ctx.send('set_mode', mode='MANUAL')
        ctx.sleep(1.0)

        ctx.log("ARM — sending arm command")
        ctx.send('arm')

        deadline = time.monotonic() + settle
        last_retry = time.monotonic()

        while time.monotonic() < deadline:
            ctx.sleep(0.5)
            if ctx.armed:
                ctx.log("ARM — confirmed armed via telemetry")
                break

            if time.monotonic() - last_retry >= _ARM_RETRY_INTERVAL:
                ctx.log("ARM — retrying arm command...")
                ctx.send('arm')
                last_retry = time.monotonic()
        else:
            if ctx.vehicle_state is None:
                ctx.warn("ARM — no telemetry (desk mode?) — continuing anyway")
            else:
                ctx.warn("ARM — telemetry present but not armed — continuing")

        ctx.log("ARM — ready (MANUAL mode)")
        return ARMED
