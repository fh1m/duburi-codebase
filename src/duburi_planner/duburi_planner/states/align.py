"""
AlignState — activate vision alignment and wait until fully aligned.

Delegates to the existing alignment_controller via /driver/command.
Monitors /vision/alignment_status for fully_aligned==true.

Outcomes:
    "aligned"    — vision reports fully_aligned for the target
    "lost"       — target disappeared during alignment
    "timeout"    — alignment_timeout elapsed

Blackboard reads:
    ctx                PlannerContext
    target_class       str   — class being aligned to
    alignment_mode     str   — alignment command (default "pid_align")
    alignment_timeout  float — max seconds  (optional)
"""

from __future__ import annotations

import time

from yasmin import State, Blackboard

from ..bb_utils import bb_get

ALIGNED = "aligned"
LOST = "lost"
TIMEOUT = "timeout"


class AlignState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[ALIGNED, LOST, TIMEOUT])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        target = blackboard["target_class"]
        mode = bb_get(blackboard, "alignment_mode", "pid_align")
        timeout = bb_get(blackboard, "alignment_timeout", ctx.cfg.alignment_timeout)

        ctx.log(f"ALIGN — '{mode}' on '{target}' (timeout={timeout}s)")

        ctx.send(mode)

        deadline = time.monotonic() + timeout
        lost_streak = 0

        while time.monotonic() < deadline:
            ctx.sleep(0.3)

            status = ctx.alignment_status
            if status is None:
                lost_streak += 1
                if lost_streak > 10:
                    ctx.warn("ALIGN — no alignment_status messages")
                    ctx.send('vision_stop')
                    return LOST
                continue

            if status.target_class != target:
                continue

            if not status.target_detected and not status.kalman_predicted:
                lost_streak += 1
                if lost_streak > 15:
                    ctx.warn(f"ALIGN — '{target}' lost for too long")
                    ctx.send('vision_stop')
                    return LOST
                continue

            lost_streak = 0

            if status.fully_aligned:
                ctx.log(f"ALIGN — fully aligned to '{target}' "
                        f"(err_x={status.error_x:.3f} err_y={status.error_y:.3f})")
                ctx.send('vision_stop')
                return ALIGNED

        ctx.warn(f"ALIGN — timeout after {timeout}s")
        ctx.send('vision_stop')
        return TIMEOUT
