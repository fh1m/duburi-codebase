"""
SearchState — rotate in place looking for a target class.

Yaw-sweeps in increments until the vision pipeline detects the
requested object class, or until the search timeout expires.

Outcomes:
    "found"     — target_class detected in /vision/detections
    "timeout"   — search_timeout elapsed without detection

Blackboard reads:
    ctx             PlannerContext
    target_class    str   — YOLO class name to look for
    search_yaw_step float — degrees per sweep step  (optional)
    search_speed    int   — rotation speed           (optional)
    search_timeout  float — max search time          (optional)
"""

from __future__ import annotations

import time

from yasmin import State, Blackboard

from ..bb_utils import bb_get

FOUND = "found"
TIMEOUT = "timeout"


class SearchState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[FOUND, TIMEOUT])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        target = blackboard["target_class"]
        yaw_step = bb_get(blackboard, "search_yaw_step", 30.0)
        speed = bb_get(blackboard, "search_speed", ctx.cfg.default_speed)
        timeout = bb_get(blackboard, "search_timeout", ctx.cfg.search_timeout)

        ctx.log(f"SEARCH — scanning for '{target}' (timeout={timeout}s)")

        deadline = time.monotonic() + timeout
        direction = 1  # 1 = right, -1 = left (zigzag)
        steps_this_sweep = 0
        max_steps_per_sweep = int(360 / yaw_step) + 1

        while time.monotonic() < deadline:
            if ctx.has_detection(target):
                ctx.log(f"SEARCH — '{target}' detected!")
                return FOUND

            # Rotate one step
            current = ctx.heading
            delta = yaw_step * direction
            new_heading = (current + delta) % 360
            ctx.send('pid_yaw_to_heading', angle=new_heading, speed=speed)
            ctx.sleep(2.0)

            steps_this_sweep += 1
            if steps_this_sweep >= max_steps_per_sweep:
                direction *= -1
                steps_this_sweep = 0
                ctx.log("SEARCH — reversing sweep direction")

        ctx.warn(f"SEARCH — '{target}' not found within {timeout}s")
        return TIMEOUT
