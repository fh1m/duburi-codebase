"""
SurfaceState — ascend to surface and disarm.

Outcomes:
    "surfaced"  — vehicle at surface and disarmed
"""

from __future__ import annotations

from yasmin import State, Blackboard

SURFACED = "surfaced"


class SurfaceState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[SURFACED])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]

        ctx.log("SURFACE — ascending and disarming")
        ctx.send('surface')
        ctx.sleep(5.0)
        ctx.send('disarm')
        ctx.sleep(2.0)
        ctx.log("SURFACE — complete")
        return SURFACED
