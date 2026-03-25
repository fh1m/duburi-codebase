"""
SendCommandState — publish a single DriverCommand and immediately transition.

Thin glue state for injecting one-shot commands (stop, vision_stop,
set_mode, etc.) into a state machine without writing a custom state.

Outcomes:
    "done"  — command published

Blackboard reads:
    ctx         PlannerContext
    cmd_name    str   — command name
    cmd_kwargs  dict  — extra keyword args for make_command (optional)
"""

from __future__ import annotations

from yasmin import State, Blackboard

from ..bb_utils import bb_get

DONE = "done"


class SendCommandState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[DONE])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        name = blackboard["cmd_name"]
        kwargs = bb_get(blackboard, "cmd_kwargs", {})

        ctx.send(name, **kwargs)
        return DONE
