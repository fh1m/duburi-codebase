"""
WaitFeedbackState — block until DriverCommandFeedback arrives.

Listens to /driver/feedback for acknowledgement of the last command.
Useful after commands that have a definite completion signal
(e.g. depth reached, heading reached).

Outcomes:
    "reached"    — feedback.status == "reached"
    "completed"  — feedback.status == "completed"
    "rejected"   — feedback.status == "rejected"
    "timeout"    — no feedback within deadline

Blackboard reads:
    ctx               PlannerContext
    feedback_timeout  float (optional)
    expected_command  str   (optional — if set, only match this command)
"""

from __future__ import annotations

from yasmin import State, Blackboard

from ..bb_utils import bb_get

REACHED = "reached"
COMPLETED = "completed"
REJECTED = "rejected"
TIMEOUT = "timeout"


class WaitFeedbackState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[REACHED, COMPLETED, REJECTED, TIMEOUT])

    def execute(self, blackboard: Blackboard) -> str:
        ctx = blackboard["ctx"]
        timeout = bb_get(blackboard, "feedback_timeout", ctx.cfg.feedback_timeout)
        expected = bb_get(blackboard, "expected_command", None)

        ctx.log(f"WAIT_FEEDBACK — waiting {timeout}s"
                f"{f' for {expected}' if expected else ''}")
        ctx.clear_feedback()

        fb = ctx.wait_for_feedback(timeout=timeout)
        if fb is None:
            ctx.warn("WAIT_FEEDBACK — timeout, no feedback received")
            return TIMEOUT

        if expected and fb.command != expected:
            ctx.warn(f"WAIT_FEEDBACK — got '{fb.command}' but expected '{expected}'")
            return TIMEOUT

        status = fb.status
        ctx.log(f"WAIT_FEEDBACK — {fb.command} → {status}"
                f"{f' ({fb.detail})' if fb.detail else ''}")

        if status == "reached":
            return REACHED
        elif status == "completed":
            return COMPLETED
        elif status == "rejected":
            return REJECTED
        else:
            return COMPLETED
