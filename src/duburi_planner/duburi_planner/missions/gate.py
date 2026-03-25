"""
Gate mission — RoboSub Task 1: Begin Assessment.

Sub-state-machine:
    SEARCH_GATE  →  ALIGN_GATE  →  DRIVE_THROUGH  →  (done)

The gate is a PVC frame floating just below surface with RED/BLACK
panels. The AUV must pass through. Detection class is configurable
(default "gate") so the YOLO model class name can be changed without
touching the state machine.

Outcomes (of the sub-SM):
    "gate_passed"  — successfully navigated through the gate
    "gate_failed"  — search or alignment failed (abort this task)
"""

from __future__ import annotations

from yasmin import StateMachine, CbState, Blackboard

from ..states.search import SearchState, FOUND, TIMEOUT as SEARCH_TIMEOUT
from ..states.align import AlignState, ALIGNED, LOST, TIMEOUT as ALIGN_TIMEOUT
from ..states.drive import DriveState, DONE as DRIVE_DONE
from ..planner_config import TaskConfig

GATE_PASSED = "gate_passed"
GATE_FAILED = "gate_failed"


def _setup_gate_blackboard(blackboard: Blackboard) -> str:
    """Inject gate-specific parameters into the blackboard."""
    ctx = blackboard["ctx"]
    gate: TaskConfig = ctx.cfg.tasks['gate']

    blackboard["target_class"] = gate.target_class
    blackboard["search_yaw_step"] = gate.search_yaw_step
    blackboard["search_speed"] = gate.search_speed
    blackboard["search_timeout"] = gate.search_timeout
    blackboard["alignment_mode"] = "pid_align_forward"
    blackboard["alignment_timeout"] = gate.alignment_timeout
    blackboard["drive_command"] = "move_forward"
    blackboard["drive_duration"] = gate.drive_through_time
    blackboard["drive_speed"] = gate.approach_speed
    blackboard["drive_heading"] = None

    ctx.log("GATE — parameters loaded, starting search")
    return "configured"


def _record_heading(blackboard: Blackboard) -> str:
    """Snapshot heading after alignment so drive-through holds it."""
    ctx = blackboard["ctx"]
    heading = ctx.heading
    blackboard["drive_heading"] = heading
    ctx.log(f"GATE — locked heading {heading:.1f}° for drive-through")
    return "recorded"


def build_gate_task() -> StateMachine:
    """Construct the Gate task sub-state-machine."""
    sm = StateMachine(outcomes=[GATE_PASSED, GATE_FAILED])

    sm.add_state(
        "SETUP",
        CbState(["configured"], _setup_gate_blackboard),
        transitions={"configured": "SEARCH_GATE"},
    )

    sm.add_state(
        "SEARCH_GATE",
        SearchState(),
        transitions={
            FOUND: "ALIGN_GATE",
            SEARCH_TIMEOUT: GATE_FAILED,
        },
    )

    sm.add_state(
        "ALIGN_GATE",
        AlignState(),
        transitions={
            ALIGNED: "LOCK_HEADING",
            LOST: "SEARCH_GATE",
            ALIGN_TIMEOUT: GATE_FAILED,
        },
    )

    sm.add_state(
        "LOCK_HEADING",
        CbState(["recorded"], _record_heading),
        transitions={"recorded": "DRIVE_THROUGH"},
    )

    sm.add_state(
        "DRIVE_THROUGH",
        DriveState(),
        transitions={DRIVE_DONE: GATE_PASSED},
    )

    return sm
