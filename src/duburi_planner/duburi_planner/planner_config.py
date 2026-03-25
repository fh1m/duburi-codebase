"""
Planner configuration — typed accessors over ROS parameters.

All values come from the YAML config loaded via launch file.
This module provides a clean interface so states never touch
raw parameter dictionaries.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from rclpy.node import Node


@dataclass(frozen=True)
class TaskConfig:
    """Per-task tunables — one instance per RoboSub task."""
    target_class: str = "gate"
    search_yaw_step: float = 30.0
    search_speed: int = 40
    approach_speed: int = 50
    drive_through_time: float = 5.0
    alignment_timeout: float = 20.0
    search_timeout: float = 45.0


@dataclass(frozen=True)
class PlannerConfig:
    """Immutable snapshot of all planner parameters."""

    default_speed: int = 50
    dive_depth: float = 0.6
    surface_depth: float = 0.0

    arm_settle_time: float = 4.0
    dive_settle_time: float = 3.0
    feedback_timeout: float = 10.0
    alignment_timeout: float = 15.0
    search_timeout: float = 30.0
    drive_through_time: float = 4.0

    enable_viewer: bool = True
    viewer_name: str = "DUBURI_MISSION"

    tasks: dict[str, TaskConfig] = field(default_factory=dict)


def load_config(node: Node) -> PlannerConfig:
    """Declare parameters on *node* and return an immutable PlannerConfig."""

    def _p(name: str, default):
        return node.declare_parameter(name, default).value

    gate_cfg = TaskConfig(
        target_class=_p('gate.target_class', 'gate'),
        search_yaw_step=_p('gate.search_yaw_step', 30.0),
        search_speed=_p('gate.search_speed', 40),
        approach_speed=_p('gate.approach_speed', 50),
        drive_through_time=_p('gate.drive_through_time', 5.0),
        alignment_timeout=_p('gate.alignment_timeout', 20.0),
        search_timeout=_p('gate.search_timeout', 45.0),
    )

    slalom_cfg = TaskConfig(
        target_class=_p('slalom.target_class', 'pipe_red'),
        search_timeout=_p('slalom.search_timeout', 30.0),
    )

    bins_cfg = TaskConfig(
        target_class=_p('bins.target_class', 'bin'),
        search_timeout=_p('bins.search_timeout', 30.0),
    )

    return PlannerConfig(
        default_speed=_p('default_speed', 50),
        dive_depth=_p('dive_depth', 0.6),
        surface_depth=_p('surface_depth', 0.0),
        arm_settle_time=_p('arm_settle_time', 4.0),
        dive_settle_time=_p('dive_settle_time', 3.0),
        feedback_timeout=_p('feedback_timeout', 10.0),
        alignment_timeout=_p('alignment_timeout', 15.0),
        search_timeout=_p('search_timeout', 30.0),
        drive_through_time=_p('drive_through_time', 4.0),
        enable_viewer=_p('enable_viewer', True),
        viewer_name=_p('viewer_name', 'DUBURI_MISSION'),
        tasks={
            'gate': gate_cfg,
            'slalom': slalom_cfg,
            'bins': bins_cfg,
        },
    )
