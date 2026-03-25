"""
mission_node.py — Top-level ROS 2 node for the Duburi YASMIN planner.

Builds the full mission state machine:
    SUBMERGE → GATE_TASK → SURFACE → (outcome)

Each task is a hierarchical sub-SM (HFSM). Add more tasks by
inserting them between GATE_TASK and SURFACE.

The YASMIN web viewer is enabled by default so the FSM can be
monitored in real time at http://localhost:5000/ during pool tests.

Run:
    ros2 launch duburi_planner planner.launch.py
    # or standalone:
    ros2 run duburi_planner mission_node
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

import yasmin
from yasmin import StateMachine, Blackboard
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from .planner_config import load_config
from .planner_context import PlannerContext
from .states.submerge import SubmergeState, SUBMERGED, FAILED as SUB_FAILED
from .states.surface import SurfaceState, SURFACED
from .missions.gate import build_gate_task, GATE_PASSED, GATE_FAILED

MISSION_SUCCESS = "mission_success"
MISSION_ABORTED = "mission_aborted"


def build_mission(ctx: PlannerContext) -> StateMachine:
    """Construct the top-level mission state machine.

    Flow:
        SUBMERGE ──submerged──→ GATE_TASK ──gate_passed──→ SURFACE
            │                       │
            └─failed──→ ABORT   └─gate_failed──→ SURFACE_ABORT
    """
    sm = StateMachine(
        outcomes=[MISSION_SUCCESS, MISSION_ABORTED],
        handle_sigint=True,
    )

    sm.add_state(
        "SUBMERGE",
        SubmergeState(),
        transitions={
            SUBMERGED: "GATE_TASK",
            SUB_FAILED: MISSION_ABORTED,
        },
    )

    sm.add_state(
        "GATE_TASK",
        build_gate_task(),
        transitions={
            GATE_PASSED: "SURFACE",
            GATE_FAILED: "SURFACE_ABORT",
        },
    )

    sm.add_state(
        "SURFACE",
        SurfaceState(),
        transitions={SURFACED: MISSION_SUCCESS},
    )

    sm.add_state(
        "SURFACE_ABORT",
        SurfaceState(),
        transitions={SURFACED: MISSION_ABORTED},
    )

    return sm


def main() -> None:
    rclpy.init()
    set_ros_loggers()

    node = rclpy.create_node('mission_node')
    cfg = load_config(node)
    ctx = PlannerContext(node, cfg)

    yasmin.YASMIN_LOG_INFO("╔══════════════════════════════════════════╗")
    yasmin.YASMIN_LOG_INFO("║   DUBURI PLANNER — YASMIN Mission Node  ║")
    yasmin.YASMIN_LOG_INFO("╚══════════════════════════════════════════╝")

    sm = build_mission(ctx)

    if cfg.enable_viewer:
        YasminViewerPub(sm, cfg.viewer_name)
        yasmin.YASMIN_LOG_INFO(
            f"YASMIN Viewer active — open http://localhost:5000/ "
            f"and filter '{cfg.viewer_name}'")

    blackboard = Blackboard()
    blackboard["ctx"] = ctx

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        outcome = sm(blackboard)
        if outcome == MISSION_SUCCESS:
            yasmin.YASMIN_LOG_INFO("══ MISSION COMPLETE — SUCCESS ══")
        else:
            yasmin.YASMIN_LOG_WARN(f"══ MISSION ENDED — {outcome} ══")
    except KeyboardInterrupt:
        yasmin.YASMIN_LOG_WARN("Mission interrupted by operator")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(f"Mission exception: {e}")
    finally:
        try:
            ctx.send('stop')
            ctx.send('disarm')
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
