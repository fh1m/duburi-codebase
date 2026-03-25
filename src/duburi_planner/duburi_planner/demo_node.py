"""
demo_node.py — Run the demo square mission to verify the planner stack.

Flow: ARM → SQUARE (4 legs + 4 turns) → DISARM

The arm step sends arm + set_mode MANUAL so the vehicle actually moves.
If no Pixhawk/telemetry is connected (desk testing), it warns but
continues — so you can still watch the FSM and command sequence.

The RC controller has a trapezoidal velocity ramp, so commands
transition smoothly without jerking thrusters to zero between moves.
Only the final turn sends "stop" to end the pattern.

Usage:
    ros2 run duburi_planner demo_node

    # In a separate terminal, watch the commands:
    ros2 topic echo /driver/command

    # Optionally start the YASMIN viewer:
    ros2 run yasmin_viewer yasmin_viewer_node
    # Then open http://localhost:5000/ and filter 'DUBURI_DEMO_SQUARE'

Expected output sequence on /driver/command:
    arm → set_mode MANUAL →
    move_forward → pid_yaw_to_heading → move_forward → pid_yaw_to_heading →
    move_forward → pid_yaw_to_heading → move_forward → pid_yaw_to_heading → stop →
    surface → disarm
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
from .states.arm import ArmState, ARMED, FAILED as ARM_FAILED
from .states.surface import SurfaceState, SURFACED
from .missions.demo_square import build_demo_square, SQUARE_DONE

DEMO_SUCCESS = "demo_success"
DEMO_FAILED = "demo_failed"


def build_demo_mission(ctx: PlannerContext) -> StateMachine:
    """Top-level SM: ARM → SQUARE → DISARM."""
    sm = StateMachine(
        outcomes=[DEMO_SUCCESS, DEMO_FAILED],
        handle_sigint=True,
    )

    sm.add_state(
        "ARM",
        ArmState(),
        transitions={
            ARMED: "SQUARE",
            ARM_FAILED: DEMO_FAILED,
        },
    )

    sm.add_state(
        "SQUARE",
        build_demo_square(),
        transitions={SQUARE_DONE: "DISARM"},
    )

    sm.add_state(
        "DISARM",
        SurfaceState(),
        transitions={SURFACED: DEMO_SUCCESS},
    )

    return sm


def main() -> None:
    rclpy.init()
    set_ros_loggers()

    node = rclpy.create_node('demo_node')
    cfg = load_config(node)
    ctx = PlannerContext(node, cfg)

    yasmin.YASMIN_LOG_INFO("╔══════════════════════════════════════════╗")
    yasmin.YASMIN_LOG_INFO("║  DUBURI PLANNER — Demo Square Mission   ║")
    yasmin.YASMIN_LOG_INFO("╠══════════════════════════════════════════╣")
    yasmin.YASMIN_LOG_INFO("║  Traces a square: fwd + 90° turn × 4    ║")
    yasmin.YASMIN_LOG_INFO("║  Safe to run on desk (no arm required)   ║")
    yasmin.YASMIN_LOG_INFO("╚══════════════════════════════════════════╝")

    sm = build_demo_mission(ctx)

    viewer_name = "DUBURI_DEMO_SQUARE"
    YasminViewerPub(sm, viewer_name)
    yasmin.YASMIN_LOG_INFO(
        f"YASMIN Viewer → http://localhost:5000/ filter '{viewer_name}'")

    blackboard = Blackboard()
    blackboard["ctx"] = ctx

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        outcome = sm(blackboard)
        if outcome == DEMO_SUCCESS:
            yasmin.YASMIN_LOG_INFO("══ DEMO COMPLETE — square traced! ══")
        else:
            yasmin.YASMIN_LOG_WARN(f"══ DEMO ENDED — {outcome} ══")
    except KeyboardInterrupt:
        yasmin.YASMIN_LOG_WARN("Demo interrupted by operator")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(f"Demo exception: {e}")
    finally:
        try:
            ctx.send('stop')
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
