"""
Planner context — shared ROS resources for all YASMIN states.

Every state receives this via the Blackboard under the key "ctx".
Holds the ROS node, publishers, subscribers, and latest topic caches
so states stay thin and don't create their own subscriptions.

Design rationale:
  * Single ROS node for the entire planner (YASMIN executes states
    sequentially on a blocking thread; one node is sufficient).
  * Thread-safe caches for vehicle_state, alignment_status, detections,
    and driver_feedback since rclpy callbacks fire from the executor.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import yasmin

from duburi_interfaces.msg import (
    AlignmentStatus,
    DetectionArray,
    DriverCommand,
    DriverCommandFeedback,
    VehicleState,
)
from mavlink_driver.driver_client import make_command

from .planner_config import PlannerConfig

RELIABLE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


class PlannerContext:
    """Shared ROS bridge — injected into Blackboard for all states."""

    __slots__ = (
        'node', 'cfg', '_lock',
        '_cmd_pub', '_feedback_sub', '_state_sub',
        '_alignment_sub', '_detection_sub',
        '_vehicle_state', '_alignment_status',
        '_detections', '_last_feedback',
        '_feedback_event',
    )

    def __init__(self, node: Node, cfg: PlannerConfig) -> None:
        self.node = node
        self.cfg = cfg
        self._lock = threading.Lock()

        self._vehicle_state: VehicleState | None = None
        self._alignment_status: AlignmentStatus | None = None
        self._detections: DetectionArray | None = None
        self._last_feedback: DriverCommandFeedback | None = None
        self._feedback_event = threading.Event()

        self._cmd_pub = node.create_publisher(
            DriverCommand, '/driver/command', RELIABLE_QOS)

        self._feedback_sub = node.create_subscription(
            DriverCommandFeedback, '/driver/feedback',
            self._on_feedback, RELIABLE_QOS)

        self._state_sub = node.create_subscription(
            VehicleState, '/mavlink/vehicle_state',
            self._on_vehicle_state, SENSOR_QOS)

        self._alignment_sub = node.create_subscription(
            AlignmentStatus, '/vision/alignment_status',
            self._on_alignment, SENSOR_QOS)

        self._detection_sub = node.create_subscription(
            DetectionArray, '/vision/detections',
            self._on_detections, SENSOR_QOS)

    # ── Startup ─────────────────────────────────────────────────────

    def wait_for_ready(self, timeout: float = 10.0) -> bool:
        """Block until /driver/command has at least one subscriber.

        DDS discovery can take 1-3 s. If we publish before a subscriber
        matches, the message is silently dropped. This method ensures
        the inspector (or any other subscriber) is connected before the
        first command is sent.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._cmd_pub.get_subscription_count() > 0:
                yasmin.YASMIN_LOG_INFO(
                    "[planner] /driver/command subscriber connected")
                return True
            time.sleep(0.25)
        yasmin.YASMIN_LOG_WARN(
            "[planner] No subscriber on /driver/command after "
            f"{timeout}s — commands may be dropped")
        return False

    # ── Publishers ────────────────────────────────────────────────────

    def publish_command(self, cmd: DriverCommand) -> None:
        self._cmd_pub.publish(cmd)
        yasmin.YASMIN_LOG_INFO(
            f"[planner] CMD → {cmd.command}"
            f"{f' depth={cmd.depth}' if cmd.depth else ''}"
            f"{f' angle={cmd.angle}' if cmd.angle else ''}"
            f"{f' dur={cmd.duration}' if cmd.duration else ''}"
            f"{f' spd={cmd.speed}' if cmd.speed else ''}"
        )

    def send(self, command: str, **kwargs) -> None:
        """Shorthand: build a DriverCommand and publish it."""
        self.publish_command(make_command(command, **kwargs))

    # ── Subscription callbacks ────────────────────────────────────────

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        with self._lock:
            self._vehicle_state = msg

    def _on_alignment(self, msg: AlignmentStatus) -> None:
        with self._lock:
            self._alignment_status = msg

    def _on_detections(self, msg: DetectionArray) -> None:
        with self._lock:
            self._detections = msg

    def _on_feedback(self, msg: DriverCommandFeedback) -> None:
        with self._lock:
            self._last_feedback = msg
        self._feedback_event.set()

    # ── Thread-safe reads ─────────────────────────────────────────────

    @property
    def vehicle_state(self) -> VehicleState | None:
        with self._lock:
            return self._vehicle_state

    @property
    def alignment_status(self) -> AlignmentStatus | None:
        with self._lock:
            return self._alignment_status

    @property
    def detections(self) -> DetectionArray | None:
        with self._lock:
            return self._detections

    @property
    def last_feedback(self) -> DriverCommandFeedback | None:
        with self._lock:
            return self._last_feedback

    def clear_feedback(self) -> None:
        """Reset feedback event so wait_for_feedback blocks fresh."""
        self._feedback_event.clear()
        with self._lock:
            self._last_feedback = None

    def wait_for_feedback(self, timeout: float | None = None) -> DriverCommandFeedback | None:
        """Block until a DriverCommandFeedback arrives or timeout."""
        if timeout is None:
            timeout = self.cfg.feedback_timeout
        self._feedback_event.wait(timeout=timeout)
        return self.last_feedback

    def has_detection(self, class_name: str) -> bool:
        """Check if the latest detection frame contains *class_name*."""
        dets = self.detections
        if dets is None:
            return False
        return any(d.class_name == class_name for d in dets.detections)

    @property
    def heading(self) -> float:
        vs = self.vehicle_state
        return vs.yaw if vs else 0.0

    @property
    def depth(self) -> float:
        vs = self.vehicle_state
        return vs.depth if vs else 0.0

    @property
    def armed(self) -> bool:
        vs = self.vehicle_state
        return vs.armed if vs else False

    def log(self, msg: str) -> None:
        yasmin.YASMIN_LOG_INFO(f"[planner] {msg}")

    def warn(self, msg: str) -> None:
        yasmin.YASMIN_LOG_WARN(f"[planner] {msg}")

    def sleep(self, seconds: float) -> None:
        """Non-busy sleep that keeps the ROS executor alive."""
        time.sleep(seconds)
