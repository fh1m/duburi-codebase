#!/usr/bin/env python3
"""
Mission executor - runs predefined or loaded missions by publishing DriverCommand sequence.

NOTE: This is a simple sequential executor for pool testing and basic missions.
A proper state-machine-based planning package is planned for autonomous missions.
This module will be superseded by that planner; until then, it serves as a
lightweight way to script and replay command sequences.

Features:
  - Ctrl+C gracefully aborts the mission (sends stop, does NOT kill the node)
  - ``pause`` / ``resume`` commands in mission files
  - Interruptible sleeps — abort takes effect within 0.1 s

Usage:
  ros2 run mavlink_driver mission_executor --ros-args -p mission:=pool_test
  ros2 run mavlink_driver mission_executor --ros-args -p mission_file:=gate.txt
"""

import signal
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from duburi_interfaces.msg import DriverCommand, MavlinkEvent, VehicleState

from mavlink_driver.driver_client import (
    arm,
    disarm,
    move_forward,
    move_left,
    move_right,
    pid_depth,
    pid_depth_off,
    set_mode,
    stop,
    surface,
)
from mavlink_driver.mission_parser import parse_file_command


from duburi_common.constants import MISSION_PATHS


class MissionExecutorNode(Node):
    """Executes missions by publishing DriverCommand with delays.

    Supports:
      - Built-in named missions (param: mission)
      - Mission files with one command per line (param: mission_file)
    """

    def __init__(self):
        super().__init__('mission_executor')
        self._cmd_pub = self.create_publisher(
            DriverCommand, '/driver/command',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        # Subscribe to events for feedback
        self._event_sub = self.create_subscription(
            MavlinkEvent, '/mavlink/events', self._on_event,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        # Subscribe to vehicle state for current heading (used by relative turn)
        self._state_sub = self.create_subscription(
            VehicleState, '/mavlink/vehicle_state', self._on_state,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        )
        self._current_heading = 0.0  # updated from telemetry
        self._mission_name = self.declare_parameter('mission', 'pool_test').value
        self._mission_file = self.declare_parameter('mission_file', '').value

        # ── Abort / pause state ──────────────────────────────────────────
        self._abort = False
        self._paused = threading.Event()
        self._paused.set()  # starts UN-paused (set = not waiting)

        # Install SIGINT handler so Ctrl+C aborts mission gracefully
        signal.signal(signal.SIGINT, self._sigint_handler)

        self.get_logger().info(
            f'Mission executor ready. '
            f'Mission: {self._mission_name}, File: {self._mission_file or "(none)"}'
        )
        # Run mission once after brief delay to allow inspector to be ready
        self._run_timer = self.create_timer(3.0, self._run_once)

    # ── Signal handling ──────────────────────────────────────────────────

    def _sigint_handler(self, signum, frame):
        """Ctrl+C: abort mission gracefully — send stop, set flag."""
        if self._abort:
            # Second Ctrl+C — force exit
            self.get_logger().warn('Force exit (second Ctrl+C)')
            raise SystemExit(1)
        self._abort = True
        self._paused.set()  # unblock any pause-wait so abort can proceed
        self.get_logger().warn('Ctrl+C received — aborting mission...')
        try:
            self._cmd_pub.publish(stop())
        except Exception:
            pass

    def _on_event(self, msg: MavlinkEvent):
        """Log relevant autopilot events during mission."""
        if msg.event_type in ('command_ack', 'command_rejected', 'arm_failed'):
            self.get_logger().info(f'  [{msg.event_type}] {msg.description}')

    def _on_state(self, msg: VehicleState):
        """Track current heading for relative turn commands."""
        self._current_heading = msg.yaw

    def _publish(self, cmd: DriverCommand, delay: float = 0.5):
        """Publish command and wait. Returns False if mission aborted."""
        if self._abort:
            return False
        # Honour pause — block until resumed (or aborted)
        self._paused.wait()
        if self._abort:
            return False
        self._cmd_pub.publish(cmd)
        self.get_logger().info(f'  >> {cmd.command}'
                               f'{f" depth={cmd.depth}" if cmd.depth != 0.0 else ""}'
                               f'{f" angle={cmd.angle}" if cmd.angle != 0.0 else ""}'
                               f'{f" speed={cmd.speed}" if cmd.speed != 0 else ""}'
                               f'{f" duration={cmd.duration}s" if cmd.duration != 0.0 else ""}')
        if delay > 0:
            if not self._interruptible_sleep(delay):
                return False
        return not self._abort

    def _interruptible_sleep(self, seconds: float) -> bool:
        """Sleep in 0.1 s steps, checking _abort each step.

        Returns True if completed without abort, False if aborted.
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._abort:
                return False
            # Also honour pause during sleep
            self._paused.wait()
            if self._abort:
                return False
            time.sleep(min(0.1, end - time.monotonic()))
        return True

    def _run_once(self):
        """Run mission once, then cancel timer."""
        self._run_timer.cancel()
        if self._mission_file:
            self._run_file_mission(self._mission_file)
        else:
            missions = {
                'pool_test': self._mission_pool_test,
            }
            fn = missions.get(self._mission_name)
            if fn:
                fn()
                if self._abort:
                    self.get_logger().warn('=== Mission ABORTED ===')
            else:
                avail = ', '.join(missions.keys())
                self.get_logger().error(
                    f'Unknown mission: {self._mission_name}. Available: {avail}'
                )

    def resume(self):
        """Resume a paused mission (can be called from an external service/topic)."""
        if not self._paused.is_set():
            self._paused.set()

    # ── Built-in missions ────────────────────────────────────────────────

    def _mission_pool_test(self):
        """Basic pool test: arm, dive, move, stop."""
        self.get_logger().info('=== Pool Test Mission ===')
        if not self._publish(set_mode('MANUAL')):
            return
        if not self._publish(arm(), delay=4.0):
            return
        # Software depth PID first (safer, works in MANUAL)
        if not self._publish(pid_depth(0.5), delay=5.0):
            return
        if not self._publish(move_forward(duration=3, speed=60)):
            return
        if not self._interruptible_sleep(4):
            return
        if not self._publish(move_left(duration=2, speed=50)):
            return
        if not self._interruptible_sleep(3):
            return
        if not self._publish(move_right(duration=2, speed=50)):
            return
        if not self._interruptible_sleep(3):
            return
        self._publish(pid_depth_off())
        if not self._publish(surface(), delay=5.0):
            return
        self._publish(stop())
        self._publish(disarm(), delay=2.0)
        self.get_logger().info('=== Pool Test Complete ===')

    # ── File-based missions ──────────────────────────────────────────────

    def _find_file(self, name: str) -> Path | None:
        """Find mission file by name."""
        # Try as absolute/relative path first
        p = Path(name)
        if p.is_file():
            return p
        # Search mission directories
        for base in MISSION_PATHS:
            for candidate in [base / name, base / f'{name}.txt']:
                if candidate.is_file():
                    return candidate
        return None

    def _run_file_mission(self, name: str):
        """Run a mission from a text file (same format as runner missions)."""
        path = self._find_file(name)
        if not path:
            self.get_logger().error(f'Mission file not found: {name}')
            return
        self.get_logger().info(f'=== Running mission file: {path.name} ===')
        with open(path) as f:
            for num, line in enumerate(f, 1):
                if self._abort:
                    break
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                self.get_logger().info(f'  [{num}] {line}')
                parts = line.split()
                cmd = parts[0].lower()
                args = parts[1:]

                if cmd in ('sleep', 'wait'):
                    secs = float(args[0]) if args else 1.0
                    if not self._interruptible_sleep(secs):
                        break
                    continue

                if cmd == 'pause':
                    self.get_logger().info('  ⏸  Mission paused. Send resume to continue.')
                    self._paused.clear()  # block until set()
                    self._paused.wait()
                    if self._abort:
                        break
                    self.get_logger().info('  ▶  Resumed.')
                    continue

                if cmd == 'resume':
                    # resume in a file is a no-op (resumes are triggered externally)
                    continue

                # Map text commands to DriverCommand
                driver_cmd = self._parse_file_command(cmd, args)
                if driver_cmd:
                    if not self._publish(driver_cmd, delay=0.3):
                        self.get_logger().warn('Mission aborted.')
                        return
                    # If command has duration, wait for it
                    if driver_cmd.duration > 0:
                        if not self._interruptible_sleep(driver_cmd.duration + 0.5):
                            break
                else:
                    self.get_logger().warn(f'  Skipping unknown command: {line}')

        if self._abort:
            self.get_logger().warn('=== Mission ABORTED ===')
        else:
            self.get_logger().info('=== Mission file complete ===')

    def _parse_file_command(self, cmd: str, args: list[str]) -> DriverCommand | None:
        """Parse a single command line into a DriverCommand."""
        return parse_file_command(
            self._current_heading, cmd, args, logger=self.get_logger()
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        # SIGINT is handled by _sigint_handler; this catches a second Ctrl+C
        # or the SystemExit raised by the handler.
        pass
    finally:
        # Ensure vehicle is stopped on exit
        try:
            if rclpy.ok():
                node._cmd_pub.publish(stop())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
