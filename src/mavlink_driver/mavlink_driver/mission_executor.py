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
    go_combo,
    move_at,
    move_combo,
    move_forward,
    move_back,
    move_left,
    move_right,
    move_up,
    move_down,
    set_depth,
    set_mode,
    stop,
    yaw_to_heading,
    yaw_left,
    yaw_right,
    pid_yaw,
    pid_depth,
    pid_depth_off,
    surface,
    turn_left,
    turn_right,
    pid_turn_left,
    pid_turn_right,
    # Instant (no-ramp) fallbacks
    just_move_forward,
    just_move_back,
    just_move_left,
    just_move_right,
    just_move_up,
    just_move_down,
    just_yaw_left,
    just_yaw_right,
    just_move_combo,
    just_move_at,
    just_go_combo,
    just_surface,
)


# Mission file search paths
MISSION_PATHS = [
    Path.cwd() / 'missions',
    Path(__file__).resolve().parent.parent / 'missions',
    Path.home() / '.duburi' / 'missions',
]


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
        """Parse a single command line into a DriverCommand.

        Supports 'just' prefix for instant (no-ramp) fallbacks:
          just forward 3 50  → just_move_forward(duration=3, speed=50)
          just go forward 90 5 60  → just_go_combo(...)
        """
        try:
            # ── 'just' prefix: instant (no-ramp) fallback ───────────────
            is_just = False
            if cmd == 'just':
                is_just = True
                if not args:
                    return None
                cmd = args[0]
                args = args[1:]

            # ── Resolve ~ prefix (PID) and backward-compatible aliases ──
            is_pid = False
            if cmd.startswith('~'):
                is_pid = True
                cmd = cmd[1:]
            if cmd == 'dive':
                cmd = 'depth'
            elif cmd == 'p_dive':
                cmd = 'depth'
                is_pid = True
            elif cmd == 'yaw':
                cmd = 'heading'
            elif cmd == 'p_yaw':
                cmd = 'heading'
                is_pid = True
            elif cmd == 'p_turn':
                cmd = 'turn'
                is_pid = True

            if cmd == 'arm':
                return arm()
            elif cmd == 'disarm':
                return disarm()
            elif cmd == 'stop':
                return stop()
            elif cmd == 'surface':
                return just_surface() if is_just else surface()
            elif cmd in ('mode', 'set_mode'):
                return set_mode(args[0].upper() if args else 'MANUAL')
            elif cmd == 'forward':
                dur = float(args[0]) if args else 3.0
                spd = int(args[1]) if len(args) > 1 else 50
                return just_move_forward(duration=dur, speed=spd) if is_just else move_forward(duration=dur, speed=spd)
            elif cmd == 'back':
                dur = float(args[0]) if args else 3.0
                spd = int(args[1]) if len(args) > 1 else 50
                return just_move_back(duration=dur, speed=spd) if is_just else move_back(duration=dur, speed=spd)
            elif cmd == 'left':
                dur = float(args[0]) if args else 3.0
                spd = int(args[1]) if len(args) > 1 else 50
                return just_move_left(duration=dur, speed=spd) if is_just else move_left(duration=dur, speed=spd)
            elif cmd == 'right':
                dur = float(args[0]) if args else 3.0
                spd = int(args[1]) if len(args) > 1 else 50
                return just_move_right(duration=dur, speed=spd) if is_just else move_right(duration=dur, speed=spd)
            elif cmd == 'up':
                dur = float(args[0]) if args else 3.0
                spd = int(args[1]) if len(args) > 1 else 50
                return just_move_up(duration=dur, speed=spd) if is_just else move_up(duration=dur, speed=spd)
            elif cmd == 'down':
                dur = float(args[0]) if args else 3.0
                spd = int(args[1]) if len(args) > 1 else 50
                return just_move_down(duration=dur, speed=spd) if is_just else move_down(duration=dur, speed=spd)
            elif cmd == 'depth':
                if is_pid:
                    if args and args[0] == 'off':
                        return pid_depth_off()
                    return pid_depth(float(args[0]) if args else 0.0)
                else:
                    return set_depth(float(args[0])) if args else None
            elif cmd == 'heading':
                if not args:
                    return None
                if args[0] in ('left', 'right'):
                    dur = float(args[1]) if len(args) > 1 else 3.0
                    spd = int(args[2]) if len(args) > 2 else 50
                    if args[0] == 'left':
                        return just_yaw_left(duration=dur, speed=spd) if is_just else yaw_left(duration=dur, speed=spd)
                    else:
                        return just_yaw_right(duration=dur, speed=spd) if is_just else yaw_right(duration=dur, speed=spd)
                if is_pid:
                    return pid_yaw(float(args[0])) if args else None
                else:
                    return yaw_to_heading(float(args[0])) if args else None
            elif cmd == 'turn':
                # turn/~turn left/right <degrees> [speed]
                if not args or len(args) < 2:
                    return None
                direction = args[0]
                angle = float(args[1])
                spd = int(args[2]) if len(args) > 2 else 50
                if is_pid:
                    if direction == 'left':
                        return pid_turn_left(self._current_heading, angle, speed=spd)
                    elif direction == 'right':
                        return pid_turn_right(self._current_heading, angle, speed=spd)
                else:
                    if direction == 'left':
                        return turn_left(self._current_heading, angle, speed=spd)
                    elif direction == 'right':
                        return turn_right(self._current_heading, angle, speed=spd)
                return None
            elif cmd == 'move':
                # Support 'move forward 3 50' syntax (matches runner format)
                # Preserve 'just' prefix: 'just move forward' → dispatch 'just forward'
                if not args:
                    return None
                if args[0] == 'at':
                    # move at <angle> [duration] [speed]
                    if len(args) < 2:
                        return None
                    bearing = float(args[1])
                    dur = float(args[2]) if len(args) > 2 else 0.0
                    spd = int(args[3]) if len(args) > 3 else 50
                    return just_move_at(bearing, duration=dur, speed=spd) if is_just else move_at(bearing, duration=dur, speed=spd)
                if is_just:
                    return self._parse_file_command('just', [args[0]] + args[1:])
                return self._parse_file_command(args[0], args[1:])
            elif cmd == 'at':
                # at <angle> [duration] [speed]
                if not args:
                    return None
                bearing = float(args[0])
                dur = float(args[1]) if len(args) > 1 else 0.0
                spd = int(args[2]) if len(args) > 2 else 50
                return just_move_at(bearing, duration=dur, speed=spd) if is_just else move_at(bearing, duration=dur, speed=spd)
            elif cmd == 'go':
                # go <direction> <heading> [duration] [speed]
                # Supports: go forward 90 5 60, go forward-right 45 5 60
                if not args or len(args) < 2:
                    return None
                direction = args[0]
                heading = float(args[1])
                dur = float(args[2]) if len(args) > 2 else 0.0
                spd = int(args[3]) if len(args) > 3 else 50
                if is_just:
                    return just_go_combo(direction, angle=heading, duration=dur, speed=spd)
                return go_combo(direction, angle=heading, duration=dur, speed=spd)
            elif '-' in cmd:
                # Compound diagonal: forward-right 5 50 (horizontal only)
                VALID = {'forward', 'back', 'backward', 'left', 'right'}
                parts = cmd.split('-')
                if len(parts) == 2 and all(p in VALID for p in parts):
                    dur = float(args[0]) if args else 3.0
                    spd = int(args[1]) if len(args) > 1 else 50
                    if is_just:
                        return just_move_combo(cmd, duration=dur, speed=spd)
                    return move_combo(cmd, duration=dur, speed=spd)
                return None
            elif cmd == 'grabber':
                if args and args[0] == 'open':
                    from mavlink_driver.driver_client import open_grabber
                    return open_grabber()
                elif args and args[0] == 'close':
                    from mavlink_driver.driver_client import close_grabber
                    return close_grabber()
        except (IndexError, ValueError) as e:
            self.get_logger().warn(f'Bad arguments for {cmd}: {e}')
        return None


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
