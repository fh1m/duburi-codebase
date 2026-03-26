#!/usr/bin/env python3
"""
MAVLink Inspector — Thin orchestrator for BRACU Duburi 4.2.

Owns the ROS 2 node and wires together the focused modules:
  - ConnectionManager  — serial link, heartbeat, reconnect
  - TelemetryParser    — MAVLink messages → vehicle state
  - RcController       — RC override with velocity ramp
  - PidController      — depth & yaw PID loops
  - CommandHandler     — DriverCommand dispatch

All external APIs (topics, parameters, messages) are unchanged.
"""

from __future__ import annotations

import math
import os
import threading
import time

os.environ['MAVLINK20'] = '1'

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

from duburi_interfaces.msg import (
    DriverCommand, DriverCommandFeedback, MavlinkEvent,
    TeleopCommand, VehicleDiagnostics, VehicleState,
)

from .connection_manager import ConnectionManager
from .telemetry_parser import TelemetryParser
from .rc_controller import (
    RcController, NEUTRAL_CHANNELS, NEUTRAL_PWM, PWM_RANGE,
    CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW,
)
from .command_handler import CommandHandler


class MavlinkInspectorNode(Node):
    """Main MAVLink inspector node — owns Pixhawk connection."""

    def __init__(self):
        super().__init__('mavlink_inspector')
        self._boot_time = time.time()

        # ── ROS parameters ───────────────────────────────────────────
        _conn_desc = ParameterDescriptor(
            description=(
                'Serial device (e.g. /dev/ttyACM0) or pymavlink URL. '
                'For ArduPilot SITL, match the transport of your link: '
                'if sim_vehicle/MAVProxy uses --out=udp:HOST:PORT, use '
                'udpin:HOST:PORT here. tcp:HOST:5760 is SITL’s TCP listener '
                'and often carries no stream for a second client once MAVProxy '
                'is connected.'
            ),
        )
        conn_port = self.declare_parameter(
            'connection_port', '/dev/ttyACM0', _conn_desc).value
        baud = self.declare_parameter('baud', 115200).value
        yaw_source = self.declare_parameter('yaw_source', 'attitude').value

        self._ramp_rate = self.declare_parameter('ramp_rate', 800).value

        # Yaw PID gains
        self._yaw_kp = self.declare_parameter('yaw_kp', 2.0).value
        self._yaw_ki = self.declare_parameter('yaw_ki', 0.05).value
        self._yaw_kd = self.declare_parameter('yaw_kd', 0.5).value
        self._yaw_max_integral = self.declare_parameter(
            'yaw_max_integral', 50.0).value

        # Depth PID gains (POOL-TUNED defaults)
        self._depth_kp = self.declare_parameter('depth_kp', 800.0).value
        self._depth_ki = self.declare_parameter('depth_ki', 50.0).value
        self._depth_kd = self.declare_parameter('depth_kd', 100.0).value
        self._depth_max_integral = self.declare_parameter(
            'depth_max_integral', 1.0).value
        self._depth_tolerance = self.declare_parameter(
            'depth_tolerance', 0.08).value

        self._pid_max_rate = self.declare_parameter(
            'pid_max_rate', 50).value
        self._nominal_voltage = self.declare_parameter(
            'nominal_voltage', 0.0).value
        self._surface_depth = self.declare_parameter(
            'surface_depth', 0.0).value
        self._ack_timeout = self.declare_parameter(
            'ack_timeout', 3.0).value
        self._surface_throttle_duration = self.declare_parameter(
            'surface_throttle_duration', 10.0).value

        # Connection health (Design Issue 3: parameterized timing)
        heartbeat_timeout = self.declare_parameter(
            'heartbeat_timeout', 3.0).value
        reconnect_backoff = self.declare_parameter(
            'reconnect_backoff', 2.0).value
        reconnect_max = self.declare_parameter(
            'reconnect_max', 15.0).value

        # ── Modules ──────────────────────────────────────────────────
        self._conn = ConnectionManager(
            port=conn_port,
            baud=baud,
            heartbeat_timeout=heartbeat_timeout,
            reconnect_backoff=reconnect_backoff,
            reconnect_max=reconnect_max,
            logger=self.get_logger(),
            on_event=self._publish_event,
        )
        self._telemetry = TelemetryParser(yaw_source=yaw_source)
        self._rc = RcController(ramp_rate=self._ramp_rate)
        self._cmd_handler = CommandHandler(self)

        # ── Movement state ───────────────────────────────────────────
        self._current_movement = None   # {channels, end_time, bypass_ramp, command}
        self._movement_lock = threading.Lock()

        # ── Depth PID state ──────────────────────────────────────────
        self._depth_pid = None          # PidController or None
        self._depth_pid_target = None   # target depth (m, negative)
        self._depth_pid_last_time = 0.0

        # ── Yaw heading state ────────────────────────────────────────
        self._yaw_pid = None            # PidController or None (PID mode)
        self._yaw_target = None         # target heading degrees or None
        self._yaw_tolerance = 3.0
        self._yaw_bang_offset = None    # PWM offset (bang-bang mode)
        self._yaw_command = ''          # command name for feedback
        self._yaw_pid_last_time = 0.0

        # ── ALT_HOLD depth target ────────────────────────────────────
        self._alt_hold_target = None

        # ── Command ACK tracking ─────────────────────────────────────
        self._pending_acks: dict[int, dict] = {}
        self._pending_acks_lock = threading.Lock()
        self._pending_mode_change = None

        # ── Publishers ───────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=10)
        reliable_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=1)

        self._event_pub = self.create_publisher(
            MavlinkEvent, '/mavlink/events', reliable_qos)
        self._state_pub = self.create_publisher(
            VehicleState, '/mavlink/vehicle_state', reliable_1)
        self._diag_pub = self.create_publisher(
            VehicleDiagnostics, '/mavlink/diagnostics', reliable_1)
        self._feedback_pub = self.create_publisher(
            DriverCommandFeedback, '/driver/feedback', reliable_qos)

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(
            DriverCommand, '/driver/command',
            self._cmd_handler.handle, reliable_qos)
        self.create_subscription(
            TeleopCommand, '/driver/teleop',
            self._cmd_handler.handle_teleop, reliable_qos)

        # ── Timers ───────────────────────────────────────────────────
        self.create_timer(0.02, self._read_mavlink)       # 50 Hz
        self.create_timer(0.1, self._publish_state)       # 10 Hz
        self.create_timer(1.0, self._conn.send_heartbeat) # 1 Hz
        self.create_timer(0.05, self._send_rc_override)   # 20 Hz
        self.create_timer(0.5, self._publish_diagnostics) # 2 Hz
        self.create_timer(0.5, self._resend_depth_target) # 2 Hz
        self.create_timer(0.5, self._check_ack_timeouts)  # 2 Hz

        # ── Start connection ─────────────────────────────────────────
        self._conn.start_background()

    # ── Event / feedback helpers ─────────────────────────────────────

    def _publish_event(self, event_type: str, description: str,
                       raw_data: str = ''):
        """Publish a MAVLink event.  Safe during shutdown."""
        try:
            if not rclpy.ok():
                return
            msg = MavlinkEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mavlink'
            msg.event_type = event_type
            msg.description = description
            msg.raw_data = raw_data
            self._event_pub.publish(msg)
            self.get_logger().info(f'[{event_type}] {description}')
        except Exception:
            pass

    def _publish_feedback(self, command: str, status: str,
                          error: float = 0.0, detail: str = ''):
        """Publish command feedback (DESIGN 6).  Safe during shutdown."""
        try:
            if not rclpy.ok():
                return
            msg = DriverCommandFeedback()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'inspector'
            msg.command = command
            msg.status = status
            msg.error = float(error)
            msg.detail = detail
            self._feedback_pub.publish(msg)
        except Exception:
            pass

    # ── State publishing ─────────────────────────────────────────────

    def _publish_state(self):
        try:
            if not rclpy.ok():
                return
            t = self._telemetry
            msg = VehicleState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.armed = t.armed
            msg.flight_mode = t.flight_mode
            msg.depth = float(t.depth)
            msg.yaw = float(t.yaw)
            msg.pitch = float(t.pitch)
            msg.roll = float(t.roll)
            msg.voltage = float(t.voltage)
            msg.current = float(t.current)
            self._state_pub.publish(msg)
        except Exception:
            pass

    def _publish_diagnostics(self):
        try:
            if not rclpy.ok():
                return
            t = self._telemetry
            msg = VehicleDiagnostics()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.heading_rate = float(t.heading_rate)
            msg.pressure = float(t.pressure)
            msg.temperature = float(t.temperature)
            msg.servo_output = [int(v) for v in t.servo_output]
            msg.rc_channels = [int(v) for v in t.rc_channels]
            msg.cpu_load = float(t.cpu_load)
            self._diag_pub.publish(msg)
        except Exception:
            pass

    # ── MAVLink reading ──────────────────────────────────────────────

    def _read_mavlink(self):
        for msg in self._conn.read_messages():
            # Link alive if any inbound telemetry arrives (not only HEARTBEAT).
            # SITL + TCP often streams ATTITUDE/AHRS2 while HEARTBEAT to a
            # second client can be sparse or contended with MAVProxy on 5760.
            self._conn.last_heartbeat = time.time()
            self._process_message(msg)

    def _process_message(self, msg):
        msg_type = msg.get_type()

        # ── Connection health: first good packet after a loss ──────────
        if self._conn.heartbeat_lost_notified:
            self._conn.heartbeat_lost_notified = False
            self._publish_event(
                'heartbeat_restored', 'Vehicle telemetry restored')
            self.get_logger().info('Vehicle telemetry restored.')

        # ── COMMAND_ACK — own ACK tracking ───────────────────────────
        if msg_type == 'COMMAND_ACK':
            self._handle_command_ack(msg)
            return

        # ── All other messages → telemetry parser ────────────────────
        events = self._telemetry.process(msg, self._conn.master)
        for ev_type, ev_desc, ev_raw in events:
            self._publish_event(ev_type, ev_desc, ev_raw)
            # POOL FIX 4: clear control state on disarm
            if ev_type == 'disarmed':
                self._clear_all_control()

    # ── Control state management ─────────────────────────────────────

    def _clear_all_control(self):
        """Clear all movement, PID, and heading control state."""
        with self._movement_lock:
            self._current_movement = None
            self._depth_pid = None
            self._depth_pid_target = None
            self._yaw_pid = None
            self._yaw_target = None
            self._yaw_bang_offset = None
        self._rc.clear_ramp()
        self._alt_hold_target = None

    def _stop_all(self):
        """Stop all thrusters — safety override (bypasses ramp)."""
        self._clear_all_control()
        self._rc.send_rc(NEUTRAL_CHANNELS, self._conn.master,
                         self.get_logger())
        self._publish_event('movement', 'Stop - all thrusters neutral')

    @staticmethod
    def _angle_error(current: float, target: float) -> float:
        """Shortest-path angle error in [−180, 180] degrees."""
        err = (target - current) % 360
        if err > 180:
            err -= 360
        return err

    # ── RC override (20 Hz) — 4-layer channel builder ────────────────

    def _send_rc_override(self):
        if not self._conn.connected or self._conn.master is None:
            return

        now = time.time()

        # Snapshot state under lock
        with self._movement_lock:
            mv = self._current_movement
            depth_pid = self._depth_pid
            depth_target = self._depth_pid_target
            yaw_pid = self._yaw_pid
            yaw_target = self._yaw_target
            yaw_tolerance = self._yaw_tolerance
            yaw_bang = self._yaw_bang_offset

        # ── Movement expiry ──────────────────────────────────────────
        if mv is not None and now >= mv['end_time']:
            expired_cmd = mv.get('command', 'unknown')
            with self._movement_lock:
                self._current_movement = None
            self._publish_event('movement', 'Movement duration expired')
            self._publish_feedback(expired_cmd, 'completed',
                                   detail='duration expired')
            mv = None

        # ── Layer 1+2: neutral + movement (with ramp) ────────────────
        bypass = mv.get('bypass_ramp', False) if mv else False
        channels = dict(NEUTRAL_CHANNELS)
        self._rc.apply_movement(channels, mv, bypass)

        # ── Layer 3: depth PID (overrides CH_THROTTLE) ───────────────
        if depth_pid is not None and depth_target is not None:
            dt = now - self._depth_pid_last_time if self._depth_pid_last_time > 0 else 0.05
            self._depth_pid_last_time = now
            if dt <= 0:
                dt = 0.05

            t = self._telemetry
            error = depth_target - t.depth
            raw_rate = (t.depth - t.prev_depth) / dt
            output = depth_pid.compute(error, dt,
                                       measurement_rate=raw_rate)

            if depth_pid.in_deadband:
                channels[CH_THROTTLE] = NEUTRAL_PWM
            else:
                channels[CH_THROTTLE] = NEUTRAL_PWM + output

        # ── Layer 4: yaw heading (overrides CH_YAW) ──────────────────
        if yaw_target is not None:
            err = self._angle_error(self._telemetry.yaw, yaw_target)

            if abs(err) <= yaw_tolerance:
                # Target reached
                with self._movement_lock:
                    self._yaw_pid = None
                    self._yaw_target = None
                    self._yaw_bang_offset = None
                self._publish_event(
                    'movement', f'Heading reached: {yaw_target}°')
                self._publish_feedback(
                    self._yaw_command or 'yaw_to_heading', 'reached',
                    error=err, detail=f'heading={yaw_target}°')

            elif yaw_pid is not None:
                # PID mode
                dt_y = now - self._yaw_pid_last_time if self._yaw_pid_last_time > 0 else 0.05
                self._yaw_pid_last_time = now
                if dt_y <= 0:
                    dt_y = 0.05
                output = yaw_pid.compute(
                    err, dt_y,
                    measurement_rate=self._telemetry.heading_rate)
                channels[CH_YAW] = NEUTRAL_PWM + output

            elif yaw_bang is not None:
                # Bang-bang mode
                channels[CH_YAW] = NEUTRAL_PWM + (
                    yaw_bang if err > 0 else -yaw_bang)

        # ── Send combined channels ───────────────────────────────────
        if not self._rc.send_rc(channels, self._conn.master,
                                self.get_logger()):
            self._conn.connected = False

    # ── MAVLink command infrastructure ───────────────────────────────

    @staticmethod
    def _mav_cmd_name(cmd_id: int) -> str:
        try:
            name = mavutil.mavlink.enums['MAV_CMD'][cmd_id].name
            return name[8:] if name.startswith('MAV_CMD_') else name
        except (KeyError, AttributeError):
            return f'CMD_{cmd_id}'

    @staticmethod
    def _mav_result_name(result: int) -> str:
        _MAP = {
            0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
            3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS',
            6: 'CANCELLED',
        }
        return _MAP.get(result, f'RESULT_{result}')

    def _send_command_long(self, mav_cmd: int,
                           p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0,
                           description: str = '',
                           confirmation: int = 0) -> threading.Event | None:
        """Send COMMAND_LONG and register for ACK tracking."""
        if not self._conn.connected or self._conn.master is None:
            return None
        desc = description or self._mav_cmd_name(mav_cmd)
        self.get_logger().info(
            f'TX COMMAND_LONG  {desc}  '
            f'cmd={mav_cmd} p1={p1} p2={p2} p3={p3} '
            f'p4={p4} p5={p5} p6={p6} p7={p7}')

        ack_event = threading.Event()
        with self._pending_acks_lock:
            self._pending_acks[mav_cmd] = {
                'sent_at': time.time(), 'desc': desc,
                'event': ack_event, 'result': None,
            }
        try:
            self._conn.master.mav.command_long_send(
                self._conn.master.target_system,
                self._conn.master.target_component,
                mav_cmd, confirmation,
                p1, p2, p3, p4, p5, p6, p7)
        except Exception as e:
            self.get_logger().error(f'command_long_send failed: {e}')
            with self._pending_acks_lock:
                self._pending_acks.pop(mav_cmd, None)
            return None
        return ack_event

    def _handle_command_ack(self, msg):
        cmd_id = msg.command
        result = msg.result
        cmd_name = self._mav_cmd_name(cmd_id)
        result_name = self._mav_result_name(result)

        with self._pending_acks_lock:
            pending = self._pending_acks.pop(cmd_id, None)

        desc = pending['desc'] if pending else cmd_name

        if result == 0:
            ack_msg, ack_type = f'{desc}: ACCEPTED', 'command_accepted'
        elif result == 1:
            ack_msg = f'{desc}: TEMPORARILY REJECTED (retry later)'
            ack_type = 'command_rejected'
        elif result == 2:
            ack_msg = f'{desc}: DENIED (bad params or state)'
            ack_type = 'command_denied'
        elif result == 3:
            ack_msg = f'{desc}: UNSUPPORTED by firmware'
            ack_type = 'command_denied'
        elif result == 4:
            ack_msg, ack_type = f'{desc}: FAILED', 'command_failed'
        elif result == 5:
            progress = getattr(msg, 'progress', 255)
            pct = f' ({progress}%)' if progress != 255 else ''
            ack_msg = f'{desc}: IN PROGRESS{pct}'
            ack_type = 'command_ack'
            if pending:
                with self._pending_acks_lock:
                    self._pending_acks[cmd_id] = pending
                pending = None  # don't resolve yet
        elif result == 6:
            ack_msg, ack_type = f'{desc}: CANCELLED', 'command_cancelled'
        else:
            ack_msg, ack_type = f'{desc}: {result_name}', 'command_ack'

        self._publish_event(ack_type, ack_msg,
                            raw_data=str(msg.to_dict()))

        if pending is not None:
            pending['result'] = result
            pending['event'].set()

    def _check_ack_timeouts(self):
        now = time.time()
        timed_out = []
        with self._pending_acks_lock:
            for cmd_id, info in list(self._pending_acks.items()):
                if now - info['sent_at'] > self._ack_timeout:
                    timed_out.append((cmd_id, info))
            for cmd_id, _ in timed_out:
                self._pending_acks.pop(cmd_id, None)

        for cmd_id, info in timed_out:
            self._publish_event(
                'command_timeout',
                f'{info["desc"]}: NO RESPONSE '
                f'(timeout {self._ack_timeout:.0f}s)')
            info['result'] = -1
            info['event'].set()

        # Mode change verification
        pmc = self._pending_mode_change
        if pmc is not None:
            if self._telemetry.flight_mode == pmc['target']:
                self._publish_event('mode_verified',
                    f'Mode change confirmed: {pmc["target"]}')
                self._pending_mode_change = None
            elif now - pmc['sent_at'] > self._ack_timeout:
                self._publish_event(
                    'mode_timeout',
                    f'Mode change to {pmc["target"]} NOT confirmed '
                    f'(current: {self._telemetry.flight_mode})')
                self._pending_mode_change = None

    # ── Vehicle control helpers ──────────────────────────────────────

    def _set_target_attitude(self, roll: float, pitch: float, yaw: float):
        if not self._conn.connected or self._conn.master is None:
            return
        self._conn.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self._boot_time)),
            self._conn.master.target_system,
            self._conn.master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            QuaternionBase([math.radians(a) for a in (roll, pitch, yaw)]),
            0, 0, 0, 0)

    def _set_target_depth(self, depth: float):
        if not self._conn.connected or self._conn.master is None:
            return
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._conn.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self._boot_time)),
            self._conn.master.target_system,
            self._conn.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            0, 0, depth, 0, 0, 0, 0, 0, 0, 0, 0)

    def _resend_depth_target(self):
        if self._alt_hold_target is not None and self._conn.connected:
            self._set_target_depth(self._alt_hold_target)

    def _arm_disarm(self, do_arm: bool):
        if not self._conn.connected or self._conn.master is None:
            return
        action = 'Arming' if do_arm else 'Disarming'
        v = 1 if do_arm else 0
        self.get_logger().info(
            f'TX COMMAND_LONG  COMPONENT_ARM_DISARM  arm={v}')
        self._publish_event('arm' if do_arm else 'disarm',
                            f'{action}...')
        try:
            self._conn.master.mav.command_long_send(
                self._conn.master.target_system,
                self._conn.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                v, 0, 0, 0, 0, 0, 0)
        except Exception as e:
            ev = 'arm_failed' if do_arm else 'disarm_failed'
            self._publish_event(ev, str(e))

    def _set_mode(self, mode: str):
        if not self._conn.connected or self._conn.master is None:
            return
        mode = (mode or 'MANUAL').upper()
        mapping = self._conn.master.mode_mapping()
        if mode not in mapping:
            self.get_logger().error(
                f'Unknown mode: {mode}. '
                f'Available: {list(mapping.keys())}')
            return
        mode_id = mapping[mode]
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=mode_id,
            description=f'SET_MODE {mode}')
        self._pending_mode_change = {
            'target': mode, 'sent_at': time.time()}
        self._publish_event('mode_change', f'Setting mode to {mode}')

    def _set_servo_pwm(self, servo_n: int, microseconds: int):
        if not self._conn.connected or self._conn.master is None:
            return
        self.get_logger().info(
            f'TX COMMAND_LONG  DO_SET_SERVO  '
            f'ch={servo_n + 8} pwm={microseconds}')
        self._conn.master.mav.command_long_send(
            self._conn.master.target_system,
            self._conn.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            servo_n + 8, microseconds, 0, 0, 0, 0, 0)


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkInspectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral RC directly (bypass ROS) for safety
        try:
            master = node._conn.master
            if master and node._conn.connected:
                rc = [NEUTRAL_PWM] * 8 + [65535] * 10
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component, *rc)
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
