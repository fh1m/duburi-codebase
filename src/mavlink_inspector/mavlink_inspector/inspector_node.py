#!/usr/bin/env python3
"""
MAVLink Inspector - Main Pixhawk connection hub for BRACU Duburi 4.2.

Owns the MAVLink connection to Pixhawk via /dev/ttyACM0.
Publishes vehicle state and events. Subscribes to driver commands and executes them.
"""

from __future__ import annotations

import math
import os
import threading
import time
from pathlib import Path

# Use MAVLink 2
os.environ['MAVLINK20'] = '1'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

from duburi_interfaces.msg import DriverCommand, MavlinkEvent, VehicleDiagnostics, VehicleState


# Duburi 4.2 channel mapping (ArduSub)
# Ch 1: Pitch, Ch 2: Roll, Ch 3: Throttle (depth), Ch 4: Yaw
# Ch 5: Forward, Ch 6: Lateral (left/right)
# Ch 7-8: Reserved (camera, grabber, etc.)
CH_FORWARD = 5
CH_LATERAL = 6
CH_THROTTLE = 3
CH_YAW = 4
CH_PITCH = 1
CH_ROLL = 2

NEUTRAL_PWM = 1500
PWM_RANGE = 400  # 1100-1900, so ±400 from 1500


# Direction component → (channel, sign) for compound diagonal movement.
# Only horizontal axes are supported for diagonals — vertical movement
# (up/down) should always use depth PID (p_dive) or ALT_HOLD (dive)
# for reliable, smooth control.
DIRECTION_COMPONENTS = {
    'forward':  (CH_FORWARD,  +1),
    'back':     (CH_FORWARD,  -1),
    'backward': (CH_FORWARD,  -1),
    'left':     (CH_LATERAL,  -1),
    'right':    (CH_LATERAL,  +1),
}


def percent_to_pwm(percent: float) -> int:
    """Convert -100..100 percent to PWM offset from 1500."""
    percent = max(-100, min(100, percent))
    return int(1500 + (percent / 100) * PWM_RANGE)


def _build_diagonal_channels(parts: list[str], speed_pwm: int) -> tuple[dict, str] | None:
    """Build channel dict for horizontal diagonal movement with √2 scaling.

    Only accepts horizontal directions (forward/back/left/right).
    For 2-axis diagonals, each axis gets speed/√2 ≈ 71% per channel
    so the resultant vector magnitude equals the requested speed.

    Returns (channels_dict, label) or None if invalid.
    """
    channels_map: dict[int, int] = {}  # ch → sign
    for part in parts:
        if part not in DIRECTION_COMPONENTS:
            return None
        ch, sign = DIRECTION_COMPONENTS[part]
        if ch in channels_map:  # conflicting axes (e.g. forward+back)
            return None
        channels_map[ch] = sign
    if not channels_map or len(channels_map) > 2:
        return None  # Max 2 horizontal axes
    n = len(channels_map)
    offset = speed_pwm - NEUTRAL_PWM  # e.g. 1700−1500 = 200
    scaled = int(offset / math.sqrt(n)) if n > 1 else offset
    channels = {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM,
                CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM}
    for ch, sign in channels_map.items():
        channels[ch] = NEUTRAL_PWM + sign * scaled
    label = '-'.join(parts)
    return channels, label


class MavlinkInspectorNode(Node):
    """Main MAVLink inspector node - owns Pixhawk connection."""

    def __init__(self):
        super().__init__('mavlink_inspector')
        self._connection_port = self.declare_parameter('connection_port', '/dev/ttyACM0').value
        self._baud = self.declare_parameter('baud', 115200).value
        self._boot_time = time.time()

        # Connection state
        self._master = None
        self._connected = False
        self._last_heartbeat = 0
        self._heartbeat_timeout = 3.0  # seconds without heartbeat before declaring lost
        self._heartbeat_lost_notified = False

        # Reconnection state
        self._reconnecting = False
        self._reconnect_backoff = 2.0  # initial retry delay (seconds)
        self._reconnect_backoff_max = 15.0
        self._reconnect_backoff_current = 2.0
        # Ports to scan when configured port isn't found
        self._fallback_ports = [
            '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3',
            '/dev/ttyUSB0', '/dev/ttyUSB1',
        ]

        # Cached state
        self._armed = False
        self._flight_mode = 'UNKNOWN'
        self._depth = 0.0
        self._yaw = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._voltage = 0.0
        self._current = 0.0
        self._prev_armed = None
        self._prev_mode = None

        # Diagnostics state (lower priority, published at 2 Hz)
        self._heading_rate = 0.0      # deg/s
        self._pressure = 0.0          # hPa
        self._temperature = 0.0       # °C
        self._servo_output = [0] * 8  # PWM per motor channel
        self._rc_channels = [0] * 8   # RC readback
        self._cpu_load = 0.0          # %

        # Movement state: RC must be sent continuously (ArduSub timeout ~3s)
        self._current_movement = None  # {ch_*: pwm, end_time}
        self._movement_lock = threading.Lock()
        # Yaw-to-heading: use thrusters to reach target (not set_attitude_target)
        self._yaw_to_heading = None  # {target_deg, gain_offset, tolerance_deg, ...}

        # PID yaw gains (tunable via ROS params)
        self._yaw_kp = self.declare_parameter('yaw_kp', 2.0).value
        self._yaw_ki = self.declare_parameter('yaw_ki', 0.05).value
        self._yaw_kd = self.declare_parameter('yaw_kd', 0.5).value
        self._yaw_max_integral = self.declare_parameter('yaw_max_integral', 50.0).value

        # ── Software depth PID (via RC throttle channel, works in any mode) ──
        self._depth_pid = None  # {target_m, kp, ki, kd, integral, last_error, last_time, max_integral}
        # Depth PID gains (tunable via ROS params)
        # POOL TODO: Tune these gains underwater. If AUV oscillates around target,
        #   reduce Kp first (try 300→200). If it settles slowly, increase Kp.
        #   Ki handles steady-state drift — reduce if integral windup is visible.
        #   Kd damps overshoot — increase if AUV yo-yos past target.
        self._depth_kp = self.declare_parameter('depth_kp', 500.0).value
        self._depth_ki = self.declare_parameter('depth_ki', 100.0).value
        self._depth_kd = self.declare_parameter('depth_kd', 200.0).value
        # POOL TODO: If integral grows too fast in pool, lower this cap or reduce Ki.
        self._depth_max_integral = self.declare_parameter('depth_max_integral', 2.0).value
        # POOL TODO: Increase tolerance if depth sensor noise causes throttle jitter
        #   at rest (try 0.10 or 0.15 if 0.05 is too tight).
        self._depth_tolerance = self.declare_parameter('depth_tolerance', 0.05).value  # metres

        # ── ALT_HOLD depth target (for periodic re-send) ──
        # POOL TODO: Verify ALT_HOLD mode actually engages with our Pixhawk 2.4.8
        #   firmware. If `dive` command has no effect, firmware may not support
        #   SET_POSITION_TARGET_GLOBAL_INT — fall back to `p_dive` (software PID).
        self._alt_hold_target = None  # Target depth (negative m) to re-send in ALT_HOLD

        # ── Command ACK tracking ──
        # Tracks COMMAND_LONG messages awaiting COMMAND_ACK from Pixhawk.
        # Each entry: {mav_cmd_id: {'sent_at': float, 'desc': str, 'event': Event, 'result': int|None}}
        self._pending_acks = {}  # type: dict[int, dict]
        self._pending_acks_lock = threading.Lock()
        self._ack_timeout = self.declare_parameter('ack_timeout', 3.0).value  # seconds
        # Mode change verification
        self._pending_mode_change = None  # {'target': str, 'sent_at': float}

        # Publishers
        self._event_pub = self.create_publisher(
            MavlinkEvent, '/mavlink/events', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, depth=10
            )
        )
        self._state_pub = self.create_publisher(
            VehicleState, '/mavlink/vehicle_state', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, depth=1
            )
        )
        self._diag_pub = self.create_publisher(
            VehicleDiagnostics, '/mavlink/diagnostics', QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, depth=1
            )
        )

        # Subscriber for driver commands
        self._cmd_sub = self.create_subscription(
            DriverCommand, '/driver/command', self._on_driver_command,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )

        # Timers
        self._read_timer = self.create_timer(0.02, self._read_mavlink)  # 50 Hz
        self._state_timer = self.create_timer(0.1, self._publish_state)  # 10 Hz
        self._heartbeat_timer = self.create_timer(1.0, self._send_heartbeat)
        # RC override at 20 Hz - ArduSub requires constant rate or failsafe (~3s timeout)
        self._rc_override_timer = self.create_timer(0.05, self._send_rc_override)
        # Diagnostics at 2 Hz
        self._diag_timer = self.create_timer(0.5, self._publish_diagnostics)
        # Re-send ALT_HOLD depth target at 2 Hz (ArduSub may forget if not refreshed)
        self._depth_target_timer = self.create_timer(0.5, self._resend_depth_target)
        # Check for timed-out command ACKs at 2 Hz
        self._ack_timeout_timer = self.create_timer(0.5, self._check_ack_timeouts)

        # Start connection in background
        self._connect_thread = threading.Thread(target=self._connect, daemon=True)
        self._connect_thread.start()

    def _publish_event(self, event_type: str, description: str, raw_data: str = ''):
        """Publish a MAVLink event. Safe to call during shutdown."""
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
            self.get_logger().info(f"[{event_type}] {description}")
        except Exception:
            pass

    # ── MAVLink command name/result helpers ────────────────────────────

    @staticmethod
    def _mav_cmd_name(cmd_id: int) -> str:
        """Human-readable name for a MAV_CMD id."""
        try:
            entry = mavutil.mavlink.enums['MAV_CMD'][cmd_id]
            # Strip the MAV_CMD_ prefix for brevity
            name = entry.name
            if name.startswith('MAV_CMD_'):
                name = name[8:]
            return name
        except (KeyError, AttributeError):
            return f'CMD_{cmd_id}'

    @staticmethod
    def _mav_result_name(result: int) -> str:
        """Human-readable name for a MAV_RESULT value."""
        _RESULT_MAP = {
            0: 'ACCEPTED',
            1: 'TEMPORARILY_REJECTED',
            2: 'DENIED',
            3: 'UNSUPPORTED',
            4: 'FAILED',
            5: 'IN_PROGRESS',
            6: 'CANCELLED',
        }
        return _RESULT_MAP.get(result, f'RESULT_{result}')

    def _close_master(self):
        """Safely close and null the MAVLink connection."""
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None

    def _find_pixhawk_port(self) -> str | None:
        """Find the Pixhawk serial port. Tries configured port first, then scans fallbacks."""
        # Try configured port first
        if Path(self._connection_port).exists():
            return self._connection_port
        # Scan fallback ports
        for port in self._fallback_ports:
            if port != self._connection_port and Path(port).exists():
                self.get_logger().info(f'Configured port {self._connection_port} not found, trying {port}')
                return port
        return None

    def _connect(self):
        """Connect to Pixhawk (runs in thread). Auto-detects port, handles backoff."""
        self._reconnecting = True
        try:
            # Always clean up stale connection first
            self._close_master()

            port = self._find_pixhawk_port()
            if port is None:
                raise ConnectionError(
                    f'No Pixhawk found on {self._connection_port} or fallbacks {self._fallback_ports}'
                )

            self.get_logger().info(f'Connecting to Pixhawk at {port}...')
            self._master = mavutil.mavlink_connection(port, self._baud)
            self._master.wait_heartbeat(timeout=10)
            self._connected = True
            self._last_heartbeat = time.time()
            self._heartbeat_lost_notified = False
            self._reconnect_backoff_current = self._reconnect_backoff  # reset backoff
            # Update connection_port to the port that actually worked
            self._connection_port = port
            self._publish_event('connected', f'Pixhawk connected on {port}')
            self.get_logger().info(f'Pixhawk connected and active on {port}.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self._publish_event('connection_failed', str(e))
            self._connected = False
            # Clean up on failure so stale fd doesn't block next attempt
            self._close_master()
            # Increase backoff for next attempt
            self._reconnect_backoff_current = min(
                self._reconnect_backoff_current * 2, self._reconnect_backoff_max
            )
        finally:
            self._reconnecting = False

    def _send_heartbeat(self):
        """Send GCS heartbeat and check connection health."""
        # --- Reconnection logic ---
        if not self._connected:
            if not self._reconnecting:
                delay = self._reconnect_backoff_current
                self.get_logger().info(
                    f'Not connected. Reconnecting in {delay:.0f}s...'
                )
                # Schedule reconnect after backoff delay in a thread
                def _delayed_reconnect():
                    time.sleep(delay)
                    if not self._connected and not self._reconnecting:
                        self._connect()
                threading.Thread(target=_delayed_reconnect, daemon=True).start()
            return

        if self._master is None:
            return

        # --- Heartbeat loss detection ---
        now = time.time()
        if self._last_heartbeat > 0 and (now - self._last_heartbeat) > self._heartbeat_timeout:
            if not self._heartbeat_lost_notified:
                self._heartbeat_lost_notified = True
                elapsed = now - self._last_heartbeat
                self._publish_event(
                    'heartbeat_lost',
                    f'No Pixhawk heartbeat for {elapsed:.1f}s (timeout={self._heartbeat_timeout}s)'
                )
                self.get_logger().warn('Pixhawk heartbeat lost!')
            # If heartbeat lost for too long, mark disconnected to trigger reconnect
            if (now - self._last_heartbeat) > self._heartbeat_timeout * 3:
                self.get_logger().error('Heartbeat lost too long — marking disconnected for reconnect.')
                self._connected = False
                self._publish_event('connection_lost', 'Heartbeat timeout exceeded, will reconnect')
            return  # Don't send heartbeat to a dead link

        # --- Send GCS heartbeat ---
        try:
            self._master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
            )
        except Exception as e:
            self.get_logger().warn(f'Heartbeat send failed: {e}')
            self._connected = False

    def _read_mavlink(self):
        """Read and process MAVLink messages."""
        if not self._connected or self._master is None:
            return
        try:
            msg = self._master.recv_match(blocking=False)
            while msg is not None:
                self._process_message(msg)
                msg = self._master.recv_match(blocking=False)
        except Exception as e:
            self.get_logger().error(f'Read error: {e}')
            self._connected = False
            self._publish_event('connection_lost', str(e))

    def _process_message(self, msg):
        """Process a received MAVLink message."""
        msg_type = msg.get_type()
        if msg_type == 'HEARTBEAT':
            self._last_heartbeat = time.time()
            # Heartbeat restored after loss
            if self._heartbeat_lost_notified:
                self._heartbeat_lost_notified = False
                self._publish_event('heartbeat_restored', 'Pixhawk heartbeat restored')
                self.get_logger().info('Pixhawk heartbeat restored.')
            self._armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            mode_id = msg.custom_mode
            self._flight_mode = self._get_mode_name(mode_id)
            if self._prev_armed is not None and self._prev_armed != self._armed:
                ev = 'armed' if self._armed else 'disarmed'
                self._publish_event(ev, f'Motors {"ARMED" if self._armed else "DISARMED"}')
            self._prev_armed = self._armed
            if self._prev_mode is not None and self._prev_mode != self._flight_mode:
                self._publish_event('mode_change', f'Flight mode: {self._flight_mode}')
            self._prev_mode = self._flight_mode
        elif msg_type == 'AHRS2':
            self._depth = msg.altitude
            self._yaw = math.degrees(msg.yaw) % 360
            self._pitch = math.degrees(msg.pitch)
            self._roll = math.degrees(msg.roll)
        elif msg_type == 'ATTITUDE':
            yaw_rad = msg.yaw
            if yaw_rad < 0:
                yaw_rad += 2 * math.pi
            self._yaw = math.degrees(yaw_rad) % 360
            self._heading_rate = math.degrees(msg.yawspeed)
        elif msg_type == 'SYS_STATUS':
            self._voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 0xFFFF else 0
            self._current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            self._cpu_load = msg.load / 10.0  # SYS_STATUS.load is in 0.1% units
        elif msg_type == 'SCALED_PRESSURE':
            self._pressure = msg.press_abs      # hPa
            self._temperature = msg.temperature / 100.0  # cdegC → °C
        elif msg_type == 'SERVO_OUTPUT_RAW':
            self._servo_output = [
                msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
            ]
        elif msg_type == 'RC_CHANNELS':
            self._rc_channels = [
                msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
            ]
        elif msg_type == 'COMMAND_ACK':
            cmd_id = msg.command
            result = msg.result
            cmd_name = self._mav_cmd_name(cmd_id)
            result_name = self._mav_result_name(result)

            # Match against pending ACK registry
            with self._pending_acks_lock:
                pending = self._pending_acks.pop(cmd_id, None)

            desc = pending['desc'] if pending else cmd_name

            # Build human-readable result message
            if result == 0:  # ACCEPTED
                ack_msg = f'{desc}: ACCEPTED'
                ack_type = 'command_accepted'
            elif result == 1:  # TEMPORARILY_REJECTED
                ack_msg = f'{desc}: TEMPORARILY REJECTED (retry later)'
                ack_type = 'command_rejected'
            elif result == 2:  # DENIED
                ack_msg = f'{desc}: DENIED (bad params or state)'
                ack_type = 'command_denied'
            elif result == 3:  # UNSUPPORTED
                ack_msg = f'{desc}: UNSUPPORTED by firmware'
                ack_type = 'command_denied'
            elif result == 4:  # FAILED
                ack_msg = f'{desc}: FAILED'
                ack_type = 'command_failed'
            elif result == 5:  # IN_PROGRESS
                progress = getattr(msg, 'progress', 255)
                pct = f' ({progress}%)' if progress != 255 else ''
                ack_msg = f'{desc}: IN PROGRESS{pct}'
                ack_type = 'command_ack'
                # Re-register so we get the final result too
                if pending:
                    with self._pending_acks_lock:
                        self._pending_acks[cmd_id] = pending
                    pending = None  # don't resolve event yet
            elif result == 6:  # CANCELLED
                ack_msg = f'{desc}: CANCELLED'
                ack_type = 'command_cancelled'
            else:
                ack_msg = f'{desc}: {result_name}'
                ack_type = 'command_ack'

            self._publish_event(ack_type, ack_msg,
                                raw_data=str(msg.to_dict()))

            # Resolve the pending event so callers waiting on it unblock
            if pending is not None:
                pending['result'] = result
                pending['event'].set()

    def _get_mode_name(self, mode_id: int) -> str:
        """Get mode name from ID."""
        if self._master is None:
            return 'UNKNOWN'
        mapping = self._master.mode_mapping()
        for name, mid in mapping.items():
            if mid == mode_id:
                return name
        return f'MODE_{mode_id}'

    def _publish_state(self):
        """Publish current vehicle state. Safe to call during shutdown."""
        try:
            if not rclpy.ok():
                return
            msg = VehicleState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.armed = self._armed
            msg.flight_mode = self._flight_mode
            msg.depth = float(self._depth)
            msg.yaw = float(self._yaw)
            msg.pitch = float(self._pitch)
            msg.roll = float(self._roll)
            msg.voltage = float(self._voltage)
            msg.current = float(self._current)
            self._state_pub.publish(msg)
        except Exception:
            pass

    def _publish_diagnostics(self):
        """Publish diagnostics snapshot (2 Hz)."""
        try:
            if not rclpy.ok():
                return
            msg = VehicleDiagnostics()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.heading_rate = float(self._heading_rate)
            msg.pressure = float(self._pressure)
            msg.temperature = float(self._temperature)
            msg.servo_output = [int(v) for v in self._servo_output]
            msg.rc_channels = [int(v) for v in self._rc_channels]
            msg.cpu_load = float(self._cpu_load)
            self._diag_pub.publish(msg)
        except Exception:
            pass

    def _send_rc_channels(self, channels: dict[int, int]):
        """Send a single RC_CHANNELS_OVERRIDE with all specified channels.

        Args:
            channels: {channel_id (1-18): pwm_value} dict.
                      Unspecified channels default to 65535 (no change).
                      PWM values are clamped to 1100-1900.
        """
        if not self._connected or self._master is None:
            return
        rc = [65535] * 18
        for ch, pwm in channels.items():
            if ch < 1 or ch > 18:
                self.get_logger().warn(f'Invalid RC channel {ch}, skipping')
                continue
            rc[ch - 1] = int(max(1100, min(1900, pwm)))
        try:
            self._master.mav.rc_channels_override_send(
                self._master.target_system, self._master.target_component, *rc
            )
        except Exception as e:
            self.get_logger().error(f'RC override send failed: {e}')
            self._connected = False

    def _set_single_rc_channel(self, channel_id: int, pwm: int):
        """Set a single RC channel (for one-off commands like servo/grabber).

        For periodic RC override, use _send_rc_channels() instead.
        """
        self._send_rc_channels({channel_id: pwm})

    # ── Command ACK infrastructure ────────────────────────────────────

    def _send_command_long(self, mav_cmd: int, p1=0, p2=0, p3=0, p4=0,
                           p5=0, p6=0, p7=0, description: str = '',
                           confirmation: int = 0) -> threading.Event | None:
        """Send a COMMAND_LONG and register it for ACK tracking.

        Returns a threading.Event that will be set when the ACK arrives
        (or on timeout). Caller can optionally wait on it.
        Returns None if not connected.
        """
        if not self._connected or self._master is None:
            return None
        desc = description or self._mav_cmd_name(mav_cmd)
        self.get_logger().info(
            f'TX COMMAND_LONG  {desc}  '
            f'cmd={mav_cmd} p1={p1} p2={p2} p3={p3} p4={p4} p5={p5} p6={p6} p7={p7}'
        )
        ack_event = threading.Event()
        with self._pending_acks_lock:
            self._pending_acks[mav_cmd] = {
                'sent_at': time.time(),
                'desc': desc,
                'event': ack_event,
                'result': None,
            }
        try:
            self._master.mav.command_long_send(
                self._master.target_system, self._master.target_component,
                mav_cmd, confirmation,
                p1, p2, p3, p4, p5, p6, p7
            )
        except Exception as e:
            self.get_logger().error(f'command_long_send failed: {e}')
            with self._pending_acks_lock:
                self._pending_acks.pop(mav_cmd, None)
            return None
        return ack_event

    def _check_ack_timeouts(self):
        """Check for COMMAND_LONG ACKs that never arrived."""
        now = time.time()
        timed_out = []
        with self._pending_acks_lock:
            for cmd_id, info in list(self._pending_acks.items()):
                if now - info['sent_at'] > self._ack_timeout:
                    timed_out.append((cmd_id, info))
            for cmd_id, _ in timed_out:
                self._pending_acks.pop(cmd_id, None)
        for cmd_id, info in timed_out:
            desc = info['desc']
            self._publish_event(
                'command_timeout',
                f'{desc}: NO RESPONSE (timeout {self._ack_timeout:.0f}s)',
            )
            info['result'] = -1  # sentinel for timeout
            info['event'].set()

        # ── Mode change verification ──
        pmc = self._pending_mode_change
        if pmc is not None:
            if self._flight_mode == pmc['target']:
                self._publish_event('mode_verified',
                    f'Mode change confirmed: {pmc["target"]}')
                self._pending_mode_change = None
            elif now - pmc['sent_at'] > self._ack_timeout:
                self._publish_event('mode_timeout',
                    f'Mode change to {pmc["target"]} NOT confirmed '
                    f'(current: {self._flight_mode})')
                self._pending_mode_change = None

    def _set_target_attitude(self, roll: float, pitch: float, yaw: float):
        """Set target attitude (degrees)."""
        if not self._connected or self._master is None:
            return
        self._master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self._boot_time)),
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            QuaternionBase([math.radians(a) for a in (roll, pitch, yaw)]),
            0, 0, 0, 0
        )

    def _set_target_depth(self, depth: float):
        """Set target depth (negative = below surface) via SET_POSITION_TARGET_GLOBAL_INT.

        Requires ALT_HOLD mode. Use the proper typemask as documented:
        https://www.ardusub.com/developers/pymavlink.html#set-target-depthattitude

        POOL TODO: Confirm that sending depth as negative altitude works with our
        barometer. ArduSub uses barometric pressure → altitude 0=surface, negative=down.
        If depth reads wrong, check AHRS.altitude vs VFR_HUD.alt.
        """
        if not self._connected or self._master is None:
            return
        # Proper typemask: ignore everything except Z position
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # NOT Z_IGNORE — we use Z (depth)
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # NOT FORCE_SET
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self._master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self._boot_time)),
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            0, 0, depth,  # lat, lon (ignored), alt (depth)
            0, 0, 0,      # velocities (ignored)
            0, 0, 0,      # accelerations (ignored)
            0, 0           # yaw, yaw_rate (ignored)
        )

    def _resend_depth_target(self):
        """Re-send ALT_HOLD depth target periodically so ArduSub maintains the setpoint.

        POOL TODO: Check if 2Hz re-send rate (0.5s timer) is sufficient. If
        ArduSub drops the target between sends, increase to 5Hz (0.2s timer).
        """
        if self._alt_hold_target is not None and self._connected:
            self._set_target_depth(self._alt_hold_target)

    # Neutral channels dict — reused for idle/stop sends
    _NEUTRAL_CHANNELS = {
        CH_PITCH: NEUTRAL_PWM, CH_ROLL: NEUTRAL_PWM,
        CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM,
        CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM,
    }

    def _stop_all(self):
        """Stop all thrusters (neutral) in a single RC message."""
        with self._movement_lock:
            self._current_movement = None
            self._yaw_to_heading = None
            self._depth_pid = None
        self._alt_hold_target = None
        self._send_rc_channels(self._NEUTRAL_CHANNELS)
        self._publish_event('movement', 'Stop - all thrusters neutral')

    def _angle_error(self, current: float, target: float) -> float:
        """Shortest-path angle error in [-180, 180] degrees."""
        err = (target - current) % 360
        if err > 180:
            err -= 360
        return err

    def _send_rc_override(self):
        """Send RC override continuously — ArduSub failsafes (disarms) if RC stops.

        Builds a single channel dict per tick with layered priority:
          1. Base: neutral on all channels
          2. Active movement (sets forward, lateral, throttle, yaw)
          3. Depth PID (overrides CH_THROTTLE)
          4. Yaw-to-heading PID/bang-bang (overrides CH_YAW)
        Sends one RC_CHANNELS_OVERRIDE per tick (~20 Hz).
        """
        if not self._connected or self._master is None:
            return

        now = time.time()
        with self._movement_lock:
            mv = self._current_movement
            y2h = self._yaw_to_heading
            dh = self._depth_pid

        # ── Movement expiry ──────────────────────────────────────────────
        if mv is not None and now >= mv['end_time']:
            with self._movement_lock:
                self._current_movement = None
            self._publish_event('movement', 'Movement duration expired')
            mv = None  # don't overlay expired movement this tick

        # ── Layer 1: base neutral ────────────────────────────────────────
        channels = dict(self._NEUTRAL_CHANNELS)

        # ── Layer 2: active movement ─────────────────────────────────────
        if mv is not None:
            channels.update(mv.get('channels', {}))

        # ── Layer 3: depth PID (overrides CH_THROTTLE) ───────────────────
        if dh is not None:
            depth_error = dh['target_m'] - self._depth
            dt = now - dh.get('last_time', now)
            if dt <= 0:
                dt = 0.05  # fallback to one tick

            # Proportional
            p_out = dh['kp'] * depth_error

            # Integral with anti-windup
            integral = dh.get('integral', 0.0) + depth_error * dt
            max_i = dh.get('max_integral', self._depth_max_integral)
            integral = max(-max_i, min(max_i, integral))
            i_out = dh['ki'] * integral

            # Derivative (on error, with dt guard)
            last_err = dh.get('last_error', depth_error)
            d_out = dh['kd'] * (depth_error - last_err) / dt

            # Total PID → throttle PWM
            # Depth convention: altitude 0 = surface, negative = below.
            # Error negative → need to go deeper → throttle > 1500.
            # Formula: throttle_pwm = NEUTRAL - pid_output
            #   (negative pid → positive offset → down)
            # POOL TODO: Verify throttle direction! If AUV goes UP when told to
            #   go DOWN (or vice versa), flip the sign: change `-pid_output` to
            #   `pid_output` below. This is the most critical thing to check.
            pid_output = p_out + i_out + d_out
            throttle_offset = max(-PWM_RANGE, min(PWM_RANGE, int(-pid_output)))
            channels[CH_THROTTLE] = NEUTRAL_PWM + throttle_offset

            # Update PID state
            with self._movement_lock:
                if self._depth_pid is not None:
                    self._depth_pid['integral'] = integral
                    self._depth_pid['last_error'] = depth_error
                    self._depth_pid['last_time'] = now

        # ── Layer 4: yaw-to-heading (overrides CH_YAW) ──────────────────
        if y2h is not None:
            err = self._angle_error(self._yaw, y2h['target_deg'])
            if abs(err) <= y2h.get('tolerance_deg', 5.0):
                with self._movement_lock:
                    self._yaw_to_heading = None
                self._publish_event('movement', f'Heading reached: {y2h["target_deg"]}°')
            elif y2h.get('use_pid', False):
                # ── PID yaw controller ──
                now_t = time.time()
                dt = now_t - y2h.get('last_time', now_t)
                if dt <= 0:
                    dt = 0.05

                p_out = y2h['kp'] * err
                integral = y2h.get('integral', 0.0) + err * dt
                max_i = y2h.get('max_integral', self._yaw_max_integral)
                integral = max(-max_i, min(max_i, integral))
                i_out = y2h['ki'] * integral
                last_err = y2h.get('last_error', err)
                d_out = y2h['kd'] * (err - last_err) / dt

                pid_output = p_out + i_out + d_out
                pwm_offset = max(-PWM_RANGE, min(PWM_RANGE, int(pid_output)))
                channels[CH_YAW] = NEUTRAL_PWM + pwm_offset

                with self._movement_lock:
                    if self._yaw_to_heading is not None:
                        self._yaw_to_heading['integral'] = integral
                        self._yaw_to_heading['last_error'] = err
                        self._yaw_to_heading['last_time'] = now_t
            else:
                # ── Bang-bang yaw controller ──
                offset = y2h['gain_offset']
                channels[CH_YAW] = NEUTRAL_PWM + (offset if err > 0 else -offset)

        # ── Send combined channels ───────────────────────────────────────
        self._send_rc_channels(channels)

    def _on_driver_command(self, cmd: DriverCommand):
        """Handle incoming driver command."""
        if not self._connected or self._master is None:
            self.get_logger().warn('Ignoring command - not connected')
            return
        c = cmd.command.lower()

        # Log every incoming command with relevant parameters
        params = []
        if cmd.mode:
            params.append(f'mode={cmd.mode}')
        if cmd.depth != 0.0:
            params.append(f'depth={cmd.depth}')
        if cmd.angle != 0.0:
            params.append(f'angle={cmd.angle}')
        if cmd.duration > 0:
            params.append(f'dur={cmd.duration}s')
        if cmd.speed != 0:
            params.append(f'speed={cmd.speed}')
        param_str = f'  ({", ".join(params)})' if params else ''
        self.get_logger().info(f'RX command: {c}{param_str}')

        # Commands allowed when disarmed
        UNARMED_ALLOWED = {'arm', 'disarm', 'set_mode', 'stop', 'pid_depth_off', 'surface'}
        if not self._armed and c not in UNARMED_ALLOWED:
            self.get_logger().warn(f'Rejecting "{c}" - vehicle not armed. Arm first.')
            self._publish_event('command_rejected', f'"{c}" rejected: vehicle not armed')
            return

        # Speed: 0-100 = percent (use percent_to_pwm), else PWM offset from 1500
        raw_speed = cmd.speed if cmd.speed != 0 else 50
        if 0 < raw_speed <= 100:
            speed = percent_to_pwm(raw_speed)
        else:
            speed = NEUTRAL_PWM + max(-PWM_RANGE, min(PWM_RANGE, int(raw_speed)))
        duration = cmd.duration
        if duration < 0:
            duration = 0
        end_time = time.time() + duration if duration > 0 else float('inf')

        def set_movement(channels: dict, desc: str):
            with self._movement_lock:
                self._current_movement = {'channels': channels, 'end_time': end_time}
            self._publish_event('movement', desc)

        if c == 'stop':
            self._stop_all()

        elif c in ('move_forward', 'forward'):
            set_movement(
                {CH_FORWARD: speed, CH_LATERAL: NEUTRAL_PWM, CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM},
                f'Moving forward (speed={speed})'
            )

        elif c in ('move_back', 'back', 'backward'):
            set_movement(
                {CH_FORWARD: NEUTRAL_PWM - (speed - NEUTRAL_PWM), CH_LATERAL: NEUTRAL_PWM, CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM},
                'Moving backward'
            )

        elif c in ('move_left', 'left'):
            set_movement(
                {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM - (speed - NEUTRAL_PWM), CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM},
                'Moving left'
            )

        elif c in ('move_right', 'right'):
            set_movement(
                {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: speed, CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM},
                'Moving right'
            )

        elif c in ('move_up', 'up'):
            set_movement(
                {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM, CH_THROTTLE: NEUTRAL_PWM - (speed - NEUTRAL_PWM), CH_YAW: NEUTRAL_PWM},
                'Moving up'
            )

        elif c in ('move_down', 'down'):
            set_movement(
                {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM, CH_THROTTLE: speed, CH_YAW: NEUTRAL_PWM},
                'Moving down'
            )

        # ── Compound diagonal movement ──────────────────────────────────
        # Handles 2-axis horizontal diagonals: move_forward_right,
        # move_back_left, etc.  Speed is scaled by 1/√2 so the
        # resultant vector keeps the requested magnitude.
        elif c.startswith('move_') and '_' in c[5:]:
            parts = c[5:].split('_')  # e.g. ['forward', 'right']
            result = _build_diagonal_channels(parts, speed)
            if result:
                channels, label = result
                n = len(parts)
                scaled = int((speed - NEUTRAL_PWM) / math.sqrt(n))
                set_movement(channels,
                    f'Moving {label} ({n}-axis, ±{abs(scaled)}pwm/axis, speed={speed})')
            else:
                self.get_logger().warn(f'Invalid compound direction: {c}')

        elif c == 'yaw_angle':
            # Legacy: set_attitude_target (may not work in MANUAL)
            self._set_target_attitude(0, 0, cmd.angle)
            self._publish_event('movement', f'Setting heading to {cmd.angle}°')

        elif c == 'yaw_to_heading':
            # Bang-bang: use thrusters to rotate to target heading (works in MANUAL)
            gain_offset = abs(speed - NEUTRAL_PWM) or PWM_RANGE // 2
            with self._movement_lock:
                self._yaw_to_heading = {
                    'target_deg': cmd.angle % 360,
                    'gain_offset': min(PWM_RANGE, gain_offset),
                    'tolerance_deg': 5.0,
                    'use_pid': False,
                }
            self._publish_event('movement', f'Yaw to heading {cmd.angle}° (bang-bang)')

        elif c == 'pid_yaw_to_heading':
            # PID: use thrusters with proportional-integral-derivative control
            gain_offset = abs(speed - NEUTRAL_PWM) or PWM_RANGE // 2
            with self._movement_lock:
                self._yaw_to_heading = {
                    'target_deg': cmd.angle % 360,
                    'gain_offset': min(PWM_RANGE, gain_offset),
                    'tolerance_deg': 3.0,
                    'use_pid': True,
                    'kp': self._yaw_kp,
                    'ki': self._yaw_ki,
                    'kd': self._yaw_kd,
                    'integral': 0.0,
                    'last_error': 0.0,
                    'last_time': time.time(),
                    'max_integral': self._yaw_max_integral,
                }
            self._publish_event('movement', f'PID yaw to heading {cmd.angle}° (Kp={self._yaw_kp} Ki={self._yaw_ki} Kd={self._yaw_kd})')

        elif c == 'yaw_left':
            set_movement(
                {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM, CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM - (speed - NEUTRAL_PWM)},
                'Yaw left'
            )

        elif c == 'yaw_right':
            set_movement(
                {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM, CH_THROTTLE: NEUTRAL_PWM, CH_YAW: speed},
                'Yaw right'
            )

        # ── Simultaneous movement + heading (go) ────────────────────────
        # The RC override tick already layers: movement → depth PID → yaw PID.
        # These 'go_*' commands set BOTH _current_movement (direction channels)
        # AND _yaw_to_heading (PID yaw) simultaneously so the AUV moves in a
        # direction while rotating to a target heading.
        # Supports single (go_forward) and compound (go_forward_right) directions.
        # Only horizontal axes (forward/back/left/right) — vertical is
        # controlled by depth PID; yaw is controlled by heading PID.
        elif c.startswith('go_'):
            dir_str = c[3:]  # e.g. 'forward' or 'forward_right'
            parts = dir_str.split('_')
            # Filter to horizontal-only directions for go commands
            HORIZONTAL = {'forward', 'back', 'backward', 'left', 'right'}
            if not all(p in HORIZONTAL for p in parts):
                self.get_logger().warn(
                    f'go commands only support horizontal directions '
                    f'(forward/back/left/right), got: {dir_str}. '
                    f'Use p_dive for depth control.')
            else:
                result = _build_diagonal_channels(parts, speed)
                if result is None:
                    self.get_logger().warn(f'Invalid go direction: {c}')
                else:
                    mv_channels, label = result
                    # CH_YAW is controlled by PID, set to neutral in movement layer
                    mv_channels[CH_YAW] = NEUTRAL_PWM
                    # CH_THROTTLE left neutral (depth PID overrides if active)
                    mv_channels[CH_THROTTLE] = NEUTRAL_PWM
                    with self._movement_lock:
                        self._current_movement = {'channels': mv_channels, 'end_time': end_time}
                        self._yaw_to_heading = {
                            'target_deg': cmd.angle % 360,
                            'gain_offset': abs(speed - NEUTRAL_PWM) or PWM_RANGE // 2,
                            'tolerance_deg': 3.0,
                            'use_pid': True,
                            'kp': self._yaw_kp,
                            'ki': self._yaw_ki,
                            'kd': self._yaw_kd,
                            'integral': 0.0,
                            'last_error': 0.0,
                            'last_time': time.time(),
                            'max_integral': self._yaw_max_integral,
                        }
                    dur_str = f' {duration}s' if duration > 0 else ''
                    self._publish_event('movement',
                        f'Go {label} → {cmd.angle:.0f}°{dur_str} (speed={speed})')

        elif c in ('depth', 'set_depth'):
            d = float(cmd.depth)
            d = -abs(d) if d > 0 else d  # ArduSub: negative = below surface
            # Auto-switch to ALT_HOLD if not already (ArduSub firmware depth hold)
            if self._flight_mode != 'ALT_HOLD':
                self.get_logger().info('Auto-switching to ALT_HOLD for firmware depth hold')
                self._set_mode('ALT_HOLD')
                time.sleep(0.5)  # Give mode switch time to take effect
            self._alt_hold_target = d  # Store for periodic re-send
            self._set_target_depth(d)
            # Disable software depth PID if active (ALT_HOLD replaces it)
            with self._movement_lock:
                self._depth_pid = None
            self._publish_event('movement', f'ALT_HOLD depth target: {abs(d):.2f}m')

        elif c == 'pid_depth':
            # Software PID depth hold via RC throttle (works in any mode)
            # POOL TODO: Test procedure — start in MANUAL mode at surface:
            #   1. arm  →  p_dive 0.5  (shallow first, easy to recover)
            #   2. Watch throttle PWM in telemetry — should settle near 1500±small
            #   3. If oscillating: stop, reduce Kp via `ros2 param set`
            #   4. Gradually test deeper: p_dive 1.0, p_dive 1.5
            #   5. Only after PID is verified, test `dive` (ALT_HOLD) mode
            target = float(cmd.depth) if cmd.depth != 0.0 else self._depth
            target = -abs(target) if target > 0 else target  # Ensure negative
            if abs(target) < 0.02:
                target = self._depth  # "hold current depth"
            self._alt_hold_target = None  # Disable ALT_HOLD re-send
            with self._movement_lock:
                self._depth_pid = {
                    'target_m': target,
                    'kp': self._depth_kp,
                    'ki': self._depth_ki,
                    'kd': self._depth_kd,
                    'integral': 0.0,
                    'last_error': 0.0,
                    'last_time': time.time(),
                    'max_integral': self._depth_max_integral,
                }
            self._publish_event('movement',
                f'PID depth hold ON: target {abs(target):.2f}m '
                f'(Kp={self._depth_kp} Ki={self._depth_ki} Kd={self._depth_kd})')

        elif c == 'pid_depth_off':
            with self._movement_lock:
                self._depth_pid = None
            self._alt_hold_target = None
            self._publish_event('movement', 'PID depth hold OFF')

        elif c == 'surface':
            # Surface: stop everything, then command to near-surface
            # POOL TODO: Test surfacing from various depths (0.5m, 1m, 2m).
            #   - ALT_HOLD: verify target -0.1m doesn't slam into surface.
            #     If it does, change to -0.2m or add approach slowdown.
            #   - MANUAL: verify 50% throttle for 10s is enough to reach surface
            #     from max pool depth. Adjust PWM_RANGE//2 and duration.
            with self._movement_lock:
                self._current_movement = None
                self._yaw_to_heading = None
                self._depth_pid = None
            self._alt_hold_target = None
            if self._flight_mode == 'ALT_HOLD':
                self._alt_hold_target = -0.1
                self._set_target_depth(-0.1)
                self._publish_event('movement', 'Surfacing (ALT_HOLD to -0.1m)')
            else:
                # In MANUAL, command throttle up briefly
                with self._movement_lock:
                    self._current_movement = {
                        'channels': {
                            CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM,
                            CH_THROTTLE: NEUTRAL_PWM - PWM_RANGE // 2,  # Up at 50%
                            CH_YAW: NEUTRAL_PWM,
                        },
                        'end_time': time.time() + 10.0,  # Up for up to 10s
                    }
                self._publish_event('movement', 'Surfacing (MANUAL, throttle up 10s)')

        elif c == 'arm':
            self._arm_disarm(True)

        elif c == 'disarm':
            self._arm_disarm(False)

        elif c == 'set_mode':
            self._set_mode(cmd.mode)

        elif c == 'open_grabber':
            self._set_servo_pwm(1, 1100)
            self._publish_event('actuator', 'Grabber open')

        elif c == 'close_grabber':
            self._set_servo_pwm(1, 1900)
            self._publish_event('actuator', 'Grabber close')

        else:
            self.get_logger().warn(f'Unknown command: {c}')

    def _set_servo_pwm(self, servo_n: int, microseconds: int):
        """Set servo PWM (AUX output).

        Note: ArduSub may not send COMMAND_ACK for DO_SET_SERVO, so we send
        directly without ACK tracking to avoid false timeout warnings.
        """
        if not self._connected or self._master is None:
            return
        self.get_logger().info(
            f'TX COMMAND_LONG  DO_SET_SERVO  ch={servo_n + 8} pwm={microseconds}'
        )
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            servo_n + 8, microseconds, 0, 0, 0, 0, 0
        )

    def _arm_disarm(self, do_arm: bool):
        """Arm or disarm the vehicle.

        Confirmation comes from heartbeat flags (armed bit) which is already
        processed in _process_message → HEARTBEAT handler.  We do NOT use
        motors_armed_wait() because it calls recv_match() which races with our
        _read_mavlink loop and consumes messages (including any COMMAND_ACK).
        ArduSub on Pixhawk 2.4.8 does not reliably ACK arm/disarm, so we also
        skip _send_command_long ACK tracking to avoid false timeouts.
        """
        if not self._connected or self._master is None:
            return
        action = 'Arming' if do_arm else 'Disarming'
        v = 1 if do_arm else 0
        self.get_logger().info(
            f'TX COMMAND_LONG  COMPONENT_ARM_DISARM  arm={v}'
        )
        self._publish_event('arm' if do_arm else 'disarm', f'{action}...')
        try:
            self._master.mav.command_long_send(
                self._master.target_system, self._master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                v, 0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            self._publish_event('arm_failed' if do_arm else 'disarm_failed', str(e))

    def _set_mode(self, mode: str):
        """Set flight mode. Uses COMMAND_LONG for ACK tracking + heartbeat verification."""
        if not self._connected or self._master is None:
            return
        mode = (mode or 'MANUAL').upper()
        if mode not in self._master.mode_mapping():
            self.get_logger().error(f'Unknown mode: {mode}. Available: {list(self._master.mode_mapping().keys())}')
            return
        mode_id = self._master.mode_mapping()[mode]
        # Use COMMAND_LONG + MAV_CMD_DO_SET_MODE for proper ACK tracking
        # param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2 = custom_mode_id
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=mode_id,
            description=f'SET_MODE {mode}',
        )
        # Register for heartbeat-based verification (belt + suspenders)
        self._pending_mode_change = {
            'target': mode,
            'sent_at': time.time(),
        }
        self._publish_event('mode_change', f'Setting mode to {mode}')


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkInspectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send neutral RC directly to MAVLink (bypass ROS publish)
        try:
            if node._master and node._connected:
                rc = [NEUTRAL_PWM] * 8 + [65535] * 10
                node._master.mav.rc_channels_override_send(
                    node._master.target_system,
                    node._master.target_component, *rc
                )
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
