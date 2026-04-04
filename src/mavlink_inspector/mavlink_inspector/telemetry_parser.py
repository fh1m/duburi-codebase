"""Telemetry parser for MAVLink messages from Pixhawk.

Parses sensor and state messages, updates vehicle state fields, and
returns event tuples for state transitions (arm/disarm, mode change).

COMMAND_ACK is NOT handled here — the orchestrator manages ACK tracking.
"""

from __future__ import annotations

import math
import time

from pymavlink import mavutil


class TelemetryParser:
    """Parses MAVLink messages and maintains vehicle state."""

    def __init__(self, yaw_source: str = 'attitude'):
        self._yaw_source = yaw_source

        # ── Vehicle state ────────────────────────────────────────────
        self.armed: bool = False
        self.flight_mode: str = 'UNKNOWN'
        self.depth: float = 0.0
        self.prev_depth: float = 0.0
        self.yaw: float = 0.0
        self.prev_yaw: float = 0.0
        self.pitch: float = 0.0
        self.roll: float = 0.0
        self.voltage: float = 0.0
        self.current: float = 0.0

        # ── Diagnostics ──────────────────────────────────────────────
        self.heading_rate: float = 0.0   # deg/s from ATTITUDE.yawspeed
        self.pressure: float = 0.0       # hPa
        self.temperature: float = 0.0    # °C
        self.servo_output: list[int] = [0] * 8
        self.rc_channels: list[int] = [0] * 8
        self.cpu_load: float = 0.0       # %

        # ── Transition tracking ──────────────────────────────────────
        self._prev_armed: bool | None = None
        self._prev_mode: str | None = None

        # ── Depth calibration ────────────────────────────────────────
        self.surface_pressure: float = 0.0  # Calibrated surface pressure (hPa)
        
        # ── Issue #27: MAVLink message rate watchdog ──────────────────
        # Track timestamps for stale telemetry detection
        self._last_attitude = 0.0
        self._last_ahrs2 = 0.0
        self._last_depth = 0.0
        self._last_imu = 0.0
        self._last_heartbeat = 0.0
        self._watchdog_timeout = 2.0  # seconds (stale if not received in this time)

        # ── Dispatch table (Design Issue 5) ──────────────────────────
        # Uniform signature: handler(msg, master, events)
        # Add new message types here — one method + one dict entry.
        self._dispatch: dict[str, callable] = {
            'HEARTBEAT':       self._handle_heartbeat,
            'AHRS2':           self._handle_ahrs2,
            'ATTITUDE':        self._handle_attitude,
            'SYS_STATUS':      self._handle_sys_status,
            'SCALED_PRESSURE': self._handle_scaled_pressure,
            'SERVO_OUTPUT_RAW': self._handle_servo_output,
            'RC_CHANNELS':     self._handle_rc_channels,
            'SCALED_IMU2':     self._handle_scaled_imu,  # Primary IMU with body-frame accel
        }

    # ── Public API ───────────────────────────────────────────────────

    def process(self, msg, master=None) -> list[tuple[str, str, str]]:
        """Process a MAVLink message.

        Returns:
            List of ``(event_type, description, raw_data)`` tuples for
            state-transition events.  Empty list for sensor-only messages.

        COMMAND_ACK is NOT dispatched here — the orchestrator handles it.
        """
        events: list[tuple[str, str, str]] = []
        handler = self._dispatch.get(msg.get_type())
        if handler is not None:
            handler(msg, master, events)
        return events
    
    def get_orientation(self):
        """
        Get current vehicle orientation as a quaternion.
        
        Converts stored Euler angles (roll, pitch, yaw) to quaternion format.
        This is needed for gravity rotation correction in velocity estimation.
        
        Returns:
            Object with (w, x, y, z) quaternion components representing
            the rotation from world frame to body frame.
            Returns None if scipy is not available.
        """
        try:
            from scipy.spatial.transform import Rotation
        except ImportError:
            # Fallback: scipy not available, return identity quaternion
            class Quaternion:
                w, x, y, z = 1.0, 0.0, 0.0, 0.0
            return Quaternion()
        
        try:
            # Convert Euler angles to quaternion
            # Roll (X), Pitch (Y), Yaw (Z) in degrees to radians
            r = Rotation.from_euler('xyz', 
                                    [self.roll, self.pitch, self.yaw], 
                                    degrees=True)
            # scipy returns [x, y, z, w], we need [w, x, y, z]
            q_scipy = r.as_quat()
            
            # Create a simple quaternion object
            class Quaternion:
                pass
            
            quat = Quaternion()
            quat.x = q_scipy[0]
            quat.y = q_scipy[1]
            quat.z = q_scipy[2]
            quat.w = q_scipy[3]
            
            return quat
        except Exception:
            # Return identity quaternion on any error
            class Quaternion:
                w, x, y, z = 1.0, 0.0, 0.0, 0.0
            return Quaternion()
    
    def check_watchdog(self) -> dict:
        """
        Check for stale MAVLink messages (Issue #27).
        
        Returns:
            Dictionary of {message_type: seconds_since_last_receipt} for
            messages that haven't been received within the watchdog timeout.
            Empty dict if all critical messages are current.
        """
        now = time.time()
        stale = {}
        
        if self._last_heartbeat > 0 and now - self._last_heartbeat > self._watchdog_timeout:
            stale['heartbeat'] = now - self._last_heartbeat
        if self._last_attitude > 0 and now - self._last_attitude > self._watchdog_timeout:
            stale['attitude'] = now - self._last_attitude
        if self._last_ahrs2 > 0 and now - self._last_ahrs2 > self._watchdog_timeout:
            stale['ahrs2'] = now - self._last_ahrs2
        if self._last_depth > 0 and now - self._last_depth > self._watchdog_timeout:
            stale['depth'] = now - self._last_depth
        if self._last_imu > 0 and now - self._last_imu > self._watchdog_timeout:
            stale['imu'] = now - self._last_imu
            
        return stale

    # ── Message handlers ─────────────────────────────────────────────

    def _handle_heartbeat(self, msg, master, events):
        # Only treat HEARTBEAT messages from an autopilot as ground truth.
        # GCS / onboard-controller heartbeats use MAV_AUTOPILOT_INVALID and
        # should not flip armed/mode state.
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return
        
        # Issue #27: Update heartbeat timestamp
        self._last_heartbeat = time.time()

        self.armed = (
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ) != 0
        self.flight_mode = self._get_mode_name(msg.custom_mode, master)

        # Arm / disarm transition events
        if self._prev_armed is not None and self._prev_armed != self.armed:
            if self.armed:
                events.append(('armed', 'Motors ARMED', ''))
            else:
                events.append(('disarmed', 'Motors DISARMED', ''))
        self._prev_armed = self.armed

        # Mode change events
        if (self._prev_mode is not None
                and self._prev_mode != self.flight_mode):
            events.append(
                ('mode_change', f'Flight mode: {self.flight_mode}', '')
            )
        self._prev_mode = self.flight_mode

    def _handle_ahrs2(self, msg, _master, _events):
        # CRITICAL FIX #3: Yaw/pitch/roll from AHRS2 only.
        # NOTE: Depth now computed from SCALED_PRESSURE (not MSL altitude)
        # Issue #27: Update AHRS2 timestamp
        self._last_ahrs2 = time.time()
        
        if self._yaw_source in ('ahrs2', 'both'):
            self.prev_yaw = self.yaw
            self.yaw = math.degrees(msg.yaw) % 360
        self.pitch = math.degrees(msg.pitch)
        self.roll = math.degrees(msg.roll)

    def _handle_attitude(self, msg, _master, _events):
        # Issue #27: Update ATTITUDE timestamp
        self._last_attitude = time.time()
        
        yaw_rad = msg.yaw
        if yaw_rad < 0:
            yaw_rad += 2 * math.pi
        # BUG6 FIX: yaw only from ATTITUDE if yaw_source='attitude' or 'both'
        if self._yaw_source in ('attitude', 'both'):
            self.prev_yaw = self.yaw
            self.yaw = math.degrees(yaw_rad) % 360
        self.heading_rate = math.degrees(msg.yawspeed)

    def _handle_sys_status(self, msg, _master, _events):
        self.voltage = (
            msg.voltage_battery / 1000.0
            if msg.voltage_battery != 0xFFFF else 0
        )
        self.current = (
            msg.current_battery / 100.0
            if msg.current_battery != -1 else 0
        )
        self.cpu_load = msg.load / 10.0  # 0.1% units → %

    def _handle_scaled_pressure(self, msg, _master, _events):
        # Issue #27: Update depth message timestamp
        self._last_depth = time.time()
        
        self.pressure = msg.press_abs           # hPa
        self.temperature = msg.temperature / 100.0  # cdegC → °C
        
        # CRITICAL FIX #3: Compute depth from pressure (not from AHRS2 altitude)
        # For freshwater: 1 mbar ≈ 1 cm depth
        # depth = (current_pressure - surface_pressure) * 0.01 meters
        if self.surface_pressure > 0:
            self.prev_depth = self.depth
            self.depth = (self.pressure - self.surface_pressure) * 0.01
        else:
            # First reading - calibrate surface pressure and set depth to 0
            self.surface_pressure = self.pressure
            self.prev_depth = 0.0
            self.depth = 0.0

    def _handle_servo_output(self, msg, _master, _events):
        self.servo_output = [
            msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
            msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
        ]

    def _handle_rc_channels(self, msg, _master, _events):
        self.rc_channels = [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
        ]
    
    def _handle_scaled_imu(self, msg, _master, _events):
        """
        Parse SCALED_IMU2 message for body-frame acceleration.
        
        SCALED_IMU2 provides calibrated IMU data in body frame:
        - xacc, yacc, zacc in milliG (1000 = 1G = 9.81 m/s²)
        
        Body frame: X=forward, Y=right, Z=down (NED convention)
        """
        # Issue #27: Update IMU timestamp
        self._last_imu = time.time()
        
        # Convert milliG to m/s²
        G = 9.81
        self.accel_x = (msg.xacc / 1000.0) * G  # Forward
        self.accel_y = (msg.yacc / 1000.0) * G  # Right  
        self.accel_z = (msg.zacc / 1000.0) * G  # Down

    # ── Helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _get_mode_name(mode_id: int, master) -> str:
        if master is None:
            return 'UNKNOWN'
        mapping = master.mode_mapping()
        for name, mid in mapping.items():
            if mid == mode_id:
                return name
        return f'MODE_{mode_id}'
