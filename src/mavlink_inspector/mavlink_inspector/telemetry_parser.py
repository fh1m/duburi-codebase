"""Telemetry parser for MAVLink messages from Pixhawk.

Parses sensor and state messages, updates vehicle state fields, and
returns event tuples for state transitions (arm/disarm, mode change).

COMMAND_ACK is NOT handled here — the orchestrator manages ACK tracking.
"""

from __future__ import annotations

import math

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

    # ── Public API ───────────────────────────────────────────────────

    def process(self, msg, master=None) -> list[tuple[str, str, str]]:
        """Process a MAVLink message.

        Returns:
            List of ``(event_type, description, raw_data)`` tuples for
            state-transition events.  Empty list for sensor-only messages.
        """
        events: list[tuple[str, str, str]] = []
        msg_type = msg.get_type()

        if msg_type == 'HEARTBEAT':
            self._handle_heartbeat(msg, master, events)
        elif msg_type == 'AHRS2':
            self._handle_ahrs2(msg)
        elif msg_type == 'ATTITUDE':
            self._handle_attitude(msg)
        elif msg_type == 'SYS_STATUS':
            self._handle_sys_status(msg)
        elif msg_type == 'SCALED_PRESSURE':
            self._handle_scaled_pressure(msg)
        elif msg_type == 'SERVO_OUTPUT_RAW':
            self._handle_servo_output(msg)
        elif msg_type == 'RC_CHANNELS':
            self._handle_rc_channels(msg)
        # COMMAND_ACK handled by orchestrator

        return events

    # ── Message handlers ─────────────────────────────────────────────

    def _handle_heartbeat(self, msg, master, events):
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

    def _handle_ahrs2(self, msg):
        # Depth always comes from AHRS2
        self.prev_depth = self.depth
        self.depth = msg.altitude
        # BUG6 FIX: yaw only from AHRS2 if yaw_source='ahrs2' or 'both'
        if self._yaw_source in ('ahrs2', 'both'):
            self.prev_yaw = self.yaw
            self.yaw = math.degrees(msg.yaw) % 360
        self.pitch = math.degrees(msg.pitch)
        self.roll = math.degrees(msg.roll)

    def _handle_attitude(self, msg):
        yaw_rad = msg.yaw
        if yaw_rad < 0:
            yaw_rad += 2 * math.pi
        # BUG6 FIX: yaw only from ATTITUDE if yaw_source='attitude' or 'both'
        if self._yaw_source in ('attitude', 'both'):
            self.prev_yaw = self.yaw
            self.yaw = math.degrees(yaw_rad) % 360
        self.heading_rate = math.degrees(msg.yawspeed)

    def _handle_sys_status(self, msg):
        self.voltage = (
            msg.voltage_battery / 1000.0
            if msg.voltage_battery != 0xFFFF else 0
        )
        self.current = (
            msg.current_battery / 100.0
            if msg.current_battery != -1 else 0
        )
        self.cpu_load = msg.load / 10.0  # 0.1% units → %

    def _handle_scaled_pressure(self, msg):
        self.pressure = msg.press_abs           # hPa
        self.temperature = msg.temperature / 100.0  # cdegC → °C

    def _handle_servo_output(self, msg):
        self.servo_output = [
            msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
            msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
        ]

    def _handle_rc_channels(self, msg):
        self.rc_channels = [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
        ]

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
