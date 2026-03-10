"""Command handler for DriverCommand dispatch.

Replaces the 600+ line ``elif`` chain in the original inspector_node
with a dispatch-table approach.  Each command or command group has a
focused handler method.

The handler takes a reference to the inspector node for state access
and control method calls.  This is a deliberate coupling — the command
handler *is* the command layer of the inspector.
"""

from __future__ import annotations

import math
import time

from duburi_interfaces.msg import DriverCommand

from .pid_controller import PidController
from .rc_controller import (
    CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW,
    NEUTRAL_PWM, PWM_RANGE,
    build_diagonal_channels, percent_to_pwm,
)


class CommandHandler:
    """Dispatches DriverCommand messages to handler methods."""

    # Commands allowed when the vehicle is disarmed
    UNARMED_ALLOWED = frozenset({
        'arm', 'disarm', 'set_mode', 'stop', 'pid_depth_off',
        'surface', 'just_surface', 'teleop_idle', 'calibrate_depth',
    })

    def __init__(self, node):
        """
        Args:
            node: ``MavlinkInspectorNode`` reference — provides state,
                  publishers, and control methods.
        """
        self._n = node

        # Direct-lookup dispatch table
        self._dispatch: dict[str, callable] = {
            'stop':                self._cmd_stop,
            'move_forward':        self._cmd_move_forward,
            'forward':             self._cmd_move_forward,
            'move_back':           self._cmd_move_back,
            'back':                self._cmd_move_back,
            'backward':            self._cmd_move_back,
            'move_left':           self._cmd_move_left,
            'left':                self._cmd_move_left,
            'move_right':          self._cmd_move_right,
            'right':               self._cmd_move_right,
            'move_up':             self._cmd_move_up,
            'up':                  self._cmd_move_up,
            'move_down':           self._cmd_move_down,
            'down':                self._cmd_move_down,
            'move_at':             self._cmd_move_at,
            'yaw_angle':           self._cmd_yaw_angle,
            'yaw_to_heading':      self._cmd_yaw_to_heading,
            'pid_yaw_to_heading':  self._cmd_pid_yaw_to_heading,
            'yaw_left':            self._cmd_yaw_left,
            'yaw_right':           self._cmd_yaw_right,
            'depth':               self._cmd_depth,
            'set_depth':           self._cmd_depth,
            'pid_depth':           self._cmd_pid_depth,
            'pid_depth_off':       self._cmd_pid_depth_off,
            'calibrate_depth':     self._cmd_calibrate_depth,
            'surface':             self._cmd_surface,
            'arm':                 self._cmd_arm,
            'disarm':              self._cmd_disarm,
            'set_mode':            self._cmd_set_mode,
            'open_grabber':        self._cmd_open_grabber,
            'close_grabber':       self._cmd_close_grabber,
            'cruise':              self._cmd_cruise,
            'just_cruise':         self._cmd_just_cruise,
            'teleop':              self._cmd_teleop,
            'teleop_idle':         self._cmd_teleop_idle,
        }

    # ── Public entry point ───────────────────────────────────────────

    def handle(self, cmd: DriverCommand):
        """Dispatch a driver command."""
        n = self._n

        if not n._conn.connected or n._conn.master is None:
            n.get_logger().warn('Ignoring command - not connected')
            return

        self._cmd_name = cmd.command.lower()
        c = self._cmd_name

        # Log every incoming command
        self._log_command(c, cmd)

        # Armed-state gate
        if not n._telemetry.armed and c not in self.UNARMED_ALLOWED:
            n.get_logger().warn(
                f'Rejecting "{c}" - vehicle not armed. Arm first.')
            n._publish_event('command_rejected',
                             f'"{c}" rejected: vehicle not armed')
            n._publish_feedback(c, 'rejected', detail='vehicle not armed')
            return

        # Parse speed / duration (available to all handlers via self._*)
        self._parse_speed_duration(cmd)

        # Try direct dispatch
        handler = self._dispatch.get(c)
        if handler:
            handler(cmd)
            return

        # Prefix-based dispatch
        if c.startswith('just_'):
            self._handle_just(c, cmd)
        elif c.startswith('go_'):
            self._handle_go(c, cmd)
        elif c.startswith('move_') and '_' in c[5:]:
            self._handle_compound_move(c, cmd)
        else:
            n.get_logger().warn(f'Unknown command: {c}')
            n._publish_feedback(c, 'rejected', detail='unknown command')

    # ── Shared helpers ───────────────────────────────────────────────

    def _log_command(self, c: str, cmd: DriverCommand):
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
        self._n.get_logger().info(f'RX command: {c}{param_str}')

    def _parse_speed_duration(self, cmd: DriverCommand):
        n = self._n
        raw_speed = cmd.speed if cmd.speed != 0 else 50
        if 0 < raw_speed <= 100:
            speed = percent_to_pwm(raw_speed)
        else:
            speed = NEUTRAL_PWM + max(-PWM_RANGE, min(PWM_RANGE, int(raw_speed)))

        duration = max(0, cmd.duration)
        end_time = time.time() + duration if duration > 0 else float('inf')
        offset = speed - NEUTRAL_PWM

        # MISSING 3: Battery voltage compensation
        if n._nominal_voltage > 0 and n._telemetry.voltage > 1.0:
            offset = int(offset * n._nominal_voltage / n._telemetry.voltage)

        self._speed = speed
        self._raw_speed = cmd.speed if cmd.speed != 0 else 50
        self._offset = offset
        self._duration = duration
        self._end_time = end_time

    def _set_movement(self, channels: dict, desc: str,
                      bypass_ramp: bool = False):
        """Set movement target channels."""
        n = self._n
        c = self._cmd_name
        with n._movement_lock:
            n._current_movement = {
                'channels': channels,
                'end_time': self._end_time,
                'bypass_ramp': bypass_ramp,
                'command': c,
            }
        n._publish_event('movement', desc)
        n._publish_feedback(c, 'accepted', detail=desc)

    def _resolve_depth_target(self, depth_value: float) -> float:
        """Resolve user depth to actual target (with surface offset)."""
        n = self._n
        if abs(depth_value) < 0.02:
            return n._telemetry.depth  # hold current depth
        # POOL FIX 3: surface_depth - abs(user_depth)
        return n._surface_depth - abs(depth_value)

    def _make_depth_pid(self, target: float) -> PidController:
        """Create a depth PidController with current params."""
        n = self._n
        return PidController(
            kp=n._depth_kp, ki=n._depth_ki, kd=n._depth_kd,
            output_limit=PWM_RANGE, max_integral=n._depth_max_integral,
            tolerance=n._depth_tolerance, ema_alpha=0.3,
            max_rate=n._pid_max_rate, anti_windup=True,
        )

    def _make_yaw_pid(self, gain_offset: int) -> PidController:
        """Create a yaw PidController with current params."""
        n = self._n
        return PidController(
            kp=n._yaw_kp, ki=n._yaw_ki, kd=n._yaw_kd,
            output_limit=min(PWM_RANGE, gain_offset),
            max_integral=n._yaw_max_integral,
            tolerance=0, ema_alpha=1.0,
            max_rate=n._pid_max_rate, anti_windup=False,
        )

    def _activate_depth_pid(self, target: float):
        """Activate software depth PID."""
        n = self._n
        n._alt_hold_target = None
        with n._movement_lock:
            n._depth_pid = self._make_depth_pid(target)
            n._depth_pid_target = target
        n._depth_pid_last_time = time.time()

    def _activate_yaw_pid(self, heading_deg: float,
                          gain_offset: int | None = None,
                          tolerance: float = 3.0):
        """Activate yaw PID heading control."""
        n = self._n
        if gain_offset is None:
            gain_offset = abs(self._speed - NEUTRAL_PWM) or PWM_RANGE // 2
        with n._movement_lock:
            n._yaw_pid = self._make_yaw_pid(gain_offset)
            n._yaw_target = heading_deg % 360
            n._yaw_tolerance = tolerance
            n._yaw_bang_offset = None
            n._yaw_command = self._cmd_name
        n._yaw_pid_last_time = time.time()

    def _activate_yaw_bang(self, heading_deg: float,
                           gain_offset: int | None = None,
                           tolerance: float = 5.0):
        """Activate bang-bang heading control."""
        n = self._n
        if gain_offset is None:
            gain_offset = abs(self._speed - NEUTRAL_PWM) or PWM_RANGE // 2
        with n._movement_lock:
            n._yaw_pid = None
            n._yaw_target = heading_deg % 360
            n._yaw_tolerance = tolerance
            n._yaw_bang_offset = min(PWM_RANGE, gain_offset)
            n._yaw_command = self._cmd_name

    # ── Movement commands ────────────────────────────────────────────

    def _cmd_move_forward(self, cmd):
        self._set_movement(
            {CH_FORWARD: NEUTRAL_PWM + self._offset,
             CH_LATERAL: NEUTRAL_PWM},
            f'Moving forward (speed={self._speed})')

    def _cmd_move_back(self, cmd):
        self._set_movement(
            {CH_FORWARD: NEUTRAL_PWM - self._offset,
             CH_LATERAL: NEUTRAL_PWM},
            'Moving backward')

    def _cmd_move_left(self, cmd):
        self._set_movement(
            {CH_FORWARD: NEUTRAL_PWM,
             CH_LATERAL: NEUTRAL_PWM - self._offset},
            'Moving left')

    def _cmd_move_right(self, cmd):
        self._set_movement(
            {CH_FORWARD: NEUTRAL_PWM,
             CH_LATERAL: NEUTRAL_PWM + self._offset},
            'Moving right')

    def _cmd_move_up(self, cmd):
        self._set_movement(
            {CH_THROTTLE: NEUTRAL_PWM + self._offset}, 'Moving up')

    def _cmd_move_down(self, cmd):
        self._set_movement(
            {CH_THROTTLE: NEUTRAL_PWM - self._offset}, 'Moving down')

    def _cmd_move_at(self, cmd):
        rad = math.radians(cmd.angle)
        fwd = NEUTRAL_PWM + int(self._offset * math.cos(rad))
        lat = NEUTRAL_PWM + int(self._offset * math.sin(rad))
        self._set_movement(
            {CH_FORWARD: fwd, CH_LATERAL: lat},
            f'Moving at {cmd.angle}° (fwd={fwd} lat={lat})')

    def _handle_compound_move(self, c: str, cmd):
        """Handle 2-axis diagonal: move_forward_right, etc."""
        parts = c[5:].split('_')  # strip 'move_'
        result = build_diagonal_channels(parts, self._speed)
        if result:
            channels, label = result
            channels = {k: v for k, v in channels.items()
                        if k in (CH_FORWARD, CH_LATERAL)}
            n = len(parts)
            scaled = int((self._speed - NEUTRAL_PWM) / math.sqrt(n))
            self._set_movement(
                channels,
                f'Moving {label} ({n}-axis, ±{abs(scaled)}pwm/axis, '
                f'speed={self._speed})')
        else:
            self._n.get_logger().warn(f'Invalid compound direction: {c}')

    # ── Yaw commands ─────────────────────────────────────────────────

    def _cmd_yaw_angle(self, cmd):
        n = self._n
        n._set_target_attitude(0, 0, cmd.angle)
        n._publish_event('movement', f'Setting heading to {cmd.angle}°')
        n._publish_feedback('yaw_angle', 'accepted',
                            detail=f'target={cmd.angle}° (attitude target)')

    def _cmd_yaw_to_heading(self, cmd):
        n = self._n
        # POOL FIX 1: Stop active movement so yaw doesn't fight inertia
        with n._movement_lock:
            n._current_movement = None
        self._activate_yaw_bang(cmd.angle)
        n._rc.snap_channels_neutral(CH_FORWARD, CH_LATERAL)
        n._publish_event('movement',
                         f'Yaw to heading {cmd.angle}° (bang-bang)')
        n._publish_feedback('yaw_to_heading', 'accepted',
                            detail=f'target={cmd.angle % 360}° bang-bang')

    def _cmd_pid_yaw_to_heading(self, cmd):
        n = self._n
        # POOL FIX 1: Stop active movement
        with n._movement_lock:
            n._current_movement = None
        self._activate_yaw_pid(cmd.angle)
        n._rc.snap_channels_neutral(CH_FORWARD, CH_LATERAL)
        n._publish_event(
            'movement',
            f'PID yaw to heading {cmd.angle}° '
            f'(Kp={n._yaw_kp} Ki={n._yaw_ki} Kd={n._yaw_kd})')
        n._publish_feedback('pid_yaw_to_heading', 'accepted',
                            detail=f'target={cmd.angle % 360}° PID')

    def _cmd_yaw_left(self, cmd):
        self._set_movement(
            {CH_YAW: NEUTRAL_PWM - self._offset}, 'Yaw left')

    def _cmd_yaw_right(self, cmd):
        self._set_movement(
            {CH_YAW: NEUTRAL_PWM + self._offset}, 'Yaw right')

    # ── Depth commands ───────────────────────────────────────────────

    def _cmd_depth(self, cmd):
        n = self._n
        d = self._resolve_depth_target(float(cmd.depth))
        if n._telemetry.flight_mode != 'ALT_HOLD':
            n.get_logger().info(
                'Auto-switching to ALT_HOLD for firmware depth hold')
            n._set_mode('ALT_HOLD')
        n._alt_hold_target = d
        n._set_target_depth(d)
        with n._movement_lock:
            n._depth_pid = None
            n._depth_pid_target = None
        n._publish_event(
            'movement',
            f'ALT_HOLD depth target: {abs(d):.2f}m (raw={d:.3f})')

    def _cmd_pid_depth(self, cmd):
        n = self._n
        target = self._resolve_depth_target(float(cmd.depth))
        self._activate_depth_pid(target)
        n._publish_event(
            'movement',
            f'PID depth hold ON: target {abs(target):.2f}m '
            f'(raw={target:.3f}) '
            f'(Kp={n._depth_kp} Ki={n._depth_ki} Kd={n._depth_kd})')
        n._publish_feedback(
            'pid_depth', 'accepted',
            detail=f'target={abs(target):.2f}m '
                   f'surface_offset={n._surface_depth:.3f}m')

    def _cmd_pid_depth_off(self, cmd):
        n = self._n
        with n._movement_lock:
            n._depth_pid = None
            n._depth_pid_target = None
        n._alt_hold_target = None
        n._publish_event('movement', 'PID depth hold OFF')
        n._publish_feedback('pid_depth_off', 'accepted',
                            detail='depth PID disabled')

    def _cmd_calibrate_depth(self, cmd):
        n = self._n
        n._surface_depth = n._telemetry.depth
        n.get_logger().info(
            f'Surface depth calibrated: {n._surface_depth:.3f}m')
        n._publish_event(
            'calibration',
            f'Surface depth set to {n._surface_depth:.3f}m')
        n._publish_feedback(
            'calibrate_depth', 'accepted',
            detail=f'surface_depth={n._surface_depth:.3f}m')

    def _cmd_surface(self, cmd):
        n = self._n
        n._clear_all_control()
        if n._telemetry.flight_mode == 'ALT_HOLD':
            n._alt_hold_target = -0.1
            n._set_target_depth(-0.1)
            n._publish_event('movement', 'Surfacing (ALT_HOLD to -0.1m)')
        else:
            with n._movement_lock:
                n._current_movement = {
                    'channels': {CH_THROTTLE: NEUTRAL_PWM + PWM_RANGE // 2},
                    'end_time': time.time() + 10.0,
                    'command': 'surface',
                }
            n._publish_event('movement',
                             'Surfacing (MANUAL, throttle up 10s)')
        n._publish_feedback('surface', 'accepted', detail='surfacing')

    # ── System commands ──────────────────────────────────────────────

    def _cmd_arm(self, cmd):
        n = self._n
        # POOL FIX 3: Auto-calibrate surface depth on first arm
        if n._surface_depth == 0.0 and abs(n._telemetry.depth) > 0.01:
            n._surface_depth = n._telemetry.depth
            n.get_logger().info(
                f'Auto-calibrated surface depth: {n._surface_depth:.3f}m')
        n._arm_disarm(True)
        n._publish_feedback(
            'arm', 'accepted',
            detail=f'arm requested '
                   f'(surface_depth={n._surface_depth:.3f}m)')

    def _cmd_disarm(self, cmd):
        n = self._n
        # POOL FIX 4: Clear all state before disarming
        n._stop_all()
        n._arm_disarm(False)
        n._publish_feedback('disarm', 'accepted', detail='disarm requested')

    def _cmd_set_mode(self, cmd):
        n = self._n
        n._set_mode(cmd.mode)
        n._publish_feedback('set_mode', 'accepted', detail=f'mode={cmd.mode}')

    def _cmd_open_grabber(self, cmd):
        n = self._n
        n._set_servo_pwm(1, 1100)
        n._publish_event('actuator', 'Grabber open')
        n._publish_feedback('open_grabber', 'accepted', detail='grabber open')

    def _cmd_close_grabber(self, cmd):
        n = self._n
        n._set_servo_pwm(1, 1900)
        n._publish_event('actuator', 'Grabber close')
        n._publish_feedback('close_grabber', 'accepted',
                            detail='grabber close')

    # ── Cruise (movement + depth PID + yaw PID) ─────────────────────

    def _cruise_common(self, cmd, bypass_ramp: bool):
        n = self._n
        prefix = '[JUST] ' if bypass_ramp else ''
        bearing_deg = cmd.angle
        rad = math.radians(bearing_deg)
        fwd = NEUTRAL_PWM + int(self._offset * math.cos(rad))
        lat = NEUTRAL_PWM + int(self._offset * math.sin(rad))

        self._set_movement(
            {CH_FORWARD: fwd, CH_LATERAL: lat},
            f'{prefix}Cruise bearing={bearing_deg}° heading={cmd.mode}° '
            f'depth={cmd.depth}m speed={self._raw_speed}%',
            bypass_ramp=bypass_ramp)

        # Depth PID
        target_depth = self._resolve_depth_target(float(cmd.depth))
        self._activate_depth_pid(target_depth)

        # Yaw PID
        try:
            heading_deg = float(cmd.mode) % 360
        except (ValueError, TypeError):
            heading_deg = n._telemetry.yaw % 360
        gain_offset = abs(self._speed - NEUTRAL_PWM) or PWM_RANGE // 2
        self._activate_yaw_pid(heading_deg, gain_offset)

        n._publish_event(
            'movement',
            f'{prefix}Cruise: bearing={bearing_deg}° heading={heading_deg}° '
            f'depth={abs(target_depth):.2f}m '
            f'speed={self._raw_speed}% dur={self._duration}s')

    def _cmd_cruise(self, cmd):
        self._cruise_common(cmd, bypass_ramp=False)

    def _cmd_just_cruise(self, cmd):
        self._cruise_common(cmd, bypass_ramp=True)

    # ── Teleop ───────────────────────────────────────────────────────

    def _cmd_teleop(self, cmd):
        clamp = lambda v: max(-PWM_RANGE, min(PWM_RANGE, int(v)))
        fwd = clamp(cmd.speed)
        lat = clamp(cmd.duration)
        thr = clamp(cmd.depth)
        yaw = clamp(cmd.angle)
        self._set_movement(
            {CH_FORWARD: NEUTRAL_PWM + fwd,
             CH_LATERAL: NEUTRAL_PWM + lat,
             CH_THROTTLE: NEUTRAL_PWM + thr,
             CH_YAW: NEUTRAL_PWM + yaw},
            f'Teleop (fwd={fwd} lat={lat} thr={thr} yaw={yaw})')

    def _cmd_teleop_idle(self, cmd):
        with self._n._movement_lock:
            self._n._current_movement = None
        # No event — teleop_idle fires every tick when joystick centred

    def _cmd_stop(self, cmd):
        n = self._n
        n._stop_all()
        n._publish_feedback('stop', 'accepted',
                            detail='All thrusters neutral')

    # ── just_* prefix handler ────────────────────────────────────────

    def _handle_just(self, c: str, cmd):
        """Handle ``just_*`` commands (bypass ramp)."""
        n = self._n
        raw = c[5:]  # strip 'just_'

        if raw in ('move_forward', 'forward'):
            self._set_movement(
                {CH_FORWARD: NEUTRAL_PWM + self._offset,
                 CH_LATERAL: NEUTRAL_PWM},
                f'[JUST] Moving forward (speed={self._speed})',
                bypass_ramp=True)

        elif raw in ('move_back', 'back', 'backward'):
            self._set_movement(
                {CH_FORWARD: NEUTRAL_PWM - self._offset,
                 CH_LATERAL: NEUTRAL_PWM},
                '[JUST] Moving backward', bypass_ramp=True)

        elif raw in ('move_left', 'left'):
            self._set_movement(
                {CH_FORWARD: NEUTRAL_PWM,
                 CH_LATERAL: NEUTRAL_PWM - self._offset},
                '[JUST] Moving left', bypass_ramp=True)

        elif raw in ('move_right', 'right'):
            self._set_movement(
                {CH_FORWARD: NEUTRAL_PWM,
                 CH_LATERAL: NEUTRAL_PWM + self._offset},
                '[JUST] Moving right', bypass_ramp=True)

        elif raw in ('move_up', 'up'):
            self._set_movement(
                {CH_THROTTLE: NEUTRAL_PWM + self._offset},
                '[JUST] Moving up', bypass_ramp=True)

        elif raw in ('move_down', 'down'):
            self._set_movement(
                {CH_THROTTLE: NEUTRAL_PWM - self._offset},
                '[JUST] Moving down', bypass_ramp=True)

        elif raw == 'yaw_left':
            self._set_movement(
                {CH_YAW: NEUTRAL_PWM - self._offset},
                '[JUST] Yaw left', bypass_ramp=True)

        elif raw == 'yaw_right':
            self._set_movement(
                {CH_YAW: NEUTRAL_PWM + self._offset},
                '[JUST] Yaw right', bypass_ramp=True)

        elif raw == 'move_at':
            rad = math.radians(cmd.angle)
            fwd = NEUTRAL_PWM + int(self._offset * math.cos(rad))
            lat = NEUTRAL_PWM + int(self._offset * math.sin(rad))
            self._set_movement(
                {CH_FORWARD: fwd, CH_LATERAL: lat},
                f'[JUST] Moving at {cmd.angle}° (fwd={fwd} lat={lat})',
                bypass_ramp=True)

        elif raw == 'teleop':
            clamp = lambda v: max(-PWM_RANGE, min(PWM_RANGE, int(v)))
            fwd = clamp(cmd.speed)
            lat = clamp(cmd.duration)
            thr = clamp(cmd.depth)
            yaw = clamp(cmd.angle)
            self._set_movement(
                {CH_FORWARD: NEUTRAL_PWM + fwd,
                 CH_LATERAL: NEUTRAL_PWM + lat,
                 CH_THROTTLE: NEUTRAL_PWM + thr,
                 CH_YAW: NEUTRAL_PWM + yaw},
                f'[JUST] Teleop (fwd={fwd} lat={lat} thr={thr} yaw={yaw})',
                bypass_ramp=True)

        elif raw == 'surface':
            n._clear_all_control()
            if n._telemetry.flight_mode == 'ALT_HOLD':
                n._alt_hold_target = -0.1
                n._set_target_depth(-0.1)
                n._publish_event('movement',
                                 '[JUST] Surfacing (ALT_HOLD to -0.1m)')
            else:
                with n._movement_lock:
                    n._current_movement = {
                        'channels': {
                            CH_THROTTLE: NEUTRAL_PWM + PWM_RANGE // 2,
                        },
                        'end_time': time.time() + 10.0,
                        'bypass_ramp': True,
                        'command': c,
                    }
                n._publish_event(
                    'movement',
                    '[JUST] Surfacing (MANUAL, instant throttle up 10s)')
            n._publish_feedback(c, 'accepted', detail='surfacing (instant)')

        # just_move_forward_right, etc.
        elif raw.startswith('move_') and '_' in raw[5:]:
            parts = raw[5:].split('_')
            result = build_diagonal_channels(parts, self._speed)
            if result:
                ch, label = result
                ch = {k: v for k, v in ch.items()
                      if k in (CH_FORWARD, CH_LATERAL)}
                nax = len(parts)
                scaled = int((self._speed - NEUTRAL_PWM) / math.sqrt(nax))
                self._set_movement(
                    ch,
                    f'[JUST] Moving {label} ({nax}-axis, '
                    f'±{abs(scaled)}pwm/axis, speed={self._speed})',
                    bypass_ramp=True)
            else:
                n.get_logger().warn(
                    f'Invalid just compound direction: {c}')

        # just_go_forward, just_go_forward_right, etc.
        elif raw.startswith('go_'):
            self._handle_go(c, cmd, bypass_ramp=True)

        else:
            n.get_logger().warn(f'Unknown just_ command: {c}')

    # ── go_* prefix handler (movement + yaw PID) ────────────────────

    def _handle_go(self, c: str, cmd, bypass_ramp: bool = False):
        """Handle go_forward, go_forward_right, etc.

        Sets movement AND yaw PID simultaneously so the AUV moves in a
        direction while rotating to a target heading.
        """
        n = self._n
        prefix = '[JUST] ' if bypass_ramp else ''

        # Strip prefix to get direction
        if c.startswith('just_go_'):
            dir_str = c[8:]
        elif c.startswith('go_'):
            dir_str = c[3:]
        else:
            n.get_logger().warn(f'Invalid go command: {c}')
            return

        parts = dir_str.split('_')
        HORIZONTAL = {'forward', 'back', 'backward', 'left', 'right'}
        if not all(p in HORIZONTAL for p in parts):
            n.get_logger().warn(
                f'go commands only support horizontal directions, '
                f'got: {dir_str}')
            return

        result = build_diagonal_channels(parts, self._speed)
        if result is None:
            n.get_logger().warn(f'Invalid go direction: {c}')
            return

        mv_channels, label = result
        mv_channels = {k: v for k, v in mv_channels.items()
                       if k in (CH_FORWARD, CH_LATERAL)}

        with n._movement_lock:
            n._current_movement = {
                'channels': mv_channels,
                'end_time': self._end_time,
                'bypass_ramp': bypass_ramp,
                'command': c,
            }

        # Activate yaw PID to target heading
        self._activate_yaw_pid(cmd.angle)

        dur_str = f' {self._duration}s' if self._duration > 0 else ''
        n._publish_event(
            'movement',
            f'{prefix}Go {label} → {cmd.angle:.0f}°{dur_str} '
            f'(speed={self._speed})')
        n._publish_feedback(
            c, 'accepted',
            detail=f'Go {label} → {cmd.angle:.0f}°')
