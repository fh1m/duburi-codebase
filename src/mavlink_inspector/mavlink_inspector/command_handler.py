"""Command handler for DriverCommand dispatch.

System commands (arm, disarm, mode, depth, stop) live here.
Movement commands live in ``movement_commands.py`` — edit that file
to add custom maneuvers.

Every movement registered via @register decorator automatically gets a
``just_*`` variant that bypasses the velocity ramp.
"""

from __future__ import annotations

import math
import time

from duburi_interfaces.msg import DriverCommand, TeleopCommand

from duburi_common.command_registry import get_command, CommandCategory

from .pid_controller import PidController
from .rc_controller import (
    CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW,
    NEUTRAL_PWM, PWM_RANGE,
    percent_to_pwm,
)
from .movement_commands import MOVEMENTS, handle_go, handle_compound_move


class CommandHandler:
    """Dispatches DriverCommand messages to handler methods."""

    from duburi_common.constants import UNARMED_ALLOWED_INSPECTOR as UNARMED_ALLOWED

    # Commands handled by other nodes (vision alignment controller).
    # Inspector logs them but does not dispatch or reject.
    _PASSTHROUGH_COMMANDS = frozenset({
        'lat_align', 'dep_align', 'align', 'align_forward',
        'pid_lat_align', 'pid_dep_align', 'pid_align', 'pid_align_forward',
        'just_lat_align', 'just_dep_align', 'just_align', 'just_align_forward',
        'vision_stop',
    })

    def __init__(self, node):
        """
        Args:
            node: ``MavlinkInspectorNode`` reference — provides state,
                  publishers, and control methods.
        """
        self._n = node
        self._just_mode = False

        # Per-command state (set by _parse_speed_duration)
        self._offset = 0
        self._speed = NEUTRAL_PWM
        self._raw_speed = 50
        self._duration = 0.0
        self._end_time = float('inf')
        self._cmd_name = ''

        # System commands — always live here
        self._system_dispatch: dict[str, callable] = {
            'stop':                self._cmd_stop,
            'depth':               self._cmd_depth,
            'set_depth':           self._cmd_depth,
            'pid_depth':           self._cmd_pid_depth,
            'pid_depth_off':       self._cmd_pid_depth_off,
            'calibrate_depth':     self._cmd_calibrate_depth,
            'arm':                 self._cmd_arm,
            'disarm':              self._cmd_disarm,
            'set_mode':            self._cmd_set_mode,
            'open_grabber':        self._cmd_open_grabber,
            'close_grabber':       self._cmd_close_grabber,
            'teleop_idle':         self._cmd_teleop_idle,
        }

    # ── Properties for movement functions ────────────────────────────

    @property
    def offset(self) -> int:
        """PWM offset from neutral (positive, voltage-compensated)."""
        return self._offset

    @property
    def speed(self) -> int:
        """Absolute PWM value (1500 + offset)."""
        return self._speed

    @property
    def raw_speed(self) -> int:
        """Original speed value (0–100 percent)."""
        return self._raw_speed

    @property
    def duration(self) -> float:
        """Duration in seconds (0 = infinite)."""
        return self._duration

    @property
    def end_time(self) -> float:
        """Expiry timestamp (time.time() + duration)."""
        return self._end_time

    @property
    def cmd_name(self) -> str:
        """Current command string."""
        return self._cmd_name

    @property
    def node(self):
        """MavlinkInspectorNode reference."""
        return self._n

    # ── Public entry point ───────────────────────────────────────────

    def handle(self, cmd: DriverCommand):
        """Dispatch a driver command."""
        n = self._n

        if not n._conn.connected or n._conn.master is None:
            n.get_logger().warn('Ignoring command - not connected')
            return

        self._cmd_name = cmd.command.lower()
        c = self._cmd_name

        # Check if bypass_ramp is set (new field) or just_* prefix
        if getattr(cmd, 'bypass_ramp', False) or c.startswith('just_'):
            self._just_mode = True
            if c.startswith('just_'):
                # Check if it's an explicit just_* command in registry first
                spec = get_command(c)
                if spec is None or spec.handler is None:
                    # Not explicitly registered, delegate to auto handler
                    pass
                # else: fall through to normal dispatch

        # Log every incoming command
        self._log_command(c, cmd)

        # Vision commands are handled by the alignment_controller node,
        # not by the inspector.  Log and pass through silently.
        if c in self._PASSTHROUGH_COMMANDS:
            n.get_logger().info(f'Passthrough to vision: {c}')
            n._publish_feedback(c, 'accepted', detail='routed to vision alignment controller')
            return

        # Armed-state gate (check registry for requires_armed flag)
        spec = get_command(c)
        requires_armed = True
        if spec is not None:
            requires_armed = spec.requires_armed
        if not n._telemetry.armed and c not in self.UNARMED_ALLOWED and requires_armed:
            n.get_logger().warn(
                f'Rejecting "{c}" - vehicle not armed. Arm first.')
            n._publish_event('command_rejected',
                             f'"{c}" rejected: vehicle not armed')
            n._publish_feedback(c, 'rejected', detail='vehicle not armed')
            return

        # Parse speed / duration (available to all handlers via self._*)
        self._parse_speed_duration(cmd)

        # 1. System commands (always in this file)
        handler = self._system_dispatch.get(c)
        if handler:
            handler(cmd)
            return

        # 2. Try registry lookup first (preferred path)
        spec = get_command(c)
        if spec is not None and spec.handler is not None:
            spec.handler(self, cmd)
            return

        # 3. Fallback to MOVEMENTS dict (backward compatibility)
        handler = MOVEMENTS.get(c)
        if handler:
            handler(self, cmd)
            return

        # 4. just_* auto-delegation
        if c.startswith('just_'):
            self._handle_just_auto(c, cmd)
            return

        # 5. go_* prefix (movement + yaw PID)
        if c.startswith('go_'):
            handle_go(self, c, cmd)
            return

        # 6. Compound diagonal: move_forward_right, etc.
        if c.startswith('move_') and '_' in c[5:]:
            handle_compound_move(self, c, cmd)
            return

        # Unknown
        n.get_logger().warn(f'Unknown command: {c}')
        n._publish_feedback(c, 'rejected', detail='unknown command')

    # ── just_* auto-delegation ───────────────────────────────────────

    def _handle_just_auto(self, c: str, cmd):
        """Handle ``just_*`` — auto-delegates with bypass ramp.

        Strips ``just_`` prefix and calls the base handler with
        ``_just_mode=True``, which causes ``set_movement()`` to force
        ``bypass_ramp=True`` and prepend ``[JUST]`` to descriptions.
        """
        raw = c[5:]  # strip 'just_'

        self._just_mode = True
        try:
            # Try registry lookup first
            spec = get_command(raw)
            if spec is not None and spec.handler is not None:
                spec.handler(self, cmd)
                return

            # Fallback to MOVEMENTS dict (backward compatibility)
            handler = MOVEMENTS.get(raw)
            if handler:
                handler(self, cmd)
                return

            # go_* prefix?
            if raw.startswith('go_'):
                handle_go(self, c, cmd)
                return

            # Compound diagonal?
            if raw.startswith('move_') and '_' in raw[5:]:
                handle_compound_move(self, raw, cmd)
                return

            self._n.get_logger().warn(f'Unknown just_ command: {c}')
        finally:
            self._just_mode = False

    # ── Public API for movement functions ────────────────────────────

    def set_movement(self, channels: dict, desc: str,
                     bypass_ramp: bool = False,
                     end_time: float | None = None):
        """Set movement target channels with event + feedback.

        When called during a ``just_*`` command, ``bypass_ramp`` is
        forced True and ``[JUST]`` is prepended to the description.
        """
        n = self._n
        if self._just_mode:
            bypass_ramp = True
            if not desc.startswith('[JUST]'):
                desc = f'[JUST] {desc}'
        with n._movement_lock:
            n._current_movement = {
                'channels': channels,
                'end_time': end_time if end_time is not None else self._end_time,
                'bypass_ramp': bypass_ramp,
                'command': self._cmd_name,
            }
        n._publish_event('movement', desc)
        n._publish_feedback(self._cmd_name, 'accepted', detail=desc)

    def resolve_depth(self, depth_value: float) -> float:
        """Resolve user depth to actual target (with surface offset)."""
        n = self._n
        if abs(depth_value) < 0.02:
            return n._telemetry.depth  # hold current depth
        # POOL FIX 3: surface_depth - abs(user_depth)
        return n._surface_depth - abs(depth_value)

    def activate_depth_pid(self, target: float):
        """Activate software depth PID."""
        n = self._n
        n._alt_hold_target = None
        with n._movement_lock:
            n._depth_pid = self._make_depth_pid()
            n._depth_pid_target = target
        n._depth_pid_last_time = time.time()

    def activate_yaw_pid(self, heading_deg: float,
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

    def activate_yaw_bang(self, heading_deg: float,
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

    # ── Private helpers ──────────────────────────────────────────────

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
            speed = NEUTRAL_PWM + max(-PWM_RANGE,
                                      min(PWM_RANGE, int(raw_speed)))

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

    def _make_depth_pid(self) -> PidController:
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
            tolerance=0, ema_alpha=0.3,  # C3 fix: enable derivative filtering
            max_rate=n._pid_max_rate, anti_windup=True,  # C3 fix: enable anti-windup
        )

    # ── System commands ──────────────────────────────────────────────

    def _cmd_stop(self, cmd):
        n = self._n
        n._stop_all()
        n._publish_feedback('stop', 'accepted',
                            detail='All thrusters neutral')

    def _cmd_teleop_idle(self, cmd):
        with self._n._movement_lock:
            self._n._current_movement = None
        # No event — teleop_idle fires every tick when joystick centred

    # ── Depth commands ───────────────────────────────────────────────

    def _cmd_depth(self, cmd):
        n = self._n
        d = self.resolve_depth(float(cmd.depth))
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
        target = self.resolve_depth(float(cmd.depth))
        self.activate_depth_pid(target)
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

    # ── Arm / Disarm / Mode ──────────────────────────────────────────

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
        n._publish_feedback('set_mode', 'accepted',
                            detail=f'mode={cmd.mode}')

    # ── Actuators ────────────────────────────────────────────────────

    def _cmd_open_grabber(self, cmd):
        n = self._n
        n._set_servo_pwm(1, 1100)
        n._publish_event('actuator', 'Grabber open')
        n._publish_feedback('open_grabber', 'accepted',
                            detail='grabber open')

    def _cmd_close_grabber(self, cmd):
        n = self._n
        n._set_servo_pwm(1, 1900)
        n._publish_event('actuator', 'Grabber close')
        n._publish_feedback('close_grabber', 'accepted',
                            detail='grabber close')

    # ── Teleop (dedicated TeleopCommand message) ─────────────────────

    def handle_teleop(self, msg: TeleopCommand):
        """Handle TeleopCommand from /driver/teleop topic."""
        n = self._n
        if not n._conn.connected or n._conn.master is None:
            return

        if msg.idle:
            with n._movement_lock:
                n._current_movement = None
            return

        max_offset = min(PWM_RANGE, abs(msg.speed) if msg.speed != 0 else 200)
        clamp = lambda v: max(-max_offset, min(max_offset, int(v * max_offset)))
        fwd = clamp(msg.linear_x)
        lat = clamp(msg.linear_y)
        thr = clamp(msg.linear_z)
        yaw = clamp(msg.angular_z)

        with n._movement_lock:
            n._current_movement = {
                'channels': {
                    CH_FORWARD: NEUTRAL_PWM + fwd,
                    CH_LATERAL: NEUTRAL_PWM + lat,
                    CH_THROTTLE: NEUTRAL_PWM + thr,
                    CH_YAW: NEUTRAL_PWM + yaw,
                },
                'end_time': float('inf'),
                'bypass_ramp': True,
                'command': 'teleop',
            }
