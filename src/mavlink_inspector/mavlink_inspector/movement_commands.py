"""Movement command definitions for BRACU Duburi 4.2.

This file contains all movement, yaw, cruise, and teleop handlers.
Each movement gets an automatic ``just_*`` variant (bypasses ramp).

Adding a New Movement
=====================

1. Define a handler function::

       def cmd_my_maneuver(h, cmd):
           h.set_movement(
               {CH_FORWARD: NEUTRAL_PWM + h.offset,
                CH_YAW: NEUTRAL_PWM - h.offset // 3},
               f'My maneuver (speed={h.speed})')

2. Register it in the ``MOVEMENTS`` dict at the bottom::

       MOVEMENTS['my_maneuver'] = cmd_my_maneuver

3. (Optional) Add an alias::

       MOVEMENTS['maneuver'] = cmd_my_maneuver

4. ``just_my_maneuver`` works automatically (bypasses ramp).

Available Context (``h``)
=========================

Properties (read-only, set per-command from DriverCommand fields)::

    h.offset       PWM offset from neutral (positive, voltage-compensated)
    h.speed        Absolute PWM value (1500 + offset)
    h.raw_speed    Original speed value (0-100 percent)
    h.duration     Duration in seconds (0 = infinite)
    h.end_time     Expiry timestamp (time.time() + duration)
    h.cmd_name     Current command string (e.g. 'move_forward')
    h.node         MavlinkInspectorNode reference

Methods::

    h.set_movement(channels, desc, bypass_ramp=False, end_time=None)
    h.activate_depth_pid(target_m)
    h.activate_yaw_pid(heading_deg, gain_offset=None, tolerance=3.0)
    h.activate_yaw_bang(heading_deg, gain_offset=None, tolerance=5.0)
    h.resolve_depth(depth_value) → float

Channel constants::

    CH_PITCH=1  CH_ROLL=2  CH_THROTTLE=3  CH_YAW=4
    CH_FORWARD=5  CH_LATERAL=6
    NEUTRAL_PWM=1500  PWM_RANGE=400
"""

from __future__ import annotations

import math
import time

from .rc_controller import (
    CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW,
    NEUTRAL_PWM, PWM_RANGE,
    build_diagonal_channels,
)


# ── Basic directional movements ─────────────────────────────────────

def cmd_move_forward(h, cmd):
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM + h.offset, CH_LATERAL: NEUTRAL_PWM},
        f'Moving forward (speed={h.speed})')


def cmd_move_back(h, cmd):
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM - h.offset, CH_LATERAL: NEUTRAL_PWM},
        'Moving backward')


def cmd_move_left(h, cmd):
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM - h.offset},
        'Moving left')


def cmd_move_right(h, cmd):
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM + h.offset},
        'Moving right')


def cmd_move_up(h, cmd):
    h.set_movement({CH_THROTTLE: NEUTRAL_PWM + h.offset}, 'Moving up')


def cmd_move_down(h, cmd):
    h.set_movement({CH_THROTTLE: NEUTRAL_PWM - h.offset}, 'Moving down')


def cmd_move_at(h, cmd):
    """Move at an arbitrary angle (body-frame vector control)."""
    rad = math.radians(cmd.angle)
    fwd = NEUTRAL_PWM + int(h.offset * math.cos(rad))
    lat = NEUTRAL_PWM + int(h.offset * math.sin(rad))
    h.set_movement(
        {CH_FORWARD: fwd, CH_LATERAL: lat},
        f'Moving at {cmd.angle}° (fwd={fwd} lat={lat})')


# ── Compound diagonal movements ─────────────────────────────────────

def handle_compound_move(h, c: str, cmd):
    """Handle 2-axis diagonal: move_forward_right, etc.

    Each axis gets speed / √2 so the resultant vector magnitude
    equals the requested speed.
    """
    parts = c[5:].split('_')  # strip 'move_'
    result = build_diagonal_channels(parts, h.speed)
    if result:
        channels, label = result
        channels = {k: v for k, v in channels.items()
                    if k in (CH_FORWARD, CH_LATERAL)}
        n = len(parts)
        scaled = int((h.speed - NEUTRAL_PWM) / math.sqrt(n))
        h.set_movement(
            channels,
            f'Moving {label} ({n}-axis, ±{abs(scaled)}pwm/axis, '
            f'speed={h.speed})')
    else:
        h.node.get_logger().warn(f'Invalid compound direction: {c}')


# ── Yaw commands ─────────────────────────────────────────────────────

def cmd_yaw_angle(h, cmd):
    """Set heading via SET_ATTITUDE_TARGET (firmware-level)."""
    n = h.node
    n._set_target_attitude(0, 0, cmd.angle)
    n._publish_event('movement', f'Setting heading to {cmd.angle}°')
    n._publish_feedback('yaw_angle', 'accepted',
                        detail=f'target={cmd.angle}° (attitude target)')


def cmd_yaw_to_heading(h, cmd):
    """Bang-bang yaw to a target heading."""
    n = h.node
    # POOL FIX 1: Stop active movement so yaw doesn't fight inertia
    with n._movement_lock:
        n._current_movement = None
    h.activate_yaw_bang(cmd.angle)
    n._rc.snap_channels_neutral(CH_FORWARD, CH_LATERAL)
    n._publish_event('movement',
                     f'Yaw to heading {cmd.angle}° (bang-bang)')
    n._publish_feedback('yaw_to_heading', 'accepted',
                        detail=f'target={cmd.angle % 360}° bang-bang')


def cmd_pid_yaw_to_heading(h, cmd):
    """PID-controlled yaw to a target heading."""
    n = h.node
    # POOL FIX 1: Stop active movement
    with n._movement_lock:
        n._current_movement = None
    h.activate_yaw_pid(cmd.angle)
    n._rc.snap_channels_neutral(CH_FORWARD, CH_LATERAL)
    n._publish_event(
        'movement',
        f'PID yaw to heading {cmd.angle}° '
        f'(Kp={n._yaw_kp} Ki={n._yaw_ki} Kd={n._yaw_kd})')
    n._publish_feedback('pid_yaw_to_heading', 'accepted',
                        detail=f'target={cmd.angle % 360}° PID')


def cmd_yaw_left(h, cmd):
    h.set_movement({CH_YAW: NEUTRAL_PWM - h.offset}, 'Yaw left')


def cmd_yaw_right(h, cmd):
    h.set_movement({CH_YAW: NEUTRAL_PWM + h.offset}, 'Yaw right')


# ── Surface ──────────────────────────────────────────────────────────

def cmd_surface(h, cmd):
    """Surface: ALT_HOLD → firmware target −0.1 m, MANUAL → throttle up."""
    n = h.node
    prefix = '[JUST] ' if getattr(h, '_just_mode', False) else ''
    n._clear_all_control()
    if n._telemetry.flight_mode == 'ALT_HOLD':
        n._alt_hold_target = -0.1
        n._set_target_depth(-0.1)
        n._publish_event('movement',
                         f'{prefix}Surfacing (ALT_HOLD to -0.1m)')
    else:
        dur = n._surface_throttle_duration
        h.set_movement(
            {CH_THROTTLE: NEUTRAL_PWM + PWM_RANGE // 2},
            f'Surfacing (MANUAL, throttle up {dur:.0f}s)',
            end_time=time.time() + dur)
    n._publish_feedback(h.cmd_name, 'accepted',
                        detail=f'{prefix}surfacing'.strip())


# ── Cruise (movement + depth PID + yaw PID) ─────────────────────────

def cmd_cruise(h, cmd):
    _cruise_common(h, cmd, bypass_ramp=False)


def cmd_just_cruise(h, cmd):
    _cruise_common(h, cmd, bypass_ramp=True)


def _cruise_common(h, cmd, bypass_ramp: bool):
    n = h.node
    prefix = '[JUST] ' if bypass_ramp else ''
    bearing_deg = cmd.angle
    rad = math.radians(bearing_deg)
    fwd = NEUTRAL_PWM + int(h.offset * math.cos(rad))
    lat = NEUTRAL_PWM + int(h.offset * math.sin(rad))

    h.set_movement(
        {CH_FORWARD: fwd, CH_LATERAL: lat},
        f'{prefix}Cruise bearing={bearing_deg}° heading={cmd.mode}° '
        f'depth={cmd.depth}m speed={h.raw_speed}%',
        bypass_ramp=bypass_ramp)

    # Depth PID
    target_depth = h.resolve_depth(float(cmd.depth))
    h.activate_depth_pid(target_depth)

    # Yaw PID
    try:
        heading_deg = float(cmd.mode) % 360
    except (ValueError, TypeError):
        heading_deg = n._telemetry.yaw % 360
    gain_offset = abs(h.speed - NEUTRAL_PWM) or PWM_RANGE // 2
    h.activate_yaw_pid(heading_deg, gain_offset)

    n._publish_event(
        'movement',
        f'{prefix}Cruise: bearing={bearing_deg}° heading={heading_deg}° '
        f'depth={abs(target_depth):.2f}m '
        f'speed={h.raw_speed}% dur={h.duration}s')


# ── Teleop ───────────────────────────────────────────────────────────

def cmd_teleop(h, cmd):
    """Direct PWM teleop — fields repurposed as axis offsets."""
    clamp = lambda v: max(-PWM_RANGE, min(PWM_RANGE, int(v)))
    fwd = clamp(cmd.speed)
    lat = clamp(cmd.duration)
    thr = clamp(cmd.depth)
    yaw = clamp(cmd.angle)
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM + fwd,
         CH_LATERAL: NEUTRAL_PWM + lat,
         CH_THROTTLE: NEUTRAL_PWM + thr,
         CH_YAW: NEUTRAL_PWM + yaw},
        f'Teleop (fwd={fwd} lat={lat} thr={thr} yaw={yaw})')


# ── go_* prefix handler (movement + yaw PID) ────────────────────────

def handle_go(h, c: str, cmd):
    """Handle go_forward, go_forward_right, etc.

    Sets movement AND yaw PID simultaneously so the AUV moves in a
    direction while rotating to a target heading.
    """
    n = h.node
    bypass_ramp = getattr(h, '_just_mode', False)
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

    result = build_diagonal_channels(parts, h.speed)
    if result is None:
        n.get_logger().warn(f'Invalid go direction: {c}')
        return

    mv_channels, label = result
    mv_channels = {k: v for k, v in mv_channels.items()
                   if k in (CH_FORWARD, CH_LATERAL)}

    with n._movement_lock:
        n._current_movement = {
            'channels': mv_channels,
            'end_time': h.end_time,
            'bypass_ramp': bypass_ramp,
            'command': c,
        }

    # Activate yaw PID to target heading
    h.activate_yaw_pid(cmd.angle)

    dur_str = f' {h.duration}s' if h.duration > 0 else ''
    n._publish_event(
        'movement',
        f'{prefix}Go {label} → {cmd.angle:.0f}°{dur_str} '
        f'(speed={h.speed})')
    n._publish_feedback(
        c, 'accepted',
        detail=f'Go {label} → {cmd.angle:.0f}°')


# ═════════════════════════════════════════════════════════════════════
#  MOVEMENT REGISTRY
# ═════════════════════════════════════════════════════════════════════
# Every entry automatically gets a ``just_*`` variant for free.
# To add a custom movement, define a function above and register it
# here.  Aliases are just additional dict entries to the same handler.
#
# Prefix-based commands (go_*, move_*_*) are matched automatically
# and do NOT need registry entries.

MOVEMENTS: dict[str, callable] = {
    # ── Directional ──────────────────────────────────────────────
    'move_forward':       cmd_move_forward,
    'forward':            cmd_move_forward,
    'move_back':          cmd_move_back,
    'back':               cmd_move_back,
    'backward':           cmd_move_back,
    'move_left':          cmd_move_left,
    'left':               cmd_move_left,
    'move_right':         cmd_move_right,
    'right':              cmd_move_right,
    'move_up':            cmd_move_up,
    'up':                 cmd_move_up,
    'move_down':          cmd_move_down,
    'down':               cmd_move_down,
    'move_at':            cmd_move_at,

    # ── Yaw ──────────────────────────────────────────────────────
    'yaw_angle':          cmd_yaw_angle,
    'yaw_to_heading':     cmd_yaw_to_heading,
    'pid_yaw_to_heading': cmd_pid_yaw_to_heading,
    'yaw_left':           cmd_yaw_left,
    'yaw_right':          cmd_yaw_right,

    # ── Surface ──────────────────────────────────────────────────
    'surface':            cmd_surface,

    # ── Cruise (movement + depth PID + yaw PID) ──────────────────
    'cruise':             cmd_cruise,
    'just_cruise':        cmd_just_cruise,

    # ── Teleop (legacy DriverCommand fallback — prefer /driver/teleop topic)
    'teleop':             cmd_teleop,
}


# ═════════════════════════════════════════════════════════════════════
#  CUSTOM MOVEMENTS — Add your test maneuvers below
# ═════════════════════════════════════════════════════════════════════
#
# Example: Spiral descent (forward + yaw + depth PID)
#
# def cmd_spiral_descent(h, cmd):
#     """Forward + slow yaw left while descending to target depth."""
#     h.set_movement(
#         {CH_FORWARD: NEUTRAL_PWM + h.offset,
#          CH_YAW: NEUTRAL_PWM - h.offset // 3},
#         f'Spiral descent (speed={h.speed})')
#     h.activate_depth_pid(h.resolve_depth(cmd.depth))
#
# MOVEMENTS['spiral_descent'] = cmd_spiral_descent
#
#
# Example: Strafe-while-turning (lateral + yaw PID to heading)
#
# def cmd_strafe_turn(h, cmd):
#     """Strafe right while PID-rotating to target heading."""
#     h.set_movement(
#         {CH_FORWARD: NEUTRAL_PWM,
#          CH_LATERAL: NEUTRAL_PWM + h.offset},
#         f'Strafe-turn → {cmd.angle}°')
#     h.activate_yaw_pid(cmd.angle)
#
# MOVEMENTS['strafe_turn'] = cmd_strafe_turn
