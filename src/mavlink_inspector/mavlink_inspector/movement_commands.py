"""Movement command definitions for BRACU Duburi 4.2.

This file contains all movement, yaw, cruise, and teleop handlers.
Each movement gets an automatic ``just_*`` variant (bypasses ramp).

Commands are registered using the @register decorator from command_registry.
The MOVEMENTS dict is auto-populated from the registry for backward compatibility.

Adding a New Movement
=====================

1. Define a handler function with the @register decorator::

       @register('my_maneuver', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
                 description='My custom maneuver',
                 channels=['CH_FORWARD', 'CH_YAW'])
       def cmd_my_maneuver(h, cmd):
           h.set_movement(
               {CH_FORWARD: NEUTRAL_PWM + h.offset,
                CH_YAW: NEUTRAL_PWM - h.offset // 3},
               f'My maneuver (speed={h.speed})')

2. ``just_my_maneuver`` works automatically (bypasses ramp).

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

from duburi_common.command_registry import (
    register, CommandCategory, CommandTransport, get_all_commands,
)

from .rc_controller import (
    CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW,
    NEUTRAL_PWM, PWM_RANGE,
    build_diagonal_channels,
)


# ── Basic directional movements ─────────────────────────────────────

@register('move_forward', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
          description='Thrust forward on CH_FORWARD',
          channels=['CH_FORWARD', 'CH_LATERAL'],
          aliases=['forward'])
def cmd_move_forward(h, cmd):
    """
    Move forward with optional distance-based control (Phase 3).
    
    Time-based (current):
        forward 30% 5s
    
    Distance-based (Phase 3 - requires cascade_enabled: true):
        To use distance control, call programmatically:
        h.move_distance_cascade('surge', distance=2.0, max_speed_pct=30)
        
        Future: Add 'distance' field to DriverCommand message
    """
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM + h.offset, CH_LATERAL: NEUTRAL_PWM},
        f'Moving forward (speed={h.speed})')



@register('move_back', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
          description='Thrust backward on CH_FORWARD',
          channels=['CH_FORWARD', 'CH_LATERAL'],
          aliases=['back', 'backward'])
def cmd_move_back(h, cmd):
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM - h.offset, CH_LATERAL: NEUTRAL_PWM},
        'Moving backward')


@register('move_left', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
          description='Strafe left on CH_LATERAL',
          channels=['CH_FORWARD', 'CH_LATERAL'],
          aliases=['left'])
def cmd_move_left(h, cmd):
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM - h.offset},
        'Moving left')


@register('move_right', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
          description='Strafe right on CH_LATERAL',
          channels=['CH_FORWARD', 'CH_LATERAL'],
          aliases=['right'])
def cmd_move_right(h, cmd):
    h.set_movement(
        {CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM + h.offset},
        'Moving right')


@register('move_up', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
          description='Thrust up on CH_THROTTLE',
          channels=['CH_THROTTLE'],
          supports_depth=True,
          aliases=['up'])
def cmd_move_up(h, cmd):
    h.set_movement({CH_THROTTLE: NEUTRAL_PWM + h.offset}, 'Moving up')


@register('move_down', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
          description='Thrust down on CH_THROTTLE',
          channels=['CH_THROTTLE'],
          supports_depth=True,
          aliases=['down'])
def cmd_move_down(h, cmd):
    h.set_movement({CH_THROTTLE: NEUTRAL_PWM - h.offset}, 'Moving down')


@register('move_at', CommandCategory.TRANSLATION, CommandTransport.TOPIC,
          description='Move at arbitrary angle (body-frame vector)',
          channels=['CH_FORWARD', 'CH_LATERAL'],
          supports_bearing=True,
          supports_angle=True)
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

@register('yaw_angle', CommandCategory.HEADING, CommandTransport.ACTION,
          description='Set heading via SET_ATTITUDE_TARGET (firmware-level)',
          channels=['CH_YAW'],
          supports_angle=True)
def cmd_yaw_angle(h, cmd):
    """Set heading via SET_ATTITUDE_TARGET (firmware-level)."""
    n = h.node
    n._set_target_attitude(0, 0, cmd.angle)
    n._publish_event('movement', f'Setting heading to {cmd.angle}°')
    n._publish_feedback('yaw_angle', 'accepted',
                        detail=f'target={cmd.angle}° (attitude target)')


@register('yaw_to_heading', CommandCategory.HEADING, CommandTransport.ACTION,
          description='Bang-bang yaw to target heading',
          channels=['CH_YAW'],
          supports_angle=True)
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


@register('pid_yaw_to_heading', CommandCategory.HEADING, CommandTransport.ACTION,
          description='PID-controlled yaw to target heading',
          channels=['CH_YAW'],
          supports_angle=True,
          is_pid=True)
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


# ── Phase 2: Rotate-in-Place Commands ────────────────────────────────

@register('turn', CommandCategory.HEADING, CommandTransport.ACTION,
          description='Rotate in place to relative heading (Phase 2)',
          channels=['CH_YAW'],
          supports_angle=True,
          aliases=['rotate'])
def cmd_turn_relative(h, cmd):
    """
    Rotate in place to relative heading (Phase 2).
    
    Syntax: turn left|right <degrees> [<speed%>]
    Example: turn left 90 50%
    
    Three-phase execution:
    1. Stop all translation (wait for convergence)
    2. Rotate using precision PID (sharp, on-axis turn)
    3. Brief stability hold
    """
    n = h.node
    
    # Check if rotate-in-place is enabled
    if not n._rotate_in_place_enabled:
        n.get_logger().warning("Rotate-in-place disabled - using legacy yaw command")
        # Fallback to old PID yaw
        h.activate_yaw_pid(cmd.angle)
        n._publish_feedback('turn', 'accepted', detail='legacy mode')
        return
    
    # Parse direction from command text or angle
    # Assuming cmd has 'text' field with something like "turn left 90"
    # For now, determine direction from angle sign or parse cmd.text
    direction = 'right' if cmd.angle >= 0 else 'left'
    angle_abs = abs(cmd.angle)
    speed_pct = h.raw_speed if h.raw_speed > 0 else 50.0
    
    # Calculate target heading
    current_heading = n._telemetry.yaw
    if direction == 'left':
        target_heading = (current_heading + angle_abs) % 360
    else:
        target_heading = (current_heading - angle_abs) % 360
    
    n.get_logger().info(
        f"Rotate-in-place: {direction} {angle_abs:.0f}° "
        f"(from {current_heading:.1f}° to {target_heading:.1f}°)")
    
    # PHASE 1: Stop all translation
    n.get_logger().debug("Phase 1: Stopping translation...")
    h.stop_all_translation(keep_depth=True)
    
    # PHASE 2: Pure rotation with precision PID
    n.get_logger().debug("Phase 2: Rotating...")
    gain_offset = int(PWM_RANGE * speed_pct / 100)
    success = h.activate_yaw_pid_precise(
        heading_deg=target_heading,
        gain_offset=gain_offset
    )
    
    # PHASE 3: Stability hold (brief)
    if success:
        n.get_logger().debug("Phase 3: Stability hold...")
        time.sleep(0.3)  # Brief hold
        
        # Final convergence check on yaw rate
        if n._convergence_enabled:
            # Check heading rate < 1 deg/s for 200ms
            stable_count = 0
            for _ in range(4):  # 4 × 50ms = 200ms
                if abs(n._telemetry.heading_rate) < 1.0:
                    stable_count += 1
                time.sleep(0.05)
            
            if stable_count >= 3:
                n.get_logger().info("✓ Rotation complete and stable")
            else:
                n.get_logger().warning("⚠ Heading still moving after rotation")
        
        n._publish_feedback('turn', 'reached',
                           detail=f'{direction} {angle_abs:.0f}° complete')
    else:
        n._publish_feedback('turn', 'timeout',
                           detail=f'{direction} {angle_abs:.0f}° timeout')


@register('turn_to', CommandCategory.HEADING, CommandTransport.ACTION,
          description='Rotate in place to absolute heading (Phase 2)',
          channels=['CH_YAW'],
          supports_angle=True)
def cmd_turn_absolute(h, cmd):
    """
    Rotate in place to absolute heading.
    
    Syntax: turn_to <heading>
    Example: turn_to 180
    """
    n = h.node
    
    if not n._rotate_in_place_enabled:
        n.get_logger().warning("Rotate-in-place disabled - using legacy yaw command")
        h.activate_yaw_pid(cmd.angle)
        n._publish_feedback('turn_to', 'accepted', detail='legacy mode')
        return
    
    target_heading = cmd.angle % 360
    current_heading = n._telemetry.yaw
    
    n.get_logger().info(
        f"Rotate-in-place to {target_heading:.1f}° "
        f"(currently {current_heading:.1f}°)")
    
    # Same three-phase approach
    h.stop_all_translation(keep_depth=True)
    
    speed_pct = h.raw_speed if h.raw_speed > 0 else 50.0
    gain_offset = int(PWM_RANGE * speed_pct / 100)
    
    success = h.activate_yaw_pid_precise(
        heading_deg=target_heading,
        gain_offset=gain_offset
    )
    
    if success:
        time.sleep(0.3)  # Stability hold
        n._publish_feedback('turn_to', 'reached',
                           detail=f'heading {target_heading:.1f}° reached')
    else:
        n._publish_feedback('turn_to', 'timeout',
                           detail=f'heading {target_heading:.1f}° timeout')


@register('yaw_left', CommandCategory.HEADING, CommandTransport.TOPIC,
          description='Continuous yaw left on CH_YAW',
          channels=['CH_YAW'])
def cmd_yaw_left(h, cmd):
    h.set_movement({CH_YAW: NEUTRAL_PWM - h.offset}, 'Yaw left')


@register('yaw_right', CommandCategory.HEADING, CommandTransport.TOPIC,
          description='Continuous yaw right on CH_YAW',
          channels=['CH_YAW'])
def cmd_yaw_right(h, cmd):
    h.set_movement({CH_YAW: NEUTRAL_PWM + h.offset}, 'Yaw right')


# ── Surface ──────────────────────────────────────────────────────────

@register('surface', CommandCategory.DEPTH, CommandTransport.ACTION,
          description='Ascend to surface (ALT_HOLD or MANUAL throttle)',
          channels=['CH_THROTTLE'],
          supports_depth=True,
          requires_armed=False)
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

@register('cruise', CommandCategory.COMPOUND, CommandTransport.ACTION,
          description='Coordinated movement + depth PID + yaw PID',
          channels=['CH_FORWARD', 'CH_LATERAL', 'CH_THROTTLE', 'CH_YAW'],
          supports_bearing=True,
          supports_angle=True,
          supports_depth=True,
          is_pid=True)
def cmd_cruise(h, cmd):
    _cruise_common(h, cmd, bypass_ramp=False)


@register('just_cruise', CommandCategory.COMPOUND, CommandTransport.ACTION,
          description='Cruise with bypass ramp (instant)',
          channels=['CH_FORWARD', 'CH_LATERAL', 'CH_THROTTLE', 'CH_YAW'],
          supports_bearing=True,
          supports_angle=True,
          supports_depth=True,
          is_pid=True,
          is_instant=True)
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

@register('teleop', CommandCategory.COMPOUND, CommandTransport.TOPIC,
          description='Direct PWM teleop (fields repurposed as axis offsets)',
          channels=['CH_FORWARD', 'CH_LATERAL', 'CH_THROTTLE', 'CH_YAW'],
          supports_duration=False,
          supports_speed=False)
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
#  MOVEMENT REGISTRY - BACKWARD COMPATIBILITY
# ═════════════════════════════════════════════════════════════════════
# Build MOVEMENTS dict from the registry for backward compatibility.
# New code should use get_command() from command_registry instead.
#
# Every entry automatically gets a ``just_*`` variant for free.
# Prefix-based commands (go_*, move_*_*) are matched automatically
# and do NOT need registry entries.

def _build_movements_dict() -> dict[str, callable]:
    """Build MOVEMENTS dict from registry + aliases."""
    result = {}
    for name, spec in get_all_commands().items():
        if spec.handler is not None and spec.category in (
                CommandCategory.TRANSLATION, CommandCategory.HEADING,
                CommandCategory.COMPOUND, CommandCategory.DEPTH):
            result[name] = spec.handler
            # Add aliases
            for alias in spec.aliases:
                result[alias] = spec.handler
    return result

MOVEMENTS: dict[str, callable] = _build_movements_dict()


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
