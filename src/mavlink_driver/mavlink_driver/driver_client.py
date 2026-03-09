"""
Driver client - helper to publish DriverCommand.
Used by mavlink_runner, mission_executor, and custom mission nodes.

Naming convention:
  - ArduSub (firmware) functions: plain names  (set_depth, yaw_to_heading, turn_left)
  - PID (software) functions:     pid_ prefix  (pid_depth, pid_yaw, pid_turn_left)
  - Instant (no-ramp) functions:  just_ prefix (just_move_forward, just_go_left)
  - Movement:  move_*  = single/compound thrust (ramped)
  - Go:        go_*    = movement + PID heading hold simultaneously (ramped)
  - Just:      just_*  = raw bang-bang fallback, bypasses PWM ramp

Runner/mission CLI mapping:
  depth  → set_depth()       |  ~depth  → pid_depth()
  heading → yaw_to_heading() |  ~heading → pid_yaw()
  turn   → turn_left/right() |  ~turn   → pid_turn_left/right()
  move   → move_*()          |  go      → go_*()
  just forward → just_move_forward()  (instant, no ramp)
"""

from duburi_interfaces.msg import DriverCommand


def make_command(
    command: str,
    mode: str = '',
    depth: float = 0.0,
    angle: float = 0.0,
    duration: float = 0.0,
    speed: int = 50,
    status: str = '',
) -> DriverCommand:
    """Create a DriverCommand message."""
    cmd = DriverCommand()
    cmd.command = command
    cmd.mode = mode
    cmd.depth = depth
    cmd.angle = angle
    cmd.duration = duration
    cmd.speed = speed
    cmd.status = status
    return cmd


# Movement helpers (return DriverCommand for publishing)
def move_forward(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('move_forward', duration=duration, speed=speed)


def move_back(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('move_back', duration=duration, speed=speed)


def move_left(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('move_left', duration=duration, speed=speed)


def move_right(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('move_right', duration=duration, speed=speed)


def move_up(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('move_up', duration=duration, speed=speed)


def move_down(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('move_down', duration=duration, speed=speed)


def yaw_angle(angle: float) -> DriverCommand:
    return make_command('yaw_angle', angle=angle)


def pid_yaw(angle: float, speed: int = 50) -> DriverCommand:
    """PID yaw to absolute heading (smooth, closed-loop)."""
    return make_command('pid_yaw_to_heading', angle=angle, speed=speed)


def yaw_to_heading(angle: float, speed: int = 50) -> DriverCommand:
    """Bang-bang yaw to absolute heading."""
    return make_command('yaw_to_heading', angle=angle, speed=speed)


def yaw_left(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('yaw_left', duration=duration, speed=speed)


def yaw_right(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('yaw_right', duration=duration, speed=speed)


# ── Relative yaw helpers ─────────────────────────────────────────────────

def resolve_relative_yaw(current_heading: float, direction: str, angle: float) -> float:
    """Compute absolute heading from a relative turn.

    Args:
        current_heading: current heading in degrees (0-360).
        direction: 'left' or 'right'.
        angle: degrees to turn (positive).

    Returns:
        Absolute heading in degrees (0-360).
    """
    angle = abs(angle)
    if direction == 'left':
        return (current_heading - angle) % 360
    else:  # right
        return (current_heading + angle) % 360


def turn_left(current_heading: float, angle: float, speed: int = 50) -> DriverCommand:
    """Bang-bang relative turn left by `angle` degrees."""
    target = resolve_relative_yaw(current_heading, 'left', angle)
    return yaw_to_heading(target, speed=speed)


def turn_right(current_heading: float, angle: float, speed: int = 50) -> DriverCommand:
    """Bang-bang relative turn right by `angle` degrees."""
    target = resolve_relative_yaw(current_heading, 'right', angle)
    return yaw_to_heading(target, speed=speed)


def pid_turn_left(current_heading: float, angle: float, speed: int = 50) -> DriverCommand:
    """PID smooth relative turn left by `angle` degrees."""
    target = resolve_relative_yaw(current_heading, 'left', angle)
    return pid_yaw(target, speed=speed)


def pid_turn_right(current_heading: float, angle: float, speed: int = 50) -> DriverCommand:
    """PID smooth relative turn right by `angle` degrees."""
    target = resolve_relative_yaw(current_heading, 'right', angle)
    return pid_yaw(target, speed=speed)


def set_depth(depth: float) -> DriverCommand:
    """Firmware depth hold via ALT_HOLD (auto-switches mode)."""
    return make_command('set_depth', depth=depth)


def pid_depth(depth: float = 0.0) -> DriverCommand:
    """Software PID depth hold via RC throttle (works in any mode).

    depth=0.0 means 'hold current depth'.
    """
    return make_command('pid_depth', depth=depth)


def pid_depth_off() -> DriverCommand:
    """Disable software PID depth hold."""
    return make_command('pid_depth_off')


def surface() -> DriverCommand:
    """Ascend to surface (stops movement + depth hold)."""
    return make_command('surface')


def stop() -> DriverCommand:
    return make_command('stop')


def arm() -> DriverCommand:
    return make_command('arm')


def disarm() -> DriverCommand:
    return make_command('disarm')


def set_mode(mode: str) -> DriverCommand:
    return make_command('set_mode', mode=mode)


def open_grabber() -> DriverCommand:
    return make_command('open_grabber')


def close_grabber() -> DriverCommand:
    return make_command('close_grabber')


# Compound diagonal movement
def move_combo(direction: str, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Horizontal diagonal movement.

    direction: hyphen-separated, e.g. 'forward-right', 'back-left'.
    Only horizontal axes (forward/back/left/right) are supported.
    Speed is auto-scaled per axis by inspector (÷√2).
    """
    parts = direction.replace('backward', 'back').split('-')
    cmd_name = 'move_' + '_'.join(parts)
    return make_command(cmd_name, duration=duration, speed=speed)


# Body-frame vector movement
def move_at(bearing: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Move at arbitrary bearing (body-frame, 0°=forward, 90°=right).

    Decomposes into forward + lateral channels via cos/sin.
    bearing: angle in degrees (0-360, body-relative).
    """
    return make_command('move_at', angle=bearing, duration=duration, speed=speed)


# Simultaneous movement + yaw (go) commands
def go_combo(direction: str, angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Compound diagonal movement + PID yaw to heading.

    direction: hyphen-separated, e.g. 'forward-right'.
    angle: target heading in degrees.
    """
    parts = direction.replace('backward', 'back').split('-')
    cmd_name = 'go_' + '_'.join(parts)
    return make_command(cmd_name, angle=angle, duration=duration, speed=speed)


def go_forward(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Move forward while PID-yawing to heading."""
    return make_command('go_forward', angle=angle, duration=duration, speed=speed)


def go_back(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Move backward while PID-yawing to heading."""
    return make_command('go_back', angle=angle, duration=duration, speed=speed)


def go_left(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Strafe left while PID-yawing to heading."""
    return make_command('go_left', angle=angle, duration=duration, speed=speed)


def go_right(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Strafe right while PID-yawing to heading."""
    return make_command('go_right', angle=angle, duration=duration, speed=speed)


# ── Instant (no-ramp) fallback commands ──────────────────────────────────
# 'just_*' variants bypass the PWM ramp and apply target PWM instantly.
# Use these as fallbacks when ramping causes issues during testing, or
# when you need immediate response without acceleration delay.
#
# Every ramped movement has a just_* counterpart:
#   move_forward()     → just_move_forward()     (instant)
#   move_combo()       → just_move_combo()       (instant diagonal)
#   go_forward()       → just_go_forward()       (instant move + PID yaw)
#   yaw_left()         → just_yaw_left()         (instant open-loop spin)
#   surface()          → just_surface()          (instant throttle up)
#
# Depth PID and heading PID/bang-bang don't use the ramp, so they don't
# need just_* variants.  They already act as their own fallbacks:
#   pid_depth()  ↔  set_depth()       (PID vs firmware)
#   pid_yaw()    ↔  yaw_to_heading()  (PID vs bang-bang)

def just_move_forward(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant forward (no ramp)."""
    return make_command('just_move_forward', duration=duration, speed=speed)


def just_move_back(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant backward (no ramp)."""
    return make_command('just_move_back', duration=duration, speed=speed)


def just_move_left(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant left (no ramp)."""
    return make_command('just_move_left', duration=duration, speed=speed)


def just_move_right(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant right (no ramp)."""
    return make_command('just_move_right', duration=duration, speed=speed)


def just_move_up(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant up (no ramp)."""
    return make_command('just_move_up', duration=duration, speed=speed)


def just_move_down(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant down (no ramp)."""
    return make_command('just_move_down', duration=duration, speed=speed)


def just_yaw_left(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant yaw left (no ramp)."""
    return make_command('just_yaw_left', duration=duration, speed=speed)


def just_yaw_right(duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant yaw right (no ramp)."""
    return make_command('just_yaw_right', duration=duration, speed=speed)


def just_move_combo(direction: str, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant diagonal movement (no ramp)."""
    parts = direction.replace('backward', 'back').split('-')
    cmd_name = 'just_move_' + '_'.join(parts)
    return make_command(cmd_name, duration=duration, speed=speed)


def just_move_at(bearing: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant move at arbitrary bearing (no ramp)."""
    return make_command('just_move_at', angle=bearing, duration=duration, speed=speed)


def just_go_forward(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant forward + PID yaw (no ramp on movement)."""
    return make_command('just_go_forward', angle=angle, duration=duration, speed=speed)


def just_go_back(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant backward + PID yaw (no ramp on movement)."""
    return make_command('just_go_back', angle=angle, duration=duration, speed=speed)


def just_go_left(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant strafe left + PID yaw (no ramp on movement)."""
    return make_command('just_go_left', angle=angle, duration=duration, speed=speed)


def just_go_right(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant strafe right + PID yaw (no ramp on movement)."""
    return make_command('just_go_right', angle=angle, duration=duration, speed=speed)


def just_go_combo(direction: str, angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant diagonal + PID yaw (no ramp on movement)."""
    parts = direction.replace('backward', 'back').split('-')
    cmd_name = 'just_go_' + '_'.join(parts)
    return make_command(cmd_name, angle=angle, duration=duration, speed=speed)


def just_surface() -> DriverCommand:
    """Instant surface (no ramp on throttle)."""
    return make_command('just_surface')


# ── Coordinated maneuver: cruise ─────────────────────────────────────────
# Simultaneously activates movement (bearing), depth PID, and yaw PID.
# Field encoding:
#   angle    → bearing (body-frame, 0°=forward, 90°=right)
#   depth    → target depth (positive metres)
#   speed    → movement speed (0-100%)
#   duration → movement duration (seconds)
#   mode     → target heading (degrees, 0-360) — repurposed string field

def cruise(bearing: float, heading: float, depth: float = 0.0,
           duration: float = 0, speed: int = 50) -> DriverCommand:
    """Coordinated cruise: movement + depth PID + yaw PID simultaneously.

    bearing: body-frame direction (0°=forward, 90°=right).
    heading: target heading in degrees (0-360).
    depth:   target depth in metres (positive).
    """
    return make_command('cruise', mode=str(heading), angle=bearing,
                        depth=depth, duration=duration, speed=speed)


def just_cruise(bearing: float, heading: float, depth: float = 0.0,
                duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant coordinated cruise (no ramp on movement)."""
    return make_command('just_cruise', mode=str(heading), angle=bearing,
                        depth=depth, duration=duration, speed=speed)
