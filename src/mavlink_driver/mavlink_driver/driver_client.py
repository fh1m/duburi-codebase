"""
Driver client functions for Duburi AUV.

DEPRECATED: Use DuburiClient class instead for new code.
This module is kept for backward compatibility with existing missions.

Example migration:
    # Old way (deprecated):
    from mavlink_driver.driver_client import move_forward, arm
    msg = move_forward(duration=3, speed=50)
    
    # New way (preferred):
    from mavlink_driver.duburi_client import DuburiClient
    duburi = DuburiClient(node)
    duburi.move_forward(speed=50, duration=3)
"""

import warnings
from duburi_interfaces.msg import DriverCommand

_DEPRECATION_MSG = (
    "driver_client functions are deprecated. "
    "Use DuburiClient class from duburi_client module instead."
)


def make_command(
    command: str,
    mode: str = '',
    depth: float = 0.0,
    angle: float = 0.0,
    duration: float = 0.0,
    speed: int = 50,
    status: str = '',
) -> DriverCommand:
    """Create a DriverCommand message.

    DEPRECATED: Use DuburiClient methods instead.
    """
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    msg = DriverCommand()
    msg.command = command
    msg.mode = mode
    msg.depth = depth
    msg.angle = angle
    msg.duration = duration
    msg.speed = speed
    msg.status = status
    # Also set new fields for forward compat
    msg.speed_pct = float(speed) if speed else 0.0
    msg.target_depth = depth
    msg.target_heading = angle
    msg.flight_mode = mode
    return msg


# Movement helpers (return DriverCommand for publishing)
def move_forward(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('move_forward', duration=duration, speed=speed)


def move_back(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('move_back', duration=duration, speed=speed)


def move_left(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('move_left', duration=duration, speed=speed)


def move_right(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('move_right', duration=duration, speed=speed)


def move_up(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('move_up', duration=duration, speed=speed)


def move_down(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('move_down', duration=duration, speed=speed)


def yaw_angle(angle: float) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('yaw_angle', angle=angle)


def pid_yaw(angle: float, speed: int = 50) -> DriverCommand:
    """PID yaw to absolute heading (smooth, closed-loop)."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('pid_yaw_to_heading', angle=angle, speed=speed)


def yaw_to_heading(angle: float, speed: int = 50) -> DriverCommand:
    """Bang-bang yaw to absolute heading."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('yaw_to_heading', angle=angle, speed=speed)


def yaw_left(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('yaw_left', duration=duration, speed=speed)


def yaw_right(duration: float = 0, speed: int = 50) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
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
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    target = resolve_relative_yaw(current_heading, 'left', angle)
    return yaw_to_heading(target, speed=speed)


def turn_right(current_heading: float, angle: float, speed: int = 50) -> DriverCommand:
    """Bang-bang relative turn right by `angle` degrees."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    target = resolve_relative_yaw(current_heading, 'right', angle)
    return yaw_to_heading(target, speed=speed)


def pid_turn_left(current_heading: float, angle: float, speed: int = 50) -> DriverCommand:
    """PID smooth relative turn left by `angle` degrees."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    target = resolve_relative_yaw(current_heading, 'left', angle)
    return pid_yaw(target, speed=speed)


def pid_turn_right(current_heading: float, angle: float, speed: int = 50) -> DriverCommand:
    """PID smooth relative turn right by `angle` degrees."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    target = resolve_relative_yaw(current_heading, 'right', angle)
    return pid_yaw(target, speed=speed)


def set_depth(depth: float) -> DriverCommand:
    """Firmware depth hold via ALT_HOLD (auto-switches mode)."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('set_depth', depth=depth)


def pid_depth(depth: float = 0.0) -> DriverCommand:
    """Software PID depth hold via RC throttle (works in any mode).

    depth=0.0 means 'hold current depth'.
    """
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('pid_depth', depth=depth)


def pid_depth_off() -> DriverCommand:
    """Disable software PID depth hold."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('pid_depth_off')


def surface() -> DriverCommand:
    """Ascend to surface (stops movement + depth hold)."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('surface')


def stop() -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('stop')


def arm() -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('arm')


def disarm() -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('disarm')


def set_mode(mode: str) -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('set_mode', mode=mode)


def calibrate_depth() -> DriverCommand:
    """Record current depth as the surface reference for PID depth offset."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('calibrate_depth')


def open_grabber() -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('open_grabber')


def close_grabber() -> DriverCommand:
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('close_grabber')


# Compound diagonal movement
def move_combo(direction: str, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Horizontal diagonal movement.

    direction: hyphen-separated, e.g. 'forward-right', 'back-left'.
    Only horizontal axes (forward/back/left/right) are supported.
    Speed is auto-scaled per axis by inspector (÷√2).
    """
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    parts = direction.replace('backward', 'back').split('-')
    cmd_name = 'move_' + '_'.join(parts)
    return make_command(cmd_name, duration=duration, speed=speed)


# Body-frame vector movement
def move_at(bearing: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Move at arbitrary bearing (body-frame, 0°=forward, 90°=right).

    Decomposes into forward + lateral channels via cos/sin.
    bearing: angle in degrees (0-360, body-relative).
    """
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('move_at', angle=bearing, duration=duration, speed=speed)


# Simultaneous movement + yaw (go) commands
def go_combo(direction: str, angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Compound diagonal movement + PID yaw to heading.

    direction: hyphen-separated, e.g. 'forward-right'.
    angle: target heading in degrees.
    """
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    parts = direction.replace('backward', 'back').split('-')
    cmd_name = 'go_' + '_'.join(parts)
    return make_command(cmd_name, angle=angle, duration=duration, speed=speed)


def go_forward(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Move forward while PID-yawing to heading."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('go_forward', angle=angle, duration=duration, speed=speed)


def go_back(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Move backward while PID-yawing to heading."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('go_back', angle=angle, duration=duration, speed=speed)


def go_left(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Strafe left while PID-yawing to heading."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('go_left', angle=angle, duration=duration, speed=speed)


def go_right(angle: float, duration: float = 0, speed: int = 50) -> DriverCommand:
    """Strafe right while PID-yawing to heading."""
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('go_right', angle=angle, duration=duration, speed=speed)


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
    warnings.warn(_DEPRECATION_MSG, DeprecationWarning, stacklevel=2)
    return make_command('cruise', mode=str(heading), angle=bearing,
                        depth=depth, duration=duration, speed=speed)
