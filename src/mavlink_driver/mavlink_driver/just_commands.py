"""
Instant (no-ramp) fallback commands for Duburi 4.2 AUV.

'just_*' variants bypass the PWM ramp and apply target PWM instantly.
Use these as fallbacks when ramping causes issues during testing, or
when you need immediate response without acceleration delay.

Every ramped movement has a just_* counterpart:
  move_forward()     → just_move_forward()     (instant)
  move_combo()       → just_move_combo()       (instant diagonal)
  go_forward()       → just_go_forward()       (instant move + PID yaw)
  yaw_left()         → just_yaw_left()         (instant open-loop spin)
  surface()          → just_surface()          (instant throttle up)

Depth PID and heading PID/bang-bang don't use the ramp, so they don't
need just_* variants.  They already act as their own fallbacks:
  pid_depth()  ↔  set_depth()       (PID vs firmware)
  pid_yaw()    ↔  yaw_to_heading()  (PID vs bang-bang)
"""

from duburi_interfaces.msg import DriverCommand

from mavlink_driver.driver_client import make_command


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


def just_cruise(bearing: float, heading: float, depth: float = 0.0,
                duration: float = 0, speed: int = 50) -> DriverCommand:
    """Instant coordinated cruise (no ramp on movement)."""
    return make_command('just_cruise', mode=str(heading), angle=bearing,
                        depth=depth, duration=duration, speed=speed)
