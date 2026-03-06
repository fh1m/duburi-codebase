"""
Driver client - helper to publish DriverCommand.
Used by mavlink_runner and mission nodes.
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
    """PID yaw to heading (smooth, closed-loop)."""
    return make_command('pid_yaw_to_heading', angle=angle, speed=speed)


def yaw_to_heading(angle: float, speed: int = 50) -> DriverCommand:
    """Bang-bang yaw to heading."""
    return make_command('yaw_to_heading', angle=angle, speed=speed)


def yaw_left(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('yaw_left', duration=duration, speed=speed)


def yaw_right(duration: float = 0, speed: int = 50) -> DriverCommand:
    return make_command('yaw_right', duration=duration, speed=speed)


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
