"""
Mission file command parser for Duburi 4.2 AUV.

Parses individual command lines from mission text files into DriverCommand
messages.  Supports 'just' prefix, '~' PID prefix, and backward-compatible
aliases (dive→depth, p_dive→~depth, etc.).
"""

from duburi_interfaces.msg import DriverCommand

from mavlink_driver.driver_client import (
    arm,
    calibrate_depth,
    disarm,
    go_combo,
    move_at,
    move_combo,
    move_forward,
    move_back,
    move_left,
    move_right,
    move_up,
    move_down,
    set_depth,
    set_mode,
    stop,
    yaw_to_heading,
    yaw_left,
    yaw_right,
    pid_yaw,
    pid_depth,
    pid_depth_off,
    surface,
    turn_left,
    turn_right,
    pid_turn_left,
    pid_turn_right,
    cruise,
)
from mavlink_driver.just_commands import (
    just_move_forward,
    just_move_back,
    just_move_left,
    just_move_right,
    just_move_up,
    just_move_down,
    just_yaw_left,
    just_yaw_right,
    just_move_combo,
    just_move_at,
    just_go_combo,
    just_surface,
    just_cruise,
)


def parse_file_command(current_heading: float, cmd: str,
                       args: list[str], logger=None) -> DriverCommand | None:
    """Parse a single mission file command line into a DriverCommand.

    Args:
        current_heading: Current vehicle heading in degrees (for relative turns).
        cmd: The command keyword (first word of the line).
        args: Remaining words after the command.
        logger: Optional ROS logger for warnings.

    Supports 'just' prefix for instant (no-ramp) fallbacks:
      just forward 3 50  → just_move_forward(duration=3, speed=50)
      just go forward 90 5 60  → just_go_combo(...)
    """
    try:
        # ── 'just' prefix: instant (no-ramp) fallback ───────────────
        is_just = False
        if cmd == 'just':
            is_just = True
            if not args:
                return None
            cmd = args[0]
            args = args[1:]

        # ── Resolve ~ prefix (PID) and backward-compatible aliases ──
        is_pid = False
        if cmd.startswith('~'):
            is_pid = True
            cmd = cmd[1:]
        if cmd == 'dive':
            cmd = 'depth'
        elif cmd == 'p_dive':
            cmd = 'depth'
            is_pid = True
        elif cmd == 'yaw':
            cmd = 'heading'
        elif cmd == 'p_yaw':
            cmd = 'heading'
            is_pid = True
        elif cmd == 'p_turn':
            cmd = 'turn'
            is_pid = True
        elif cmd in ('cal_depth', 'calibrate', 'cal'):
            cmd = 'calibrate_depth'

        if cmd == 'arm':
            return arm()
        elif cmd == 'disarm':
            return disarm()
        elif cmd == 'stop':
            return stop()
        elif cmd == 'surface':
            return just_surface() if is_just else surface()
        elif cmd in ('mode', 'set_mode'):
            return set_mode(args[0].upper() if args else 'MANUAL')
        elif cmd == 'calibrate_depth':
            return calibrate_depth()
        elif cmd == 'forward':
            dur = float(args[0]) if args else 3.0
            spd = int(args[1]) if len(args) > 1 else 50
            return just_move_forward(duration=dur, speed=spd) if is_just else move_forward(duration=dur, speed=spd)
        elif cmd == 'back':
            dur = float(args[0]) if args else 3.0
            spd = int(args[1]) if len(args) > 1 else 50
            return just_move_back(duration=dur, speed=spd) if is_just else move_back(duration=dur, speed=spd)
        elif cmd == 'left':
            dur = float(args[0]) if args else 3.0
            spd = int(args[1]) if len(args) > 1 else 50
            return just_move_left(duration=dur, speed=spd) if is_just else move_left(duration=dur, speed=spd)
        elif cmd == 'right':
            dur = float(args[0]) if args else 3.0
            spd = int(args[1]) if len(args) > 1 else 50
            return just_move_right(duration=dur, speed=spd) if is_just else move_right(duration=dur, speed=spd)
        elif cmd == 'up':
            dur = float(args[0]) if args else 3.0
            spd = int(args[1]) if len(args) > 1 else 50
            return just_move_up(duration=dur, speed=spd) if is_just else move_up(duration=dur, speed=spd)
        elif cmd == 'down':
            dur = float(args[0]) if args else 3.0
            spd = int(args[1]) if len(args) > 1 else 50
            return just_move_down(duration=dur, speed=spd) if is_just else move_down(duration=dur, speed=spd)
        elif cmd == 'depth':
            if is_pid:
                if args and args[0] == 'off':
                    return pid_depth_off()
                return pid_depth(float(args[0]) if args else 0.0)
            else:
                return set_depth(float(args[0])) if args else None
        elif cmd == 'heading':
            if not args:
                return None
            if args[0] in ('left', 'right'):
                dur = float(args[1]) if len(args) > 1 else 3.0
                spd = int(args[2]) if len(args) > 2 else 50
                if args[0] == 'left':
                    return just_yaw_left(duration=dur, speed=spd) if is_just else yaw_left(duration=dur, speed=spd)
                else:
                    return just_yaw_right(duration=dur, speed=spd) if is_just else yaw_right(duration=dur, speed=spd)
            if is_pid:
                return pid_yaw(float(args[0])) if args else None
            else:
                return yaw_to_heading(float(args[0])) if args else None
        elif cmd == 'turn':
            # turn/~turn left/right <degrees> [speed]
            if not args or len(args) < 2:
                return None
            direction = args[0]
            angle = float(args[1])
            spd = int(args[2]) if len(args) > 2 else 50
            if is_pid:
                if direction == 'left':
                    return pid_turn_left(current_heading, angle, speed=spd)
                elif direction == 'right':
                    return pid_turn_right(current_heading, angle, speed=spd)
            else:
                if direction == 'left':
                    return turn_left(current_heading, angle, speed=spd)
                elif direction == 'right':
                    return turn_right(current_heading, angle, speed=spd)
            return None
        elif cmd == 'move':
            # Support 'move forward 3 50' syntax (matches runner format)
            if not args:
                return None
            if args[0] == 'at':
                # move at <angle> [duration] [speed]
                if len(args) < 2:
                    return None
                bearing = float(args[1])
                dur = float(args[2]) if len(args) > 2 else 0.0
                spd = int(args[3]) if len(args) > 3 else 50
                return just_move_at(bearing, duration=dur, speed=spd) if is_just else move_at(bearing, duration=dur, speed=spd)
            if is_just:
                return parse_file_command(current_heading, 'just', [args[0]] + args[1:], logger=logger)
            return parse_file_command(current_heading, args[0], args[1:], logger=logger)
        elif cmd == 'at':
            # at <angle> [duration] [speed]
            if not args:
                return None
            bearing = float(args[0])
            dur = float(args[1]) if len(args) > 1 else 0.0
            spd = int(args[2]) if len(args) > 2 else 50
            return just_move_at(bearing, duration=dur, speed=spd) if is_just else move_at(bearing, duration=dur, speed=spd)
        elif cmd == 'go':
            # go <direction> <heading> [duration] [speed]
            if not args or len(args) < 2:
                return None
            direction = args[0]
            heading = float(args[1])
            dur = float(args[2]) if len(args) > 2 else 0.0
            spd = int(args[3]) if len(args) > 3 else 50
            if is_just:
                return just_go_combo(direction, angle=heading, duration=dur, speed=spd)
            return go_combo(direction, angle=heading, duration=dur, speed=spd)
        elif cmd == 'cruise':
            # cruise <bearing°> <heading°> <depth_m> [duration] [speed]
            if not args or len(args) < 3:
                return None
            bearing = float(args[0])
            heading_val = float(args[1])
            depth_val = float(args[2])
            dur = float(args[3]) if len(args) > 3 else 0.0
            spd = int(args[4]) if len(args) > 4 else 50
            if is_just:
                return just_cruise(bearing, heading_val, depth=depth_val, duration=dur, speed=spd)
            return cruise(bearing, heading_val, depth=depth_val, duration=dur, speed=spd)
        elif '-' in cmd:
            # Compound diagonal: forward-right 5 50 (horizontal only)
            VALID = {'forward', 'back', 'backward', 'left', 'right'}
            parts = cmd.split('-')
            if len(parts) == 2 and all(p in VALID for p in parts):
                dur = float(args[0]) if args else 3.0
                spd = int(args[1]) if len(args) > 1 else 50
                if is_just:
                    return just_move_combo(cmd, duration=dur, speed=spd)
                return move_combo(cmd, duration=dur, speed=spd)
            return None
        elif cmd == 'grabber':
            if args and args[0] == 'open':
                from mavlink_driver.driver_client import open_grabber
                return open_grabber()
            elif args and args[0] == 'close':
                from mavlink_driver.driver_client import close_grabber
                return close_grabber()
    except (IndexError, ValueError) as e:
        if logger:
            logger.warn(f'Bad arguments for {cmd}: {e}')
    return None
