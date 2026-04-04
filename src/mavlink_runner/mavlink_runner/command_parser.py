"""
Grammar-based command parser for the Duburi AUV CLI.
Parses user input into DriverCommand messages using the central registry.
"""
from __future__ import annotations
import re
from duburi_interfaces.msg import DriverCommand
from duburi_common.command_vocabulary import (
    resolve_prefixes, DIRECTION_TO_COMMAND, build_command_name,
    build_compound_name, is_valid_compound, ALIASES,
)
from duburi_common.command_registry import is_registered
from .constants import HELP_TEXT
from .status_display import print_status

RE_PERCENT, RE_SECONDS = re.compile(r'(\d+(?:\.\d+)?)\s*%'), re.compile(r'(\d+(?:\.\d+)?)\s*s\b')

def _extract(line: str, default: float) -> tuple[float, float, str]:
    """Extract speed percentage and duration from command line.
    
    Args:
        line: Command line string
        default: Default speed percentage if not specified
    
    Returns:
        Tuple of (speed_percent, duration_seconds, cleaned_line)
    """
    speed_percent = min(100, max(0, float(m.group(1)))) if (m := RE_PERCENT.search(line)) else default
    duration_seconds = float(m.group(1)) if (m := RE_SECONDS.search(line)) else 0.0
    cleaned_line = ' '.join(RE_SECONDS.sub('', RE_PERCENT.sub('', line)).split())
    return speed_percent, duration_seconds, cleaned_line

def _jn(name: str, j: bool) -> str: return f'just_{name}' if j else name

def parse_command(node, line: str) -> tuple[bool, float]:
    """Parse and execute one command. Returns (continue, wait_sec)."""
    if not (line := line.strip().lower()): return True, 0.0
    speed_percent, duration_seconds, line = _extract(line, node._default_speed)
    if not (parts := line.split()): return True, 0.0
    cmd, args = parts[0], parts[1:]
    cmd, args, is_just, is_pid = resolve_prefixes(cmd, args)
    just_prefix = '[JUST] ' if is_just else ''

    # Meta commands
    if cmd in ('quit', 'exit', 'q'): return False, 0.0
    if cmd == 'help': print(HELP_TEXT); return True, 0.0
    if cmd == 'status': print_status(node._last_state, node._last_diag, node._last_state_time); return True, 0.0
    if cmd == 'list' and args and args[0] == 'missions': node._list_missions(); return True, 0.0
    if cmd == 'planner':
        sub = args[0] if args else ''
        {'stop': node._planner_stop, 'viewer': node._planner_viewer}.get(sub, node._planner_list)()
        return True, 0.0

    # System commands
    if cmd == 'stop': node._publish(DriverCommand(command='stop')); print('Stopping.'); return True, 0.0
    if cmd == 'arm': node._publish(DriverCommand(command='arm')); print('Arming...'); return True, 4.0
    if cmd == 'disarm': node._publish(DriverCommand(command='disarm')); print('Disarming...'); return True, 2.0
    if cmd == 'mode':
        mode_str = args[0].upper().replace('MANNUAL', 'MANUAL') if args else 'MANUAL'
        node._publish(DriverCommand(command='set_mode', mode=mode_str)); print(f'Mode {mode_str}'); return True, 0.0
    if cmd == 'calibrate_depth': node._publish(DriverCommand(command='calibrate_depth')); print('Calibrating.'); return True, 0.0

    # Depth
    if cmd == 'depth':
        if is_pid:
            if args and args[0] == 'off': node._publish(DriverCommand(command='pid_depth_off')); print('~depth OFF'); return True, 0.0
            depth_value = float(args[0].rstrip('m')) if args else 0.0
            node._publish(DriverCommand(command='set_mode', mode='STABILIZE'))
            node._publish(DriverCommand(command='pid_depth', depth=depth_value)); print(f'~depth {depth_value}m'); return True, 0.5
        if not args: print('Usage: depth <m>'); return True, 0.0
        depth_value = float(args[0].rstrip('m')); node._publish(DriverCommand(command='set_depth', depth=depth_value)); print(f'Depth {depth_value}m'); return True, 1.0
    if cmd == 'surface': node._publish(DriverCommand(command=_jn('surface', is_just))); print(f'{lbl}Surfacing'); return True, 2.0

    # Heading
    if cmd == 'heading':
        if not args: print('Usage: heading <deg>'); return True, 0.0
        if args[0] in ('left', 'right'):
            node._publish(DriverCommand(command=_jn(f'yaw_{args[0]}', is_just), duration=duration_seconds, speed=int(speed_percent)))
            print(f'{just_prefix}Rotate {args[0]}'); return True, duration_seconds
        heading_cmd = 'pid_yaw_to_heading' if is_pid else 'yaw_to_heading'
        node._publish(DriverCommand(command=heading_cmd, angle=float(args[0]), speed=int(speed_percent))); print(f'Heading {args[0]}°'); return True, 0.0

    # Turn (relative)
    if cmd == 'turn':
        if len(args) < 2 or args[0] not in ('left', 'right'): print('Usage: turn left/right <deg>'); return True, 0.0
        if node._last_state is None: print('[WARN] No telemetry'); return True, 0.0
        current_yaw, angle_offset = node._last_state.yaw, float(args[1])
        target_yaw = (current_yaw + (-angle_offset if args[0] == 'left' else angle_offset)) % 360
        node._publish(DriverCommand(command='pid_yaw_to_heading' if is_pid else 'yaw_to_heading', angle=target_yaw, speed=int(speed_percent)))
        print(f'Turn {args[0]} {angle_offset}° ({current_yaw:.0f}→{target_yaw:.0f})'); return True, 0.0

    # Go / Cruise
    if cmd == 'go':
        if len(args) < 2: print('Usage: go <dir> <hdg>'); return True, 0.0
        direction_parts = args[0].replace('backward', 'back').split('-')
        node._publish(DriverCommand(command=_jn('go_' + '_'.join(direction_parts), is_just), angle=float(args[1]), duration=duration_seconds, speed=int(speed_percent)))
        print(f'{just_prefix}Go {args[0]} → {args[1]}°'); return True, duration_seconds
    if cmd == 'cruise':
        if len(args) < 3: print('Usage: cruise <brg> <hdg> <dep>'); return True, 0.0
        bearing, heading, depth_value = float(args[0]), float(args[1]), float(args[2].rstrip('m'))
        node._publish(DriverCommand(command=_jn('cruise', is_just), angle=bearing, mode=str(heading), depth=depth_value, duration=duration_seconds, speed=int(speed_percent)))
        print(f'{just_prefix}Cruise {bearing}° {heading}° {depth_value}m'); return True, duration_seconds

    # Move
    if cmd == 'move':
        if not args: print('Usage: move <dir>'); return True, 0.0
        direction = args[0]
        if direction == 'at':
            bearing = float(args[1]) if len(args) > 1 else 0.0
            node._publish(DriverCommand(command=_jn('move_at', is_just), angle=bearing, duration=duration_seconds, speed=int(speed_percent)))
            print(f'{just_prefix}At {bearing}°'); return True, duration_seconds
        if direction == 'depth':
            depth_value = float(args[1].rstrip('m')) if len(args) > 1 else 0.0
            node._publish(DriverCommand(command='set_depth', depth=depth_value)); print(f'Depth {depth_value}m'); return True, 1.0
        command_spec = build_command_name(direction, is_just) or build_compound_name(direction.split('-'), is_just)
        if command_spec: node._publish(DriverCommand(command=command_spec, duration=duration_seconds, speed=int(speed_percent))); print(f'{just_prefix}Moving {direction}'); return True, duration_seconds
        print(f'Unknown: {direction}'); return True, 0.0

    if cmd == 'at':
        bearing = float(args[0]) if args else 0.0
        node._publish(DriverCommand(command=_jn('move_at', is_just), angle=bearing, duration=duration_seconds, speed=int(speed_percent)))
        print(f'{just_prefix}At {bearing}°'); return True, duration_seconds

    # Bare direction / compound
    if cmd in DIRECTION_TO_COMMAND:
        node._publish(DriverCommand(command=build_command_name(cmd, is_just), duration=duration_seconds, speed=int(speed_percent)))
        print(f'{just_prefix}Moving {cmd}'); return True, duration_seconds
    if '-' in cmd and is_valid_compound(cmd):
        node._publish(DriverCommand(command=build_compound_name(cmd.split('-'), is_just), duration=duration_seconds, speed=int(speed_percent)))
        print(f'{just_prefix}Moving {cmd}'); return True, duration_seconds

    # Grabber / Vision
    if cmd == 'grabber' and args:
        node._publish(DriverCommand(command='open_grabber' if args[0] == 'open' else 'close_grabber'))
        print(f'{args[0].capitalize()}ing grabber'); return True, 0.0
    vision_mapping = {'lat-align': 'lat_align', 'dep-align': 'dep_align', 'align': 'align', 'align-forward': 'align_forward',
          'vision-stop': 'vision_stop', 'vstop': 'vision_stop'}
    if cmd in vision_mapping:
        vision_cmd = f'pid_{vision_mapping[cmd]}' if is_pid and not vision_mapping[cmd].startswith('vision') else vision_mapping[cmd]
        node._publish(DriverCommand(command=vision_cmd, speed=int(speed_percent), duration=duration_seconds, status='until_aligned' if 'until' in args else ''))
        print(f'Vision {cmd}'); return True, duration_seconds

    # Registry fallback
    canonical_name = ALIASES.get(cmd, cmd)
    if is_registered(canonical_name) or is_registered(f'move_{cmd}'):
        final_cmd = canonical_name if is_registered(canonical_name) else f'move_{cmd}'
        node._publish(DriverCommand(command=final_cmd, duration=duration_seconds, speed=int(speed_percent))); print(f'Exec {final_cmd}'); return True, duration_seconds

    print(f'Unknown: {line}. Type "help".'); return True, 0.0
