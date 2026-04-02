"""
Mission file command parser for Duburi AUV.
Parses mission file lines into DriverCommand messages using command_vocabulary and command_registry.
"""
from __future__ import annotations
from typing import Optional
from duburi_interfaces.msg import DriverCommand
from duburi_common.command_vocabulary import (
    resolve_prefixes, DIRECTION_TO_COMMAND, build_command_name,
    build_compound_name, ALIASES, HORIZONTAL_DIRS,
)
from duburi_common.command_registry import is_registered

def _jn(name: str, j: bool) -> str: return f'just_{name}' if j else name

def parse_file_command(current_heading: float, cmd: str, args: list[str], logger=None) -> Optional[DriverCommand]:
    """Parse a mission file command line into DriverCommand."""
    if not cmd or cmd.startswith('#'): return None
    try:
        cmd, args, is_just, is_pid = resolve_prefixes(cmd, args)
        if is_just and not args and cmd == 'just': return None

        # Extract speed/duration
        spd, dur, rem = 50, 0.0, []
        for a in args:
            if a.endswith('%'): spd = int(float(a.rstrip('%')))
            elif a.endswith('s') and a[:-1].replace('.', '').isdigit(): dur = float(a.rstrip('s'))
            else: rem.append(a)

        # System
        if cmd in ('arm', 'disarm', 'stop'): return DriverCommand(command=cmd)
        if cmd in ('mode', 'set_mode'): return DriverCommand(command='set_mode', mode=rem[0].upper() if rem else 'MANUAL')
        if cmd in ('calibrate', 'cal', 'calibrate_depth'): return DriverCommand(command='calibrate_depth')

        # Depth
        if cmd in ('depth', 'dive'):
            dv = float(rem[0].rstrip('m')) if rem else 0.0
            return DriverCommand(command='pid_depth' if is_pid else 'set_depth', depth=dv)
        if cmd == 'surface': return DriverCommand(command=_jn('surface', is_just))

        # Heading
        if cmd in ('heading', 'yaw'):
            if not rem: return None
            if rem[0] in ('left', 'right'):
                d = float(rem[1]) if len(rem) > 1 else 3.0
                return DriverCommand(command=_jn(f'yaw_{rem[0]}', is_just), duration=d, speed=spd)
            return DriverCommand(command='pid_yaw_to_heading' if is_pid else 'yaw_to_heading', angle=float(rem[0]), speed=spd)

        # Turn
        if cmd == 'turn':
            if len(rem) < 2: return None
            delta = float(rem[1]) * (1 if rem[0] == 'right' else -1)
            return DriverCommand(command='pid_yaw_to_heading' if is_pid else 'yaw_to_heading',
                                 angle=(current_heading + delta) % 360, speed=spd)

        # Go / Cruise
        if cmd == 'go':
            if len(rem) < 2: return None
            p = rem[0].replace('backward', 'back').split('-')
            return DriverCommand(command=_jn('go_' + '_'.join(p), is_just), angle=float(rem[1]), duration=dur, speed=spd)
        if cmd == 'cruise':
            if len(rem) < 3: return None
            return DriverCommand(command=_jn('cruise', is_just), angle=float(rem[0]),
                                 mode=str(float(rem[1])), depth=float(rem[2]), duration=dur, speed=spd)

        # Direction commands
        if cmd in DIRECTION_TO_COMMAND:
            d = float(rem[0]) if rem else dur or 3.0
            s = int(rem[1]) if len(rem) > 1 else spd
            return DriverCommand(command=build_command_name(cmd, is_just), duration=d, speed=s)

        # Move
        if cmd == 'move' and rem:
            dr = rem[0]
            if dr == 'at':
                br = float(rem[1]) if len(rem) > 1 else 0.0
                return DriverCommand(command=_jn('move_at', is_just), angle=br, duration=dur, speed=spd)
            cs = build_command_name(dr, is_just) if dr in DIRECTION_TO_COMMAND else build_compound_name(dr.split('-'), is_just)
            d = float(rem[1]) if len(rem) > 1 else dur or 3.0
            return DriverCommand(command=cs, duration=d, speed=spd)

        if cmd == 'at':
            return DriverCommand(command=_jn('move_at', is_just), angle=float(rem[0]) if rem else 0.0, duration=dur, speed=spd)

        # Compound
        if '-' in cmd and all(p in HORIZONTAL_DIRS for p in cmd.split('-')):
            d = float(rem[0]) if rem else dur or 3.0
            return DriverCommand(command=build_compound_name(cmd.split('-'), is_just), duration=d, speed=spd)

        # Grabber
        if cmd == 'grabber' and rem:
            return DriverCommand(command='open_grabber' if rem[0] == 'open' else 'close_grabber')

        # Registry fallback
        cn = ALIASES.get(cmd, cmd)
        if is_registered(cn): return DriverCommand(command=cn, duration=dur, speed=spd)

    except (IndexError, ValueError) as e:
        if logger: logger.warn(f'Bad args for {cmd}: {e}')
    return None
