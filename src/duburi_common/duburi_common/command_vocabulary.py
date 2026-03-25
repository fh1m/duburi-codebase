"""
Shared command vocabulary for the Duburi AUV 4.2 stack.

Centralizes alias resolution, prefix handling, and direction maps so
that both the interactive CLI (mavlink_runner) and mission file parser
(mavlink_driver) share identical logic. Change once, works everywhere.
"""

from __future__ import annotations

# ── Aliases: old name -> canonical name (or tuple with PID flag) ─────
ALIASES: dict[str, str | tuple[str, bool]] = {
    'dive':       'depth',
    'p_dive':     ('depth', True),
    'yaw':        'heading',
    'p_yaw':      ('heading', True),
    'p_turn':     ('turn', True),
    'cal_depth':  'calibrate_depth',
    'calibrate':  'calibrate_depth',
    'cal':        'calibrate_depth',
}

# ── Direction name -> DriverCommand.command name ─────────────────────
DIRECTION_TO_COMMAND: dict[str, str] = {
    'left':     'move_left',
    'right':    'move_right',
    'forward':  'move_forward',
    'back':     'move_back',
    'backward': 'move_back',
    'up':       'move_up',
    'down':     'move_down',
}

# ── Valid horizontal directions (for compound/go commands) ───────────
HORIZONTAL_DIRS = frozenset({'forward', 'back', 'backward', 'left', 'right'})


def resolve_prefixes(cmd: str, args: list[str]
                     ) -> tuple[str, list[str], bool, bool]:
    """Resolve 'just' prefix, '~' PID prefix, and aliases.

    Returns:
        (cmd, args, is_just, is_pid)
    """
    is_just = False
    if cmd == 'just':
        is_just = True
        if not args:
            return cmd, args, is_just, False
        cmd = args[0]
        args = args[1:]

    is_pid = False
    if cmd.startswith('~'):
        is_pid = True
        cmd = cmd[1:]

    alias = ALIASES.get(cmd)
    if alias is not None:
        if isinstance(alias, tuple):
            cmd, is_pid = alias[0], True
        else:
            cmd = alias

    return cmd, args, is_just, is_pid


def build_command_name(direction: str, is_just: bool = False) -> str | None:
    """Map a bare direction to a DriverCommand name.

    Returns None if the direction is not recognized.
    """
    base = DIRECTION_TO_COMMAND.get(direction)
    if base is None:
        return None
    return f'just_{base}' if is_just else base


def build_compound_name(parts: list[str], is_just: bool = False) -> str | None:
    """Build a compound diagonal command name from direction parts.

    E.g. ['forward', 'right'] -> 'move_forward_right'
    Returns None if any part is not a valid horizontal direction.
    """
    normalized = [p.replace('backward', 'back') for p in parts]
    if not all(p in {'forward', 'back', 'left', 'right'} for p in normalized):
        return None
    name = 'move_' + '_'.join(normalized)
    return f'just_{name}' if is_just else name


def is_valid_compound(direction_str: str) -> bool:
    """Check if a hyphenated string is a valid compound direction."""
    parts = direction_str.replace('backward', 'back').split('-')
    return (len(parts) == 2
            and all(p in HORIZONTAL_DIRS for p in parts))
