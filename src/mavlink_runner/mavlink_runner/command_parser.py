"""
Command parser for the Duburi AUV CLI runner.

Parses free-form user input into DriverCommand messages and publishes them
via the runner node.
"""

import re

from duburi_interfaces.msg import DriverCommand

from .constants import HELP_TEXT
from .status_display import print_status


# Valid horizontal directions for compound commands
_HORIZONTAL_DIRS = {'forward', 'back', 'backward', 'left', 'right'}


def try_compound_move(node, direction: str, gain: float, duration: float,
                      is_just: bool = False
                      ) -> tuple[bool, float] | tuple[None, float]:
    """Try to parse a hyphenated compound direction (e.g. forward-right).

    Only horizontal directions are supported (forward/back + left/right).
    If is_just=True, prepends 'just_' to the command name for no-ramp.
    Returns (True/False, wait_seconds) on valid compound, or (None, 0) if
    the direction string is not a valid compound.
    """
    parts = direction.replace('backward', 'back').split('-')
    if len(parts) != 2 or not all(d in _HORIZONTAL_DIRS for d in parts):
        return None, 0.0
    cmd_name = 'move_' + '_'.join(p.replace('backward', 'back') for p in parts)
    if is_just:
        cmd_name = f'just_{cmd_name}'
    c = DriverCommand(command=cmd_name, duration=duration, speed=int(gain))
    if node._publish(c):
        extra = []
        if gain != node._default_speed:
            extra.append(f'{gain}%')
        if duration:
            extra.append(f'{duration}s')
        label = '[JUST] ' if is_just else ''
        print(f'{label}Moving {direction}' + (' ' + ' '.join(extra) if extra else '')
              + (' (indefinite)' if not duration else ''))
        return True, duration
    return True, 0.0


def parse_command(node, line: str) -> tuple[bool, float]:
    """Parse and execute one command. Returns (continue, wait_sec).

    Args:
        node: DuburiRunnerNode instance (provides _publish, _default_speed,
              _last_state, _last_diag, _last_state_time, _list_missions).
        line: Raw user input string.
    """
    line = line.strip().lower()
    if not line:
        return True, 0.0

    # Extract duration and gain (don't strip from depth values)
    duration = 0.0
    gain = node._default_speed
    for m in re.finditer(r'(\d+(?:\.\d+)?)\s*s\b', line):
        duration = float(m.group(1))
    for m in re.finditer(r'(\d+(?:\.\d+)?)\s*%', line):
        gain = min(100, max(0, float(m.group(1))))
    line = re.sub(r'\d+(?:\.\d+)?\s*s\b', '', line)
    line = re.sub(r'\d+(?:\.\d+)?\s*%', '', line)
    line = ' '.join(line.split())

    parts = line.split()
    if not parts:
        return True, 0.0

    cmd = parts[0]
    args = parts[1:]

    # ── 'just' prefix — instant (no-ramp) fallback ─────────────────
    # Strips 'just' and prepends 'just_' to the DriverCommand name.
    # e.g. 'just forward 50% 3s' → command='just_forward'
    #      'just move left'      → command='just_move_left'
    #      'just go forward 90'  → command='just_go_forward'
    is_just = False
    if cmd == 'just':
        is_just = True
        if not args:
            print('Usage: just <command> — prefix any movement with "just" for no-ramp.')
            return True, 0.0
        cmd = args[0]
        args = args[1:]

    # ── Resolve ~ prefix (PID) and backward-compatible aliases ────
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

    if cmd in ('quit', 'exit', 'q'):
        return False, 0.0

    if cmd == 'help':
        print(HELP_TEXT)
        return True, 0.0

    if cmd == 'status':
        print_status(node._last_state, node._last_diag, node._last_state_time)
        return True, 0.0

    if cmd == 'list' and args and args[0] == 'missions':
        node._list_missions()
        return True, 0.0

    if cmd == 'stop':
        node._publish(DriverCommand(command='stop'))
        print('Stopping all thrusters.')
        return True, 0.0

    # Helper: prepend 'just_' to command name if is_just is set
    def _cmd_name(name: str) -> str:
        return f'just_{name}' if is_just else name

    _just_label = '[JUST] ' if is_just else ''

    if cmd == 'arm':
        print('Arming...')
        node._publish(DriverCommand(command='arm'))
        return True, 4.0  # Wait for vehicle to arm before next command

    if cmd == 'disarm':
        print('Disarming...')
        node._publish(DriverCommand(command='disarm'))
        return True, 2.0  # Wait for disarm to complete

    if cmd == 'mode':
        mode = args[0].upper().replace('MANNUAL', 'MANUAL') if args else 'MANUAL'
        node._publish(DriverCommand(command='set_mode', mode=mode))
        print(f'Setting mode to {mode}')
        return True, 0.0

    if cmd == 'calibrate_depth':
        node._publish(DriverCommand(command='calibrate_depth'))
        print('Calibrating surface depth offset from current reading.')
        return True, 0.0

    # ── Depth commands (depth = ALT_HOLD, ~depth = PID) ─────────────
    # Note: depth commands are firmware/PID controlled — 'just' prefix
    # is not applicable (they don't use the ramp system).
    if cmd == 'depth':
        if is_pid:
            # ~depth — software PID depth hold (auto STABILIZE)
            if args and args[0] == 'off':
                node._publish(DriverCommand(command='pid_depth_off'))
                print('~depth OFF')
                return True, 0.0
            depth_val = 0.0
            if args:
                try:
                    depth_val = float(args[0].rstrip('m'))
                except ValueError:
                    print('Usage: ~depth [meters] | ~depth off')
                    return True, 0.0
            node._publish(DriverCommand(command='set_mode', mode='STABILIZE'))
            if node._publish(DriverCommand(command='pid_depth', depth=depth_val)):
                if depth_val == 0.0:
                    print('~depth ON — holding current depth (STABILIZE)')
                else:
                    print(f'~depth ON at {depth_val}m (STABILIZE)')
            return True, 0.5
        else:
            # depth — ArduSub ALT_HOLD firmware depth
            if not args:
                print('Usage: depth <meters>  (e.g. depth 0.5)')
                return True, 0.0
            try:
                depth = float(args[0].rstrip('m'))
                if node._publish(DriverCommand(command='set_depth', depth=depth)):
                    print(f'Depth {depth}m (ALT_HOLD)')
                return True, 1.0
            except ValueError:
                print('Invalid depth (use float e.g. 0.5)')
                return True, 0.0

    if cmd == 'surface':
        node._publish(DriverCommand(command=_cmd_name('surface')))
        print(f'{_just_label}Surfacing...')
        return True, 2.0

    # ── Turn commands (relative — turn = bang-bang, ~turn = PID) ─────
    if cmd == 'turn':
        if len(args) < 2:
            prefix = '~turn' if is_pid else 'turn'
            print(f'Usage: {prefix} left/right <degrees> [gain%]')
            return True, 0.0
        direction = args[0]
        if direction not in ('left', 'right'):
            prefix = '~turn' if is_pid else 'turn'
            print(f'Usage: {prefix} left/right <degrees> [gain%]')
            return True, 0.0
        try:
            rel_angle = float(args[1])
        except ValueError:
            print('Invalid angle (use degrees e.g. 90)')
            return True, 0.0
        if node._last_state is None:
            print('  [WARNING] No telemetry yet — cannot compute relative turn.')
            print('  Wait for inspector or use absolute heading/~heading.')
            return True, 0.0
        current = node._last_state.yaw
        if direction == 'left':
            target = (current - abs(rel_angle)) % 360
        else:
            target = (current + abs(rel_angle)) % 360
        if is_pid:
            driver_cmd = DriverCommand(command='pid_yaw_to_heading', angle=target, speed=int(gain))
            mode = 'PID'
        else:
            driver_cmd = DriverCommand(command='yaw_to_heading', angle=target, speed=int(gain))
            mode = 'bang-bang'
        if node._publish(driver_cmd):
            print(f'Turn {direction} {rel_angle}° ({mode}: {current:.0f}° → {target:.0f}°)')
        return True, 0.0

    # ── Heading commands (heading = bang-bang, ~heading = PID) ────────
    if cmd == 'heading':
        if not args:
            prefix = '~heading' if is_pid else 'heading'
            print(f'Usage: {prefix} <degrees> [gain%]')
            print(f'       heading left/right [gain%] [Ns]')
            return True, 0.0
        if args[0] == 'left':
            if node._publish(DriverCommand(command=_cmd_name('yaw_left'), duration=duration, speed=int(gain))):
                print(f'{_just_label}Rotate left{f" {gain}%" if gain != node._default_speed else ""}{f" {duration}s" if duration else ""}')
                return True, duration
            return True, 0.0
        elif args[0] == 'right':
            if node._publish(DriverCommand(command=_cmd_name('yaw_right'), duration=duration, speed=int(gain))):
                print(f'{_just_label}Rotate right{f" {gain}%" if gain != node._default_speed else ""}{f" {duration}s" if duration else ""}')
                return True, duration
            return True, 0.0
        else:
            try:
                angle = float(args[0])
                if is_pid:
                    icmd = 'pid_yaw_to_heading'
                    label = 'PID'
                else:
                    icmd = 'yaw_to_heading'
                    label = 'bang-bang'
                if node._publish(DriverCommand(command=icmd, angle=angle, speed=int(gain))):
                    print(f'Heading {angle}° ({label}){f" ({gain}%)" if gain != node._default_speed else ""}')
                return True, 0.0
            except ValueError:
                print('Invalid heading angle')
                return True, 0.0

    if cmd == 'move':
        if not args:
            print('Usage: move <direction> [gain%] [N]s or move depth <m>')
            print('       move at <angle°> [gain%] [Ns]  — body-frame vector')
            return True, 0.0
        direction = args[0].lower()
        if direction == 'at':
            # Body-frame vector: move at <angle> [gain%] [Ns]
            if len(args) < 2:
                print('Usage: move at <angle°> [gain%] [Ns]')
                print('  0°=forward, 90°=right, 180°=back, 270°=left')
                return True, 0.0
            try:
                bearing = float(args[1])
            except ValueError:
                print('Invalid angle (use degrees e.g. 45)')
                return True, 0.0
            c = DriverCommand(command=_cmd_name('move_at'),
                              angle=bearing, duration=duration, speed=int(gain))
            if node._publish(c):
                extra = []
                if gain != node._default_speed:
                    extra.append(f'{gain}%')
                if duration:
                    extra.append(f'{duration}s')
                print(f'{_just_label}Moving at {bearing}°'
                      + (' ' + ' '.join(extra) if extra else '')
                      + (' (indefinite)' if not duration else ''))
                return True, duration
            return True, 0.0
        elif direction == 'depth':
            if len(args) < 2:
                print('Usage: move depth <m>  (same as: depth <m>)')
                return True, 0.0
            try:
                depth_str = args[1].rstrip('m')
                depth = float(depth_str)
                if node._publish(DriverCommand(command='set_depth', depth=depth)):
                    print(f'Depth {depth}m (ALT_HOLD)')
            except ValueError:
                print('Invalid depth (use float e.g. 0.2)')
            return True, 1.0
        elif direction in ('left', 'right', 'forward', 'back', 'backward', 'up', 'down'):
            cmd_map = {
                'left': 'move_left', 'right': 'move_right',
                'forward': 'move_forward', 'back': 'move_back', 'backward': 'move_back',
                'up': 'move_up', 'down': 'move_down',
            }
            c = DriverCommand(command=_cmd_name(cmd_map[direction]), duration=duration, speed=int(gain))
            if node._publish(c):
                extra = []
                if gain != node._default_speed:
                    extra.append(f'{gain}%')
                if duration:
                    extra.append(f'{duration}s')
                print(f'{_just_label}Moving {direction}' + (' ' + ' '.join(extra) if extra else '') + (' (indefinite)' if not duration else ''))
                return True, duration
            return True, 0.0
        else:
            # Try compound direction: forward-right, back-left-up, etc.
            ok, wait = try_compound_move(node, direction, gain, duration, is_just=is_just)
            if ok is not None:
                return True, wait
            print(f'Unknown direction: {direction}')
            return True, 0.0

    if cmd == 'grabber':
        if not args:
            print('Usage: grabber open | grabber close')
            return True, 0.0
        if args[0] == 'open':
            if node._publish(DriverCommand(command='open_grabber')):
                print('Opening grabber')
        elif args[0] == 'close':
            if node._publish(DriverCommand(command='close_grabber')):
                print('Closing grabber')
        else:
            print('Usage: grabber open | grabber close')
        return True, 0.0

    # ── Body-frame vector shorthand: at <angle°> [gain%] [Ns] ────────
    if cmd == 'at':
        if not args:
            print('Usage: at <angle°> [gain%] [Ns]')
            print('  0°=forward, 90°=right, 180°=back, 270°=left')
            return True, 0.0
        try:
            bearing = float(args[0])
        except ValueError:
            print('Invalid angle (use degrees e.g. 45)')
            return True, 0.0
        c = DriverCommand(command=_cmd_name('move_at'),
                          angle=bearing, duration=duration, speed=int(gain))
        if node._publish(c):
            extra = []
            if gain != node._default_speed:
                extra.append(f'{gain}%')
            if duration:
                extra.append(f'{duration}s')
            print(f'{_just_label}Moving at {bearing}°'
                  + (' ' + ' '.join(extra) if extra else '')
                  + (' (indefinite)' if not duration else ''))
            return True, duration
        return True, 0.0

    if cmd in ('left', 'right', 'forward', 'back', 'backward', 'up', 'down'):
        cmd_map = {
            'left': 'move_left', 'right': 'move_right',
            'forward': 'move_forward', 'back': 'move_back', 'backward': 'move_back',
            'up': 'move_up', 'down': 'move_down',
        }
        c = DriverCommand(command=_cmd_name(cmd_map[cmd]), duration=duration, speed=int(gain))
        if node._publish(c):
            extra = []
            if gain != node._default_speed:
                extra.append(f'{gain}%')
            if duration:
                extra.append(f'{duration}s')
            print(f'{_just_label}Moving {cmd}' + (' ' + ' '.join(extra) if extra else '') + (' (indefinite)' if not duration else ''))
            return True, duration
        return True, 0.0

    # ── Compound bare shorthand: forward-right 50% 5s ───────────────
    if '-' in cmd:
        ok, wait = try_compound_move(node, cmd, gain, duration, is_just=is_just)
        if ok is not None:
            return True, wait

    # ── Simultaneous move + heading (go) ─────────────────────────────
    if cmd == 'go':
        if len(args) < 2:
            print('Usage: go <direction> <heading°> [gain%] [Ns]')
            print('  e.g. go forward 90 60% 5s')
            return True, 0.0
        direction = args[0]
        try:
            heading = float(args[1])
        except ValueError:
            print('Invalid heading (use degrees e.g. 90)')
            return True, 0.0
        # Build go command name — single or compound (horizontal only)
        parts = direction.replace('backward', 'back').split('-')
        VALID_DIRS = {'forward', 'back', 'left', 'right'}
        if not all(d in VALID_DIRS for d in parts) or len(parts) > 2:
            print(f'Unknown direction: {direction}. Use: forward, back, left, right (or diagonal e.g. forward-right)')
            return True, 0.0
        go_cmd = DriverCommand(
            command=_cmd_name('go_' + '_'.join(parts)),
            angle=heading,
            duration=duration,
            speed=int(gain),
        )
        if node._publish(go_cmd):
            extra = []
            if gain != node._default_speed:
                extra.append(f'{gain}%')
            if duration:
                extra.append(f'{duration}s')
            print(f'{_just_label}Go {direction} → {heading}°' + (' ' + ' '.join(extra) if extra else ''))
            return True, duration
        return True, 0.0

    # ── Coordinated cruise (move + depth PID + heading PID) ──────────
    if cmd == 'cruise':
        # cruise <bearing°> <heading°> <depth_m> [gain%] [Ns]
        if len(args) < 3:
            print('Usage: cruise <bearing°> <heading°> <depth_m> [gain%] [Ns]')
            print('  e.g. cruise 0 90 0.5 60% 10s')
            return True, 0.0
        try:
            bearing = float(args[0])
            heading = float(args[1])
            depth_val = float(args[2].rstrip('m'))
        except ValueError:
            print('Invalid arguments. Usage: cruise <bearing°> <heading°> <depth_m>')
            return True, 0.0
        cruise_cmd = DriverCommand(
            command=_cmd_name('cruise'),
            angle=bearing,
            mode=str(heading),
            depth=depth_val,
            duration=duration,
            speed=int(gain),
        )
        if node._publish(cruise_cmd):
            extra = []
            if gain != node._default_speed:
                extra.append(f'{gain}%')
            if duration:
                extra.append(f'{duration}s')
            print(f'{_just_label}Cruise bearing={bearing}° heading={heading}° depth={depth_val}m'
                  + (' ' + ' '.join(extra) if extra else ''))
            return True, duration
        return True, 0.0

    # ── Vision alignment commands ────────────────────────────────────
    # lat-align / dep-align / align / align-forward [gain%] [N]s [until]
    # "until" = stop when aligned; without it, run until timer expires (or indefinite)
    # ~ prefix -> PID versions (pid_lat_align, etc.)
    # just-* -> bang-bang, no PID, no Kalman
    _vision_map = {
        'lat-align': 'lat_align',
        'dep-align': 'dep_align',
        'align': 'align',
        'align-forward': 'align_forward',
        'just-lat-align': 'just_lat_align',
        'just-dep-align': 'just_dep_align',
        'just-align': 'just_align',
        'just-align-forward': 'just_align_forward',
    }
    if cmd in _vision_map:
        align_until = 'until' in args
        driver_cmd_name = _vision_map[cmd]
        if not driver_cmd_name.startswith('just_') and is_pid:
            driver_cmd_name = 'pid_' + _vision_map[cmd]
        mode_label = 'PID' if is_pid else ('just (bang-bang)' if 'just' in cmd else 'proportional')
        c = DriverCommand(
            command=driver_cmd_name,
            speed=int(gain),
            duration=duration,
            status='until_aligned' if align_until else '',
        )
        if node._publish(c):
            extra = []
            if gain != node._default_speed:
                extra.append(f'{gain}%')
            if duration > 0:
                extra.append(f'{duration}s')
            if align_until:
                extra.append('until')
            print(f'Vision {cmd} ({mode_label})' + (' ' + ' '.join(extra) if extra else '') + ' -- alignment active')
        return True, duration if duration > 0 else 0.0

    if cmd in ('vision-stop', 'vstop'):
        node._publish(DriverCommand(command='vision_stop'))
        print('Vision alignment stopped.')
        return True, 0.0

    print(f'Unknown command: {line}. Type "help" for commands.')
    return True, 0.0
