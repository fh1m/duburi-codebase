#!/usr/bin/env python3
"""
Duburi CLI - Interactive prompt for AUV control.

Usage examples:
  Duburi > move left 50% 10s
  Duburi > move depth 0.2
  Duburi > yaw 260 50%
  Duburi > arm; move forward 5s; move left 3s
  Duburi > run gate
  Duburi > help
"""

import os
import re
import signal
import sys
import time
import threading
from pathlib import Path

try:
    import readline  # Enables up/down history, left/right cursor (Unix)
except ImportError:
    pass

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from duburi_interfaces.msg import DriverCommand, MavlinkEvent, VehicleDiagnostics, VehicleState


# Mission file search paths
MISSION_PATHS = [
    Path.cwd() / 'missions',
    Path(__file__).resolve().parent.parent / 'missions',
    Path.home() / '.duburi' / 'missions',
]

HISTORY_FILE = Path.home() / '.duburi_history'


HELP_TEXT = """
Duburi 4.2 AUV Control - Quick Reference
========================================

  Prefix:  bare = ArduSub (firmware)   ~  = PID (software, smooth)

Movement (gain as %%, duration as Ns):
  move left [gain%%] [N]s      e.g. move left 50%% 10s
  move right/forward/back/up/down [gain%%] [N]s
  forward [gain%%] [N]s         Shorthand (no 'move' prefix)

Diagonal Movement:
  move forward-right [gain%%] [N]s     Diagonal (√2 speed scaled)
  forward-right [gain%%] [N]s          Shorthand
  Combos: forward-right, forward-left, back-right, back-left

Depth Control:
  depth <m>            ArduSub ALT_HOLD firmware depth (e.g. depth 0.5)
  ~depth               PID hold current depth (auto STABILIZE)
  ~depth <m>           PID hold specific depth (auto STABILIZE)
  ~depth off           Disable software depth PID
  surface              Ascend to surface

Heading (absolute):
  heading <deg> [gain%%]    Bang-bang yaw (e.g. heading 260 50%%)
  ~heading <deg> [gain%%]   PID smooth yaw (e.g. ~heading 260)
  heading left/right [gain%%] [N]s   Open-loop rotate

Heading (relative — from current heading):
  turn left <deg> [gain%%]    Bang-bang turn (e.g. turn left 90)
  turn right <deg> [gain%%]   Bang-bang turn right
  ~turn left <deg> [gain%%]   PID smooth turn left
  ~turn right <deg> [gain%%]  PID smooth turn right

Simultaneous Move + Heading (go):
  go <dir> <deg> [gain%%] [N]s   Move + PID yaw simultaneously
                                e.g. go forward 90 60%% 5s
  go forward-right 45 60%% 5s   Diagonal + PID heading
                                Dirs: forward, back, left, right
                                (and diagonal combos)

Mode & Arm (non-blocking):
  mode <MODE>          MANUAL, ALT_HOLD, STABILIZE
  arm                  Arm motors
  disarm               Disarm motors

Stop & Actuators:
  stop                 Stop all thrusters + depth PID
  grabber open/close   Grabber control

Chained commands & Missions:
  cmd1; cmd2; cmd3     Run multiple commands (sequential)
  run <mission>        Run mission file from missions/<mission>
  list missions        List available mission files

Backward-compatible aliases:
  dive = depth  |  p_dive = ~depth  |  yaw = heading
  p_yaw = ~heading  |  p_turn = ~turn

Instant (no-ramp) fallbacks:
  just forward [gain%%] [N]s     Raw bang-bang (no accel/decel ramp)
  just move left [gain%%] [N]s   Same as 'just left', with 'move' prefix
  just forward-right [gain%%]    Instant diagonal
  just heading left [gain%%]     Instant open-loop yaw
  just go forward 90 60%%        Instant movement + PID heading
  just surface                  Instant surface throttle
  (Prefix any movement command with 'just' to bypass ramp)

Other:
  help                 Show this help
  status               Vehicle status
  quit / exit          Exit
"""


class DuburiRunnerNode(Node):
    """CLI runner node - parses user input and publishes DriverCommand."""

    def __init__(self):
        super().__init__('mavlink_runner')
        self._cmd_pub = self.create_publisher(
            DriverCommand, '/driver/command',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        self._event_sub = self.create_subscription(
            MavlinkEvent, '/mavlink/events', self._on_event,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        self._state_sub = self.create_subscription(
            VehicleState, '/mavlink/vehicle_state', self._on_state,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        )
        self._diag_sub = self.create_subscription(
            VehicleDiagnostics, '/mavlink/diagnostics', self._on_diag,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        )
        self._default_speed = 50
        self._armed = False  # Track vehicle arm state
        self._last_state = None   # type: VehicleState | None
        self._last_diag = None    # type: VehicleDiagnostics | None
        self._last_state_time = 0.0  # monotonic timestamp of last VehicleState
        self._inspector_warned = False  # avoid spamming warnings

        # Health monitor: check every 3 s if inspector is alive
        self._health_timer = self.create_timer(3.0, self._check_inspector_health)

    def _on_event(self, msg: MavlinkEvent):
        """Print arm/disarm and rejection events (non-blocking).

        Command ACK feedback is shown in the inspector log (ros2 topic echo
        /mavlink/events or watch the inspector terminal).  The runner CLI
        only shows events that directly affect the operator's workflow.
        """
        etype = msg.event_type
        if etype in ('armed', 'disarmed', 'arm_failed', 'disarm_failed'):
            label = {'armed': 'Armed.', 'disarmed': 'Disarmed.',
                     'arm_failed': 'Arm failed.', 'disarm_failed': 'Disarm failed.'}.get(etype, etype)
            print(f'\r  [{label}]')
            if etype == 'armed':
                self._armed = True
            elif etype == 'disarmed':
                self._armed = False
        elif etype == 'command_rejected':
            print(f'\r  [WARNING] {msg.description}. Arm first!')

    def _on_state(self, msg: VehicleState):
        """Track vehicle arm state from telemetry."""
        self._armed = msg.armed
        self._last_state = msg
        self._last_state_time = time.monotonic()
        if self._inspector_warned:
            self._inspector_warned = False
            print('\r  [OK] Inspector connection restored.')

    def _on_diag(self, msg: VehicleDiagnostics):
        """Store latest diagnostics for status display."""
        self._last_diag = msg

    def _check_inspector_health(self):
        """Warn if no VehicleState received for >5 seconds."""
        if self._last_state_time == 0.0:
            # Never received — skip until first message arrives (or 10 s)
            return
        elapsed = time.monotonic() - self._last_state_time
        if elapsed > 5.0 and not self._inspector_warned:
            self._inspector_warned = True
            print(f'\r  \033[93m⚠ No telemetry for {elapsed:.0f}s — '
                  f'is mavlink_inspector running?\033[0m')

    def _print_status(self):
        """Print a human-friendly status dashboard."""
        s = self._last_state
        d = self._last_diag
        if s is None:
            print('  No telemetry received yet. Is mavlink_inspector running?')
            return

        G = '\033[92m'  # green
        DIM = '\033[90m'  # dim grey
        R = '\033[0m'    # reset

        arm_label = f'{G}ARMED{R}' if s.armed else f'{DIM}DISARMED{R}'
        depth_str = f'{abs(s.depth):.2f}m' if s.depth != 0.0 else '0.00m (sfc)'
        compass = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        heading_arrow = compass[int((s.yaw + 22.5) % 360 / 45)]

        # Battery bar: 16.8V=full, 14.0V=empty for 4S LiPo
        batt_pct = max(0, min(100, (s.voltage - 14.0) / (16.8 - 14.0) * 100)) if s.voltage > 0 else 0
        n = int(batt_pct / 10)
        batt_bar = f'{G}{"█" * n}{DIM}{"░" * (10 - n)}{R}'

        def row(text: str):
            """Print a row padded to fixed width (50 visible chars)."""
            # Strip ANSI to measure visible length
            vis = re.sub(r'\033\[[0-9;]*m', '', text)
            pad = 50 - len(vis)
            if pad < 0:
                pad = 0
            print(f'  │{text}{" " * pad}│')

        sep = '─' * 50
        print(f'  ┌{sep}┐')
        row(f'{"Duburi AUV Status":^50}')
        print(f'  ├{sep}┤')
        row(f'  Motors: {arm_label}    Mode: {s.flight_mode}')
        row(f'  Depth:  {depth_str:<12s}Heading: {s.yaw:5.1f}° {heading_arrow}')
        row(f'  Pitch:  {s.pitch:+6.1f}°       Roll:    {s.roll:+6.1f}°')
        if d:
            row(f'  Yaw rate: {d.heading_rate:+.1f}°/s')
        print(f'  ├{sep}┤')
        row(f'  Battery: {s.voltage:.1f}V  {s.current:.1f}A  {batt_bar} {batt_pct:.0f}%')
        if d:
            row(f'  Pressure: {d.pressure:.0f} hPa   Temp: {d.temperature:.1f}°C')
            row(f'  CPU load: {d.cpu_load:.0f}%')
        print(f'  ├{sep}┤')
        if d:
            servos = ' '.join(f'{v:4d}' for v in d.servo_output)
            rc_vals = ' '.join(f'{v:4d}' for v in d.rc_channels)
            row(f'  Servos: {servos}')
            row(f'  RC in:  {rc_vals}')
        else:
            row('  (diagnostics not yet received)')
        # Inspector health
        if self._last_state_time > 0:
            age = time.monotonic() - self._last_state_time
            if age > 5.0:
                Y = '\033[93m'
                row(f'  {Y}⚠ Telemetry stale ({age:.0f}s ago){R}')
        print(f'  └{sep}┘')

    def _publish(self, cmd: DriverCommand) -> bool:
        """Publish command. Returns True if sent, False if rejected (not armed)."""
        c = cmd.command.lower()
        UNARMED_ALLOWED = {'arm', 'disarm', 'set_mode', 'stop', 'pid_depth_off', 'surface', 'just_surface'}
        if not self._armed and c not in UNARMED_ALLOWED:
            print(f'  [WARNING] Vehicle not armed! Arm first.')
            return False
        self._cmd_pub.publish(cmd)
        return True

    def _parse_one(self, line: str) -> tuple[bool, float]:
        """Parse and execute one command. Returns (continue, wait_sec)."""
        line = line.strip().lower()
        if not line:
            return True, 0.0

        # Extract duration and gain (don't strip from depth values)
        duration = 0.0
        gain = self._default_speed
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

        if cmd in ('quit', 'exit', 'q'):
            return False, 0.0

        if cmd == 'help':
            print(HELP_TEXT)
            return True, 0.0

        if cmd == 'status':
            self._print_status()
            return True, 0.0

        if cmd == 'list' and args and args[0] == 'missions':
            self._list_missions()
            return True, 0.0

        if cmd == 'stop':
            self._publish(DriverCommand(command='stop'))
            print('Stopping all thrusters.')
            return True, 0.0

        # Helper: prepend 'just_' to command name if is_just is set
        def _cmd_name(name: str) -> str:
            return f'just_{name}' if is_just else name

        _just_label = '[JUST] ' if is_just else ''

        if cmd == 'arm':
            print('Arming...')
            self._publish(DriverCommand(command='arm'))
            return True, 4.0  # Wait for vehicle to arm before next command

        if cmd == 'disarm':
            print('Disarming...')
            self._publish(DriverCommand(command='disarm'))
            return True, 2.0  # Wait for disarm to complete

        if cmd == 'mode':
            mode = args[0].upper().replace('MANNUAL', 'MANUAL') if args else 'MANUAL'
            self._publish(DriverCommand(command='set_mode', mode=mode))
            print(f'Setting mode to {mode}')
            return True, 0.0

        # ── Depth commands (depth = ALT_HOLD, ~depth = PID) ─────────────
        # Note: depth commands are firmware/PID controlled — 'just' prefix
        # is not applicable (they don't use the ramp system).
        if cmd == 'depth':
            if is_pid:
                # ~depth — software PID depth hold (auto STABILIZE)
                if args and args[0] == 'off':
                    self._publish(DriverCommand(command='pid_depth_off'))
                    print('~depth OFF')
                    return True, 0.0
                depth_val = 0.0
                if args:
                    try:
                        depth_val = float(args[0].rstrip('m'))
                    except ValueError:
                        print('Usage: ~depth [meters] | ~depth off')
                        return True, 0.0
                self._publish(DriverCommand(command='set_mode', mode='STABILIZE'))
                if self._publish(DriverCommand(command='pid_depth', depth=depth_val)):
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
                    if self._publish(DriverCommand(command='set_depth', depth=depth)):
                        print(f'Depth {depth}m (ALT_HOLD)')
                    return True, 1.0
                except ValueError:
                    print('Invalid depth (use float e.g. 0.5)')
                    return True, 0.0

        if cmd == 'surface':
            self._publish(DriverCommand(command=_cmd_name('surface')))
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
            if self._last_state is None:
                print('  [WARNING] No telemetry yet — cannot compute relative turn.')
                print('  Wait for inspector or use absolute heading/~heading.')
                return True, 0.0
            current = self._last_state.yaw
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
            if self._publish(driver_cmd):
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
                if self._publish(DriverCommand(command=_cmd_name('yaw_left'), duration=duration, speed=int(gain))):
                    print(f'{_just_label}Rotate left{f" {gain}%" if gain != self._default_speed else ""}{f" {duration}s" if duration else ""}')
                    return True, duration
                return True, 0.0
            elif args[0] == 'right':
                if self._publish(DriverCommand(command=_cmd_name('yaw_right'), duration=duration, speed=int(gain))):
                    print(f'{_just_label}Rotate right{f" {gain}%" if gain != self._default_speed else ""}{f" {duration}s" if duration else ""}')
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
                    if self._publish(DriverCommand(command=icmd, angle=angle, speed=int(gain))):
                        print(f'Heading {angle}° ({label}){f" ({gain}%)" if gain != self._default_speed else ""}')
                    return True, 0.0
                except ValueError:
                    print('Invalid heading angle')
                    return True, 0.0

        if cmd == 'move':
            if not args:
                print('Usage: move <direction> [gain%] [N]s or move depth <m>')
                return True, 0.0
            direction = args[0].lower()
            if direction == 'depth':
                if len(args) < 2:
                    print('Usage: move depth <m>  (same as: depth <m>)')
                    return True, 0.0
                try:
                    depth_str = args[1].rstrip('m')
                    depth = float(depth_str)
                    if self._publish(DriverCommand(command='set_depth', depth=depth)):
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
                if self._publish(c):
                    extra = []
                    if gain != self._default_speed:
                        extra.append(f'{gain}%')
                    if duration:
                        extra.append(f'{duration}s')
                    print(f'{_just_label}Moving {direction}' + (' ' + ' '.join(extra) if extra else '') + (' (indefinite)' if not duration else ''))
                    return True, duration
                return True, 0.0
            else:
                # Try compound direction: forward-right, back-left-up, etc.
                ok, wait = self._try_compound_move(direction, gain, duration, is_just=is_just)
                if ok is not None:
                    return True, wait
                print(f'Unknown direction: {direction}')
                return True, 0.0

        if cmd == 'grabber':
            if not args:
                print('Usage: grabber open | grabber close')
                return True, 0.0
            if args[0] == 'open':
                if self._publish(DriverCommand(command='open_grabber')):
                    print('Opening grabber')
            elif args[0] == 'close':
                if self._publish(DriverCommand(command='close_grabber')):
                    print('Closing grabber')
            else:
                print('Usage: grabber open | grabber close')
            return True, 0.0

        if cmd in ('left', 'right', 'forward', 'back', 'backward', 'up', 'down'):
            cmd_map = {
                'left': 'move_left', 'right': 'move_right',
                'forward': 'move_forward', 'back': 'move_back', 'backward': 'move_back',
                'up': 'move_up', 'down': 'move_down',
            }
            c = DriverCommand(command=_cmd_name(cmd_map[cmd]), duration=duration, speed=int(gain))
            if self._publish(c):
                extra = []
                if gain != self._default_speed:
                    extra.append(f'{gain}%')
                if duration:
                    extra.append(f'{duration}s')
                print(f'{_just_label}Moving {cmd}' + (' ' + ' '.join(extra) if extra else '') + (' (indefinite)' if not duration else ''))
                return True, duration
            return True, 0.0

        # ── Compound bare shorthand: forward-right 50% 5s ───────────────
        if '-' in cmd:
            ok, wait = self._try_compound_move(cmd, gain, duration, is_just=is_just)
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
                command='go_' + '_'.join(parts),
                angle=heading,
                duration=duration,
                speed=int(gain),
            )
            if self._publish(go_cmd):
                extra = []
                if gain != self._default_speed:
                    extra.append(f'{gain}%')
                if duration:
                    extra.append(f'{duration}s')
                print(f'Go {direction} → {heading}°' + (' ' + ' '.join(extra) if extra else ''))
                return True, duration
            return True, 0.0

        print(f'Unknown command: {line}. Type "help" for commands.')
        return True, 0.0

    # ── Compound direction helper ────────────────────────────────────────
    _HORIZONTAL_DIRS = {'forward', 'back', 'backward', 'left', 'right'}

    def _try_compound_move(self, direction: str, gain: float, duration: float,
                           is_just: bool = False
                           ) -> tuple[bool, float] | tuple[None, float]:
        """Try to parse a hyphenated compound direction (e.g. forward-right).

        Only horizontal directions are supported (forward/back + left/right).
        If is_just=True, prepends 'just_' to the command name for no-ramp.
        Returns (True/False, wait_seconds) on valid compound, or (None, 0) if
        the direction string is not a valid compound.
        """
        parts = direction.replace('backward', 'back').split('-')
        if len(parts) != 2 or not all(d in self._HORIZONTAL_DIRS for d in parts):
            return None, 0.0
        cmd_name = 'move_' + '_'.join(p.replace('backward', 'back') for p in parts)
        if is_just:
            cmd_name = f'just_{cmd_name}'
        c = DriverCommand(command=cmd_name, duration=duration, speed=int(gain))
        if self._publish(c):
            extra = []
            if gain != self._default_speed:
                extra.append(f'{gain}%')
            if duration:
                extra.append(f'{duration}s')
            label = '[JUST] ' if is_just else ''
            print(f'{label}Moving {direction}' + (' ' + ' '.join(extra) if extra else '')
                  + (' (indefinite)' if not duration else ''))
            return True, duration
        return True, 0.0

    def _list_missions(self):
        """List available mission files."""
        found = set()
        for base in MISSION_PATHS:
            if base.is_dir():
                for f in base.iterdir():
                    if f.is_file() and not f.name.startswith('.'):
                        found.add(f.stem)
        if found:
            for name in sorted(found):
                print(f'  {name}')
        else:
            print('No missions found. Create missions/ folder with .txt files.')

    def _find_mission(self, name: str) -> Path | None:
        """Find mission file by name."""
        for base in MISSION_PATHS:
            for p in [base / name, base / f'{name}.txt']:
                if p.is_file():
                    return p
        return None

    def _execute_chain(self, text: str) -> bool:
        """Execute chained commands (semicolon-separated). Returns False to exit."""
        parts = [p.strip() for p in text.split(';') if p.strip()]
        for part in parts:
            cont, wait_sec = self._parse_one(part)
            if not cont:
                return False
            if wait_sec > 0:
                time.sleep(wait_sec)
        return True

    def _run_mission(self, name: str) -> bool:
        """Run mission file. Returns False to exit."""
        path = self._find_mission(name)
        if not path:
            print(f'Mission not found: {name}')
            return True
        print(f'Running mission: {path.name}')
        with open(path) as f:
            for num, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                print(f'  [{num}] {line}')
                # Handle sleep/wait/pause directly (not standard runner commands)
                parts = line.split()
                lcmd = parts[0].lower()
                if lcmd in ('sleep', 'wait'):
                    secs = float(parts[1]) if len(parts) > 1 else 1.0
                    time.sleep(secs)
                    continue
                if lcmd == 'pause':
                    input('  ⏸  Mission paused. Press Enter to resume...')
                    continue
                if not self._execute_chain(line):
                    return False
        print('Mission complete.')
        return True

    def run_interactive(self):
        """Run the interactive prompt loop."""
        try:
            readline.read_history_file(str(HISTORY_FILE))
        except (FileNotFoundError, OSError, AttributeError, NameError):
            pass

        print('BRACU Duburi 4.2 AUV Control')
        print('Type "help" for commands, "quit" to exit. Up/Down: history, Left/Right: cursor.')
        print()
        while rclpy.ok():
            try:
                line = input('Duburi > ')
                line = line.strip()
                if not line:
                    continue
                if line.startswith('run '):
                    name = line[4:].strip()
                    if name:
                        if not self._run_mission(name):
                            break
                    else:
                        print('Usage: run <mission_name>')
                else:
                    if not self._execute_chain(line):
                        break
            except EOFError:
                break
            except KeyboardInterrupt:
                print('\nInterrupted — shutting down.')
                self._safe_disarm()
                break
            except Exception as e:
                print(f'Error: {e}')
        try:
            readline.write_history_file(str(HISTORY_FILE))
        except (AttributeError, OSError, NameError):
            pass
        print('Goodbye.')

    def _safe_disarm(self):
        """Send stop + disarm directly, ignoring arm-check guard."""
        try:
            if not rclpy.ok():
                return
            self._cmd_pub.publish(DriverCommand(command='stop'))
            time.sleep(0.3)
            if self._armed:
                print('Vehicle armed — sending disarm...')
                self._cmd_pub.publish(DriverCommand(command='disarm'))
                time.sleep(1.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DuburiRunnerNode()
    # Spin executor in background thread so subscriptions fire while input() blocks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    # Block SIGINT during cleanup so disarm isn't interrupted
    original_sigint = signal.getsignal(signal.SIGINT)
    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ignore further Ctrl-C
        node._safe_disarm()
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
