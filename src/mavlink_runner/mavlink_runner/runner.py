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

import signal
import subprocess
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
from duburi_interfaces.msg import (
    DriverCommand, MavlinkEvent,
    VehicleDiagnostics, VehicleState,
)

from .constants import MISSION_PATHS, HISTORY_FILE
from .command_parser import parse_command


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
        self._armed = False
        self._last_state = None   # type: VehicleState | None
        self._last_diag = None    # type: VehicleDiagnostics | None
        self._last_state_time = 0.0
        self._inspector_warned = False
        self._last_reject_print = 0.0

        self._health_timer = self.create_timer(3.0, self._check_inspector_health)

        self._planner_proc: subprocess.Popen | None = None

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
            now = time.monotonic()
            if now - self._last_reject_print >= 5.0:
                self._last_reject_print = now
                print(f'\r  [WARNING] {msg.description}')

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

    def _publish(self, cmd: DriverCommand) -> bool:
        """Publish command. Returns True if sent, False if rejected (not armed)."""
        c = cmd.command.lower()
        from duburi_common.constants import UNARMED_ALLOWED
        if not self._armed and c not in UNARMED_ALLOWED:
            print(f'  [WARNING] Vehicle not armed! Arm first.')
            return False
        self._cmd_pub.publish(cmd)
        return True

    def _parse_one(self, line: str) -> tuple[bool, float]:
        """Parse and execute one command. Returns (continue, wait_sec)."""
        return parse_command(self, line)

    # ── Planner (YASMIN FSM) mission support ────────────────────────

    PLANNER_MISSIONS = {
        'demo':    ('duburi_planner', 'demo_node',    'Demo square (fwd + 90° turn × 4)'),
        'mission': ('duburi_planner', 'mission_node', 'Full competition mission (gate → ...)'),
    }

    def _planner_list(self):
        """Print available YASMIN planner missions."""
        print('  Available planner missions:')
        print()
        for name, (pkg, exe, desc) in self.PLANNER_MISSIONS.items():
            print(f'    {name:<12s} {desc}')
        print()
        print('  Usage:  planner <name>        Launch mission')
        print('          planner stop          Stop running mission')
        print('          planner viewer        Start YASMIN web viewer')
        print()
        active = self._planner_is_running()
        if active:
            print(f'  ● Planner is running (PID {self._planner_proc.pid})')
        else:
            print('  ○ No planner mission running')

    def _planner_is_running(self) -> bool:
        if self._planner_proc is None:
            return False
        ret = self._planner_proc.poll()
        if ret is not None:
            self._planner_proc = None
            return False
        return True

    def _planner_launch(self, name: str):
        """Launch a YASMIN planner mission as a subprocess."""
        if self._planner_is_running():
            print(f'  Planner already running (PID {self._planner_proc.pid}).')
            print('  Stop it first: planner stop')
            return

        if name not in self.PLANNER_MISSIONS:
            print(f'  Unknown planner mission: {name}')
            print(f'  Available: {", ".join(self.PLANNER_MISSIONS)}')
            return

        pkg, exe, desc = self.PLANNER_MISSIONS[name]
        print(f'  Launching: {desc}')
        print(f'  (ros2 run {pkg} {exe})')
        print(f'  YASMIN Viewer → http://localhost:5000/')
        print()

        try:
            self._planner_proc = subprocess.Popen(
                ['ros2', 'run', pkg, exe],
                stdout=None, stderr=None,
            )
            time.sleep(0.5)
            if self._planner_is_running():
                print(f'  ● Started (PID {self._planner_proc.pid})')
                print('  Use "planner stop" to end the mission.')
            else:
                rc = self._planner_proc.returncode
                print(f'  ✗ Exited immediately (code {rc})')
                self._planner_proc = None
        except FileNotFoundError:
            print('  ✗ Could not find ros2 command. Is ROS 2 sourced?')
        except Exception as e:
            print(f'  ✗ Launch failed: {e}')

    def _planner_stop(self):
        """Stop the running planner mission."""
        if not self._planner_is_running():
            print('  No planner mission running.')
            return

        pid = self._planner_proc.pid
        print(f'  Stopping planner (PID {pid})...')
        self._planner_proc.terminate()
        try:
            self._planner_proc.wait(timeout=5)
            print('  ● Planner stopped.')
        except subprocess.TimeoutExpired:
            print('  Force-killing planner...')
            self._planner_proc.kill()
            self._planner_proc.wait()
            print('  ● Planner killed.')
        self._planner_proc = None

    def _planner_viewer(self):
        """Start the YASMIN viewer node as a background process."""
        print('  Starting YASMIN viewer...')
        try:
            subprocess.Popen(
                ['ros2', 'run', 'yasmin_viewer', 'yasmin_viewer_node'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            time.sleep(1.0)
            print('  ● YASMIN Viewer running → http://localhost:5000/')
        except FileNotFoundError:
            print('  ✗ Could not find ros2 command.')
        except Exception as e:
            print(f'  ✗ Failed: {e}')

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
                elif line == 'planner' or line == 'planner list':
                    self._planner_list()
                elif line.startswith('planner '):
                    planner_args = line[8:].strip().split()
                    pcmd = planner_args[0] if planner_args else ''
                    if pcmd == 'stop':
                        self._planner_stop()
                    elif pcmd == 'viewer':
                        self._planner_viewer()
                    elif pcmd == 'list':
                        self._planner_list()
                    elif pcmd in self.PLANNER_MISSIONS:
                        self._planner_launch(pcmd)
                    else:
                        print(f'  Unknown planner command: {pcmd}')
                        print('  Usage: planner [list|demo|mission|stop|viewer]')
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
        if self._planner_is_running():
            self._planner_stop()
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
