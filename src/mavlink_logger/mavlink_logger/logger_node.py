#!/usr/bin/env python3
"""
MAVLink Logger - Extensive logging for Duburi 4.2 testing.

Subscribes to:
  - /mavlink/events (MavlinkEvent)
  - /mavlink/vehicle_state (VehicleState)
  - /driver/command (DriverCommand)

Logs to <workspace>/logs/<session_datetime>/ with rotating files.

Features:
  - Date-time session folders: logs/2026-03-06_14-30-00/
  - RotatingFileHandler: 5 MB max per file, 3 backups
  - State CSV throttled to avoid huge files
"""

import logging
import logging.handlers
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from duburi_interfaces.msg import DriverCommand, MavlinkEvent, VehicleState


# ── Workspace-relative log directory ─────────────────────────────────────
# Walk up from this file to find the workspace root (dir containing src/).
def _find_workspace_root() -> Path:
    """Find the ROS 2 workspace root by looking for a parent that contains src/."""
    p = Path(__file__).resolve()
    for parent in p.parents:
        if (parent / 'src').is_dir() and (parent / 'build').is_dir():
            return parent
    return Path.cwd()


WORKSPACE_ROOT = _find_workspace_root()
DEFAULT_LOG_BASE = str(WORKSPACE_ROOT / 'logs')

# Rotation defaults (bytes)
MAX_LOG_BYTES = 5 * 1024 * 1024  # 5 MB
BACKUP_COUNT = 3


class MavlinkLoggerNode(Node):
    """Logs all MAVLink-related activity with rotating files in session folders."""

    def __init__(self):
        super().__init__('mavlink_logger')

        log_base = Path(
            self.declare_parameter('log_directory', DEFAULT_LOG_BASE).value
        )
        # Create session subfolder: logs/2026-03-06_14-30-00/
        session_stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self._log_dir = log_base / session_stamp
        self._log_dir.mkdir(parents=True, exist_ok=True)

        max_bytes = int(self.declare_parameter('max_log_bytes', MAX_LOG_BYTES).value)
        backup_count = int(self.declare_parameter('backup_count', BACKUP_COUNT).value)

        # ── Python logging with RotatingFileHandler ──────────────────────
        self._session_logger = self._make_rotating_logger(
            'duburi.session', self._log_dir / 'session.log', max_bytes, backup_count,
            header='# BRACU Duburi 4.2 MAVLink Session Log\n'
                   f'# Started: {self._timestamp()}\n#\n',
        )
        self._events_logger = self._make_rotating_logger(
            'duburi.events', self._log_dir / 'events.log', max_bytes, backup_count,
        )
        self._commands_logger = self._make_rotating_logger(
            'duburi.commands', self._log_dir / 'commands.log', max_bytes, backup_count,
        )

        # State CSV uses simple file + manual rotation (needs CSV header)
        self._state_path = self._log_dir / 'state.csv'
        self._state_file = self._open_state_csv()
        self._state_max_bytes = max_bytes
        self._state_backup_count = backup_count
        self._state_bytes_written = 0

        self.get_logger().info(f'Logging to {self._log_dir}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._event_sub = self.create_subscription(
            MavlinkEvent, '/mavlink/events', self._on_event, qos
        )
        self._state_sub = self.create_subscription(
            VehicleState, '/mavlink/vehicle_state', self._on_state, qos
        )
        self._cmd_sub = self.create_subscription(
            DriverCommand, '/driver/command', self._on_command, qos
        )

        # Throttle state logging (e.g. 1 Hz) to avoid huge files
        self._state_log_interval = self.declare_parameter('state_log_interval', 1.0).value
        self._last_state_log = 0.0

    # ── Helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _timestamp() -> str:
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    @staticmethod
    def _make_rotating_logger(
        name: str, path: Path, max_bytes: int, backup_count: int,
        header: str = '',
    ) -> logging.Logger:
        """Create a named logger with a RotatingFileHandler."""
        logger = logging.getLogger(name)
        logger.setLevel(logging.INFO)
        logger.propagate = False
        handler = logging.handlers.RotatingFileHandler(
            str(path), maxBytes=max_bytes, backupCount=backup_count,
            encoding='utf-8',
        )
        handler.setFormatter(logging.Formatter('%(message)s'))
        logger.addHandler(handler)
        if header:
            logger.info(header.rstrip('\n'))
        return logger

    def _open_state_csv(self):
        """Open (or re-open) state CSV with header."""
        f = open(self._state_path, 'w', encoding='utf-8')
        hdr = 'timestamp,armed,mode,depth,yaw,pitch,roll,voltage,current\n'
        f.write(hdr)
        f.flush()
        self._state_bytes_written = len(hdr)
        return f

    def _rotate_state_csv(self):
        """Manually rotate state CSV when it exceeds max size."""
        self._state_file.close()
        # Shift backups: state.csv.N → delete, .N-1→.N, ... .1→.2
        for i in range(self._state_backup_count, 0, -1):
            target = self._state_path.with_suffix(f'.csv.{i}')
            if i == self._state_backup_count:
                if target.exists():
                    target.unlink()
            else:
                dst = self._state_path.with_suffix(f'.csv.{i + 1}')
                if target.exists():
                    target.rename(dst)
        # Current → .1
        backup1 = self._state_path.with_suffix('.csv.1')
        if self._state_path.exists():
            self._state_path.rename(backup1)
        self._state_file = self._open_state_csv()

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_event(self, msg: MavlinkEvent):
        """Log MAVLink events."""
        line = f"{msg.event_type} | {msg.description}"
        if msg.raw_data:
            line += f" | {msg.raw_data}"
        ts = self._timestamp()
        entry = f'[{ts}] [EVENT] {line}'
        self._session_logger.info(entry)
        self._events_logger.info(entry)

    def _on_state(self, msg: VehicleState):
        """Log vehicle state (throttled)."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_state_log < self._state_log_interval:
            return
        self._last_state_log = now
        row = (
            f"{self._timestamp()},{msg.armed},{msg.flight_mode},"
            f"{msg.depth:.3f},{msg.yaw:.1f},{msg.pitch:.1f},{msg.roll:.1f},"
            f"{msg.voltage:.2f},{msg.current:.2f}\n"
        )
        self._state_file.write(row)
        self._state_file.flush()
        self._state_bytes_written += len(row)
        if self._state_bytes_written >= self._state_max_bytes:
            self._rotate_state_csv()

    def _on_command(self, msg: DriverCommand):
        """Log driver commands."""
        line = (
            f"command={msg.command} mode={msg.mode} depth={msg.depth} "
            f"angle={msg.angle} duration={msg.duration} speed={msg.speed}"
        )
        ts = self._timestamp()
        entry = f'[{ts}] [CMD] {line}'
        self._session_logger.info(entry)
        self._commands_logger.info(entry)

    def destroy_node(self):
        """Close log files on shutdown."""
        try:
            self._state_file.close()
        except Exception:
            pass
        # Shutdown Python loggers
        for name in ('duburi.session', 'duburi.events', 'duburi.commands'):
            lg = logging.getLogger(name)
            for h in lg.handlers[:]:
                h.close()
                lg.removeHandler(h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
