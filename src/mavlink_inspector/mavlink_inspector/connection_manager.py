"""Connection manager for the MAVLink serial link to Pixhawk.

Handles:
- Serial port detection (configured + fallback scanning)
- Connection with exponential backoff on failure
- GCS heartbeat sending and health monitoring
- Heartbeat loss detection and automatic reconnection
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil


class ConnectionManager:
    """Manages the MAVLink serial connection to Pixhawk."""

    def __init__(
        self,
        port: str = '/dev/ttyACM0',
        baud: int = 115200,
        heartbeat_timeout: float = 3.0,
        reconnect_backoff: float = 2.0,
        reconnect_max: float = 15.0,
        fallback_ports: list[str] | None = None,
        logger=None,
        on_event=None,
    ):
        self._port = port
        self._baud = baud
        self._heartbeat_timeout = heartbeat_timeout
        self._reconnect_backoff = reconnect_backoff
        self._reconnect_max = reconnect_max
        self._reconnect_current = reconnect_backoff
        self._fallback_ports = fallback_ports or [
            '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3',
            '/dev/ttyUSB0', '/dev/ttyUSB1',
        ]
        self._logger = logger
        self._on_event = on_event or (lambda *_a: None)

        self._master = None
        self._connected = False
        self._reconnecting = False
        self._last_heartbeat = 0.0
        self._heartbeat_lost_notified = False

    # ── Properties ───────────────────────────────────────────────────

    @property
    def master(self):
        """The pymavlink MAVLink connection object (or None)."""
        return self._master

    @property
    def connected(self) -> bool:
        return self._connected

    @connected.setter
    def connected(self, value: bool):
        self._connected = value

    @property
    def port(self) -> str:
        return self._port

    @property
    def last_heartbeat(self) -> float:
        return self._last_heartbeat

    @last_heartbeat.setter
    def last_heartbeat(self, value: float):
        self._last_heartbeat = value

    @property
    def heartbeat_lost_notified(self) -> bool:
        return self._heartbeat_lost_notified

    @heartbeat_lost_notified.setter
    def heartbeat_lost_notified(self, value: bool):
        self._heartbeat_lost_notified = value

    # ── Logging helpers ──────────────────────────────────────────────

    def _log(self, level: str, msg: str):
        if self._logger:
            getattr(self._logger, level)(msg)

    # ── Connection lifecycle ─────────────────────────────────────────

    def close(self):
        """Safely close and null the MAVLink connection."""
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None

    def _find_port(self) -> str | None:
        """Find Pixhawk serial port.  Tries configured port, then fallbacks."""
        if Path(self._port).exists():
            return self._port
        for port in self._fallback_ports:
            if port != self._port and Path(port).exists():
                self._log('info',
                          f'Configured port {self._port} not found, trying {port}')
                return port
        return None

    def connect(self):
        """Connect to Pixhawk (blocking — call from a background thread)."""
        self._reconnecting = True
        try:
            self.close()  # clean up any stale connection

            port = self._find_port()
            if port is None:
                raise ConnectionError(
                    f'No Pixhawk found on {self._port} '
                    f'or fallbacks {self._fallback_ports}'
                )

            self._log('info', f'Connecting to Pixhawk at {port}...')
            self._master = mavutil.mavlink_connection(port, self._baud)
            self._master.wait_heartbeat(timeout=10)

            self._connected = True
            self._last_heartbeat = time.time()
            self._heartbeat_lost_notified = False
            self._reconnect_current = self._reconnect_backoff
            self._port = port

            self._on_event('connected', f'Pixhawk connected on {port}')
            self._log('info', f'Pixhawk connected and active on {port}.')
        except Exception as e:
            self._log('error', f'Failed to connect: {e}')
            self._on_event('connection_failed', str(e))
            self._connected = False
            self.close()
            self._reconnect_current = min(
                self._reconnect_current * 2, self._reconnect_max
            )
        finally:
            self._reconnecting = False

    def start_background(self) -> threading.Thread:
        """Start connection attempt in a background thread."""
        thread = threading.Thread(target=self.connect, daemon=True)
        thread.start()
        return thread

    # ── Heartbeat & health ───────────────────────────────────────────

    def send_heartbeat(self):
        """Send GCS heartbeat and check connection health.

        Call from a 1 Hz timer.  Handles:
        - Triggering reconnect when disconnected (with backoff)
        - Detecting heartbeat loss
        - Marking disconnected after prolonged loss (3× timeout)
        """
        # --- Reconnection when disconnected ---
        if not self._connected:
            if not self._reconnecting:
                delay = self._reconnect_current
                self._log('info',
                          f'Not connected. Reconnecting in {delay:.0f}s...')

                def _delayed_reconnect():
                    time.sleep(delay)
                    if not self._connected and not self._reconnecting:
                        self.connect()

                threading.Thread(
                    target=_delayed_reconnect, daemon=True
                ).start()
            return

        if self._master is None:
            return

        # --- Heartbeat loss detection ---
        now = time.time()
        if (self._last_heartbeat > 0
                and (now - self._last_heartbeat) > self._heartbeat_timeout):
            if not self._heartbeat_lost_notified:
                self._heartbeat_lost_notified = True
                elapsed = now - self._last_heartbeat
                self._on_event(
                    'heartbeat_lost',
                    f'No Pixhawk heartbeat for {elapsed:.1f}s '
                    f'(timeout={self._heartbeat_timeout}s)',
                )
                self._log('warn', 'Pixhawk heartbeat lost!')

            # Too long → mark disconnected to trigger reconnect
            if (now - self._last_heartbeat) > self._heartbeat_timeout * 3:
                self._log('error',
                          'Heartbeat lost too long — marking disconnected '
                          'for reconnect.')
                self._connected = False
                self._on_event('connection_lost',
                               'Heartbeat timeout exceeded, will reconnect')
            return  # don't send heartbeat to a dead link

        # --- Send GCS heartbeat ---
        try:
            self._master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0,
            )
        except Exception as e:
            self._log('warn', f'Heartbeat send failed: {e}')
            self._connected = False

    # ── Message reading ──────────────────────────────────────────────

    def read_messages(self) -> list:
        """Read all pending MAVLink messages (non-blocking).

        Returns a list of messages.  Sets connected=False on read errors.
        """
        if not self._connected or self._master is None:
            return []
        try:
            msgs = []
            msg = self._master.recv_match(blocking=False)
            while msg is not None:
                msgs.append(msg)
                msg = self._master.recv_match(blocking=False)
            return msgs
        except Exception as e:
            self._log('error', f'Read error: {e}')
            self._connected = False
            self._on_event('connection_lost', str(e))
            return []
