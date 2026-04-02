"""Connection manager for the MAVLink link to Pixhawk or SITL.

Handles:
- Serial port detection (configured + fallback scanning) for USB Pixhawk
- Direct TCP/UDP URLs for ArduPilot SITL (e.g. ``tcp:127.0.0.1:5760``)
- BlueOS UDP endpoint auto-detection (udpin:0.0.0.0:14550)
- Connection with exponential backoff on failure
- GCS heartbeat sending and health monitoring
- Heartbeat loss detection and automatic reconnection
"""

from __future__ import annotations

import os
import socket
import threading
import time
from pathlib import Path

os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil

# Import network config from duburi_common if available
try:
    from duburi_common.constants import (
        DEFAULT_UDP_ENDPOINTS,
        DEFAULT_SERIAL_PORTS,
        MAVLINK_UDP_PORT,
    )
except ImportError:
    # Fallback defaults if duburi_common not available
    MAVLINK_UDP_PORT = 14550
    DEFAULT_UDP_ENDPOINTS = [
        ('BlueOS UDP (listen)', f'udpin:0.0.0.0:{MAVLINK_UDP_PORT}'),
    ]
    DEFAULT_SERIAL_PORTS = [
        '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3',
        '/dev/ttyUSB0', '/dev/ttyUSB1',
    ]


def _is_network_mavlink_url(port: str) -> bool:
    """True if *port* is a pymavlink network-style connection string (SITL, UDP relay)."""
    p = port.strip().lower()
    return p.startswith(
        (
            'tcp:',
            'tcpin:',
            'tcpout:',
            'udp:',
            'udpin:',
            'udpout:',
            'udpbcast:',
        ),
    )


def _check_udp_port_available(port: int, timeout: float = 0.5) -> bool:
    """Check if a UDP port can receive data (non-blocking test)."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', port))
        sock.close()
        return True
    except (OSError, socket.error):
        return False


class ConnectionManager:
    """Manages the MAVLink connection to Pixhawk (serial) or SITL/BlueOS (tcp/udp)."""

    def __init__(
        self,
        port: str = '/dev/ttyACM0',
        baud: int = 115200,
        heartbeat_timeout: float = 3.0,
        reconnect_backoff: float = 2.0,
        reconnect_max: float = 15.0,
        fallback_ports: list[str] | None = None,
        fallback_udp_endpoints: list[tuple[str, str]] | None = None,
        auto_search_udp: bool = True,
        logger=None,
        on_event=None,
    ):
        self._port = port
        self._baud = baud
        self._heartbeat_timeout = heartbeat_timeout
        self._reconnect_backoff = reconnect_backoff
        self._reconnect_max = reconnect_max
        self._reconnect_current = reconnect_backoff
        self._fallback_ports = fallback_ports or list(DEFAULT_SERIAL_PORTS)
        self._fallback_udp_endpoints = fallback_udp_endpoints or list(DEFAULT_UDP_ENDPOINTS)
        self._auto_search_udp = auto_search_udp
        self._logger = logger
        self._on_event = on_event or (lambda *_a: None)

        self._master = None
        self._connected = False
        self._reconnecting = False
        self._last_heartbeat = 0.0
        self._heartbeat_lost_notified = False

        # Thread safety lock for shared state (_master, _connected, _reconnecting)
        self._lock = threading.Lock()

    # ── Properties ───────────────────────────────────────────────────

    @property
    def master(self):
        """The pymavlink MAVLink connection object (or None)."""
        with self._lock:
            return self._master

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @connected.setter
    def connected(self, value: bool):
        with self._lock:
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

    # ── Logging ──────────────────────────────────────────────────────
    # Note: rclpy logger caches severity per caller; using getattr(logger, level)()
    # triggers "Logger severity cannot be changed between calls" when level varies.
    # Call logger.info/warn/error directly at each site so each caller has fixed severity.

    # ── Connection lifecycle ─────────────────────────────────────────

    def close(self):
        """Safely close and null the MAVLink connection."""
        with self._lock:
            if self._master is not None:
                try:
                    self._master.close()
                except Exception as e:
                    if self._logger:
                        self._logger.warning(f'Close connection failed: {e}')
                self._master = None

    def _find_port(self) -> str | None:
        """Resolve connection endpoint: network (tcp/udp), serial, or BlueOS UDP.

        Search order:
        1. If configured port is a network URL, use it directly
        2. If configured port is an existing serial device, use it
        3. Search fallback serial ports
        4. If auto_search_udp is enabled, try UDP endpoints (BlueOS)
        """
        # 1. Network URL (SITL, BlueOS, etc.) - use directly
        if _is_network_mavlink_url(self._port):
            return self._port

        # 2. Configured serial port exists
        if Path(self._port).exists():
            return self._port

        # 3. Search fallback serial ports
        for port in self._fallback_ports:
            if port != self._port and Path(port).exists():
                if self._logger:
                    self._logger.info(
                        f'Configured port {self._port} not found, trying {port}'
                    )
                return port

        # 4. Auto-search UDP endpoints (BlueOS network connection)
        if self._auto_search_udp:
            if self._logger:
                self._logger.info(
                    f'No serial ports found, searching UDP endpoints...'
                )
            for desc, udp_url in self._fallback_udp_endpoints:
                if self._logger:
                    self._logger.info(f'Trying {desc}: {udp_url}')
                # For udpin, we can connect and see if we get data
                # Return the URL and let connect() verify with wait_heartbeat
                return udp_url

        return None

    def connect(self):
        """Connect to Pixhawk (blocking — call from a background thread).

        Searches serial ports and UDP endpoints automatically.
        For BlueOS setup, will try udpin:0.0.0.0:14550 if no serial found.
        """
        with self._lock:
            self._reconnecting = True
        try:
            self.close()  # clean up any stale connection

            port = self._find_port()
            if port is None:
                udp_hint = ''
                if self._auto_search_udp:
                    udp_hint = f' or UDP endpoints {[u[1] for u in self._fallback_udp_endpoints]}'
                raise ConnectionError(
                    f'No MAVLink source found on {self._port}, '
                    f'serial fallbacks {self._fallback_ports}{udp_hint}'
                )

            if self._logger:
                if _is_network_mavlink_url(port):
                    label = 'BlueOS/network' if 'udpin' in port else 'SITL/network'
                else:
                    label = 'Pixhawk (serial)'
                self._logger.info(f'Connecting to {label} at {port}...')

            master = mavutil.mavlink_connection(port, self._baud)
            master.wait_heartbeat(timeout=10)

            with self._lock:
                self._master = master
                self._connected = True
                self._last_heartbeat = time.time()
                self._heartbeat_lost_notified = False
                self._reconnect_current = self._reconnect_backoff
                self._port = port

            self._on_event('connected', f'MAVLink connected on {port}')
            if self._logger:
                self._logger.info(f'MAVLink connected and active on {port}.')
        except Exception as e:
            if self._logger:
                self._logger.error(f'Failed to connect: {e}')
            self._on_event('connection_failed', str(e))
            with self._lock:
                self._connected = False
            self.close()
            self._reconnect_current = min(
                self._reconnect_current * 2, self._reconnect_max
            )
        finally:
            with self._lock:
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
        with self._lock:
            connected = self._connected
            reconnecting = self._reconnecting

        if not connected:
            if not reconnecting:
                delay = self._reconnect_current
                if self._logger:
                    self._logger.info(
                        f'Not connected. Reconnecting in {delay:.0f}s...'
                    )

                def _delayed_reconnect():
                    time.sleep(delay)
                    with self._lock:
                        should_connect = not self._connected and not self._reconnecting
                    if should_connect:
                        self.connect()

                threading.Thread(
                    target=_delayed_reconnect, daemon=True
                ).start()
            return

        with self._lock:
            master = self._master
        if master is None:
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
                if self._logger:
                    self._logger.warning('Pixhawk heartbeat lost!')

            # Too long → mark disconnected to trigger reconnect
            if (now - self._last_heartbeat) > self._heartbeat_timeout * 3:
                if self._logger:
                    self._logger.error(
                        'Heartbeat lost too long — marking disconnected '
                        'for reconnect.'
                    )
                with self._lock:
                    self._connected = False
                self._on_event('connection_lost',
                               'Heartbeat timeout exceeded, will reconnect')
            return  # don't send heartbeat to a dead link

        # --- Send GCS heartbeat ---
        try:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0,
            )
        except Exception as e:
            if self._logger:
                self._logger.warning(f'Heartbeat send failed: {e}')
            with self._lock:
                self._connected = False

    # ── Message reading ──────────────────────────────────────────────

    def read_messages(self) -> list:
        """Read all pending MAVLink messages (non-blocking).

        Returns a list of messages.  Sets connected=False on read errors.
        On mid-batch failure, returns messages already collected (preserving
        partial telemetry rather than discarding it).
        """
        with self._lock:
            connected = self._connected
            master = self._master
        if not connected or master is None:
            return []
        msgs = []
        try:
            msg = master.recv_match(blocking=False)
            while msg is not None:
                msgs.append(msg)
                msg = master.recv_match(blocking=False)
        except Exception as e:
            if self._logger:
                self._logger.error(f'Read error: {e}')
            with self._lock:
                self._connected = False
            self._on_event('connection_lost', str(e))
        return msgs
