"""
Shared constants for the Duburi AUV 4.2 stack.

Single source of truth for mission paths, timing, speed defaults,
network configuration, and command allow-lists. All packages import from here.
"""

from pathlib import Path

# ── Mission file search paths ────────────────────────────────────────
MISSION_PATHS = [
    Path.cwd() / 'missions',
    Path.home() / '.duburi' / 'missions',
]

HISTORY_FILE = Path.home() / '.duburi_history'

# ── Network Configuration ────────────────────────────────────────────
# BlueOS (Raspberry Pi 4B) running MAVLink router
BLUEOS_HOST = '192.168.2.2'

# Jetson Orin Nano (ROS 2 companion computer)
JETSON_HOST = '192.168.2.69'

# This PC / GCS
GCS_HOST = '192.168.2.1'

# MAVLink ports
MAVLINK_UDP_PORT = 14550  # Standard MAVLink UDP port

# BlueOS service ports
BLUEOS_PORT_HELPER = 81
BLUEOS_PORT_SYSTEM_INFO = 6030
BLUEOS_PORT_MAVLINK2REST = 6040
BLUEOS_PORT_ARDUPILOT_MANAGER = 8000
BLUEOS_PORT_CABLE_GUY = 9090

# Default UDP endpoints to try when auto-connecting
# Format: (description, pymavlink_url)
DEFAULT_UDP_ENDPOINTS = [
    ('BlueOS UDP (listen)', f'udpin:0.0.0.0:{MAVLINK_UDP_PORT}'),
    ('BlueOS UDP (localhost)', f'udpin:127.0.0.1:{MAVLINK_UDP_PORT}'),
]

# Default serial ports to try (existing behavior)
DEFAULT_SERIAL_PORTS = [
    '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3',
    '/dev/ttyUSB0', '/dev/ttyUSB1',
]

# ── Speed / timing defaults ──────────────────────────────────────────
DEFAULT_SPEED_PERCENT = 50  # Default thrust percentage (0-100%)
DEFAULT_SPEED = DEFAULT_SPEED_PERCENT  # Backwards compatibility
ARM_WAIT = 4.0
DISARM_WAIT = 2.0
SURFACE_WAIT = 5.0

# ── Commands allowed when the vehicle is disarmed ────────────────────
# Runner uses this set directly; inspector may extend with extras.
UNARMED_ALLOWED = frozenset({
    'arm', 'disarm', 'set_mode', 'stop', 'pid_depth_off',
    'surface', 'just_surface', 'calibrate_depth',
    'vision_stop',
})

# Inspector-specific additions (teleop_idle + vision alignment passthrough)
UNARMED_ALLOWED_INSPECTOR = UNARMED_ALLOWED | frozenset({
    'teleop_idle',
    'lat_align', 'dep_align', 'align', 'align_forward',
    'pid_lat_align', 'pid_dep_align', 'pid_align', 'pid_align_forward',
})
