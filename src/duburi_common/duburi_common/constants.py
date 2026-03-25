"""
Shared constants for the Duburi AUV 4.2 stack.

Single source of truth for mission paths, timing, speed defaults,
and command allow-lists. All packages import from here.
"""

from pathlib import Path

# ── Mission file search paths ────────────────────────────────────────
MISSION_PATHS = [
    Path.cwd() / 'missions',
    Path.home() / '.duburi' / 'missions',
]

HISTORY_FILE = Path.home() / '.duburi_history'

# ── Speed / timing defaults ──────────────────────────────────────────
DEFAULT_SPEED = 50
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
