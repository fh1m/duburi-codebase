"""
Shared constants and default parameters for the vision package.

All values here are defaults -- every one is overridable via ROS parameters
on the nodes that use them.  Centralising defaults in one place makes it
easy to keep detector, tracker, and alignment controller in sync.
"""

from __future__ import annotations

# ── YOLO Detection ────────────────────────────────────────────────────
DEFAULT_MODEL = "yolo11n.pt"
DEFAULT_CONFIDENCE = 0.5
DEFAULT_IOU = 0.45
DEFAULT_DEVICE = "auto"
DEFAULT_MAX_DET = 50
DEFAULT_TARGET_CLASS = "person"


def resolve_device(requested: str = "auto") -> str:
    """Return the best available inference device.

    ``"auto"`` -> ``"cuda:0"`` if CUDA is available, else ``"cpu"``.
    Any explicit value (``"cpu"``, ``"cuda:0"``, etc.) is passed through.
    """
    if requested.lower() != "auto":
        return requested
    try:
        import torch
        if torch.cuda.is_available():
            return "cuda:0"
    except ImportError:
        pass
    return "cpu"

# ── Alignment / Visual Servo ──────────────────────────────────────────
FRAME_CENTER = 0.5

DEAD_ZONE_X = 0.05          # +/- 5 % of frame width
DEAD_ZONE_Y = 0.05          # +/- 5 % of frame height
DEAD_ZONE_AREA = 0.02       # +/- 2 % of frame area

TARGET_AREA_RATIO = 0.15    # desired object area as fraction of frame

MAX_SPEED = 200              # max PWM offset from 1500 neutral
OUTPUT_DEADBAND = 15.0      # min PWM to send movement; below this = HOLD (reduces jitter)
CONTROL_RATE_HZ = 10.0       # alignment control loop frequency
LOST_TIMEOUT_SEC = 2.0       # seconds without detection before sending stop

# ── PID Defaults (Wescott tuning order: D first, P, then I) ──────────
# Lateral (left / right strafe)
PID_LAT_KP = 300.0
PID_LAT_KI = 5.0
PID_LAT_KD = 60.0

# Vertical (up / down depth adjustment)
PID_VERT_KP = 300.0
PID_VERT_KI = 5.0
PID_VERT_KD = 60.0

# Forward / backward (approach / retreat)
PID_FWD_KP = 250.0
PID_FWD_KI = 3.0
PID_FWD_KD = 50.0

PID_INTEGRAL_MAX = 200.0     # anti-windup clamp (matches MAX_SPEED)
PID_D_FILTER_COEFF = 0.35    # derivative low-pass filter (higher = smoother, less jitter)

# ── Kalman Filter Defaults ────────────────────────────────────────────
KF_PROCESS_NOISE = 1e-3      # Q diagonal -- higher = trust measurements more
KF_MEASUREMENT_NOISE = 5e-3  # R diagonal -- higher = trust predictions more
KF_MAX_DROPOUT_FRAMES = 5    # predict-only frames before declaring target lost

# ── Proportional-only fallback (when PID is disabled) ─────────────────
PROPORTIONAL_GAIN = 400.0    # simple K * error scaling for basic testing
