"""RC override controller for ArduSub thruster management.

Manages:
- Channel constants and neutral PWM values
- Trapezoidal velocity ramp (smooth accel/decel profiles)
- RC_CHANNELS_OVERRIDE sending
- Helper to build diagonal compound movement channels

Channel mapping (Duburi 4.2 / ArduSub):
  Ch 1: Pitch, Ch 2: Roll, Ch 3: Throttle (depth), Ch 4: Yaw,
  Ch 5: Forward, Ch 6: Lateral
"""

from __future__ import annotations

import math


# ── Channel constants ────────────────────────────────────────────────

CH_PITCH = 1
CH_ROLL = 2
CH_THROTTLE = 3
CH_YAW = 4
CH_FORWARD = 5
CH_LATERAL = 6

NEUTRAL_PWM = 1500
PWM_RANGE = 400  # ±400 from 1500 (1100-1900)

NEUTRAL_CHANNELS = {
    CH_PITCH: NEUTRAL_PWM, CH_ROLL: NEUTRAL_PWM,
    CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM,
    CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM,
}

# Channels that participate in the velocity ramp
RAMP_CHANNELS = (CH_FORWARD, CH_LATERAL, CH_THROTTLE, CH_YAW)

# Direction components for compound diagonal movement
DIRECTION_COMPONENTS = {
    'forward':  (CH_FORWARD, +1),
    'back':     (CH_FORWARD, -1),
    'backward': (CH_FORWARD, -1),
    'left':     (CH_LATERAL, -1),
    'right':    (CH_LATERAL, +1),
}


# ── Utility functions ────────────────────────────────────────────────

def percent_to_pwm(percent: float) -> int:
    """Convert −100..100 percent to absolute PWM (1100-1900, centred at 1500)."""
    percent = max(-100, min(100, percent))
    return int(NEUTRAL_PWM + (percent / 100) * PWM_RANGE)


def build_diagonal_channels(
    parts: list[str], speed_pwm: int,
) -> tuple[dict[int, int], str] | None:
    """Build channel dict for horizontal diagonal movement with √2 scaling.

    Only accepts horizontal directions (forward/back/left/right).
    For 2-axis diagonals, each axis gets speed/√2 ≈ 71 % per channel
    so the resultant vector magnitude equals the requested speed.

    Returns ``(channels_dict, label)`` or *None* if invalid.
    """
    channels_map: dict[int, int] = {}  # ch → sign
    for part in parts:
        if part not in DIRECTION_COMPONENTS:
            return None
        ch, sign = DIRECTION_COMPONENTS[part]
        if ch in channels_map:  # conflicting axes (e.g. forward+back)
            return None
        channels_map[ch] = sign
    if not channels_map or len(channels_map) > 2:
        return None
    n = len(channels_map)
    offset = speed_pwm - NEUTRAL_PWM
    scaled = int(offset / math.sqrt(n)) if n > 1 else offset
    channels: dict[int, int] = {
        CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM,
        CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM,
    }
    for ch, sign in channels_map.items():
        channels[ch] = NEUTRAL_PWM + sign * scaled
    label = '-'.join(parts)
    return channels, label


# ── RC Controller class ─────────────────────────────────────────────

class RcController:
    """Manages RC override with trapezoidal velocity ramping.

    The ramp produces smooth accel/decel profiles instead of instant PWM
    jumps.  ``stop`` bypasses the ramp for immediate safety halt.
    PID layers write directly (inherently smooth).
    """

    def __init__(self, ramp_rate: int = 800):
        self._ramp_rate = ramp_rate               # PWM per second
        self._ramped: dict[int, float] = {}  # ch → actual PWM (float)

    @property
    def ramp_rate(self) -> int:
        return self._ramp_rate

    @ramp_rate.setter
    def ramp_rate(self, value: int):
        self._ramp_rate = value

    # ── Ramp management ──────────────────────────────────────────────

    def apply_movement(
        self,
        channels: dict[int, int],
        movement: dict | None,
        bypass_ramp: bool = False,
    ) -> dict[int, int]:
        """Apply movement targets with optional ramping to *channels*.

        Args:
            channels: base channels dict (start with a copy of NEUTRAL_CHANNELS).
            movement: movement dict with ``'channels'`` key, or *None*.
            bypass_ramp: skip ramp — instant PWM (``just_*`` commands).

        Returns:
            The mutated *channels* dict.
        """
        targets = movement.get('channels', {}) if movement is not None else {}

        if bypass_ramp:
            for ch in RAMP_CHANNELS:
                val = float(targets.get(ch, NEUTRAL_PWM))
                self._ramped[ch] = val
                channels[ch] = int(round(val))
        else:
            dt = 0.05  # 20 Hz tick period
            max_step = self._ramp_rate * dt
            for ch in RAMP_CHANNELS:
                target = float(targets.get(ch, NEUTRAL_PWM))
                current = self._ramped.get(ch, float(NEUTRAL_PWM))
                diff = target - current
                if abs(diff) <= max_step:
                    current = target
                else:
                    current += max_step if diff > 0 else -max_step
                self._ramped[ch] = current
                channels[ch] = int(round(current))

        return channels

    def clear_ramp(self):
        """Reset ramp state — instant neutral for safety stop."""
        self._ramped.clear()

    def snap_channels_neutral(self, *channel_ids: int):
        """Snap specific channels to neutral in ramp state.

        Used by yaw commands to immediately stop forward/lateral thrust
        (POOL FIX 1) without resetting all ramp state.
        """
        for ch in channel_ids:
            self._ramped[ch] = float(NEUTRAL_PWM)

    # ── RC sending ───────────────────────────────────────────────────

    @staticmethod
    def send_rc(
        channels: dict[int, int],
        master,
        logger=None,
    ) -> bool:
        """Send a single RC_CHANNELS_OVERRIDE message.

        Args:
            channels: ``{channel_id (1-18): pwm_value}``.
                Unspecified channels default to 65535 (no change).
                PWM values are clamped to 1100-1900.
            master: pymavlink connection object.
            logger: optional ROS logger for warnings/errors.

        Returns:
            *True* on success, *False* on send failure.
        """
        if master is None:
            return False
        rc = [65535] * 18
        for ch, pwm in channels.items():
            if ch < 1 or ch > 18:
                if logger:
                    logger.warn(f'Invalid RC channel {ch}, skipping')
                continue
            rc[ch - 1] = int(max(1100, min(1900, pwm)))
        try:
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component, *rc
            )
            return True
        except Exception as e:
            if logger:
                logger.error(f'RC override send failed: {e}')
            return False
