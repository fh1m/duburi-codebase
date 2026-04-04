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
import threading
import time


# ── Channel constants ────────────────────────────────────────────────

CH_PITCH = 1
CH_ROLL = 2
CH_THROTTLE = 3
CH_YAW = 4
CH_FORWARD = 5
CH_LATERAL = 6

# PWM Configuration - can be overridden via parameters
NEUTRAL_PWM = 1500
PWM_RANGE = 400  # ±400 from 1500 (1100-1900)

def set_pwm_config(neutral: int, pwm_range: int):
    """Update PWM configuration (called by inspector_node during init)."""
    global NEUTRAL_PWM, PWM_RANGE
    NEUTRAL_PWM = neutral
    PWM_RANGE = pwm_range
    # Update NEUTRAL_CHANNELS dict with new neutral value
    global NEUTRAL_CHANNELS
    NEUTRAL_CHANNELS = {
        CH_PITCH: NEUTRAL_PWM, CH_ROLL: NEUTRAL_PWM,
        CH_THROTTLE: NEUTRAL_PWM, CH_YAW: NEUTRAL_PWM,
        CH_FORWARD: NEUTRAL_PWM, CH_LATERAL: NEUTRAL_PWM,
    }

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

    Phase-aware movement control:
    - ramping_up: accelerating toward target speed
    - cruising: maintaining target speed
    - ramping_down: decelerating toward neutral before end
    - braking: applying reverse thrust to stop faster
    - neutral: no active movement
    """

    def __init__(
        self,
        ramp_rate: int = 800,
        brake_enabled: bool = True,
        brake_strength: float = 0.3,
        brake_duration: float = 0.5,
        decel_time: float = 1.0,
        min_speed_pct: float = 10.0,
    ):
        self._ramp_rate = ramp_rate
        self._brake_enabled = brake_enabled
        self._brake_strength = brake_strength      # Fraction of original speed for reverse
        self._brake_duration = brake_duration      # Seconds of reverse thrust
        self._decel_time = decel_time              # Seconds before end to start ramp down
        self._min_speed_pct = min_speed_pct        # Minimum meaningful speed
        self._ramped: dict[int, float] = {}
        self._last_ramp_time: float | None = None  # C4 fix: track real dt for ramp
        self._lock = threading.Lock()              # Thread safety for _ramped dict

        # Movement phase tracking
        self._movement_start_time: float | None = None
        self._movement_end_time: float | None = None
        self._movement_original_targets: dict[int, int] = {}
        self._current_phase: str = 'neutral'  # 'ramping_up', 'cruising', 'ramping_down', 'braking', 'neutral'
        self._brake_start_time: float | None = None

    @property
    def ramp_rate(self) -> int:
        return self._ramp_rate

    @ramp_rate.setter
    def ramp_rate(self, value: int):
        self._ramp_rate = value

    @property
    def brake_enabled(self) -> bool:
        return self._brake_enabled

    @brake_enabled.setter
    def brake_enabled(self, value: bool):
        self._brake_enabled = value

    @property
    def brake_strength(self) -> float:
        return self._brake_strength

    @brake_strength.setter
    def brake_strength(self, value: float):
        self._brake_strength = value

    @property
    def brake_duration(self) -> float:
        return self._brake_duration

    @brake_duration.setter
    def brake_duration(self, value: float):
        self._brake_duration = value

    @property
    def decel_time(self) -> float:
        return self._decel_time

    @decel_time.setter
    def decel_time(self, value: float):
        self._decel_time = value

    @property
    def min_speed_pct(self) -> float:
        return self._min_speed_pct

    @min_speed_pct.setter
    def min_speed_pct(self, value: float):
        self._min_speed_pct = value

    @property
    def current_phase(self) -> str:
        return self._current_phase

    # ── Phase-aware movement control ────────────────────────────────

    def start_movement(self, channels: dict[int, int], end_time: float | None):
        """Start a new movement with phase tracking."""
        now = time.time()
        self._movement_start_time = now
        self._movement_end_time = end_time
        self._movement_original_targets = dict(channels)
        self._current_phase = 'ramping_up'
        self._brake_start_time = None

    def get_current_phase(self, end_time: float | None) -> str:
        """Determine current movement phase."""
        if end_time is None:
            return 'cruising' if self._current_phase != 'neutral' else 'neutral'

        now = time.time()
        time_remaining = end_time - now

        if time_remaining <= 0:
            if self._brake_enabled and self._current_phase == 'ramping_down':
                return 'braking'
            return 'neutral'
        elif time_remaining <= self._decel_time:
            return 'ramping_down'
        elif self._current_phase == 'ramping_up':
            # Check if we've reached target speed
            with self._lock:
                all_at_target = all(
                    abs(self._ramped.get(ch, NEUTRAL_PWM) - tgt) < 5
                    for ch, tgt in self._movement_original_targets.items()
                    if ch in RAMP_CHANNELS
                )
            if all_at_target:
                return 'cruising'
            return 'ramping_up'
        return 'cruising'

    def compute_braking_targets(self) -> dict[int, int]:
        """Compute reverse thrust targets for braking.
        
        Uses dynamic brake strength: higher speeds get stronger braking.
        """
        targets = {}
        for ch, original in self._movement_original_targets.items():
            if ch in RAMP_CHANNELS:
                offset = original - NEUTRAL_PWM
                # Dynamic brake strength: scale based on speed
                # Higher speed (larger offset) = stronger brake
                speed_factor = abs(offset) / PWM_RANGE  # 0.0 to 1.0
                dynamic_strength = min(1.0, self._brake_strength * (1.0 + speed_factor))
                reverse_offset = -int(offset * dynamic_strength)
                targets[ch] = NEUTRAL_PWM + reverse_offset
        return targets

    def end_movement(self):
        """End current movement and reset phase tracking."""
        self._movement_start_time = None
        self._movement_end_time = None
        self._movement_original_targets.clear()
        self._current_phase = 'neutral'
        self._brake_start_time = None

    # ── Ramp management ──────────────────────────────────────────────

    def apply_movement(
        self,
        channels: dict[int, int],
        movement: dict | None,
        bypass_ramp: bool = False,
        end_time: float | None = None,
    ) -> dict[int, int]:
        """Apply movement targets with optional ramping to *channels*.

        Args:
            channels: base channels dict (start with a copy of NEUTRAL_CHANNELS).
            movement: movement dict with ``'channels'`` key, or *None*.
            bypass_ramp: skip ramp — instant PWM (``just_*`` commands).
            end_time: optional movement end time for phase-aware control.

        Returns:
            The mutated *channels* dict.
        """
        targets = movement.get('channels', {}) if movement is not None else {}

        if bypass_ramp:
            with self._lock:
                for ch in RAMP_CHANNELS:
                    val = float(targets.get(ch, NEUTRAL_PWM))
                    self._ramped[ch] = val
                    channels[ch] = int(round(val))
            self._last_ramp_time = time.time()  # C4 fix: update time on bypass too
            self._current_phase = 'neutral'
        else:
            # C4 fix: calculate real dt instead of hardcoded 0.05
            now = time.time()
            dt = now - self._last_ramp_time if self._last_ramp_time else 0.05
            self._last_ramp_time = now
            max_step = self._ramp_rate * dt

            # Update phase if we have end_time
            if end_time is not None and self._movement_original_targets:
                new_phase = self.get_current_phase(end_time)
                self._current_phase = new_phase

            # Determine effective targets based on phase
            effective_targets = dict(targets)
            if self._current_phase == 'ramping_down':
                # Ramp toward neutral
                effective_targets = {ch: NEUTRAL_PWM for ch in RAMP_CHANNELS}
            elif self._current_phase == 'braking':
                # Apply reverse thrust
                if self._brake_start_time is None:
                    self._brake_start_time = now
                brake_elapsed = now - self._brake_start_time
                if brake_elapsed < self._brake_duration:
                    effective_targets = self.compute_braking_targets()
                else:
                    # Braking complete, return to neutral
                    effective_targets = {ch: NEUTRAL_PWM for ch in RAMP_CHANNELS}
                    self._current_phase = 'neutral'
                    self.end_movement()

            with self._lock:
                for ch in RAMP_CHANNELS:
                    target = float(effective_targets.get(ch, NEUTRAL_PWM))
                    current = self._ramped.get(ch, float(NEUTRAL_PWM))
                    
                    # Issue #19: Skip ramping if braking is active for this channel
                    if self._current_phase == 'braking' and ch in [CH_FORWARD, CH_LATERAL, CH_THROTTLE]:
                        # Instant brake - no ramping for movement channels
                        current = target
                    else:
                        # Normal ramping logic
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
        with self._lock:
            self._ramped.clear()
        self._last_ramp_time = None  # C4 fix: reset time tracking
        self.end_movement()  # Reset phase tracking

    def snap_channels_neutral(self, *channel_ids: int):
        """Snap specific channels to neutral in ramp state.

        Used by yaw commands to immediately stop forward/lateral thrust
        (POOL FIX 1) without resetting all ramp state.
        """
        with self._lock:
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
