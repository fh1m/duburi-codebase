"""Reusable PID controller for the BRACU Duburi 4.2 AUV.

Supports:
- Configurable gains (Kp, Ki, Kd)
- Deadband tolerance (suppress noise near target)
- EMA-filtered derivative on measurement
- Anti-windup (conditional integration when output saturated)
- Output rate limiting (prevent thruster hunting)

Used for both depth hold and yaw-to-heading control with different configs.
"""

from __future__ import annotations


class PidController:
    """Generic PID controller with deadband, anti-windup, and rate limiting.

    Depth PID config example::

        PidController(kp=800, ki=50, kd=100, output_limit=400,
                      max_integral=1.0, tolerance=0.08, ema_alpha=0.3,
                      max_rate=50, anti_windup=True)

    Yaw PID config example::

        PidController(kp=2.0, ki=0.05, kd=0.5, output_limit=200,
                      max_integral=50.0, tolerance=0, ema_alpha=1.0,
                      max_rate=50, anti_windup=False)
    """

    __slots__ = (
        '_kp', '_ki', '_kd', '_output_limit', '_max_integral',
        '_tolerance', '_ema_alpha', '_max_rate', '_anti_windup',
        '_integral', '_filtered_rate', '_last_output', '_in_deadband',
    )

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: int,
        max_integral: float = 1.0,
        tolerance: float = 0.0,
        ema_alpha: float = 0.3,
        max_rate: int = 50,
        anti_windup: bool = True,
    ):
        # Gains
        self._kp = kp
        self._ki = ki
        self._kd = kd

        # Limits
        self._output_limit = output_limit  # max |PWM offset|
        self._max_integral = max_integral
        self._tolerance = tolerance        # deadband width (0 = disabled)
        self._max_rate = max_rate          # max output change per tick

        # Behaviour
        self._ema_alpha = ema_alpha        # EMA coefficient (1.0 = no filtering)
        self._anti_windup = anti_windup    # only integrate when not saturated

        # State
        self._integral = 0.0
        self._filtered_rate = 0.0
        self._last_output = 0
        self._in_deadband = False

    # ── Main compute ─────────────────────────────────────────────────

    def compute(
        self,
        error: float,
        dt: float,
        measurement_rate: float | None = None,
    ) -> int:
        """Compute PID output for one tick.

        Args:
            error: setpoint − measurement.  Positive = above target (depth)
                   or shortest-path angle error (yaw).
            dt: time delta in seconds since last call.
            measurement_rate: rate of change of the measured variable.
                For depth: ``(depth − prev_depth) / dt`` (m/s).
                For yaw:   ``heading_rate`` from gyro (deg/s).
                If *None*, derivative term uses the last filtered rate.

        Returns:
            PWM offset from neutral, clamped to ±output_limit and rate-limited.
            Returns **0** when inside the deadband (caller should apply neutral).
        """
        if dt <= 0:
            dt = 0.05  # fallback to one 20 Hz tick

        # ── Deadband ─────────────────────────────────────────────────
        if self._tolerance > 0 and abs(error) < self._tolerance:
            self._in_deadband = True
            # Smoothly decay rate-limiter state toward 0 so re-entry
            # doesn't cause a PWM jump.
            if abs(self._last_output) <= self._max_rate:
                self._last_output = 0
            elif self._last_output > 0:
                self._last_output -= self._max_rate
            else:
                self._last_output += self._max_rate
            return 0

        self._in_deadband = False

        # ── Proportional ─────────────────────────────────────────────
        p_out = self._kp * error

        # ── Derivative (EMA-filtered measurement rate) ───────────────
        if measurement_rate is not None:
            self._filtered_rate = (
                self._ema_alpha * measurement_rate
                + (1.0 - self._ema_alpha) * self._filtered_rate
            )
        d_out = -self._kd * self._filtered_rate

        # ── Integral (with anti-windup checking CURRENT output) ──────
        # C2 safety fix: Check preliminary output BEFORE integrating
        ki_term = error * dt
        preliminary = p_out + (self._integral + ki_term) * self._ki + d_out

        # Only integrate if not saturating
        if not self._anti_windup or abs(preliminary) < self._output_limit:
            self._integral += ki_term
            self._integral = max(-self._max_integral,
                                 min(self._max_integral, self._integral))
        # else: Saturating - do not accumulate integral (anti-windup)

        i_out = self._ki * self._integral

        # ── Total → clamp → rate-limit ───────────────────────────────
        pid_output = p_out + i_out + d_out
        output = max(-self._output_limit,
                     min(self._output_limit, int(pid_output)))
        output = max(self._last_output - self._max_rate,
                     min(self._last_output + self._max_rate, output))
        self._last_output = output
        return output

    # ── State management ─────────────────────────────────────────────

    def reset(self):
        """Reset all controller state (call on mode transitions)."""
        self._integral = 0.0
        self._filtered_rate = 0.0
        self._last_output = 0
        self._in_deadband = False

    @property
    def in_deadband(self) -> bool:
        """True when error is within tolerance (output forced to 0)."""
        return self._in_deadband

    @property
    def last_output(self) -> int:
        """Last computed output (for rate-limiting continuity)."""
        return self._last_output

    @property
    def integral(self) -> float:
        """Current integral accumulator value."""
        return self._integral

    # ── Dynamic reconfigure setters ──────────────────────────────────────

    @property
    def kp(self) -> float:
        return self._kp

    @kp.setter
    def kp(self, value: float):
        """Update proportional gain (for dynamic reconfigure)."""
        self._kp = value

    @property
    def ki(self) -> float:
        return self._ki

    @ki.setter
    def ki(self, value: float):
        """Update integral gain (for dynamic reconfigure)."""
        self._ki = value

    @property
    def kd(self) -> float:
        return self._kd

    @kd.setter
    def kd(self, value: float):
        """Update derivative gain (for dynamic reconfigure)."""
        self._kd = value

    @property
    def output_limit(self) -> int:
        return self._output_limit

    @output_limit.setter
    def output_limit(self, value: int):
        """Update output limit (for dynamic reconfigure)."""
        self._output_limit = value

    @property
    def max_integral(self) -> float:
        return self._max_integral

    @max_integral.setter
    def max_integral(self, value: float):
        """Update max integral accumulator (for dynamic reconfigure)."""
        self._max_integral = value
        # Clamp current integral if it exceeds new limit
        if self._integral > value:
            self._integral = value
        elif self._integral < -value:
            self._integral = -value

    @property
    def tolerance(self) -> float:
        return self._tolerance

    @tolerance.setter
    def tolerance(self, value: float):
        """Update deadband tolerance (for dynamic reconfigure)."""
        self._tolerance = value

    @property
    def ema_alpha(self) -> float:
        return self._ema_alpha

    @ema_alpha.setter
    def ema_alpha(self, value: float):
        """Update EMA filter coefficient (for dynamic reconfigure)."""
        self._ema_alpha = max(0.0, min(1.0, value))  # Clamp to [0, 1]

    @property
    def max_rate(self) -> int:
        return self._max_rate

    @max_rate.setter
    def max_rate(self, value: int):
        """Update max output rate change (for dynamic reconfigure)."""
        self._max_rate = value

    @property
    def anti_windup(self) -> bool:
        return self._anti_windup

    @anti_windup.setter
    def anti_windup(self, value: bool):
        """Enable/disable anti-windup (for dynamic reconfigure)."""
        self._anti_windup = value
