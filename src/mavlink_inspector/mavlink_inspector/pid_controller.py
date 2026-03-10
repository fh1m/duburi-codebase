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
        'kp', 'ki', 'kd', 'output_limit', 'max_integral',
        'tolerance', 'ema_alpha', 'max_rate', 'anti_windup',
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
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Limits
        self.output_limit = output_limit  # max |PWM offset|
        self.max_integral = max_integral
        self.tolerance = tolerance        # deadband width (0 = disabled)
        self.max_rate = max_rate          # max output change per tick

        # Behaviour
        self.ema_alpha = ema_alpha        # EMA coefficient (1.0 = no filtering)
        self.anti_windup = anti_windup    # only integrate when not saturated

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
        if self.tolerance > 0 and abs(error) < self.tolerance:
            self._in_deadband = True
            # Smoothly decay rate-limiter state toward 0 so re-entry
            # doesn't cause a PWM jump.
            if abs(self._last_output) <= self.max_rate:
                self._last_output = 0
            elif self._last_output > 0:
                self._last_output -= self.max_rate
            else:
                self._last_output += self.max_rate
            return 0

        self._in_deadband = False

        # ── Proportional ─────────────────────────────────────────────
        p_out = self.kp * error

        # ── Integral (with conditional anti-windup) ──────────────────
        if not self.anti_windup or abs(self._last_output) < self.output_limit:
            self._integral += error * dt
        self._integral = max(-self.max_integral,
                             min(self.max_integral, self._integral))
        i_out = self.ki * self._integral

        # ── Derivative (EMA-filtered measurement rate) ───────────────
        if measurement_rate is not None:
            self._filtered_rate = (
                self.ema_alpha * measurement_rate
                + (1.0 - self.ema_alpha) * self._filtered_rate
            )
        d_out = -self.kd * self._filtered_rate

        # ── Total → clamp → rate-limit ───────────────────────────────
        pid_output = p_out + i_out + d_out
        output = max(-self.output_limit,
                     min(self.output_limit, int(pid_output)))
        output = max(self._last_output - self.max_rate,
                     min(self._last_output + self.max_rate, output))
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
