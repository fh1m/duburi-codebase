"""
PID controller -- pure Python, no ROS dependency.

Implements the complete PID structure from "PID Without a PhD" (Wescott):
  - Derivative-on-measurement (not error) to avoid derivative kick
  - Anti-windup via integral clamping
  - Optional low-pass filter on derivative to suppress sensor noise
  - Output clamping to safe actuator range

Usage without ROS (for unit testing):
    pid = PIDController(kp=300, ki=5, kd=60,
                        output_min=-200, output_max=200)
    drive = pid.compute(error=0.15, measurement=0.65, dt=0.1)
"""

from __future__ import annotations


class PIDController:
    """Discrete PID controller with anti-windup and derivative filtering."""

    __slots__ = (
        "kp", "ki", "kd",
        "output_min", "output_max",
        "integral_min", "integral_max",
        "d_filter_coeff",
        "_integral", "_prev_measurement", "_prev_derivative",
        "_first_call",
    )

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -200.0,
        output_max: float = 200.0,
        integral_min: float | None = None,
        integral_max: float | None = None,
        d_filter_coeff: float = 0.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min if integral_min is not None else output_min
        self.integral_max = integral_max if integral_max is not None else output_max
        self.d_filter_coeff = max(0.0, min(1.0, d_filter_coeff))

        self._integral = 0.0
        self._prev_measurement = 0.0
        self._prev_derivative = 0.0
        self._first_call = True

    def compute(self, error: float, measurement: float, dt: float) -> float:
        """Compute one PID step.

        Args:
            error: setpoint - measurement (positive = need to move in + direction)
            measurement: current plant output (used for derivative-on-measurement)
            dt: time since last call in seconds (must be > 0)

        Returns:
            Clamped drive output.
        """
        if dt <= 0.0:
            return 0.0

        # ── Proportional ──────────────────────────────────────────────
        p_term = self.kp * error

        # ── Integral with anti-windup (Wescott: clamp to drive limits) ─
        self._integral += error * dt
        self._integral = max(self.integral_min / max(self.ki, 1e-9),
                             min(self._integral,
                                 self.integral_max / max(self.ki, 1e-9)))
        i_term = self.ki * self._integral

        # ── Derivative on measurement (not error) ─────────────────────
        if self._first_call:
            raw_derivative = 0.0
            self._first_call = False
        else:
            raw_derivative = -(measurement - self._prev_measurement) / dt

        if self.d_filter_coeff > 0.0:
            filtered = (self.d_filter_coeff * raw_derivative
                        + (1.0 - self.d_filter_coeff) * self._prev_derivative)
        else:
            filtered = raw_derivative

        d_term = self.kd * filtered
        self._prev_derivative = filtered
        self._prev_measurement = measurement

        # ── Sum and clamp ─────────────────────────────────────────────
        output = p_term + i_term + d_term
        return max(self.output_min, min(output, self.output_max))

    def reset(self) -> None:
        """Clear all internal state.  Call when switching targets or after timeout."""
        self._integral = 0.0
        self._prev_measurement = 0.0
        self._prev_derivative = 0.0
        self._first_call = True

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update gains without resetting state (for live tuning)."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
