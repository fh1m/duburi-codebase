"""
Kalman filter object tracker for bounding box smoothing.

Implements the recursive Kalman algorithm (MIT Tutorial, Lacey Ch. 11):
  Predict  ->  Compute Gain  ->  Correct  ->  Update Covariance

Design choices drawn from:
  - MIT Kalman Filter tutorial: state-space formulation, MSE minimisation
  - SMART-TRACK (Zahana): dropout handling via predict-only when detection lost
  - robot_localization (Moore & Stouch): continuous estimation even without measurements

Pure Python + NumPy, no ROS dependency -- unit-testable standalone.

State vector (6D):  [cx, cy, vx, vy, w, h]
  cx, cy  = normalised bbox center (0.0 - 1.0)
  vx, vy  = velocity per time step (normalised units)
  w, h    = normalised bbox width, height

Measurement vector (4D):  [cx, cy, w, h]
  Velocities are hidden state, inferred from position changes.
"""

from __future__ import annotations

import numpy as np


class SingleObjectKF:
    """Kalman filter for one tracked object."""

    def __init__(
        self,
        process_noise: float = 1e-3,
        measurement_noise: float = 5e-3,
        max_dropout_frames: int = 5,
        dt: float = 1.0,
    ) -> None:
        self.max_dropout_frames = max_dropout_frames
        self._dropout_count = 0
        self._initialized = False
        self._dt = dt

        n_state = 6
        n_meas = 4

        # State transition: constant-velocity model
        # x_{k+1} = Phi * x_k
        self._F = np.eye(n_state, dtype=np.float64)
        self._F[0, 2] = dt  # cx += vx * dt
        self._F[1, 3] = dt  # cy += vy * dt

        # Measurement matrix: we observe [cx, cy, w, h], not velocities
        self._H = np.zeros((n_meas, n_state), dtype=np.float64)
        self._H[0, 0] = 1.0  # cx
        self._H[1, 1] = 1.0  # cy
        self._H[2, 4] = 1.0  # w
        self._H[3, 5] = 1.0  # h

        # Process noise covariance Q
        self._Q = np.eye(n_state, dtype=np.float64) * process_noise
        self._Q[2, 2] *= 0.5  # velocity components have less process noise
        self._Q[3, 3] *= 0.5

        # Measurement noise covariance R
        self._R = np.eye(n_meas, dtype=np.float64) * measurement_noise

        # State and covariance
        self._x = np.zeros(n_state, dtype=np.float64)
        self._P = np.eye(n_state, dtype=np.float64) * 0.1

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def is_lost(self) -> bool:
        return self._dropout_count > self.max_dropout_frames

    @property
    def dropout_count(self) -> int:
        return self._dropout_count

    def init_state(self, cx: float, cy: float, w: float, h: float) -> None:
        """Initialise from first detection."""
        self._x = np.array([cx, cy, 0.0, 0.0, w, h], dtype=np.float64)
        self._P = np.eye(6, dtype=np.float64) * 0.1
        self._P[2, 2] = 1.0  # high initial velocity uncertainty
        self._P[3, 3] = 1.0
        self._initialized = True
        self._dropout_count = 0

    def predict(self) -> tuple[float, float, float, float]:
        """Predict step: project state and covariance forward.

        Returns predicted (cx, cy, w, h).
        Always call this before correct().
        """
        if not self._initialized:
            return (0.5, 0.5, 0.0, 0.0)

        # x' = F * x
        self._x = self._F @ self._x
        # P' = F * P * F^T + Q
        self._P = self._F @ self._P @ self._F.T + self._Q

        return (self._x[0], self._x[1], self._x[4], self._x[5])

    def correct(self, cx: float, cy: float, w: float, h: float) -> tuple[float, float, float, float]:
        """Correction step using a new measurement.

        Computes Kalman gain K = P'H^T(HP'H^T + R)^{-1},
        updates state and covariance.  Resets dropout counter.

        Returns corrected (cx, cy, w, h).
        """
        if not self._initialized:
            self.init_state(cx, cy, w, h)
            return (cx, cy, w, h)

        z = np.array([cx, cy, w, h], dtype=np.float64)

        # Innovation (measurement residual): y = z - H * x'
        y = z - self._H @ self._x

        # Innovation covariance: S = H * P' * H^T + R
        S = self._H @ self._P @ self._H.T + self._R

        # Kalman gain: K = P' * H^T * S^{-1}
        K = self._P @ self._H.T @ np.linalg.inv(S)

        # State update: x = x' + K * y
        self._x = self._x + K @ y

        # Covariance update: P = (I - K * H) * P'
        I = np.eye(6, dtype=np.float64)
        self._P = (I - K @ self._H) @ self._P

        self._dropout_count = 0
        return (self._x[0], self._x[1], self._x[4], self._x[5])

    def mark_missed(self) -> None:
        """Call when no detection is available this frame (predict-only mode)."""
        self._dropout_count += 1

    def get_full_state(self) -> tuple[float, float, float, float, float, float]:
        """Return (cx, cy, vx, vy, w, h) -- includes velocity estimates."""
        return tuple(self._x.tolist())

    def reset(self) -> None:
        """Full reset -- call when switching targets."""
        self._x = np.zeros(6, dtype=np.float64)
        self._P = np.eye(6, dtype=np.float64) * 0.1
        self._initialized = False
        self._dropout_count = 0


class KalmanObjectTracker:
    """Manages per-class Kalman filters for single-object tracking.

    Each target class gets its own SingleObjectKF.  When enabled, the tracker
    smooths raw detections.  When disabled (bypass mode), it passes through
    raw coordinates unchanged -- this is the key toggle for pool testing.
    """

    def __init__(
        self,
        enabled: bool = True,
        process_noise: float = 1e-3,
        measurement_noise: float = 5e-3,
        max_dropout_frames: int = 5,
        dt: float = 1.0,
    ) -> None:
        self.enabled = enabled
        self._process_noise = process_noise
        self._measurement_noise = measurement_noise
        self._max_dropout_frames = max_dropout_frames
        self._dt = dt
        self._filters: dict[str, SingleObjectKF] = {}

    def _get_filter(self, class_name: str) -> SingleObjectKF:
        if class_name not in self._filters:
            self._filters[class_name] = SingleObjectKF(
                process_noise=self._process_noise,
                measurement_noise=self._measurement_noise,
                max_dropout_frames=self._max_dropout_frames,
                dt=self._dt,
            )
        return self._filters[class_name]

    def update(
        self,
        class_name: str,
        detected: bool,
        cx: float = 0.0,
        cy: float = 0.0,
        w: float = 0.0,
        h: float = 0.0,
    ) -> tuple[float, float, float, float, bool]:
        """Process one frame for a target class.

        Args:
            class_name: object class being tracked
            detected: whether YOLO found the object this frame
            cx, cy, w, h: normalised detection coords (only used if detected=True)

        Returns:
            (cx, cy, w, h, is_lost) -- smoothed/predicted coords, and whether
            the target is considered lost (too many consecutive dropouts).
            When tracker is disabled, returns raw coords and is_lost=not detected.
        """
        if not self.enabled:
            return (cx, cy, w, h, not detected)

        kf = self._get_filter(class_name)

        kf.predict()

        if detected:
            out = kf.correct(cx, cy, w, h)
            return (*out, False)
        else:
            kf.mark_missed()
            if kf.is_lost or not kf.is_initialized:
                return (0.0, 0.0, 0.0, 0.0, True)
            state = kf.get_full_state()
            return (state[0], state[1], state[4], state[5], False)

    def reset(self, class_name: str | None = None) -> None:
        """Reset filter(s).  None = reset all."""
        if class_name is None:
            self._filters.clear()
        elif class_name in self._filters:
            self._filters[class_name].reset()

    def is_target_lost(self, class_name: str) -> bool:
        if class_name not in self._filters:
            return True
        return self._filters[class_name].is_lost
