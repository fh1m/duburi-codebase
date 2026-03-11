"""
camera_device.py – Single-camera connection lifecycle for BRACU Duburi 4.2.

Manages opening, streaming, reconnection, and status tracking for one
V4L2 camera device.  The CameraManagerNode creates one CameraDevice per
configured camera and polls it each tick.

State machine::

    DISCONNECTED ──► CONNECTING ──► CONNECTED
         ▲                              │
         └──────────────────────────────┘  (read failure)

Design notes:
    - ``read_frame()`` is non-blocking: returns (False, None) immediately
      on failure instead of retrying internally.
    - Reconnection is driven externally by the manager node via
      ``attempt_reconnect()`` — this module does not spawn threads or
      timers.
    - Device matching: tries ``device_ids`` in order; if
      ``device_name_pattern`` is set, filters by V4L2 card name first.
"""

from __future__ import annotations

import glob
import subprocess
import time
from enum import Enum, auto
from typing import Optional

import cv2
import numpy as np


class DeviceState(Enum):
    """Connection state for a single camera."""
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()


class CameraDevice:
    """Connection lifecycle manager for a single V4L2 camera.

    Args:
        name:               Logical camera name (e.g. ``'forward'``, ``'downward'``).
        device_ids:         Ordered list of /dev/videoN indices to try.
        width:              Requested frame width.
        height:             Requested frame height.
        fps:                Requested capture FPS.
        v4l2_name_pattern:  If set, match V4L2 card name to resolve device.
        reconnect_interval: Seconds between reconnect attempts.
        max_reconnect:      Max reconnect attempts (0 = unlimited).
        logger:             Optional ``rclpy`` logger for structured logging.
    """

    def __init__(
        self,
        name: str,
        device_ids: list[int] | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        v4l2_name_pattern: str | None = None,
        reconnect_interval: float = 3.0,
        max_reconnect: int = 0,
        logger=None,
    ):
        self.name = name
        self._logger = logger

        # Config
        self._device_ids: list[int] = device_ids if device_ids is not None else [0]
        self._name_pattern: str = v4l2_name_pattern or ''
        self._frame_w: int = width
        self._frame_h: int = height
        self._fps: int = fps
        self._reconnect_interval: float = reconnect_interval
        self._max_reconnect: int = max_reconnect

        # State
        self._state = DeviceState.DISCONNECTED
        self._cap: Optional[cv2.VideoCapture] = None
        self._device_id: int = -1
        self._device_path: str = ''
        self._actual_w: int = 0
        self._actual_h: int = 0
        self._actual_fps: float = 0.0
        self._reconnect_count: int = 0
        self._last_error: str = ''

        # Metrics
        self._frame_count: int = 0
        self._connect_time: float = 0.0
        self._fps_window: list[float] = []  # timestamps of recent frames
        self._fps_window_size: int = 30

    # ─── Public API ──────────────────────────────────────────────

    def open(self) -> bool:
        """Try to open the camera.  Tries ``device_ids`` in order.

        Returns True if a device was successfully opened.
        """
        self._state = DeviceState.CONNECTING

        candidates = self._resolve_device_candidates()
        for dev_id in candidates:
            if self._try_open(dev_id):
                self._state = DeviceState.CONNECTED
                self._device_path = f'/dev/video{dev_id}'
                self._connect_time = time.monotonic()
                self._reconnect_count = 0
                self._frame_count = 0
                self._fps_window.clear()
                self._last_error = ''
                if self._logger:
                    self._logger.info(
                        f'[{self.name}] Camera opened: {self._device_path}  '
                        f'{self._actual_w}x{self._actual_h}@{self._actual_fps:.0f}fps'
                    )
                return True

        self._state = DeviceState.DISCONNECTED
        self._device_path = ''
        self._last_error = f'No device opened (tried {candidates})'
        if self._logger:
            self._logger.warn(
                f'[{self.name}] Failed to open camera. '
                f'Tried device_ids={candidates}'
            )
        return False

    def close(self):
        """Release the camera device."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._state = DeviceState.DISCONNECTED
        self._device_id = -1
        self._device_path = ''
        if self._logger:
            self._logger.info(f'[{self.name}] Camera closed.')

    def read_frame(self) -> tuple[bool, Optional[np.ndarray]]:
        """Read one frame.  Non-blocking.

        Returns:
            (True, frame) on success.
            (False, None)  on failure — caller should trigger reconnection.
        """
        if self._state != DeviceState.CONNECTED or self._cap is None:
            return False, None

        ret, frame = self._cap.read()
        if not ret:
            if self._logger:
                self._logger.warn(
                    f'[{self.name}] Frame read failed — marking disconnected.',
                    throttle_duration_sec=5.0,
                )
            self._state = DeviceState.DISCONNECTED
            self._last_error = 'Frame read failed'
            self._safe_release()
            return False, None

        # Update metrics
        self._frame_count += 1
        now = time.monotonic()
        self._fps_window.append(now)
        if len(self._fps_window) > self._fps_window_size:
            self._fps_window.pop(0)

        return True, frame

    def attempt_reconnect(self) -> bool:
        """Attempt to reconnect.  Called by the manager node.

        Returns True if reconnection succeeded.
        """
        if self._state == DeviceState.CONNECTED:
            return True

        if (self._max_reconnect > 0
                and self._reconnect_count >= self._max_reconnect):
            return False

        self._reconnect_count += 1
        if self._logger:
            self._logger.info(
                f'[{self.name}] Reconnect attempt {self._reconnect_count}'
                + (f'/{self._max_reconnect}' if self._max_reconnect > 0 else '')
            )
        return self.open()

    # ─── Status / Properties ─────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._state == DeviceState.CONNECTED

    @property
    def is_connected(self) -> bool:
        return self._state == DeviceState.CONNECTED

    @property
    def state(self) -> DeviceState:
        return self._state

    @property
    def device_id(self) -> int:
        return self._device_id

    @property
    def device_path(self) -> str:
        return self._device_path

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def resolution(self) -> tuple[int, int]:
        return (self._actual_w, self._actual_h)

    @property
    def actual_fps(self) -> float:
        """Measured FPS from a rolling window of frame timestamps."""
        if len(self._fps_window) < 2:
            return 0.0
        dt = self._fps_window[-1] - self._fps_window[0]
        if dt <= 0:
            return 0.0
        return (len(self._fps_window) - 1) / dt

    @property
    def uptime(self) -> float:
        if self._state != DeviceState.CONNECTED or self._connect_time <= 0:
            return 0.0
        return time.monotonic() - self._connect_time

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def reconnect_interval(self) -> float:
        return self._reconnect_interval

    @property
    def frame_width(self) -> int:
        return self._actual_w

    @property
    def frame_height(self) -> int:
        return self._actual_h

    def get_status(self) -> dict:
        """Return a status dict (for health topic / logging)."""
        return {
            'camera_name': self.name,
            'connected': self.connected,
            'device_path': self._device_path,
            'width': self._actual_w,
            'height': self._actual_h,
            'actual_fps': round(self.actual_fps, 1),
            'frame_count': self._frame_count,
            'uptime': round(self.uptime, 1),
            'last_error': self._last_error,
        }

    # ─── Internal ────────────────────────────────────────────────

    def _try_open(self, dev_id: int) -> bool:
        """Attempt to open a single device index."""
        self._safe_release()

        cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_h)
        cap.set(cv2.CAP_PROP_FPS, self._fps)

        self._cap = cap
        self._device_id = dev_id
        self._actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._actual_fps = cap.get(cv2.CAP_PROP_FPS)

        if self._actual_w != self._frame_w or self._actual_h != self._frame_h:
            if self._logger:
                self._logger.warn(
                    f'[{self.name}] Requested {self._frame_w}x{self._frame_h} '
                    f'but camera negotiated {self._actual_w}x{self._actual_h}'
                )

        return True

    def _resolve_device_candidates(self) -> list[int]:
        """Return ordered list of device indices to try.

        If ``device_name_pattern`` is set, filter V4L2 devices by card
        name first and prepend matching indices.
        """
        if not self._name_pattern:
            return list(self._device_ids)

        matched = []
        try:
            for dev_path in sorted(glob.glob('/dev/video*')):
                try:
                    idx = int(dev_path.replace('/dev/video', ''))
                except ValueError:
                    continue
                try:
                    result = subprocess.run(
                        ['v4l2-ctl', '--device', dev_path, '--info'],
                        capture_output=True, text=True, timeout=2,
                    )
                    for line in result.stdout.splitlines():
                        if 'Card type' in line:
                            card = line.split(':', 1)[1].strip()
                            if self._name_pattern.lower() in card.lower():
                                matched.append(idx)
                            break
                except (FileNotFoundError, subprocess.TimeoutExpired):
                    pass
        except Exception:
            pass

        # Matched by name first, then fallback to configured ids
        seen = set(matched)
        for dev_id in self._device_ids:
            if dev_id not in seen:
                matched.append(dev_id)
                seen.add(dev_id)

        return matched

    def _safe_release(self):
        """Release capture if held."""
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
