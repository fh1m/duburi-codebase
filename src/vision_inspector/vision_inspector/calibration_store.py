"""
calibration_store.py – Camera calibration file management.

Handles loading, saving, and auto-discovery of calibration YAML files
that are compatible with the ROS ``sensor_msgs/CameraInfo`` message.

Used by:
    - ``camera_manager_node``: loads calibration at startup per camera.
    - ``camera_calibrator``: saves calibration after running the tool.

Calibration files follow the naming convention::

    calibration_{camera_name}_{timestamp}.yaml

The store searches a configurable directory for matching files and
selects the most recent one when no explicit path is provided.
"""

from __future__ import annotations

import os
from datetime import datetime
from typing import Optional

import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo


class CalibrationStore:
    """Load, save, and discover camera calibration YAML files.

    Args:
        calibration_dir: Base directory for calibration files.
        logger:          Optional rclpy logger.  If None, prints to stdout.
    """

    def __init__(self, calibration_dir: str | None = 'calibration', logger=None):
        self._dir = calibration_dir or os.path.expanduser('~/.ros/camera_calibrations')
        self._logger = logger
        os.makedirs(self._dir, exist_ok=True)

    # ─── Load ────────────────────────────────────────────────────

    def load(
        self,
        camera_name: str,
        width: int = 640,
        height: int = 480,
        explicit_path: str | None = '',
        image_width: int | None = None,
        image_height: int | None = None,
    ) -> tuple[CameraInfo, bool]:
        """Load calibration as a CameraInfo message.

        Resolution order:
            1. ``explicit_path`` (if non-empty and exists)
            2. Auto-lookup by ``camera_name`` in calibration dir
            3. Identity / zero-initialised fallback

        Args:
            camera_name:   Logical name for auto-lookup (e.g. ``'forward'``).
            width:         Fallback width if no calibration found.
            height:        Fallback height if no calibration found.
            explicit_path: If set, load this file directly.
            image_width:   Alias for width (backward compat).
            image_height:  Alias for height (backward compat).

        Returns:
            (CameraInfo, calibrated) -- ``calibrated`` is True if a real
            file was loaded, False if using fallback.
        """
        if image_width is not None:
            width = image_width
        if image_height is not None:
            height = image_height
        path = explicit_path or ''

        # 1. Explicit path
        if path and os.path.isfile(path):
            info = self._load_yaml(path, width, height)
            if info is not None:
                self._log_info(f'Loaded calibration from {path}')
                return info, True

        # 2. Auto-lookup
        if not path:
            found = self.find_latest(camera_name)
            if found:
                info = self._load_yaml(found, width, height)
                if info is not None:
                    self._log_info(
                        f'Auto-loaded calibration for "{camera_name}" '
                        f'from {found}'
                    )
                    return info, True

        # 3. Fallback
        if path:
            self._log_warn(f'Calibration file not found: {path}')
        return self._make_default(width, height), False

    # ─── Save ────────────────────────────────────────────────────

    def save(
        self,
        camera_name: str,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        image_size: tuple[int, int] | None = None,
        rms_error: float = 0.0,
        mean_reproj_error: float = 0.0,
        num_images: int = 0,
        width: int | None = None,
        height: int | None = None,
        rectification_matrix: np.ndarray | None = None,
        projection_matrix: np.ndarray | None = None,
    ) -> str:
        """Save calibration to a YAML file.

        Args:
            camera_name:            Logical name (used in filename).
            camera_matrix:          3x3 intrinsic matrix.
            dist_coeffs:            Distortion coefficients.
            image_size:             (width, height) tuple.
            rms_error:              RMS calibration error.
            mean_reproj_error:      Mean reprojection error.
            num_images:             Number of images used for calibration.
            width:                  Image width (alternative to image_size).
            height:                 Image height (alternative to image_size).
            rectification_matrix:   3x3 rectification matrix (default: identity).
            projection_matrix:      3x4 projection matrix (default: from camera_matrix).

        Returns:
            Absolute path to the saved YAML file.
        """
        if image_size is not None:
            w, h = image_size
        else:
            w = width or 640
            h = height or 480

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        if projection_matrix is None:
            P = np.zeros((3, 4))
            P[:3, :3] = camera_matrix
        else:
            P = projection_matrix

        R = rectification_matrix if rectification_matrix is not None else np.eye(3)

        cal_data = {
            'image_width': w,
            'image_height': h,
            'camera_name': camera_name,
            'distortion_model': 'plumb_bob',
            'camera_matrix': {
                'rows': 3, 'cols': 3,
                'data': camera_matrix.flatten().tolist(),
            },
            'distortion_coefficients': {
                'rows': 1, 'cols': len(dist_coeffs.flatten()),
                'data': dist_coeffs.flatten().tolist(),
            },
            'rectification_matrix': {
                'rows': 3, 'cols': 3,
                'data': R.flatten().tolist(),
            },
            'projection_matrix': {
                'rows': 3, 'cols': 4,
                'data': P.flatten().tolist(),
            },
            'calibration_date': timestamp,
            'rms_error': float(rms_error),
            'mean_reprojection_error': float(mean_reproj_error),
            'num_images_used': num_images,
        }

        filename = os.path.join(
            self._dir,
            f'calibration_{camera_name}_{timestamp}.yaml',
        )
        with open(filename, 'w') as f:
            yaml.dump(cal_data, f, default_flow_style=False)

        self._log_info(f'Calibration saved to {filename}')
        return os.path.abspath(filename)

    # ─── Discovery ───────────────────────────────────────────────

    def find_latest(self, camera_name: str) -> Optional[str]:
        """Find the newest calibration file for ``camera_name``.

        Searches ``calibration_dir`` for files matching
        ``calibration_{camera_name}_*.yaml`` and returns the newest
        by filename timestamp (lexicographic sort).

        Returns:
            Path to the newest file, or None if nothing matches.
        """
        if not os.path.isdir(self._dir):
            return None

        prefix = f'calibration_{camera_name}_'
        matches = [
            f for f in os.listdir(self._dir)
            if f.startswith(prefix) and f.endswith('.yaml')
        ]
        if not matches:
            return None

        matches.sort(reverse=True)  # newest timestamp first
        return os.path.join(self._dir, matches[0])

    def list_calibrations(self) -> list[dict]:
        """List all calibration files in the store.

        Returns:
            List of dicts with keys: path, camera_name, date, image_size.
        """
        results = []
        if not os.path.isdir(self._dir):
            return results

        for fname in sorted(os.listdir(self._dir)):
            if not fname.endswith('.yaml'):
                continue
            fpath = os.path.join(self._dir, fname)
            try:
                with open(fpath, 'r') as f:
                    data = yaml.safe_load(f)
                results.append({
                    'path': fpath,
                    'camera_name': data.get('camera_name', '?'),
                    'date': data.get('calibration_date', '?'),
                    'image_size': (
                        f'{data.get("image_width", "?")}x'
                        f'{data.get("image_height", "?")}'
                    ),
                    'rms_error': data.get('rms_error', None),
                })
            except Exception:
                results.append({
                    'path': fpath,
                    'camera_name': '?',
                    'date': '?',
                    'image_size': '?',
                    'rms_error': None,
                })
        return results

    # ─── Internal ────────────────────────────────────────────────

    def _load_yaml(
        self, path: str, fallback_w: int, fallback_h: int
    ) -> Optional[CameraInfo]:
        """Parse a calibration YAML into a CameraInfo message."""
        try:
            with open(path, 'r') as f:
                cal = yaml.safe_load(f)

            info = CameraInfo()
            info.width = cal.get('image_width', fallback_w)
            info.height = cal.get('image_height', fallback_h)
            info.distortion_model = cal.get('distortion_model', 'plumb_bob')

            cm = cal.get('camera_matrix', {})
            dm = cal.get('distortion_coefficients', {})
            rm = cal.get('rectification_matrix', {})
            pm = cal.get('projection_matrix', {})

            if 'data' in cm:
                info.k = [float(x) for x in cm['data']]
            if 'data' in dm:
                info.d = [float(x) for x in dm['data']]
            if 'data' in rm:
                info.r = [float(x) for x in rm['data']]
            if 'data' in pm:
                info.p = [float(x) for x in pm['data']]

            return info
        except Exception as e:
            self._log_error(f'Failed to load calibration {path}: {e}')
            return None

    @staticmethod
    def _make_default(width: int, height: int) -> CameraInfo:
        """Return a zero-initialised CameraInfo with identity R."""
        info = CameraInfo()
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.k = [0.0] * 9
        info.d = [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [0.0] * 12
        return info

    # ─── Logging helpers ─────────────────────────────────────────

    def _log_info(self, msg: str):
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg: str):
        if self._logger:
            self._logger.warn(msg)

    def _log_error(self, msg: str):
        if self._logger:
            self._logger.error(msg)
