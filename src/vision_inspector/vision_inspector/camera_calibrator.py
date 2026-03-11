"""
camera_calibrator.py – Checkerboard camera calibration tool for BRACU Duburi 4.2.

Captures frames from a camera, detects a checkerboard pattern, collects
calibration images, computes intrinsics (camera matrix + distortion), and
saves the result via CalibrationStore (ROS CameraInfo-compatible YAML).

Usage:
    ros2 run vision_inspector camera_calibrate
    ros2 run vision_inspector camera_calibrate --ros-args \\
        -p device_id:=0 -p board_width:=9 -p board_height:=6 \\
        -p square_size:=0.025 -p num_images:=20 -p camera_name:=forward

Controls during capture:
    [SPACE] – capture current frame for calibration
    [c]     – run calibration with collected frames
    [q]     – quit
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from vision_inspector.calibration_store import CalibrationStore


class CameraCalibratorNode(Node):
    """Interactive checkerboard camera calibrator."""

    def __init__(self):
        super().__init__('camera_calibrator')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('board_width', 9)
        self.declare_parameter('board_height', 6)
        self.declare_parameter('square_size', 0.025)
        self.declare_parameter('num_images', 20)
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('calibration_dir', '')

        self.device_id = self.get_parameter('device_id').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.board_w = self.get_parameter('board_width').value
        self.board_h = self.get_parameter('board_height').value
        self.square_size = self.get_parameter('square_size').value
        self.num_target = self.get_parameter('num_images').value
        self.camera_name = self.get_parameter('camera_name').value
        calib_dir = self.get_parameter('calibration_dir').value

        self.board_size = (self.board_w, self.board_h)

        # Use CalibrationStore for saving
        self._calib_store = CalibrationStore(
            calibration_dir=calib_dir or None
        )

        self.get_logger().info(
            f'Calibrator ready: device={self.device_id}  '
            f'board={self.board_w}x{self.board_h}  '
            f'square={self.square_size}m  target={self.num_target} images  '
            f'camera_name={self.camera_name}'
        )

    def run_calibration(self):
        """Interactive calibration loop (blocking)."""
        cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open /dev/video{self.device_id}')
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # 3D object points in real-world coordinates
        objp = np.zeros((self.board_w * self.board_h, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_w, 0:self.board_h].T.reshape(-1, 2)
        objp *= self.square_size

        obj_points = []
        img_points = []
        calib_images = []

        window_name = 'Duburi Camera Calibration'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, actual_w, actual_h)
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
        )

        print(f'\n  ── Camera Calibration ──')
        print(f'  Board: {self.board_w}x{self.board_h} inner corners')
        print(f'  Square size: {self.square_size}m')
        print(f'  Target: {self.num_target} calibration images')
        print(f'  Controls: [SPACE] capture  [c] calibrate  [q] quit\n')

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                display = frame.copy()

                found, corners = cv2.findChessboardCorners(
                    gray, self.board_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    + cv2.CALIB_CB_FAST_CHECK
                    + cv2.CALIB_CB_NORMALIZE_IMAGE,
                )

                if found:
                    corners_refined = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), criteria
                    )
                    cv2.drawChessboardCorners(
                        display, self.board_size, corners_refined, found
                    )
                    status = 'BOARD DETECTED – press SPACE to capture'
                    color = (0, 255, 0)
                else:
                    status = 'No board detected – show checkerboard'
                    color = (0, 0, 255)

                cv2.putText(display, status, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                cv2.putText(display,
                            f'Captured: {len(calib_images)}/{self.num_target}',
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 0), 2)
                cv2.putText(display,
                            '[SPACE] capture  [c] calibrate  [q] quit',
                            (10, actual_h - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                cv2.imshow(window_name, display)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break

                elif key == ord(' ') and found:
                    corners_refined = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), criteria
                    )
                    obj_points.append(objp)
                    img_points.append(corners_refined)
                    calib_images.append(gray.copy())

                    n = len(calib_images)
                    self.get_logger().info(
                        f'Captured calibration image {n}/{self.num_target}'
                    )

                    # Flash effect
                    cv2.imshow(window_name, np.ones_like(display) * 255)
                    cv2.waitKey(100)

                    if n >= self.num_target:
                        self.get_logger().info(
                            'Target reached! Press [c] to calibrate or '
                            'keep capturing for more accuracy.'
                        )

                elif key == ord('c'):
                    if len(calib_images) < 5:
                        self.get_logger().warn(
                            f'Need at least 5 images, have {len(calib_images)}'
                        )
                        continue

                    self.get_logger().info(
                        f'Running calibration with {len(calib_images)} images…'
                    )
                    self._compute_and_save(
                        obj_points, img_points, (actual_w, actual_h)
                    )
                    break

        except KeyboardInterrupt:
            if len(calib_images) >= 5:
                self.get_logger().info('Interrupted – saving calibration…')
                self._compute_and_save(
                    obj_points, img_points, (actual_w, actual_h)
                )
        finally:
            cap.release()
            cv2.destroyAllWindows()

    # ─────────────────────────────────────────────────────────────

    def _compute_and_save(self, obj_points, img_points, image_size):
        """Compute calibration and save via CalibrationStore."""
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, image_size, None, None
        )

        if not ret:
            self.get_logger().error('Calibration failed!')
            return

        # Reprojection error
        total_error = 0.0
        for i in range(len(obj_points)):
            reproj, _ = cv2.projectPoints(
                obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
            )
            error = cv2.norm(img_points[i], reproj, cv2.NORM_L2) / len(reproj)
            total_error += error
        mean_error = total_error / len(obj_points)

        self.get_logger().info(
            f'Calibration successful!  RMS: {ret:.4f}  '
            f'Mean reproj error: {mean_error:.4f}px'
        )

        # Build projection matrix
        P = np.zeros((3, 4))
        P[:3, :3] = camera_matrix

        # Save via CalibrationStore
        path = self._calib_store.save(
            camera_name=self.camera_name,
            width=image_size[0],
            height=image_size[1],
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            rectification_matrix=np.eye(3),
            projection_matrix=P,
        )

        self.get_logger().info(f'Calibration saved to: {path}')

        print(f'\n  ── Calibration Results ──')
        print(f'  RMS Error:              {ret:.4f}')
        print(f'  Mean Reproj Error:      {mean_error:.4f}px')
        print(f'  Images Used:            {len(obj_points)}')
        print(f'  Camera Matrix:\n{camera_matrix}')
        print(f'  Distortion Coefficients: {dist_coeffs.flatten()}')
        print(f'  Saved to: {path}\n')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibratorNode()
    node.run_calibration()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
