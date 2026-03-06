"""
camera_calibrator.py – Checkerboard camera calibration tool for BRACU Duburi 4.2.

Captures frames from a camera, detects a checkerboard pattern, collects
calibration images, computes intrinsics (camera matrix + distortion), and
saves the result as a YAML file compatible with ROS CameraInfo.

Usage:
    ros2 run vision_manager camera_calibrate
    ros2 run vision_manager camera_calibrate --ros-args \
        -p device_id:=0 -p board_width:=9 -p board_height:=6 \
        -p square_size:=0.025 -p num_images:=20

Controls during capture:
    [SPACE] – capture current frame for calibration
    [c]     – run calibration with collected frames
    [q]     – quit
"""

import os
import time
from datetime import datetime

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node


class CameraCalibratorNode(Node):
    """Interactive checkerboard camera calibrator."""

    def __init__(self):
        super().__init__('camera_calibrator')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('board_width', 9)      # inner corners per row
        self.declare_parameter('board_height', 6)      # inner corners per col
        self.declare_parameter('square_size', 0.025)   # metres
        self.declare_parameter('num_images', 20)       # target calibration images
        self.declare_parameter('output_dir', 'calibration')

        self.device_id = self.get_parameter('device_id').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.board_w = self.get_parameter('board_width').value
        self.board_h = self.get_parameter('board_height').value
        self.square_size = self.get_parameter('square_size').value
        self.num_target = self.get_parameter('num_images').value
        self.output_dir = self.get_parameter('output_dir').value

        self.board_size = (self.board_w, self.board_h)

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(
            f'Calibrator ready: device={self.device_id}  '
            f'board={self.board_w}x{self.board_h}  '
            f'square={self.square_size}m  target={self.num_target} images'
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

        # Prepare object points (3D points in real world space)
        objp = np.zeros((self.board_w * self.board_h, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_w, 0:self.board_h].T.reshape(-1, 2)
        objp *= self.square_size

        obj_points = []   # 3D points
        img_points = []   # 2D points in image plane
        calib_images = []

        window_name = 'Duburi Camera Calibration'
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

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

                # Try to find checkerboard
                found, corners = cv2.findChessboardCorners(
                    gray, self.board_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH +
                    cv2.CALIB_CB_FAST_CHECK +
                    cv2.CALIB_CB_NORMALIZE_IMAGE
                )

                if found:
                    corners_refined = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), criteria
                    )
                    cv2.drawChessboardCorners(display, self.board_size,
                                             corners_refined, found)
                    status = 'BOARD DETECTED – press SPACE to capture'
                    color = (0, 255, 0)
                else:
                    status = 'No board detected – show checkerboard'
                    color = (0, 0, 255)

                # Draw UI overlay
                cv2.putText(display, status, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                cv2.putText(display,
                            f'Captured: {len(calib_images)}/{self.num_target}',
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 0), 2)
                cv2.putText(display, '[SPACE] capture  [c] calibrate  [q] quit',
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
                    cv2.imshow(window_name,
                               np.ones_like(display) * 255)
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
                        f'Running calibration with {len(calib_images)} images...'
                    )
                    self._compute_and_save(obj_points, img_points,
                                           (actual_w, actual_h))
                    break

        except KeyboardInterrupt:
            if len(calib_images) >= 5:
                self.get_logger().info('Interrupted – saving calibration...')
                self._compute_and_save(obj_points, img_points,
                                       (actual_w, actual_h))
        finally:
            cap.release()
            cv2.destroyAllWindows()

    def _compute_and_save(self, obj_points, img_points, image_size):
        """Compute calibration and save to YAML."""
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, image_size, None, None
        )

        if not ret:
            self.get_logger().error('Calibration failed!')
            return

        # Compute reprojection error
        total_error = 0
        for i in range(len(obj_points)):
            reproj, _ = cv2.projectPoints(
                obj_points[i], rvecs[i], tvecs[i],
                camera_matrix, dist_coeffs
            )
            error = cv2.norm(img_points[i], reproj, cv2.NORM_L2) / len(reproj)
            total_error += error
        mean_error = total_error / len(obj_points)

        self.get_logger().info(f'Calibration successful!  RMS: {ret:.4f}  '
                               f'Mean reproj error: {mean_error:.4f}px')

        # Build YAML (ROS CameraInfo compatible format)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        cal_data = {
            'image_width': image_size[0],
            'image_height': image_size[1],
            'camera_name': f'duburi_cam_video{self.device_id}',
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
                'data': np.eye(3).flatten().tolist(),
            },
            'projection_matrix': {
                'rows': 3, 'cols': 4,
                'data': np.zeros((3, 4)).flatten().tolist(),
            },
            'calibration_date': timestamp,
            'rms_error': float(ret),
            'mean_reprojection_error': float(mean_error),
            'num_images_used': len(obj_points),
        }

        # Fill projection matrix from camera_matrix
        P = np.zeros((3, 4))
        P[:3, :3] = camera_matrix
        cal_data['projection_matrix']['data'] = P.flatten().tolist()

        filename = os.path.join(
            self.output_dir,
            f'calibration_video{self.device_id}_{timestamp}.yaml'
        )
        with open(filename, 'w') as f:
            yaml.dump(cal_data, f, default_flow_style=False)

        self.get_logger().info(f'Calibration saved to: {filename}')

        print(f'\n  ── Calibration Results ──')
        print(f'  RMS Error:              {ret:.4f}')
        print(f'  Mean Reproj Error:      {mean_error:.4f}px')
        print(f'  Images Used:            {len(obj_points)}')
        print(f'  Camera Matrix:\n{camera_matrix}')
        print(f'  Distortion Coefficients: {dist_coeffs.flatten()}')
        print(f'  Saved to: {filename}\n')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibratorNode()
    node.run_calibration()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
