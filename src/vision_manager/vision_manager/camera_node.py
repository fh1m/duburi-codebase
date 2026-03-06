"""
camera_node.py – Main camera streaming node for BRACU Duburi 4.2.

Captures frames from a V4L2 camera device and publishes them as
sensor_msgs/Image on /camera/image_raw, along with sensor_msgs/CameraInfo
on /camera/camera_info.

Parameters:
    device_id (int)         – V4L2 device index, default 0
    frame_width (int)       – Capture width, default 640
    frame_height (int)      – Capture height, default 480
    fps (int)               – Target framerate, default 30
    camera_name (str)       – Logical camera name, default 'duburi_cam'
    calibration_file (str)  – Path to YAML calibration file (optional)
    auto_exposure (bool)    – Let the camera handle exposure, default True
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class CameraNode(Node):
    """Streams frames from a single V4L2 camera over ROS 2 topics."""

    def __init__(self):
        super().__init__('camera_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_name', 'duburi_cam')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('auto_exposure', True)

        self.device_id = self.get_parameter('device_id').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.camera_name = self.get_parameter('camera_name').value
        self.calib_file = self.get_parameter('calibration_file').value
        self.auto_exposure = self.get_parameter('auto_exposure').value

        # ── Publishers ───────────────────────────────────────────────
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # ── Camera info (calibration) ────────────────────────────────
        self.camera_info_msg = self._load_camera_info()

        # ── Open camera ──────────────────────────────────────────────
        self.cap = None
        self._open_camera()

        # ── Timer-driven capture ─────────────────────────────────────
        period = 1.0 / max(self.fps, 1)
        self.timer = self.create_timer(period, self._capture_callback)
        self.frame_count = 0

        self.get_logger().info(
            f'Camera node started: device={self.device_id}  '
            f'{self.frame_w}x{self.frame_h}@{self.fps}fps  '
            f'name={self.camera_name}'
        )

    # ═════════════════════════════════════════════════════════════════
    #  Camera lifecycle
    # ═════════════════════════════════════════════════════════════════

    def _open_camera(self):
        """Open the V4L2 capture device."""
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open /dev/video{self.device_id}. '
                'Check connection with:  v4l2-ctl --list-devices'
            )
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        if actual_w != self.frame_w or actual_h != self.frame_h:
            self.get_logger().warn(
                f'Requested {self.frame_w}x{self.frame_h} but camera '
                f'negotiated {actual_w}x{actual_h}'
            )
            self.frame_w = actual_w
            self.frame_h = actual_h

        self.get_logger().info(
            f'Camera opened: /dev/video{self.device_id}  '
            f'{actual_w}x{actual_h}@{actual_fps:.1f}fps'
        )

    def _capture_callback(self):
        """Grab a frame and publish it."""
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed', throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()

        # ── Build sensor_msgs/Image (bgr8 encoding) ─────────────────
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.camera_name
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = 0
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()

        self.image_pub.publish(img_msg)

        # ── Publish CameraInfo alongside ─────────────────────────────
        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = self.camera_name
        self.info_pub.publish(self.camera_info_msg)

        self.frame_count += 1
        if self.frame_count % (self.fps * 10) == 0:
            self.get_logger().info(
                f'Published {self.frame_count} frames',
                throttle_duration_sec=30.0
            )

    # ═════════════════════════════════════════════════════════════════
    #  Camera calibration info
    # ═════════════════════════════════════════════════════════════════

    def _load_camera_info(self) -> CameraInfo:
        """Load calibration from YAML file or return zero-initialised info."""
        info = CameraInfo()
        info.width = self.frame_w
        info.height = self.frame_h

        if self.calib_file and os.path.isfile(self.calib_file):
            try:
                import yaml
                with open(self.calib_file, 'r') as f:
                    cal = yaml.safe_load(f)

                info.width = cal.get('image_width', self.frame_w)
                info.height = cal.get('image_height', self.frame_h)
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

                self.get_logger().info(f'Loaded calibration from {self.calib_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load calibration: {e}')
        else:
            if self.calib_file:
                self.get_logger().warn(
                    f'Calibration file not found: {self.calib_file}  '
                    '(publishing zero-initialised CameraInfo)'
                )
            # Zero-initialised – identity K
            info.distortion_model = 'plumb_bob'
            info.k = [0.0] * 9
            info.d = [0.0] * 5
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [0.0] * 12

        return info

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info('Camera released.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
