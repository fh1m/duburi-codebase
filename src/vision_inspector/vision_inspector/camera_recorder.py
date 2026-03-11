"""
camera_recorder.py – Record camera feed to video file.

Records from either a live V4L2 device or a ROS Image topic to a video
file (MP4/AVI).  Useful for collecting datasets and debugging.

Usage (device mode):
    ros2 run vision_inspector camera_record --ros-args -p device_id:=0

Usage (topic mode):
    ros2 run vision_inspector camera_record --ros-args \\
        -p source:=topic -p image_topic:=/camera/forward/image_raw
"""

import os
import time
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from vision_inspector.image_utils import ros_image_to_cv2


class CameraRecorderNode(Node):
    """Records camera frames to a video file."""

    def __init__(self):
        super().__init__('camera_recorder')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('source', 'device')  # 'device' or 'topic'
        self.declare_parameter('device_id', 0)
        self.declare_parameter('image_topic', '/camera/forward/image_raw')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('output_dir', 'recordings')
        self.declare_parameter('codec', 'mp4v')
        self.declare_parameter('duration', 0)  # seconds, 0 = until Ctrl-C

        self.source = self.get_parameter('source').value
        self.device_id = self.get_parameter('device_id').value
        self.image_topic = self.get_parameter('image_topic').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.output_dir = self.get_parameter('output_dir').value
        self.codec = self.get_parameter('codec').value
        self.duration = self.get_parameter('duration').value

        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        ext = 'mp4' if self.codec == 'mp4v' else 'avi'
        self._output_path = os.path.join(
            self.output_dir, f'recording_{timestamp}.{ext}'
        )

        self._writer = None
        self._frame_count = 0
        self._start_time = None

        if self.source == 'topic':
            self._init_topic_mode()
        else:
            self._init_device_mode()

    # ─── Device mode ─────────────────────────────────────────────

    def _init_device_mode(self):
        """Open V4L2 device and start timer-driven recording."""
        self._cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self.get_logger().error(
                f'Cannot open /dev/video{self.device_id}'
            )
            return

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.frame_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self._open_writer()
        self._start_time = time.monotonic()

        period = 1.0 / max(self.fps, 1)
        self.create_timer(period, self._device_cb)

        self.get_logger().info(
            f'Recording /dev/video{self.device_id} → {self._output_path}'
        )

    def _device_cb(self):
        ret, frame = self._cap.read()
        if not ret:
            return
        self._write_frame(frame)

    # ─── Topic mode ──────────────────────────────────────────────

    def _init_topic_mode(self):
        """Subscribe to ROS Image topic."""
        self._cap = None
        self.create_subscription(Image, self.image_topic, self._topic_cb, 10)
        self.get_logger().info(
            f'Recording {self.image_topic} → {self._output_path}'
        )

    def _topic_cb(self, msg: Image):
        frame = ros_image_to_cv2(msg)
        if frame is None:
            return
        if self._writer is None:
            self.frame_h, self.frame_w = frame.shape[:2]
            self._open_writer()
            self._start_time = time.monotonic()
        self._write_frame(frame)

    # ─── Common ──────────────────────────────────────────────────

    def _open_writer(self):
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self._writer = cv2.VideoWriter(
            self._output_path, fourcc, self.fps,
            (self.frame_w, self.frame_h)
        )

    def _write_frame(self, frame):
        if self._writer is None:
            return
        self._writer.write(frame)
        self._frame_count += 1

        if self._frame_count % (self.fps * 10) == 0:
            elapsed = time.monotonic() - self._start_time
            self.get_logger().info(
                f'Recorded {self._frame_count} frames  ({elapsed:.0f}s)'
            )

        # Duration limit
        if self.duration > 0 and self._start_time:
            if (time.monotonic() - self._start_time) >= self.duration:
                self.get_logger().info('Duration limit reached – stopping.')
                self._finalise()
                rclpy.shutdown()

    def _finalise(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None

        elapsed = time.monotonic() - (self._start_time or time.monotonic())
        self.get_logger().info(
            f'Recording complete: {self._frame_count} frames, '
            f'{elapsed:.1f}s → {self._output_path}'
        )

    def destroy_node(self):
        self._finalise()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
