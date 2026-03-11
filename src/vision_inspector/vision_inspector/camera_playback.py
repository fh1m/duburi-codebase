"""
camera_playback.py – Replay a video file as ROS Image topics.

Publishes frames from a recorded video file to ROS topics at the
original (or custom) frame rate, simulating a live camera feed.
Useful for offline testing of the perception pipeline.

Usage:
    ros2 run vision_inspector camera_playback --ros-args \\
        -p video_file:=recordings/recording_20260303_120000.mp4

    ros2 run vision_inspector camera_playback --ros-args \\
        -p video_file:=recordings/test.avi \\
        -p topic_namespace:=/camera/forward -p loop:=true
"""

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from vision_inspector.image_utils import cv2_to_ros_image


class CameraPlaybackNode(Node):
    """Replays a video file as ROS Image messages."""

    def __init__(self):
        super().__init__('camera_playback')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('video_file', '')
        self.declare_parameter('topic_namespace', '/camera/playback')
        self.declare_parameter('frame_id', 'playback_camera')
        self.declare_parameter('fps', 0)   # 0 = use video's native FPS
        self.declare_parameter('loop', False)

        self.video_file = self.get_parameter('video_file').value
        topic_ns = self.get_parameter('topic_namespace').value.rstrip('/')
        self.frame_id = self.get_parameter('frame_id').value
        self.target_fps = self.get_parameter('fps').value
        self.loop = self.get_parameter('loop').value

        if not self.video_file:
            self.get_logger().error('No video_file specified. Exiting.')
            return

        # ── Open video ───────────────────────────────────────────
        self._cap = cv2.VideoCapture(self.video_file)
        if not self._cap.isOpened():
            self.get_logger().error(f'Cannot open video: {self.video_file}')
            return

        native_fps = self._cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = self.target_fps if self.target_fps > 0 else native_fps
        if fps <= 0:
            fps = 30.0

        self._image_pub = self.create_publisher(
            Image, f'{topic_ns}/image_raw', 10
        )

        self._frame_count = 0
        self._total_frames = total_frames

        period = 1.0 / fps
        self.create_timer(period, self._publish_cb)

        self.get_logger().info(
            f'Playback: {self.video_file}  '
            f'({total_frames} frames @ {native_fps:.1f}fps)  '
            f'Publishing at {fps:.1f}fps to {topic_ns}/image_raw  '
            f'loop={self.loop}'
        )

    def _publish_cb(self):
        if self._cap is None or not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret:
            if self.loop:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.get_logger().info('Looping video from start…')
                return
            else:
                self.get_logger().info('Playback finished.')
                rclpy.shutdown()
                return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        img_msg = cv2_to_ros_image(frame, header)
        self._image_pub.publish(img_msg)

        self._frame_count += 1
        if self._frame_count % 300 == 0:
            self.get_logger().info(
                f'Published {self._frame_count}/{self._total_frames} frames'
            )

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPlaybackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
