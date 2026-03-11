"""
frame_publisher.py – Publishes OpenCV frames as ROS Image + CameraInfo.

Each CameraDevice has an associated FramePublisher that handles the
ROS side: creating ``sensor_msgs/Image`` and ``sensor_msgs/CameraInfo``
messages and publishing them to namespaced topics.

Topic layout for a camera named ``forward``::

    /camera/forward/image_raw      (sensor_msgs/Image)
    /camera/forward/camera_info    (sensor_msgs/CameraInfo)
"""

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from vision_inspector.image_utils import cv2_to_ros_image


class FramePublisher:
    """Converts frames to ROS messages and publishes to namespaced topics.

    Args:
        node:             Parent ROS node (provides publisher creation + clock).
        topic_namespace:  Topic prefix, e.g. ``'/camera/forward'``.
        frame_id:         TF frame id for the Image/CameraInfo headers.
        qos_depth:        Publisher queue depth (default 10).
    """

    def __init__(
        self,
        node: Node,
        topic_namespace: str,
        frame_id: str = 'camera',
        qos_depth: int = 10,
    ):
        self._node = node
        self._frame_id = frame_id

        # Normalise namespace (strip trailing slash)
        ns = topic_namespace.rstrip('/')

        self._image_topic = f'{ns}/image_raw'
        self._info_topic = f'{ns}/camera_info'

        self._image_pub = node.create_publisher(
            Image, self._image_topic, qos_depth
        )
        self._info_pub = node.create_publisher(
            CameraInfo, self._info_topic, qos_depth
        )

    # ─── Public API ──────────────────────────────────────────────

    def publish_frame(
        self, frame: np.ndarray, camera_info: CameraInfo
    ):
        """Convert an OpenCV frame to Image msg and publish both topics.

        Args:
            frame:       BGR uint8 numpy array from ``CameraDevice.read_frame()``.
            camera_info: CameraInfo message (from ``CalibrationStore.load()``).
        """
        stamp = self._node.get_clock().now().to_msg()

        header = Header()
        header.stamp = stamp
        header.frame_id = self._frame_id

        # Build and publish Image
        img_msg = cv2_to_ros_image(frame, header)
        self._image_pub.publish(img_msg)

        # Stamp and publish CameraInfo
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = self._frame_id
        self._info_pub.publish(camera_info)

    @property
    def image_topic(self) -> str:
        return self._image_topic

    @property
    def info_topic(self) -> str:
        return self._info_topic
