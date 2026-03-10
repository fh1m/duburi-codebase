"""
Image conversion utilities for Duburi 4.2 vision pipeline.

Drop-in replacements for cv_bridge's CvBridge.imgmsg_to_cv2() and
CvBridge.cv2_to_imgmsg().  Avoids the cv_bridge build dependency while
supporting common ROS image encodings.
"""

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def ros_image_to_cv2(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to OpenCV BGR numpy array."""
    encoding = msg.encoding.lower()
    dtype = np.uint8

    if encoding in ('bgr8', 'rgb8', '8uc3'):
        channels = 3
    elif encoding in ('bgra8', 'rgba8', '8uc4'):
        channels = 4
    elif encoding in ('mono8', '8uc1'):
        channels = 1
    elif encoding in ('16uc1', 'mono16'):
        channels = 1
        dtype = np.uint16
    else:
        # Try treating as BGR
        channels = 3

    img = np.frombuffer(msg.data, dtype=dtype)
    img = img.reshape((msg.height, msg.width, channels) if channels > 1
                      else (msg.height, msg.width))

    # Convert to BGR if needed
    if encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == 'rgba8':
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif encoding == 'bgra8':
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif encoding in ('mono8', '8uc1'):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    return img


def cv2_to_ros_image(frame: np.ndarray, header: Header) -> Image:
    """Convert OpenCV BGR frame to sensor_msgs/Image."""
    msg = Image()
    msg.header = Header()
    msg.header.stamp = header.stamp
    msg.header.frame_id = header.frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg
