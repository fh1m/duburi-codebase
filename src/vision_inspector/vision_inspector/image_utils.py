"""
image_utils.py – ROS Image ↔ OpenCV conversion utilities.

Drop-in replacements for cv_bridge that avoid the C++ build dependency.
Supports bgr8, rgb8, bgra8, rgba8, mono8, and mono16 encodings.

Used by both vision_inspector (frame publishing) and vision (detection).
"""

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def ros_image_to_cv2(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image to an OpenCV BGR numpy array.

    Supports encodings: bgr8, rgb8, bgra8, rgba8, mono8, mono16, 8uc1/3/4,
    16uc1.  Unknown encodings are treated as bgr8 (best-effort).

    For mono16/16uc1 the raw uint16 single-channel array is returned
    *without* conversion to BGR, since colour conversion would lose
    depth information.  Callers should check ``img.dtype`` if they need
    to handle 16-bit images specially.

    Returns:
        np.ndarray with dtype uint8 (BGR, 3-channel) for 8-bit colour/mono,
        or dtype uint16 (single-channel) for 16-bit mono.
    """
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
        # Best-effort: treat as BGR
        channels = 3

    img = np.frombuffer(msg.data, dtype=dtype)
    img = img.reshape(
        (msg.height, msg.width, channels) if channels > 1
        else (msg.height, msg.width)
    )

    # Convert colour formats to BGR
    if encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == 'rgba8':
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif encoding == 'bgra8':
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    elif encoding in ('mono8', '8uc1'):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # mono16 / 16uc1: return raw single-channel uint16

    return img


def cv2_to_ros_image(frame: np.ndarray, header: Header = None) -> Image:
    """Convert an OpenCV BGR frame to a sensor_msgs/Image.

    Args:
        frame:  OpenCV image (uint8, 3-channel BGR).
        header: Optional ROS Header.  If None a blank header is used.

    Returns:
        sensor_msgs/Image with bgr8 encoding.
    """
    msg = Image()
    if header is not None:
        msg.header.stamp = header.stamp
        msg.header.frame_id = header.frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]

    if frame.ndim == 2:
        # Grayscale
        msg.encoding = 'mono8'
        msg.step = frame.shape[1]
    else:
        msg.encoding = 'bgr8'
        msg.step = frame.shape[1] * frame.shape[2]

    msg.is_bigendian = 0
    msg.data = frame.tobytes()
    return msg
