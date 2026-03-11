"""
camera_enumerator.py – Discovers all V4L2 camera devices on the system.

Standalone CLI tool AND a ROS 2 node that publishes camera list on startup.
Checks /dev/video* devices, queries capabilities via V4L2/OpenCV, and reports:
  - Device path (/dev/videoN)
  - Device name / driver
  - Supported resolutions
  - Whether the device can capture video

Usage:
    ros2 run vision_inspector camera_enum
"""

import glob
import subprocess

import cv2
import rclpy
from rclpy.node import Node


def enumerate_cameras() -> list:
    """
    Enumerate all V4L2 video devices and return a list of dicts:
        {
            'device': '/dev/video0',
            'index': 0,
            'name': 'USB Camera',
            'driver': 'uvcvideo',
            'can_capture': True,
            'resolutions': ['640x480', '1280x720', ...],
        }
    """
    devices = sorted(glob.glob('/dev/video*'))
    cameras = []

    for dev_path in devices:
        try:
            idx = int(dev_path.replace('/dev/video', ''))
        except ValueError:
            continue

        info = {
            'device': dev_path,
            'index': idx,
            'name': 'unknown',
            'driver': 'unknown',
            'can_capture': False,
            'resolutions': [],
        }

        # ── Try v4l2-ctl for detailed info ───────────────────────
        try:
            result = subprocess.run(
                ['v4l2-ctl', '--device', dev_path, '--info'],
                capture_output=True, text=True, timeout=3
            )
            for line in result.stdout.splitlines():
                line = line.strip()
                if line.startswith('Card type'):
                    info['name'] = line.split(':', 1)[1].strip()
                elif line.startswith('Driver name'):
                    info['driver'] = line.split(':', 1)[1].strip()
                elif 'Video Capture' in line:
                    info['can_capture'] = True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass

        # ── Try opening with OpenCV to verify ────────────────────
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            info['can_capture'] = True
            # Probe common resolutions
            for w, h in [(320, 240), (640, 480), (800, 600),
                         (1280, 720), (1920, 1080)]:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                res = f'{aw}x{ah}'
                if res not in info['resolutions']:
                    info['resolutions'].append(res)
            cap.release()

        # ── Try v4l2-ctl for supported formats ───────────────────
        if not info['resolutions']:
            try:
                result = subprocess.run(
                    ['v4l2-ctl', '--device', dev_path, '--list-formats-ext'],
                    capture_output=True, text=True, timeout=3
                )
                for line in result.stdout.splitlines():
                    line = line.strip()
                    if 'Size:' in line:
                        parts = line.split()
                        for p in parts:
                            if 'x' in p and p[0].isdigit():
                                if p not in info['resolutions']:
                                    info['resolutions'].append(p)
            except (FileNotFoundError, subprocess.TimeoutExpired):
                pass

        cameras.append(info)

    return cameras


def print_camera_table(cameras: list):
    """Pretty-print the camera list."""
    if not cameras:
        print('\n  No V4L2 cameras detected.')
        print('  Check connections with:  ls /dev/video*  or  v4l2-ctl --list-devices\n')
        return

    print(f'\n  Found {len(cameras)} video device(s):\n')
    print(f'  {"Device":<16} {"Name":<30} {"Driver":<15} {"Capture":<9} {"Resolutions"}')
    print(f'  {"─" * 16} {"─" * 30} {"─" * 15} {"─" * 9} {"─" * 30}')

    for cam in cameras:
        cap_str = 'YES' if cam['can_capture'] else 'no'
        res_str = ', '.join(cam['resolutions'][:5]) or '—'
        print(f'  {cam["device"]:<16} {cam["name"]:<30} {cam["driver"]:<15} {cap_str:<9} {res_str}')

    capturable = [c for c in cameras if c['can_capture']]
    print(f'\n  Capturable cameras: {len(capturable)}/{len(cameras)}\n')


class CameraEnumeratorNode(Node):
    """ROS 2 node that enumerates cameras once on startup and logs results."""

    def __init__(self):
        super().__init__('camera_enumerator')
        cameras = enumerate_cameras()

        if not cameras:
            self.get_logger().warn('No V4L2 cameras detected on this system.')
        else:
            for cam in cameras:
                status = 'OK' if cam['can_capture'] else 'NO CAPTURE'
                self.get_logger().info(
                    f'{cam["device"]}  {cam["name"]}  '
                    f'driver={cam["driver"]}  [{status}]  '
                    f'res={cam["resolutions"]}'
                )
            capturable = sum(1 for c in cameras if c['can_capture'])
            self.get_logger().info(
                f'Total: {len(cameras)} devices, {capturable} capturable'
            )

        print_camera_table(cameras)


def main(args=None):
    rclpy.init(args=args)
    node = CameraEnumeratorNode()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
