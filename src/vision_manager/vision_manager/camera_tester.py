"""
camera_tester.py – Interactive camera test tool for BRACU Duburi 4.2.

Opens a camera, displays a live preview via OpenCV highgui, and reports
frame rate, resolution, and basic health. Press 'q' to quit, 's' to save
a snapshot, 'i' to print device info.

Usage:
    ros2 run vision_manager camera_test
    ros2 run vision_manager camera_test --ros-args -p device_id:=1
"""

import time

import cv2
import rclpy
from rclpy.node import Node


class CameraTesterNode(Node):
    """Interactive camera tester with OpenCV preview window."""

    def __init__(self):
        super().__init__('camera_tester')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('duration', 0)  # 0 = run until quit

        self.device_id = self.get_parameter('device_id').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.duration = self.get_parameter('duration').value

        self.get_logger().info(f'Testing /dev/video{self.device_id}  '
                               f'{self.frame_w}x{self.frame_h}@{self.fps}fps')

    def run_test(self):
        """Run the camera test loop (blocking)."""
        cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(
                f'Cannot open /dev/video{self.device_id}. '
                'Run:  ros2 run vision_manager camera_enum  to list devices.'
            )
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f'Camera opened: {actual_w}x{actual_h}@{actual_fps:.1f}fps')
        print(f'\n  Camera Test – /dev/video{self.device_id}')
        print(f'  Resolution: {actual_w}x{actual_h}  FPS: {actual_fps:.1f}')
        print(f'  Controls: [q]uit  [s]ave snapshot  [i]nfo\n')

        window_name = f'Duburi Camera Test – /dev/video{self.device_id}'
        frame_count = 0
        start_time = time.monotonic()
        fps_display = 0.0
        last_fps_time = start_time
        snapshot_count = 0
        success = True

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().error('Frame capture failed.')
                    success = False
                    break

                frame_count += 1
                now = time.monotonic()

                # Calculate FPS every second
                if now - last_fps_time >= 1.0:
                    fps_display = frame_count / (now - start_time)
                    last_fps_time = now

                # Draw info overlay
                info_text = (
                    f'FPS: {fps_display:.1f}  |  '
                    f'{actual_w}x{actual_h}  |  '
                    f'Frame #{frame_count}'
                )
                cv2.putText(frame, info_text, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, '[q]uit  [s]ave  [i]nfo', (10, actual_h - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                cv2.imshow(window_name, frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f'duburi_snapshot_{snapshot_count:03d}.png'
                    cv2.imwrite(filename, frame)
                    snapshot_count += 1
                    self.get_logger().info(f'Snapshot saved: {filename}')
                elif key == ord('i'):
                    elapsed = now - start_time
                    print(f'\n  ── Camera Info ──')
                    print(f'  Device:     /dev/video{self.device_id}')
                    print(f'  Resolution: {actual_w}x{actual_h}')
                    print(f'  Target FPS: {self.fps}   Actual FPS: {fps_display:.1f}')
                    print(f'  Frames:     {frame_count}')
                    print(f'  Uptime:     {elapsed:.1f}s\n')

                # Duration limit
                if self.duration > 0 and (now - start_time) >= self.duration:
                    self.get_logger().info(f'Test duration ({self.duration}s) reached.')
                    break

        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            cv2.destroyAllWindows()

        elapsed = time.monotonic() - start_time
        avg_fps = frame_count / max(elapsed, 0.001)

        print(f'\n  ── Test Results ──')
        print(f'  Total frames: {frame_count}')
        print(f'  Duration:     {elapsed:.1f}s')
        print(f'  Average FPS:  {avg_fps:.1f}')
        print(f'  Snapshots:    {snapshot_count}')
        print(f'  Status:       {"PASS" if success else "FAIL"}\n')

        return success


def main(args=None):
    rclpy.init(args=args)
    node = CameraTesterNode()
    node.run_test()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
