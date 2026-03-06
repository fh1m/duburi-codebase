"""
detector_standalone.py – Standalone YOLO detector with direct camera access.

A quick-test tool that opens a camera, runs YOLO inference, draws bounding
boxes with OpenCV, and prints detections to the terminal.  Does NOT require
the camera_node to be running – useful for verifying YOLO + camera work
before wiring up the full ROS pipeline.

Usage:
    ros2 run vision detector_standalone
    ros2 run vision detector_standalone --ros-args \
        -p device_id:=0 -p model:=yolov8n.pt -p confidence:=0.5

Controls:
    [q] – quit
    [s] – save annotated snapshot
    [+/-] – increase/decrease confidence threshold
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node


class DetectorStandaloneNode(Node):
    """Standalone YOLO detection with direct camera access."""

    def __init__(self):
        super().__init__('detector_standalone')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('iou', 0.45)

        self.device_id = self.get_parameter('device_id').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.model_path = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        self.yolo_device = self.get_parameter('device').value
        self.iou = self.get_parameter('iou').value

    def run(self):
        """Run standalone detection loop (blocking)."""
        # Load YOLO
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        try:
            from ultralytics import YOLO
            model = YOLO(self.model_path)
        except ImportError:
            self.get_logger().fatal('ultralytics not installed! Run: pip install ultralytics')
            return
        self.get_logger().info('Model loaded.')

        # Open camera
        cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open /dev/video{self.device_id}')
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        window_name = 'Duburi YOLO Standalone'
        frame_count = 0
        start_time = time.monotonic()
        snapshot_count = 0

        print(f'\n  ── Standalone YOLO Detector ──')
        print(f'  Camera: /dev/video{self.device_id}  {actual_w}x{actual_h}')
        print(f'  Model:  {self.model_path}  Device: {self.yolo_device}')
        print(f'  Conf:   {self.confidence}  IOU: {self.iou}')
        print(f'  Controls: [q]uit  [s]ave  [+/-] confidence\n')

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                frame_count += 1
                t0 = time.monotonic()

                # Run YOLO
                results = model(
                    frame,
                    conf=self.confidence,
                    iou=self.iou,
                    device=self.yolo_device,
                    verbose=False,
                )
                result = results[0]
                inference_ms = (time.monotonic() - t0) * 1000

                # Draw annotations
                annotated = result.plot(conf=True, line_width=2)

                # Count detections
                boxes = result.boxes
                n_det = len(boxes) if boxes is not None else 0

                elapsed = time.monotonic() - start_time
                fps_display = frame_count / max(elapsed, 0.001)

                # Overlay info
                info = (f'FPS: {fps_display:.1f}  Inference: {inference_ms:.0f}ms  '
                        f'Det: {n_det}  Conf: {self.confidence:.2f}')
                cv2.putText(annotated, info, (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                cv2.putText(annotated, '[q]uit [s]ave [+/-]conf',
                            (10, actual_h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                # Print detections
                if n_det > 0:
                    det_strs = []
                    for box in boxes:
                        cls_id = int(box.cls[0].cpu().numpy())
                        conf = float(box.conf[0].cpu().numpy())
                        name = result.names[cls_id]
                        det_strs.append(f'{name}({conf:.2f})')
                    print(f'  Frame #{frame_count}: {", ".join(det_strs)}')

                cv2.imshow(window_name, annotated)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    fn = f'duburi_yolo_snapshot_{snapshot_count:03d}.png'
                    cv2.imwrite(fn, annotated)
                    snapshot_count += 1
                    print(f'  Saved: {fn}')
                elif key == ord('+') or key == ord('='):
                    self.confidence = min(0.95, self.confidence + 0.05)
                    print(f'  Confidence: {self.confidence:.2f}')
                elif key == ord('-'):
                    self.confidence = max(0.05, self.confidence - 0.05)
                    print(f'  Confidence: {self.confidence:.2f}')

        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            cv2.destroyAllWindows()

        elapsed = time.monotonic() - start_time
        avg_fps = frame_count / max(elapsed, 0.001)
        print(f'\n  ── Summary ──')
        print(f'  Frames: {frame_count}  Duration: {elapsed:.1f}s  Avg FPS: {avg_fps:.1f}\n')


def main(args=None):
    rclpy.init(args=args)
    node = DetectorStandaloneNode()
    node.run()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
