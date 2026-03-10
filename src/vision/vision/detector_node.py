"""
detector_node.py – YOLO Object Detection ROS 2 Node for BRACU Duburi 4.2.

Subscribes to /camera/image_raw (sensor_msgs/Image), runs Ultralytics YOLO
inference, and publishes:
  - /vision/detections        (duburi_interfaces/DetectionArray)
  - /vision/annotated_image   (sensor_msgs/Image – with bounding boxes)

Also prints detections to the terminal.

Parameters:
    model (str)            – YOLO model file, default 'yolov8n.pt'
    confidence (float)     – Detection threshold, default 0.5
    device (str)           – 'cpu' or 'cuda:0', default 'cpu'
    image_topic (str)      – Input image topic, default '/camera/image_raw'
    enable_display (bool)  – Show OpenCV window, default False
    publish_annotated (bool) – Publish annotated image, default True
    max_det (int)          – Max detections per frame, default 50
    classes (str)          – Comma-separated class filter (empty = all)
    iou (float)            – NMS IoU threshold, default 0.45

Architecture note:
    This node is designed for easy future integration with mavlink controls.
    The /vision/detections topic publishes DetectionArray messages with
    normalised center coordinates (0.0-1.0), making it straightforward for
    a planning/behaviour node to convert detections into movement commands.
"""

import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from duburi_interfaces.msg import Detection, DetectionArray
from vision.image_utils import ros_image_to_cv2, cv2_to_ros_image


class DetectorNode(Node):
    """YOLO object detection node."""

    def __init__(self):
        super().__init__('detector_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('enable_display', False)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('max_det', 50)
        self.declare_parameter('classes', '')  # e.g. '0,1,2' or 'person,car'
        self.declare_parameter('iou', 0.45)

        self.model_path = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        self.device = self.get_parameter('device').value
        self.image_topic = self.get_parameter('image_topic').value
        self.enable_display = self.get_parameter('enable_display').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.max_det = self.get_parameter('max_det').value
        self.classes_filter = self.get_parameter('classes').value
        self.iou = self.get_parameter('iou').value

        # ── Load YOLO model ──────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model: {self.model_path}  device={self.device}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            self.get_logger().info('YOLO model loaded successfully.')
        except ImportError:
            self.get_logger().fatal(
                'ultralytics not installed!  Run: pip install ultralytics'
            )
            raise SystemExit(1)
        except Exception as e:
            self.get_logger().fatal(f'Failed to load YOLO model: {e}')
            raise SystemExit(1)

        # Parse class filter
        self._class_indices = None
        if self.classes_filter:
            try:
                self._class_indices = [int(c.strip()) for c in self.classes_filter.split(',')]
            except ValueError:
                # Might be class names – resolve after first inference
                self._class_names_filter = [c.strip().lower() for c in self.classes_filter.split(',')]
                self._class_indices = None

        # ── Publishers ───────────────────────────────────────────────
        self.det_pub = self.create_publisher(
            DetectionArray, '/vision/detections', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/vision/annotated_image', 10
        )

        # ── Subscriber ───────────────────────────────────────────────
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos
        )

        # ── Stats ────────────────────────────────────────────────────
        self.frame_count = 0
        self.total_detections = 0
        self.last_fps_time = time.monotonic()
        self.fps_frame_count = 0
        self.current_fps = 0.0

        self.get_logger().info(
            f'Detector node started.  Subscribing to: {self.image_topic}  '
            f'Confidence: {self.confidence}  IOU: {self.iou}'
        )

    # ═════════════════════════════════════════════════════════════════
    #  Image callback
    # ═════════════════════════════════════════════════════════════════

    def _image_callback(self, msg: Image):
        """Process incoming image: run YOLO, publish detections, annotate."""
        # ── Convert ROS Image → numpy ────────────────────────────────
        try:
            frame = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}',
                                   throttle_duration_sec=5.0)
            return

        h, w = frame.shape[:2]

        # ── Run YOLO inference ───────────────────────────────────────
        results = self.model(
            frame,
            conf=self.confidence,
            iou=self.iou,
            device=self.device,
            max_det=self.max_det,
            classes=self._class_indices,
            verbose=False,
        )

        result = results[0]

        # Resolve class name filter on first inference
        if hasattr(self, '_class_names_filter') and self._class_indices is None:
            name_to_id = {v.lower(): k for k, v in result.names.items()}
            self._class_indices = [
                name_to_id[n] for n in self._class_names_filter
                if n in name_to_id
            ]
            if self._class_indices:
                self.get_logger().info(
                    f'Resolved class filter: {self._class_names_filter} → {self._class_indices}'
                )

        # ── Build DetectionArray ─────────────────────────────────────
        det_array = DetectionArray()
        det_array.header = Header()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id
        det_array.image_width = w
        det_array.image_height = h

        boxes = result.boxes
        detection_strs = []

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                # Extract box data
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                cls_name = result.names[cls_id]

                x1, y1, x2, y2 = xyxy
                bx = int(x1)
                by = int(y1)
                bw = int(x2 - x1)
                bh = int(y2 - y1)
                cx = (x1 + x2) / 2.0 / w
                cy = (y1 + y2) / 2.0 / h

                det = Detection()
                det.class_name = cls_name
                det.class_id = cls_id
                det.confidence = conf
                det.bbox_x = bx
                det.bbox_y = by
                det.bbox_w = bw
                det.bbox_h = bh
                det.center_x = float(cx)
                det.center_y = float(cy)

                det_array.detections.append(det)
                detection_strs.append(f'{cls_name}({conf:.2f})')

        self.det_pub.publish(det_array)

        # ── Print to terminal ────────────────────────────────────────
        n = len(det_array.detections)
        self.total_detections += n
        self.frame_count += 1
        self.fps_frame_count += 1

        now = time.monotonic()
        if now - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_frame_count / (now - self.last_fps_time)
            self.fps_frame_count = 0
            self.last_fps_time = now

        if n > 0:
            det_summary = ', '.join(detection_strs)
            self.get_logger().info(
                f'[{self.current_fps:.1f}fps] Frame #{self.frame_count}: '
                f'{n} detection(s) – {det_summary}'
            )

        # ── Annotated image ──────────────────────────────────────────
        if self.publish_annotated or self.enable_display:
            annotated = result.plot(conf=True, line_width=2)

            # Add FPS overlay
            cv2.putText(annotated, f'FPS: {self.current_fps:.1f}  Det: {n}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if self.publish_annotated:
                ann_msg = cv2_to_ros_image(annotated, msg.header)
                self.annotated_pub.publish(ann_msg)

            if self.enable_display:
                cv2.imshow('Duburi Vision – YOLO Detections', annotated)
                cv2.waitKey(1)

    def destroy_node(self):
        if self.enable_display:
            cv2.destroyAllWindows()
        self.get_logger().info(
            f'Detector shutting down. Processed {self.frame_count} frames, '
            f'{self.total_detections} total detections.'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
