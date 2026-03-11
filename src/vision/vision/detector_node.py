"""
detector_node.py -- YOLO Object Detection ROS 2 Node for BRACU Duburi 4.2.

Subscribes to /camera/forward/image_raw (sensor_msgs/Image), runs Ultralytics
YOLO inference, and publishes:
  - /vision/detections        (duburi_interfaces/DetectionArray)
  - /vision/annotated_image   (sensor_msgs/Image -- with bounding boxes)

Uses Roboflow Supervision for annotation (BoundingBox, Labels, Trace).

Parameters:
    model (str)            -- YOLO model file, default 'yolo11n.pt'
    confidence (float)     -- Detection threshold, default 0.5
    device (str)           -- 'auto', 'cpu', or 'cuda:0' (auto uses GPU if available)
    image_topic (str)      -- Input image topic
    enable_display (bool)  -- Show OpenCV window, default False
    publish_annotated (bool) -- Publish annotated image, default True
    max_det (int)          -- Max detections per frame, default 50
    classes (str)          -- Comma-separated class filter (empty = all)
    iou (float)            -- NMS IoU threshold, default 0.45
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from duburi_interfaces.msg import AlignmentStatus, Detection, DetectionArray
from vision_inspector.image_utils import ros_image_to_cv2, cv2_to_ros_image

from . import config as C

_sv_available = False
try:
    import supervision as sv
    _sv_available = True
except ImportError:
    pass


class DetectorNode(Node):
    """YOLO object detection node with Supervision annotation."""

    def __init__(self):
        super().__init__("detector_node")

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter("model", C.DEFAULT_MODEL)
        self.declare_parameter("confidence", C.DEFAULT_CONFIDENCE)
        self.declare_parameter("device", C.DEFAULT_DEVICE)
        self.declare_parameter("image_topic", "/camera/forward/image_raw")
        self.declare_parameter("enable_display", False)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("max_det", C.DEFAULT_MAX_DET)
        self.declare_parameter("classes", "")
        self.declare_parameter("iou", C.DEFAULT_IOU)
        self.declare_parameter("target_class", C.DEFAULT_TARGET_CLASS)
        self.declare_parameter("show_alignment", True)
        self.declare_parameter("show_kalman", True)

        self.model_path = self.get_parameter("model").value
        self.confidence = self.get_parameter("confidence").value
        self.device = C.resolve_device(self.get_parameter("device").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.enable_display = self.get_parameter("enable_display").value
        self.publish_annotated = self.get_parameter("publish_annotated").value
        self.max_det = self.get_parameter("max_det").value
        self.classes_filter = self.get_parameter("classes").value
        self.iou = self.get_parameter("iou").value
        self.target_class = self.get_parameter("target_class").value
        self.show_alignment = self.get_parameter("show_alignment").value
        self.show_kalman = self.get_parameter("show_kalman").value

        # ── Load YOLO model ──────────────────────────────────────────
        self.get_logger().info(
            f"Loading YOLO model: {self.model_path}  device={self.device} "
            f"(requested={self.get_parameter('device').value})"
        )
        try:
            from ultralytics import YOLO

            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO model loaded successfully.")
        except ImportError:
            self.get_logger().fatal(
                "ultralytics not installed!  Run: pip install ultralytics"
            )
            raise SystemExit(1)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model: {e}")
            raise SystemExit(1)

        # Parse class filter
        self._class_indices = None
        self._class_names_filter: list[str] | None = None
        if self.classes_filter:
            try:
                self._class_indices = [
                    int(c.strip()) for c in self.classes_filter.split(",")
                ]
            except ValueError:
                self._class_names_filter = [
                    c.strip().lower() for c in self.classes_filter.split(",")
                ]

        # ── Supervision annotators ───────────────────────────────────
        if _sv_available:
            self._box_annotator = sv.BoxAnnotator(thickness=2)
            self._label_annotator = sv.LabelAnnotator(
                text_thickness=1, text_scale=0.5, text_padding=4,
            )
            self.get_logger().info("Supervision annotators ready.")
        else:
            self._box_annotator = None
            self._label_annotator = None
            self.get_logger().warn(
                "supervision not installed -- falling back to ultralytics plot(). "
                "Install with: pip install supervision"
            )

        # ── Publishers ───────────────────────────────────────────────
        self.det_pub = self.create_publisher(
            DetectionArray, "/vision/detections", 10
        )
        self.annotated_pub = self.create_publisher(
            Image, "/vision/annotated_image", 10
        )

        # ── Subscribers ──────────────────────────────────────────────
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos
        )
        qos_rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._alignment_sub = self.create_subscription(
            AlignmentStatus, "/vision/alignment_status",
            self._alignment_cb, qos_rel
        )
        self._last_alignment: AlignmentStatus | None = None
        self._kf_trail: list[tuple[float, float]] = []
        self._kf_trail_max = 30

        # ── Display window (created once) ─────────────────────────────
        self._display_created = False
        if self.enable_display:
            cv2.namedWindow("Duburi Vision", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Duburi Vision", 960, 540)
            self._display_created = True

        # ── Stats ────────────────────────────────────────────────────
        self.frame_count = 0
        self.total_detections = 0
        self.last_fps_time = time.monotonic()
        self.fps_frame_count = 0
        self.current_fps = 0.0

        self.get_logger().info(
            f"Detector node started.  Subscribing to: {self.image_topic}  "
            f"Confidence: {self.confidence}  IOU: {self.iou}"
        )

    # ═════════════════════════════════════════════════════════════════
    #  Image callback
    # ═════════════════════════════════════════════════════════════════

    def _image_callback(self, msg: Image):
        try:
            frame = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(
                f"Image conversion failed: {e}", throttle_duration_sec=5.0
            )
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
        if self._class_names_filter and self._class_indices is None:
            name_to_id = {v.lower(): k for k, v in result.names.items()}
            self._class_indices = [
                name_to_id[n]
                for n in self._class_names_filter
                if n in name_to_id
            ]
            if self._class_indices:
                self.get_logger().info(
                    f"Resolved class filter: {self._class_names_filter} "
                    f"-> {self._class_indices}"
                )

        # ── Build DetectionArray ─────────────────────────────────────
        det_array = DetectionArray()
        det_array.header = Header()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id
        det_array.image_width = w
        det_array.image_height = h

        boxes = result.boxes
        detection_log_lines: list[str] = []

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
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

                err_x = cx - C.FRAME_CENTER
                err_y = cy - C.FRAME_CENTER
                area_pct = (bw * bh) / max(w * h, 1) * 100.0
                detection_log_lines.append(
                    f"  {cls_name}({conf:.2f}) "
                    f"center=({cx:.3f},{cy:.3f}) "
                    f"err=({err_x:+.3f},{err_y:+.3f}) "
                    f"bbox={bw}x{bh} area={area_pct:.1f}%"
                )

        self.det_pub.publish(det_array)

        # ── Terminal logging ─────────────────────────────────────────
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
            header = (
                f"[{self.current_fps:.1f}fps] "
                f"Frame #{self.frame_count}: {n} detection(s)"
            )
            log_msg = header + "\n" + "\n".join(detection_log_lines)
            self.get_logger().info(log_msg)

        # ── Annotated image ──────────────────────────────────────────
        if self.publish_annotated or self.enable_display:
            annotated = self._annotate_frame(frame, result)

            cv2.putText(
                annotated,
                f"FPS: {self.current_fps:.1f}  Det: {n}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            # Alignment overlay: how much to move lat/dep (visual guide only)
            if self.show_alignment and n > 0:
                annotated = self._draw_alignment_overlay(annotated, result, w, h)

            # Kalman filter tracking box (from alignment_controller)
            if self.show_kalman:
                annotated = self._draw_kalman_overlay(annotated, w, h)

            if self.publish_annotated:
                ann_msg = cv2_to_ros_image(annotated, msg.header)
                self.annotated_pub.publish(ann_msg)

            if self.enable_display:
                if not self._display_created:
                    cv2.namedWindow("Duburi Vision", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("Duburi Vision", 960, 540)
                    self._display_created = True
                cv2.imshow("Duburi Vision", annotated)

    def _draw_alignment_overlay(
        self, img: np.ndarray, result, w: int, h: int
    ) -> np.ndarray:
        """Draw alignment guide: frame center, target center, lat/dep arrows and text."""
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return img

        # Pick biggest detection of target_class
        best = None
        best_area = -1.0
        for box in boxes:
            cls_id = int(box.cls[0].cpu().numpy())
            cls_name = result.names.get(cls_id, "")
            if cls_name.lower() != self.target_class.lower():
                continue
            xyxy = box.xyxy[0].cpu().numpy()
            area = (xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1])
            if area > best_area:
                best = (xyxy, cls_name)
                best_area = area

        if best is None:
            return img

        xyxy, _ = best
        x1, y1, x2, y2 = xyxy
        cx_px = (x1 + x2) / 2.0
        cy_px = (y1 + y2) / 2.0
        cx = cx_px / max(w, 1)
        cy = cy_px / max(h, 1)

        center_x = w / 2.0
        center_y = h / 2.0
        err_x = cx - 0.5  # + = object right of center -> move right
        err_y = cy - 0.5  # + = object below center -> move down

        # Colors
        cyan = (255, 255, 0)
        green = (0, 255, 0)
        red = (0, 0, 255)
        white = (255, 255, 255)

        # Frame center crosshair
        cs = 12
        cv2.line(img, (int(center_x) - cs, int(center_y)), (int(center_x) + cs, int(center_y)), cyan, 2)
        cv2.line(img, (int(center_x), int(center_y) - cs), (int(center_x), int(center_y) + cs), cyan, 2)
        cv2.circle(img, (int(center_x), int(center_y)), 4, cyan, 2)

        # Target center
        cv2.circle(img, (int(cx_px), int(cy_px)), 6, green, 2)

        # Line from target to center
        cv2.line(img, (int(cx_px), int(cy_px)), (int(center_x), int(center_y)), (100, 255, 100), 1)

        # Lateral arrow: horizontal component
        ax_len = 40
        lat_dir = 1 if err_x > 0 else -1
        lat_pct = abs(err_x) * 100
        ax_start = (int(center_x), int(center_y) + 60)
        ax_end = (int(center_x) + lat_dir * ax_len, int(center_y) + 60)
        cv2.arrowedLine(img, ax_start, ax_end, red, 3, tipLength=0.3)
        lat_text = f"LAT: move {'right' if err_x > 0 else 'left'} {lat_pct:.0f}%"
        cv2.putText(img, lat_text, (10, h - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, white, 1)

        # Depth arrow: vertical component
        dep_dir = 1 if err_y > 0 else -1
        dep_pct = abs(err_y) * 100
        ay_start = (int(center_x) + 80, int(center_y))
        ay_end = (int(center_x) + 80, int(center_y) + dep_dir * ax_len)
        cv2.arrowedLine(img, ay_start, ay_end, red, 3, tipLength=0.3)
        dep_text = f"DEP: move {'down' if err_y > 0 else 'up'} {dep_pct:.0f}%"
        cv2.putText(img, dep_text, (10, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, white, 1)

        # Dead zone indicator
        dz = int(0.05 * min(w, h))
        cv2.rectangle(
            img,
            (int(center_x) - dz, int(center_y) - dz),
            (int(center_x) + dz, int(center_y) + dz),
            (100, 100, 100),
            1,
        )
        cv2.putText(img, "align zone", (int(center_x) - dz, int(center_y) - dz - 4),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        return img

    def _alignment_cb(self, msg: AlignmentStatus) -> None:
        """Store latest alignment status for Kalman overlay."""
        self._last_alignment = msg
        if msg.target_detected and msg.use_kalman:
            self._kf_trail.append((msg.target_center_x, msg.target_center_y))
            if len(self._kf_trail) > self._kf_trail_max:
                self._kf_trail.pop(0)
        else:
            self._kf_trail.clear()

    def _draw_kalman_overlay(self, img: np.ndarray, w: int, h: int) -> np.ndarray:
        """Draw Kalman-filtered tracking box using Supervision."""
        if not _sv_available or self._last_alignment is None:
            return img
        status = self._last_alignment
        if not status.target_detected or not status.use_kalman:
            return img

        cx = status.target_center_x
        cy = status.target_center_y
        tw = status.target_width if status.target_width > 0 else 0.0
        th = status.target_height if status.target_height > 0 else 0.0
        if tw <= 0 or th <= 0:
            area = status.target_area_ratio
            if area <= 0:
                return img
            side = (area * w * h) ** 0.5
            tw = side / w
            th = side / h

        cx_px = cx * w
        cy_px = cy * h
        hw = (tw * w) / 2.0
        hh = (th * h) / 2.0
        x1 = max(0, cx_px - hw)
        y1 = max(0, cy_px - hh)
        x2 = min(w, cx_px + hw)
        y2 = min(h, cy_px + hh)
        if x2 <= x1 or y2 <= y1:
            return img

        xyxy = np.array([[x1, y1, x2, y2]], dtype=np.float32)
        kf_detections = sv.Detections(
            xyxy=xyxy,
            confidence=np.array([1.0]),
            class_id=np.array([0]),
        )
        color = sv.Color.from_hex("#FF00FF") if status.kalman_predicted else sv.Color.from_hex("#00FFFF")
        label = "KF pred" if status.kalman_predicted else "KF"
        box_annotator = sv.BoxAnnotator(color=color, thickness=2)
        label_annotator = sv.LabelAnnotator(text_scale=0.5, text_thickness=1)
        img = box_annotator.annotate(scene=img, detections=kf_detections)
        img = label_annotator.annotate(
            scene=img, detections=kf_detections, labels=[label]
        )

        # Kalman trail (path of tracked center)
        if len(self._kf_trail) >= 2:
            pts = [
                (int(x * w), int(y * h))
                for x, y in self._kf_trail
            ]
            cv2.polylines(img, [np.array(pts)], False, (255, 255, 0), 2)

        return img

    def _annotate_frame(self, frame: np.ndarray, result) -> np.ndarray:
        """Draw detections using Supervision (or ultralytics fallback)."""
        if _sv_available and self._box_annotator is not None:
            detections = sv.Detections.from_ultralytics(result)
            annotated = frame.copy()

            annotated = self._box_annotator.annotate(
                scene=annotated, detections=detections,
            )

            if len(detections) > 0:
                labels = [
                    f"{result.names[cid]} {conf:.2f}"
                    for cid, conf in zip(
                        detections.class_id, detections.confidence
                    )
                ]
                annotated = self._label_annotator.annotate(
                    scene=annotated, detections=detections, labels=labels,
                )

            return annotated

        return result.plot(conf=True, line_width=2)

    def destroy_node(self):
        if self.enable_display:
            cv2.destroyAllWindows()
        self.get_logger().info(
            f"Detector shutting down. Processed {self.frame_count} frames, "
            f"{self.total_detections} total detections."
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        if node.enable_display:
            # Manual spin loop so cv2.waitKey can pump the GUI event queue
            # on the main thread.  rclpy.spin() would block and freeze the
            # OpenCV window.
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    node.get_logger().info("Quit requested from display window.")
                    break
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
