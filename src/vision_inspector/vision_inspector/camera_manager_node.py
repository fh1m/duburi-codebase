"""
camera_manager_node.py – Multi-camera orchestrator for BRACU Duburi 4.2.

This is the main entry-point node for vision_inspector.  It reads
``cameras.yaml``, creates a CameraDevice + FramePublisher + CalibrationStore
for each configured camera, and runs three timer loops:

* **capture** (per-camera target FPS) – grab frames and publish Image/CameraInfo
* **health**  (1 Hz)                  – publish CameraStatusArray
* **reconnect** (2 s)                 – attempt reconnection for disconnected cameras

Topic layout (example for camera named ``forward``)::

    /camera/forward/image_raw       (sensor_msgs/Image)
    /camera/forward/camera_info     (sensor_msgs/CameraInfo)
    /vision_inspector/status        (duburi_interfaces/CameraStatusArray)

Parameters (node-level):
    config_file (str) – path to cameras.yaml, default uses package share
"""

from __future__ import annotations

import os
from typing import Dict, List

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from duburi_interfaces.msg import CameraStatus, CameraStatusArray
from vision_inspector.calibration_store import CalibrationStore
from vision_inspector.camera_device import CameraDevice
from vision_inspector.frame_publisher import FramePublisher


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Per-camera bundle
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class _CameraSlot:
    """Groups the three helpers that belong to one physical camera."""

    def __init__(
        self,
        name: str,
        device: CameraDevice,
        publisher: FramePublisher,
        calib_store: CalibrationStore,
        camera_info,
        calibrated: bool,
    ):
        self.name = name
        self.device = device
        self.publisher = publisher
        self.calib_store = calib_store
        self.camera_info = camera_info
        self.calibrated = calibrated


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Node
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class CameraManagerNode(Node):
    """Multi-camera orchestrator node."""

    def __init__(self):
        super().__init__('camera_manager')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value

        if not config_path:
            try:
                share_dir = get_package_share_directory('vision_inspector')
                config_path = os.path.join(share_dir, 'config', 'cameras.yaml')
            except Exception:
                config_path = ''

        # ── Load config ──────────────────────────────────────────
        self._camera_cfg = self._load_config(config_path)
        if not self._camera_cfg:
            self.get_logger().error(
                'No camera configuration found.  Pass config_file param '
                'or install cameras.yaml to the package share directory.'
            )

        # ── Status publisher ─────────────────────────────────────
        self._status_pub = self.create_publisher(
            CameraStatusArray, '/vision_inspector/status', 10
        )

        # ── Build per-camera slots ───────────────────────────────
        self._slots: List[_CameraSlot] = []
        for cfg in self._camera_cfg:
            slot = self._make_slot(cfg)
            self._slots.append(slot)
            self.get_logger().info(
                f'Camera [{slot.name}] configured  '
                f'devices={cfg.get("device_ids", [])}  '
                f'{cfg.get("width", 640)}x{cfg.get("height", 480)}'
                f'@{cfg.get("fps", 30)}fps'
            )

        # ── Auto-open all cameras ────────────────────────────────
        for slot in self._slots:
            self._try_open(slot)

        # ── Timers ───────────────────────────────────────────────
        # Capture timer runs at the fastest configured FPS
        max_fps = max((c.get('fps', 30) for c in self._camera_cfg), default=30)
        capture_period = 1.0 / max(max_fps, 1)
        self.create_timer(capture_period, self._capture_cb)

        # Health / status at 1 Hz
        self.create_timer(1.0, self._health_cb)

        # Reconnect sweep every 2 seconds
        self.create_timer(2.0, self._reconnect_cb)

        self.get_logger().info(
            f'CameraManager started – {len(self._slots)} camera(s) configured'
        )

    # ═════════════════════════════════════════════════════════════
    #  Config loading
    # ═════════════════════════════════════════════════════════════

    def _load_config(self, path: str) -> List[dict]:
        """Parse cameras.yaml and return a list of camera dicts."""
        if not path or not os.path.isfile(path):
            self.get_logger().warn(f'Config not found: {path}')
            return []
        try:
            import yaml
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            cameras = data.get('cameras', [])
            if not isinstance(cameras, list):
                self.get_logger().error('cameras key must be a list')
                return []
            return cameras
        except Exception as e:
            self.get_logger().error(f'Failed to parse config: {e}')
            return []

    # ═════════════════════════════════════════════════════════════
    #  Slot creation
    # ═════════════════════════════════════════════════════════════

    def _make_slot(self, cfg: dict) -> _CameraSlot:
        """Create CameraDevice + FramePublisher + CalibrationStore."""
        name = cfg.get('name', 'camera')
        device_ids = cfg.get('device_ids', [0])
        width = cfg.get('width', 640)
        height = cfg.get('height', 480)
        fps = cfg.get('fps', 30)
        v4l2_name = cfg.get('v4l2_name_pattern', None)
        calib_dir = cfg.get('calibration_dir', '')
        calib_file = cfg.get('calibration_file', '')
        frame_id = cfg.get('frame_id', f'{name}_camera')

        device = CameraDevice(
            name=name,
            device_ids=device_ids,
            width=width,
            height=height,
            fps=fps,
            v4l2_name_pattern=v4l2_name,
        )

        publisher = FramePublisher(
            node=self,
            topic_namespace=f'/camera/{name}',
            frame_id=frame_id,
        )

        calib_store = CalibrationStore(calibration_dir=calib_dir or None)

        # Attempt to load calibration
        camera_info, calibrated = calib_store.load(
            camera_name=name,
            width=width,
            height=height,
            explicit_path=calib_file or None,
        )
        if calibrated:
            self.get_logger().info(f'[{name}] Calibration loaded')
        else:
            self.get_logger().info(f'[{name}] No calibration – using zero-init')

        return _CameraSlot(
            name=name,
            device=device,
            publisher=publisher,
            calib_store=calib_store,
            camera_info=camera_info,
            calibrated=calibrated,
        )

    def _try_open(self, slot: _CameraSlot):
        """Open a camera device, log result."""
        ok = slot.device.open()
        if ok:
            self.get_logger().info(
                f'[{slot.name}] Opened {slot.device.device_path}  '
                f'{slot.device.resolution[0]}x{slot.device.resolution[1]}'
            )
        else:
            self.get_logger().warn(
                f'[{slot.name}] Failed to open – will retry in reconnect loop'
            )

    # ═════════════════════════════════════════════════════════════
    #  Timer callbacks
    # ═════════════════════════════════════════════════════════════

    def _capture_cb(self):
        """Grab a frame from every connected camera and publish."""
        for slot in self._slots:
            if not slot.device.is_connected:
                continue
            ok, frame = slot.device.read_frame()
            if ok and frame is not None:
                slot.publisher.publish_frame(frame, slot.camera_info)

    def _health_cb(self):
        """Publish CameraStatusArray at 1 Hz."""
        msg = CameraStatusArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vision_inspector'

        for slot in self._slots:
            status = CameraStatus()
            st = slot.device.get_status()

            status.camera_name = slot.name
            status.connected = st['connected']
            status.device_path = st.get('device_path', '')
            status.width = st.get('width', 0)
            status.height = st.get('height', 0)
            status.actual_fps = float(st.get('actual_fps', 0.0))
            status.frame_count = st.get('frame_count', 0)
            status.uptime = float(st.get('uptime', 0.0))
            status.calibrated = slot.calibrated
            status.last_error = st.get('last_error', '')

            msg.cameras.append(status)

        self._status_pub.publish(msg)

    def _reconnect_cb(self):
        """Attempt to reconnect any disconnected cameras."""
        for slot in self._slots:
            if slot.device.is_connected:
                continue
            ok = slot.device.attempt_reconnect()
            if ok:
                self.get_logger().info(
                    f'[{slot.name}] Reconnected to {slot.device.device_path}'
                )

    # ═════════════════════════════════════════════════════════════
    #  Shutdown
    # ═════════════════════════════════════════════════════════════

    def destroy_node(self):
        for slot in self._slots:
            slot.device.close()
            self.get_logger().info(f'[{slot.name}] Camera released')
        super().destroy_node()


# ─── Entry point ─────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
