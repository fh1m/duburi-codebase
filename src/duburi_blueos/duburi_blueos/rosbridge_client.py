#!/usr/bin/env python3
"""
ROSBridge WebSocket Client for BlueOS ROS1 Extension.

Connects to rosbridge_websocket running on BlueOS (Pi) and provides:
- Topic subscription with callbacks
- Topic publishing
- Service calls
- Access to all MAVROS topics from ROS1

This allows ROS2 nodes to communicate with ROS1 MAVROS on BlueOS
without needing ros1_bridge or ROS1 installed locally.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional
from queue import Queue, Empty

try:
    import websocket
except ImportError:
    websocket = None


class ROSBridgeError(Exception):
    """Exception raised for ROSBridge errors."""
    pass


@dataclass
class ROSMessage:
    """Container for a ROS message received via rosbridge."""
    topic: str
    msg_type: str
    data: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)


class ROSBridgeClient:
    """
    WebSocket client for rosbridge_server.

    Connects to ROS1 rosbridge_websocket on BlueOS and provides
    pub/sub/service functionality.

    Usage:
        client = ROSBridgeClient('192.168.2.2', 8889)
        client.connect()

        # Subscribe to topic
        def callback(msg):
            print(f"State: armed={msg['armed']}, mode={msg['mode']}")
        client.subscribe('/mavros/state', 'mavros_msgs/State', callback)

        # Publish to topic
        client.publish('/mavros/rc/override', 'mavros_msgs/OverrideRCIn', {...})

        # Call service
        result = client.call_service('/mavros/cmd/arming', {'value': True})

        client.disconnect()
    """

    def __init__(
        self,
        host: str = '192.168.2.2',
        port: int = 8889,
        reconnect: bool = True,
        reconnect_interval: float = 2.0,
    ):
        if websocket is None:
            raise ROSBridgeError(
                "websocket-client not installed. Run: pip install websocket-client"
            )

        self.host = host
        self.port = port
        self.url = f"ws://{host}:{port}"
        self.reconnect = reconnect
        self.reconnect_interval = reconnect_interval

        self._ws: Optional[websocket.WebSocketApp] = None
        self._connected = False
        self._subscriptions: Dict[str, List[Callable]] = {}
        self._topic_types: Dict[str, str] = {}
        self._service_results: Dict[str, Queue] = {}
        self._service_id = 0
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._shutdown = False

        # Callbacks
        self.on_connect: Optional[Callable] = None
        self.on_disconnect: Optional[Callable] = None
        self.on_error: Optional[Callable[[str], None]] = None

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self, blocking: bool = False, timeout: float = 10.0) -> bool:
        """
        Connect to rosbridge server.

        Args:
            blocking: If True, wait for connection before returning.
            timeout: Connection timeout in seconds (only if blocking).

        Returns:
            True if connected (or connection started if non-blocking).
        """
        if self._connected:
            return True

        self._shutdown = False
        self._ws = websocket.WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        if blocking:
            start = time.time()
            while not self._connected and (time.time() - start) < timeout:
                time.sleep(0.1)
            return self._connected

        return True

    def disconnect(self):
        """Disconnect from rosbridge server."""
        self._shutdown = True
        self.reconnect = False
        if self._ws:
            self._ws.close()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._connected = False

    def _run(self):
        """WebSocket run loop with reconnection."""
        while not self._shutdown:
            try:
                self._ws.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as e:
                if self.on_error:
                    self.on_error(f"WebSocket error: {e}")

            if self._shutdown or not self.reconnect:
                break

            time.sleep(self.reconnect_interval)

    def _on_open(self, ws):
        """Called when WebSocket connects."""
        self._connected = True
        if self.on_connect:
            self.on_connect()

        # Resubscribe to all topics
        with self._lock:
            for topic, msg_type in self._topic_types.items():
                self._send_subscribe(topic, msg_type)

    def _on_close(self, ws, close_status_code, close_msg):
        """Called when WebSocket disconnects."""
        self._connected = False
        if self.on_disconnect:
            self.on_disconnect()

    def _on_error(self, ws, error):
        """Called on WebSocket error."""
        if self.on_error:
            self.on_error(str(error))

    def _on_message(self, ws, message):
        """Called when a message is received."""
        try:
            data = json.loads(message)
            op = data.get('op')

            if op == 'publish':
                # Topic message
                topic = data.get('topic')
                msg = data.get('msg', {})
                with self._lock:
                    callbacks = self._subscriptions.get(topic, [])
                for cb in callbacks:
                    try:
                        cb(msg)
                    except Exception:
                        pass

            elif op == 'service_response':
                # Service response
                service_id = data.get('id')
                if service_id and service_id in self._service_results:
                    self._service_results[service_id].put(data)

        except json.JSONDecodeError:
            pass

    def _send(self, msg: Dict):
        """Send a message to rosbridge."""
        if self._connected and self._ws:
            try:
                self._ws.send(json.dumps(msg))
            except Exception:
                pass

    def _send_subscribe(self, topic: str, msg_type: str):
        """Send subscribe message."""
        self._send({
            'op': 'subscribe',
            'topic': topic,
            'type': msg_type,
        })

    # ── Public API ────────────────────────────────────────────────────

    def subscribe(
        self,
        topic: str,
        msg_type: str,
        callback: Callable[[Dict], None],
        throttle_rate: int = 0,
        queue_length: int = 1,
    ):
        """
        Subscribe to a ROS topic.

        Args:
            topic: Topic name (e.g., '/mavros/state')
            msg_type: Message type (e.g., 'mavros_msgs/State')
            callback: Function called with message dict
            throttle_rate: Min ms between messages (0 = no throttle)
            queue_length: Message queue length
        """
        with self._lock:
            if topic not in self._subscriptions:
                self._subscriptions[topic] = []
                self._topic_types[topic] = msg_type
            self._subscriptions[topic].append(callback)

        if self._connected:
            msg = {
                'op': 'subscribe',
                'topic': topic,
                'type': msg_type,
                'throttle_rate': throttle_rate,
                'queue_length': queue_length,
            }
            self._send(msg)

    def unsubscribe(self, topic: str, callback: Optional[Callable] = None):
        """
        Unsubscribe from a topic.

        Args:
            topic: Topic name
            callback: Specific callback to remove, or None to remove all
        """
        with self._lock:
            if topic in self._subscriptions:
                if callback:
                    self._subscriptions[topic] = [
                        cb for cb in self._subscriptions[topic] if cb != callback
                    ]
                else:
                    self._subscriptions[topic] = []

                if not self._subscriptions[topic]:
                    del self._subscriptions[topic]
                    del self._topic_types[topic]
                    self._send({'op': 'unsubscribe', 'topic': topic})

    def publish(self, topic: str, msg_type: str, msg: Dict):
        """
        Publish a message to a ROS topic.

        Args:
            topic: Topic name
            msg_type: Message type
            msg: Message data as dict
        """
        self._send({
            'op': 'publish',
            'topic': topic,
            'type': msg_type,
            'msg': msg,
        })

    def advertise(self, topic: str, msg_type: str):
        """
        Advertise a topic for publishing.

        Args:
            topic: Topic name
            msg_type: Message type
        """
        self._send({
            'op': 'advertise',
            'topic': topic,
            'type': msg_type,
        })

    def unadvertise(self, topic: str):
        """Stop advertising a topic."""
        self._send({'op': 'unadvertise', 'topic': topic})

    def call_service(
        self,
        service: str,
        args: Optional[Dict] = None,
        timeout: float = 5.0,
    ) -> Optional[Dict]:
        """
        Call a ROS service.

        Args:
            service: Service name (e.g., '/mavros/cmd/arming')
            args: Service arguments
            timeout: Response timeout in seconds

        Returns:
            Service response dict, or None on timeout/error
        """
        with self._lock:
            self._service_id += 1
            service_id = f"call_service:{self._service_id}"
            self._service_results[service_id] = Queue()

        self._send({
            'op': 'call_service',
            'id': service_id,
            'service': service,
            'args': args or {},
        })

        try:
            result = self._service_results[service_id].get(timeout=timeout)
            return result.get('values')
        except Empty:
            return None
        finally:
            with self._lock:
                del self._service_results[service_id]

    def get_topics(self) -> List[str]:
        """Get list of available topics."""
        result = self.call_service('/rosapi/topics', {})
        if result:
            return result.get('topics', [])
        return []

    def get_topic_type(self, topic: str) -> Optional[str]:
        """Get the message type for a topic."""
        result = self.call_service('/rosapi/topic_type', {'topic': topic})
        if result:
            return result.get('type')
        return None

    def get_services(self) -> List[str]:
        """Get list of available services."""
        result = self.call_service('/rosapi/services', {})
        if result:
            return result.get('services', [])
        return []


# ── MAVROS-specific helpers ───────────────────────────────────────────

class MAVROSBridge(ROSBridgeClient):
    """
    Specialized ROSBridge client for MAVROS topics.

    Provides convenient methods for common MAVROS operations.
    """

    # Common MAVROS topics and their types
    TOPICS = {
        # State
        '/mavros/state': 'mavros_msgs/State',
        '/mavros/extended_state': 'mavros_msgs/ExtendedState',
        '/mavros/battery': 'sensor_msgs/BatteryState',
        '/mavros/vfr_hud': 'mavros_msgs/VFR_HUD',

        # IMU
        '/mavros/imu/data': 'sensor_msgs/Imu',
        '/mavros/imu/mag': 'sensor_msgs/MagneticField',
        '/mavros/imu/static_pressure': 'sensor_msgs/FluidPressure',

        # Position
        '/mavros/local_position/pose': 'geometry_msgs/PoseStamped',
        '/mavros/local_position/velocity_local': 'geometry_msgs/TwistStamped',
        '/mavros/local_position/odom': 'nav_msgs/Odometry',
        '/mavros/global_position/global': 'sensor_msgs/NavSatFix',
        '/mavros/global_position/rel_alt': 'std_msgs/Float64',
        '/mavros/global_position/compass_hdg': 'std_msgs/Float64',

        # RC
        '/mavros/rc/in': 'mavros_msgs/RCIn',
        '/mavros/rc/out': 'mavros_msgs/RCOut',
        '/mavros/rc/override': 'mavros_msgs/OverrideRCIn',

        # Manual control
        '/mavros/manual_control/control': 'mavros_msgs/ManualControl',
        '/mavros/manual_control/send': 'mavros_msgs/ManualControl',
    }

    # MAVROS services
    SERVICES = {
        'arm': '/mavros/cmd/arming',
        'set_mode': '/mavros/set_mode',
        'command_long': '/mavros/cmd/command',
        'set_home': '/mavros/cmd/set_home',
    }

    def subscribe_state(self, callback: Callable[[Dict], None]):
        """Subscribe to /mavros/state."""
        self.subscribe('/mavros/state', self.TOPICS['/mavros/state'], callback)

    def subscribe_battery(self, callback: Callable[[Dict], None]):
        """Subscribe to /mavros/battery."""
        self.subscribe('/mavros/battery', self.TOPICS['/mavros/battery'], callback)

    def subscribe_imu(self, callback: Callable[[Dict], None]):
        """Subscribe to /mavros/imu/data."""
        self.subscribe('/mavros/imu/data', self.TOPICS['/mavros/imu/data'], callback)

    def subscribe_pose(self, callback: Callable[[Dict], None]):
        """Subscribe to /mavros/local_position/pose."""
        self.subscribe(
            '/mavros/local_position/pose',
            self.TOPICS['/mavros/local_position/pose'],
            callback
        )

    def subscribe_vfr_hud(self, callback: Callable[[Dict], None]):
        """Subscribe to /mavros/vfr_hud."""
        self.subscribe('/mavros/vfr_hud', self.TOPICS['/mavros/vfr_hud'], callback)

    def subscribe_rc_in(self, callback: Callable[[Dict], None]):
        """Subscribe to /mavros/rc/in."""
        self.subscribe('/mavros/rc/in', self.TOPICS['/mavros/rc/in'], callback)

    def subscribe_rc_out(self, callback: Callable[[Dict], None]):
        """Subscribe to /mavros/rc/out."""
        self.subscribe('/mavros/rc/out', self.TOPICS['/mavros/rc/out'], callback)

    # ── Commands ──────────────────────────────────────────────────────

    def arm(self, arm: bool = True) -> bool:
        """
        Arm or disarm the vehicle.

        Args:
            arm: True to arm, False to disarm

        Returns:
            True if command succeeded
        """
        result = self.call_service(self.SERVICES['arm'], {'value': arm})
        return result.get('success', False) if result else False

    def disarm(self) -> bool:
        """Disarm the vehicle."""
        return self.arm(False)

    def set_mode(self, mode: str) -> bool:
        """
        Set flight mode.

        Args:
            mode: Mode name (e.g., 'MANUAL', 'STABILIZE', 'ALT_HOLD')

        Returns:
            True if command succeeded
        """
        result = self.call_service(
            self.SERVICES['set_mode'],
            {'custom_mode': mode}
        )
        return result.get('mode_sent', False) if result else False

    def send_rc_override(self, channels: List[int]):
        """
        Send RC override.

        Args:
            channels: List of 8-18 channel values (1000-2000, 0 = release)
        """
        # Pad to 18 channels
        while len(channels) < 18:
            channels.append(0)

        self.publish(
            '/mavros/rc/override',
            'mavros_msgs/OverrideRCIn',
            {'channels': channels[:18]}
        )

    def send_manual_control(
        self,
        x: int = 0,
        y: int = 0,
        z: int = 500,
        r: int = 0,
        buttons: int = 0,
    ):
        """
        Send manual control (joystick-style).

        Args:
            x: Forward/back (-1000 to 1000)
            y: Left/right (-1000 to 1000)
            z: Throttle (0 to 1000, 500 = neutral)
            r: Yaw (-1000 to 1000)
            buttons: Button bitmask
        """
        self.publish(
            '/mavros/manual_control/send',
            'mavros_msgs/ManualControl',
            {'x': x, 'y': y, 'z': z, 'r': r, 'buttons': buttons}
        )
