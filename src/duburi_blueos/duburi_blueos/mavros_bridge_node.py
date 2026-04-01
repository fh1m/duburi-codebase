#!/usr/bin/env python3
"""
MAVROS Bridge Node - ROS1 to ROS2 bridge via rosbridge_websocket.

This node connects to the rosbridge_websocket server running on BlueOS (Pi)
and republishes MAVROS topics as ROS2 topics. This enables ROS2 nodes on
the Jetson/PC to access MAVROS data from ROS1 without ros1_bridge.

Published Topics:
    ~/state (mavros_msgs/State equiv -> duburi_msgs/VehicleState)
    ~/battery (sensor_msgs/BatteryState)
    ~/imu/data (sensor_msgs/Imu)
    ~/vfr_hud (std_msgs/Float32MultiArray - heading, airspeed, groundspeed, altitude, climb, throttle)
    ~/rc/in (std_msgs/UInt16MultiArray - RC input channels)
    ~/rc/out (std_msgs/UInt16MultiArray - RC output channels)
    ~/global_position (sensor_msgs/NavSatFix)
    ~/local_position/pose (geometry_msgs/PoseStamped)
    ~/compass_hdg (std_msgs/Float64)

Subscribed Topics:
    ~/rc/override (std_msgs/UInt16MultiArray - RC override commands)
    ~/manual_control (geometry_msgs/Twist - manual control input)

Services:
    ~/arm (std_srvs/SetBool)
    ~/set_mode (TODO: custom service)
"""

from __future__ import annotations

import threading
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Standard ROS2 messages
from std_msgs.msg import Float64, UInt16MultiArray, Float32MultiArray, Bool, String
from std_srvs.srv import SetBool
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, MagneticField, FluidPressure
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3

from .rosbridge_client import MAVROSBridge, ROSBridgeError


class MAVROSBridgeNode(Node):
    """
    ROS2 node that bridges MAVROS topics from ROS1 via rosbridge.
    """

    def __init__(self):
        super().__init__('mavros_bridge')

        # Declare parameters
        self.declare_parameter('blueos_host', '192.168.2.2')
        self.declare_parameter('rosbridge_port', 8889)
        self.declare_parameter('reconnect', True)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('publish_rate_hz', 50.0)

        # Get parameters
        self.host = self.get_parameter('blueos_host').value
        self.port = self.get_parameter('rosbridge_port').value
        self.reconnect = self.get_parameter('reconnect').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # State storage
        self._connected = False
        self._armed = False
        self._mode = 'UNKNOWN'
        self._system_status = 0
        self._lock = threading.Lock()

        # Create publishers
        self._setup_publishers()

        # Create subscribers (for sending commands back to MAVROS)
        self._setup_subscribers()

        # Create services
        self._setup_services()

        # Initialize rosbridge client
        self._bridge: Optional[MAVROSBridge] = None
        self._connect_rosbridge()

        # Status timer
        self.create_timer(5.0, self._publish_status)

        self.get_logger().info(
            f'MAVROS Bridge initialized - connecting to ws://{self.host}:{self.port}'
        )

    def _setup_publishers(self):
        """Create ROS2 publishers for bridged topics."""
        # Connection status
        self.connected_pub = self.create_publisher(
            Bool, '~/connected', self.state_qos
        )

        # Vehicle state (simplified - armed, mode, etc.)
        self.armed_pub = self.create_publisher(Bool, '~/armed', self.state_qos)
        self.mode_pub = self.create_publisher(String, '~/mode', self.state_qos)

        # Battery
        self.battery_pub = self.create_publisher(
            BatteryState, '~/battery', self.sensor_qos
        )

        # IMU
        self.imu_pub = self.create_publisher(Imu, '~/imu/data', self.sensor_qos)
        self.mag_pub = self.create_publisher(
            MagneticField, '~/imu/mag', self.sensor_qos
        )
        self.pressure_pub = self.create_publisher(
            FluidPressure, '~/imu/pressure', self.sensor_qos
        )

        # VFR HUD (heading, speeds, altitude, etc.)
        # [heading, airspeed, groundspeed, altitude, climb, throttle]
        self.vfr_hud_pub = self.create_publisher(
            Float32MultiArray, '~/vfr_hud', self.sensor_qos
        )

        # Position
        self.global_pos_pub = self.create_publisher(
            NavSatFix, '~/global_position', self.sensor_qos
        )
        self.local_pose_pub = self.create_publisher(
            PoseStamped, '~/local_position/pose', self.sensor_qos
        )
        self.local_vel_pub = self.create_publisher(
            TwistStamped, '~/local_position/velocity', self.sensor_qos
        )
        self.compass_hdg_pub = self.create_publisher(
            Float64, '~/compass_hdg', self.sensor_qos
        )
        self.rel_alt_pub = self.create_publisher(
            Float64, '~/rel_alt', self.sensor_qos
        )

        # RC
        self.rc_in_pub = self.create_publisher(
            UInt16MultiArray, '~/rc/in', self.sensor_qos
        )
        self.rc_out_pub = self.create_publisher(
            UInt16MultiArray, '~/rc/out', self.sensor_qos
        )

    def _setup_subscribers(self):
        """Create ROS2 subscribers for commands to send to MAVROS."""
        # RC override
        self.rc_override_sub = self.create_subscription(
            UInt16MultiArray,
            '~/rc/override',
            self._on_rc_override,
            10
        )

        # Manual control (Twist)
        self.manual_control_sub = self.create_subscription(
            Twist,
            '~/manual_control',
            self._on_manual_control,
            10
        )

    def _setup_services(self):
        """Create ROS2 services for MAVROS commands."""
        self.arm_srv = self.create_service(
            SetBool, '~/arm', self._arm_callback
        )

    def _connect_rosbridge(self):
        """Connect to rosbridge server."""
        try:
            self._bridge = MAVROSBridge(
                host=self.host,
                port=self.port,
                reconnect=self.reconnect,
                reconnect_interval=self.reconnect_interval,
            )

            # Set callbacks
            self._bridge.on_connect = self._on_rosbridge_connect
            self._bridge.on_disconnect = self._on_rosbridge_disconnect
            self._bridge.on_error = self._on_rosbridge_error

            # Connect (non-blocking)
            self._bridge.connect(blocking=False)

        except ROSBridgeError as e:
            self.get_logger().error(f'Failed to create rosbridge client: {e}')

    def _on_rosbridge_connect(self):
        """Called when rosbridge connects."""
        self.get_logger().info('Connected to rosbridge')
        with self._lock:
            self._connected = True

        # Publish connected status
        msg = Bool()
        msg.data = True
        self.connected_pub.publish(msg)

        # Subscribe to MAVROS topics
        self._subscribe_mavros_topics()

    def _on_rosbridge_disconnect(self):
        """Called when rosbridge disconnects."""
        self.get_logger().warn('Disconnected from rosbridge')
        with self._lock:
            self._connected = False

        # Publish disconnected status
        msg = Bool()
        msg.data = False
        self.connected_pub.publish(msg)

    def _on_rosbridge_error(self, error: str):
        """Called on rosbridge error."""
        self.get_logger().error(f'ROSBridge error: {error}')

    def _subscribe_mavros_topics(self):
        """Subscribe to all MAVROS topics of interest."""
        if not self._bridge:
            return

        # State
        self._bridge.subscribe_state(self._on_mavros_state)

        # Battery
        self._bridge.subscribe_battery(self._on_mavros_battery)

        # IMU
        self._bridge.subscribe_imu(self._on_mavros_imu)
        self._bridge.subscribe(
            '/mavros/imu/mag',
            'sensor_msgs/MagneticField',
            self._on_mavros_mag
        )
        self._bridge.subscribe(
            '/mavros/imu/static_pressure',
            'sensor_msgs/FluidPressure',
            self._on_mavros_pressure
        )

        # VFR HUD
        self._bridge.subscribe_vfr_hud(self._on_mavros_vfr_hud)

        # Position
        self._bridge.subscribe(
            '/mavros/global_position/global',
            'sensor_msgs/NavSatFix',
            self._on_mavros_global_pos
        )
        self._bridge.subscribe_pose(self._on_mavros_local_pose)
        self._bridge.subscribe(
            '/mavros/local_position/velocity_local',
            'geometry_msgs/TwistStamped',
            self._on_mavros_local_vel
        )
        self._bridge.subscribe(
            '/mavros/global_position/compass_hdg',
            'std_msgs/Float64',
            self._on_mavros_compass_hdg
        )
        self._bridge.subscribe(
            '/mavros/global_position/rel_alt',
            'std_msgs/Float64',
            self._on_mavros_rel_alt
        )

        # RC
        self._bridge.subscribe_rc_in(self._on_mavros_rc_in)
        self._bridge.subscribe_rc_out(self._on_mavros_rc_out)

        self.get_logger().info('Subscribed to MAVROS topics')

    # ── MAVROS topic callbacks ────────────────────────────────────────

    def _on_mavros_state(self, msg: Dict[str, Any]):
        """Handle /mavros/state message."""
        with self._lock:
            self._armed = msg.get('armed', False)
            self._mode = msg.get('mode', 'UNKNOWN')
            self._system_status = msg.get('system_status', 0)

        # Publish armed state
        armed_msg = Bool()
        armed_msg.data = self._armed
        self.armed_pub.publish(armed_msg)

        # Publish mode
        mode_msg = String()
        mode_msg.data = self._mode
        self.mode_pub.publish(mode_msg)

    def _on_mavros_battery(self, msg: Dict[str, Any]):
        """Handle /mavros/battery message."""
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = msg.get('voltage', 0.0)
        battery.current = msg.get('current', float('nan'))
        battery.percentage = msg.get('percentage', float('nan'))
        battery.present = True

        # Handle cell voltages if available
        cell_voltages = msg.get('cell_voltage', [])
        if cell_voltages:
            battery.cell_voltage = cell_voltages

        self.battery_pub.publish(battery)

    def _on_mavros_imu(self, msg: Dict[str, Any]):
        """Handle /mavros/imu/data message."""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Orientation
        orientation = msg.get('orientation', {})
        imu.orientation.x = orientation.get('x', 0.0)
        imu.orientation.y = orientation.get('y', 0.0)
        imu.orientation.z = orientation.get('z', 0.0)
        imu.orientation.w = orientation.get('w', 1.0)

        # Angular velocity
        angular = msg.get('angular_velocity', {})
        imu.angular_velocity.x = angular.get('x', 0.0)
        imu.angular_velocity.y = angular.get('y', 0.0)
        imu.angular_velocity.z = angular.get('z', 0.0)

        # Linear acceleration
        linear = msg.get('linear_acceleration', {})
        imu.linear_acceleration.x = linear.get('x', 0.0)
        imu.linear_acceleration.y = linear.get('y', 0.0)
        imu.linear_acceleration.z = linear.get('z', 0.0)

        self.imu_pub.publish(imu)

    def _on_mavros_mag(self, msg: Dict[str, Any]):
        """Handle /mavros/imu/mag message."""
        mag = MagneticField()
        mag.header.stamp = self.get_clock().now().to_msg()
        mag.header.frame_id = 'imu_link'

        field = msg.get('magnetic_field', {})
        mag.magnetic_field.x = field.get('x', 0.0)
        mag.magnetic_field.y = field.get('y', 0.0)
        mag.magnetic_field.z = field.get('z', 0.0)

        self.mag_pub.publish(mag)

    def _on_mavros_pressure(self, msg: Dict[str, Any]):
        """Handle /mavros/imu/static_pressure message."""
        pressure = FluidPressure()
        pressure.header.stamp = self.get_clock().now().to_msg()
        pressure.fluid_pressure = msg.get('fluid_pressure', 0.0)
        pressure.variance = msg.get('variance', 0.0)

        self.pressure_pub.publish(pressure)

    def _on_mavros_vfr_hud(self, msg: Dict[str, Any]):
        """Handle /mavros/vfr_hud message."""
        # Pack into Float32MultiArray:
        # [heading, airspeed, groundspeed, altitude, climb, throttle]
        vfr = Float32MultiArray()
        vfr.data = [
            float(msg.get('heading', 0)),
            float(msg.get('airspeed', 0.0)),
            float(msg.get('groundspeed', 0.0)),
            float(msg.get('altitude', 0.0)),
            float(msg.get('climb', 0.0)),
            float(msg.get('throttle', 0.0)),
        ]
        self.vfr_hud_pub.publish(vfr)

    def _on_mavros_global_pos(self, msg: Dict[str, Any]):
        """Handle /mavros/global_position/global message."""
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'map'
        fix.latitude = msg.get('latitude', 0.0)
        fix.longitude = msg.get('longitude', 0.0)
        fix.altitude = msg.get('altitude', 0.0)

        # Covariance
        cov = msg.get('position_covariance', [])
        if len(cov) >= 9:
            fix.position_covariance = cov[:9]
            fix.position_covariance_type = msg.get(
                'position_covariance_type',
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            )

        self.global_pos_pub.publish(fix)

    def _on_mavros_local_pose(self, msg: Dict[str, Any]):
        """Handle /mavros/local_position/pose message."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'

        position = msg.get('pose', {}).get('position', {})
        pose.pose.position.x = position.get('x', 0.0)
        pose.pose.position.y = position.get('y', 0.0)
        pose.pose.position.z = position.get('z', 0.0)

        orientation = msg.get('pose', {}).get('orientation', {})
        pose.pose.orientation.x = orientation.get('x', 0.0)
        pose.pose.orientation.y = orientation.get('y', 0.0)
        pose.pose.orientation.z = orientation.get('z', 0.0)
        pose.pose.orientation.w = orientation.get('w', 1.0)

        self.local_pose_pub.publish(pose)

    def _on_mavros_local_vel(self, msg: Dict[str, Any]):
        """Handle /mavros/local_position/velocity_local message."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'odom'

        linear = msg.get('twist', {}).get('linear', {})
        twist.twist.linear.x = linear.get('x', 0.0)
        twist.twist.linear.y = linear.get('y', 0.0)
        twist.twist.linear.z = linear.get('z', 0.0)

        angular = msg.get('twist', {}).get('angular', {})
        twist.twist.angular.x = angular.get('x', 0.0)
        twist.twist.angular.y = angular.get('y', 0.0)
        twist.twist.angular.z = angular.get('z', 0.0)

        self.local_vel_pub.publish(twist)

    def _on_mavros_compass_hdg(self, msg: Dict[str, Any]):
        """Handle /mavros/global_position/compass_hdg message."""
        hdg = Float64()
        hdg.data = msg.get('data', 0.0)
        self.compass_hdg_pub.publish(hdg)

    def _on_mavros_rel_alt(self, msg: Dict[str, Any]):
        """Handle /mavros/global_position/rel_alt message."""
        alt = Float64()
        alt.data = msg.get('data', 0.0)
        self.rel_alt_pub.publish(alt)

    def _on_mavros_rc_in(self, msg: Dict[str, Any]):
        """Handle /mavros/rc/in message."""
        rc = UInt16MultiArray()
        channels = msg.get('channels', [])
        rc.data = [int(c) for c in channels]
        self.rc_in_pub.publish(rc)

    def _on_mavros_rc_out(self, msg: Dict[str, Any]):
        """Handle /mavros/rc/out message."""
        rc = UInt16MultiArray()
        channels = msg.get('channels', [])
        rc.data = [int(c) for c in channels]
        self.rc_out_pub.publish(rc)

    # ── Command callbacks ─────────────────────────────────────────────

    def _on_rc_override(self, msg: UInt16MultiArray):
        """Send RC override to MAVROS."""
        if self._bridge and self._connected:
            channels = list(msg.data)
            self._bridge.send_rc_override(channels)
            self.get_logger().debug(f'Sent RC override: {channels[:8]}...')

    def _on_manual_control(self, msg: Twist):
        """Send manual control to MAVROS."""
        if self._bridge and self._connected:
            # Convert Twist to MAVROS manual control
            # x: linear.x scaled to -1000..1000
            # y: linear.y scaled to -1000..1000
            # z: linear.z scaled to 0..1000 (throttle)
            # r: angular.z scaled to -1000..1000 (yaw)
            x = int(msg.linear.x * 1000)
            y = int(msg.linear.y * 1000)
            z = int((msg.linear.z + 1.0) * 500)  # Assuming -1..1 input -> 0..1000
            r = int(msg.angular.z * 1000)

            self._bridge.send_manual_control(x=x, y=y, z=z, r=r)
            self.get_logger().debug(f'Sent manual control: x={x} y={y} z={z} r={r}')

    def _arm_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response
    ) -> SetBool.Response:
        """Handle arm/disarm service call."""
        if not self._bridge or not self._connected:
            response.success = False
            response.message = 'Not connected to rosbridge'
            return response

        try:
            success = self._bridge.arm(request.data)
            response.success = success
            response.message = (
                f"{'Armed' if request.data else 'Disarmed'} "
                f"{'successfully' if success else 'failed'}"
            )
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def _publish_status(self):
        """Periodically publish status."""
        with self._lock:
            connected = self._connected
            armed = self._armed
            mode = self._mode

        if connected:
            self.get_logger().debug(
                f'MAVROS Bridge: connected, armed={armed}, mode={mode}'
            )

    def destroy_node(self):
        """Clean up on shutdown."""
        if self._bridge:
            self._bridge.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MAVROSBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
