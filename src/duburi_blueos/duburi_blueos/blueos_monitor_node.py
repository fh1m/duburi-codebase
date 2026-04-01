#!/usr/bin/env python3
"""
BlueOS Monitor Node — System monitoring for BRACU Duburi 4.2.

Monitors BlueOS running on Raspberry Pi 4B and publishes:
  - System status (CPU, memory, disk, temperature)
  - Connection health to BlueOS
  - MAVLink endpoint status
  - Service availability

Publishes to:
  - /blueos/system_status (diagnostic_msgs/DiagnosticArray)
  - /blueos/connected (std_msgs/Bool)

Services:
  - /blueos/restart_autopilot (std_srvs/Trigger)
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .blueos_api import BlueOSAPI, BlueOSError


class BlueOSMonitorNode(Node):
    """
    ROS 2 node for monitoring BlueOS system status.

    Periodically queries BlueOS REST API and publishes diagnostics.
    """

    def __init__(self):
        super().__init__('blueos_monitor')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter(
            'blueos_host',
            '192.168.2.2',
            ParameterDescriptor(description='BlueOS IP address (Raspberry Pi)'),
        )
        self.declare_parameter(
            'poll_rate_hz',
            1.0,
            ParameterDescriptor(description='Status polling rate in Hz'),
        )
        self.declare_parameter(
            'timeout_sec',
            2.0,
            ParameterDescriptor(description='API request timeout in seconds'),
        )
        self.declare_parameter(
            'warn_cpu_percent',
            80.0,
            ParameterDescriptor(description='CPU usage warning threshold (%)'),
        )
        self.declare_parameter(
            'warn_memory_percent',
            80.0,
            ParameterDescriptor(description='Memory usage warning threshold (%)'),
        )
        self.declare_parameter(
            'warn_disk_percent',
            90.0,
            ParameterDescriptor(description='Disk usage warning threshold (%)'),
        )
        self.declare_parameter(
            'warn_temp_celsius',
            70.0,
            ParameterDescriptor(description='CPU temperature warning threshold (C)'),
        )

        # Get parameter values
        self._blueos_host = self.get_parameter('blueos_host').value
        self._poll_rate = self.get_parameter('poll_rate_hz').value
        self._timeout = self.get_parameter('timeout_sec').value
        self._warn_cpu = self.get_parameter('warn_cpu_percent').value
        self._warn_memory = self.get_parameter('warn_memory_percent').value
        self._warn_disk = self.get_parameter('warn_disk_percent').value
        self._warn_temp = self.get_parameter('warn_temp_celsius').value

        # ── BlueOS API client ─────────────────────────────────────────
        self._api = BlueOSAPI(host=self._blueos_host, timeout=self._timeout)
        self._connected = False
        self._last_error: Optional[str] = None

        # ── Publishers ────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._diag_pub = self.create_publisher(
            DiagnosticArray,
            '/blueos/system_status',
            reliable_qos,
        )
        self._connected_pub = self.create_publisher(
            Bool,
            '/blueos/connected',
            reliable_qos,
        )

        # ── Services ──────────────────────────────────────────────────
        self._restart_srv = self.create_service(
            Trigger,
            '/blueos/restart_autopilot',
            self._restart_autopilot_callback,
        )

        # ── Timers ────────────────────────────────────────────────────
        poll_period = 1.0 / self._poll_rate
        self._poll_timer = self.create_timer(poll_period, self._poll_status)

        self.get_logger().info(
            f"BlueOS monitor started: host={self._blueos_host}, rate={self._poll_rate}Hz"
        )

    def _poll_status(self) -> None:
        """Poll BlueOS for system status and publish diagnostics."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        try:
            # Get system info from BlueOS
            system_info = self._api.get_system_info()
            self._connected = True
            self._last_error = None

            # Build diagnostic status
            status = self._build_system_status(system_info)
            diag_array.status.append(status)

            # Try to get additional info
            try:
                board_info = self._api.get_board_info()
                board_status = self._build_board_status(board_info)
                diag_array.status.append(board_status)
            except BlueOSError:
                pass  # Board info optional

        except BlueOSError as e:
            self._connected = False
            self._last_error = str(e)

            # Publish error status
            status = DiagnosticStatus()
            status.name = "BlueOS Connection"
            status.level = DiagnosticStatus.ERROR
            status.message = f"Cannot reach BlueOS: {e}"
            status.hardware_id = self._blueos_host
            diag_array.status.append(status)

            self.get_logger().warn(f"BlueOS connection error: {e}")

        # Publish diagnostics
        self._diag_pub.publish(diag_array)

        # Publish connection status
        connected_msg = Bool()
        connected_msg.data = self._connected
        self._connected_pub.publish(connected_msg)

    def _build_system_status(self, info: dict) -> DiagnosticStatus:
        """Build DiagnosticStatus from system info."""
        status = DiagnosticStatus()
        status.name = "BlueOS System"
        status.hardware_id = self._blueos_host

        # Extract values with safe defaults
        cpu = info.get('cpu', {})
        memory = info.get('memory', {})
        disk = info.get('disk', {})
        temperature = info.get('temperature', {})

        cpu_percent = cpu.get('percent', 0.0)
        memory_percent = memory.get('percent', 0.0)
        disk_percent = disk.get('percent', 0.0)
        temp_cpu = temperature.get('cpu', 0.0)

        # Determine overall status level
        level = DiagnosticStatus.OK
        messages = []

        if cpu_percent > self._warn_cpu:
            level = max(level, DiagnosticStatus.WARN)
            messages.append(f"High CPU: {cpu_percent:.1f}%")

        if memory_percent > self._warn_memory:
            level = max(level, DiagnosticStatus.WARN)
            messages.append(f"High memory: {memory_percent:.1f}%")

        if disk_percent > self._warn_disk:
            level = max(level, DiagnosticStatus.WARN)
            messages.append(f"Low disk space: {disk_percent:.1f}% used")

        if temp_cpu and temp_cpu > self._warn_temp:
            level = max(level, DiagnosticStatus.WARN)
            messages.append(f"High temperature: {temp_cpu:.1f}C")

        status.level = level
        status.message = "; ".join(messages) if messages else "OK"

        # Add key-value pairs
        status.values = [
            KeyValue(key="cpu_percent", value=f"{cpu_percent:.1f}"),
            KeyValue(key="memory_percent", value=f"{memory_percent:.1f}"),
            KeyValue(key="memory_total_mb", value=str(memory.get('total', 0) // (1024*1024))),
            KeyValue(key="memory_used_mb", value=str(memory.get('used', 0) // (1024*1024))),
            KeyValue(key="disk_percent", value=f"{disk_percent:.1f}"),
            KeyValue(key="disk_total_gb", value=f"{disk.get('total', 0) / (1024**3):.1f}"),
            KeyValue(key="disk_free_gb", value=f"{disk.get('free', 0) / (1024**3):.1f}"),
            KeyValue(key="temperature_cpu", value=f"{temp_cpu:.1f}" if temp_cpu else "N/A"),
            KeyValue(key="uptime_seconds", value=str(info.get('uptime', 0))),
            KeyValue(key="blueos_version", value=info.get('blueos_version', 'unknown')),
        ]

        return status

    def _build_board_status(self, info: dict) -> DiagnosticStatus:
        """Build DiagnosticStatus from flight controller board info."""
        status = DiagnosticStatus()
        status.name = "Flight Controller"
        status.hardware_id = info.get('name', 'unknown')
        status.level = DiagnosticStatus.OK
        status.message = "Connected"

        status.values = [
            KeyValue(key="name", value=info.get('name', 'unknown')),
            KeyValue(key="platform", value=info.get('platform', 'unknown')),
            KeyValue(key="path", value=info.get('path', 'unknown')),
        ]

        return status

    def _restart_autopilot_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Service callback to restart ArduPilot."""
        try:
            self._api.restart_autopilot()
            response.success = True
            response.message = "ArduPilot restart requested"
            self.get_logger().info("ArduPilot restart requested via BlueOS")
        except BlueOSError as e:
            response.success = False
            response.message = f"Failed to restart ArduPilot: {e}"
            self.get_logger().error(response.message)

        return response

    def destroy_node(self) -> None:
        """Clean shutdown."""
        self.get_logger().info("BlueOS monitor shutting down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlueOSMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
