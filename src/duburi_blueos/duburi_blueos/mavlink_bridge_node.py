#!/usr/bin/env python3
"""
MAVLink Bridge Node — Endpoint management for BRACU Duburi 4.2.

Manages MAVLink endpoints in BlueOS to establish communication between:
  - Raspberry Pi 4B (BlueOS + Pixhawk 2.4.8)
  - Jetson Orin Nano (ROS 2 companion computer)

On startup, ensures a UDP endpoint exists for the Jetson to receive MAVLink.
Provides services for dynamic endpoint management.

Services:
  - /blueos/list_endpoints (duburi_interfaces/GetMavlinkEndpoints)
  - /blueos/add_endpoint (duburi_interfaces/AddMavlinkEndpoint)
  - /blueos/setup_jetson_link (std_srvs/Trigger)
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger
from std_msgs.msg import String

from .blueos_api import BlueOSAPI, BlueOSError, MavlinkEndpoint, EndpointType


class MavlinkBridgeNode(Node):
    """
    ROS 2 node for managing MAVLink endpoints in BlueOS.

    Ensures proper MAVLink routing between BlueOS (Pi) and Jetson.
    """

    # Default endpoint name for Jetson connection
    JETSON_ENDPOINT_NAME = "Duburi-Jetson"

    def __init__(self):
        super().__init__('mavlink_bridge')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter(
            'blueos_host',
            '192.168.2.2',
            ParameterDescriptor(description='BlueOS IP address (Raspberry Pi)'),
        )
        self.declare_parameter(
            'jetson_ip',
            '192.168.2.3',
            ParameterDescriptor(description='Jetson Orin Nano IP address'),
        )
        self.declare_parameter(
            'mavlink_port',
            14550,
            ParameterDescriptor(description='MAVLink UDP port for Jetson'),
        )
        self.declare_parameter(
            'auto_setup',
            True,
            ParameterDescriptor(description='Auto-create Jetson endpoint on startup'),
        )
        self.declare_parameter(
            'timeout_sec',
            5.0,
            ParameterDescriptor(description='API request timeout in seconds'),
        )

        # Get parameter values
        self._blueos_host = self.get_parameter('blueos_host').value
        self._jetson_ip = self.get_parameter('jetson_ip').value
        self._mavlink_port = self.get_parameter('mavlink_port').value
        self._auto_setup = self.get_parameter('auto_setup').value
        self._timeout = self.get_parameter('timeout_sec').value

        # ── BlueOS API client ─────────────────────────────────────────
        self._api = BlueOSAPI(host=self._blueos_host, timeout=self._timeout)

        # ── Publishers ────────────────────────────────────────────────
        self._endpoints_pub = self.create_publisher(
            String,
            '/blueos/endpoints_info',
            10,
        )

        # ── Services ──────────────────────────────────────────────────
        self._setup_srv = self.create_service(
            Trigger,
            '/blueos/setup_jetson_link',
            self._setup_jetson_link_callback,
        )
        self._list_srv = self.create_service(
            Trigger,
            '/blueos/list_endpoints',
            self._list_endpoints_callback,
        )
        self._remove_srv = self.create_service(
            Trigger,
            '/blueos/remove_jetson_link',
            self._remove_jetson_link_callback,
        )

        # ── Auto-setup on startup ─────────────────────────────────────
        if self._auto_setup:
            # Use a one-shot timer to run setup after node is fully initialized
            self._setup_timer = self.create_timer(1.0, self._auto_setup_callback)

        self.get_logger().info(
            f"MAVLink bridge started: BlueOS={self._blueos_host}, "
            f"Jetson={self._jetson_ip}:{self._mavlink_port}"
        )

    def _auto_setup_callback(self) -> None:
        """One-shot callback to auto-setup Jetson endpoint."""
        # Cancel timer so it only runs once
        self._setup_timer.cancel()

        self.get_logger().info("Auto-configuring Jetson MAVLink endpoint...")
        success, message = self._ensure_jetson_endpoint()

        if success:
            self.get_logger().info(f"Auto-setup complete: {message}")
        else:
            self.get_logger().warn(f"Auto-setup failed: {message}")

    def _ensure_jetson_endpoint(self) -> tuple[bool, str]:
        """
        Ensure MAVLink endpoint exists for Jetson.

        Returns:
            Tuple of (success, message).
        """
        try:
            # Get current endpoints
            endpoints = self._api.get_mavlink_endpoints()

            # Check if our endpoint already exists
            for ep in endpoints:
                if ep.name == self.JETSON_ENDPOINT_NAME:
                    # Verify it points to correct IP/port
                    if ep.place == self._jetson_ip and ep.argument == self._mavlink_port:
                        return True, f"Endpoint already exists: {ep}"
                    else:
                        # Remove and recreate with correct settings
                        self.get_logger().info(
                            f"Updating endpoint: {ep.place}:{ep.argument} -> "
                            f"{self._jetson_ip}:{self._mavlink_port}"
                        )
                        self._api.remove_mavlink_endpoint(ep)

            # Create new endpoint
            endpoint = MavlinkEndpoint(
                name=self.JETSON_ENDPOINT_NAME,
                owner="duburi_blueos",
                connection_type=EndpointType.UDP_CLIENT.value,
                place=self._jetson_ip,
                argument=self._mavlink_port,
                persistent=True,
                protected=False,
                enabled=True,
            )
            self._api.add_mavlink_endpoint(endpoint)

            return True, f"Created endpoint: {endpoint}"

        except BlueOSError as e:
            return False, str(e)

    def _setup_jetson_link_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Service callback to setup/verify Jetson MAVLink endpoint."""
        success, message = self._ensure_jetson_endpoint()
        response.success = success
        response.message = message

        if success:
            self.get_logger().info(f"Jetson link setup: {message}")
        else:
            self.get_logger().error(f"Jetson link setup failed: {message}")

        return response

    def _list_endpoints_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Service callback to list all MAVLink endpoints."""
        try:
            endpoints = self._api.get_mavlink_endpoints()

            # Build info string
            lines = ["MAVLink Endpoints:"]
            for ep in endpoints:
                status = "enabled" if ep.enabled else "disabled"
                lines.append(f"  - {ep.name}: {ep} ({status})")

            info = "\n".join(lines)
            response.success = True
            response.message = info

            # Also publish to topic
            msg = String()
            msg.data = info
            self._endpoints_pub.publish(msg)

            self.get_logger().info(f"Listed {len(endpoints)} endpoints")

        except BlueOSError as e:
            response.success = False
            response.message = f"Failed to list endpoints: {e}"
            self.get_logger().error(response.message)

        return response

    def _remove_jetson_link_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Service callback to remove Jetson MAVLink endpoint."""
        try:
            endpoints = self._api.get_mavlink_endpoints()

            # Find our endpoint
            for ep in endpoints:
                if ep.name == self.JETSON_ENDPOINT_NAME:
                    self._api.remove_mavlink_endpoint(ep)
                    response.success = True
                    response.message = f"Removed endpoint: {ep}"
                    self.get_logger().info(response.message)
                    return response

            response.success = False
            response.message = f"Endpoint '{self.JETSON_ENDPOINT_NAME}' not found"

        except BlueOSError as e:
            response.success = False
            response.message = f"Failed to remove endpoint: {e}"
            self.get_logger().error(response.message)

        return response

    def destroy_node(self) -> None:
        """Clean shutdown."""
        self.get_logger().info("MAVLink bridge shutting down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
