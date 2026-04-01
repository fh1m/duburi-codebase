"""
duburi_blueos — BlueOS integration for BRACU Duburi AUV 4.2.

This package provides ROS 2 nodes and utilities for interfacing with
BlueOS running on a Raspberry Pi 4B companion computer.

Modules:
    blueos_api: REST API client for BlueOS services
    blueos_monitor_node: System monitoring and diagnostics
    mavlink_bridge_node: MAVLink endpoint management
    rosbridge_client: WebSocket client for ROS1 rosbridge
    mavros_bridge_node: ROS1 MAVROS to ROS2 bridge via rosbridge

Network Architecture:
    Raspberry Pi 4B (192.168.2.2) - BlueOS + Pixhawk 2.4.8
    Jetson Orin Nano (192.168.2.3) - ROS 2 companion computer
"""

from .blueos_api import (
    BlueOSAPI,
    BlueOSError,
    MavlinkEndpoint,
    EndpointType,
    SystemInfo,
    ServiceInfo,
)

from .rosbridge_client import (
    ROSBridgeClient,
    ROSBridgeError,
    ROSMessage,
    MAVROSBridge,
)

__all__ = [
    # BlueOS API
    'BlueOSAPI',
    'BlueOSError',
    'MavlinkEndpoint',
    'EndpointType',
    'SystemInfo',
    'ServiceInfo',
    # ROSBridge
    'ROSBridgeClient',
    'ROSBridgeError',
    'ROSMessage',
    'MAVROSBridge',
]
