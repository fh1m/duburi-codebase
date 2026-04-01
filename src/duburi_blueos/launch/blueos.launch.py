#!/usr/bin/env python3
"""
Launch file for BlueOS integration nodes.

Starts:
  - blueos_monitor: System status monitoring
  - mavlink_bridge: MAVLink endpoint management
  - mavros_bridge: ROS1 MAVROS to ROS2 bridge via rosbridge

Usage:
  ros2 launch duburi_blueos blueos.launch.py
  ros2 launch duburi_blueos blueos.launch.py blueos_host:=192.168.2.2 jetson_ip:=192.168.2.3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('duburi_blueos')
    config_file = os.path.join(pkg_dir, 'config', 'blueos_config.yaml')

    # Declare launch arguments
    blueos_host_arg = DeclareLaunchArgument(
        'blueos_host',
        default_value='192.168.2.2',
        description='BlueOS IP address (Raspberry Pi)',
    )

    jetson_ip_arg = DeclareLaunchArgument(
        'jetson_ip',
        default_value='192.168.2.3',
        description='Jetson Orin Nano IP address',
    )

    mavlink_port_arg = DeclareLaunchArgument(
        'mavlink_port',
        default_value='14550',
        description='MAVLink UDP port for Jetson',
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='8889',
        description='ROSBridge WebSocket port on BlueOS',
    )

    auto_setup_arg = DeclareLaunchArgument(
        'auto_setup',
        default_value='true',
        description='Auto-create Jetson endpoint on startup',
    )

    # BlueOS Monitor Node
    blueos_monitor_node = Node(
        package='duburi_blueos',
        executable='blueos_monitor',
        name='blueos_monitor',
        parameters=[
            config_file,
            {
                'blueos_host': LaunchConfiguration('blueos_host'),
            },
        ],
        output='screen',
    )

    # MAVLink Bridge Node
    mavlink_bridge_node = Node(
        package='duburi_blueos',
        executable='mavlink_bridge',
        name='mavlink_bridge',
        parameters=[
            config_file,
            {
                'blueos_host': LaunchConfiguration('blueos_host'),
                'jetson_ip': LaunchConfiguration('jetson_ip'),
                'mavlink_port': LaunchConfiguration('mavlink_port'),
                'auto_setup': LaunchConfiguration('auto_setup'),
            },
        ],
        output='screen',
    )

    # MAVROS Bridge Node (ROS1 -> ROS2 via rosbridge)
    mavros_bridge_node = Node(
        package='duburi_blueos',
        executable='mavros_bridge',
        name='mavros_bridge',
        parameters=[
            {
                'blueos_host': LaunchConfiguration('blueos_host'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'reconnect': True,
                'reconnect_interval': 2.0,
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        blueos_host_arg,
        jetson_ip_arg,
        mavlink_port_arg,
        rosbridge_port_arg,
        auto_setup_arg,
        blueos_monitor_node,
        mavlink_bridge_node,
        mavros_bridge_node,
    ])
