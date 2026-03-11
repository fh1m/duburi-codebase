"""
cameras.launch.py – Launch camera_manager node for vision_inspector.

This is the primary launch file: it starts the multi-camera orchestrator
which reads cameras.yaml, opens all configured cameras, and begins
publishing Image + CameraInfo on per-camera namespaced topics.

Usage:
    ros2 launch vision_inspector cameras.launch.py
    ros2 launch vision_inspector cameras.launch.py config_file:=/path/to/cameras.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_inspector')
    default_config = os.path.join(pkg_share, 'config', 'cameras.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to cameras.yaml configuration file',
        ),

        Node(
            package='vision_inspector',
            executable='camera_manager',
            name='camera_manager',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
            }],
        ),
    ])
