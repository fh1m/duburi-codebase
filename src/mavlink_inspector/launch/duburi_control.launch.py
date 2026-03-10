#!/usr/bin/env python3
"""Launch Duburi 4.2 control stack: inspector + logger.

Supports loading parameters from a YAML config file::

    ros2 launch mavlink_inspector duburi_control.launch.py \\
        params_file:=$(ros2 pkg prefix mavlink_inspector)/share/mavlink_inspector/config/pool_test.yaml

If no params_file is given, defaults.yaml is loaded automatically.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mavlink_inspector')
    default_params = os.path.join(pkg_share, 'config', 'defaults.yaml')
    default_log_dir = str(Path.home() / 'auv_logs')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to YAML parameter file'),
        DeclareLaunchArgument(
            'enable_logger', default_value='true'),
        DeclareLaunchArgument(
            'log_directory', default_value=default_log_dir),
        Node(
            package='mavlink_inspector',
            executable='inspector',
            name='mavlink_inspector',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='mavlink_logger',
            executable='logger',
            name='mavlink_logger',
            condition=IfCondition(LaunchConfiguration('enable_logger')),
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
