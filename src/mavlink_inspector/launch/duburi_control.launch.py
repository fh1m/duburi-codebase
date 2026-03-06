#!/usr/bin/env python3
"""Launch Duburi 4.2 control stack: inspector + logger."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_log_dir = str(Path.home() / 'auv_logs')
    return LaunchDescription([
        DeclareLaunchArgument('connection_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('enable_logger', default_value='true'),
        DeclareLaunchArgument('log_directory', default_value=default_log_dir),
        Node(
            package='mavlink_inspector',
            executable='inspector',
            name='mavlink_inspector',
            parameters=[{'connection_port': LaunchConfiguration('connection_port')}],
        ),
        Node(
            package='mavlink_logger',
            executable='logger',
            name='mavlink_logger',
            condition=IfCondition(LaunchConfiguration('enable_logger')),
            parameters=[{'log_directory': LaunchConfiguration('log_directory')}],
        ),
    ])
