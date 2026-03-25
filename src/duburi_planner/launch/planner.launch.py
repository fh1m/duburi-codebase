"""Launch file for duburi_planner — starts mission_node with config + YASMIN viewer.

Usage:
    ros2 launch duburi_planner planner.launch.py
    ros2 launch duburi_planner planner.launch.py dive_depth:=0.8 gate.search_timeout:=60

To watch the FSM in real time, also start the YASMIN viewer node:
    ros2 run yasmin_viewer yasmin_viewer_node
Then open http://localhost:5000/ in a browser and select DUBURI_MISSION.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('duburi_planner')
    default_config = os.path.join(pkg_share, 'config', 'planner.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to planner YAML config'),
        DeclareLaunchArgument(
            'enable_viewer', default_value='true',
            description='Launch YASMIN viewer web server'),

        # ── Mission node ──────────────────────────────────────────────
        Node(
            package='duburi_planner',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),

        # ── YASMIN Viewer (Flask web UI on port 5000) ─────────────────
        Node(
            package='yasmin_viewer',
            executable='yasmin_viewer_node',
            name='yasmin_viewer_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_viewer')),
        ),
    ])
