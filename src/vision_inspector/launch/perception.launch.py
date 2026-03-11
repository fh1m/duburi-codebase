"""
perception.launch.py – Launch full perception stack (cameras + detector).

Starts the vision_inspector camera_manager, then the detector_node
from the vision package. This is the "turn-key" launch for the full
perception pipeline.

Usage:
    ros2 launch vision_inspector perception.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vi_share = get_package_share_directory('vision_inspector')
    default_config = os.path.join(vi_share, 'config', 'cameras.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='cameras.yaml path',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/forward/image_raw',
            description='Image topic for the detector',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n.pt',
            description='YOLO model path',
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Detection confidence threshold',
        ),

        # ── Camera manager ───────────────────────────────────────
        Node(
            package='vision_inspector',
            executable='camera_manager',
            name='camera_manager',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
            }],
        ),

        # ── Detector ─────────────────────────────────────────────
        Node(
            package='vision',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            }],
        ),
    ])
