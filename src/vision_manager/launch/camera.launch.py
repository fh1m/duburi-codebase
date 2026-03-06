"""Launch file for vision_manager camera node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0',
                              description='V4L2 device index'),
        DeclareLaunchArgument('frame_width', default_value='640',
                              description='Capture width'),
        DeclareLaunchArgument('frame_height', default_value='480',
                              description='Capture height'),
        DeclareLaunchArgument('fps', default_value='30',
                              description='Target framerate'),
        DeclareLaunchArgument('camera_name', default_value='duburi_cam',
                              description='Logical camera name'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Path to calibration YAML'),

        Node(
            package='vision_manager',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'camera_name': LaunchConfiguration('camera_name'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
        ),
    ])
