"""Launch file for the YOLO detector node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='yolov8n.pt',
                              description='YOLO model file'),
        DeclareLaunchArgument('confidence', default_value='0.5',
                              description='Detection confidence threshold'),
        DeclareLaunchArgument('device', default_value='cuda:0',
                              description='Inference device (cpu or cuda:0)'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw',
                              description='Input image topic'),
        DeclareLaunchArgument('enable_display', default_value='False',
                              description='Show OpenCV preview window'),
        DeclareLaunchArgument('publish_annotated', default_value='True',
                              description='Publish annotated image topic'),

        Node(
            package='vision',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'confidence': LaunchConfiguration('confidence'),
                'device': LaunchConfiguration('device'),
                'image_topic': LaunchConfiguration('image_topic'),
                'enable_display': LaunchConfiguration('enable_display'),
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }],
        ),
    ])
