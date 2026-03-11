"""Launch file for the vision pipeline: camera + detector + alignment controller.

Starts camera_manager (publishes /camera/forward/image_raw), detector_node,
and alignment_controller. The display shows live detections with bounding boxes.

Alignment controller modes:
  use_pid:=false  use_kalman:=false   -> raw proportional (pool test first)
  use_pid:=true   use_kalman:=false   -> PID smoothing only
  use_pid:=true   use_kalman:=true    -> full pipeline (KF + PID)

Equivalent ros run commands (no launch, manual start in 3 terminals):
  # Terminal 1: Camera (publishes /camera/forward/image_raw) – uses package config by default
  ros2 run vision_inspector camera_manager
  # Terminal 2: Detector + display
  ros2 run vision detector_node --ros-args -p enable_display:=true
  # Terminal 3: Alignment controller
  ros2 run vision alignment_controller

  # Simpler: standalone detector (direct camera, no ROS topics, single command)
  ros2 run vision detector_standalone
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    try:
        vi_share = get_package_share_directory('vision_inspector')
        default_config = os.path.join(vi_share, 'config', 'cameras.yaml')
    except Exception:
        default_config = ''

    return LaunchDescription([
        # ── Camera config (for camera_manager) ────────────────────────
        DeclareLaunchArgument(
            "config_file", default_value=default_config,
            description="Path to cameras.yaml (vision_inspector)"),

        # ── Detector arguments ────────────────────────────────────────
        DeclareLaunchArgument(
            "model", default_value="yolo11n.pt",
            description="YOLO model file"),
        DeclareLaunchArgument(
            "confidence", default_value="0.5",
            description="Detection confidence threshold"),
        DeclareLaunchArgument(
            "device", default_value="auto",
            description="Inference device: auto (GPU if available, else CPU), cpu, or cuda:0"),
        DeclareLaunchArgument(
            "image_topic", default_value="/camera/forward/image_raw",
            description="Input image topic"),
        DeclareLaunchArgument(
            "enable_display", default_value="True",
            description="Show OpenCV preview window"),
        DeclareLaunchArgument(
            "publish_annotated", default_value="True",
            description="Publish annotated image topic"),

        # ── Alignment controller arguments ────────────────────────────
        DeclareLaunchArgument(
            "target_class", default_value="person",
            description="Object class to track and align to"),
        DeclareLaunchArgument(
            "use_pid", default_value="True",
            description="Enable PID controllers (False = proportional only)"),
        DeclareLaunchArgument(
            "use_kalman", default_value="True",
            description="Enable Kalman filter tracking (False = raw detections)"),
        DeclareLaunchArgument(
            "max_speed", default_value="200",
            description="Maximum PWM offset from 1500 neutral"),
        DeclareLaunchArgument(
            "control_rate", default_value="10.0",
            description="Alignment control loop frequency (Hz)"),

        # ── Camera manager (publishes /camera/forward/image_raw) ───────
        Node(
            package="vision_inspector",
            executable="camera_manager",
            name="camera_manager",
            output="screen",
            parameters=[{
                "config_file": LaunchConfiguration("config_file"),
            }],
        ),

        # ── Detector node ─────────────────────────────────────────────
        Node(
            package="vision",
            executable="detector_node",
            name="detector_node",
            output="screen",
            parameters=[{
                "model": LaunchConfiguration("model"),
                "confidence": LaunchConfiguration("confidence"),
                "device": LaunchConfiguration("device"),
                "image_topic": LaunchConfiguration("image_topic"),
                "enable_display": LaunchConfiguration("enable_display"),
                "publish_annotated": LaunchConfiguration("publish_annotated"),
                "target_class": LaunchConfiguration("target_class"),
                "show_alignment": True,
            }],
        ),

        # ── Alignment controller node ─────────────────────────────────
        Node(
            package="vision",
            executable="alignment_controller",
            name="alignment_controller",
            output="screen",
            parameters=[{
                "target_class": LaunchConfiguration("target_class"),
                "use_pid": LaunchConfiguration("use_pid"),
                "use_kalman": LaunchConfiguration("use_kalman"),
                "max_speed": LaunchConfiguration("max_speed"),
                "control_rate": LaunchConfiguration("control_rate"),
            }],
        ),
    ])
