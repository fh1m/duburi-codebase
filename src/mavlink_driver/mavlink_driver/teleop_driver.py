#!/usr/bin/env python3
"""
Teleop driver - subscribes to /cmd_vel (Twist) and publishes TeleopCommand.

Maps Twist axes to a TeleopCommand carrying all 4 axes at once.

Axis mapping:
  - linear.x  -> forward / back
  - linear.y  -> left / right
  - linear.z  -> up / down
  - angular.z -> yaw left/right
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from duburi_interfaces.msg import TeleopCommand

_DZ = 0.1


class TeleopDriverNode(Node):
    """Converts Twist commands to TeleopCommand for the inspector."""

    def __init__(self):
        super().__init__('teleop_driver')
        self._cmd_pub = self.create_publisher(
            TeleopCommand, '/driver/teleop',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        self._twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self._twist_cb,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        )
        self._scale_linear = self.declare_parameter('scale_linear', 50.0).value
        self._scale_angular = self.declare_parameter('scale_angular', 50.0).value
        self._max_speed = self.declare_parameter('max_speed', 200).value

        self._last_was_idle = False

        self.get_logger().info(
            'Teleop driver ready (multi-axis). Publishing to /driver/teleop')

    def _twist_cb(self, msg: Twist):
        fwd = msg.linear.x if abs(msg.linear.x) > _DZ else 0.0
        lat = msg.linear.y if abs(msg.linear.y) > _DZ else 0.0
        vert = msg.linear.z if abs(msg.linear.z) > _DZ else 0.0
        yaw = msg.angular.z if abs(msg.angular.z) > _DZ else 0.0

        if fwd == 0.0 and lat == 0.0 and vert == 0.0 and yaw == 0.0:
            if not self._last_was_idle:
                cmd = TeleopCommand()
                cmd.idle = True
                cmd.speed = self._max_speed
                self._cmd_pub.publish(cmd)
                self._last_was_idle = True
            return

        self._last_was_idle = False

        cmd = TeleopCommand()
        cmd.idle = False
        cmd.linear_x = float(fwd)
        cmd.linear_y = float(lat)
        cmd.linear_z = float(vert)
        cmd.angular_z = float(yaw)
        cmd.speed = self._max_speed
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
