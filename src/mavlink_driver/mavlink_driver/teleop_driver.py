#!/usr/bin/env python3
"""
Teleop driver - subscribes to /cmd_vel (Twist) and publishes DriverCommand.

Maps simultaneous Twist axes to compound movement commands:
  - linear.x  → forward / back
  - linear.y  → left / right  (positive = right)
  - linear.z  → up / down     (positive = up)
  - angular.z → yaw_left / yaw_right

Horizontal axes are combined into compound diagonals (e.g. move_forward_right).
Vertical (linear.z) is sent as a separate command because depth PID
silently overrides CH_THROTTLE, making 3-axis compounds unreliable.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from duburi_interfaces.msg import DriverCommand


# Dead-zone threshold
_DZ = 0.1


class TeleopDriverNode(Node):
    """Converts Twist commands to DriverCommand for the inspector."""

    def __init__(self):
        super().__init__('teleop_driver')
        self._cmd_pub = self.create_publisher(
            DriverCommand, '/driver/command',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        self._twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self._twist_cb,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1)
        )
        self._scale_linear = self.declare_parameter('scale_linear', 50.0).value
        self._scale_angular = self.declare_parameter('scale_angular', 50.0).value
        self.get_logger().info('Teleop driver ready (multi-axis). Publishing to /driver/command')

    def _twist_cb(self, msg: Twist):
        """Convert Twist to one or two DriverCommands.

        Horizontal axes (x, y) are combined into a single command.
        Vertical (z) and yaw (angular.z) are sent separately if active.
        """
        published = False

        # ── Horizontal compound ──────────────────────────────────────────
        dirs: list[str] = []
        max_mag = 0.0
        if abs(msg.linear.x) > _DZ:
            dirs.append('forward' if msg.linear.x > 0 else 'back')
            max_mag = max(max_mag, abs(msg.linear.x))
        if abs(msg.linear.y) > _DZ:
            dirs.append('right' if msg.linear.y > 0 else 'left')
            max_mag = max(max_mag, abs(msg.linear.y))

        if dirs:
            cmd = DriverCommand()
            cmd.command = 'move_' + '_'.join(dirs)
            cmd.speed = int(self._scale_linear * max_mag)
            cmd.duration = 0
            self._cmd_pub.publish(cmd)
            published = True

        # ── Vertical ─────────────────────────────────────────────────────
        if abs(msg.linear.z) > _DZ:
            cmd = DriverCommand()
            cmd.command = 'move_up' if msg.linear.z > 0 else 'move_down'
            cmd.speed = int(self._scale_linear * abs(msg.linear.z))
            cmd.duration = 0
            self._cmd_pub.publish(cmd)
            published = True

        # ── Yaw ──────────────────────────────────────────────────────────
        if abs(msg.angular.z) > _DZ:
            cmd = DriverCommand()
            cmd.command = 'yaw_left' if msg.angular.z > 0 else 'yaw_right'
            cmd.speed = int(self._scale_angular * abs(msg.angular.z))
            cmd.duration = 0
            self._cmd_pub.publish(cmd)
            published = True

        # ── All axes zero → stop ─────────────────────────────────────────
        if not published:
            cmd = DriverCommand()
            cmd.command = 'stop'
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
