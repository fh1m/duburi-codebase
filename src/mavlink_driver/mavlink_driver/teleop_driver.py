#!/usr/bin/env python3
"""
Teleop driver - subscribes to /cmd_vel (Twist) and publishes DriverCommand.

Maps simultaneous Twist axes to a SINGLE 'teleop' DriverCommand carrying
all 4 axes at once.  This avoids the old pattern where per-axis commands
stomped _current_movement in the inspector.

Axis mapping:
  - linear.x  → forward / back  (speed field, PWM offset)
  - linear.y  → left / right    (duration field, +right / -left)
  - linear.z  → up / down       (depth field, +up / -down)
  - angular.z → yaw left/right  (angle field, +CCW / -CW)
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

        # BUG4 FIX: Track whether we've already sent teleop_idle so we
        # don't flood the inspector with idle commands every tick.
        self._last_was_idle = False

        self.get_logger().info('Teleop driver ready (multi-axis). Publishing to /driver/command')

    def _twist_cb(self, msg: Twist):
        """Convert Twist to a single teleop DriverCommand.

        BUG3 FIX: All axes combined into ONE command.  The inspector's
        'teleop' handler reads speed/duration/depth/angle as forward/
        lateral/throttle/yaw PWM offsets, sets all channels at once.

        BUG4 FIX: When all axes are in the dead-zone, send 'teleop_idle'
        ONCE (clears movement without nuking depth PID / yaw hold).
        """
        fwd = msg.linear.x if abs(msg.linear.x) > _DZ else 0.0
        lat = msg.linear.y if abs(msg.linear.y) > _DZ else 0.0
        vert = msg.linear.z if abs(msg.linear.z) > _DZ else 0.0
        yaw = msg.angular.z if abs(msg.angular.z) > _DZ else 0.0

        if fwd == 0.0 and lat == 0.0 and vert == 0.0 and yaw == 0.0:
            # All dead-zone → idle
            if not self._last_was_idle:
                cmd = DriverCommand()
                cmd.command = 'teleop_idle'
                self._cmd_pub.publish(cmd)
                self._last_was_idle = True
            return

        self._last_was_idle = False

        cmd = DriverCommand()
        cmd.command = 'teleop'
        # Encode PWM offsets: scale * magnitude, preserve sign
        cmd.speed = int(self._scale_linear * fwd)       # forward (+) / back (-)
        cmd.duration = self._scale_linear * lat          # right (+) / left (-)
        cmd.depth = self._scale_linear * vert            # up (+) / down (-)
        cmd.angle = self._scale_angular * yaw            # CCW/left (+) / CW/right (-)
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
