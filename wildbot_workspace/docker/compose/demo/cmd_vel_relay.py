#!/usr/bin/env python3
"""Relay Nav2 /cmd_vel (Twist) → Wildbot /base_controller/cmd_vel (TwistStamped)."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self._pub = self.create_publisher(
            TwistStamped, "/base_controller/cmd_vel", 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._callback, 10)
        self.get_logger().info(
            "Relaying /cmd_vel (Twist) → /base_controller/cmd_vel (TwistStamped)"
        )

    def _callback(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist = msg
        self._pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
