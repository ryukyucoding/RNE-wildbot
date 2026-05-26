#!/usr/bin/env python3
"""Bridge Wildbot wheel odom → /odom + broadcast odom→base_link TF for SLAM/AMCL."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    def __init__(self):
        super().__init__("odom_tf_bridge")
        self._tf = TransformBroadcaster(self)
        self._pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(
            Odometry, "/base_controller/odom", self._callback, 10
        )
        self.get_logger().info(
            "Bridging /base_controller/odom → /odom + TF odom→base_link"
        )

    def _pose_is_valid(self, msg: Odometry) -> bool:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in vals):
            return False
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        return math.isfinite(norm) and norm > 1e-6

    def _callback(self, msg: Odometry):
        if not self._pose_is_valid(msg):
            self.get_logger().warn(
                "Skip invalid /base_controller/odom (NaN/Inf pose) — "
                "確認 kros_car 已啟動且車體 odom 正常",
                throttle_duration_sec=5.0,
            )
            return

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose = msg.pose
        odom.twist = msg.twist
        self._pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self._tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
