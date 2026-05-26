#!/usr/bin/env python3
"""
LiDAR 側邊 + 後方避障測試腳本
- 左邊有障礙 → 右轉
- 右邊有障礙 → 左轉
- 後方有障礙 → 往前走
- 兩邊都淨空 → 停止

用法（在 wildbot 容器內）：
  source /opt/ros/jazzy/setup.bash
  source /workspaces/install/setup.bash
  python3 /workspaces/lidar_side_test.py

Ctrl+C 停止，車子會自動停車。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

# ── LiDAR 扇區設定 ──
SIDE_MIN_DEG    = 35.0    # 側邊起始角度（接 front ±35° 邊界）
SIDE_MAX_DEG    = 100.0   # 側邊結束角度（接 rear ≥100° 邊界）
REAR_MIN_DEG    = 100.0   # 後方起始角度（左右各一扇）
REAR_MAX_DEG    = 180.0   # 後方結束角度
LIDAR_MIN_M     = 0.12
LIDAR_MAX_M     = 2.8

# ── 側邊避障參數 ──
SIDE_REACT_M    = 0.45    # 開始反應的距離
SIDE_STOP_M     = 0.20    # 最近距離（全速轉）
MAX_ANG_RAD     = 0.40    # 最大轉速 (rad/s)

# ── 後方避障參數 ──
REAR_REACT_M    = 0.30    # 開始反應的距離（可調）
REAR_STOP_M     = 0.15    # 最近距離（全速前進）
MAX_LIN_MPS     = 0.15    # 最大前進速度 (m/s)


class LidarSideTest(Node):
    def __init__(self):
        super().__init__("lidar_side_test")

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self._cb_scan, qos
        )
        self.pub_cmd = self.create_publisher(
            TwistStamped, "/base_controller/cmd_vel", 10
        )
        self.timer = self.create_timer(0.05, self._tick)

        self._left_m: float | None = None
        self._right_m: float | None = None
        self._rear_m: float | None = None

        self.get_logger().info(
            f"LiDAR 避障測試啟動 "
            f"side(react={SIDE_REACT_M}m stop={SIDE_STOP_M}m max={MAX_ANG_RAD:.2f}rad/s) "
            f"rear(react={REAR_REACT_M}m stop={REAR_STOP_M}m max={MAX_LIN_MPS:.2f}m/s)"
        )
        self.get_logger().info("左/右擋 → 轉向；後方擋 → 往前走。Ctrl+C 停車。")

    # ── 解析 LiDAR ──
    def _cb_scan(self, msg: LaserScan):
        left_ranges = []
        right_ranges = []
        rear_ranges = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (LIDAR_MIN_M < r < LIDAR_MAX_M):
                continue
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
            deg = math.degrees(angle_rad)
            if SIDE_MIN_DEG <= deg <= SIDE_MAX_DEG:
                left_ranges.append(r)
            elif -SIDE_MAX_DEG <= deg <= -SIDE_MIN_DEG:
                right_ranges.append(r)
            elif REAR_MIN_DEG <= abs(deg) <= REAR_MAX_DEG:
                rear_ranges.append(r)

        self._left_m  = min(left_ranges)  if left_ranges  else None
        self._right_m = min(right_ranges) if right_ranges else None
        self._rear_m  = min(rear_ranges)  if rear_ranges  else None

    # ── 控制迴圈 ──
    def _tick(self):
        lm = self._left_m
        rm = self._right_m
        bm = self._rear_m

        def side_intensity(dist: float | None) -> float:
            if dist is None or not math.isfinite(dist) or dist >= SIDE_REACT_M:
                return 0.0
            t = max(0.0, (dist - SIDE_STOP_M) / max(SIDE_REACT_M - SIDE_STOP_M, 0.01))
            return 1.0 - t

        def rear_intensity(dist: float | None) -> float:
            if dist is None or not math.isfinite(dist) or dist >= REAR_REACT_M:
                return 0.0
            t = max(0.0, (dist - REAR_STOP_M) / max(REAR_REACT_M - REAR_STOP_M, 0.01))
            return 1.0 - t

        cl = side_intensity(lm)
        cr = side_intensity(rm)
        cb = rear_intensity(bm)

        angular_z = (cr - cl) * MAX_ANG_RAD   # 左近→右轉(負), 右近→左轉(正)
        linear_x  = cb * MAX_LIN_MPS            # 後方近→前進(正)

        # ── log ──
        lm_s = f"{lm:.2f}m" if lm is not None else " n/a "
        rm_s = f"{rm:.2f}m" if rm is not None else " n/a "
        bm_s = f"{bm:.2f}m" if bm is not None else " n/a "

        actions = []
        if abs(angular_z) > 0.02:
            actions.append(f"{'← 左轉' if angular_z > 0 else '右轉 →'} {abs(angular_z):.2f}rad/s")
        if linear_x > 0.01:
            actions.append(f"↑ 前進 {linear_x:.2f}m/s")
        action_s = "  ".join(actions) if actions else "停止"

        self.get_logger().info(
            f"left={lm_s}  right={rm_s}  rear={bm_s}  |  {action_s}"
        )

        # ── 發布速度 ──
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x  = linear_x
        cmd.twist.angular.z = angular_z
        self.pub_cmd.publish(cmd)

    def stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        self.pub_cmd.publish(cmd)
        self.get_logger().info("已停車。")


def main():
    rclpy.init()
    node = LidarSideTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
