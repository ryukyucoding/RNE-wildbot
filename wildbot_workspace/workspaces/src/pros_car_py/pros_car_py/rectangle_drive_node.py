"""
長方形自動走行（rectangle_drive）

行為：前進 → 右轉 90° × 3 次，共 4 段直線。
使用 wheel odom 閉迴路，不需 AMCL / Nav2 / 地圖。

用法（wildbot 容器內）：
  cd /workspaces
  colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash
  ros2 run pros_car_py rectangle_drive
  ros2 run pros_car_py rectangle_drive --ros-args -p length_m:=2.0 -p width_m:=1.0 -p turn_deg:=85.0
  ros2 run pros_car_py rectangle_drive --ros-args \
    -p side1_m:=0.85 -p side2_m:=2.95 -p side3_m:=0.85 -p side4_m:=2.95 \
    -p turn1_deg:=48.5 -p turn2_deg:=49.0 -p turn3_deg:=50.0
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from pros_car_py.ros_communicator import RosCommunicator

# ── 預設參數（可用 ros-args 覆寫） ──
DEFAULT_LENGTH_M = 0.85
DEFAULT_WIDTH_M = 2.95
# 實車慣性會多滑一點，預設少轉一點以補償（目標約 90° 時可設 85° 左右）
DEFAULT_TURN_DEG = 48.5
TURN_TOLERANCE_DEG = 2.0
FORWARD_ACTION = "FORWARD_SLOW"
TURN_ACTION = "CLOCKWISE_ROTATION"
FORWARD_PULSE_SEC = 0.15
TURN_PULSE_SEC = 0.12
SEGMENT_TIMEOUT_SEC = 30.0
ODOM_WAIT_TIMEOUT_SEC = 10.0
NUM_SIDES = 4
# 待機姿勢：夾爪上提、打開（與 bear_mission 實車初始化一致；3.5 偏低會朝下）
DEFAULT_ARM_STANDBY = [3.67, 0.5, 4.0]
ARM_STOW_WAIT_SEC = 1.5


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


class RectangleDriveNode(RosCommunicator):
    def __init__(self):
        super().__init__("rectangle_drive")
        self.declare_parameter("length_m", DEFAULT_LENGTH_M)
        self.declare_parameter("width_m", DEFAULT_WIDTH_M)
        self.declare_parameter("turn_deg", DEFAULT_TURN_DEG)
        self.declare_parameter("raise_arm_on_start", True)
        length_m = self.get_parameter("length_m").get_parameter_value().double_value
        width_m = self.get_parameter("width_m").get_parameter_value().double_value
        turn_deg = self.get_parameter("turn_deg").get_parameter_value().double_value
        default_turn = min(max(30.0, float(turn_deg)), 90.0)
        self.length_m = max(0.1, float(length_m))
        self.width_m = max(0.1, float(width_m))
        default_sides = [
            self.length_m,
            self.width_m,
            self.length_m,
            self.width_m,
        ]
        self.side_lengths_m = []
        for i in range(1, NUM_SIDES + 1):
            pname = f"side{i}_m"
            self.declare_parameter(pname, default_sides[i - 1])
            val = self.get_parameter(pname).get_parameter_value().double_value
            self.side_lengths_m.append(max(0.1, float(val)))
        self.turn_degs = []
        for i in range(1, NUM_SIDES):
            pname = f"turn{i}_deg"
            self.declare_parameter(pname, default_turn)
            val = self.get_parameter(pname).get_parameter_value().double_value
            self.turn_degs.append(min(max(30.0, float(val)), 90.0))
        self.raise_arm_on_start = (
            self.get_parameter("raise_arm_on_start")
            .get_parameter_value()
            .bool_value
        )
        self._mission_thread: threading.Thread | None = None
        self._abort = False
        self.get_logger().info(
            "Rectangle drive config: "
            f"sides=[{', '.join(f'{d:.2f}m' for d in self.side_lengths_m)}] "
            f"(length={self.length_m:.2f}m, width={self.width_m:.2f}m 為 side 預設), "
            f"turns=[{', '.join(f'{d:.1f}°' for d in self.turn_degs)}] "
            f"±{TURN_TOLERANCE_DEG:.1f}° (慣性補償：目標角小於 90°), "
            f"forward={FORWARD_ACTION}, turn_action={TURN_ACTION}, "
            f"pulse(forward={FORWARD_PULSE_SEC:.2f}s, turn={TURN_PULSE_SEC:.2f}s), "
            f"raise_arm_on_start={self.raise_arm_on_start}"
        )

    def _raise_arm_standby(self) -> None:
        shoulder, elbow, gripper = DEFAULT_ARM_STANDBY
        self.get_logger().info(
            f"手臂待機（夾爪上提）: "
            f"[{shoulder:.2f}, {elbow:.2f}, {gripper:.2f}] …"
        )
        self.publish_robot_arm_angle(DEFAULT_ARM_STANDBY)
        time.sleep(ARM_STOW_WAIT_SEC)

    @staticmethod
    def _segment_timeout_for(distance_m: float) -> float:
        return max(SEGMENT_TIMEOUT_SEC, distance_m / 0.12 * 1.5)

    def _wait_for_odom(self, timeout_sec: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            if self.get_latest_odom() is not None:
                return True
            time.sleep(0.05)
        return False

    def _get_xy_yaw(self) -> tuple[float, float, float] | None:
        odom = self.get_latest_odom()
        if odom is None:
            return None
        pose = odom.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            _yaw_from_quat(pose.orientation),
        )

    def _drive_forward(self, distance_m: float, side_idx: int) -> bool:
        segment_timeout_sec = self._segment_timeout_for(distance_m)
        start = self._get_xy_yaw()
        if start is None:
            self.get_logger().error(f"[side {side_idx}] 無 odom，前進段中止。")
            return False

        x0, y0, _ = start
        t0 = time.monotonic()
        last_log = 0.0

        self.get_logger().info(
            f"[side {side_idx}] 前進 {distance_m:.2f}m "
            f"(起點 x={x0:.3f}, y={y0:.3f})"
        )

        while rclpy.ok() and not self._abort:
            if time.monotonic() - t0 > segment_timeout_sec:
                self.publish_car_control("STOP")
                self.get_logger().warn(
                    f"[side {side_idx}] 前進 timeout（>{segment_timeout_sec:.1f}s）。"
                )
                return False

            cur = self._get_xy_yaw()
            if cur is None:
                self.publish_car_control("STOP")
                time.sleep(0.05)
                continue

            x, y, _ = cur
            traveled = math.hypot(x - x0, y - y0)
            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    f"[side {side_idx}] 已走 {traveled:.3f}m / {distance_m:.3f}m"
                )
                last_log = now

            if traveled >= distance_m:
                self.publish_car_control("STOP")
                time.sleep(0.03)
                self.get_logger().info(
                    f"[side {side_idx}] 前進完成，實際 {traveled:.3f}m"
                )
                return True

            self.publish_car_control(FORWARD_ACTION)
            time.sleep(FORWARD_PULSE_SEC)
            self.publish_car_control("STOP")
            time.sleep(0.03)

        self.publish_car_control("STOP")
        return False

    def _turn_right(self, turn_deg: float, turn_idx: int) -> bool:
        target_rad = -math.radians(turn_deg)
        tol_rad = math.radians(TURN_TOLERANCE_DEG)
        segment_timeout_sec = SEGMENT_TIMEOUT_SEC
        start = self._get_xy_yaw()
        if start is None:
            self.get_logger().error(f"[turn {turn_idx}] 無 odom，轉彎段中止。")
            return False

        _, _, start_yaw = start
        accumulated = 0.0
        last_yaw = start_yaw
        t0 = time.monotonic()
        last_log = 0.0

        self.get_logger().info(
            f"[turn {turn_idx}] 右轉 {turn_deg:.1f}° "
            f"(起點 yaw={math.degrees(start_yaw):.1f}°)"
        )

        while rclpy.ok() and not self._abort:
            if time.monotonic() - t0 > segment_timeout_sec:
                self.publish_car_control("STOP")
                self.get_logger().warn(
                    f"[turn {turn_idx}] 轉彎 timeout（>{segment_timeout_sec:.1f}s）。"
                )
                return False

            cur = self._get_xy_yaw()
            if cur is None:
                self.publish_car_control("STOP")
                time.sleep(0.05)
                continue

            _, _, yaw = cur
            dyaw = _normalize_angle(yaw - last_yaw)
            accumulated += dyaw
            last_yaw = yaw

            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    f"[turn {turn_idx}] 已轉 {math.degrees(accumulated):+.1f}° "
                    f"/ {math.degrees(target_rad):+.1f}°"
                )
                last_log = now

            if accumulated <= target_rad + tol_rad:
                self.publish_car_control("STOP")
                time.sleep(0.03)
                self.get_logger().info(
                    f"[turn {turn_idx}] 轉彎完成，實際 {math.degrees(accumulated):+.1f}°"
                )
                return True

            self.publish_car_control(TURN_ACTION)
            time.sleep(TURN_PULSE_SEC)
            self.publish_car_control("STOP")
            time.sleep(0.03)

        self.publish_car_control("STOP")
        return False

    def _run_rectangle(self) -> None:
        if not self._wait_for_odom(ODOM_WAIT_TIMEOUT_SEC):
            self.get_logger().error(
                f"等待 odom timeout（>{ODOM_WAIT_TIMEOUT_SEC:.1f}s），任務中止。"
            )
            return

        self.get_logger().info(
            f"=== 開始走行（"
            f"{' → '.join(f'{d:.2f}m' for d in self.side_lengths_m)}）==="
        )

        for side_idx in range(1, NUM_SIDES + 1):
            if not rclpy.ok() or self._abort:
                break

            distance_m = self.side_lengths_m[side_idx - 1]
            if not self._drive_forward(distance_m, side_idx):
                self.get_logger().error(f"第 {side_idx} 邊前進失敗，任務中止。")
                return

            if side_idx < NUM_SIDES:
                if not self._turn_right(self.turn_degs[side_idx - 1], side_idx):
                    self.get_logger().error(f"第 {side_idx} 次轉彎失敗，任務中止。")
                    return

        self.publish_car_control("STOP")
        if rclpy.ok() and not self._abort:
            self.get_logger().info("=== 長方形走行完成 ===")

    def start(self) -> None:
        self._mission_thread = threading.Thread(
            target=self._run_rectangle, daemon=True
        )
        self._mission_thread.start()

    def stop(self) -> None:
        self._abort = True
        self.publish_car_control("STOP")


def main(args=None):
    rclpy.init(args=args)
    node = RectangleDriveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
