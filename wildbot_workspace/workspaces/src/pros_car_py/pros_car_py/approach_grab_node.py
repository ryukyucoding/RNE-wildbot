"""
接近並抓取（approach_grab）

行為：前進 → 左轉 → 前進 → 在三個角度各嘗試夾取一次（odom 閉迴路，不需 YOLO / Nav2）。

用法（wildbot 容器內）：
  cd /workspaces
  colcon build --packages-select pros_car_py --symlink-install && source install/setup.bash
  ros2 run pros_car_py approach_grab
  ros2 run pros_car_py approach_grab --ros-args \
    -p forward1_m:=0.85 -p forward2_m:=0.55 -p turn_deg:=48.5 \
    -p grab_turn1_deg:=16.0 -p grab_turn2_deg:=32.0 -p grab_turn3_deg:=48.0
"""

from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING

import rclpy
from rclpy.executors import MultiThreadedExecutor

from pros_car_py.ros_communicator import RosCommunicator

if TYPE_CHECKING:
    from pros_car_py.arm_controller_2D import ArmController

DEFAULT_FORWARD1_M = 0.85
DEFAULT_FORWARD2_M = 0.55
DEFAULT_TURN_DEG = 48.5
DEFAULT_GRAB_TURNS_DEG = (16.0, 32.0, 48.0)
TURN_TOLERANCE_DEG = 2.0
FORWARD_ACTION = "FORWARD_SLOW"
TURN_RIGHT_ACTION = "CLOCKWISE_ROTATION"
TURN_LEFT_ACTION = "COUNTERCLOCKWISE_ROTATION"
FORWARD_PULSE_SEC = 0.15
TURN_PULSE_SEC = 0.12
SEGMENT_TIMEOUT_SEC = 30.0
ODOM_WAIT_TIMEOUT_SEC = 10.0
SETTLE_SEC = 0.2
DEFAULT_ARM_STANDBY = [3.67, 0.5, 4.0]
ARM_STOW_WAIT_SEC = 1.5
NUM_GRAB_ATTEMPTS = 3


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


class ApproachGrabNode(RosCommunicator):
    def __init__(self):
        super().__init__("approach_grab")
        self.declare_parameter("forward1_m", DEFAULT_FORWARD1_M)
        self.declare_parameter("forward2_m", DEFAULT_FORWARD2_M)
        self.declare_parameter("turn_deg", DEFAULT_TURN_DEG)
        self.declare_parameter("grab_turn1_deg", DEFAULT_GRAB_TURNS_DEG[0])
        self.declare_parameter("grab_turn2_deg", DEFAULT_GRAB_TURNS_DEG[1])
        self.declare_parameter("grab_turn3_deg", DEFAULT_GRAB_TURNS_DEG[2])
        self.declare_parameter("grab_turn_direction", "left")
        self.declare_parameter("grasp_pause_sec", 1.0)
        self.declare_parameter("reopen_gripper_between_tries", True)
        self.declare_parameter("raise_arm_on_start", True)
        self.declare_parameter("stop_after_grab_attempts", NUM_GRAB_ATTEMPTS)

        self.forward1_m = max(0.1, self._double_param("forward1_m"))
        self.forward2_m = max(0.1, self._double_param("forward2_m"))
        self.turn_deg = min(max(0.0, self._double_param("turn_deg")), 180.0)
        self.grab_turn_degs = [
            max(0.0, self._double_param(f"grab_turn{i}_deg"))
            for i in range(1, NUM_GRAB_ATTEMPTS + 1)
        ]
        direction = str(self.get_parameter("grab_turn_direction").value).strip().lower()
        self.grab_turn_direction = "left" if direction == "left" else "right"
        self.grasp_pause_sec = max(0.0, self._double_param("grasp_pause_sec"))
        self.reopen_gripper_between_tries = self._bool_param(
            "reopen_gripper_between_tries"
        )
        self.raise_arm_on_start = self._bool_param("raise_arm_on_start")
        self.num_grab_attempts = min(
            NUM_GRAB_ATTEMPTS,
            max(1, int(self._double_param("stop_after_grab_attempts"))),
        )

        self.arm_controller: ArmController | None = None
        self._mission_thread: threading.Thread | None = None
        self._abort = False

        self.get_logger().info(
            "Approach grab config: "
            f"forward1={self.forward1_m:.2f}m, turn={self.turn_deg:.1f}° (left), "
            f"forward2={self.forward2_m:.2f}m, "
            f"grab_turns=[{', '.join(f'{d:.1f}°' for d in self.grab_turn_degs[:self.num_grab_attempts])}] "
            f"({self.grab_turn_direction}), "
            f"attempts={self.num_grab_attempts}, "
            f"reopen_between={self.reopen_gripper_between_tries}, "
            f"raise_arm_on_start={self.raise_arm_on_start}"
        )

    def _double_param(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    def _bool_param(self, name: str) -> bool:
        return bool(self.get_parameter(name).get_parameter_value().bool_value)

    def set_arm_controller(self, arm: ArmController) -> None:
        self.arm_controller = arm

    def _raise_arm_standby(self) -> None:
        self.get_logger().info(
            f"手臂待機: {DEFAULT_ARM_STANDBY} …"
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

    def _drive_forward(self, distance_m: float, segment_idx: int) -> bool:
        segment_timeout_sec = self._segment_timeout_for(distance_m)
        start = self._get_xy_yaw()
        if start is None:
            self.get_logger().error(f"[forward {segment_idx}] 無 odom，前進段中止。")
            return False

        x0, y0, _ = start
        t0 = time.monotonic()
        last_log = 0.0

        self.get_logger().info(
            f"[forward {segment_idx}] 前進 {distance_m:.2f}m "
            f"(起點 x={x0:.3f}, y={y0:.3f})"
        )

        while rclpy.ok() and not self._abort:
            if time.monotonic() - t0 > segment_timeout_sec:
                self.publish_car_control("STOP")
                self.get_logger().warn(
                    f"[forward {segment_idx}] 前進 timeout（>{segment_timeout_sec:.1f}s）。"
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
                    f"[forward {segment_idx}] 已走 {traveled:.3f}m / {distance_m:.3f}m"
                )
                last_log = now

            if traveled >= distance_m:
                self.publish_car_control("STOP")
                time.sleep(0.03)
                self.get_logger().info(
                    f"[forward {segment_idx}] 前進完成，實際 {traveled:.3f}m"
                )
                return True

            self.publish_car_control(FORWARD_ACTION)
            time.sleep(FORWARD_PULSE_SEC)
            self.publish_car_control("STOP")
            time.sleep(0.03)

        self.publish_car_control("STOP")
        return False

    def _turn(self, turn_deg: float, turn_idx: int, direction: str) -> bool:
        if turn_deg <= 0.0:
            return True

        is_right = direction == "right"
        target_rad = -math.radians(turn_deg) if is_right else math.radians(turn_deg)
        turn_action = TURN_RIGHT_ACTION if is_right else TURN_LEFT_ACTION
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
        dir_label = "右轉" if is_right else "左轉"

        self.get_logger().info(
            f"[turn {turn_idx}] {dir_label} {turn_deg:.1f}° "
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

            if is_right:
                reached = accumulated <= target_rad + tol_rad
            else:
                reached = accumulated >= target_rad - tol_rad

            if reached:
                self.publish_car_control("STOP")
                time.sleep(0.03)
                self.get_logger().info(
                    f"[turn {turn_idx}] 轉彎完成，實際 {math.degrees(accumulated):+.1f}°"
                )
                return True

            self.publish_car_control(turn_action)
            time.sleep(TURN_PULSE_SEC)
            self.publish_car_control("STOP")
            time.sleep(0.03)

        self.publish_car_control("STOP")
        return False

    def _run_grab_attempts(self) -> bool:
        if self.arm_controller is None:
            self.get_logger().error("ArmController 未設定，無法夾取。")
            return False

        arm = self.arm_controller
        for attempt in range(1, self.num_grab_attempts + 1):
            if not rclpy.ok() or self._abort:
                return False

            turn_deg = self.grab_turn_degs[attempt - 1]
            if turn_deg > 0.0:
                if not self._turn(
                    turn_deg, attempt, self.grab_turn_direction
                ):
                    self.get_logger().error(
                        f"第 {attempt} 次抓取前轉向失敗，任務中止。"
                    )
                    return False

            self.publish_car_control("STOP")
            time.sleep(SETTLE_SEC)

            self.get_logger().info(f"[grab {attempt}/{self.num_grab_attempts}] 開始夾取 …")
            try:
                arm.run_grasp_blocking()
            except Exception as exc:
                self.get_logger().error(f"[grab {attempt}] 夾取失敗: {exc}")
                return False

            self.get_logger().info(f"[grab {attempt}/{self.num_grab_attempts}] 夾取動作完成。")

            if (
                self.reopen_gripper_between_tries
                and attempt < self.num_grab_attempts
                and rclpy.ok()
                and not self._abort
            ):
                self.get_logger().info(
                    f"[grab {attempt}] 打開夾爪，準備下一角度 …"
                )
                self._raise_arm_standby()
                time.sleep(self.grasp_pause_sec)

        return True

    def _run_mission(self) -> None:
        if not self._wait_for_odom(ODOM_WAIT_TIMEOUT_SEC):
            self.get_logger().error(
                f"等待 odom timeout（>{ODOM_WAIT_TIMEOUT_SEC:.1f}s），任務中止。"
            )
            return

        if self.raise_arm_on_start:
            self._raise_arm_standby()

        self.get_logger().info("=== 開始 approach_grab 走行 ===")

        if not self._drive_forward(self.forward1_m, 1):
            self.get_logger().error("第一段前進失敗，任務中止。")
            return

        if not self._turn(self.turn_deg, 1, "left"):
            self.get_logger().error("轉彎失敗，任務中止。")
            return

        if not self._drive_forward(self.forward2_m, 2):
            self.get_logger().error("第二段前進失敗，任務中止。")
            return

        self.get_logger().info("=== 走行完成，開始多角度夾取 ===")

        if not self._run_grab_attempts():
            return

        self.publish_car_control("STOP")
        if rclpy.ok() and not self._abort:
            self.get_logger().info("=== approach_grab 任務完成 ===")

    def start(self) -> None:
        self._mission_thread = threading.Thread(
            target=self._run_mission, daemon=True
        )
        self._mission_thread.start()

    def stop(self) -> None:
        self._abort = True
        self.publish_car_control("STOP")


def main(args=None):
    from pros_car_py.arm_controller_2D import ArmController
    from pros_car_py.data_processor import DataProcessor

    rclpy.init(args=args)
    node = ApproachGrabNode()
    data_processor = DataProcessor(node)
    arm_controller = ArmController(node, data_processor)
    node.set_arm_controller(arm_controller)

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
