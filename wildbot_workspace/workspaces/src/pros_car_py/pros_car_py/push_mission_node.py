"""
推熊回家任務（push_mission）

策略：
  APPROACH  - YOLO 視覺伺服靠近熊，直到熊卡進前輪擋板
  COLLECT   - 再前進一段確保熊夾緊
  NAV_HOME  - NavigateToPose 回記錄的 AMCL home（熊被擋板推著走）
  SCORE     - 夾爪夾緊 → 肩膀上抬 → 放下 → 任務結束
"""

from __future__ import annotations

import copy
import math
import time
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
try:
    from nav2_msgs.action import NavigateToPose
    _NAV2_AVAILABLE = True
except ImportError:
    NavigateToPose = None
    _NAV2_AVAILABLE = False
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from pros_car_py.arm_controller_2D import ArmController
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.ros_communicator import RosCommunicator


# ─────────────────────────────────────────────
# 工具函式
# ─────────────────────────────────────────────

def _quat_to_yaw(q) -> float:
    """四元數轉 yaw（弧度）。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _angle_diff(a: float, b: float) -> float:
    """a - b，結果在 (-π, π]。"""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _dist_xy(p1, p2) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


# ─────────────────────────────────────────────
# 主節點
# ─────────────────────────────────────────────

class PushMissionHost(RosCommunicator):

    def __init__(self):
        super().__init__(node_name="push_mission")

        # ── 參數 ────────────────────────────────────────
        self.declare_parameter("amcl_wait_timeout_sec", 30.0)

        # 接近
        self.declare_parameter("approach_stop_dist_m", 0.28)
        self.declare_parameter("approach_max_speed_mps", 0.55)
        self.declare_parameter("approach_max_sec", 12.0)
        self.declare_parameter("approach_stall_sec", 0.5)
        self.declare_parameter("approach_stall_delta_m", 0.04)
        self.declare_parameter("approach_stall_min_sec", 4.0)
        self.declare_parameter("approach_stall_max_dist_m", 0.45)
        self.declare_parameter("visual_servo_yaw_deadband_px", 80.0)
        self.declare_parameter("visual_servo_yaw_soft_scale_px", 300.0)
        self.declare_parameter("visual_servo_max_yaw_near", 10.0)
        self.declare_parameter("visual_servo_max_yaw_far", 18.0)
        self.declare_parameter("visual_servo_max_forward_speed_far", 450.0)
        self.declare_parameter("visual_servo_dx_ema_alpha", 0.20)
        self.declare_parameter("visual_servo_depth_ema_alpha", 0.25)
        # YOLO 丟失搜索（參考 bear_mission）
        self.declare_parameter("approach_yolo_lost_grace_sec", 1.2)
        self.declare_parameter("approach_yolo_search_spin_speed", 8.0)
        self.declare_parameter("approach_yolo_explore_forward_sec", 2.0)

        # 收集
        self.declare_parameter("collect_forward_sec", 0.8)

        # 導航回家
        self.declare_parameter("nav_home_timeout_sec", 120.0)
        self.declare_parameter("home_arrival_thresh_m", 0.30)

        # 得分動作
        self.declare_parameter("score_lift_deg", 20.0)
        self.declare_parameter("score_back_max_sec", 5.0)   # 後退上限；看到熊就停
        self.declare_parameter("score_approach_dist_m", 0.30)

        self._home_pose: tuple[float, float, float] | None = None   # (x, y, yaw)
        self._home_pose_amcl = None                                  # geometry_msgs.Pose
        self._mission_thread: threading.Thread | None = None

    # ─────────────────────────────────────────────
    # 內部工具
    # ─────────────────────────────────────────────

    def _get_amcl_xy_yaw(self):
        """取得目前 AMCL 位置 (x, y, yaw)，若無則 None。"""
        msg = self.get_latest_amcl_pose()
        if msg is None:
            return None
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return (p.x, p.y, _quat_to_yaw(q))

    def _send_twist(self, linear_x: float, angular_z: float):
        """直接送 TwistStamped 給底盤。"""
        from geometry_msgs.msg import TwistStamped
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher_cmd_vel.publish(msg)

    # ─────────────────────────────────────────────
    # 任務各階段
    # ─────────────────────────────────────────────

    def _phase_wait_amcl(self, timeout: float) -> bool:
        self.get_logger().info("等待 AMCL …")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.get_latest_amcl_pose()
            if msg is not None:
                self._home_pose_amcl = copy.deepcopy(msg.pose.pose)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                self._home_pose = (p.x, p.y, _quat_to_yaw(q))
                self.get_logger().info(
                    f"Home 記錄：x={p.x:.2f} y={p.y:.2f} yaw={math.degrees(self._home_pose[2]):.1f}°"
                )
                return True
            time.sleep(0.2)
        self.get_logger().warn("AMCL 等待逾時，以 odom 原點作為 home（精度較低）")
        self._home_pose = (0.0, 0.0, 0.0)
        return False

    def _phase_approach(self) -> bool:
        """YOLO 視覺伺服靠近熊，直到 depth < stop_dist 或熊消失。"""
        self.get_logger().info("[approach] 開始靠近熊 …")
        dp = DataProcessor(self)
        nav = Nav2Processing(self, dp)

        stop_dist  = self.get_parameter("approach_stop_dist_m").get_parameter_value().double_value
        max_fwd    = self.get_parameter("visual_servo_max_forward_speed_far").get_parameter_value().double_value
        deadband   = self.get_parameter("visual_servo_yaw_deadband_px").get_parameter_value().double_value
        soft_scale = self.get_parameter("visual_servo_yaw_soft_scale_px").get_parameter_value().double_value
        max_yaw    = self.get_parameter("visual_servo_max_yaw_near").get_parameter_value().double_value
        dx_alpha   = self.get_parameter("visual_servo_dx_ema_alpha").get_parameter_value().double_value
        dep_alpha  = self.get_parameter("visual_servo_depth_ema_alpha").get_parameter_value().double_value

        approach_max_sec  = self.get_parameter("approach_max_sec").get_parameter_value().double_value
        stall_sec         = self.get_parameter("approach_stall_sec").get_parameter_value().double_value
        stall_delta_m     = self.get_parameter("approach_stall_delta_m").get_parameter_value().double_value
        stall_min_sec     = self.get_parameter("approach_stall_min_sec").get_parameter_value().double_value
        stall_max_dist_m  = self.get_parameter("approach_stall_max_dist_m").get_parameter_value().double_value
        lost_grace_sec    = self.get_parameter("approach_yolo_lost_grace_sec").get_parameter_value().double_value
        search_spin_spd   = self.get_parameter("approach_yolo_search_spin_speed").get_parameter_value().double_value
        explore_fwd_sec   = self.get_parameter("approach_yolo_explore_forward_sec").get_parameter_value().double_value
        t_start = time.monotonic()

        last_valid_dist: float | None = None
        last_dx: float = 0.0          # 上次看到熊的水平偏移，決定搜索轉向
        dist_history: list[tuple[float, float]] = []  # (timestamp, dist)
        lost_frames = 0
        t_lost: float | None = None   # 開始丟失的時間點
        dt = 0.05

        while rclpy.ok():
            ti = dp.get_yolo_target_info()
            target_live = (
                ti is not None
                and len(ti) >= 3
                and float(ti[0]) == 1.0
                and float(ti[1]) > 0.0
            )

            bear_detected = (ti is not None and len(ti) >= 3 and float(ti[0]) == 1.0)

            if target_live:
                dist = float(ti[1])
                last_valid_dist = dist
                last_dx = float(ti[2]) if len(ti) >= 3 else 0.0
                lost_frames = 0
                t_lost = None
                now = time.monotonic()
                dist_history.append((now, dist))
                dist_history = [(t, d) for t, d in dist_history if now - t <= stall_sec]

                elapsed = time.monotonic() - t_start
                if (
                    elapsed >= stall_min_sec
                    and dist <= stall_max_dist_m
                    and len(dist_history) >= 5
                ):
                    dists = [d for _, d in dist_history]
                    if max(dists) - min(dists) < stall_delta_m:
                        self.publish_car_control("STOP")
                        self.get_logger().info(
                            f"[approach] 深度停滯 {stall_sec:.1f}s（Δ={max(dists)-min(dists):.3f}m，dist={dist:.2f}m），視為已收集"
                        )
                        return True

                if dist <= stop_dist:
                    self.publish_car_control("STOP")
                    self.get_logger().info(f"[approach] 到達 dist={dist:.2f}m，停止")
                    return True

                wheel_cmd = nav.camera_nav_pid_command(
                    yolo_target_info=ti,
                    max_forward_speed=max_fwd,
                    max_forward_speed_far=max_fwd,
                    yaw_deadband_px=deadband,
                    yaw_soft_scale_px=soft_scale,
                    max_yaw_speed=max_yaw,
                    dx_ema_alpha=dx_alpha,
                    depth_ema_alpha=dep_alpha,
                )
                self.publish_raw_car_control(wheel_cmd)
            elif bear_detected:
                # 有偵測到熊但 depth=0 → 只做方向修正
                lost_frames = 0
                dx = float(ti[2]) if len(ti) >= 3 else 0.0
                if dx > deadband:
                    self.publish_raw_car_control([max_yaw, -max_yaw, max_yaw, -max_yaw])
                elif dx < -deadband:
                    self.publish_raw_car_control([-max_yaw, max_yaw, -max_yaw, max_yaw])
                else:
                    self.publish_car_control("STOP")
            else:
                lost_frames += 1
                # 靠太近消失 → 視為已收集
                if last_valid_dist is not None and last_valid_dist <= stop_dist * 1.5 and lost_frames >= 5:
                    self.publish_car_control("STOP")
                    self.get_logger().info("[approach] 熊靠太近消失，視為已收集")
                    return True
                # 三段式搜索：grace → spin → explore
                if t_lost is None:
                    t_lost = time.monotonic()
                time_lost = time.monotonic() - t_lost
                spin_cycle = lost_grace_sec + 4.0  # 每個搜索週期長度
                phase_in_cycle = time_lost % spin_cycle
                if phase_in_cycle < lost_grace_sec:
                    # Grace：原地停等
                    self.publish_car_control("STOP")
                elif phase_in_cycle < lost_grace_sec + (spin_cycle - lost_grace_sec - explore_fwd_sec):
                    # Spin：朝上次看到熊的方向轉
                    spd = search_spin_spd
                    if last_dx >= 0:
                        self.publish_raw_car_control([spd, -spd, spd, -spd])
                    else:
                        self.publish_raw_car_control([-spd, spd, -spd, spd])
                else:
                    # Explore：往前走再繼續找
                    self.publish_car_control("FORWARD_SLOW")

            if time.monotonic() - t_start >= approach_max_sec:
                self.publish_car_control("STOP")
                self.get_logger().info(
                    f"[approach] 接近逾時 {approach_max_sec:.0f}s，視為已收集進入下一階段"
                )
                return True

            time.sleep(dt)

        return False

    def _phase_collect(self):
        """前進一段時間確保熊卡進擋板之間。"""
        sec = self.get_parameter("collect_forward_sec").get_parameter_value().double_value
        self.get_logger().info(f"[collect] 前進 {sec:.1f}s 確保熊進入收集區 …")
        self.publish_car_control("FORWARD_SLOW")
        time.sleep(sec)
        self.publish_car_control("STOP")
        time.sleep(0.3)

    def _phase_nav_home(self) -> bool:
        """使用 NavigateToPose（AMCL）推熊回 home；若無 Nav2 則 fallback 直線推回。"""
        nav_timeout = self.get_parameter("nav_home_timeout_sec").get_parameter_value().double_value
        threshold   = self.get_parameter("home_arrival_thresh_m").get_parameter_value().double_value

        if self._home_pose is None:
            self.get_logger().error("[nav_home] home 位置未記錄")
            return False

        if not _NAV2_AVAILABLE or self._home_pose_amcl is None:
            self.get_logger().warn("[nav_home] Nav2 不可用，改用 heading 直線推回法")
            return self._phase_push_home_fallback()

        self.get_logger().info("[nav_home] 送出 NavigateToPose goal 推熊回 home …")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = copy.deepcopy(self._home_pose_amcl)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        client: ActionClient = self.navigate_to_pose_action_client
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("[nav_home] NavigateToPose action server 未就緒，fallback")
            return self._phase_push_home_fallback()

        send_future = client.send_goal_async(nav_goal)
        t0 = time.monotonic()
        while not send_future.done() and time.monotonic() - t0 < 30.0:
            time.sleep(0.02)

        if not send_future.done():
            self.get_logger().error("[nav_home] send_goal_async 超時")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                "[nav_home] Goal 被拒絕。"
                "請確認已設 initial pose 後執行 ./scripts/restart_navigation.sh"
            )
            return False

        self.get_logger().info("[nav_home] Goal 接受，等待到達 home …")
        result_future = goal_handle.get_result_async()
        t0 = time.monotonic()
        last_log = 0.0

        while not result_future.done() and time.monotonic() - t0 < nav_timeout:
            now = time.monotonic()
            if now - last_log >= 3.0:
                cur = self._get_amcl_xy_yaw()
                if cur is not None and self._home_pose is not None:
                    dist = _dist_xy(cur, self._home_pose)
                    self.get_logger().info(f"[nav_home] 距 home {dist:.2f}m …")
                last_log = now
            time.sleep(0.05)

        if not result_future.done():
            self.get_logger().warn(f"[nav_home] 導航逾時（{nav_timeout:.0f}s）")
            return False

        nav_result = result_future.result()
        status = nav_result.status
        nav_succeeded = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f"[nav_home] 導航結果 status={status} succeeded={nav_succeeded}")

        cur = self._get_amcl_xy_yaw()
        if cur is not None and self._home_pose is not None:
            dist = _dist_xy(cur, self._home_pose)
            self.get_logger().info(f"[nav_home] 到達後距 home = {dist:.2f}m（門檻={threshold:.2f}m）")
            if dist <= threshold:
                return True

        return nav_succeeded

    def _phase_push_home_fallback(self) -> bool:
        """直線推熊回 home（heading 修正，Nav2 不可用時的備援）。"""
        if self._home_pose is None:
            self.get_logger().error("[push_fallback] home 位置未記錄")
            return False

        push_spd  = 0.20
        ang_kp    = 1.0
        ang_max   = 0.5
        threshold = self.get_parameter("home_arrival_thresh_m").get_parameter_value().double_value

        hx, hy, _ = self._home_pose
        self.get_logger().info(f"[push_fallback] 直線推熊回 home ({hx:.2f}, {hy:.2f}) …")

        timeout = time.monotonic() + 60.0

        while rclpy.ok() and time.monotonic() < timeout:
            cur = self._get_amcl_xy_yaw()
            if cur is None:
                time.sleep(0.05)
                continue

            cx, cy, cyaw = cur
            dist = _dist_xy(cur, self._home_pose)

            if dist <= threshold:
                self.publish_car_control("STOP")
                self.get_logger().info(f"[push_fallback] 到達 home（dist={dist:.2f}m）")
                return True

            target_yaw = math.atan2(hy - cy, hx - cx)
            err_yaw = _angle_diff(target_yaw, cyaw)
            angular_z = max(-ang_max, min(ang_max, ang_kp * err_yaw))
            self._send_twist(push_spd, angular_z)
            time.sleep(0.05)

        self.publish_car_control("STOP")
        self.get_logger().warn("[push_fallback] 推熊逾時")
        return False

    def _phase_score(self):
        """後退看熊 → 視覺靠近 → 夾起上抬 → 放下。"""
        self.get_logger().info("[score] 開始得分動作 …")
        dp = DataProcessor(self)
        nav = Nav2Processing(self, dp)
        arm = ArmController(self, dp)

        lift_deg          = self.get_parameter("score_lift_deg").get_parameter_value().double_value
        score_back_max_sec = self.get_parameter("score_back_max_sec").get_parameter_value().double_value
        score_stop_dist   = self.get_parameter("score_approach_dist_m").get_parameter_value().double_value
        deadband          = self.get_parameter("visual_servo_yaw_deadband_px").get_parameter_value().double_value
        soft_scale        = self.get_parameter("visual_servo_yaw_soft_scale_px").get_parameter_value().double_value
        max_yaw           = self.get_parameter("visual_servo_max_yaw_near").get_parameter_value().double_value
        dx_alpha          = self.get_parameter("visual_servo_dx_ema_alpha").get_parameter_value().double_value
        dep_alpha         = self.get_parameter("visual_servo_depth_ema_alpha").get_parameter_value().double_value

        # 步驟 1：後退直到 YOLO 看到熊（至少退 0.5s，最多 score_back_max_sec）
        self.get_logger().info(f"[score] 後退中，等待 YOLO 偵測到熊（上限 {score_back_max_sec:.1f}s）…")
        t_back = time.monotonic()
        while rclpy.ok() and time.monotonic() - t_back < score_back_max_sec:
            self.publish_car_control("BACKWARD_SLOW")
            time.sleep(0.05)
            # 至少退 0.5 秒再開始偵測，避免熊貼近時立刻跳過
            if time.monotonic() - t_back < 0.5:
                continue
            ti = dp.get_yolo_target_info()
            if ti is not None and len(ti) >= 3 and float(ti[0]) == 1.0 and float(ti[1]) > 0.0:
                self.get_logger().info(f"[score] 偵測到熊 dist={float(ti[1]):.2f}m，停止後退")
                break
        self.publish_car_control("STOP")
        time.sleep(0.3)

        # 步驟 2：視覺靠近熊
        self.get_logger().info("[score] 視覺靠近熊 …")
        lost_frames = 0
        dt = 0.05
        t_limit = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < t_limit:
            ti = dp.get_yolo_target_info()
            target_live = (
                ti is not None and len(ti) >= 3
                and float(ti[0]) == 1.0 and float(ti[1]) > 0.0
            )
            if target_live:
                dist = float(ti[1])
                lost_frames = 0
                if dist <= score_stop_dist:
                    self.publish_car_control("STOP")
                    self.get_logger().info(f"[score] 靠近完成 dist={dist:.2f}m")
                    break
                wheel_cmd = nav.camera_nav_pid_command(
                    yolo_target_info=ti,
                    max_forward_speed=200.0,
                    max_forward_speed_far=200.0,
                    yaw_deadband_px=deadband,
                    yaw_soft_scale_px=soft_scale,
                    max_yaw_speed=max_yaw,
                    dx_ema_alpha=dx_alpha,
                    depth_ema_alpha=dep_alpha,
                )
                self.publish_raw_car_control(wheel_cmd)
            else:
                lost_frames += 1
                if lost_frames >= 20:
                    self.publish_car_control("STOP")
                    self.get_logger().warn("[score] 找不到熊，直接執行夾取")
                    break
            time.sleep(dt)
        self.publish_car_control("STOP")
        time.sleep(0.3)

        # 步驟 3：夾緊
        self.get_logger().info("[score] 夾緊 …")
        arm._smooth_move_to([None, None, arm.joint_limits[2]["min_angle"]], step=5.0, delay=0.1)
        time.sleep(0.8)

        # 步驟 4：上抬
        self.get_logger().info(f"[score] 上抬 {lift_deg:.0f}° …")
        shoulder_lift = arm.joint_angles[0] - lift_deg
        shoulder_lift = max(arm.joint_limits[0]["min_angle"], shoulder_lift)
        arm._smooth_move_to([shoulder_lift, None, None], step=5.0, delay=0.1)
        time.sleep(0.8)

        # 步驟 5：在高點放開夾爪（讓熊落在得分區）
        self.get_logger().info("[score] 放開夾爪 …")
        arm._smooth_move_to([None, None, arm.joint_limits[2]["max_angle"]], step=5.0, delay=0.1)
        time.sleep(0.5)

        # 步驟 6：手臂回歸待機位置
        self.get_logger().info("[score] 手臂歸位 …")
        arm._smooth_move_to([arm.joint_limits[0]["init"], None, None], step=5.0, delay=0.1)
        time.sleep(0.3)

        self.get_logger().info("[score] 得分動作完成！")

    # ─────────────────────────────────────────────
    # 主流程
    # ─────────────────────────────────────────────

    def _run_mission(self):
        time.sleep(1.0)

        timeout = self.get_parameter("amcl_wait_timeout_sec").get_parameter_value().double_value

        # 1. 等 AMCL，記錄 home
        self._phase_wait_amcl(timeout)

        # 2. 手臂伸出收集姿態
        self.publish_robot_arm_angle([3.67, 0.5, 3.0])
        time.sleep(1.5)

        # 3. 靠近熊
        if not self._phase_approach():
            self.get_logger().error("接近失敗，任務中止")
            return

        # 4. 收集（前進夾緊）
        self._phase_collect()

        # 5. NavigateToPose 推熊回 home
        self._phase_nav_home()

        # 確保 Nav2 殘留速度清除
        self.publish_car_control("STOP")
        time.sleep(0.8)

        # 6. 得分
        self._phase_score()

        self.get_logger().info("=== 推熊任務完成 ===")

    def start(self):
        self._mission_thread = threading.Thread(target=self._run_mission, daemon=True)
        self._mission_thread.start()


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PushMissionHost()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_car_control("STOP")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
