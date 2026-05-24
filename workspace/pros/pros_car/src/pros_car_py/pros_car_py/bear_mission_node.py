"""
任務一（整段自動）：訂閱對側 YOLO 的 topic → 視覺逼近 → 夾取 → NavigateToPose 回記錄的起點。

典型用法（兩個環境／容器，同一 ROS_DOMAIN_ID）：
  - ros2_yolo_integration：`ros2 run yolo_example_pkg yolo_node --ros-args -p target_class:=bear ...`
  - pros_car：`ros2 run pros_car_py bear_mission`，或 `ros2 launch pros_car_py bear_task1.launch.py`（僅 bear_mission）

預設 **auto_start:=true**：延遲後自動跑完整流程；手動觸發：`auto_start:=false` 再呼叫 `/start_bear_mission`。

請勿與 `robot_control` 同時對底盤發輪速。

前提：Nav2、`/navigate_to_pose`；車端 Docker **網路**須與 pros_app localization 同一 bridge（見 pros_car `car_control.sh` 之 ROS_BRIDGE_NETWORK）。預設訂閱 `amcl_pose_topic`=/amcl_pose；對側已發 `/yolo/target_info`、`/yolo/target_marker`。
"""

from __future__ import annotations

import copy
import math
import os
import threading
import time
from typing import Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from pros_car_py.arm_controller_2D import ArmController
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.obstacle_guard import ObstacleGuard, get_lidar_sector_minimums, get_lidar_rear_minimum, _finite_clearance
from pros_car_py.ros_communicator import RosCommunicator


class BearMissionHost(RosCommunicator):
    _GOAL_STATUS_TEXT = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN(未知)",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED(已接受)",
        GoalStatus.STATUS_EXECUTING: "EXECUTING(執行中)",
        GoalStatus.STATUS_CANCELING: "CANCELING(取消中)",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED(成功)",
        GoalStatus.STATUS_CANCELED: "CANCELED(已取消)",
        GoalStatus.STATUS_ABORTED: "ABORTED(中止/失敗)",
    }

    def __init__(self):
        super().__init__(node_name="bear_mission")

        self.get_logger().info(
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '') or '(unset→與 .env 預設相同，常為 0)'}。"
            "須與 **yolo、localization** 容器完全一致（在 yolo 裡 `echo $ROS_DOMAIN_ID`；若不一致則收不到 /amcl_pose）。"
            "ROS_BRIDGE_NETWORK 只能是 **docker network ls** 裡的網路名，勿設成 /amcl_pose。"
        )

        self.declare_parameter("use_unity_camera_nav", False)
        self.declare_parameter("unity_stow_elbow_enabled", True)
        self.declare_parameter("unity_stow_elbow_deg", 180.0)
        self.declare_parameter("approach_period_sec", 0.1)
        self.declare_parameter("approach_timeout_sec", 120.0)
        self.declare_parameter("visual_servo_enabled", True)
        self.declare_parameter("visual_servo_target_depth_m", 0.55)
        self.declare_parameter("visual_servo_search_spin_speed", 130.0)
        self.declare_parameter("visual_servo_max_forward_speed", 170.0)
        self.declare_parameter("visual_servo_max_forward_speed_far", 300.0)
        self.declare_parameter("visual_servo_far_distance_m", 0.90)
        self.declare_parameter("visual_servo_yaw_deadband_px", 12.0)
        self.declare_parameter("visual_servo_yaw_soft_scale_px", 100.0)
        self.declare_parameter("visual_servo_max_yaw_near", 175.0)
        self.declare_parameter("visual_servo_max_yaw_far", 300.0)
        self.declare_parameter("visual_servo_min_yaw_large_px", 115.0)
        self.declare_parameter("approach_turn_stuck_time_sec", 2.0)
        self.declare_parameter("approach_stuck_time_sec", 3.0)
        self.declare_parameter("approach_stuck_back_sec", 0.55)
        self.declare_parameter("approach_stuck_shift_sec", 0.45)
        self.declare_parameter("approach_stuck_forward_sec", 0.35)
        self.declare_parameter("motion_stuck_min_progress_m", 0.06)
        self.declare_parameter("visual_servo_lost_timeout_sec", 0.8)
        self.declare_parameter("approach_yolo_lost_grace_sec", 1.5)
        self.declare_parameter("approach_yolo_lost_min_frames", 12)
        self.declare_parameter("approach_yolo_search_spin_speed_tier", "slow")
        self.declare_parameter("approach_yolo_lost_search_circle_rad", 6.283185307)
        self.declare_parameter("approach_yolo_explore_forward_sec", 2.0)
        self.declare_parameter("approach_yolo_explore_forward_speed", 180.0)
        self.declare_parameter("approach_yolo_lost_motion_log_sec", 2.0)
        self.declare_parameter("align_pixel_thresh", 40.0)
        self.declare_parameter("align_pixel_bias_px", 0.0)
        self.declare_parameter("align_stable_frames", 5)
        # 切入慢速對齊的距離閾值
        self.declare_parameter("approach_slow_dist_m", 0.90)
        # 低於此深度 or 熊消失 → 立刻停下夾取
        self.declare_parameter("grasp_trigger_dist_m", 0.65)
        # 熊剛消失後再多等幾幀再夾（等車身慣性停穩）
        self.declare_parameter("grasp_trigger_lost_frames", 3)
        self.declare_parameter("grasp_depth_jump_m", 0.35)
        # 前瞻煞車：用 v²/(2a) 估算「現在這個速度要多久、多遠才停得住」
        self.declare_parameter("approach_stop_dist_m", 0.55)
        self.declare_parameter("approach_decel_mps2", 0.45)
        self.declare_parameter("approach_max_speed_mps", 0.55)
        self.declare_parameter("approach_brake_safety_m", 0.12)
        self.declare_parameter("visual_servo_center_deadband_px", 32.0)
        self.declare_parameter("visual_servo_image_half_width_px", 320.0)
        self.declare_parameter("grasp_depth_max_m", 0.62)
        self.declare_parameter("grasp_depth_min_m", 0.12)
        self.declare_parameter("marker_wait_sec", 3.0)
        self.declare_parameter("nav_home_timeout_sec", 180.0)
        self.declare_parameter("home_reached_dist_thresh_m", 0.30)
        self.declare_parameter("nav_retry_count", 1)
        self.declare_parameter("nav_feedback_log_sec", 1.5)
        self.declare_parameter("nav_stuck_time_sec", 8.0)
        self.declare_parameter("nav_stuck_min_progress_m", 0.08)
        self.declare_parameter("nav_unstick_enabled", True)
        self.declare_parameter("nav_unstick_back_sec", 0.8)
        self.declare_parameter("nav_unstick_rotate_sec", 0.8)
        self.declare_parameter("fallback_home_enabled", True)
        self.declare_parameter("fallback_home_timeout_sec", 25.0)
        self.declare_parameter("fallback_arrival_xy_thresh_m", 0.22)
        self.declare_parameter("fallback_heading_thresh_deg", 10.0)
        self.declare_parameter("fallback_rotate_step_sec", 0.18)
        self.declare_parameter("fallback_forward_step_sec", 0.20)
        self.declare_parameter("drop_at_home", True)
        # True：先回到 home 位置 → 放下娃娃 → 再轉回任務開始時的面向
        self.declare_parameter("drop_before_final_heading", True)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_start_delay_sec", 5.0)
        self.declare_parameter("amcl_wait_timeout_sec", 120.0)
        self.declare_parameter("obstacle_guard_enabled", True)
        self.declare_parameter("obstacle_stop_m", -1.0)
        self.declare_parameter("obstacle_slow_m", -1.0)
        self.declare_parameter("obstacle_side_stop_m", -1.0)
        self.declare_parameter("obstacle_depth_max_m", -1.0)
        self.declare_parameter("obstacle_lidar_max_m", -1.0)
        self.declare_parameter("nav_obstacle_guard_enabled", True)
        self.declare_parameter("nav_obstacle_stop_m", -1.0)
        self.declare_parameter("obstacle_log_interval_sec", 1.0)
        self.declare_parameter("obstacle_source_debug_enabled", True)
        self.declare_parameter("obstacle_scale_floor_far", 0.60)
        self.declare_parameter("obstacle_scale_floor_mid", 0.30)
        self.declare_parameter("obstacle_scale_floor_slow", 0.20)

        self._mission_busy = threading.Lock()
        self._running = False
        self._last_nav_feedback_distance = None
        self._last_nav_feedback_time = 0.0
        self._last_nav_progress_time = 0.0
        self._nav_stuck_min_progress_m = 0.08
        self._last_home_dist = None

        self._srv = self.create_service(Trigger, "start_bear_mission", self._cb_start)

        if self.get_parameter("auto_start").get_parameter_value().bool_value:
            delay = max(
                0.0,
                self.get_parameter("auto_start_delay_sec")
                .get_parameter_value()
                .double_value,
            )
            self._auto_start_timer = self.create_timer(delay, self._on_auto_start_timer)
            self.get_logger().info(
                f"auto_start enabled: full pipeline begins in {delay:.1f}s "
                "(record home → approach → grasp → nav home). "
                "Set auto_start:=false to use /start_bear_mission only."
            )
        else:
            self._auto_start_timer = None
            self.get_logger().info(
                "auto_start disabled; call `ros2 service call /start_bear_mission std_srvs/srv/Trigger`."
            )

    def _on_auto_start_timer(self):
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None
        ok, msg = self._begin_mission_if_idle()
        if ok:
            self.get_logger().info(f"auto_start: {msg}")
        else:
            self.get_logger().warn(f"auto_start skipped: {msg}")

    def _begin_mission_if_idle(self) -> Tuple[bool, str]:
        if not self._mission_busy.acquire(blocking=False):
            return False, "mission lock busy"
        try:
            if self._running:
                return False, "mission already running"
            self._running = True
        finally:
            self._mission_busy.release()

        threading.Thread(target=self._run_mission_safe, daemon=True).start()
        return True, "mission thread started"

    def _cb_start(self, _req: Trigger.Request, response: Trigger.Response):
        ok, msg = self._begin_mission_if_idle()
        response.success = ok
        response.message = msg
        return response

    def _run_mission_safe(self):
        try:
            self._run_mission()
        finally:
            self._running = False
            if rclpy.ok():
                try:
                    self.publish_car_control("STOP")
                except Exception:
                    pass

    def _build_obstacle_guard(self, use_unity: bool) -> ObstacleGuard:
        guard = ObstacleGuard.from_profile(use_unity)
        stop_m = self.get_parameter("obstacle_stop_m").get_parameter_value().double_value
        slow_m = self.get_parameter("obstacle_slow_m").get_parameter_value().double_value
        side_m = (
            self.get_parameter("obstacle_side_stop_m").get_parameter_value().double_value
        )
        depth_max = (
            self.get_parameter("obstacle_depth_max_m").get_parameter_value().double_value
        )
        lidar_max = (
            self.get_parameter("obstacle_lidar_max_m").get_parameter_value().double_value
        )
        if stop_m > 0.0:
            guard.stop_m = stop_m
        if slow_m > 0.0:
            guard.slow_m = max(slow_m, guard.stop_m + 0.05)
        if side_m > 0.0:
            guard.side_stop_m = side_m
        if depth_max > 0.0:
            guard.depth_max_m = depth_max
        if lidar_max > 0.0:
            guard.lidar_max_m = lidar_max
        return guard

    @staticmethod
    def _approach_zone_name(dist_m: float, zone_far: float, zone_mid: float) -> str:
        if dist_m > zone_far:
            return "far"
        if dist_m > zone_mid:
            return "mid"
        return "slow"

    def _obstacle_speed_scale_floor(
        self,
        dist_m: float,
        zone_far: float,
        zone_mid: float,
    ) -> float:
        zone = self._approach_zone_name(dist_m, zone_far, zone_mid)
        if zone == "far":
            return (
                self.get_parameter("obstacle_scale_floor_far")
                .get_parameter_value()
                .double_value
            )
        if zone == "mid":
            return (
                self.get_parameter("obstacle_scale_floor_mid")
                .get_parameter_value()
                .double_value
            )
        return (
            self.get_parameter("obstacle_scale_floor_slow")
            .get_parameter_value()
            .double_value
        )

    def _evaluate_obstacles(
        self,
        dp: DataProcessor,
        guard: ObstacleGuard,
        approach_target_depth_m: float | None = None,
        approach_mode: bool = False,
        speed_scale_floor: float | None = None,
    ):
        return guard.evaluate(
            lidar_sectors=get_lidar_sector_minimums(dp),
            multi_depth=dp.get_camera_x_multi_depth(),
            sector_depth=dp.get_obstacle_sector_depth(),
            approach_target_depth_m=approach_target_depth_m,
            approach_mode=approach_mode,
            speed_scale_floor=speed_scale_floor,
            lidar_rear_m=get_lidar_rear_minimum(dp),
        )

    @staticmethod
    def _reset_motion_progress(state: dict) -> None:
        state["last_progress_time"] = time.monotonic()
        state["last_pose_xy"] = None

    @staticmethod
    def _update_motion_progress(
        state: dict,
        pose_msg,
        min_progress_m: float,
        min_progress_yaw_rad: float | None = None,
    ) -> None:
        if pose_msg is None:
            return
        x = float(pose_msg.pose.pose.position.x)
        y = float(pose_msg.pose.pose.position.y)
        last = state.get("last_pose_xy")
        if last is not None:
            if math.hypot(x - last[0], y - last[1]) >= min_progress_m:
                state["last_progress_time"] = time.monotonic()
        state["last_pose_xy"] = (x, y)
        if min_progress_yaw_rad is not None:
            yaw = BearMissionHost._yaw_from_quat(pose_msg.pose.pose.orientation)
            last_yaw = state.get("last_pose_yaw")
            if last_yaw is not None:
                dyaw = abs(BearMissionHost._normalize_angle(yaw - last_yaw))
                if dyaw >= min_progress_yaw_rad:
                    state["last_progress_time"] = time.monotonic()
            state["last_pose_yaw"] = yaw

    @staticmethod
    def _reset_yolo_search_state(state: dict) -> None:
        state["phase"] = "idle"
        state["accumulated_rad"] = 0.0
        state["last_yaw"] = None
        state["explore_until"] = 0.0
        state["logged_spin"] = False

    @staticmethod
    def _target_live(yolo_target_info) -> bool:
        if yolo_target_info is None or len(yolo_target_info) < 3:
            return False
        return (
            float(yolo_target_info[0]) == 1.0
            and float(yolo_target_info[1]) > 0.0
        )

    @staticmethod
    def _spin_action_names(speed_tier: str) -> tuple[str, str]:
        tier = (speed_tier or "slow").strip().lower()
        if tier == "median":
            return (
                "COUNTERCLOCKWISE_ROTATION_MEDIAN",
                "CLOCKWISE_ROTATION_MEDIAN",
            )
        if tier in ("fast", "full"):
            return ("COUNTERCLOCKWISE_ROTATION", "CLOCKWISE_ROTATION")
        return (
            "COUNTERCLOCKWISE_ROTATION_SLOW",
            "CLOCKWISE_ROTATION_SLOW",
        )

    @staticmethod
    def _normalize_spin_action_tier(action: str, speed_tier: str) -> str:
        ccw, cw = BearMissionHost._spin_action_names(speed_tier)
        if action in (
            "COUNTERCLOCKWISE_ROTATION",
            "COUNTERCLOCKWISE_ROTATION_MEDIAN",
            "COUNTERCLOCKWISE_ROTATION_SLOW",
        ):
            return ccw
        if action in (
            "CLOCKWISE_ROTATION",
            "CLOCKWISE_ROTATION_MEDIAN",
            "CLOCKWISE_ROTATION_SLOW",
        ):
            return cw
        return action

    @staticmethod
    def _track_yolo_search_yaw(state: dict, pose_msg, max_step_rad: float = 0.28) -> float:
        """Accumulate absolute yaw during confirmed yolo-lost search spin."""
        if pose_msg is None:
            return float(state.get("accumulated_rad", 0.0))
        yaw = BearMissionHost._yaw_from_quat(pose_msg.pose.pose.orientation)
        last_yaw = state.get("last_yaw")
        if last_yaw is not None:
            dyaw = abs(BearMissionHost._normalize_angle(yaw - last_yaw))
            dyaw = min(dyaw, max(0.05, float(max_step_rad)))
            state["accumulated_rad"] = float(state.get("accumulated_rad", 0.0)) + dyaw
        state["last_yaw"] = yaw
        return float(state.get("accumulated_rad", 0.0))

    @staticmethod
    def _motion_is_stuck(state: dict, stuck_time_sec: float) -> bool:
        return (
            time.monotonic() - state["last_progress_time"] > stuck_time_sec
        )

    @staticmethod
    def _yolo_search_action_key(
        obs=None, dx_px: float | None = None, spin_tier: str = "slow"
    ) -> str:
        """Pick mapped spin action for yolo-lost search."""
        ccw, cw = BearMissionHost._spin_action_names(spin_tier)
        if dx_px is not None:
            if dx_px < -20.0:
                return ccw
            if dx_px > 20.0:
                return cw
        if obs is not None:
            left = obs.left_clearance_m
            right = obs.right_clearance_m
            if math.isfinite(left) and math.isfinite(right):
                if right + 0.05 < left:
                    return ccw
                if left + 0.05 < right:
                    return cw
        return ccw

    @staticmethod
    def _yolo_search_wheel_cmd(
        spin_speed: float, obs=None, dx_px: float | None = None
    ) -> list[float]:
        """Spin in place to search; prefer YOLO dx, else LiDAR open side."""
        spin = abs(float(spin_speed))
        ccw = [-spin, spin, -spin, spin]
        cw = [spin, -spin, spin, -spin]

        if dx_px is not None:
            if dx_px < -20.0:
                return ccw
            if dx_px > 20.0:
                return cw

        if obs is not None:
            left = obs.left_clearance_m
            right = obs.right_clearance_m
            if math.isfinite(left) and math.isfinite(right):
                if right + 0.05 < left:
                    return ccw
                if left + 0.05 < right:
                    return cw
        return ccw

    def _pick_yolo_lost_spin_action(
        self,
        guard: ObstacleGuard,
        obs,
        dx_px: float | None,
        spin_tier: str = "slow",
    ) -> str:
        ccw, cw = self._spin_action_names(spin_tier)
        action = self._yolo_search_action_key(
            obs=obs, dx_px=dx_px, spin_tier=spin_tier
        )
        if obs.block_cmd in (
            "CLOCKWISE_ROTATION",
            "CLOCKWISE_ROTATION_MEDIAN",
            "CLOCKWISE_ROTATION_SLOW",
            "COUNTERCLOCKWISE_ROTATION",
            "COUNTERCLOCKWISE_ROTATION_MEDIAN",
            "COUNTERCLOCKWISE_ROTATION_SLOW",
        ):
            action = self._normalize_spin_action_tier(obs.block_cmd, spin_tier)
        elif obs.block_cmd == "STOP" and obs.speed_scale <= 0.05:
            return "STOP"
        safe = guard.side_stop_m + 0.05
        if action == ccw and obs.left_clearance_m < safe:
            if obs.right_clearance_m >= safe:
                action = cw
            else:
                action = "STOP"
        elif action == cw and obs.right_clearance_m < safe:
            if obs.left_clearance_m >= safe:
                action = ccw
            else:
                action = "STOP"
        return action

    def _pick_unstick_lateral_key(
        self,
        dp: DataProcessor,
        guard: ObstacleGuard,
        side_sign: int,
        dx_px: float | None = None,
        use_lidar: bool = True,
    ) -> str | None:
        # 有 YOLO 偏差時：往熊的方向微調，不要為閃 LiDAR 往反方向平移
        if dx_px is not None and abs(dx_px) > 25.0:
            return "RIGHT_SHIFT" if dx_px > 0.0 else "LEFT_SHIFT"

        if not use_lidar:
            return None

        obs = self._evaluate_obstacles(dp, guard, approach_mode=True)
        left = obs.left_clearance_m
        right = obs.right_clearance_m
        safe_side = guard.side_stop_m + 0.08

        if right + 0.06 < left and left >= safe_side:
            return "LEFT_SHIFT"
        if left + 0.06 < right and right >= safe_side:
            return "RIGHT_SHIFT"
        return None

    def _log_obstacle_if_due(
        self,
        obs,
        last_log_time: float,
        log_interval: float,
        prefix: str = "obstacle",
        dp: DataProcessor | None = None,
        source_debug_enabled: bool = False,
    ) -> float:
        now = time.monotonic()
        if now - last_log_time < log_interval:
            return last_log_time
        raw_f = obs.sensor_front_m
        raw_str = f"{raw_f:.2f}" if math.isfinite(raw_f) else "n/a"
        self.get_logger().info(
            f"[{prefix}] front={obs.front_clearance_m:.2f}m "
            f"(raw={raw_str}m) "
            f"left={obs.left_clearance_m:.2f}m right={obs.right_clearance_m:.2f}m "
            f"rear={obs.rear_clearance_m:.2f}m "
            f"min={obs.min_clearance_m:.2f}m scale={obs.speed_scale:.2f} "
            f"bwd={obs.backward_speed_scale:.2f} "
            f"block={obs.block_cmd or 'none'}"
        )
        if source_debug_enabled and obs.source_debug is not None:
            sd = obs.source_debug
            src_line = sd.format_compact()
            if dp is not None:
                hits = dp.get_lidar_sector_closest_hits()
                left_hit = hits.get("left")
                if left_hit is not None:
                    span = left_hit.get("close_span_deg")
                    span_str = f" span={span:.1f}°" if span is not None else ""
                    robust = left_hit.get("robust_m")
                    if robust is not None:
                        src_line += (
                            f" | lidar_raw={left_hit['raw_min_m']:.2f}m"
                            f"@{left_hit['raw_min_angle_deg']:.0f}°"
                            f"(robust={robust:.2f}m"
                            f"{span_str} n={left_hit['hit_count']})"
                        )
                    else:
                        src_line += (
                            f" | lidar_raw={left_hit['raw_min_m']:.2f}m"
                            f"@{left_hit['raw_min_angle_deg']:.0f}°"
                            f"{span_str} n={left_hit['hit_count']}"
                        )
                if sd.depth_front_left is not None and sd.depth_left is not None:
                    if (
                        sd.depth_front_left <= sd.depth_left + 1e-4
                        and sd.depth_left_combined is not None
                    ):
                        src_line += " | depth_patch=front_left(畫面左中上帶)"
                    else:
                        src_line += " | depth_patch=left(畫面最左欄上帶)"
            self.get_logger().info(f"[{prefix}/src] {src_line}")
        return now

    def _publish_backward_if_clear(
        self,
        dp: DataProcessor,
        guard: ObstacleGuard,
        enabled: bool,
        approach_target_depth_m: float | None = None,
        slow: bool = True,
        approach_mode: bool = True,
    ) -> bool:
        """Publish BACKWARD only if rear clearance allows; return False if blocked."""
        cmd = "BACKWARD_SLOW" if slow else "BACKWARD"
        if not enabled:
            self.publish_car_control(cmd)
            return True
        obs = self._evaluate_obstacles(
            dp,
            guard,
            approach_target_depth_m=approach_target_depth_m,
            approach_mode=approach_mode,
        )
        if obs.backward_speed_scale <= 0.05:
            raw_r = obs.sensor_rear_m
            raw_str = f"{raw_r:.2f}" if math.isfinite(raw_r) else "n/a"
            self.get_logger().warn(
                f"[rear/obstacle] Backward blocked — rear clearance "
                f"{obs.rear_clearance_m:.2f}m (raw={raw_str}m)"
            )
            self.publish_car_control("STOP")
            return False
        self.publish_car_control(cmd)
        return True

    def _apply_obstacle_motion(
        self,
        dp: DataProcessor,
        guard: ObstacleGuard,
        enabled: bool,
        log_interval: float,
        last_log_time: float,
        wheel_cmd: list[float] | None = None,
        search_forward_speed: float = 90.0,
        prefix: str = "obstacle",
        approach_target_depth_m: float | None = None,
        prefer_visual_yaw: bool = False,
        approach_mode: bool = False,
        speed_scale_floor: float | None = None,
        source_debug_enabled: bool = False,
    ) -> float:
        if not rclpy.ok():
            return last_log_time
        if not enabled:
            if wheel_cmd is not None:
                self.publish_raw_car_control(wheel_cmd)
            else:
                self.publish_raw_car_control(
                    [search_forward_speed] * 4
                )
            return last_log_time

        obs = self._evaluate_obstacles(
            dp,
            guard,
            approach_target_depth_m=approach_target_depth_m,
            approach_mode=approach_mode,
            speed_scale_floor=speed_scale_floor,
        )
        last_log_time = self._log_obstacle_if_due(
            obs,
            last_log_time,
            log_interval,
            prefix=prefix,
            dp=dp,
            source_debug_enabled=source_debug_enabled,
        )

        if wheel_cmd is not None:
            allow_visual_yaw = prefer_visual_yaw and not guard.must_override_visual_yaw(
                obs
            )
            scaled = guard.apply_to_wheel_cmd(
                wheel_cmd,
                obs,
                approach_mode=approach_mode,
                prefer_visual_yaw=allow_visual_yaw,
            )
            self.publish_raw_car_control(scaled)
            return last_log_time

        if obs.block_cmd and obs.speed_scale <= 0.05:
            if rclpy.ok():
                self.publish_car_control(obs.block_cmd)
            return last_log_time

        fwd = search_forward_speed * obs.speed_scale
        if fwd < 8.0:
            self.publish_car_control("STOP")
        else:
            self.publish_raw_car_control([fwd, fwd, fwd, fwd])
        return last_log_time

    def _run_mission(self):
        dp = DataProcessor(self)
        nav = Nav2Processing(self, dp)
        arm = ArmController(self, dp)

        use_unity = (
            self.get_parameter("use_unity_camera_nav").get_parameter_value().bool_value
        )
        obstacle_guard_enabled = (
            self.get_parameter("obstacle_guard_enabled").get_parameter_value().bool_value
        )
        obstacle_guard = self._build_obstacle_guard(use_unity)
        nav_obstacle_guard_enabled = (
            self.get_parameter("nav_obstacle_guard_enabled")
            .get_parameter_value()
            .bool_value
        )
        nav_obstacle_stop_m = (
            self.get_parameter("nav_obstacle_stop_m").get_parameter_value().double_value
        )
        if nav_obstacle_stop_m <= 0.0:
            nav_obstacle_stop_m = obstacle_guard.stop_m
        obstacle_log_interval = max(
            0.5,
            self.get_parameter("obstacle_log_interval_sec")
            .get_parameter_value()
            .double_value,
        )
        obstacle_source_debug_enabled = (
            self.get_parameter("obstacle_source_debug_enabled")
            .get_parameter_value()
            .bool_value
        )
        last_obstacle_log = 0.0
        approach_dt = (
            self.get_parameter("approach_period_sec").get_parameter_value().double_value
        )
        approach_dt = max(0.05, approach_dt)
        t_approach_max = (
            self.get_parameter("approach_timeout_sec").get_parameter_value().double_value
        )
        visual_servo_enabled = (
            self.get_parameter("visual_servo_enabled").get_parameter_value().bool_value
        )
        visual_servo_target_depth_m = (
            self.get_parameter("visual_servo_target_depth_m")
            .get_parameter_value()
            .double_value
        )
        visual_servo_search_spin_speed = (
            self.get_parameter("visual_servo_search_spin_speed")
            .get_parameter_value()
            .double_value
        )
        visual_servo_max_forward_speed = (
            self.get_parameter("visual_servo_max_forward_speed")
            .get_parameter_value()
            .double_value
        )
        visual_servo_max_forward_speed_far = (
            self.get_parameter("visual_servo_max_forward_speed_far")
            .get_parameter_value()
            .double_value
        )
        visual_servo_far_distance_m = (
            self.get_parameter("visual_servo_far_distance_m")
            .get_parameter_value()
            .double_value
        )
        visual_servo_yaw_deadband_px = (
            self.get_parameter("visual_servo_yaw_deadband_px")
            .get_parameter_value()
            .double_value
        )
        visual_servo_yaw_soft_scale_px = (
            self.get_parameter("visual_servo_yaw_soft_scale_px")
            .get_parameter_value()
            .double_value
        )
        visual_servo_max_yaw_near = (
            self.get_parameter("visual_servo_max_yaw_near")
            .get_parameter_value()
            .double_value
        )
        visual_servo_max_yaw_far = (
            self.get_parameter("visual_servo_max_yaw_far")
            .get_parameter_value()
            .double_value
        )
        visual_servo_min_yaw_large_px = (
            self.get_parameter("visual_servo_min_yaw_large_px")
            .get_parameter_value()
            .double_value
        )
        approach_turn_stuck_time_sec = (
            self.get_parameter("approach_turn_stuck_time_sec")
            .get_parameter_value()
            .double_value
        )
        approach_stuck_time_sec = (
            self.get_parameter("approach_stuck_time_sec")
            .get_parameter_value()
            .double_value
        )
        approach_stuck_back_sec = (
            self.get_parameter("approach_stuck_back_sec")
            .get_parameter_value()
            .double_value
        )
        approach_stuck_back_sec = max(0.0, approach_stuck_back_sec)
        approach_stuck_shift_sec = (
            self.get_parameter("approach_stuck_shift_sec")
            .get_parameter_value()
            .double_value
        )
        approach_stuck_shift_sec = max(0.0, approach_stuck_shift_sec)
        approach_stuck_forward_sec = (
            self.get_parameter("approach_stuck_forward_sec")
            .get_parameter_value()
            .double_value
        )
        approach_stuck_forward_sec = max(0.0, approach_stuck_forward_sec)
        approach_stuck_time_sec = max(2.0, approach_stuck_time_sec)
        motion_stuck_min_progress_m = (
            self.get_parameter("motion_stuck_min_progress_m")
            .get_parameter_value()
            .double_value
        )
        motion_stuck_min_progress_m = max(0.03, motion_stuck_min_progress_m)
        visual_servo_lost_timeout_sec = (
            self.get_parameter("visual_servo_lost_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        approach_yolo_lost_grace_sec = (
            self.get_parameter("approach_yolo_lost_grace_sec")
            .get_parameter_value()
            .double_value
        )
        approach_yolo_lost_grace_sec = max(0.3, approach_yolo_lost_grace_sec)
        approach_yolo_lost_min_frames = int(
            self.get_parameter("approach_yolo_lost_min_frames")
            .get_parameter_value()
            .integer_value
        )
        approach_yolo_lost_min_frames = max(3, approach_yolo_lost_min_frames)
        approach_yolo_search_spin_speed_tier = (
            self.get_parameter("approach_yolo_search_spin_speed_tier")
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        if approach_yolo_search_spin_speed_tier not in (
            "slow",
            "median",
            "fast",
            "full",
        ):
            approach_yolo_search_spin_speed_tier = "slow"
        approach_yolo_lost_search_circle_rad = (
            self.get_parameter("approach_yolo_lost_search_circle_rad")
            .get_parameter_value()
            .double_value
        )
        approach_yolo_lost_search_circle_rad = max(
            math.pi, approach_yolo_lost_search_circle_rad
        )
        approach_yolo_explore_forward_sec = (
            self.get_parameter("approach_yolo_explore_forward_sec")
            .get_parameter_value()
            .double_value
        )
        approach_yolo_explore_forward_sec = max(
            0.5, approach_yolo_explore_forward_sec
        )
        approach_yolo_explore_forward_speed = (
            self.get_parameter("approach_yolo_explore_forward_speed")
            .get_parameter_value()
            .double_value
        )
        approach_yolo_explore_forward_speed = max(
            40.0, approach_yolo_explore_forward_speed
        )
        approach_yolo_lost_motion_log_sec = (
            self.get_parameter("approach_yolo_lost_motion_log_sec")
            .get_parameter_value()
            .double_value
        )
        approach_yolo_lost_motion_log_sec = max(
            0.5, approach_yolo_lost_motion_log_sec
        )
        align_px = (
            self.get_parameter("align_pixel_thresh").get_parameter_value().double_value
        )
        align_bias_px = (
            self.get_parameter("align_pixel_bias_px").get_parameter_value().double_value
        )
        align_stable_frames = int(
            self.get_parameter("align_stable_frames").get_parameter_value().integer_value
        )
        align_stable_frames = max(1, align_stable_frames)
        vs_center_deadband_px = (
            self.get_parameter("visual_servo_center_deadband_px")
            .get_parameter_value()
            .double_value
        )
        vs_image_half_width_px = (
            self.get_parameter("visual_servo_image_half_width_px")
            .get_parameter_value()
            .double_value
        )
        dmax = (
            self.get_parameter("grasp_depth_max_m").get_parameter_value().double_value
        )
        dmin = (
            self.get_parameter("grasp_depth_min_m").get_parameter_value().double_value
        )
        approach_slow_dist_m = (
            self.get_parameter("approach_slow_dist_m").get_parameter_value().double_value
        )
        grasp_trigger_dist_m = (
            self.get_parameter("grasp_trigger_dist_m").get_parameter_value().double_value
        )
        grasp_trigger_lost_frames = int(
            self.get_parameter("grasp_trigger_lost_frames")
            .get_parameter_value()
            .integer_value
        )
        grasp_trigger_lost_frames = max(1, grasp_trigger_lost_frames)
        grasp_depth_jump_m = (
            self.get_parameter("grasp_depth_jump_m").get_parameter_value().double_value
        )
        grasp_depth_jump_m = max(0.15, grasp_depth_jump_m)
        approach_stop_dist_m = (
            self.get_parameter("approach_stop_dist_m").get_parameter_value().double_value
        )
        approach_decel_mps2 = (
            self.get_parameter("approach_decel_mps2").get_parameter_value().double_value
        )
        approach_decel_mps2 = max(0.08, approach_decel_mps2)
        approach_max_speed_mps = (
            self.get_parameter("approach_max_speed_mps").get_parameter_value().double_value
        )
        approach_max_speed_mps = max(0.1, approach_max_speed_mps)
        approach_brake_safety_m = (
            self.get_parameter("approach_brake_safety_m").get_parameter_value().double_value
        )
        approach_brake_safety_m = max(0.0, approach_brake_safety_m)
        marker_wait = (
            self.get_parameter("marker_wait_sec").get_parameter_value().double_value
        )
        nav_timeout = (
            self.get_parameter("nav_home_timeout_sec").get_parameter_value().double_value
        )
        nav_retry_count = int(
            self.get_parameter("nav_retry_count").get_parameter_value().integer_value
        )
        nav_retry_count = max(0, nav_retry_count)
        nav_feedback_log_sec = (
            self.get_parameter("nav_feedback_log_sec").get_parameter_value().double_value
        )
        nav_feedback_log_sec = max(0.2, nav_feedback_log_sec)
        nav_stuck_time_sec = (
            self.get_parameter("nav_stuck_time_sec").get_parameter_value().double_value
        )
        nav_stuck_time_sec = max(2.0, nav_stuck_time_sec)
        nav_stuck_min_progress_m = (
            self.get_parameter("nav_stuck_min_progress_m").get_parameter_value().double_value
        )
        nav_stuck_min_progress_m = max(0.01, nav_stuck_min_progress_m)
        self._nav_stuck_min_progress_m = nav_stuck_min_progress_m
        nav_unstick_enabled = (
            self.get_parameter("nav_unstick_enabled").get_parameter_value().bool_value
        )
        nav_unstick_back_sec = (
            self.get_parameter("nav_unstick_back_sec").get_parameter_value().double_value
        )
        nav_unstick_back_sec = max(0.0, nav_unstick_back_sec)
        nav_unstick_rotate_sec = (
            self.get_parameter("nav_unstick_rotate_sec").get_parameter_value().double_value
        )
        nav_unstick_rotate_sec = max(0.0, nav_unstick_rotate_sec)
        fallback_home_enabled = (
            self.get_parameter("fallback_home_enabled").get_parameter_value().bool_value
        )
        fallback_home_timeout_sec = (
            self.get_parameter("fallback_home_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        fallback_home_timeout_sec = max(3.0, fallback_home_timeout_sec)
        fallback_arrival_xy_thresh_m = (
            self.get_parameter("fallback_arrival_xy_thresh_m")
            .get_parameter_value()
            .double_value
        )
        fallback_arrival_xy_thresh_m = max(0.08, fallback_arrival_xy_thresh_m)
        fallback_heading_thresh_deg = (
            self.get_parameter("fallback_heading_thresh_deg")
            .get_parameter_value()
            .double_value
        )
        fallback_heading_thresh_deg = max(3.0, fallback_heading_thresh_deg)
        fallback_rotate_step_sec = (
            self.get_parameter("fallback_rotate_step_sec")
            .get_parameter_value()
            .double_value
        )
        fallback_rotate_step_sec = min(max(0.06, fallback_rotate_step_sec), 0.35)
        fallback_forward_step_sec = (
            self.get_parameter("fallback_forward_step_sec")
            .get_parameter_value()
            .double_value
        )
        fallback_forward_step_sec = min(max(0.06, fallback_forward_step_sec), 0.40)
        home_reached_thresh = (
            self.get_parameter("home_reached_dist_thresh_m")
            .get_parameter_value()
            .double_value
        )
        drop_at_home = (
            self.get_parameter("drop_at_home").get_parameter_value().bool_value
        )
        drop_before_final_heading = (
            self.get_parameter("drop_before_final_heading")
            .get_parameter_value()
            .bool_value
        )
        amcl_wait = (
            self.get_parameter("amcl_wait_timeout_sec").get_parameter_value().double_value
        )
        amcl_wait = max(5.0, amcl_wait)

        amcl_topic = (
            self.get_parameter("amcl_pose_topic").get_parameter_value().string_value
        )

        if obstacle_guard_enabled or nav_obstacle_guard_enabled:
            self.get_logger().info(
                f"ObstacleGuard profile={'unity' if use_unity else 'real'}: "
                f"stop={obstacle_guard.stop_m:.2f}m slow={obstacle_guard.slow_m:.2f}m "
                f"scale_floor far/mid/slow="
                f"{self.get_parameter('obstacle_scale_floor_far').get_parameter_value().double_value:.2f}/"
                f"{self.get_parameter('obstacle_scale_floor_mid').get_parameter_value().double_value:.2f}/"
                f"{self.get_parameter('obstacle_scale_floor_slow').get_parameter_value().double_value:.2f} "
                f"approach_guard={obstacle_guard_enabled} nav_guard={nav_obstacle_guard_enabled} "
                f"nav_stop={nav_obstacle_stop_m:.2f}m"
            )

        self.get_logger().info(
            f"Recording home pose from '{amcl_topic}' (waiting up to {amcl_wait:.0f}s) …"
        )
        home_pose = None
        t0 = time.monotonic()
        last_log = t0
        while time.monotonic() - t0 < amcl_wait and rclpy.ok():
            pose_msg = self.get_latest_amcl_pose()
            if pose_msg is not None:
                home_pose = copy.deepcopy(pose_msg.pose.pose)
                break
            now = time.monotonic()
            if now - last_log >= 5.0:
                self.get_logger().warn(
                    f"Still waiting for '{amcl_topic}' — 請確認 pros_app localization 已啟動，"
                    "且 car_control 容器與 localization **同一 Docker bridge**（ROS_BRIDGE_NETWORK），"
                    "ROS_DOMAIN_ID 一致。"
                )
                last_log = now
            time.sleep(0.05)

        if home_pose is None:
            self.get_logger().error(
                f"收不到 '{amcl_topic}'，無法記錄 home。"
                "請在同一 Docker bridge 跑 localization，並設 ROS_BRIDGE_NETWORK（見 car_control.sh）；"
                "或改用 -p amcl_pose_topic:=正確話題。"
            )
            return

        unity_stow_elbow_enabled = (
            self.get_parameter("unity_stow_elbow_enabled")
            .get_parameter_value()
            .bool_value
        )
        unity_stow_elbow_deg = (
            self.get_parameter("unity_stow_elbow_deg").get_parameter_value().double_value
        )
        if use_unity and unity_stow_elbow_enabled:
            self.get_logger().info(
                f"Unity arm stow: lowering elbow to {unity_stow_elbow_deg:.0f}° "
                "so gripper won't block camera …"
            )
            arm.run_unity_vision_stow_blocking(unity_stow_elbow_deg)

        # 距離分區定義（公尺）
        #  dist > zone_far   : 全速直衝（300），方向只做大角度修正
        #  zone_mid < dist   : 中速（160），精準轉向對準
        #  zone_slow < dist  : 慢速（80），最後微調
        #  dist <= grasp_zone: 主動煞車後停下 → 夾取
        zone_far  = 1.50
        zone_mid  = 0.80
        zone_slow = grasp_trigger_dist_m  # 預設 0.65

        self.get_logger().info(
            "Approaching bear with braking lookahead "
            f"(stop={approach_stop_dist_m:.2f}m, decel={approach_decel_mps2:.2f}m/s², "
            f"grasp_zone={zone_slow:.2f}m) …"
        )

        t_start = time.monotonic()
        aligned = False
        last_valid_dist = None
        lost_frames = 0
        last_approach_log = 0.0
        last_obstacle_log = 0.0
        prev_dist_sample = None
        prev_dist_sample_time = None
        closure_speed_mps = 0.0
        last_turn_dx = None
        last_turn_progress_time = time.monotonic()
        turn_stuck_boost = 1.0
        last_approach_progress_time = time.monotonic()
        last_progress_dist = None
        last_progress_dx = None
        last_approach_unstick_time = 0.0
        last_yolo_lost_motion_log = 0.0
        last_yolo_hold_log = 0.0
        last_target_valid_time = 0.0
        held_ti = None
        approach_unstick_side = 1
        motion_progress = {"last_progress_time": time.monotonic(), "last_pose_xy": None}
        yolo_search_state = {
            "phase": "idle",
            "accumulated_rad": 0.0,
            "last_yaw": None,
            "explore_until": 0.0,
            "logged_spin": False,
        }
        # 鎖定第一幀目標的方向，旋轉中不因 YOLO 抖動換目標
        locked_dx = None       # 鎖住的水平偏差（像素）
        locked_dx_time = None  # 鎖住的時間
        prev_approach_dist = None
        nav.reset_visual_servo()

        while time.monotonic() - t_start < t_approach_max and rclpy.ok():
            ti = dp.get_yolo_target_info()
            target_live = self._target_live(ti)

            # 深度突然跳遠（YOLO 換成下一隻熊 / 太近深度失效後重選）
            if target_live:
                live_dist = float(ti[1])
                live_dx = float(ti[2])
            else:
                live_dist = -1.0
                live_dx = 0.0

            if (
                target_live
                and prev_approach_dist is not None
                and prev_approach_dist <= zone_slow * 1.35
                and (live_dist - prev_approach_dist) >= grasp_depth_jump_m
            ):
                aligned = True
                self.get_logger().warn(
                    f"[approach] Depth jumped away ({prev_approach_dist:.2f}→{live_dist:.2f}m) "
                    f"(>{grasp_depth_jump_m:.2f}m) — likely too close / YOLO re-target "
                    "→ brake → GRASP"
                )
                self._publish_backward_if_clear(
                    dp,
                    obstacle_guard,
                    obstacle_guard_enabled,
                    approach_target_depth_m=live_dist,
                    slow=True,
                )
                time.sleep(0.22)
                break

            if target_live:
                held_ti = list(ti)
                last_target_valid_time = time.monotonic()
                locked_dx = live_dx
                locked_dx_time = time.monotonic()
                lost_frames = 0
                if yolo_search_state["phase"] != "idle":
                    self.get_logger().info(
                        "[approach/yolo_lost] target reacquired → resume approach"
                    )
                self._reset_yolo_search_state(yolo_search_state)
            else:
                lost_frames += 1

            since_valid = (
                time.monotonic() - last_target_valid_time
                if last_target_valid_time > 0.0
                else float("inf")
            )
            in_hold = (
                held_ti is not None
                and since_valid < approach_yolo_lost_grace_sec
                and lost_frames < approach_yolo_lost_min_frames
            )
            use_approach = target_live or in_hold
            yolo_search_confirmed = not use_approach and (
                held_ti is not None or last_valid_dist is not None
            )

            if in_hold:
                now_hold_log = time.monotonic()
                if now_hold_log - last_yolo_hold_log >= 1.0:
                    hold_dist = (
                        float(held_ti[1])
                        if held_ti is not None and float(held_ti[1]) > 0.0
                        else last_valid_dist
                    )
                    self.get_logger().info(
                        "[approach/yolo_hold] brief dropout "
                        f"({lost_frames} frames, {since_valid:.2f}s) "
                        f"→ keep approaching last target "
                        f"(dist={hold_dist if hold_dist is not None else 'n/a'}m)"
                    )
                    last_yolo_hold_log = now_hold_log

            if yolo_search_confirmed and yolo_search_state["phase"] == "idle":
                yolo_search_state["phase"] = "spin"
                yolo_search_state["accumulated_rad"] = 0.0
                yolo_search_state["last_yaw"] = None
                if not yolo_search_state["logged_spin"]:
                    ccw_action, _ = self._spin_action_names(
                        approach_yolo_search_spin_speed_tier
                    )
                    self.get_logger().info(
                        "[approach/yolo_lost] target lost (confirmed) → in-place search "
                        f"(1 circle={math.degrees(approach_yolo_lost_search_circle_rad):.0f}° "
                        f"spin={ccw_action}, "
                        f"then explore {approach_yolo_explore_forward_sec:.1f}s forward)"
                    )
                    yolo_search_state["logged_spin"] = True

            approach_ti = ti if target_live else held_ti
            if use_approach and approach_ti is not None and len(approach_ti) >= 3:
                detected = float(approach_ti[0]) == 1.0
                dist = (
                    float(approach_ti[1])
                    if detected and float(approach_ti[1]) > 0.0
                    else (last_valid_dist if last_valid_dist is not None else -1.0)
                )
                raw_dx_fresh = float(approach_ti[2])
            else:
                detected = False
                dist = -1.0
                raw_dx_fresh = 0.0

            # 轉向用「當前幀」像素偏差（置中控制），鎖定只用於目標選擇
            raw_dx = raw_dx_fresh if use_approach else (locked_dx or 0.0)

            # 轉向卡住偵測：dx 長時間沒改善 → 暫時加大轉向力
            if use_approach and dist > 0.0:
                cur_turn_err = abs(raw_dx_fresh - align_bias_px)
                if (
                    last_turn_dx is None
                    or (last_turn_dx - cur_turn_err) >= 12.0
                ):
                    last_turn_progress_time = time.monotonic()
                    turn_stuck_boost = 1.0
                last_turn_dx = cur_turn_err
                if (
                    cur_turn_err > align_px
                    and time.monotonic() - last_turn_progress_time
                    > approach_turn_stuck_time_sec
                ):
                    turn_stuck_boost = 1.45

            # 記錄最後有效距離 + 估算接近速度（深度變化率）
            if target_live:
                now_sample = time.monotonic()
                if prev_dist_sample is not None and prev_dist_sample_time is not None:
                    dt_s = now_sample - prev_dist_sample_time
                    if dt_s > 0.02:
                        instant_v = max(
                            0.0, (prev_dist_sample - live_dist) / dt_s
                        )
                        closure_speed_mps = 0.65 * closure_speed_mps + 0.35 * instant_v
                prev_dist_sample = live_dist
                prev_dist_sample_time = now_sample
                last_valid_dist = live_dist
                lost_frames = 0

                # 接近進展偵測（距離縮短 / 置中改善 / 有接近速度）
                progressed = False
                cur_dx_err = abs(live_dx - align_bias_px)
                if last_progress_dist is not None and (last_progress_dist - live_dist) >= 0.04:
                    progressed = True
                if last_progress_dx is not None and (last_progress_dx - cur_dx_err) >= 12.0:
                    progressed = True
                if closure_speed_mps > 0.07:
                    progressed = True
                if progressed:
                    last_approach_progress_time = time.monotonic()
                    motion_progress["last_progress_time"] = time.monotonic()
                last_progress_dist = live_dist
                last_progress_dx = cur_dx_err
                prev_approach_dist = live_dist

            amcl_pose = self.get_latest_amcl_pose()
            progress_yaw_rad = 0.10 if yolo_search_confirmed else None
            self._update_motion_progress(
                motion_progress,
                amcl_pose,
                motion_stuck_min_progress_m,
                min_progress_yaw_rad=progress_yaw_rad,
            )
            if yolo_search_confirmed and yolo_search_state["phase"] == "spin":
                search_yaw_rad = self._track_yolo_search_yaw(
                    yolo_search_state, amcl_pose
                )
                if search_yaw_rad >= approach_yolo_lost_search_circle_rad:
                    yolo_search_state["phase"] = "explore"
                    yolo_search_state["explore_until"] = (
                        time.monotonic() + approach_yolo_explore_forward_sec
                    )
                    yolo_search_state["accumulated_rad"] = 0.0
                    yolo_search_state["last_yaw"] = None
                    self.get_logger().info(
                        "[approach/yolo_lost] full circle without target "
                        f"({math.degrees(search_yaw_rad):.0f}°) → explore forward "
                        f"{approach_yolo_explore_forward_sec:.1f}s"
                    )
            elif (
                yolo_search_confirmed
                and yolo_search_state["phase"] == "explore"
                and time.monotonic() >= yolo_search_state["explore_until"]
            ):
                yolo_search_state["phase"] = "spin"
                yolo_search_state["accumulated_rad"] = 0.0
                yolo_search_state["last_yaw"] = None
                self.get_logger().info(
                    "[approach/yolo_lost] explore finished, still no target "
                    "→ search spin again"
                )

            d_eff = last_valid_dist if last_valid_dist is not None else 9.9
            obs_scale_floor = self._obstacle_speed_scale_floor(
                d_eff, zone_far, zone_mid
            )

            obs_for_stuck = (
                self._evaluate_obstacles(
                    dp,
                    obstacle_guard,
                    approach_target_depth_m=last_valid_dist,
                    approach_mode=True,
                    speed_scale_floor=obs_scale_floor,
                )
                if obstacle_guard_enabled
                else None
            )
            wall_stuck = False
            if obs_for_stuck is not None:
                sf = obs_for_stuck.sensor_front_m
                bear_ahead = (
                    target_live
                    and live_dist > 0.0
                    and math.isfinite(sf)
                    and live_dist > sf + 0.25
                )
                wall_stuck = (
                    math.isfinite(sf)
                    and sf < obstacle_guard.stop_m + 0.08
                    and not bear_ahead
                ) or (
                    min(
                        obs_for_stuck.left_clearance_m,
                        obs_for_stuck.right_clearance_m,
                    )
                    < obstacle_guard.side_stop_m + 0.06
                    and not bear_ahead
                )
            skip_unstick = (
                use_approach
                and dist > 0.0
                and dist <= zone_mid
                and not wall_stuck
            ) or (yolo_search_confirmed and not wall_stuck)

            # ── 通用脫困：AMCL/接近無進展 > N 秒（YOLO 丟失或貼牆也觸發）──
            if (
                not skip_unstick
                and self._motion_is_stuck(motion_progress, approach_stuck_time_sec)
                and time.monotonic() - last_approach_unstick_time
                > approach_stuck_time_sec + 0.4
            ):
                self._motion_unstick_maneuver(
                    dp=dp,
                    guard=obstacle_guard,
                    back_sec=approach_stuck_back_sec,
                    shift_sec=approach_stuck_shift_sec,
                    forward_sec=approach_stuck_forward_sec,
                    side_sign=approach_unstick_side,
                    yolo_lost=yolo_search_confirmed,
                    search_spin_speed=visual_servo_search_spin_speed,
                    dx_px=raw_dx_fresh if use_approach else locked_dx,
                    use_lidar_for_unstick=obstacle_guard_enabled,
                    approach_target_depth_m=(
                        live_dist if target_live else last_valid_dist
                    ),
                )
                last_approach_unstick_time = time.monotonic()
                self._reset_motion_progress(motion_progress)
                last_approach_progress_time = time.monotonic()
                last_turn_progress_time = time.monotonic()
                turn_stuck_boost = 1.0
                approach_unstick_side *= -1
                nav.reset_visual_servo()
                prev_dist_sample = None
                prev_dist_sample_time = None
                closure_speed_mps = 0.0
                continue

            # ── 觸發：深度失效／丟失但已很近（避免衝過頭才看到 -1）──
            if (
                last_valid_dist is not None
                and last_valid_dist <= zone_slow * 1.15
                and (
                    not target_live
                    or live_dist <= 0.0
                )
                and lost_frames >= 1
            ):
                aligned = True
                self.get_logger().info(
                    f"[approach] Target lost/invalid near grasp "
                    f"(last_dist={last_valid_dist:.2f}m, lost={lost_frames}) → brake → GRASP"
                )
                self._publish_backward_if_clear(
                    dp,
                    obstacle_guard,
                    obstacle_guard_enabled,
                    approach_target_depth_m=(
                        live_dist if target_live else last_valid_dist
                    ),
                    slow=True,
                )
                time.sleep(0.22)
                break

            # ── 觸發：熊靠太近消失（最後距離 ≤ zone_slow*1.4，連失 N 幀）──
            if (
                not target_live
                and last_valid_dist is not None
                and last_valid_dist <= zone_slow * 1.4
                and lost_frames >= grasp_trigger_lost_frames
            ):
                aligned = True
                self.get_logger().info(
                    f"[approach] Bear out of view at close range "
                    f"(last_dist={last_valid_dist:.2f}m) → brake → GRASP"
                )
                break

            # ── 觸發：進入 grasp zone，且畫面已置中 → 主動煞車 ──
            grasp_center_thresh = align_px * 1.5   # 夾取前 dx 門檻（比接近門檻寬一點）
            if use_approach and dist > 0.0 and dist <= zone_slow + 0.12:
                if abs(raw_dx) <= grasp_center_thresh:
                    self.get_logger().info(
                        f"[approach] Grasp zone reached & centered: "
                        f"dist={dist:.2f}m, dx={raw_dx:.0f}px → brake → GRASP"
                    )
                    brake_sec = 0.18 if last_valid_dist is not None and last_valid_dist < zone_mid else 0.10
                    self._publish_backward_if_clear(
                        dp,
                        obstacle_guard,
                        obstacle_guard_enabled,
                        approach_target_depth_m=dist,
                        slow=True,
                    )
                    time.sleep(brake_sec)
                    aligned = True
                    break
                else:
                    # 進了 grasp zone 但還沒對準：連續小幅度轉向
                    self.publish_car_control("STOP")
                    yaw_w = nav.compute_yaw_wheel_from_pixel(
                        raw_dx_fresh,
                        max_yaw_wheel=visual_servo_max_yaw_near * 0.92,
                        deadband_px=visual_servo_yaw_deadband_px,
                        soft_scale_px=visual_servo_yaw_soft_scale_px,
                        dt=approach_dt,
                    )
                    self.publish_raw_car_control(
                        [yaw_w, -yaw_w, yaw_w, -yaw_w]
                    )
                    time.sleep(approach_dt)
                    continue

            # ── log（加快到 0.35 秒一次）──
            now_t = time.monotonic()
            if use_approach and now_t - last_approach_log >= 0.35:
                d_eff = last_valid_dist if last_valid_dist is not None else dist
                if d_eff is not None:
                    margin = max(0.0, d_eff - approach_stop_dist_m)
                    brake_need = self._braking_distance_m(
                        closure_speed_mps,
                        approach_decel_mps2,
                        approach_brake_safety_m,
                    )
                    v_allow = self._max_allowable_speed_mps(
                        margin, approach_decel_mps2
                    )
                    zone_name = (
                        "FAR" if d_eff > zone_far
                        else "MID" if d_eff > zone_mid
                        else "SLOW"
                    )
                    self.get_logger().info(
                        f"[approach/{zone_name}] dist={d_eff:.2f}m, dx={raw_dx:.0f}px, "
                        f"v={closure_speed_mps:.2f}m/s, brake_need={brake_need:.2f}m, "
                        f"v_allow={v_allow:.2f}m/s"
                        + (
                            f", yaw_boost=x{turn_stuck_boost:.2f}"
                            if turn_stuck_boost > 1.01
                            else ""
                        )
                    )
                last_approach_log = now_t

            # ── 行進速度：煞車距離前瞻 + 連續視覺轉向 + 障礙護欄 ──
            d = last_valid_dist if last_valid_dist is not None else 9.9
            approach_depth_hint = (
                live_dist if target_live else last_valid_dist
            )
            if not use_approach:
                obs_search = self._evaluate_obstacles(
                    dp,
                    obstacle_guard,
                    approach_target_depth_m=approach_depth_hint,
                    approach_mode=True,
                    speed_scale_floor=obs_scale_floor,
                )
                last_obstacle_log = self._log_obstacle_if_due(
                    obs_search,
                    last_obstacle_log,
                    obstacle_log_interval,
                    prefix="approach/obstacle",
                    dp=dp,
                    source_debug_enabled=obstacle_source_debug_enabled,
                )
                if yolo_search_state["phase"] == "explore":
                    explore_cmd = [
                        approach_yolo_explore_forward_speed
                    ] * 4
                    if obstacle_guard_enabled:
                        scaled = obstacle_guard.apply_to_wheel_cmd(
                            explore_cmd,
                            obs_search,
                            approach_mode=True,
                            prefer_visual_yaw=False,
                        )
                        self.publish_raw_car_control(scaled)
                    else:
                        self.publish_car_control("FORWARD_SLOW")
                else:
                    spin_action = self._pick_yolo_lost_spin_action(
                        obstacle_guard,
                        obs_search,
                        locked_dx,
                        spin_tier=approach_yolo_search_spin_speed_tier,
                    )
                    if spin_action == "STOP":
                        self.publish_car_control("STOP")
                    else:
                        self.publish_car_control(spin_action)
                now_lost_log = time.monotonic()
                if now_lost_log - last_yolo_lost_motion_log >= approach_yolo_lost_motion_log_sec:
                    acc_deg = math.degrees(
                        float(yolo_search_state.get("accumulated_rad", 0.0))
                    )
                    motion_label = (
                        "explore_forward"
                        if yolo_search_state["phase"] == "explore"
                        else spin_action
                    )
                    self.get_logger().info(
                        "[approach/yolo_lost] phase="
                        f"{yolo_search_state['phase']} action={motion_label} "
                        f"search_yaw={acc_deg:.0f}° "
                        f"scale={obs_search.speed_scale:.2f} "
                        f"block={obs_search.block_cmd or 'none'}"
                    )
                    last_yolo_lost_motion_log = now_lost_log
            else:
                margin = max(0.0, d - approach_stop_dist_m)
                fwd_wheel = self._lookahead_forward_wheel_speed(
                    dist_m=d,
                    closure_mps=closure_speed_mps,
                    stop_m=approach_stop_dist_m,
                    decel_mps2=approach_decel_mps2,
                    safety_m=approach_brake_safety_m,
                    max_wheel=visual_servo_max_forward_speed_far,
                    max_mps=approach_max_speed_mps,
                )
                # 已對準且進入 pre-grasp 區：再压低前進上限
                if (
                    use_approach
                    and dist > 0.0
                    and dist <= zone_slow + 0.45
                    and abs(raw_dx_fresh) <= align_px * 1.5
                ):
                    ramp = max(
                        0.10,
                        (dist - approach_stop_dist_m)
                        / max(1e-3, zone_slow + 0.40 - approach_stop_dist_m),
                    )
                    fwd_wheel *= min(1.0, ramp ** 1.8)
                center_first = (
                    margin < (zone_mid - approach_stop_dist_m)
                    or (
                        use_approach
                        and dist > 0.0
                        and dist <= 1.05
                        and abs(raw_dx_fresh) <= vs_center_deadband_px * 1.4
                    )
                )
                yaw_cap = (
                    visual_servo_max_yaw_near
                    if center_first
                    else visual_servo_max_yaw_far
                ) * turn_stuck_boost
                wheel_cmd = nav.camera_nav_pid_command(
                    target_depth_m=approach_stop_dist_m,
                    search_spin_speed=visual_servo_search_spin_speed,
                    max_forward_speed=fwd_wheel,
                    max_forward_speed_far=fwd_wheel,
                    far_distance_m=zone_far,
                    max_yaw_speed=min(yaw_cap, 380.0),
                    lost_timeout_sec=visual_servo_lost_timeout_sec,
                    center_deadband_px=(
                        vs_center_deadband_px if center_first else 60.0
                    ),
                    image_half_width_px=vs_image_half_width_px,
                    center_first=center_first,
                    yaw_deadband_px=visual_servo_yaw_deadband_px,
                    yaw_soft_scale_px=visual_servo_yaw_soft_scale_px,
                    min_yaw_large_px=visual_servo_min_yaw_large_px,
                    pixel_offset_bias_px=align_bias_px,
                    yolo_target_info=approach_ti,
                )
                last_obstacle_log = self._apply_obstacle_motion(
                    dp,
                    obstacle_guard,
                    obstacle_guard_enabled,
                    obstacle_log_interval,
                    last_obstacle_log,
                    wheel_cmd=wheel_cmd,
                    prefix="approach/obstacle",
                    approach_target_depth_m=approach_depth_hint,
                    prefer_visual_yaw=True,
                    approach_mode=True,
                    speed_scale_floor=obs_scale_floor,
                    source_debug_enabled=obstacle_source_debug_enabled,
                )

            if not rclpy.ok():
                break
            time.sleep(approach_dt)

        self.publish_car_control("STOP")
        time.sleep(0.4)  # 多等一點讓車體完全靜止

        if not aligned:
            self.get_logger().error(
                "Approach timeout or alignment failed — check YOLO target_class, depth, TF."
            )
            return

        self.get_logger().info("Waiting for /yolo/target_marker …")
        t_m = time.monotonic()
        while time.monotonic() - t_m < marker_wait and rclpy.ok():
            if self.latest_yolo_marker is not None:
                break
            time.sleep(0.05)

        ok = arm.run_grasp_blocking()
        if not ok:
            self.get_logger().error("Grasp failed.")
            return

        current_pose_msg = self.get_latest_amcl_pose()
        if current_pose_msg is not None:
            d_before = self._dist_xy(current_pose_msg.pose.pose, home_pose)
            self.get_logger().info(f"Navigating home … (current→home distance={d_before:.3f}m)")
            if d_before <= home_reached_thresh:
                self.get_logger().warn(
                    f"Current distance to home is already small ({d_before:.3f}m <= {home_reached_thresh:.3f}m). "
                    "Mission may appear to 'not move'."
                )
        else:
            d_before = None
            self.get_logger().info("Navigating home …")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = copy.deepcopy(home_pose)
        if drop_before_final_heading:
            cur_for_goal = self.get_latest_amcl_pose()
            if cur_for_goal is not None:
                # Nav2 只要求回到 home XY；最終面向稍後手動對齊
                goal_pose.pose.orientation = copy.deepcopy(
                    cur_for_goal.pose.pose.orientation
                )

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        client: ActionClient = self.navigate_to_pose_action_client
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return

        nav_succeeded = False
        status = GoalStatus.STATUS_UNKNOWN
        for attempt in range(nav_retry_count + 1):
            self.get_logger().info(
                f"[nav] Attempt {attempt + 1}/{nav_retry_count + 1}: sending NavigateToPose goal."
            )
            send_future = client.send_goal_async(
                nav_goal,
                feedback_callback=lambda fb, _self=self, _period=nav_feedback_log_sec: _self._nav_feedback_cb(
                    fb, _period
                ),
            )
            t_nav = time.monotonic()
            while not send_future.done() and time.monotonic() - t_nav < 30.0:
                time.sleep(0.02)

            if not send_future.done():
                self.get_logger().error("[nav] send_goal_async timed out.")
                status = GoalStatus.STATUS_UNKNOWN
            else:
                goal_handle = send_future.result()
                if not goal_handle.accepted:
                    self.get_logger().error("[nav] Goal rejected by NavigateToPose server.")
                    status = GoalStatus.STATUS_UNKNOWN
                else:
                    self._last_nav_feedback_distance = None
                    self._last_nav_feedback_time = time.monotonic()
                    self._last_nav_progress_time = time.monotonic()
                    self._last_home_dist = None
                    result_future = goal_handle.get_result_async()
                    t_nav = time.monotonic()
                    last_nav_obstacle_log = 0.0
                    last_nav_obstacle_check = 0.0
                    nav_aborted_by_guard = False
                    nav_xy_reached_early = False
                    while not result_future.done() and time.monotonic() - t_nav < nav_timeout:
                        now_nav = time.monotonic()
                        if (
                            nav_obstacle_guard_enabled
                            and now_nav - last_nav_obstacle_check >= 0.15
                        ):
                            last_nav_obstacle_check = now_nav
                            obs_nav = self._evaluate_obstacles(dp, obstacle_guard)
                            last_nav_obstacle_log = self._log_obstacle_if_due(
                                obs_nav,
                                last_nav_obstacle_log,
                                obstacle_log_interval,
                                prefix="nav/obstacle",
                                dp=dp,
                                source_debug_enabled=obstacle_source_debug_enabled,
                            )
                            nav_front_raw = obs_nav.sensor_front_m
                            if (
                                math.isfinite(nav_front_raw)
                                and nav_front_raw < nav_obstacle_stop_m
                            ):
                                self.get_logger().warn(
                                    f"[nav/obstacle] Front sensor "
                                    f"{nav_front_raw:.2f}m < "
                                    f"{nav_obstacle_stop_m:.2f}m — cancel Nav2 goal."
                                )
                                cancel_future = goal_handle.cancel_goal_async()
                                t_cancel = time.monotonic()
                                while (
                                    not cancel_future.done()
                                    and time.monotonic() - t_cancel < 2.0
                                ):
                                    time.sleep(0.02)
                                self.publish_car_control("STOP")
                                status = GoalStatus.STATUS_ABORTED
                                nav_aborted_by_guard = True
                                break

                        # 優先使用 AMCL 實際「到 home 距離」判斷是否有進展，避免 distance_remaining=0 誤判
                        amcl_pose_now = self.get_latest_amcl_pose()
                        if amcl_pose_now is not None:
                            d_home_now = self._dist_xy(amcl_pose_now.pose.pose, home_pose)
                            prev_home = self._last_home_dist
                            self._last_home_dist = d_home_now
                            if (
                                prev_home is None
                                or (prev_home - d_home_now) >= self._nav_stuck_min_progress_m
                            ):
                                self._last_nav_progress_time = time.monotonic()

                            if (
                                drop_before_final_heading
                                and d_home_now <= home_reached_thresh
                            ):
                                self.get_logger().info(
                                    "[nav] Home XY reached — stopping Nav2 before "
                                    "final heading (drop-first mode)."
                                )
                                cancel_future = goal_handle.cancel_goal_async()
                                t_cancel = time.monotonic()
                                while (
                                    not cancel_future.done()
                                    and time.monotonic() - t_cancel < 2.0
                                ):
                                    time.sleep(0.02)
                                self.publish_car_control("STOP")
                                nav_succeeded = True
                                nav_xy_reached_early = True
                                status = GoalStatus.STATUS_SUCCEEDED
                                break

                        # 若長時間幾乎無進展，主動取消本次導航，交給 retry 流程
                        if (
                            self._last_home_dist is not None
                            and self._last_home_dist > home_reached_thresh
                            and time.monotonic() - self._last_nav_progress_time
                            > nav_stuck_time_sec
                        ):
                            self.get_logger().warn(
                                "[nav] Stuck detected: AMCL distance-to-home has not improved "
                                f"for {nav_stuck_time_sec:.1f}s "
                                f"(current_home_dist={self._last_home_dist:.3f}m). "
                                "Cancel current goal and retry."
                            )
                            cancel_future = goal_handle.cancel_goal_async()
                            t_cancel = time.monotonic()
                            while (
                                not cancel_future.done()
                                and time.monotonic() - t_cancel < 2.0
                            ):
                                time.sleep(0.02)
                            status = GoalStatus.STATUS_ABORTED
                            break
                        time.sleep(0.05)

                    if nav_xy_reached_early:
                        self.get_logger().info(
                            "[nav] Treating early XY arrival as success (drop-first mode)."
                        )
                    elif result_future.done():
                        nav_result = result_future.result()
                        status = nav_result.status
                        status_text = self._GOAL_STATUS_TEXT.get(
                            status, f"UNKNOWN_CODE({status})"
                        )
                        nav_succeeded = status == GoalStatus.STATUS_SUCCEEDED
                        self.get_logger().info(
                            f"[nav] Attempt {attempt + 1}: status={status} [{status_text}] "
                            f"(success_code={GoalStatus.STATUS_SUCCEEDED})."
                        )
                    elif nav_aborted_by_guard:
                        nav_succeeded = False
                        self.get_logger().warn(
                            "[nav] Goal canceled by obstacle guard (front too close)."
                        )
                    else:
                        status = GoalStatus.STATUS_UNKNOWN
                        self.get_logger().warn("[nav] Result wait timed out.")

            if nav_succeeded:
                break

            if attempt < nav_retry_count:
                self.get_logger().warn(
                    "[nav] Attempt failed. Clearing costmaps and retrying once pose is updated."
                )
                self._clear_costmaps_once()
                if nav_unstick_enabled:
                    self._unstick_maneuver(
                        nav_unstick_back_sec,
                        nav_unstick_rotate_sec,
                        dp=dp,
                        guard=obstacle_guard,
                        obstacle_guard_enabled=nav_obstacle_guard_enabled,
                    )
                time.sleep(0.6)
            else:
                self.get_logger().warn("[nav] Retries exhausted.")

        current_pose_msg = self.get_latest_amcl_pose()
        d_after = (
            self._dist_xy(current_pose_msg.pose.pose, home_pose)
            if current_pose_msg is not None
            else None
        )
        nav_feedback_inconsistent = (
            self._last_nav_feedback_distance is not None
            and self._last_nav_feedback_distance <= 0.05
            and d_after is not None
            and d_after > home_reached_thresh
        )
        need_fallback = (
            fallback_home_enabled
            and (not nav_succeeded)
            and (d_after is None or d_after > home_reached_thresh)
        )
        if need_fallback:
            if nav_feedback_inconsistent:
                self.get_logger().warn(
                    "[fallback] Triggered: Nav2 feedback is inconsistent "
                    "(distance_remaining~=0 but AMCL still far from home)."
                )
            else:
                self.get_logger().warn(
                    "[fallback] Triggered: Nav2 retries exhausted while still far from home."
                )
            fallback_ok = self._fallback_drive_home(
                home_pose=home_pose,
                timeout_sec=fallback_home_timeout_sec,
                arrival_xy_thresh_m=fallback_arrival_xy_thresh_m,
                heading_thresh_deg=fallback_heading_thresh_deg,
                rotate_step_sec=fallback_rotate_step_sec,
                forward_step_sec=fallback_forward_step_sec,
                align_heading_at_end=not drop_before_final_heading,
            )
            if fallback_ok:
                nav_succeeded = True
                status = GoalStatus.STATUS_SUCCEEDED
                self.get_logger().info("[fallback] Home fallback succeeded.")
            else:
                self.get_logger().warn("[fallback] Home fallback failed or timed out.")

        current_pose_msg = self.get_latest_amcl_pose()
        if current_pose_msg is not None:
            d_after = self._dist_xy(current_pose_msg.pose.pose, home_pose)
            self.get_logger().info(f"After nav, current→home distance={d_after:.3f}m")
        else:
            d_after = None

        # 僅在「導航成功」或「確實已回到 home 附近」時放下，避免原地誤判。
        close_to_home = d_after is not None and d_after <= home_reached_thresh
        # 安全起見：只有「確實接近 home」才放下；僅有 SUCCEEDED 不足以判定已到位
        can_release = close_to_home
        if drop_at_home and can_release:
            self.get_logger().info("Releasing target at home …")
            released = arm.run_release_blocking()
            if not released:
                self.get_logger().warn("Release sequence failed or interrupted.")
        elif drop_at_home:
            status_text = self._GOAL_STATUS_TEXT.get(status, f"UNKNOWN_CODE({status})")
            self.get_logger().warn(
                "Skip release: robot is not close enough to home "
                f"(distance={d_after if d_after is not None else 'unknown'}m, "
                f"threshold={home_reached_thresh:.3f}m, "
                f"nav_succeeded={nav_succeeded}, nav_status={status} [{status_text}])."
            )
        else:
            self.get_logger().info("drop_at_home=false, skip release.")

        if drop_before_final_heading and close_to_home:
            self.get_logger().info(
                "Aligning to recorded home heading after release …"
            )
            heading_ok = self._align_home_heading(
                home_pose=home_pose,
                timeout_sec=fallback_home_timeout_sec,
                heading_thresh_deg=fallback_heading_thresh_deg,
                rotate_step_sec=fallback_rotate_step_sec,
            )
            if heading_ok:
                self.get_logger().info("[home] Final heading aligned.")
            else:
                self.get_logger().warn("[home] Final heading alignment failed or timed out.")

    def _clear_costmaps_once(self):
        try:
            # 不同 Nav2 配置的 service 名稱不同；優先用 clear_entirely_*，並保留舊路徑做 fallback
            targets = [
                (
                    "global_costmap/clear_entirely_global_costmap",
                    self.create_client(
                        ClearEntireCostmap,
                        "/global_costmap/clear_entirely_global_costmap",
                    ),
                ),
                (
                    "local_costmap/clear_entirely_local_costmap",
                    self.create_client(
                        ClearEntireCostmap,
                        "/local_costmap/clear_entirely_local_costmap",
                    ),
                ),
                ("global_costmap/clear", self.clear_global_costmap_client),
                ("local_costmap/clear", self.clear_local_costmap_client),
            ]
            for name, client in targets:
                if not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(f"[nav] Service {name} unavailable, skip.")
                    continue
                try:
                    future = client.call_async(ClearEntireCostmap.Request())
                    t0 = time.monotonic()
                    while not future.done() and time.monotonic() - t0 < 3.0:
                        time.sleep(0.02)
                    if future.done():
                        self.get_logger().info(f"[nav] Cleared via {name}.")
                        return
                    self.get_logger().warn(f"[nav] Clear {name} timeout.")
                except Exception as e:
                    self.get_logger().warn(f"[nav] Clear {name} failed: {e}")
        except Exception as e:
            self.get_logger().warn(f"[nav] _clear_costmaps_once unexpected error: {e}")

    def _nav_feedback_cb(self, feedback_msg, log_period_sec: float):
        now = time.monotonic()
        fb = feedback_msg.feedback
        dist_rem = getattr(fb, "distance_remaining", None)
        if dist_rem is not None:
            prev = self._last_nav_feedback_distance
            self._last_nav_feedback_distance = float(dist_rem)
            self._last_nav_feedback_time = now
            if prev is None or (prev - float(dist_rem)) >= self._nav_stuck_min_progress_m:
                self._last_nav_progress_time = now
        last = getattr(self, "_last_nav_feedback_log_time", 0.0)
        if now - last < log_period_sec:
            return
        self._last_nav_feedback_log_time = now
        nav_time = getattr(fb, "navigation_time", None)
        eta = getattr(fb, "estimated_time_remaining", None)
        cmd = self.get_latest_cmd_vel()
        if cmd is not None:
            lin = math.hypot(float(cmd.linear.x), float(cmd.linear.y))
            ang = abs(float(cmd.angular.z))
            cmd_text = f"lin={lin:.3f}, ang={ang:.3f}"
        else:
            cmd_text = "lin=n/a, ang=n/a"
        self.get_logger().info(
            "[nav] feedback: "
            f"distance_remaining={dist_rem if dist_rem is not None else 'n/a'}, "
            f"nav_time={getattr(nav_time, 'sec', 'n/a')}s, "
            f"eta={getattr(eta, 'sec', 'n/a')}s, "
            f"cmd_vel({cmd_text}), "
            f"amcl_home_dist={self._last_home_dist if self._last_home_dist is not None else 'n/a'}"
        )

    def _unstick_maneuver(
        self,
        back_sec: float,
        rotate_sec: float,
        dp: DataProcessor | None = None,
        guard: ObstacleGuard | None = None,
        obstacle_guard_enabled: bool = False,
    ):
        """
        Nav2 卡住時，先執行短暫脫困再重送 goal：
        1) 慢速後退（僅後方淨空時）
        2) 慢速左旋
        3) 慢速右旋
        """
        self.get_logger().warn(
            f"[nav] Unstick maneuver: back={back_sec:.2f}s, rotate_each={rotate_sec:.2f}s"
        )
        if back_sec > 0.0:
            if dp is not None and guard is not None:
                if not self._publish_backward_if_clear(
                    dp,
                    guard,
                    obstacle_guard_enabled,
                    approach_mode=False,
                    slow=True,
                ):
                    self.get_logger().warn(
                        "[nav] Skip backward unstick — rear obstacle detected."
                    )
                else:
                    time.sleep(back_sec)
            else:
                self.publish_car_control("BACKWARD_SLOW")
                time.sleep(back_sec)
        if rotate_sec > 0.0:
            self.publish_car_control("COUNTERCLOCKWISE_ROTATION")
            time.sleep(rotate_sec)
            self.publish_car_control("CLOCKWISE_ROTATION")
            time.sleep(rotate_sec)
        self.publish_car_control("STOP")
        time.sleep(0.2)

    def _motion_unstick_maneuver(
        self,
        dp: DataProcessor,
        guard: ObstacleGuard,
        back_sec: float,
        shift_sec: float,
        forward_sec: float,
        side_sign: int,
        yolo_lost: bool = False,
        search_spin_speed: float = 70.0,
        dx_px: float | None = None,
        use_lidar_for_unstick: bool = True,
        approach_target_depth_m: float | None = None,
    ):
        """
        通用脫困：後退 →（安全時）往開闊側平移 → 短前進。
        YOLO 丟失：後退 → 原地搜尋轉圈（不側移、不前進）。
        """
        obs = (
            self._evaluate_obstacles(
                dp,
                guard,
                approach_target_depth_m=approach_target_depth_m,
                approach_mode=True,
            )
            if use_lidar_for_unstick
            else None
        )
        lateral_key = self._pick_unstick_lateral_key(
            dp,
            guard,
            side_sign,
            dx_px=dx_px,
            use_lidar=use_lidar_for_unstick,
        )
        spin_cmd = self._yolo_search_wheel_cmd(
            search_spin_speed,
            obs=obs,
            dx_px=dx_px,
        )
        spin_label = (
            "CCW" if spin_cmd[0] < 0 else "CW"
        )
        lat_msg = lateral_key or "skip"
        self.get_logger().warn(
            "[motion] Stuck > timeout: unstick "
            f"back={back_sec:.2f}s, lateral={lat_msg}, forward={forward_sec:.2f}s, "
            f"search={spin_label}"
            + (
                " (yolo_lost → back+search only)"
                if yolo_lost
                else ""
            )
        )
        self.publish_car_control("STOP")
        time.sleep(0.08)
        if back_sec > 0.0:
            if self._publish_backward_if_clear(
                dp,
                guard,
                use_lidar_for_unstick,
                approach_target_depth_m=approach_target_depth_m,
                slow=True,
            ):
                time.sleep(back_sec)
            else:
                self.get_logger().warn(
                    "[motion] Skip backward unstick — rear obstacle detected."
                )
        if yolo_lost:
            self.publish_raw_car_control(spin_cmd)
            time.sleep(0.65)
        else:
            if lateral_key is not None and shift_sec > 0.0:
                self.publish_car_control(lateral_key)
                time.sleep(shift_sec)
            elif shift_sec > 0.0:
                self.get_logger().warn(
                    "[motion] Skip lateral shift — both sides too narrow."
                )
            if forward_sec > 0.0:
                sf = (
                    obs.sensor_front_m
                    if obs is not None
                    else float("inf")
                )
                if not math.isfinite(sf) or sf >= guard.stop_m + 0.06:
                    self.publish_car_control("FORWARD_SLOW")
                    time.sleep(forward_sec)
                else:
                    raw_s = sf if math.isfinite(sf) else -1.0
                    self.get_logger().warn(
                        "[motion] Skip forward — front wall too close "
                        f"(raw={raw_s:.2f}m)."
                    )
        self.publish_car_control("STOP")
        time.sleep(0.15)

    def _approach_unstick_maneuver(
        self,
        back_sec: float,
        shift_sec: float,
        forward_sec: float,
        dx_px: float,
        side_sign: int,
    ):
        """Legacy wrapper — prefer _motion_unstick_maneuver."""
        del dx_px
        lateral_key = "LEFT_SHIFT" if side_sign > 0 else "RIGHT_SHIFT"
        self.get_logger().warn(
            "[approach] Stuck > timeout: unstick "
            f"back={back_sec:.2f}s, {lateral_key}={shift_sec:.2f}s, "
            f"forward={forward_sec:.2f}s"
        )
        self.publish_car_control("STOP")
        time.sleep(0.08)
        if back_sec > 0.0:
            self.publish_car_control("BACKWARD_SLOW")
            time.sleep(back_sec)
        if shift_sec > 0.0:
            self.publish_car_control(lateral_key)
            time.sleep(shift_sec)
        if forward_sec > 0.0:
            self.publish_car_control("FORWARD_SLOW")
            time.sleep(forward_sec)
        self.publish_car_control("STOP")
        time.sleep(0.15)

    def _align_home_heading(
        self,
        home_pose,
        timeout_sec: float,
        heading_thresh_deg: float,
        rotate_step_sec: float,
    ) -> bool:
        """在 home XY 附近，將車體轉回任務開始時記錄的面向。"""
        heading_thresh = math.radians(heading_thresh_deg)
        home_yaw = self._yaw_from_quat(home_pose.orientation)
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            amcl_pose_msg = self.get_latest_amcl_pose()
            if amcl_pose_msg is None:
                self.publish_car_control("STOP")
                time.sleep(0.05)
                continue

            curr_yaw = self._yaw_from_quat(amcl_pose_msg.pose.pose.orientation)
            yaw_err = self._normalize_angle(home_yaw - curr_yaw)
            if abs(yaw_err) <= heading_thresh:
                self.publish_car_control("STOP")
                return True
            self.publish_car_control(
                "COUNTERCLOCKWISE_ROTATION"
                if yaw_err > 0.0
                else "CLOCKWISE_ROTATION"
            )
            time.sleep(rotate_step_sec)
            self.publish_car_control("STOP")
            time.sleep(0.03)

        self.publish_car_control("STOP")
        return False

    def _fallback_drive_home(
        self,
        home_pose,
        timeout_sec: float,
        arrival_xy_thresh_m: float,
        heading_thresh_deg: float,
        rotate_step_sec: float,
        forward_step_sec: float,
        align_heading_at_end: bool = True,
    ) -> bool:
        """
        當 Nav2 異常時，用 AMCL 做簡單閉迴路回家：
        1) 面向 home 方向
        2) 前進直到進入 home 半徑
        3) （可選）對齊 home 原始朝向
        """
        heading_thresh = math.radians(heading_thresh_deg)
        t0 = time.monotonic()
        last_log = 0.0
        phase = "goto"
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            amcl_pose_msg = self.get_latest_amcl_pose()
            if amcl_pose_msg is None:
                self.publish_car_control("STOP")
                time.sleep(0.05)
                continue

            p = amcl_pose_msg.pose.pose
            dx = float(home_pose.position.x - p.position.x)
            dy = float(home_pose.position.y - p.position.y)
            dist = math.hypot(dx, dy)
            curr_yaw = self._yaw_from_quat(p.orientation)
            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    f"[fallback] phase={phase}, dist={dist:.3f}m, "
                    f"yaw={math.degrees(curr_yaw):.1f}deg"
                )
                last_log = now

            if dist <= arrival_xy_thresh_m:
                self.publish_car_control("STOP")
                if not align_heading_at_end:
                    return True
                return self._align_home_heading(
                    home_pose=home_pose,
                    timeout_sec=max(3.0, timeout_sec - (time.monotonic() - t0)),
                    heading_thresh_deg=heading_thresh_deg,
                    rotate_step_sec=rotate_step_sec,
                )

            phase = "goto"
            heading_to_home = math.atan2(dy, dx)
            heading_err = self._normalize_angle(heading_to_home - curr_yaw)
            if abs(heading_err) > heading_thresh:
                self.publish_car_control(
                    "COUNTERCLOCKWISE_ROTATION"
                    if heading_err > 0.0
                    else "CLOCKWISE_ROTATION"
                )
                time.sleep(rotate_step_sec)
            else:
                self.publish_car_control("FORWARD_SLOW")
                time.sleep(forward_step_sec)
            self.publish_car_control("STOP")
            time.sleep(0.03)

        self.publish_car_control("STOP")
        return False

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(theta: float) -> float:
        return math.atan2(math.sin(theta), math.cos(theta))

    @staticmethod
    def _braking_distance_m(speed_mps: float, decel_mps2: float, safety_m: float) -> float:
        """物理煞車距離：d = v² / (2a) + safety"""
        if decel_mps2 <= 1e-6:
            return safety_m
        return (max(0.0, speed_mps) ** 2) / (2.0 * decel_mps2) + safety_m

    @staticmethod
    def _max_allowable_speed_mps(margin_m: float, decel_mps2: float) -> float:
        """剩餘 margin 內能安全停下的最大速度：v = sqrt(2 a d)"""
        if margin_m <= 0.0 or decel_mps2 <= 1e-6:
            return 0.0
        return math.sqrt(2.0 * decel_mps2 * margin_m)

    @staticmethod
    def _lookahead_forward_wheel_speed(
        dist_m: float,
        closure_mps: float,
        stop_m: float,
        decel_mps2: float,
        safety_m: float,
        max_wheel: float,
        max_mps: float,
    ) -> float:
        """
        依剩餘距離與目前接近速度，計算允許的前進輪速上限。
        核心：v_allow = sqrt(2 a (dist - stop))，再映射到 wheel 指令。
        """
        margin = max(0.0, dist_m - stop_m)
        v_allow = BearMissionHost._max_allowable_speed_mps(margin, decel_mps2)
        v_cruise = min(max_mps, v_allow)

        # 接近停止距離時額外降速（預留慣性）
        if margin < 0.55:
            v_cruise = min(
                v_cruise,
                v_allow * max(0.12, (margin / 0.55) ** 1.6),
            )

        # 若實際接近速度已高於允許值，強制降速
        if closure_mps > v_allow * 1.02 and v_allow > 0.0:
            v_cruise = min(v_cruise, v_allow * 0.42)
        elif closure_mps > v_allow * 0.85 and margin < 0.45:
            v_cruise = min(v_cruise, v_allow * 0.55)

        ratio = min(1.0, v_cruise / max(max_mps, 1e-6))
        return max(0.0, min(max_wheel, ratio * max_wheel))

    @staticmethod
    def _dist_xy(pose_a, pose_b) -> float:
        dx = float(pose_a.position.x - pose_b.position.x)
        dy = float(pose_a.position.y - pose_b.position.y)
        return math.hypot(dx, dy)


def main(args=None):
    rclpy.init(args=args)
    node = BearMissionHost()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()
    try:
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
