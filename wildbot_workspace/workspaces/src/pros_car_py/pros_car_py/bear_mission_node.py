"""
任務一（整段自動）：訂閱對側 YOLO 的 topic → 視覺逼近 → 夾取 → NavigateToPose 回記錄的起點。

典型用法（兩個環境／容器，同一 ROS_DOMAIN_ID）：
  - ros2_yolo_integration：`ros2 run yolo_example_pkg yolo_node --ros-args -p target_class:=bear ...`
  - pros_car：`ros2 run pros_car_py bear_mission`，或 `ros2 launch pros_car_py bear_task1.launch.py`（僅 bear_mission）

預設 **auto_start:=true**：延遲後自動跑完整流程；手動觸發：`auto_start:=false` 再呼叫 `/start_bear_mission`。
預設 **repeat_mission:=true**：一輪結束後立即開始下一輪，直到 Ctrl+C 停止；單次執行：`-p repeat_mission:=false`。

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
from geometry_msgs.msg import PoseStamped, Quaternion
try:
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ClearEntireCostmap
    _NAV2_AVAILABLE = True
except ImportError:
    NavigateToPose = None
    ClearEntireCostmap = None
    _NAV2_AVAILABLE = False
try:
    from lifecycle_msgs.msg import State as LifecycleState
    from lifecycle_msgs.srv import GetState as LifecycleGetState
    _LIFECYCLE_AVAILABLE = True
except ImportError:
    LifecycleState = None
    LifecycleGetState = None
    _LIFECYCLE_AVAILABLE = False
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
        self.declare_parameter("visual_servo_target_depth_m", 0.38)
        self.declare_parameter("visual_servo_search_spin_speed", 130.0)
        self.declare_parameter("visual_servo_max_forward_speed", 170.0)
        self.declare_parameter("visual_servo_max_forward_speed_far", 390.0)
        self.declare_parameter("visual_servo_far_distance_m", 0.90)
        self.declare_parameter("visual_servo_yaw_deadband_px", 24.0)
        self.declare_parameter("visual_servo_yaw_soft_scale_px", 150.0)
        self.declare_parameter("visual_servo_max_yaw_near", 90.0)
        self.declare_parameter("visual_servo_max_yaw_far", 110.0)
        self.declare_parameter("visual_servo_min_yaw_large_px", 85.0)
        self.declare_parameter("approach_turn_stuck_time_sec", 2.0)
        self.declare_parameter("approach_stuck_time_sec", 3.0)
        self.declare_parameter("approach_stuck_back_sec", 0.55)
        self.declare_parameter("approach_stuck_shift_sec", 0.45)
        self.declare_parameter("approach_stuck_forward_sec", 0.35)
        self.declare_parameter("motion_stuck_min_progress_m", 0.06)
        self.declare_parameter("visual_servo_lost_timeout_sec", 0.8)
        self.declare_parameter("visual_servo_dx_ema_alpha", 0.25)
        self.declare_parameter("visual_servo_depth_ema_alpha", 0.35)
        self.declare_parameter("align_pixel_thresh", 40.0)
        self.declare_parameter("align_pixel_bias_px", 0.0)
        self.declare_parameter("align_stable_frames", 5)
        # 切入慢速對齊的距離閾值
        self.declare_parameter("approach_slow_dist_m", 0.90)
        # 低於此深度 or 熊消失 → 立刻停下夾取
        self.declare_parameter("grasp_trigger_dist_m", 0.40)
        # 熊剛消失後再多等幾幀再夾（等車身慣性停穩）
        self.declare_parameter("grasp_trigger_lost_frames", 3)
        # bbox 寬度超過此像素數 → 視為足夠近，觸發夾爪（0 = 停用）
        self.declare_parameter("grasp_bbox_px", 200.0)
        # 連續 N 幀 dist < grasp zone 才確認觸發，過濾深度突波
        self.declare_parameter("grasp_confirm_frames", 4)
        self.declare_parameter("grasp_depth_jump_m", 0.35)
        # 前瞻煞車：用 v²/(2a) 估算「現在這個速度要多久、多遠才停得住」
        self.declare_parameter("approach_stop_dist_m", 0.35)
        self.declare_parameter("approach_decel_mps2", 0.45)
        self.declare_parameter("approach_max_speed_mps", 0.55)
        self.declare_parameter("approach_brake_safety_m", 0.12)
        self.declare_parameter("visual_servo_center_deadband_px", 32.0)
        self.declare_parameter("visual_servo_image_half_width_px", 320.0)
        self.declare_parameter("grasp_depth_max_m", 0.62)
        self.declare_parameter("grasp_depth_min_m", 0.12)
        self.declare_parameter("marker_wait_sec", 3.0)
        self.declare_parameter("nav_home_timeout_sec", 180.0)
        self.declare_parameter("home_reached_dist_thresh_m", 0.50)
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
        self.declare_parameter("align_home_heading_after_drop", True)
        self.declare_parameter("home_heading_align_timeout_sec", 25.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_start_delay_sec", 2.0)
        self.declare_parameter("repeat_mission", True)
        self.declare_parameter("amcl_wait_timeout_sec", 120.0)
        self.declare_parameter("startup_forward_m", 0.0)       # 啟動後先前進這麼多（0=不動）
        self.declare_parameter("startup_forward_speed", 0.25)  # 初始前進速度 m/s
        self.declare_parameter("startup_moves", "")            # 序列如 "F:0.85,R:90,F:0.30"（優先於 startup_forward_m）
        self.declare_parameter("startup_rotation_speed", 0.8)  # 啟動旋轉速度 rad/s
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
        # YOLO 丟失後的緩衝 / 搜尋狀態機參數
        # 最後補進：熊脫離相機視野時，依最後記錄的距離與方向前進補足後再夾
        self.declare_parameter("grasp_final_nudge_enabled", True)
        self.declare_parameter("grasp_final_nudge_wheel_speed", 130.0)
        self.declare_parameter("grasp_final_nudge_max_sec", 1.8)
        self.declare_parameter("grasp_final_nudge_min_sec", 0.45)
        # 熊在此距離內消失 → 先後退一小段重找，仍找不到才記憶抓取
        self.declare_parameter("grasp_commit_dist_m", -1.0)
        self.declare_parameter("approach_yolo_lost_back_enabled", True)
        self.declare_parameter("approach_yolo_lost_back_sec", 0.55)
        self.declare_parameter("approach_yolo_lost_back_max_attempts", 2)
        self.declare_parameter("approach_yolo_lost_grace_sec", 1.5)
        self.declare_parameter("approach_yolo_lost_min_frames", 12)
        self.declare_parameter("approach_yolo_search_spin_speed_tier", "slow")
        self.declare_parameter("approach_yolo_explore_forward_sec", 3.0)
        self.declare_parameter("approach_yolo_explore_forward_speed", 180.0)
        self.declare_parameter("approach_yolo_search_turn_deg", 30.0)
        self.declare_parameter("approach_yolo_lost_motion_log_sec", 2.0)
        self.declare_parameter("status_log_interval_sec", 1.0)
        self.declare_parameter("mission_verbose_log", False)

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
            repeat = (
                self.get_parameter("repeat_mission").get_parameter_value().bool_value
            )
            repeat_hint = (
                "完成後自動接下一輪（Ctrl+C 停止）。"
                if repeat
                else "單次執行（repeat_mission:=false）。"
            )
            self.get_logger().info(
                f"auto_start enabled: full pipeline begins in {delay:.1f}s "
                "(record home → approach → grasp → nav home). "
                f"{repeat_hint} "
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
        repeat_mission = (
            self.get_parameter("repeat_mission").get_parameter_value().bool_value
        )
        round_num = 0
        try:
            while rclpy.ok():
                round_num += 1
                if repeat_mission and round_num > 1:
                    self.get_logger().info(
                        f"=== 第 {round_num} 輪夾熊任務開始（Ctrl+C 停止）==="
                    )
                self._run_mission()
                if not repeat_mission or not rclpy.ok():
                    break
                self.get_logger().info(
                    f"=== 第 {round_num} 輪完成，立即開始下一輪（Ctrl+C 停止）==="
                )
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

    @staticmethod
    def _zone_name_cn(zone: str) -> str:
        return {"far": "遠距", "mid": "中距", "slow": "近距"}.get(zone, zone)

    @staticmethod
    def _turn_dir_cn(raw_dx: float, deadband_px: float) -> str:
        if raw_dx < -deadband_px:
            return "左轉對準"
        if raw_dx > deadband_px:
            return "右轉對準"
        return "直進"

    @staticmethod
    def _block_cmd_cn(block_cmd: str | None) -> str | None:
        mapping = {
            "STOP": "停車",
            "CLOCKWISE_ROTATION": "右轉閃避",
            "COUNTERCLOCKWISE_ROTATION": "左轉閃避",
        }
        return mapping.get(block_cmd) if block_cmd else None

    def _obstacle_summary_cn(self, obs) -> str:
        block = self._block_cmd_cn(obs.block_cmd)
        if block:
            side = ""
            if obs.block_cmd == "CLOCKWISE_ROTATION":
                side = "（左側障礙）"
            elif obs.block_cmd == "COUNTERCLOCKWISE_ROTATION":
                side = "（右側障礙）"
            elif obs.block_cmd == "STOP":
                side = "（前方過近）"
            return f"障礙→{block}{side}"

        notes: list[str] = []
        if math.isfinite(obs.left_clearance_m) and math.isfinite(obs.right_clearance_m):
            if obs.left_clearance_m + 0.08 < obs.right_clearance_m:
                notes.append(f"左側較近 {obs.left_clearance_m:.2f}m")
            elif obs.right_clearance_m + 0.08 < obs.left_clearance_m:
                notes.append(f"右側較近 {obs.right_clearance_m:.2f}m")
        if math.isfinite(obs.front_clearance_m) and obs.front_clearance_m < 0.55:
            notes.append(f"前方 {obs.front_clearance_m:.2f}m")
        if obs.speed_scale < 0.98:
            notes.append(f"減速 x{obs.speed_scale:.2f}")
        return "路徑暢通" if not notes else "，".join(notes)

    def _log_status_if_due(
        self,
        last_log_time: float,
        interval_sec: float,
        message: str,
    ) -> float:
        now = time.monotonic()
        if now - last_log_time < interval_sec:
            return last_log_time
        self.get_logger().info(f"[狀態] {message}")
        return now

    def _build_approach_status_cn(
        self,
        *,
        use_approach: bool,
        yolo_search_confirmed: bool,
        in_hold: bool,
        target_live: bool,
        yolo_search_state: dict,
        dist: float,
        last_valid_dist: float | None,
        control_dx: float,
        yolo_dx: float | None,
        deadband_px: float,
        zone_far: float,
        zone_mid: float,
        obs,
    ) -> str:
        obs_text = self._obstacle_summary_cn(obs) if obs is not None else "路徑暢通"
        if yolo_search_confirmed:
            phase = yolo_search_state.get("phase", "forward")
            if phase == "back_peek":
                mode = "後退尋找"
            elif phase == "turn_right":
                acc_deg = math.degrees(float(yolo_search_state.get("accumulated_rad", 0.0)))
                mode = f"右轉搜尋（已轉 {acc_deg:.0f}°）"
            elif phase == "forward":
                remain = max(
                    0.0,
                    float(yolo_search_state.get("forward_until", 0.0)) - time.monotonic(),
                )
                mode = f"直走探索（剩 {remain:.1f}s）"
            else:
                mode = "準備搜尋"
            dist_hint = (
                f"，上次距離 {last_valid_dist:.2f}m"
                if last_valid_dist is not None
                else ""
            )
            return f"找尋目標｜{mode}{dist_hint}｜{obs_text}"

        d_eff = dist if dist > 0.0 else (last_valid_dist if last_valid_dist is not None else -1.0)
        if d_eff <= 0.0:
            phase = "等待偵測"
        elif d_eff > zone_far:
            phase = "全速接近"
        elif d_eff > zone_mid:
            phase = "中速接近"
        else:
            phase = "慢速接近"

        target_tag = "即時" if target_live else ("沿用" if in_hold else "未知")
        turn = self._turn_dir_cn(control_dx, deadband_px)
        dist_text = f"{d_eff:.2f}m" if d_eff > 0.0 else "未知"
        if yolo_dx is not None and abs(yolo_dx - control_dx) > 30.0:
            dx_text = (
                f"控制 {control_dx:+.0f}px，YOLO {yolo_dx:+.0f}px（{target_tag}）"
            )
        else:
            dx_text = f"偏差 {control_dx:+.0f}px（{target_tag}）"
        return (
            f"接近目標｜{phase}｜{turn}｜"
            f"距離 {dist_text}（{dx_text}）｜{obs_text}"
        )

    def _build_nav_status_cn(
        self,
        *,
        attempt: int,
        attempt_total: int,
        home_dist: float | None,
        nav_feedback_dist: float | None,
        obs,
    ) -> str:
        obs_text = self._obstacle_summary_cn(obs) if obs is not None else "路徑暢通"
        if home_dist is not None:
            dist_text = f"距起點 {home_dist:.2f}m"
        else:
            dist_text = "距起點 未知"
        nav2_text = (
            f"，Nav2 剩餘 {nav_feedback_dist:.2f}m"
            if nav_feedback_dist is not None
            else ""
        )
        return (
            f"回程導航（第 {attempt}/{attempt_total} 次）｜"
            f"{dist_text}{nav2_text}｜{obs_text}"
        )

    @staticmethod
    def _reset_yolo_search_state(state: dict) -> None:
        state["phase"] = "idle"
        state["accumulated_rad"] = 0.0
        state["last_yaw"] = None
        state["forward_until"] = 0.0
        state["logged_search"] = False
        state["back_until"] = 0.0
        state["logged_back"] = False
        state["back_attempts"] = 0

    @staticmethod
    def _close_range_lost(
        last_valid_dist: float | None, grasp_commit_dist_m: float
    ) -> bool:
        return (
            last_valid_dist is not None
            and last_valid_dist <= grasp_commit_dist_m
        )

    @staticmethod
    def _target_live(yolo_target_info) -> bool:
        if yolo_target_info is None or len(yolo_target_info) < 3:
            return False
        return (
            float(yolo_target_info[0]) == 1.0
            and float(yolo_target_info[1]) > 0.0
        )

    @staticmethod
    def _should_memory_grasp_commit(
        *,
        target_live: bool,
        last_valid_dist: float | None,
        lost_frames: int,
        grasp_commit_dist_m: float,
        min_lost_frames: int = 1,
        back_enabled: bool = False,
        back_attempts: int = 0,
        back_max_attempts: int = 0,
    ) -> bool:
        """熊在 commit 距離內消失 → 後退重找仍失敗後，用記憶位置直接抓取。"""
        if target_live or last_valid_dist is None:
            return False
        if not (
            last_valid_dist <= grasp_commit_dist_m
            and lost_frames >= max(1, min_lost_frames)
        ):
            return False
        if back_enabled and back_attempts < back_max_attempts:
            return False
        return True

    def _home_dist_from_amcl(self, home_pose) -> float | None:
        amcl_pose_msg = self.get_latest_amcl_pose()
        if amcl_pose_msg is None or home_pose is None:
            return None
        return self._dist_xy(amcl_pose_msg.pose.pose, home_pose)

    def _log_nav2_inactive_help(self) -> None:
        self.get_logger().error(
            "[nav] bt_navigator 未 active，NavigateToPose 會 reject goal。"
            "請依序執行：./scripts/set_initial_pose.sh → ./scripts/restart_navigation.sh"
            "（或 ./scripts/verify_nav.sh 確認全部 OK）"
        )

    def _is_bt_navigator_active(self) -> bool | None:
        """Return True/False when lifecycle service responds; None if unavailable."""
        if not _LIFECYCLE_AVAILABLE or LifecycleGetState is None:
            return None
        if not hasattr(self, "_bt_nav_get_state_client"):
            self._bt_nav_get_state_client = self.create_client(
                LifecycleGetState, "/bt_navigator/get_state"
            )
        client = self._bt_nav_get_state_client
        if not client.wait_for_service(timeout_sec=1.5):
            return None
        future = client.call_async(LifecycleGetState.Request())
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < 2.0:
            time.sleep(0.02)
        if not future.done():
            return None
        try:
            state_id = future.result().current_state.id
        except Exception:
            return None
        return state_id == LifecycleState.PRIMARY_STATE_ACTIVE

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
    def _motion_is_stuck(state: dict, stuck_time_sec: float) -> bool:
        return (
            time.monotonic() - state["last_progress_time"] > stuck_time_sec
        )

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

    @staticmethod
    def _side_obs_yaw_correction(
        dp: DataProcessor,
        guard: ObstacleGuard,
        max_yaw: float = 45.0,
        left_react_m: float = 0.28,   # 左側較不敏感（比賽從左側出發）
        right_react_m: float = 0.45,
        front_react_m: float = 0.40,  # 前方開始減速
        front_stop_m: float = 0.20,   # 前方此距離內完全擋住前進
        rear_react_m: float = 0.30,
        rear_stop_m: float = 0.15,
        max_fwd_wheel: float = 124.0,  # ≈ 0.15 m/s in OLD_MAX=450 scale
    ) -> tuple[float, float, float, float | None, float | None, float | None, float | None]:
        """
        LiDAR 左右側邊 + 前方 + 後方障礙補正。
        Returns (yaw, fwd_wheel_delta, front_block_ratio, left_m, right_m, rear_m, front_m).
        yaw > 0 = 右轉；fwd > 0 = 後方推前進；front_block_ratio 0→1 縮減前進速度。
        """
        try:
            front_m, left_m, right_m = get_lidar_sector_minimums(dp)
        except Exception:
            front_m, left_m, right_m = None, None, None

        rear_m = get_lidar_rear_minimum(dp)
        stop_m = guard.stop_m

        def _intensity(dist, react, stop):
            if dist is None or not math.isfinite(dist) or dist >= react:
                return 0.0
            t = max(0.0, (dist - stop) / max(react - stop, 0.01))
            return 1.0 - t

        yaw = (
            max_yaw * _intensity(left_m, left_react_m, stop_m)
            - max_yaw * _intensity(right_m, right_react_m, stop_m)
        )
        fwd = max_fwd_wheel * _intensity(rear_m, rear_react_m, rear_stop_m)
        front_block = _intensity(front_m, front_react_m, front_stop_m)
        return yaw, fwd, front_block, left_m, right_m, rear_m, front_m

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
        emit_detail: bool = True,
    ) -> float:
        now = time.monotonic()
        if now - last_log_time < log_interval:
            return last_log_time
        if not emit_detail:
            return now
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
                        src_line += " | depth_patch=front_left"
                    else:
                        src_line += " | depth_patch=left"
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
        emit_detail: bool = True,
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
            emit_detail=emit_detail,
        )

        if wheel_cmd is not None:
            allow_visual_yaw = prefer_visual_yaw and not guard.must_override_visual_yaw(obs)
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
        visual_servo_dx_ema_alpha = max(
            0.05,
            min(1.0, self.get_parameter("visual_servo_dx_ema_alpha").get_parameter_value().double_value),
        )
        visual_servo_depth_ema_alpha = max(
            0.05,
            min(1.0, self.get_parameter("visual_servo_depth_ema_alpha").get_parameter_value().double_value),
        )
        grasp_final_nudge_enabled = (
            self.get_parameter("grasp_final_nudge_enabled").get_parameter_value().bool_value
        )
        grasp_final_nudge_wheel_speed = max(
            40.0,
            self.get_parameter("grasp_final_nudge_wheel_speed").get_parameter_value().double_value,
        )
        grasp_final_nudge_max_sec = max(
            0.1,
            self.get_parameter("grasp_final_nudge_max_sec").get_parameter_value().double_value,
        )
        grasp_final_nudge_min_sec = max(
            0.0,
            self.get_parameter("grasp_final_nudge_min_sec").get_parameter_value().double_value,
        )
        approach_yolo_lost_grace_sec = max(
            0.3,
            self.get_parameter("approach_yolo_lost_grace_sec")
            .get_parameter_value()
            .double_value,
        )
        approach_yolo_lost_min_frames = max(
            3,
            int(
                self.get_parameter("approach_yolo_lost_min_frames")
                .get_parameter_value()
                .integer_value
            ),
        )
        approach_yolo_lost_back_enabled = (
            self.get_parameter("approach_yolo_lost_back_enabled")
            .get_parameter_value()
            .bool_value
        )
        approach_yolo_lost_back_sec = max(
            0.2,
            self.get_parameter("approach_yolo_lost_back_sec")
            .get_parameter_value()
            .double_value,
        )
        approach_yolo_lost_back_max_attempts = max(
            1,
            int(
                self.get_parameter("approach_yolo_lost_back_max_attempts")
                .get_parameter_value()
                .integer_value
            ),
        )
        approach_yolo_search_spin_speed_tier = (
            self.get_parameter("approach_yolo_search_spin_speed_tier")
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        if approach_yolo_search_spin_speed_tier not in ("slow", "median", "fast", "full"):
            approach_yolo_search_spin_speed_tier = "slow"
        approach_yolo_explore_forward_sec = max(
            0.5,
            self.get_parameter("approach_yolo_explore_forward_sec")
            .get_parameter_value()
            .double_value,
        )
        approach_yolo_explore_forward_speed = max(
            40.0,
            self.get_parameter("approach_yolo_explore_forward_speed")
            .get_parameter_value()
            .double_value,
        )
        approach_yolo_search_turn_deg = max(
            5.0,
            self.get_parameter("approach_yolo_search_turn_deg")
            .get_parameter_value()
            .double_value,
        )
        approach_yolo_search_turn_rad = math.radians(approach_yolo_search_turn_deg)
        approach_yolo_lost_motion_log_sec = max(
            0.5,
            self.get_parameter("approach_yolo_lost_motion_log_sec")
            .get_parameter_value()
            .double_value,
        )
        obstacle_source_debug_enabled = (
            self.get_parameter("obstacle_source_debug_enabled")
            .get_parameter_value()
            .bool_value
        )
        mission_verbose_log = (
            self.get_parameter("mission_verbose_log").get_parameter_value().bool_value
        )
        status_log_interval_sec = max(
            0.3,
            self.get_parameter("status_log_interval_sec")
            .get_parameter_value()
            .double_value,
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
        grasp_bbox_px = (
            self.get_parameter("grasp_bbox_px").get_parameter_value().double_value
        )
        grasp_confirm_frames = max(1, int(
            self.get_parameter("grasp_confirm_frames").get_parameter_value().integer_value
        ))
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
        align_home_heading_after_drop = (
            self.get_parameter("align_home_heading_after_drop")
            .get_parameter_value()
            .bool_value
        )
        home_heading_align_timeout_sec = max(
            3.0,
            self.get_parameter("home_heading_align_timeout_sec")
            .get_parameter_value()
            .double_value,
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

        # 優先嘗試從 odom 記錄起點（不需要 AMCL / Nav2 / 地圖）
        home_odom = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and rclpy.ok():
            odom_msg = self.get_latest_odom()
            if odom_msg is not None:
                home_odom = copy.deepcopy(odom_msg.pose.pose)
                self.get_logger().info(
                    f"Home pose recorded from /odom: "
                    f"x={home_odom.position.x:.3f}, y={home_odom.position.y:.3f}"
                )
                break
            time.sleep(0.05)
        if home_odom is None:
            self.get_logger().warn("收不到 /odom，夾取後不回起點。")

        # 同時嘗試 AMCL（若有的話，用於 Nav2 回程）
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
                self.get_logger().info(
                    f"Home pose recorded from '{amcl_topic}': "
                    f"x={home_pose.position.x:.3f}, y={home_pose.position.y:.3f}"
                )
                break
            now = time.monotonic()
            if now - last_log >= 5.0:
                self.get_logger().warn(
                    f"Still waiting for '{amcl_topic}' — 跳過 AMCL，將使用 odom 回程。"
                )
                last_log = now
            time.sleep(0.05)

        if home_pose is None:
            self.get_logger().warn(
                "收不到 AMCL pose，將使用 odom-based 回程（精度較低但不需要地圖）。"
            )

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

        # 實體車：任務開始前將手臂移至待機位置（與 COMMANDS.md 一致，夾爪打開）
        if not use_unity:
            self.get_logger().info(
                "實體車手臂初始化：待機 [3.67, 0.5, 4.0]（shoulder, elbow, gripper open）…"
            )
            self.publish_robot_arm_angle([3.67, 0.5, 4.0])
            time.sleep(2.0)

        # 啟動移動序列
        startup_forward_speed = (
            self.get_parameter("startup_forward_speed").get_parameter_value().double_value
        )
        startup_rotation_speed = (
            self.get_parameter("startup_rotation_speed").get_parameter_value().double_value
        )
        startup_moves_str = (
            self.get_parameter("startup_moves").get_parameter_value().string_value.strip()
        )
        startup_forward_m = (
            self.get_parameter("startup_forward_m").get_parameter_value().double_value
        )
        if not use_unity:
            if startup_moves_str:
                self._run_startup_sequence(
                    startup_moves_str, startup_forward_speed, startup_rotation_speed
                )
            elif startup_forward_m > 0.01:
                # 向後相容：只設了 startup_forward_m 時的舊行為
                self.get_logger().info(f"[startup] 初始前進 {startup_forward_m:.2f}m ...")
                start_odom = None
                for _ in range(50):
                    start_odom = self.get_latest_odom()
                    if start_odom is not None:
                        break
                    time.sleep(0.05)
                if start_odom is not None:
                    sx = start_odom.pose.pose.position.x
                    sy = start_odom.pose.pose.position.y
                    timeout = startup_forward_m / max(startup_forward_speed, 0.1) * 2.5
                    t0_sf = time.monotonic()
                    while rclpy.ok():
                        if time.monotonic() - t0_sf > timeout:
                            self.get_logger().warn("[startup] 初始前進 timeout，繼續任務。")
                            break
                        odom = self.get_latest_odom()
                        if odom is not None:
                            dx = odom.pose.pose.position.x - sx
                            dy = odom.pose.pose.position.y - sy
                            if math.sqrt(dx * dx + dy * dy) >= startup_forward_m:
                                self.get_logger().info(
                                    f"[startup] 前進完成 {startup_forward_m:.3f}m。"
                                )
                                break
                        self.publish_raw_car_control([startup_forward_speed, 0.0])
                        time.sleep(0.05)
                    self.publish_raw_car_control([0.0, 0.0])
                else:
                    self.get_logger().warn("[startup] 收不到 odom，跳過初始前進。")

        # 距離分區定義（公尺）
        #  dist > zone_far   : 全速直衝（300），方向只做大角度修正
        #  zone_mid < dist   : 中速（160），精準轉向對準
        #  zone_slow < dist  : 慢速（80），最後微調
        #  dist <= grasp_zone: 主動煞車後停下 → 夾取
        zone_far  = 1.50
        zone_mid  = 0.80
        zone_slow = grasp_trigger_dist_m  # 預設 0.40
        grasp_commit_dist_raw = (
            self.get_parameter("grasp_commit_dist_m").get_parameter_value().double_value
        )
        grasp_commit_dist_m = (
            max(zone_slow + 0.12, grasp_commit_dist_raw)
            if grasp_commit_dist_raw > 0.0
            else max(zone_slow * 1.85, 0.55)
        )

        self.get_logger().info(
            "Approaching bear at full speed "
            f"(stop={approach_stop_dist_m:.2f}m, grasp_zone={zone_slow:.2f}m) …"
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
        last_status_log = 0.0
        last_target_valid_time = 0.0
        held_ti = None
        approach_unstick_side = 1
        motion_progress = {"last_progress_time": time.monotonic(), "last_pose_xy": None}
        yolo_search_state = {
            "phase": "idle",
            "accumulated_rad": 0.0,
            "last_yaw": None,
            "forward_until": 0.0,
            "logged_search": False,
            "back_until": 0.0,
            "logged_back": False,
            "back_attempts": 0,
        }
        # 連續幀確認：連續 N 幀在 grasp zone 才觸發
        grasp_close_frames = 0
        # 鎖定第一幀目標的方向，旋轉中不因 YOLO 抖動換目標
        locked_dx = None       # 鎖住的水平偏差（像素）
        locked_dx_time = None  # 鎖住的時間
        prev_approach_dist = None
        nav.reset_visual_servo()

        # 去程路徑記錄：每 0.1m 存一個 AMCL + odom 座標，供回程使用
        outbound_waypoints: list = []   # [(amcl_x, amcl_y, odom_x, odom_y), ...]
        _last_wp_amcl_xy: tuple | None = None

        while time.monotonic() - t_start < t_approach_max and rclpy.ok():
            ti = dp.get_yolo_target_info()
            target_live = self._target_live(ti)

            if target_live:
                live_dist = float(ti[1])
                live_dx = float(ti[2])
            else:
                live_dist = -1.0
                live_dx = 0.0

            bbox_w_px = float(ti[3]) if (target_live and len(ti) >= 4) else 0.0

            # 深度突然跳遠（YOLO 換成下一隻熊 / 太近深度失效後重選）
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
                self.publish_car_control("STOP")
                break

            # YOLO 存活 → 更新 hold 狀態 / 重設搜尋狀態機
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
                now_hold = time.monotonic()
                if mission_verbose_log and now_hold - last_yolo_hold_log >= 1.0:
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
                    last_yolo_hold_log = now_hold

            if yolo_search_confirmed and yolo_search_state["phase"] == "idle":
                close_lost = self._close_range_lost(
                    last_valid_dist, grasp_commit_dist_m
                )
                if (
                    close_lost
                    and approach_yolo_lost_back_enabled
                    and yolo_search_state["back_attempts"]
                    < approach_yolo_lost_back_max_attempts
                ):
                    yolo_search_state["phase"] = "back_peek"
                    yolo_search_state["back_until"] = (
                        time.monotonic() + approach_yolo_lost_back_sec
                    )
                    if not yolo_search_state["logged_back"]:
                        self.get_logger().info(
                            "[approach/yolo_lost] target lost near grasp "
                            f"(last_dist={last_valid_dist:.2f}m) → back up "
                            f"{approach_yolo_lost_back_sec:.1f}s to reacquire "
                            f"(attempt {yolo_search_state['back_attempts'] + 1}/"
                            f"{approach_yolo_lost_back_max_attempts})"
                        )
                        yolo_search_state["logged_back"] = True
                elif self._should_memory_grasp_commit(
                    target_live=target_live,
                    last_valid_dist=last_valid_dist,
                    lost_frames=lost_frames,
                    grasp_commit_dist_m=grasp_commit_dist_m,
                    min_lost_frames=1,
                    back_enabled=approach_yolo_lost_back_enabled,
                    back_attempts=yolo_search_state["back_attempts"],
                    back_max_attempts=approach_yolo_lost_back_max_attempts,
                ):
                    aligned = True
                    self.get_logger().info(
                        f"[approach] Memory grasp commit: last seen {last_valid_dist:.2f}m "
                        f"(back attempts={yolo_search_state['back_attempts']}) → GRASP"
                    )
                    self.publish_car_control("STOP")
                    break
                else:
                    yolo_search_state["phase"] = "forward"
                    yolo_search_state["forward_until"] = (
                        time.monotonic() + approach_yolo_explore_forward_sec
                    )
                    yolo_search_state["accumulated_rad"] = 0.0
                    yolo_search_state["last_yaw"] = None
                    if not yolo_search_state["logged_search"]:
                        self.get_logger().info(
                            "[approach/yolo_lost] target lost (confirmed) → search loop: "
                            f"forward {approach_yolo_explore_forward_sec:.1f}s, "
                            f"then turn right {approach_yolo_search_turn_deg:.0f}° "
                            "(repeat until reacquired)"
                        )
                        yolo_search_state["logged_search"] = True

            # 從 hold data 或實時資料決定本幀的 detected/dist/raw_dx_fresh
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
                    turn_stuck_boost = 1.15

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
            _last_wp_amcl_xy = self._record_outbound_waypoint(
                amcl_pose, self.get_latest_odom(), outbound_waypoints, _last_wp_amcl_xy
            )
            progress_yaw_rad = 0.10 if yolo_search_confirmed else None
            self._update_motion_progress(
                motion_progress,
                amcl_pose,
                motion_stuck_min_progress_m,
                min_progress_yaw_rad=progress_yaw_rad,
            )
            if yolo_search_confirmed and yolo_search_state["phase"] == "forward":
                if time.monotonic() >= yolo_search_state["forward_until"]:
                    yolo_search_state["phase"] = "turn_right"
                    yolo_search_state["accumulated_rad"] = 0.0
                    yolo_search_state["last_yaw"] = None
                    self.get_logger().info(
                        "[approach/yolo_lost] forward finished, no target → "
                        f"turn right {approach_yolo_search_turn_deg:.0f}°"
                    )
            elif yolo_search_confirmed and yolo_search_state["phase"] == "turn_right":
                turned_rad = self._track_yolo_search_yaw(
                    yolo_search_state, amcl_pose
                )
                if turned_rad >= approach_yolo_search_turn_rad:
                    yolo_search_state["phase"] = "forward"
                    yolo_search_state["forward_until"] = (
                        time.monotonic() + approach_yolo_explore_forward_sec
                    )
                    yolo_search_state["accumulated_rad"] = 0.0
                    yolo_search_state["last_yaw"] = None
                    self.get_logger().info(
                        "[approach/yolo_lost] turn finished, still no target → "
                        f"forward {approach_yolo_explore_forward_sec:.1f}s again"
                    )

            d_eff = last_valid_dist if last_valid_dist is not None else 9.9
            obs_scale_floor = 1.0

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
                # 卡住時若上次有效距離在夾取範圍內，直接夾取而非後退搜尋
                if (
                    last_valid_dist is not None
                    and last_valid_dist <= grasp_commit_dist_m
                ):
                    aligned = True
                    self.get_logger().info(
                        f"[approach] Stuck near grasp zone "
                        f"(last_dist={last_valid_dist:.2f}m) → GRASP instead of backing up"
                    )
                    self.publish_car_control("STOP")
                    break
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

            # ── 觸發：depth=-1（偵測到但太近，< 40cm，感測器失效）→ 直接夾 ──
            if target_live and live_dist < 0.0:
                aligned = True
                self.get_logger().info(
                    "[approach] Bear detected but depth=-1 (too close for sensor) → GRASP"
                )
                self.publish_car_control("STOP")
                break

            # ── 觸發：bbox 夠大（熊填滿畫面，深度不可靠時的備用判斷）──
            if grasp_bbox_px > 0 and target_live and bbox_w_px >= grasp_bbox_px:
                aligned = True
                self.get_logger().info(
                    f"[approach] bbox_w={bbox_w_px:.0f}px >= {grasp_bbox_px:.0f}px → GRASP"
                )
                self.publish_car_control("STOP")
                break

            # ── 連續幀確認：dist 持續在 grasp zone 才觸發（過濾深度突波）──
            if use_approach and dist > 0.0 and dist <= zone_slow + 0.12:
                grasp_close_frames += 1
            else:
                grasp_close_frames = 0

            if grasp_close_frames >= grasp_confirm_frames:
                aligned = True
                self.get_logger().info(
                    f"[approach] dist={dist:.2f}m in grasp zone for "
                    f"{grasp_close_frames} frames → GRASP"
                )
                self.publish_car_control("STOP")
                break

            # ── 觸發：深度失效／丟失但已很近（避免衝過頭才看到 -1）──
            # 閾值 zone_slow * 1.45 ≈ 0.94m：剛好覆蓋爪子太近相機失去熊的距離段
            if (
                last_valid_dist is not None
                and last_valid_dist <= zone_slow * 1.45
                and (not target_live or live_dist <= 0.0)
                and lost_frames >= 1
            ):
                aligned = True
                self.get_logger().info(
                    f"[approach] Target lost/invalid near grasp "
                    f"(last_dist={last_valid_dist:.2f}m, lost={lost_frames}) → brake → GRASP"
                )
                self.publish_car_control("STOP")
                break

            # ── 觸發：熊靠太近消失（最後距離 ≤ zone_slow*1.4，失 N 幀）──
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
                self.publish_car_control("STOP")
                break

            # ── 觸發：進入 grasp zone，且畫面已置中 → 主動煞車 ──
            grasp_center_thresh = align_px * 1.5
            if use_approach and dist > 0.0 and dist <= zone_slow + 0.12:
                if abs(raw_dx) <= grasp_center_thresh:
                    self.get_logger().info(
                        f"[approach] Grasp zone reached & centered: "
                        f"dist={dist:.2f}m, dx={raw_dx:.0f}px → brake → GRASP"
                    )
                    self.publish_car_control("STOP")
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
                    last_status_log = self._log_status_if_due(
                        last_status_log,
                        status_log_interval_sec,
                        (
                            f"夾取準備｜最後對準｜"
                            f"{self._turn_dir_cn(raw_dx_fresh, visual_servo_yaw_deadband_px)}｜"
                            f"距離 {dist:.2f}m（偏差 {raw_dx_fresh:+.0f}px）"
                        ),
                    )
                    time.sleep(approach_dt)
                    continue

            # ── log（加快到 0.35 秒一次）──
            now_t = time.monotonic()
            if mission_verbose_log and use_approach and now_t - last_approach_log >= 0.35:
                d_eff_log = last_valid_dist if last_valid_dist is not None else dist
                if d_eff_log is not None:
                    margin = max(0.0, d_eff_log - approach_stop_dist_m)
                    brake_need = self._braking_distance_m(
                        closure_speed_mps,
                        approach_decel_mps2,
                        approach_brake_safety_m,
                    )
                    v_allow = self._max_allowable_speed_mps(
                        margin, approach_decel_mps2
                    )
                    zone_name = (
                        "FAR" if d_eff_log > zone_far
                        else "MID" if d_eff_log > zone_mid
                        else "SLOW"
                    )
                    self.get_logger().info(
                        f"[approach/{zone_name}] dist={d_eff_log:.2f}m, dx={raw_dx:.0f}px, "
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
            approach_depth_hint = live_dist if target_live else last_valid_dist
            approach_control_dx = None

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
                    emit_detail=mission_verbose_log,
                )
                if yolo_search_state["phase"] == "back_peek":
                    if time.monotonic() < yolo_search_state["back_until"]:
                        self.publish_car_control("BACKWARD_SLOW")
                    else:
                        self.publish_car_control("STOP")
                        yolo_search_state["back_attempts"] += 1
                        yolo_search_state["phase"] = "idle"
                        yolo_search_state["logged_back"] = False
                        self.get_logger().info(
                            "[approach/yolo_lost] back peek finished "
                            f"(attempts={yolo_search_state['back_attempts']}/"
                            f"{approach_yolo_lost_back_max_attempts}), "
                            f"last_dist={last_valid_dist if last_valid_dist is not None else 'n/a'}m"
                        )
                        if self._should_memory_grasp_commit(
                            target_live=target_live,
                            last_valid_dist=last_valid_dist,
                            lost_frames=lost_frames,
                            grasp_commit_dist_m=grasp_commit_dist_m,
                            min_lost_frames=1,
                            back_enabled=approach_yolo_lost_back_enabled,
                            back_attempts=yolo_search_state["back_attempts"],
                            back_max_attempts=approach_yolo_lost_back_max_attempts,
                        ):
                            aligned = True
                            self.get_logger().info(
                                f"[approach] Back peek failed — memory grasp at "
                                f"{last_valid_dist:.2f}m → GRASP"
                            )
                            break
                elif self._should_memory_grasp_commit(
                    target_live=target_live,
                    last_valid_dist=last_valid_dist,
                    lost_frames=lost_frames,
                    grasp_commit_dist_m=grasp_commit_dist_m,
                    min_lost_frames=1,
                    back_enabled=approach_yolo_lost_back_enabled,
                    back_attempts=yolo_search_state["back_attempts"],
                    back_max_attempts=approach_yolo_lost_back_max_attempts,
                ):
                    aligned = True
                    self.get_logger().info(
                        f"[approach] Memory grasp at {last_valid_dist:.2f}m → GRASP"
                    )
                    self.publish_car_control("STOP")
                    break
                elif yolo_search_state["phase"] == "forward":
                    explore_cmd = [approach_yolo_explore_forward_speed] * 4
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
                elif yolo_search_state["phase"] == "turn_right":
                    _, cw = self._spin_action_names(approach_yolo_search_spin_speed_tier)
                    turn_action = self._normalize_spin_action_tier(
                        cw, approach_yolo_search_spin_speed_tier
                    )
                    if obs_search.block_cmd == "STOP" and obs_search.speed_scale <= 0.05:
                        self.publish_car_control("STOP")
                    else:
                        self.publish_car_control(turn_action)
                else:
                    self.publish_car_control("STOP")
                now_lost_log = time.monotonic()
                if mission_verbose_log and now_lost_log - last_yolo_lost_motion_log >= 0.5:
                    acc_deg = math.degrees(
                        float(yolo_search_state.get("accumulated_rad", 0.0))
                    )
                    if yolo_search_state["phase"] == "forward":
                        remain = max(
                            0.0,
                            float(yolo_search_state.get("forward_until", 0.0))
                            - now_lost_log,
                        )
                        motion_label = f"forward ({remain:.1f}s left)"
                    elif yolo_search_state["phase"] == "turn_right":
                        motion_label = f"turn_right ({acc_deg:.0f}°/{approach_yolo_search_turn_deg:.0f}°)"
                    else:
                        motion_label = yolo_search_state["phase"]
                    reason = (
                        f"lost_frames={lost_frames} last_dist="
                        f"{last_valid_dist:.2f}m" if last_valid_dist else "lost_frames={lost_frames}"
                    )
                    self.get_logger().info(
                        f"[approach/action] YOLO_LOST phase={yolo_search_state['phase']} "
                        f"action={motion_label} search_yaw={acc_deg:.0f}° "
                        f"reason={reason} "
                        f"scale={obs_search.speed_scale:.2f} "
                        f"block={obs_search.block_cmd or 'none'}"
                    )
                    last_yolo_lost_motion_log = now_lost_log
            else:
                fwd_wheel = visual_servo_max_forward_speed_far
                center_first = (
                    use_approach
                    and dist > 0.0
                    and dist <= 1.05
                    and abs(raw_dx_fresh) <= vs_center_deadband_px * 1.4
                )
                yaw_cap = (
                    visual_servo_max_yaw_near
                    if center_first
                    else visual_servo_max_yaw_far
                ) * turn_stuck_boost
                nav_yolo_ti = (
                    list(approach_ti)
                    if use_approach and approach_ti is not None
                    else None
                )
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
                    dx_ema_alpha=visual_servo_dx_ema_alpha,
                    depth_ema_alpha=visual_servo_depth_ema_alpha,
                    near_grasp_slowdown_enabled=False,
                    yolo_target_info=nav_yolo_ti,
                )
                control_dx = nav.get_filtered_dx_px()
                if control_dx is None:
                    control_dx = raw_dx_fresh - align_bias_px
                approach_control_dx = control_dx
                turn_dir = (
                    "←turn" if raw_dx < -visual_servo_yaw_deadband_px
                    else "→turn" if raw_dx > visual_servo_yaw_deadband_px
                    else "straight"
                )
                zone_name = self._approach_zone_name(d, zone_far, zone_mid)
                if mission_verbose_log:
                    self.get_logger().info(
                        f"[approach/action] {zone_name} {turn_dir} "
                        f"dist={d:.2f}m dx={raw_dx:.0f}px fwd={fwd_wheel:.0f} "
                        f"yaw_cap={yaw_cap:.0f} center_first={center_first} "
                        f"target={'live' if target_live else 'held' if held_ti else 'lost'}"
                    )
                if obstacle_guard_enabled:
                    yaw_obs, fwd_obs, front_blk, lm, rm, bm, fm = (
                        self._side_obs_yaw_correction(dp, obstacle_guard)
                    )
                    if abs(yaw_obs) > 2.0:
                        wheel_cmd = [
                            wheel_cmd[0] + yaw_obs, wheel_cmd[1] - yaw_obs,
                            wheel_cmd[2] + yaw_obs, wheel_cmd[3] - yaw_obs,
                        ]
                    if fwd_obs > 1.0:
                        wheel_cmd = [w + fwd_obs for w in wheel_cmd]
                    if front_blk > 0.05:
                        scale = max(0.0, 1.0 - front_blk)
                        wheel_cmd = [w * scale for w in wheel_cmd]
                    now_obs = time.monotonic()
                    if now_obs - last_obstacle_log >= obstacle_log_interval:
                        lm_s = f"{lm:.2f}m" if lm is not None else "n/a"
                        rm_s = f"{rm:.2f}m" if rm is not None else "n/a"
                        bm_s = f"{bm:.2f}m" if bm is not None else "n/a"
                        fm_s = f"{fm:.2f}m" if fm is not None else "n/a"
                        active = abs(yaw_obs) > 2.0 or fwd_obs > 1.0 or front_blk > 0.05
                        if active:
                            self.get_logger().info(
                                f"[obs] ⚠ yaw={yaw_obs:+.1f} fwd={fwd_obs:.0f} "
                                f"fblk={front_blk:.2f} "
                                f"L={lm_s} R={rm_s} B={bm_s} F={fm_s}"
                            )
                        elif obstacle_source_debug_enabled:
                            self.get_logger().info(
                                f"[obs] clear L={lm_s} R={rm_s} B={bm_s} F={fm_s}"
                            )
                        last_obstacle_log = now_obs
                self.publish_raw_car_control(wheel_cmd)

            status_obs = None
            if obstacle_guard_enabled:
                status_obs = obs_search if not use_approach else obs_for_stuck
            status_control_dx = (
                approach_control_dx
                if approach_control_dx is not None
                else (raw_dx_fresh - align_bias_px if use_approach else raw_dx)
            )
            last_status_log = self._log_status_if_due(
                last_status_log,
                status_log_interval_sec,
                self._build_approach_status_cn(
                    use_approach=use_approach,
                    yolo_search_confirmed=yolo_search_confirmed,
                    in_hold=in_hold,
                    target_live=target_live,
                    yolo_search_state=yolo_search_state,
                    dist=dist,
                    last_valid_dist=last_valid_dist,
                    control_dx=status_control_dx,
                    yolo_dx=(raw_dx_fresh if use_approach else None),
                    deadband_px=visual_servo_yaw_deadband_px,
                    zone_far=zone_far,
                    zone_mid=zone_mid,
                    obs=status_obs,
                ),
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

        # ── 最後補進：依熊最後被看到的距離與方向，補足剩餘間距後再夾 ──
        if grasp_final_nudge_enabled and last_valid_dist is not None:
            gap_m = max(0.08, min(0.30, last_valid_dist - approach_stop_dist_m))
            # 先修正朝向（用最後鎖定的水平偏差）
            if locked_dx is not None and abs(locked_dx) > visual_servo_yaw_deadband_px:
                yaw_w = nav.compute_yaw_wheel_from_pixel(
                    locked_dx,
                    max_yaw_wheel=visual_servo_max_yaw_near * 0.55,
                    deadband_px=visual_servo_yaw_deadband_px,
                    soft_scale_px=visual_servo_yaw_soft_scale_px,
                    dt=approach_dt,
                )
                self.publish_raw_car_control([yaw_w, -yaw_w, yaw_w, -yaw_w])
                time.sleep(0.20)
                self.publish_car_control("STOP")
                time.sleep(0.12)
            # wheel_speed 130 ≈ 0.15 m/s（保守估算）
            nudge_sec = max(
                grasp_final_nudge_min_sec,
                min(grasp_final_nudge_max_sec, gap_m / 0.15),
            )
            self.get_logger().info(
                f"[grasp prep] Memory grasp: last seen {last_valid_dist:.2f}m — "
                f"nudge forward {gap_m:.2f}m ({nudge_sec:.1f}s, "
                f"speed={grasp_final_nudge_wheel_speed:.0f}) → close gripper"
            )
            self.publish_raw_car_control([grasp_final_nudge_wheel_speed] * 4)
            time.sleep(nudge_sec)
            self.publish_car_control("STOP")
            time.sleep(0.35)

        if not use_unity:
            self.get_logger().info("[grasp prep] Real robot: skip marker wait, grasp now.")
        else:
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

        # ── 回程：有 AMCL home + Nav2 時走 map 導航，否則 fallback odom ──
        if home_pose is None or not _NAV2_AVAILABLE:
            if home_pose is not None and not _NAV2_AVAILABLE:
                self.get_logger().warn(
                    "nav2_msgs 未安裝，無法使用 NavigateToPose，改試 odom 回程。"
                )
            if home_odom is not None:
                self.get_logger().info(
                    "開始 odom-based 回起點導航（面向起點前進，到站放下後對正起始朝向）…"
                )
                odom_arrived = self._navigate_home_odom(
                    home_odom,
                    timeout=90.0,
                    arrive_thresh=home_reached_thresh,
                )
                if not odom_arrived:
                    self.get_logger().warn("Odom 回程未到達起點附近。")
                self._finish_at_home(
                    arm=arm,
                    home_pose=home_odom,
                    drop_at_home=drop_at_home,
                    arrival_thresh_m=home_reached_thresh,
                    align_after_drop=align_home_heading_after_drop,
                    heading_thresh_deg=fallback_heading_thresh_deg,
                    rotate_step_sec=fallback_rotate_step_sec,
                    align_timeout_sec=home_heading_align_timeout_sec,
                    use_odom=True,
                )
            else:
                self.get_logger().info("home_pose 未記錄且無 odom home，跳過回起點。任務完成。")
            return

        self.get_logger().info("開始 Nav2 + AMCL map 回程 …")
        d_before = self._home_dist_from_amcl(home_pose)
        if d_before is not None:
            self.get_logger().info(
                f"Navigating home … (current→home distance={d_before:.3f}m)"
            )
        else:
            self.get_logger().info("Navigating home … (AMCL pose unavailable)")

        if d_before is not None and d_before <= home_reached_thresh:
            self.get_logger().info(
                f"Already at home ({d_before:.3f}m <= {home_reached_thresh:.3f}m) "
                "— skip Nav2, release directly."
            )
            self._finish_at_home(
                arm=arm,
                home_pose=home_pose,
                drop_at_home=drop_at_home,
                arrival_thresh_m=home_reached_thresh,
                align_after_drop=align_home_heading_after_drop,
                heading_thresh_deg=fallback_heading_thresh_deg,
                rotate_step_sec=fallback_rotate_step_sec,
                align_timeout_sec=home_heading_align_timeout_sec,
                use_odom=False,
                nav_succeeded=True,
                nav_status=GoalStatus.STATUS_SUCCEEDED,
                nav_status_text=self._GOAL_STATUS_TEXT[GoalStatus.STATUS_SUCCEEDED],
            )
            return

        bt_active = self._is_bt_navigator_active()
        if bt_active is False:
            self._log_nav2_inactive_help()
            if fallback_home_enabled:
                self.get_logger().warn(
                    "[nav] bt_navigator inactive — fallback to AMCL drive home."
                )
                fallback_ok = self._fallback_drive_home(
                    home_pose=home_pose,
                    timeout_sec=fallback_home_timeout_sec,
                    arrival_xy_thresh_m=fallback_arrival_xy_thresh_m,
                    heading_thresh_deg=fallback_heading_thresh_deg,
                    rotate_step_sec=fallback_rotate_step_sec,
                    forward_step_sec=fallback_forward_step_sec,
                )
                self._finish_at_home(
                    arm=arm,
                    home_pose=home_pose,
                    drop_at_home=drop_at_home,
                    arrival_thresh_m=home_reached_thresh,
                    align_after_drop=align_home_heading_after_drop,
                    heading_thresh_deg=fallback_heading_thresh_deg,
                    rotate_step_sec=fallback_rotate_step_sec,
                    align_timeout_sec=home_heading_align_timeout_sec,
                    use_odom=False,
                    nav_succeeded=fallback_ok,
                    nav_status=(
                        GoalStatus.STATUS_SUCCEEDED
                        if fallback_ok
                        else GoalStatus.STATUS_ABORTED
                    ),
                    nav_status_text=(
                        self._GOAL_STATUS_TEXT[GoalStatus.STATUS_SUCCEEDED]
                        if fallback_ok
                        else self._GOAL_STATUS_TEXT[GoalStatus.STATUS_ABORTED]
                    ),
                )
                return

        client: ActionClient = self.navigate_to_pose_action_client
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return

        nav_succeeded = False
        status = GoalStatus.STATUS_UNKNOWN
        for attempt in range(nav_retry_count + 1):
            nav_start_pose = self.get_latest_amcl_pose()
            from_pose = self._geometry_pose_from_amcl_msg(nav_start_pose)
            if from_pose is None:
                self.get_logger().warn(
                    "[nav] No AMCL pose for approach heading — using recorded home orientation."
                )
                from_pose = copy.deepcopy(home_pose)
            goal_pose = self._make_nav_home_goal_pose(home_pose, from_pose)
            approach_yaw = self._yaw_from_quat(goal_pose.pose.orientation)
            home_yaw = self._yaw_from_quat(home_pose.orientation)
            self.get_logger().info(
                f"[nav] Attempt {attempt + 1}/{nav_retry_count + 1}: "
                f"NavigateToPose goal approach_yaw={math.degrees(approach_yaw):.1f}° "
                f"(face toward home), post-drop home_yaw={math.degrees(home_yaw):.1f}°"
            )
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal_pose
            send_future = client.send_goal_async(
                nav_goal,
                feedback_callback=lambda fb, _self=self, _period=nav_feedback_log_sec, _verbose=mission_verbose_log: _self._nav_feedback_cb(
                    fb, _period, verbose=_verbose
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
                    d_reject = self._home_dist_from_amcl(home_pose)
                    if (
                        d_reject is not None
                        and d_reject <= home_reached_thresh
                    ):
                        self.get_logger().warn(
                            f"[nav] Goal rejected but already at home "
                            f"({d_reject:.3f}m) — skip Nav2 retry/unstick, release."
                        )
                        nav_succeeded = True
                        status = GoalStatus.STATUS_SUCCEEDED
                        break
                    self._log_nav2_inactive_help()
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
                    last_nav_status_log = 0.0
                    obs_nav = None
                    nav_aborted_by_guard = False
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
                                emit_detail=mission_verbose_log,
                            )
                            front_blocked = obs_nav.front_clearance_m < nav_obstacle_stop_m
                            if front_blocked:
                                reason = (
                                    f"front={obs_nav.front_clearance_m:.2f}m < {nav_obstacle_stop_m:.2f}m"
                                )
                                self.get_logger().warn(
                                    f"[nav/obstacle] {reason} — cancel Nav2 goal."
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

                        last_nav_status_log = self._log_status_if_due(
                            last_nav_status_log,
                            status_log_interval_sec,
                            self._build_nav_status_cn(
                                attempt=attempt + 1,
                                attempt_total=nav_retry_count + 1,
                                home_dist=self._last_home_dist,
                                nav_feedback_dist=self._last_nav_feedback_distance,
                                obs=obs_nav,
                            ),
                        )

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

                    if result_future.done():
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

            d_retry = self._home_dist_from_amcl(home_pose)
            if d_retry is not None and d_retry <= home_reached_thresh:
                self.get_logger().warn(
                    f"[nav] Nav failed but already at home ({d_retry:.3f}m) "
                    "— skip retry/unstick, release."
                )
                nav_succeeded = True
                status = GoalStatus.STATUS_SUCCEEDED
                break

            if attempt < nav_retry_count:
                self.get_logger().warn(
                    "[nav] Attempt failed. Clearing costmaps and retrying once pose is updated."
                )
                self._clear_costmaps_once()
                if nav_unstick_enabled:
                    self._unstick_maneuver(nav_unstick_back_sec, nav_unstick_rotate_sec)
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
        need_final_approach = (
            fallback_home_enabled
            and nav_succeeded
            and d_after is not None
            and d_after > home_reached_thresh
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
            # 優先用去程 waypoints 回家（AMCL primary / odom fallback）
            if outbound_waypoints:
                self.get_logger().info(
                    f"[fallback] Trying waypoint return ({len(outbound_waypoints)} waypoints)."
                )
                fallback_ok = self._navigate_home_via_waypoints(
                    waypoints=outbound_waypoints,
                    home_pose=home_pose,
                    home_odom=home_odom,
                    arrival_thresh_m=fallback_arrival_xy_thresh_m,
                    rotate_step_sec=fallback_rotate_step_sec,
                    forward_step_sec=fallback_forward_step_sec,
                )
            else:
                self.get_logger().warn(
                    "[fallback] No waypoints recorded — fallback to direct AMCL drive."
                )
                fallback_ok = self._fallback_drive_home(
                    home_pose=home_pose,
                    timeout_sec=fallback_home_timeout_sec,
                    arrival_xy_thresh_m=fallback_arrival_xy_thresh_m,
                    heading_thresh_deg=fallback_heading_thresh_deg,
                    rotate_step_sec=fallback_rotate_step_sec,
                    forward_step_sec=fallback_forward_step_sec,
                )
            if fallback_ok:
                nav_succeeded = True
                status = GoalStatus.STATUS_SUCCEEDED
                self.get_logger().info("[fallback] Home fallback succeeded.")
            else:
                self.get_logger().warn("[fallback] Home fallback failed or timed out.")

        if need_final_approach:
            self.get_logger().warn(
                f"[nav] Nav2 SUCCEEDED but AMCL still {d_after:.3f}m from home "
                f"(threshold={home_reached_thresh:.3f}m) — final approach to release zone."
            )
            final_ok = self._fallback_drive_home(
                home_pose=home_pose,
                timeout_sec=min(fallback_home_timeout_sec, 15.0),
                arrival_xy_thresh_m=home_reached_thresh,
                heading_thresh_deg=fallback_heading_thresh_deg,
                rotate_step_sec=fallback_rotate_step_sec,
                forward_step_sec=fallback_forward_step_sec,
            )
            if final_ok:
                self.get_logger().info("[nav] Final approach reached release zone.")
            else:
                self.get_logger().warn(
                    "[nav] Final approach did not reach release zone; "
                    "will check distance again before release."
                )
            current_pose_msg = self.get_latest_amcl_pose()
            if current_pose_msg is not None:
                d_after = self._dist_xy(current_pose_msg.pose.pose, home_pose)

        current_pose_msg = self.get_latest_amcl_pose()
        if current_pose_msg is not None:
            d_after = self._dist_xy(current_pose_msg.pose.pose, home_pose)
            self.get_logger().info(f"After nav, current→home distance={d_after:.3f}m")
        else:
            d_after = None

        status_text = self._GOAL_STATUS_TEXT.get(status, f"UNKNOWN_CODE({status})")
        self._finish_at_home(
            arm=arm,
            home_pose=home_pose,
            drop_at_home=drop_at_home,
            arrival_thresh_m=home_reached_thresh,
            align_after_drop=align_home_heading_after_drop,
            heading_thresh_deg=fallback_heading_thresh_deg,
            rotate_step_sec=fallback_rotate_step_sec,
            align_timeout_sec=home_heading_align_timeout_sec,
            use_odom=False,
            nav_succeeded=nav_succeeded,
            nav_status=status,
            nav_status_text=status_text,
        )

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

    def _nav_feedback_cb(self, feedback_msg, log_period_sec: float, verbose: bool = False):
        now = time.monotonic()
        fb = feedback_msg.feedback
        dist_rem = getattr(fb, "distance_remaining", None)
        if dist_rem is not None:
            prev = self._last_nav_feedback_distance
            self._last_nav_feedback_distance = float(dist_rem)
            self._last_nav_feedback_time = now
            if prev is None or (prev - float(dist_rem)) >= self._nav_stuck_min_progress_m:
                self._last_nav_progress_time = now
        if not verbose:
            return
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

    def _unstick_maneuver(self, back_sec: float, rotate_sec: float):
        """
        Nav2 卡住時，先執行短暫脫困再重送 goal：
        1) 慢速後退
        2) 慢速左旋
        3) 慢速右旋
        """
        self.get_logger().warn(
            f"[nav] Unstick maneuver: back={back_sec:.2f}s, rotate_each={rotate_sec:.2f}s"
        )
        if back_sec > 0.0:
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
            self.publish_car_control("BACKWARD_SLOW")
            time.sleep(back_sec)
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

    @staticmethod
    def _geometry_pose_from_amcl_msg(msg):
        """PoseWithCovarianceStamped → geometry_msgs/Pose."""
        if msg is None:
            return None
        inner = msg.pose
        if hasattr(inner, "position"):
            return inner
        return inner.pose

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def _approach_yaw_to_home(from_pose, home_pose) -> float:
        dx = float(home_pose.position.x - from_pose.position.x)
        dy = float(home_pose.position.y - from_pose.position.y)
        if math.hypot(dx, dy) < 0.08:
            return BearMissionHost._yaw_from_quat(home_pose.orientation)
        return math.atan2(dy, dx)

    def _make_nav_home_goal_pose(self, home_pose, from_pose) -> PoseStamped:
        """Nav2 goal: home position + yaw facing toward home (not final home heading)."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = copy.deepcopy(home_pose.position)
        approach_yaw = self._approach_yaw_to_home(from_pose, home_pose)
        goal_pose.pose.orientation = self._yaw_to_quaternion(approach_yaw)
        return goal_pose

    def _dist_to_home_pose(self, pose, home_pose) -> float | None:
        if pose is None or home_pose is None:
            return None
        return self._dist_xy(pose, home_pose)

    def _align_to_home_yaw(
        self,
        home_pose,
        heading_thresh_deg: float,
        rotate_step_sec: float,
        timeout_sec: float,
        use_odom: bool = False,
    ) -> bool:
        """Rotate in place to match recorded home orientation after drop."""
        heading_thresh = math.radians(heading_thresh_deg)
        home_yaw = self._yaw_from_quat(home_pose.orientation)
        t0 = time.monotonic()
        last_log = 0.0
        self.get_logger().info(
            f"[home] 對正起始朝向 target={math.degrees(home_yaw):.1f}° …"
        )
        while rclpy.ok() and time.monotonic() - t0 < timeout_sec:
            if not rclpy.ok():
                break
            if use_odom:
                odom = self.get_latest_odom()
                if odom is None:
                    time.sleep(0.05)
                    continue
                curr_yaw = self._yaw_from_quat(odom.pose.pose.orientation)
            else:
                amcl_pose_msg = self.get_latest_amcl_pose()
                if amcl_pose_msg is None:
                    time.sleep(0.05)
                    continue
                curr_yaw = self._yaw_from_quat(amcl_pose_msg.pose.pose.orientation)

            yaw_err = self._normalize_angle(home_yaw - curr_yaw)
            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    f"[狀態] 回程完成｜對正起始朝向｜"
                    f"剩餘 {math.degrees(yaw_err):+.1f}°"
                )
                last_log = now

            if abs(yaw_err) <= heading_thresh:
                self.publish_car_control("STOP")
                self.get_logger().info(
                    f"[home] 起始朝向已對正（誤差 {math.degrees(yaw_err):+.1f}°）。"
                )
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
        self.get_logger().warn("[home] 對正起始朝向 timeout。")
        return False

    def _finish_at_home(
        self,
        arm,
        home_pose,
        drop_at_home: bool,
        arrival_thresh_m: float,
        align_after_drop: bool,
        heading_thresh_deg: float,
        rotate_step_sec: float,
        align_timeout_sec: float,
        use_odom: bool = False,
        nav_succeeded: bool | None = None,
        nav_status: int | None = None,
        nav_status_text: str | None = None,
    ) -> None:
        if use_odom:
            odom = self.get_latest_odom()
            current_pose = odom.pose.pose if odom is not None else None
        else:
            amcl_pose_msg = self.get_latest_amcl_pose()
            current_pose = (
                amcl_pose_msg.pose.pose if amcl_pose_msg is not None else None
            )

        d_after = self._dist_to_home_pose(current_pose, home_pose)
        if current_pose is not None and d_after is not None:
            self.get_logger().info(f"After nav, current→home distance={d_after:.3f}m")

        close_to_home = d_after is not None and d_after <= arrival_thresh_m
        if drop_at_home and close_to_home:
            self.get_logger().info("[狀態] 回程完成｜到達起點｜放下目標 …")
            self.get_logger().info("Releasing target at home …")
            released = arm.run_release_blocking()
            if not released:
                self.get_logger().warn("Release sequence failed or interrupted.")
        elif drop_at_home:
            status_part = ""
            if nav_status is not None:
                status_part = (
                    f", nav_succeeded={nav_succeeded}, "
                    f"nav_status={nav_status} [{nav_status_text}]"
                )
            self.get_logger().warn(
                "Skip release: robot is not close enough to home "
                f"(distance={d_after if d_after is not None else 'unknown'}m, "
                f"threshold={arrival_thresh_m:.3f}m{status_part})."
            )
            return
        elif not drop_at_home:
            self.get_logger().info("drop_at_home=false, skip release.")

        if align_after_drop and close_to_home:
            self._align_to_home_yaw(
                home_pose=home_pose,
                heading_thresh_deg=heading_thresh_deg,
                rotate_step_sec=rotate_step_sec,
                timeout_sec=align_timeout_sec,
                use_odom=use_odom,
            )
        elif align_after_drop:
            self.get_logger().warn(
                "Skip heading align: robot is not close enough to home."
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Startup move sequence
    # ──────────────────────────────────────────────────────────────────────────

    def _run_startup_sequence(
        self,
        moves_str: str,
        fwd_speed: float,
        rot_speed: float,
    ) -> None:
        """
        Execute a configurable startup movement sequence.

        moves_str format: comma-separated tokens, each "<DIR>:<value>"
          F:<m>    forward m metres
          B:<m>    backward m metres
          R:<deg>  rotate clockwise deg degrees in place
          L:<deg>  rotate counterclockwise deg degrees in place

        Example: "F:0.85,R:90,F:0.30"
        """
        steps = []
        for token in moves_str.split(","):
            token = token.strip()
            if not token:
                continue
            parts = token.split(":", 1)
            if len(parts) != 2:
                self.get_logger().warn(f"[startup] Bad token '{token}' — skip.")
                continue
            direction = parts[0].strip().upper()
            try:
                value = float(parts[1].strip())
            except ValueError:
                self.get_logger().warn(f"[startup] Bad value in '{token}' — skip.")
                continue
            if direction not in ("F", "B", "R", "L"):
                self.get_logger().warn(
                    f"[startup] Unknown direction '{direction}' (use F/B/R/L) — skip."
                )
                continue
            steps.append((direction, value))

        if not steps:
            self.get_logger().warn("[startup] No valid steps parsed, skip sequence.")
            return

        self.get_logger().info(
            f"[startup] Sequence ({len(steps)} steps): "
            + ", ".join(f"{d}:{v}" for d, v in steps)
        )

        for idx, (direction, value) in enumerate(steps):
            step_label = f"step {idx + 1}/{len(steps)} {direction}:{value}"
            odom = self.get_latest_odom()
            if odom is None:
                self.get_logger().warn(f"[startup] No odom for {step_label} — skip.")
                continue

            if direction in ("F", "B"):
                sx = odom.pose.pose.position.x
                sy = odom.pose.pose.position.y
                linear = fwd_speed if direction == "F" else -fwd_speed
                timeout = value / max(abs(fwd_speed), 0.05) * 2.5
                self.get_logger().info(
                    f"[startup] {step_label}: "
                    f"{'forward' if direction == 'F' else 'backward'} {value:.2f}m …"
                )
                t0 = time.monotonic()
                while rclpy.ok() and time.monotonic() - t0 < timeout:
                    odom = self.get_latest_odom()
                    if odom is not None:
                        dx = odom.pose.pose.position.x - sx
                        dy = odom.pose.pose.position.y - sy
                        if math.hypot(dx, dy) >= value:
                            self.get_logger().info(
                                f"[startup] {step_label} done ({math.hypot(dx, dy):.3f}m)."
                            )
                            break
                    self.publish_raw_car_control([linear, 0.0])
                    time.sleep(0.05)
                else:
                    self.get_logger().warn(f"[startup] {step_label} timeout.")

            else:  # R or L
                target_rad = math.radians(value)
                # R = clockwise = negative angular_z; L = counterclockwise = positive
                angular = rot_speed if direction == "L" else -rot_speed
                timeout = target_rad / max(rot_speed, 0.1) * 2.5
                self.get_logger().info(
                    f"[startup] {step_label}: "
                    f"rotate {'left' if direction == 'L' else 'right'} {value:.1f}° …"
                )
                accumulated = 0.0
                last_yaw = self._yaw_from_quat(odom.pose.pose.orientation)
                t0 = time.monotonic()
                while rclpy.ok() and time.monotonic() - t0 < timeout:
                    odom = self.get_latest_odom()
                    if odom is not None:
                        cur_yaw = self._yaw_from_quat(odom.pose.pose.orientation)
                        accumulated += abs(self._normalize_angle(cur_yaw - last_yaw))
                        last_yaw = cur_yaw
                        if accumulated >= target_rad:
                            self.get_logger().info(
                                f"[startup] {step_label} done "
                                f"({math.degrees(accumulated):.1f}°)."
                            )
                            break
                    self.publish_raw_car_control([0.0, angular])
                    time.sleep(0.05)
                else:
                    self.get_logger().warn(f"[startup] {step_label} timeout.")

            self.publish_raw_car_control([0.0, 0.0])
            time.sleep(0.1)  # brief pause between steps

    # ──────────────────────────────────────────────────────────────────────────
    # Waypoint-based return navigation
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _record_outbound_waypoint(
        amcl_msg,
        odom_msg,
        waypoints: list,
        last_xy: tuple | None,
        step_m: float = 0.1,
    ) -> tuple | None:
        """Record (amcl_x, amcl_y, odom_x, odom_y) every step_m. Returns updated last_xy."""
        if amcl_msg is None:
            return last_xy
        ax = amcl_msg.pose.pose.position.x
        ay = amcl_msg.pose.pose.position.y
        if last_xy is None or math.hypot(ax - last_xy[0], ay - last_xy[1]) >= step_m:
            ox = odom_msg.pose.pose.position.x if odom_msg is not None else 0.0
            oy = odom_msg.pose.pose.position.y if odom_msg is not None else 0.0
            waypoints.append((ax, ay, ox, oy))
            return (ax, ay)
        return last_xy

    def _navigate_home_via_waypoints(
        self,
        waypoints: list,
        home_pose,
        home_odom,
        arrival_thresh_m: float = 0.30,
        wp_thresh_m: float = 0.25,
        rotate_step_sec: float = 0.30,
        forward_step_sec: float = 0.30,
        amcl_cov_thresh: float = 0.25,
        amcl_stale_sec: float = 2.0,
        amcl_no_progress_sec: float = 8.0,
        per_wp_timeout_sec: float = 25.0,
    ) -> bool:
        """
        Navigate home through outbound waypoints in reverse.
        Primary: AMCL closed-loop.  Fallback: odom closed-loop.
        Returns True when within arrival_thresh_m of home.
        """
        import rclpy.time as rclpy_time

        if not waypoints:
            self.get_logger().warn("[wp_nav] No waypoints — skip.")
            return False

        targets = list(reversed(waypoints))
        self.get_logger().info(
            f"[wp_nav] Waypoint return: {len(targets)} waypoints, "
            f"every ~0.1m, AMCL primary / odom fallback."
        )

        amcl_ok = True

        for i, (ax, ay, ox, oy) in enumerate(targets):
            wp_label = f"{i + 1}/{len(targets)}"
            is_last = i == len(targets) - 1
            goal_thresh = arrival_thresh_m if is_last else wp_thresh_m
            t_wp = time.monotonic()
            last_dist_to_wp = None
            last_progress_t = time.monotonic()

            while rclpy.ok() and time.monotonic() - t_wp < per_wp_timeout_sec:
                amcl_msg = self.get_latest_amcl_pose()

                if amcl_msg is None:
                    amcl_ok = False
                    self.get_logger().warn("[wp_nav] AMCL unavailable → odom fallback.")
                    break

                if amcl_ok:
                    # Quality checks
                    cov = amcl_msg.pose.covariance
                    xy_var = max(cov[0], cov[7])
                    age = (
                        self.get_clock().now()
                        - rclpy_time.Time.from_msg(amcl_msg.header.stamp)
                    ).nanoseconds * 1e-9
                    if xy_var > amcl_cov_thresh or age > amcl_stale_sec:
                        self.get_logger().warn(
                            f"[wp_nav] AMCL quality low (cov={xy_var:.3f}, age={age:.1f}s) "
                            "→ odom fallback."
                        )
                        amcl_ok = False
                        break

                cx = amcl_msg.pose.pose.position.x
                cy = amcl_msg.pose.pose.position.y

                # Check if already at home (skip remaining waypoints)
                d_home = self._dist_xy(amcl_msg.pose.pose, home_pose)
                if d_home <= arrival_thresh_m:
                    self.publish_car_control("STOP")
                    self.get_logger().info(
                        f"[wp_nav] Home reached ({d_home:.3f}m) at wp {wp_label}."
                    )
                    return True

                # Progress check toward current waypoint
                d_wp = math.hypot(cx - ax, cy - ay)
                if last_dist_to_wp is None or (last_dist_to_wp - d_wp) >= 0.05:
                    last_dist_to_wp = d_wp
                    last_progress_t = time.monotonic()
                elif time.monotonic() - last_progress_t > amcl_no_progress_sec:
                    self.get_logger().warn(
                        f"[wp_nav] No progress for {amcl_no_progress_sec:.0f}s "
                        f"at wp {wp_label} → odom fallback."
                    )
                    amcl_ok = False
                    break

                if d_wp <= goal_thresh:
                    self.publish_car_control("STOP")
                    break

                # Steer toward waypoint
                curr_yaw = self._yaw_from_quat(amcl_msg.pose.pose.orientation)
                heading_err = self._normalize_angle(
                    math.atan2(ay - cy, ax - cx) - curr_yaw
                )
                if abs(heading_err) > math.radians(20.0):
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

            if not amcl_ok:
                # Build remaining odom targets: current wp onwards + home
                remaining = [(ox2, oy2) for (_, _, ox2, oy2) in targets[i:]]
                if home_odom is not None:
                    remaining.append(
                        (home_odom.position.x, home_odom.position.y)
                    )
                self.get_logger().warn(
                    f"[wp_nav] Odom fallback: {len(remaining)} segments remaining."
                )
                return self._navigate_waypoints_odom(
                    remaining, arrival_thresh_m=arrival_thresh_m
                )

        # All waypoints visited — verify final position
        final_msg = self.get_latest_amcl_pose()
        if final_msg is not None:
            d = self._dist_xy(final_msg.pose.pose, home_pose)
            self.get_logger().info(f"[wp_nav] All waypoints done, dist_home={d:.3f}m.")
            return d <= arrival_thresh_m
        return False

    def _navigate_waypoints_odom(
        self,
        waypoints_xy: list,
        arrival_thresh_m: float = 0.30,
        per_wp_thresh_m: float = 0.22,
        per_wp_timeout_sec: float = 20.0,
    ) -> bool:
        """Navigate through a list of (x, y) odom positions in order."""
        for i, (tx, ty) in enumerate(waypoints_xy):
            is_last = i == len(waypoints_xy) - 1
            goal_thresh = arrival_thresh_m if is_last else per_wp_thresh_m
            t0 = time.monotonic()
            while rclpy.ok() and time.monotonic() - t0 < per_wp_timeout_sec:
                odom = self.get_latest_odom()
                if odom is None:
                    time.sleep(0.05)
                    continue
                cx = odom.pose.pose.position.x
                cy = odom.pose.pose.position.y
                dist = math.hypot(tx - cx, ty - cy)
                if dist <= goal_thresh:
                    self.publish_car_control("STOP")
                    break
                cur_yaw = self._yaw_from_quat(odom.pose.pose.orientation)
                yaw_err = self._normalize_angle(math.atan2(ty - cy, tx - cx) - cur_yaw)
                if abs(yaw_err) > 0.40:
                    self.publish_raw_car_control(
                        [0.0, 0.9 * math.copysign(1.0, yaw_err)]
                    )
                else:
                    self.publish_raw_car_control(
                        [min(0.20, dist * 0.5), 1.0 * yaw_err]
                    )
                time.sleep(0.05)
            self.publish_car_control("STOP")

        odom = self.get_latest_odom()
        if odom is None or not waypoints_xy:
            return False
        tx, ty = waypoints_xy[-1]
        return math.hypot(tx - odom.pose.pose.position.x, ty - odom.pose.pose.position.y) <= arrival_thresh_m

    def _fallback_drive_home(
        self,
        home_pose,
        timeout_sec: float,
        arrival_xy_thresh_m: float,
        heading_thresh_deg: float,
        rotate_step_sec: float,
        forward_step_sec: float,
    ) -> bool:
        """
        當 Nav2 異常時，用 AMCL 做簡單閉迴路回家：
        1) 面向 home 方向
        2) 前進直到進入 home 半徑
        放下與對正起始朝向由 _finish_at_home() 負責。
        """
        heading_thresh = math.radians(heading_thresh_deg)
        t0 = time.monotonic()
        last_log = 0.0
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
                    f"[fallback] dist={dist:.3f}m, yaw={math.degrees(curr_yaw):.1f}°"
                )
                last_log = now

            if dist <= arrival_xy_thresh_m:
                self.publish_car_control("STOP")
                self.get_logger().info(
                    f"[fallback] Arrived within {dist:.3f}m — "
                    "hand off to drop + heading align."
                )
                return True

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

    def _navigate_home_odom(
        self,
        home_pose,
        timeout=90.0,
        arrive_thresh=0.25,
        heading_thresh=0.25,
    ) -> bool:
        """
        Odom-based 回程：面向起點前進直到進入 arrival 半徑。
        放下與對正起始朝向由 _finish_at_home() 負責。
        """
        t_start = time.monotonic()
        home_x = float(home_pose.position.x)
        home_y = float(home_pose.position.y)

        self.get_logger().info(
            f"[nav_odom] 目標位置: x={home_x:.3f}, y={home_y:.3f} "
            f"(arrive_thresh={arrive_thresh:.2f}m)"
        )

        while time.monotonic() - t_start < timeout and rclpy.ok():
            odom = self.get_latest_odom()
            if odom is None:
                time.sleep(0.05)
                continue

            cur = odom.pose.pose
            cur_x = float(cur.position.x)
            cur_y = float(cur.position.y)
            cur_yaw = self._yaw_from_quat(cur.orientation)

            dx = home_x - cur_x
            dy = home_y - cur_y
            dist = math.hypot(dx, dy)

            if dist <= arrive_thresh:
                self.publish_car_control("STOP")
                self.get_logger().info(
                    f"[nav_odom] 到達起點附近 dist={dist:.2f}m — "
                    "hand off to drop + heading align."
                )
                return True

            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - cur_yaw)

            # 大偏差先原地旋轉，小偏差邊走邊修正
            if abs(yaw_err) > 0.45:
                angular_z = 0.9 * math.copysign(1.0, yaw_err)
                self.publish_raw_car_control([angular_z, -angular_z])
            else:
                linear_x = min(0.20, dist * 0.5)
                angular_z = 1.0 * yaw_err
                self.publish_raw_car_control([linear_x, angular_z])

            if int(time.monotonic() - t_start) % 3 == 0:
                self.get_logger().info(
                    f"[nav_odom] dist={dist:.2f}m, yaw_err={math.degrees(yaw_err):.1f}°"
                )

            time.sleep(0.05)

        self.publish_car_control("STOP")
        self.get_logger().warn("[nav_odom] 回程 timeout，停止。")
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
