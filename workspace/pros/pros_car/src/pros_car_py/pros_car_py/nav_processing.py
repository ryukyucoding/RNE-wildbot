from pros_car_py.nav2_utils import (
    get_yaw_from_quaternion,
    get_direction_vector,
    get_angle_to_target,
    calculate_angle_point,
    cal_distance,
)
import math
import time

from pros_car_py.obstacle_guard import ObstacleGuard, get_lidar_sector_minimums


class Nav2Processing:
    def __init__(self, ros_communicator, data_processor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        self.finishFlag = False
        self.global_plan_msg = None
        self.index = 0
        self.index_length = 0
        self.recordFlag = 0
        self.goal_published_flag = False
        self._vs_last_time = None
        self._vs_i_yaw = 0.0
        self._vs_i_depth = 0.0
        self._vs_prev_yaw_err = 0.0
        self._vs_prev_depth_err = 0.0
        self._vs_last_seen_time = 0.0
        self._vs_last_x_offset = None
        self._vs_last_depth = None
        self._vs_prev_dx_px = None

    def reset_nav_process(self):
        self.finishFlag = False
        self.recordFlag = 0
        self.goal_published_flag = False

    def finish_nav_process(self):
        self.finishFlag = True
        self.recordFlag = 1

    def get_finish_flag(self):
        return self.finishFlag

    def get_action_from_nav2_plan(self, goal_coordinates=None):
        if goal_coordinates is not None and not self.goal_published_flag:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
            self.goal_published_flag = True
        orientation_points, coordinates = (
            self.data_processor.get_processed_received_global_plan()
        )
        action_key = "STOP"
        if not orientation_points or not coordinates:
            action_key = "STOP"
        else:
            try:
                z, w = orientation_points[0]
                plan_yaw = get_yaw_from_quaternion(z, w)
                car_position, car_orientation = (
                    self.data_processor.get_processed_amcl_pose()
                )
                car_orientation_z, car_orientation_w = (
                    car_orientation[2],
                    car_orientation[3],
                )
                goal_position = self.ros_communicator.get_latest_goal()
                target_distance = cal_distance(car_position, goal_position)
                if target_distance < 0.5:
                    action_key = "STOP"
                    self.finishFlag = True
                else:
                    car_yaw = get_yaw_from_quaternion(
                        car_orientation_z, car_orientation_w
                    )
                    diff_angle = (plan_yaw - car_yaw) % 360.0
                    if diff_angle < 30.0 or (diff_angle > 330 and diff_angle < 360):
                        action_key = "FORWARD"
                    elif diff_angle > 30.0 and diff_angle < 180.0:
                        action_key = "COUNTERCLOCKWISE_ROTATION"
                    elif diff_angle > 180.0 and diff_angle < 330.0:
                        action_key = "CLOCKWISE_ROTATION"
                    else:
                        action_key = "STOP"
            except:
                action_key = "STOP"
        return action_key

    def get_action_from_nav2_plan_no_dynamic_p_2_p(self, goal_coordinates=None):
        if goal_coordinates is not None and not self.goal_published_flag:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
            self.goal_published_flag = True

        # 只抓第一次路径
        if self.recordFlag == 0:
            if not self.check_data_availability():
                return "STOP"
            else:
                print("Get first path")
                self.index = 0
                self.global_plan_msg = (
                    self.data_processor.get_processed_received_global_plan_no_dynamic()
                )
                self.recordFlag = 1
                action_key = "STOP"

        car_position, car_orientation = self.data_processor.get_processed_amcl_pose()

        goal_position = self.ros_communicator.get_latest_goal()
        target_distance = cal_distance(car_position, goal_position)

        # 抓最近的物標(可調距離)
        target_x, target_y = self.get_next_target_point(car_position)

        if target_x is None or target_distance < 0.5:
            self.ros_communicator.reset_nav2()
            self.finish_nav_process()
            return "STOP"

        # 計算角度誤差
        diff_angle = self.calculate_diff_angle(
            car_position, car_orientation, target_x, target_y
        )
        if diff_angle < 20 and diff_angle > -20:
            action_key = "FORWARD"
        elif diff_angle < -20 and diff_angle > -180:
            action_key = "CLOCKWISE_ROTATION"
        elif diff_angle > 20 and diff_angle < 180:
            action_key = "COUNTERCLOCKWISE_ROTATION"
        return action_key

    def check_data_availability(self):
        return (
            self.data_processor.get_processed_received_global_plan_no_dynamic()
            and self.data_processor.get_processed_amcl_pose()
            and self.ros_communicator.get_latest_goal()
        )

    def get_next_target_point(self, car_position, min_required_distance=0.5):
        """
        選擇距離車輛 min_required_distance 以上最短路徑然後返回 target_x, target_y
        """
        if self.global_plan_msg is None or self.global_plan_msg.poses is None:
            print("Error: global_plan_msg is None or poses is missing!")
            return None, None
        while self.index < len(self.global_plan_msg.poses) - 1:
            target_x = self.global_plan_msg.poses[self.index].pose.position.x
            target_y = self.global_plan_msg.poses[self.index].pose.position.y
            distance_to_target = cal_distance(car_position, (target_x, target_y))

            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                self.ros_communicator.publish_selected_target_marker(
                    x=target_x, y=target_y
                )
                return target_x, target_y

        return None, None

    def calculate_diff_angle(self, car_position, car_orientation, target_x, target_y):
        target_pos = [target_x, target_y]
        diff_angle = calculate_angle_point(
            car_orientation[2], car_orientation[3], car_position[:2], target_pos
        )
        return diff_angle

    def filter_negative_one(self, depth_list):
        return [depth for depth in depth_list if depth != -1.0]

    def camera_nav(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        """
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"

        camera_forward_depth = self.filter_negative_one(camera_multi_depth[7:13])
        camera_left_depth = self.filter_negative_one(camera_multi_depth[0:7])
        camera_right_depth = self.filter_negative_one(camera_multi_depth[13:20])

        action = "STOP"
        limit_distance = 0.7

        # if all(depth > limit_distance for depth in camera_forward_depth):
        if yolo_target_info[0] == 1:
            if yolo_target_info[2] > 200.0:
                action = "CLOCKWISE_ROTATION"
            elif yolo_target_info[2] < -200.0:
                action = "COUNTERCLOCKWISE_ROTATION"
            else:
                if yolo_target_info[1] < 0.5:
                    action = "STOP"
                else:
                    action = "FORWARD_SLOW"
        else:
            action = "CLOCKWISE_ROTATION"
        return action

    def apply_obstacle_guard_action(
        self,
        action: str,
        guard: ObstacleGuard,
        enabled: bool = True,
    ) -> str:
        """Override discrete nav action when LiDAR/depth sectors are too close."""
        if not enabled:
            return action
        obs = guard.evaluate(
            lidar_sectors=get_lidar_sector_minimums(self.data_processor),
            multi_depth=self.data_processor.get_camera_x_multi_depth(),
            sector_depth=self.data_processor.get_obstacle_sector_depth(),
        )
        if obs.block_cmd and obs.speed_scale <= 0.05:
            return obs.block_cmd
        if action in ("FORWARD_SLOW", "FORWARD") and obs.speed_scale < 0.35:
            return "STOP"
        return action

    def reset_visual_servo(self):
        self._vs_last_time = None
        self._vs_i_yaw = 0.0
        self._vs_i_depth = 0.0
        self._vs_prev_yaw_err = 0.0
        self._vs_prev_depth_err = 0.0
        self._vs_last_seen_time = 0.0
        self._vs_last_x_offset = None
        self._vs_last_depth = None
        self._vs_prev_dx_px = None

    def compute_yaw_wheel_from_pixel(
        self,
        dx_px: float,
        max_yaw_wheel: float,
        deadband_px: float = 28.0,
        soft_scale_px: float = 100.0,
        dt: float = 0.1,
        min_yaw_large_px: float = 100.0,
    ) -> float:
        """
        像素偏差 → 連續轉向輪速。
        - 小偏差：tanh 平滑，避免 overshoot
        - 大偏差：保底最小轉向力，避免原地卡住
        """
        scale = max(40.0, float(soft_scale_px))
        cap = float(max_yaw_wheel)

        # 軟死區：小偏差仍給弱轉向，避免 dx≈20px 時完全不修而持續偏一側
        if abs(dx_px) <= deadband_px:
            self._vs_prev_dx_px = dx_px
            if abs(dx_px) < 2.0:
                return 0.0
            ratio = abs(dx_px) / max(deadband_px, 1.0)
            yaw_soft = math.copysign(cap * 0.18 * (ratio**1.4), dx_px)
            return max(-cap * 0.22, min(cap * 0.22, yaw_soft))

        err = dx_px - math.copysign(deadband_px, dx_px)
        abs_err = abs(err)

        if abs_err >= 120.0:
            ratio = min(1.0, abs_err / (scale * 1.4))
            yaw_mag = max(cap * 0.85, ratio * cap * 0.98)
        elif abs_err >= 70.0:
            ratio = math.tanh(abs_err / scale)
            yaw_mag = ratio * cap * 1.12
        else:
            ratio = math.tanh(abs_err / max(scale * 0.75, 30.0))
            yaw_mag = ratio * cap * 0.95

        yaw_cmd = math.copysign(yaw_mag, err)

        # 大偏差保底：至少 min_yaw_large_px 等效輪速
        if abs_err >= 85.0:
            floor = min(cap, float(min_yaw_large_px))
            if abs(yaw_cmd) < floor:
                yaw_cmd = math.copysign(floor, err)
        if abs_err >= 180.0:
            strong_floor = min(cap, float(min_yaw_large_px) * 1.35)
            if abs(yaw_cmd) < strong_floor:
                yaw_cmd = math.copysign(strong_floor, err)

        # 僅在接近中心時做 overshoot 阻尼
        if self._vs_prev_dx_px is not None and dt > 1e-3 and abs(dx_px) < 70.0:
            dx_rate = (dx_px - self._vs_prev_dx_px) / dt
            if dx_px * dx_rate < 0.0:
                damp = max(0.35, 1.0 - abs(dx_rate) / 140.0)
                yaw_cmd *= damp

        self._vs_prev_dx_px = dx_px
        return max(-cap, min(cap, yaw_cmd))

    def camera_nav_pid_command(
        self,
        target_depth_m=0.42,
        search_spin_speed=70.0,
        max_forward_speed=170.0,
        max_forward_speed_far=300.0,
        far_distance_m=0.90,
        max_yaw_speed=300.0,
        lost_timeout_sec=0.8,
        center_deadband_px=45.0,
        image_half_width_px=320.0,
        large_turn_pixel_thresh=100.0,
        center_first=True,
        yaw_gain_per_px=0.0,
        yaw_soft_scale_px=120.0,
        yaw_deadband_px=12.0,
        min_yaw_large_px=100.0,
        pixel_offset_bias_px=0.0,
        yolo_target_info=None,
    ):
        """
        視覺伺服（近距離）：
        以 YOLO 的像素中心偏移 + 深度誤差做 PID，直接輸出輪速。
        回傳: [rear_left, rear_right, front_left, front_right]
        """
        now = time.monotonic()
        if self._vs_last_time is None:
            self._vs_last_time = now
        dt = max(0.02, min(0.2, now - self._vs_last_time))
        self._vs_last_time = now

        if yolo_target_info is None:
            yolo_target_info = self.data_processor.get_yolo_target_info()
        if yolo_target_info is None or len(yolo_target_info) < 3:
            # 沒資料時改搜尋自轉，避免輸出全零輪速
            spin = abs(float(search_spin_speed))
            return [-spin, spin, -spin, spin]

        detected = yolo_target_info[0] == 1.0
        depth = float(yolo_target_info[1])
        x_offset_px = float(yolo_target_info[2]) - float(pixel_offset_bias_px)

        # 目標短暫丟失：沿用上一幀誤差繼續走，避免多熊切換時原地空轉
        if not detected:
            hold_sec = max(float(lost_timeout_sec), 1.2)
            if (
                self._vs_last_x_offset is not None
                and (now - self._vs_last_seen_time) <= hold_sec
            ):
                detected = True
                x_offset_px = float(self._vs_last_x_offset)
                depth = (
                    float(self._vs_last_depth)
                    if self._vs_last_depth is not None
                    else depth
                )
            else:
                spin = abs(float(search_spin_speed))
                return [-spin, spin, -spin, spin]
        else:
            self._vs_last_seen_time = now
            self._vs_last_x_offset = x_offset_px
            if depth > 0.0:
                self._vs_last_depth = depth

        depth_valid = depth > 0.0
        far_mode = depth_valid and depth > float(far_distance_m)
        fwd_cap = (
            float(max_forward_speed_far) if far_mode else float(max_forward_speed)
        )
        yaw_cap = float(max_yaw_speed)

        # 連續視覺轉向：像素 → 輪速（取代 bang-bang 全速自轉）
        yaw_cmd = self.compute_yaw_wheel_from_pixel(
            x_offset_px,
            max_yaw_wheel=yaw_cap,
            deadband_px=yaw_deadband_px,
            soft_scale_px=yaw_soft_scale_px,
            dt=dt,
            min_yaw_large_px=min_yaw_large_px,
        )

        depth_err = (depth - target_depth_m) if depth_valid else 0.0
        dep_kp, dep_ki, dep_kd = 200.0, 8.0, 8.0

        if depth_valid:
            self._vs_i_depth = max(-1.0, min(1.0, self._vs_i_depth + depth_err * dt))
            dep_der = (depth_err - self._vs_prev_depth_err) / dt
            self._vs_prev_depth_err = depth_err
            forward_cmd = (
                dep_kp * depth_err + dep_ki * self._vs_i_depth + dep_kd * dep_der
            )
            if far_mode:
                if depth_err < 0.40:
                    forward_cmd = min(fwd_cap, forward_cmd)
                else:
                    forward_cmd = max(fwd_cap * 0.85, forward_cmd)
            else:
                forward_cmd = min(fwd_cap, forward_cmd)
            # 進入 grasp 前段：距離越近越強制降速（避免直衝停不住）
            if depth <= target_depth_m + 0.55:
                t = max(0.0, (depth - target_depth_m) / 0.55)
                forward_cmd = min(
                    forward_cmd,
                    fwd_cap * max(0.08, t ** 2.0),
                )
        else:
            forward_cmd = fwd_cap * 0.7 if far_mode else 0.0

        # 未置中時抑制前進；大偏差允許邊轉邊走（arc turn），避免原地卡死
        if (
            center_first
            and not far_mode
            and abs(x_offset_px) > float(center_deadband_px)
        ):
            if abs(x_offset_px) > 90.0:
                forward_cmd *= 0.45
            else:
                forward_cmd *= 0.18
        elif far_mode and abs(x_offset_px) > 100.0:
            # 大角度偏差時降低前進，讓轉向輪速主導（避免弧線往反側偏）
            misalign = min(1.0, abs(x_offset_px) / 260.0)
            forward_cmd *= max(0.30, 1.0 - 0.62 * misalign)

        forward_cmd = max(-fwd_cap, min(fwd_cap, forward_cmd))

        wheel_cap = max(fwd_cap, yaw_cap) * 1.15
        # 與 COUNTERCLOCKWISE_ROTATION [-v,+v] 一致：yaw<0 → 左轉，left=fwd+yaw, right=fwd-yaw
        left = max(-wheel_cap, min(wheel_cap, forward_cmd + yaw_cmd))
        right = max(-wheel_cap, min(wheel_cap, forward_cmd - yaw_cmd))

        return [left, right, left, right]

    def camera_nav_unity(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        """
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        yolo_target_info[1] *= 1
        camera_multi_depth = list(
            map(lambda x: x * 1.0, self.data_processor.get_camera_x_multi_depth())
        )

        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"

        camera_forward_depth = self.filter_negative_one(camera_multi_depth[7:13])
        camera_left_depth = self.filter_negative_one(camera_multi_depth[0:7])
        camera_right_depth = self.filter_negative_one(camera_multi_depth[13:20])
        action = "STOP"
        limit_distance = 10.0
        print(yolo_target_info[1])
        if all(depth > limit_distance for depth in camera_forward_depth):
            if yolo_target_info[0] == 1:
                if yolo_target_info[2] > 200.0:
                    action = "CLOCKWISE_ROTATION"
                elif yolo_target_info[2] < -200.0:
                    action = "COUNTERCLOCKWISE_ROTATION"
                else:
                    if yolo_target_info[1] < 2.0:
                        action = "STOP"
                    else:
                        action = "FORWARD_SLOW"
            else:
                action = "FORWARD"
        elif any(depth < limit_distance for depth in camera_left_depth):
            action = "CLOCKWISE_ROTATION"
        elif any(depth < limit_distance for depth in camera_right_depth):
            action = "COUNTERCLOCKWISE_ROTATION"
        return action

    def stop_nav(self):
        return "STOP"
