"""
door_open_task.py
=================
自動開門任務腳本（有限狀態機 FSM）

任務流程：
  State 0  NAVIGATE_TO_DOOR    —— 使用 Nav2 導航至門前預設座標
  State 1  SEARCH_HANDLE       —— 原地旋轉直到 YOLO 偵測到門把
  State 2  ALIGN_CAR           —— 車身水平對齊門把（pixel offset 置中）
  State 3  DRIVE_TO_DOOR       —— 車子前進到門前（深度感測器停車）
  State 4  ARM_AIM             —— 手臂（夾爪張開）移到門把正上方
  State 5  PRESS_DOWN          —— 夾爪合上門把 → 直接往下壓
  State 6  OPEN_DOOR           —— 車子後退推開門
  State 7  DONE                —— 完成，重置手臂回歸初始姿態

使用方式：
  1. 在 main2.py 或其他入口點建立此類別的物件
  2. 在你的控制迴圈中呼叫 door_open_task.step()
  3. 或直接使用 door_open_task.run_blocking() 阻塞執行到結束

ros2 run pros_car_py door_open  （需在 setup.py 加入 entry點）
"""

import time
import math
import threading
import rclpy

# ── 各狀態的數值參數（可依實際環境調整） ──────────────────────────────────────────

# State 0：導航目標點（門前約 0.5m，用 Foxglove 或 SLAM 實際量測後填入）
DOOR_APPROACH_GOAL = [2.5, 1.0]   # [x, y] in map frame，請改成實際座標

# 開發與測試專用：如果已經手動把車子開到門前，設為 True 可以直接跳過 Nav2 導航
SKIP_NAVIGATION = True

# YOLO 目標類別名稱 —— 對應你的模型訓練時使用的 class label
# 常見標記名稱："knob"、"door_handle"、"handle"，請依實際模型調整
YOLO_TARGET_LABEL = "knob"

# State 1：旋轉搜尋時的最大等待次數（每次 sleep 0.1s，共 N * 0.1 秒）
# 設為 600（60秒）以確保車子有足夠時間至少原地旋轉滿一整圈（360度）
SEARCH_MAX_ITER = 600             # 600 * 0.1s = 60 秒

# ── Visual Servoing (PID) 參數 ────────────────────────────────────────────────
# 基礎前進速度（rad/s，對應輪胎轉速）
# 為了讓車子有足夠時間把門把置中，降低前進速度
VS_BASE_SPEED = 60.0
# 轉向 PID 控制器參數：根據 pixel_offset 計算左右輪速差
VS_KP_STEER = 1.5           # 提高比例係數，讓車子更積極轉向對齊
VS_MAX_STEER = 250.0        # 最大轉速差限制（加大以對抗物理引擎摩擦力）
VS_MIN_STEER = 120.0        # 最小轉速差限制（克服原地旋轉時的靜摩擦力死區）

# ── 相機/手臂左右中心偏差補償 (像素) ───────────────────────────────────────────
# 如果當車子停在門前時，門把總是偏向手臂的右側，代表車子應該要再往右偏一些來對準。
# 可以把目標對齊像素設為「負值」（例如 -30.0 ~ -50.0），強迫車子在靠近時，將門把保持在影像的偏左側，使位於車身中線的手臂能夠往右偏對準門把！
# 反之，如果門把總是偏左，就設為「正值」。
VS_TARGET_PIXEL_OFFSET = 0.0

# State 2：YOLO 丟失時的耐心（秒）。丟失後維持目前速度，不直接停車。
VS_PATIENCE_SECS = 3.0      

# State 2：Visual Servoing 的停車條件
# 當 LiDAR 測量到門面小於等於這個距離，或者 YOLO depth 小於等於這個距離時停車
VS_STOP_DISTANCE = 0.30     # 0.30m 停車 (離門把更安全且剛好夠得到)

# ── 兩階段停車：精對齊 (Fine Alignment) 參數 ─────────────────────────────────
# 車子到達停車距離後，會先原地旋轉直到門把在畫面中心（誤差 < 容差），才進入手臂動作。
# 這樣不管從哪個角度靠近，都能確保手臂正對門把。
VS_ALIGN_PIXEL_TOL = 20.0    # 允許的像素誤差（±20px 內視為對齊）
VS_ALIGN_KP = 0.6            # 精對齊時的純旋轉 PID 比例係數（加大以確保能推動車子）
VS_ALIGN_TIMEOUT = 5.0       # 精對齊最長等待時間（秒），超時則直接進入手臂動作

# 深度 EMA 平滑係數（0~1，越小越平滑，越大越即時）
DEPTH_EMA_ALPHA = 0.3

# State 4：手臂可抵達的最大水平距離（保守估計，公尺）
# 實體手臂總長 ~19cm，設定略保守的 17cm
ARM_MAX_REACH = 0.17

# State 4：手臂先移到門把「正上方」的偏移量（公尺）
# 夾爪張開後，末端停在門把上方 5cm
PRESS_ABOVE_OFFSET = 0.05        # 5cm

# State 4 & 5：實車機構補償參數 (取代缺失的 TF 坐標系)
# 攝影機鏡頭到「手臂基座」的 X 軸距離補償 (公尺)
# 例如：攝影機安裝在手臂後方 15cm，則寫 -0.15。
# 這樣手臂距離門把 = depth + CAMERA_X_OFFSET
CAMERA_X_OFFSET = -0.15

# 門把相對於手臂基座的固定高度 (公尺)。請實際測量！
KNOB_Z_HEIGHT = 0.05

# State 5：夾爪合上後，向下壓的距離（公尺）
# 往下壓 7cm（door handle 行程通常 5~8cm）
PRESS_Z_DOWN = 0.07              # 7cm

# State 6：開門的車輛動作與時間
# 如果是要往外拉門，請填 "BACKWARD_SLOW"
# 如果是要往內推門，請填 "FORWARD_SLOW"
DOOR_OPEN_ACTION = "FORWARD_SLOW"
OPEN_DOOR_DURATION = 2.5         # 秒

# ─────────────────────────────────────────────────────────────────────────────


class DoorOpenState:
    """枚舉各狀態（重構為連續視覺伺服）"""
    NAVIGATE_TO_DOOR       = 0
    SEARCH_HANDLE          = 1
    VISUAL_SERVO_APPROACH  = 2   # ← 取代舊的 ALIGN_CAR 和 DRIVE_TO_DOOR
    ARM_AIM                = 4
    PRESS_DOWN             = 6
    OPEN_DOOR              = 7
    DONE                   = 8
    ERROR                  = 99


class DoorOpenTask:
    """
    完整的開門任務控制器。

    Args:
        car_controller  : CarController 實例
        arm_controller  : ArmController 實例
        nav_processing  : Nav2Processing 實例
        data_processor  : DataProcessor  實例
        ros_communicator: RosCommunicator 實例
    """

    def __init__(
        self,
        car_controller,
        arm_controller,
        nav_processing,
        data_processor,
        ros_communicator,
    ):
        self.car  = car_controller
        self.arm  = arm_controller
        self.nav  = nav_processing
        self.dp   = data_processor
        self.rc   = ros_communicator

        # 根據設定決定初始狀態
        if SKIP_NAVIGATION:
            self.state = DoorOpenState.SEARCH_HANDLE
        else:
            self.state = DoorOpenState.NAVIGATE_TO_DOOR

        self._iter        = 0          # 通用計數器
        self._press_count = 0          # State 5 已壓次數
        self._open_start  = None       # State 6 開始時間
        self._nav_started = False      # State 0 是否已啟動導航執行緒
        self._last_knob_x = 0.4        # 門把 X 座標備份
        self._last_knob_z = 0.0        # 門把 Z 座標備份
        self._last_knob_depth = None   # State 3 停車時的 YOLO 深度備份
        self._last_knob_map_pos = None # 最後一次偵測到 knob 的地圖座標 (x, y)
        self._depth_ema = None         # 深度 EMA 平滑值
        # 時間戳記型的丟失追蹤
        self._vs_lost_since = None
        self._fine_align_since = None     # State 2 精對齊階段的開始時間戳記
        self._last_vs_velocities = [0.0, 0.0, 0.0, 0.0]  # [RL, RR, FL, FR]

    # ──────────────────────────────────────────────────────────────────────────
    # 主要介面
    # ──────────────────────────────────────────────────────────────────────────

    def step(self):
        """
        單步執行 FSM（非阻塞）。
        在控制迴圈中每 ~100ms 呼叫一次即可。

        Returns:
            bool: True 表示任務完成（DONE 或 ERROR），False 表示仍在執行中。
        """
        if self.state == DoorOpenState.NAVIGATE_TO_DOOR:
            return self._state_navigate()
        elif self.state == DoorOpenState.SEARCH_HANDLE:
            return self._state_search()
        elif self.state == DoorOpenState.VISUAL_SERVO_APPROACH:
            return self._state_visual_servo_approach()
        elif self.state == DoorOpenState.ARM_AIM:
            return self._state_arm_aim()
        elif self.state == DoorOpenState.PRESS_DOWN:
            return self._state_press_down()
        elif self.state == DoorOpenState.OPEN_DOOR:
            return self._state_open_door()
        elif self.state == DoorOpenState.DONE:
            return True
        elif self.state == DoorOpenState.ERROR:
            print("[DoorOpenTask] 任務失敗，停止車子。")
            self.car.update_action("STOP")
            return True
        return False

    def run_blocking(self, spin_interval=0.1):
        """
        阻塞方式執行整個任務直到完成。
        通常在獨立執行緒中呼叫，或作為 ros2 run 的主函式。
        """
        print("[DoorOpenTask] 任務開始")

        # ── 任務起始：先把手臂舉到最高安全位置 ──────────────────────────────
        # 讓 Shoulder 盡量往上收（-180°），Elbow 打直（0°），夾爪張開（90°）
        # 為了避免 ROS 2 node 剛啟動時 publisher 遺失第一個封包，我們連發幾次
        print("[DoorOpenTask] 手臂舉到最高，準備執行任務…")
        try:
            self.arm.joint_angles = [-180.0, 0.0, 90.0]
            for _ in range(5):
                self.arm._clamp_and_publish()
                time.sleep(0.2)
        except Exception as e:
            print(f"[DoorOpenTask] 手臂初始化警告（忽略）: {e}")
        # ──────────────────────────────────────────────────────────────────────

        while not self.step():
            time.sleep(spin_interval)
        print("[DoorOpenTask] 任務結束，狀態:", self.state)

    # ──────────────────────────────────────────────────────────────────────────
    # State 0：使用 Nav2 導航至門前
    # ──────────────────────────────────────────────────────────────────────────

    def _state_navigate(self):
        if not self._nav_started:
            print(f"[State 0] 導航至門前 {DOOR_APPROACH_GOAL}")
            # 把目標座標發布給 Nav2，然後啟動 target_auto_nav 背景執行緒
            self.car.target_list = [DOOR_APPROACH_GOAL]
            self.car.target_idx  = 0
            self.nav.reset_nav_process()
            self.car.auto_control(mode="target_auto_nav")
            self._nav_started = True

        # 等待 Nav2 抵達目標
        if self.nav.get_finish_flag():
            print("[State 0] 已抵達門前，停車")
            # 停止導航執行緒
            if self.car._thread_running:
                self.car._stop_event.set()
                self.car._auto_nav_thread.join(timeout=2.0)
                self.car._thread_running = False
            self.car.update_action("STOP")
            self._transition(DoorOpenState.SEARCH_HANDLE)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # 工具：記錄 knob 在地圖上的世界座標（每次 YOLO 偵測到時呼叫）
    # ──────────────────────────────────────────────────────────────────────────

    def _save_knob_map_pos(self, depth):
        """用 AMCL 位姿 + YOLO 深度，估算 knob 在地圖上的 (x, y)。"""
        # 直接從 rc 檢查是否已收到 AMCL 資料，避免呼叫 getter 觸發大量的警告 Log
        if self.rc.latest_amcl_pose is None:
            return
        try:
            pose, quat = self.dp.get_processed_amcl_pose()
            robot_x, robot_y = pose[0], pose[1]
            # 從四元數取得 yaw（車子朝向）
            yaw = math.atan2(
                2 * (quat[3] * quat[2] + quat[0] * quat[1]),
                1 - 2 * (quat[1] ** 2 + quat[2] ** 2),
            )
            self._last_knob_map_pos = (
                robot_x + depth * math.cos(yaw),
                robot_y + depth * math.sin(yaw),
            )
        except Exception:
            pass   # AMCL 還沒就緒時忽略

    def _heading_to_last_knob(self):
        """根據上次記錄的 knob 地圖座標，回傳車子應轉向的動作指令。
        回傳 'CLOCKWISE_ROTATION_SLOW' / 'COUNTERCLOCKWISE_ROTATION_SLOW' / 'FORWARD_SLOW' / None。"""
        if self._last_knob_map_pos is None or self.rc.latest_amcl_pose is None:
            return None
        try:
            pose, quat = self.dp.get_processed_amcl_pose()
            robot_x, robot_y = pose[0], pose[1]
            yaw = math.atan2(
                2 * (quat[3] * quat[2] + quat[0] * quat[1]),
                1 - 2 * (quat[1] ** 2 + quat[2] ** 2),
            )
            target_x, target_y = self._last_knob_map_pos
            desired_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
            diff = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) < 0.3:   # 約 17° 以內視為已對齊
                return "FORWARD_SLOW"
            return "CLOCKWISE_ROTATION_SLOW" if diff > 0 else "COUNTERCLOCKWISE_ROTATION_SLOW"
        except Exception:
            return None


    # ──────────────────────────────────────────────────────────────────────────
    # State 1：原地旋轉搜尋門把
    # ──────────────────────────────────────────────────────────────────────────

    def _state_search(self):
        if self._iter == 0:
            print(f"[State 1] 開始搜尋門把，通知 YOLO 追蹤類別: '{YOLO_TARGET_LABEL}'")
            # 告訴 YOLO 節點現在要追蹤的物件類別（門把/knob）
            # YOLO 節點收到後，/yolo/target_info 才會回報該類別的偵測結果
            self.rc.publish_target_label(YOLO_TARGET_LABEL)
            time.sleep(0.3)  # 等 YOLO 節點處理完訂閱更新

        yolo = self.dp.get_yolo_target_info()

        if yolo is not None and yolo[0] == 1:
            # 偵測到目標
            print("[State 1] 偵測到門把！")
            self.car.update_action("STOP")
            self._transition(DoorOpenState.VISUAL_SERVO_APPROACH)
            return False

        # 緩慢順時針旋轉搜尋
        self.car.update_action("CLOCKWISE_ROTATION_SLOW")
        self._iter += 1

        if self._iter > SEARCH_MAX_ITER:
            print("[State 1] 搜尋逾時，任務失敗")
            self._transition(DoorOpenState.ERROR)

        time.sleep(0.05)  # 讓出 CPU，避免忙等待（run_blocking 的 spin_interval 控制主節奏）
        return False

    def _get_front_lidar_min(self):
        """獲取正前方 LiDAR 最小距離（避障與測距防備）"""
        try:
            lidar_msg = self.rc.latest_lidar
            if not lidar_msg: return None
            ranges = lidar_msg.ranges
            n = len(ranges)
            if n == 0: return None
            # 取正前方約 30 度角 (15度 ~ -15度)
            cone = max(1, int(n * (30.0 / 360.0) / 2))
            front = ranges[:cone] + ranges[-cone:]
            valid = [r for r in front if 0.1 < r < 4.0]  # 濾除 0 或過遠噪訊
            if valid:
                return min(valid)
        except Exception:
            pass
        return None

    # ──────────────────────────────────────────────────────────────────────────
    # State 2：Visual Servoing (PID) 閉迴路控制靠近
    # ──────────────────────────────────────────────────────────────────────────

    def _state_visual_servo_approach(self):
        if self._iter == 0:
            print(f"[State 2] 開始 Visual Servoing 靠近門把… 目標距離={VS_STOP_DISTANCE}m")
            self._last_vs_velocities = [0.0, 0.0, 0.0, 0.0]
            self._vs_lost_since = None
            self._fine_align_since = None   # 精對齊開始時間（None = 尚在靠近階段）

        yolo = self.dp.get_yolo_target_info()
        lidar_dist = self._get_front_lidar_min()

        # ── YOLO 遺失統一處理 ──
        if yolo is None or yolo[0] == 0:
            if self._vs_lost_since is None:
                self._vs_lost_since = time.time()
                print(f"[State 2] YOLO 暫時丟失，維持當前速度中 ({VS_PATIENCE_SECS}s 耐心)")
            lost_dur = time.time() - self._vs_lost_since
            if lost_dur < VS_PATIENCE_SECS:
                self.rc.publish_raw_car_control(self._last_vs_velocities)
                self._iter += 1
                return False
            print(f"[State 2] 目標持續丟失超過 {VS_PATIENCE_SECS}s，退回搜尋狀態")
            self._vs_lost_since = None
            self.car.update_action("STOP")
            self._transition(DoorOpenState.SEARCH_HANDLE)
            return False

        self._vs_lost_since = None
        pixel_offset = yolo[2]
        error = pixel_offset - VS_TARGET_PIXEL_OFFSET

        # ─────────────────────────────────────────────────────────────────
        # 精對齊階段：LiDAR 或 YOLO 深度已達停車距離，只做旋轉對齊
        # ─────────────────────────────────────────────────────────────────
        distance_reached = (lidar_dist and lidar_dist <= VS_STOP_DISTANCE)
        if yolo[1] > 0:
            d = yolo[1]
            self._depth_ema = d if self._depth_ema is None else (
                DEPTH_EMA_ALPHA * d + (1 - DEPTH_EMA_ALPHA) * self._depth_ema)
            if self._depth_ema <= VS_STOP_DISTANCE:
                distance_reached = True

        if distance_reached:
            # 第一次進入精對齊，記錄時間
            if self._fine_align_since is None:
                self._fine_align_since = time.time()
                self._last_knob_depth = lidar_dist if lidar_dist else self._depth_ema
                print(f"[State 2] 到達停車距離！開始精對齊 (誤差={error:.1f}px, 容差=±{VS_ALIGN_PIXEL_TOL}px)…")

            # 對齊完成或超時：進入手臂動作
            align_elapsed = time.time() - self._fine_align_since
            if abs(error) <= VS_ALIGN_PIXEL_TOL:
                print(f"[State 2] ✅ 精對齊完成！最終誤差={error:.1f}px，進入手臂動作")
                self.car.update_action("STOP")
                self._transition(DoorOpenState.ARM_AIM)
                return False
            if align_elapsed > VS_ALIGN_TIMEOUT:
                print(f"[State 2] ⚠️ 精對齊逾時 ({VS_ALIGN_TIMEOUT}s)，最終誤差={error:.1f}px，強制進入手臂動作")
                self.car.update_action("STOP")
                self._transition(DoorOpenState.ARM_AIM)
                return False

            # 純原地旋轉：不前進，只修正左右
            rotate_output = VS_ALIGN_KP * error
            if abs(rotate_output) < VS_MIN_STEER:
                rotate_output = VS_MIN_STEER if rotate_output > 0 else -VS_MIN_STEER
            rotate_output = max(-VS_MAX_STEER, min(VS_MAX_STEER, rotate_output))
            
            v_left  =  rotate_output
            v_right = -rotate_output
            velocities = [v_left, v_right, v_left, v_right]
            self.rc.publish_raw_car_control(velocities)
            self._last_vs_velocities = velocities

            if self._iter % 10 == 0:
                print(f"[Align] 誤差={error:.1f}px, 原地旋轉 L={v_left:.0f}, R={v_right:.0f} ({align_elapsed:.1f}s)")
            self._iter += 1
            return False

        # ─────────────────────────────────────────────────────────────────
        # 靠近階段：同時前進 + 轉向
        # ─────────────────────────────────────────────────────────────────
        steer_output = VS_KP_STEER * error
        if abs(error) > 5.0 and abs(steer_output) < VS_MIN_STEER:
            steer_output = VS_MIN_STEER if steer_output > 0 else -VS_MIN_STEER
        steer_output = max(-VS_MAX_STEER, min(VS_MAX_STEER, steer_output))

        v_left  = VS_BASE_SPEED + steer_output
        v_right = VS_BASE_SPEED - steer_output

        velocities = [v_left, v_right, v_left, v_right]
        self.rc.publish_raw_car_control(velocities)
        self._last_vs_velocities = velocities

        if self._iter % 20 == 0:
            print(f"[VS] offset={pixel_offset:.1f}px (target={VS_TARGET_PIXEL_OFFSET:.1f}), steer={steer_output:.1f}, L={v_left:.0f}, R={v_right:.0f} | LiDAR={lidar_dist if lidar_dist else 0:.2f}m")

        self._iter += 1
        return False


    # ──────────────────────────────────────────────────────────────────────────
    # State 4：手臂移到門把正上方，夾爪張開
    # ──────────────────────────────────────────────────────────────────────────

    def _state_arm_aim(self):
        if self._iter == 0:
            print("[State 4] 夾爪張開，準備移動手臂…")
            self.arm.set_last_joint_angle(70.0) # PyBullet 模擬器極限張開
            self._iter = 1
            time.sleep(0.3)

        # 車子在 State 2 已經因為 LiDAR 或 YOLO 深度達到 VS_STOP_DISTANCE (0.30m) 而精準停車。
        # 所以 arm_x = VS_STOP_DISTANCE + CAMERA_X_OFFSET = 0.30 + (-0.15) = 0.15m
        # 0.15m 完美落在 ARM_MAX_REACH (0.17m) 內，絕不超距，且關節角度更舒展！
        
        check_depth = self._last_knob_depth if self._last_knob_depth else VS_STOP_DISTANCE
        arm_x = check_depth + CAMERA_X_OFFSET
        
        print(f"[State 4] 手臂可達！門距離={check_depth:.3f}m, 手臂目標X={arm_x:.3f}m")

        x_target = arm_x
        z_target = KNOB_Z_HEIGHT
        x_above  = x_target
        z_above  = z_target + PRESS_ABOVE_OFFSET
        
        try:
            print("[State 4] 步驟1: 靠近車身並舉高 (由 PyBullet 規劃收縮姿態) 避免卡住門把")
            retract_pos = [0.05, 0.0, 0.15]
            self.arm.move_to_position(retract_pos)
            time.sleep(0.3)

            print(f"[State 4] 步驟2: 移到門把正上方 (X={x_above:.3f}m, Z={z_above:.3f}m)")
            above_pos = [x_above, 0.0, z_above]
            self.arm.move_to_position(above_pos)
            
            self._last_knob_x = x_target
            self._last_knob_z = z_target
            print("[State 4] 手臂已到達門把正上方，準備下壓")
            self._transition(DoorOpenState.PRESS_DOWN)
        except Exception as e:
            print(f"[State 4] 移動失敗: {e}")
            self._transition(DoorOpenState.ERROR)

        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 5（已移除）：ARM_APPROACH — 本策略不再從側面伸入，由 step() 跳過
    # ──────────────────────────────────────────────────────────────────────────

    def _state_arm_approach(self):
        # 策略已改為「正上方壓下」，此狀態不會被使用，直接跳到 PRESS_DOWN
        self._transition(DoorOpenState.PRESS_DOWN)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 6：夾爪合起 → 直接往下壓
    # ──────────────────────────────────────────────────────────────────────────

    def _state_press_down(self):
        if self._press_count == 0:
            print("[State 5] 夾爪合起，壓住門把…")
            self.arm.set_last_joint_angle(10.0)  # PyBullet 極限合攏
            time.sleep(0.1)
            self.arm.set_last_joint_angle(12.0)  # 退 2 度以防燒壞馬達
            time.sleep(0.3)                       # 等待微調完成

            print(f"[State 5] 往下壓 {PRESS_Z_DOWN * 100:.1f}cm，目標: X={self._last_knob_x:.3f}m, Z={self._last_knob_z - PRESS_Z_DOWN:.3f}m")

            try:
                # 直接一步壓到目標高度
                down_pos = [self._last_knob_x, 0.0, self._last_knob_z - PRESS_Z_DOWN]
                self.arm.move_to_position(down_pos)
                print("[State 5] 門把已壓下")
                time.sleep(0.5)
                self._transition(DoorOpenState.OPEN_DOOR)
            except Exception as e:
                print(f"[State 5] 下壓失敗: {e}")
                self._transition(DoorOpenState.ERROR)

            self._press_count = 1

        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 6：車子後退推開門
    # ──────────────────────────────────────────────────────────────────────────

    def _state_open_door(self):
        if self._open_start is None:
            print("[State 6] 後退推開門…")
            self._open_start = time.time()

        elapsed = time.time() - self._open_start

        if elapsed < OPEN_DOOR_DURATION:
            self.car.update_action(DOOR_OPEN_ACTION)
        else:
            self.car.update_action("STOP")
            print("[State 6] 開門完成！")
            self._transition(DoorOpenState.DONE)

        time.sleep(0.05)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # 輔助方法
    # ──────────────────────────────────────────────────────────────────────────

    def _transition(self, new_state):
        """切換狀態並重置計數器"""
        print(f"[FSM] 狀態切換：{self.state} → {new_state}")

        # 任務結束（成功或失敗）時清除 YOLO 追蹤目標，並釋放夾爪、重置手臂
        if new_state in (DoorOpenState.DONE, DoorOpenState.ERROR):
            self.rc.publish_target_label("")
            try:
                print("[FSM] 釋放夾爪並重置手臂姿態…")
                self.arm.set_last_joint_angle(70.0) # 鬆開夾爪
                time.sleep(0.5)
                self.arm.reset_arm(all_angle_degrees=90.0)                 # 回歸初始姿態
            except Exception as e:
                print(f"[FSM] 重置手臂失敗: {e}")

        self.state        = new_state
        self._iter        = 0
        self._press_count = 0
        self._open_start  = None


# ─────────────────────────────────────────────────────────────────────────────
# ros2 run pros_car_py door_open  的入口點
# ─────────────────────────────────────────────────────────────────────────────

def main():
    """
    獨立執行入口。
    將此函式加入 setup.py 的 console_scripts：
        "door_open = pros_car_py.door_open_task:main",
    """
    from pros_car_py.car_controller   import CarController
    from pros_car_py.arm_controller   import ArmController
    from pros_car_py.ik_solver        import PybulletRobotController
    from pros_car_py.data_processor   import DataProcessor
    from pros_car_py.nav_processing   import Nav2Processing
    from pros_car_py.ros_communicator import RosCommunicator

    rclpy.init()
    ros_communicator = RosCommunicator()

    # 在背景執行 ROS spin
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_communicator,), daemon=True
    )
    spin_thread.start()

    # 等待感測器資料就緒（AMCL、YOLO）
    print("[main] 等待感測器就緒…")
    time.sleep(3.0)

    data_processor  = DataProcessor(ros_communicator)
    nav_processing  = Nav2Processing(ros_communicator, data_processor)
    ik_solver       = PybulletRobotController(end_eff_index=5)
    car_controller  = CarController(ros_communicator, nav_processing)
    arm_controller  = ArmController(ros_communicator, data_processor, ik_solver)

    task = DoorOpenTask(
        car_controller  = car_controller,
        arm_controller  = arm_controller,
        nav_processing  = nav_processing,
        data_processor  = data_processor,
        ros_communicator= ros_communicator,
    )

    try:
        task.run_blocking(spin_interval=0.05)  # 20Hz FSM 更新率，確保馬達指令連續
    except KeyboardInterrupt:
        print("[main] 中斷，停車")
        car_controller.update_action("STOP")
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
