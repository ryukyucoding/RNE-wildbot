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

# State 2：對齊門把的像素容差（畫面寬度約 640px，小於此值視為置中）
ALIGN_PIXEL_TOL = 60             # px

# State 2.5：車子前進靠近門的停止深度（公尺），小於此值表示已夠近可以伸手臂
DRIVE_STOP_DEPTH = 0.6            # 距門把 0.6m 內停車

# State 4：手臂先移到門把「正上方」的偏移量（公尺）
# 夾爪張開後，末端停在門把上方 5cm
PRESS_ABOVE_OFFSET = 0.05        # 5cm

# State 5：夾爪合上後，向下壓的距離（公尺）
# 往下壓 7cm（door handle 行程通常 5~8cm）
PRESS_Z_DOWN = 0.07              # 7cm

# State 6：後退開門的秒數
OPEN_DOOR_DURATION = 2.5         # 秒

# ─────────────────────────────────────────────────────────────────────────────


class DoorOpenState:
    """枚舉各狀態"""
    NAVIGATE_TO_DOOR = 0
    SEARCH_HANDLE    = 1
    ALIGN_CAR        = 2
    DRIVE_TO_DOOR    = 3   # ← 新增：視覺對齊後開車靠近門
    ARM_AIM          = 4
    ARM_APPROACH     = 5
    PRESS_DOWN       = 6
    OPEN_DOOR        = 7
    DONE             = 8
    ERROR            = 99


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
        elif self.state == DoorOpenState.ALIGN_CAR:
            return self._state_align_car()
        elif self.state == DoorOpenState.DRIVE_TO_DOOR:
            return self._state_drive_to_door()
        elif self.state == DoorOpenState.ARM_AIM:
            return self._state_arm_aim()
        elif self.state == DoorOpenState.ARM_APPROACH:
            return self._state_arm_approach()
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
            self._transition(DoorOpenState.ALIGN_CAR)
            return False

        # 緩慢順時針旋轉搜尋
        self.car.update_action("CLOCKWISE_ROTATION_SLOW")
        self._iter += 1

        if self._iter > SEARCH_MAX_ITER:
            print("[State 1] 搜尋逾時，任務失敗")
            self._transition(DoorOpenState.ERROR)

        time.sleep(0.1)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 2：車身水平對齊門把（利用 YOLO pixel offset）
    # ──────────────────────────────────────────────────────────────────────────

    def _state_align_car(self):
        if self._iter == 0:
            print("[State 2] 水平對齊門把…")

        yolo = self.dp.get_yolo_target_info()

        if yolo is None or yolo[0] == 0:
            # 目標丟失，回到搜尋
            print("[State 2] 目標丟失，回到搜尋")
            self._transition(DoorOpenState.SEARCH_HANDLE)
            return False

        pixel_offset = yolo[2]   # 正 = 目標在右側，負 = 目標在左側

        if abs(pixel_offset) <= ALIGN_PIXEL_TOL:
            print(f"[State 2] 對齊完成（offset={pixel_offset:.1f}px），開始前進靠近門")
            self.car.update_action("STOP")
            self._transition(DoorOpenState.DRIVE_TO_DOOR)
            return False

        # 根據偏移方向旋轉
        if pixel_offset > 0:
            self.car.update_action("CLOCKWISE_ROTATION_SLOW")
        else:
            self.car.update_action("COUNTERCLOCKWISE_ROTATION_SLOW")

        self._iter += 1
        if self._iter > SEARCH_MAX_ITER:
            print("[State 2] 對齊逾時，任務失敗")
            self._transition(DoorOpenState.ERROR)

        time.sleep(0.1)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 3：前進靠近門，直到 YOLO 深度 < DRIVE_STOP_DEPTH
    # ──────────────────────────────────────────────────────────────────────────

    def _state_drive_to_door(self):
        if self._iter == 0:
            print(f"[State 3] 前進靠近門把，目標距離 < {DRIVE_STOP_DEPTH}m…")

        yolo = self.dp.get_yolo_target_info()

        if yolo is None or yolo[0] == 0:
            # 前進中目標丟失（可能被遮到），先停車再回去搜尋
            print("[State 3] 前進中目標丟失，停車回到搜尋")
            self.car.update_action("STOP")
            self._transition(DoorOpenState.SEARCH_HANDLE)
            return False

        depth = yolo[1]
        pixel_offset = yolo[2]

        # 已夠近，停車進入手臂對準
        if 0 < depth < DRIVE_STOP_DEPTH:
            print(f"[State 3] 已靠近門把（depth={depth:.3f}m），停車")
            self.car.update_action("STOP")
            self._transition(DoorOpenState.ARM_AIM)
            return False

        # 前進中若有偏移（> 容差的 2 倍），順手微調方向
        if abs(pixel_offset) > ALIGN_PIXEL_TOL * 2:
            if pixel_offset > 0:
                self.car.update_action("RIGHT_FRONT")
            else:
                self.car.update_action("LEFT_FRONT")
        else:
            # 對準就直直前進
            self.car.update_action("FORWARD_SLOW")

        self._iter += 1
        if self._iter > 400:   # 最多前進 40 秒防呆
            print("[State 3] 前進逾時，任務失敗")
            self._transition(DoorOpenState.ERROR)

        time.sleep(0.1)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 4：手臂移到門把正上方，夾爪張開
    # ──────────────────────────────────────────────────────────────────────────

    def _state_arm_aim(self):
        print("[State 4] 夾爪張開，手臂移到門把正上方…")

        # 先張開夾爪
        self.arm.set_last_joint_angle(90.0)
        time.sleep(0.5)

        # 最多重試 5 次等 TF 座標
        for attempt in range(5):
            coords = self.arm.get_target_relative_coords()
            if coords is not None:
                break
            print(f"[State 4] 等待 TF 座標 ({attempt + 1}/5)…")
            time.sleep(0.3)
        else:
            print("[State 4] 無法獲取門把 TF 座標，任務失敗")
            self._transition(DoorOpenState.ERROR)
            return False

        x_target, z_target = coords
        print(f"[State 4] 門把相對座標: X={x_target:.3f}m, Z={z_target:.3f}m")

        # 目標點：門把正上方 5cm（z + 0.05），x 對齊門把
        x_above = x_target
        z_above = z_target + PRESS_ABOVE_OFFSET
        print(f"[State 4] 移到正上方: X={x_above:.3f}m, Z={z_above:.3f}m")

        try:
            self.arm.move_to_2d_position(x_above, z_above, step=3.0, delay=0.05)
            # 儲存基準座標，讓下壓使用
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
            self.arm.set_last_joint_angle(20.0)   # 夾爪合攏（20度）
            time.sleep(0.8)                       # 等待夾爪咬合

            print(f"[State 5] 往下壓 {PRESS_Z_DOWN * 100:.1f}cm，目標: X={self._last_knob_x:.3f}m, Z={self._last_knob_z - PRESS_Z_DOWN:.3f}m")

            try:
                # 直接一步壓到目標高度
                self.arm.move_to_2d_position(
                    self._last_knob_x,
                    self._last_knob_z - PRESS_Z_DOWN,
                    step=2.0,
                    delay=0.05,
                )
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
            self.car.update_action("BACKWARD_SLOW")
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
                self.arm.set_last_joint_angle(90.0)  # 2D 鬆開夾爪（張開到最大為 90度）
                time.sleep(0.5)
                self.arm.reset_arm()                 # 回歸初始姿態
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
    from pros_car_py.arm_controller_2D import ArmController
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
    car_controller  = CarController(ros_communicator, nav_processing)
    arm_controller  = ArmController(ros_communicator, data_processor)

    task = DoorOpenTask(
        car_controller  = car_controller,
        arm_controller  = arm_controller,
        nav_processing  = nav_processing,
        data_processor  = data_processor,
        ros_communicator= ros_communicator,
    )

    try:
        task.run_blocking(spin_interval=0.1)
    except KeyboardInterrupt:
        print("[main] 中斷，停車")
        car_controller.update_action("STOP")
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
