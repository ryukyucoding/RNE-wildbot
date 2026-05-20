"""
door_open_task.py
=================
自動開門任務腳本（有限狀態機 FSM）

任務流程：
  State 0  NAVIGATE_TO_DOOR    —— 使用 Nav2 導航至門前預設座標
  State 1  SEARCH_HANDLE       —— 原地旋轉直到 YOLO 偵測到門把
  State 2  ALIGN_CAR           —— 車身水平對齊門把（pixel offset 置中）
  State 3  ARM_AIM             —— IK 解算，手臂末端瞄準門把前方
  State 4  ARM_APPROACH        —— 手臂緩慢前伸直到深度感測觸碰
  State 5  PRESS_DOWN          —— 末端向下壓（模擬壓門把）
  State 6  OPEN_DOOR           —— 車子後退推開門
  State 7  DONE                —— 完成，重置手臂回歸初始姿態

使用方式：
  1. 在 main2.py 或其他入口點建立此類別的物件
  2. 在你的控制迴圈中呼叫 door_open_task.step()
  3. 或直接使用 door_open_task.run_blocking() 阻塞執行到結束

ros2 run pros_car_py door_open  （需在 setup.py 加入 entry point）
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

# State 3：IK 對齊容差（公尺）
ARM_AIM_TOL = 0.04

# State 4：手臂前進步長及目標距離
ARM_APPROACH_STEP = 0.02         # 每步 2cm
ARM_TOUCH_DEPTH   = 0.06         # 深度相機回傳值 < 6cm 表示已觸碰

# State 5：壓下門把的總下移距離與步長
PRESS_Z_STEP   = -0.015          # 每步向下 1.5cm
PRESS_STEPS    = 4               # 共壓 4 步 ≈ 6cm

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
    # State 4：手臂 IK 對準門把（先對齊，不碰到）
    # ──────────────────────────────────────────────────────────────────────────

    def _state_arm_aim(self):
        print("[State 3] 手臂 IK 對準門把…")
        self.arm.ensure_joint_pos_initialized()

        # 開啟夾爪（最後一軸張開）
        self.arm.set_last_joint_angle(70.0)
        time.sleep(0.3)

        # 利用現有的 align_to_target_with_yolo_offset 對齊，
        # 它會反覆微調直到畫面 offset 落在容差內，最多 10 次迭代
        aligned = self.arm.align_to_target_with_yolo_offset(
            step_size=0.06, tolerance=ARM_AIM_TOL
        )

        if aligned:
            print("[State 3] 手臂對齊完成")
            self._transition(DoorOpenState.ARM_APPROACH)
        else:
            print("[State 3] 手臂對齊失敗，重試一次")
            # 再試一次，若還是失敗才放棄
            aligned = self.arm.align_to_target_with_yolo_offset(
                step_size=0.04, tolerance=ARM_AIM_TOL
            )
            if aligned:
                self._transition(DoorOpenState.ARM_APPROACH)
            else:
                print("[State 3] 手臂對齊失敗，任務失敗")
                self._transition(DoorOpenState.ERROR)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 4：手臂緩慢前伸直到觸碰門把
    # ──────────────────────────────────────────────────────────────────────────

    def _state_arm_approach(self):
        if self._iter == 0:
            print("[State 4] 手臂前進觸碰門把…")

        # 讀取深度相機正前方深度（YOLO 偵測到的物體深度）
        yolo = self.dp.get_yolo_target_info()
        depth = yolo[1] if (yolo is not None and yolo[0] == 1) else None

        if depth is not None and 0 < depth < ARM_TOUCH_DEPTH:
            # 已觸碰（深度小於觸碰閾值）
            print(f"[State 4] 觸碰到門把（depth={depth:.3f}m）")
            self._transition(DoorOpenState.PRESS_DOWN)
            return False

        if depth is not None and depth <= 0:
            # 深度感測器回傳 -1（float）表示已非常近（< ~4cm），也算觸碰到
            print(f"[State 4] 已非常接近門把（depth={depth}，視為觸碰）")
            self._transition(DoorOpenState.PRESS_DOWN)
            return False

        # 沿手臂末端執行器的「正前方向」前進一步（世界座標系）。
        # get_forward_position() 會利用末端旋轉矩陣的 X 軸方向計算目標點，
        # 確保是沿手臂朝向前進，而非世界座標系的固定軸。
        target_pos = self.arm.get_forward_position(
            offset_distance=ARM_APPROACH_STEP,
            z_offset=0.0,   # 不需要額外的世界 Z 偏移
        )
        self.arm.move_to_position(target_pos)

        self._iter += 1
        if self._iter > 80:   # 最多前進 80 * 2cm = 160cm，防呆
            print("[State 4] 手臂前進逾時，任務失敗")
            self._transition(DoorOpenState.ERROR)

        time.sleep(0.15)
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # State 5：向下壓門把
    # ──────────────────────────────────────────────────────────────────────────

    def _state_press_down(self):
        if self._press_count == 0:
            print("[State 5] 觸碰完成，先夾緊夾爪（10度）咬住門把…")
            self.arm.set_last_joint_angle(10.0)   # 夾緊夾爪
            time.sleep(0.8)                       # 等待夾爪咬合完成
            print("[State 5] 開始向下壓門把…")

        if self._press_count >= PRESS_STEPS:
            print("[State 5] 門把已壓下")
            time.sleep(0.5)   # 確保機械臂到位
            self._transition(DoorOpenState.OPEN_DOOR)
            return False

        print(f"[State 5] 下壓步驟 {self._press_count + 1}/{PRESS_STEPS}")
        # 末端沿世界 Z 軸向下移動
        self.arm.move_end_effector(z_offset=PRESS_Z_STEP)
        self._press_count += 1
        time.sleep(0.3)
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
                self.arm.set_last_joint_angle(70.0)  # 鬆開夾爪（張開到最大）
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
    from pros_car_py.arm_controller   import ArmController
    from pros_car_py.data_processor   import DataProcessor
    from pros_car_py.nav_processing   import Nav2Processing
    from pros_car_py.ros_communicator import RosCommunicator
    from pros_car_py.ik_solver        import PybulletRobotController

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
    arm_controller  = ArmController(
        ros_communicator, data_processor, ik_solver, num_joints=6
    )

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
