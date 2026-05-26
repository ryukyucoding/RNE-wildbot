# return 角度一律 radians
import math
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from pros_car_py.car_models import DeviceDataTypeEnum
import numpy as np
import time
import sys


import numpy as np
import threading
import rclpy


class ArmController:
    def __init__(self, ros_communicator, data_processor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        self.target_marker = None
        
        # 建立 TF2 監聽器 (使用 ros_communicator 作為 Node)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.ros_communicator)
        
        # ==========================================
        # 1. 手臂基礎與機構設定 (統一管理區)
        # ==========================================
        self.base_link_name = 'base_link' # 第一個馬達的基準座標系
        
        # 動態定義所有關節 (加入 length 臂長 與 angle_offset 角度補償)
        # angle_offset: IK 算出來的數學 0 度可能不是 Unity 的 0 度，可透過這個補償
        self.joint_limits = [
            {"length": 0.08089007, "min_angle": -180, "max_angle": 0, "init": -180, "offset": 270,  "dir": -1.0},  # Joint 0 (Shoulder)
            {"length": 0.11,       "min_angle": -240, "max_angle": 0, "init": -0,   "offset": -120, "dir": -1.0},  # Joint 1 (Elbow)
            {"length": 0.00,       "min_angle": 20,   "max_angle": 229, "init": 229,  "offset": 0.0,  "dir": 1.0},   # Joint 2 (Gripper)  229° * 1.0 dir = 4.0 rad (fully open)
        ]

        self.joint_angles = [joint["init"] for joint in self.joint_limits]
        self.manual_step = 3.0

        self.gripper_min_rad = 0.0

        print(f"🦾 Arm Controller Initialized: {len(self.joint_limits)} Joints Managed.")

    # ==========================================
    # 2. 手動控制邏輯 (Manual Control)
    # ==========================================
    def manual_control(self, index, key):
        """處理手動按鍵輸入，並根據 index 控制特定關節"""
        
        # 處理不依賴 index 的全域指令 ('b', 'q')
        if key == "b":  
            # 🌟 重置手臂：讀取 __init__ 裡面設定的 init 初始角度
            self.joint_angles = [joint["init"] for joint in self.joint_limits]
            self._clamp_and_publish()
            self._visualize_arm_lines()
            print("手臂已重置為初始角度。")
            return False
            
        elif key == "q":  
            # 結束控制
            print("結束手臂手動控制。")
            return True

        # 處理針對特定 index 的控制 ('i', 'k')
        if 0 <= index < len(self.joint_limits):
            if key == "i":
                self.joint_angles[index] += self.manual_step
            elif key == "k":
                self.joint_angles[index] -= self.manual_step
            else:
                print(f"按鍵 '{key}' 無效，請使用 'i'(增加), 'k'(減少), 'b'(重置), 或 'q'(取消)。")
                return False
                
            # 計算完畢後，進行安全檢查並發布
            self._clamp_and_publish()
            self._visualize_arm_lines()
        else:
            print(f"索引 {index} 無效，請確保其在範圍內（0-{len(self.joint_limits) - 1}）。")
            
        return False

    def auto_control(self, key=None, mode="auto_arm_control"):
        """自動抓取 /yolo/target_marker 的目標"""
        
        # 1. 取得最新目標位置
        target_marker = self.ros_communicator.latest_yolo_marker
        if not target_marker:
            print("尚未收到 YOLO 目標，等待中...")
            return
        
        if key == "g":
            self.target_marker = target_marker
        elif key == "q":
            self.target_marker = None
            return
        elif key == "b":  
            # 🌟 重置手臂：讀取 __init__ 裡面設定的 init 初始角度
            self.joint_angles = [joint["init"] for joint in self.joint_limits]
            self._clamp_and_publish()
            self._visualize_arm_lines()
            print("手臂已重置為初始角度。")
            return
        else :
            print(f"按鍵 '{key}' 無效，請使用 'g'(抓取), 'b'(重置), 或 'q'(取消)。")
            return

        # 2. 建立目標的 PointStamped (原本在 map 座標系)
        target_map = PointStamped()
        target_map.header.frame_id = target_marker.header.frame_id # 通常是 'map'
        target_map.header.stamp = self.ros_communicator.get_clock().now().to_msg()
        target_map.point = target_marker.pose.position

        try:
            # 3. 將 map 上的網球，轉換到手臂基準座標系
            transform = self.tf_buffer.lookup_transform(
                self.base_link_name,
                target_map.header.frame_id,
                rclpy.time.Time()
            )
            target_base = tf2_geometry_msgs.do_transform_point(target_map, transform)
            
            x_target = target_base.point.x
            z_target = target_base.point.z
            
            print(f"🎯 目標相對基座座標: X={x_target:.3f}, Z={z_target:.3f}")

            # 🌟 4. 開啟背景執行緒，執行「抓取與緩慢歸位」的完整排程
            # 使用 daemon=True 確保程式關閉時執行緒會自動結束
            threading.Thread(
                target=self._execute_grab_sequence, 
                args=(x_target, z_target), 
                daemon=True
            ).start()

        except Exception as e:
            print(f"⚠️ 座標轉換或 TF 失敗: {e}")

    def run_grasp_blocking(self, marker_msg=None):
        """
        實體車抓取流程：在當前水平位置直接夾緊，再上抬。
        不做 IK 手臂定位，不需要 YOLO marker 或 TF。
        """
        self._execute_grab_sequence()
        return True

    def _physical_deg_to_internal(self, joint_idx, physical_deg):
        """Unity / 實體關節角度 → internal angle（physical = internal * dir）。"""
        direction = self.joint_limits[joint_idx].get("dir", 1.0)
        if direction == 0:
            return physical_deg
        return physical_deg / direction

    def _internal_deg_to_physical(self, joint_idx, internal_deg):
        return internal_deg * self.joint_limits[joint_idx].get("dir", 1.0)

    def run_unity_vision_stow_blocking(self, unity_elbow_deg=180.0):
        """
        Unity 虛擬環境：任務開始前將 Elbow 放低，避免夾爪擋住相機視野。
        unity_elbow_deg: Unity 介面顯示的角度（最低約 180°）。
        """
        try:
            target_internal = self._physical_deg_to_internal(1, unity_elbow_deg)
            min_a = self.joint_limits[1]["min_angle"]
            max_a = self.joint_limits[1]["max_angle"]
            target_internal = max(min_a, min(max_a, target_internal))
            physical = self._internal_deg_to_physical(1, target_internal)
            print(
                f"📷 [unity stow] Elbow → Unity {physical:.1f}° "
                f"(internal {target_internal:.1f}°)"
            )
            self._smooth_move_to([None, target_internal, None], step=5.0, delay=0.1)
            time.sleep(0.3)
            return True
        except Exception as e:
            print(f"⚠️ run_unity_vision_stow_blocking 失敗: {e}")
            return False

    def run_release_blocking(self):
        """
        在「已經帶著目標回家」後同步執行放下：
        1) 先開夾爪讓熊自然落地
        2) 等熊落穩後再把手臂收回安全姿態
        """
        try:
            print("🧺 [release] 打開夾爪放下目標...")
            target_open = [None, None, self.joint_limits[2]["max_angle"]]
            self._smooth_move_to(target_open, step=5.0, delay=0.1)
            time.sleep(0.8)  # 等熊落地

            print("🧺 [release] 手臂收回安全姿態...")
            init_angles = [None, self.joint_limits[1]["init"], None]
            self._smooth_move_to(init_angles, step=5.0, delay=0.1)
            init_angles = [self.joint_limits[0]["init"], None, None]
            self._smooth_move_to(init_angles, step=5.0, delay=0.1)
            time.sleep(0.3)

            print("✅ [release] 目標已放下，手臂歸位")
            return True
        except Exception as e:
            print(f"⚠️ run_release_blocking 失敗: {e}")
            return False

    def _execute_grab_sequence(self):
        """實體車抓取：elbow 微降 → 原位夾緊 → 上抬"""

        # 步驟 1：夾取前 elbow 再低一點，讓夾爪更貼近熊
        print("🦾 [1/3] Elbow 微降準備夾取...")
        elbow_lower = self.joint_angles[1] - 12.0
        elbow_lower = max(self.joint_limits[1]["min_angle"], elbow_lower)
        self._smooth_move_to([None, elbow_lower, None], step=4.0, delay=0.08)
        time.sleep(0.2)

        # 步驟 2：夾緊
        print("✊ [2/3] 夾緊目標...")
        target_close = [None, None, self.joint_limits[2]["min_angle"]]
        self._smooth_move_to(target_close, step=5.0, delay=0.1)
        time.sleep(0.8)

        # 步驟 3：肩膀上抬 25 度，夾爪保持夾住
        print("🏠 [3/3] 上抬固定...")
        shoulder_lift = self.joint_angles[0] - 25  # 負方向 = 抬起
        shoulder_lift = max(self.joint_limits[0]["min_angle"], shoulder_lift)
        self._smooth_move_to([shoulder_lift, None, None], step=5.0, delay=0.1)
        time.sleep(0.3)

        print("✅ 抓取完成！")

    def _calculate_2d_ik(self, x, z):
        """計算 2D 逆向運動學，並回傳目標角度 (不直接移動手臂)"""
        L1 = self.joint_limits[0]["length"]
        L2 = self.joint_limits[1]["length"]
        
        D = math.sqrt(x**2 + z**2)
        if D > (L1 + L2):
            print("⚠️ 目標超出最遠抓取距離，以最大伸展姿態計算。")
            D = L1 + L2 - 0.001 
        
        cos_theta2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2)) 
        
        # 由上往下夾 (Elbow Up)
        theta2_rad = -math.acos(cos_theta2) 
        
        alpha = math.atan2(z, x)
        beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
        theta1_rad = alpha + beta 
        
        # 轉換為 Degrees 並加上偏移量
        deg1 = math.degrees(theta1_rad) + self.joint_limits[0]["offset"]
        deg2 = math.degrees(theta2_rad) + self.joint_limits[1]["offset"]
        
        deg1 = self._normalize_angle(deg1, self.joint_limits[0]["min_angle"], self.joint_limits[0]["max_angle"])
        deg2 = self._normalize_angle(deg2, self.joint_limits[1]["min_angle"], self.joint_limits[1]["max_angle"])
        
        print(f"🧮 IK 計算完成: Shoulder={deg1:.1f}°, Elbow={deg2:.1f}°")
        return deg1, deg2
    
    def _smooth_move_to(self, target_angles, step=2.0, delay=0.05):
        """
        將手臂平滑地移動到目標角度 (Trajectory Interpolation)
        - target_angles: [j0_target, j1_target, j2_target] (若填 None 則該軸不動)
        - step: 每次更新的最大度數 (越小越滑順)
        - delay: 每次更新間隔的時間 (越大越慢)
        """
        # 先把目標夾到合法範圍，避免超範圍目標導致軌跡無法收斂
        sanitized_targets = list(target_angles)
        for i in range(len(self.joint_angles)):
            if sanitized_targets[i] is None:
                continue
            min_a = self.joint_limits[i]["min_angle"]
            max_a = self.joint_limits[i]["max_angle"]
            sanitized_targets[i] = max(min_a, min(max_a, sanitized_targets[i]))

        while True:
            all_reached = True
            
            for i in range(len(self.joint_angles)):
                if sanitized_targets[i] is None:
                    continue # None 代表該關節不移動
                    
                diff = sanitized_targets[i] - self.joint_angles[i]
                
                if abs(diff) <= step:
                    self.joint_angles[i] = sanitized_targets[i]
                else:
                    self.joint_angles[i] += step if diff > 0 else -step
                    all_reached = False # 只要有一個關節還沒到，就繼續迴圈
            
            # 發布這一小步的姿態
            self._clamp_and_publish()
            self._visualize_arm_lines()
            
            if all_reached:
                break
                
            time.sleep(delay)

    def _normalize_angle(self, angle, min_limit, max_limit):
        """
        嘗試加減 360 度，尋找是否有多轉或少轉一圈後，
        剛好能落入 [min_limit, max_limit] 物理極限內的同界角。
        """
        # 1. 將角度正規化到 0 ~ 360 的基準
        base_angle = angle % 360.0
        
        # 2. 準備三個候選角度：少一圈、當前(0~360)、多一圈
        candidates = [base_angle - 360.0, base_angle, base_angle + 360.0]
        
        # 3. 檢查哪一個落在合法範圍內
        for cand in candidates:
            if min_limit <= cand <= max_limit:
                return cand # 找到合法的同界角，直接回傳！
                
        # 如果加減 360 度後都不在範圍內，直接夾到邊界，避免目標超範圍導致迴圈卡住。
        return max(min_limit, min(max_limit, angle))
    # ==========================================
    # 5. 視覺化手臂 (Foxglove Lines)
    # ==========================================
    def _visualize_arm_lines(self):
        """根據當前的角度和長度，算出 3 個點的座標並發布 3D 視覺化線條"""
        L1 = self.joint_limits[0]["length"]
        L2 = self.joint_limits[1]["length"]
        
        # 扣除補償值，轉回純數學弧度，方便做正向運動學(FK)
        th1 = math.radians(self.joint_angles[0] - self.joint_limits[0]["offset"])
        th2 = math.radians(self.joint_angles[1] - self.joint_limits[1]["offset"])
        
        # 基座位置 P0
        p0 = Point(x=0.0, y=0.0, z=0.0)
        # 關節1位置 P1
        p1 = Point(x=L1 * math.cos(th1), y=0.0, z=L1 * math.sin(th1))
        # 夾爪末端位置 P2
        p2 = Point(x=p1.x + L2 * math.cos(th1 + th2), y=0.0, z=p1.z + L2 * math.sin(th1 + th2))
        
        # 建立 Marker
        marker = Marker()
        marker.header.frame_id = self.base_link_name
        marker.header.stamp = self.ros_communicator.get_clock().now().to_msg()
        marker.ns = "arm_kinematics"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.02 # 線條粗細
        marker.color.a = 1.0  # 不透明度
        marker.color.r = 0.0  # 青藍色
        marker.color.g = 1.0
        marker.color.b = 1.0
        
        marker.points = [p0, p1, p2]
        
        self.ros_communicator.publish_arm_visual_lines(marker)

    def _clamp_and_publish(self):
        """確保所有數值在安全範圍內，並轉換為「弧度」後發布"""
        for i in range(len(self.joint_limits)):
            min_a = self.joint_limits[i]["min_angle"]
            max_a = self.joint_limits[i]["max_angle"]
            self.joint_angles[i] = max(min_a, min(max_a, self.joint_angles[i]))
            
        joint_pos_radians = [
            math.radians(float(self.joint_angles[i]) * self.joint_limits[i].get("dir", 1.0))
            for i in range(len(self.joint_angles))
        ]
        if self.gripper_min_rad > 0.0:
            joint_pos_radians[2] = max(self.gripper_min_rad, joint_pos_radians[2])
        self.ros_communicator.publish_robot_arm_angle(joint_pos_radians)