"""
bridge_mission / bridge_mission_node.py
【負責人：組員 B】

任務目標（決賽第二/三階段，每隊 2.5 分鐘）：
    1. 從起始點導航至橋樑入口。
    2. 慢速謹慎地上橋，抵達橋中點（+5分）。
    3. 在橋上辨識並夾取目標物件（+5分）。
    4. 從橋的「另一側」下橋。
    5. 導航回基地放下物件（+5分）。

注意事項：
    - 橋邊緣碰撞 = 扣分！上橋速度要慢，優先用低速模式。
    - 不能走回頭路，必須從橋的另一側下橋。
    - 時間只有 2.5 分鐘，要快但不失精確。

啟動方式（在 Docker 容器內）：
    ros2 run bridge_mission bridge_mission_node
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from wildbot_common.navigation_client import NavigationClient
from wildbot_common.gripper_controller import GripperController
from wildbot_common.robot_state import (
    BridgeMissionState, BASE_POSITION, BRIDGE_ENTRY_POSITION
)


class BridgeMissionNode(Node):
    """
    上橋夾熊任務節點。

    執行流程：導航至橋 → 謹慎上橋 → 到橋中點 → 夾熊 → 下橋 → 回基地。
    """

    # 【TODO - 組員 B】：確認 YOLO 輸出的 Topic 名稱（與 bear_grabber 相同）
    BEAR_DETECTION_TOPIC = '/yolo/detections'

    # 【TODO - 組員 B】：用 SLAM 地圖確認後填入這些座標
    BRIDGE_MIDPOINT = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}   # 橋中點（得分點）
    BRIDGE_EXIT     = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}   # 橋的另一側出口

    # 上橋時的速度限制（比平地慢，避免碰到橋邊緣）
    BRIDGE_MAX_SPEED = 0.15  # m/s

    def __init__(self):
        super().__init__('bridge_mission_node')
        self.get_logger().info('=== 上橋夾熊任務節點啟動 ===')

        # 共用模組
        self.nav = NavigationClient(self)
        self.gripper = GripperController(self)

        # 用於上橋精細控制的直接速度發布
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 狀態機
        self.state = BridgeMissionState.IDLE
        self.current_bear_position = None

        # 訂閱 YOLO 偵測結果
        self.bear_sub = self.create_subscription(
            Float32MultiArray,
            self.BEAR_DETECTION_TOPIC,
            self._on_bear_detected,
            10
        )

        # 主控制迴圈
        self.timer = self.create_timer(0.5, self._state_machine_loop)
        self.get_logger().info('初始化完成，等待任務開始...')

    def _on_bear_detected(self, msg):
        """接收 YOLO 偵測到的橋上熊的位置。"""
        # 【TODO - 組員 B】：解析出熊的世界座標
        if len(msg.data) >= 2:
            self.current_bear_position = (msg.data[0], msg.data[1])

    def _move_slow(self, linear_x: float, angular_z: float = 0.0):
        """
        直接發送慢速指令（繞過 Nav2），用於在橋上精細控制。
        只在上橋過程中使用，平地導航請用 self.nav.go_to()。
        """
        twist = Twist()
        # 限制速度不超過橋上安全速度
        twist.linear.x = max(-self.BRIDGE_MAX_SPEED,
                              min(self.BRIDGE_MAX_SPEED, linear_x))
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def _stop(self):
        """緊急停止。"""
        self._move_slow(0.0, 0.0)

    def _state_machine_loop(self):
        """主狀態機迴圈。"""

        if self.state == BridgeMissionState.IDLE:
            self.get_logger().info('狀態：IDLE → 準備前往橋樑...')
            self.state = BridgeMissionState.NAVIGATING_TO_BRIDGE

        elif self.state == BridgeMissionState.NAVIGATING_TO_BRIDGE:
            self.get_logger().info('導航至橋樑入口...')
            success = self.nav.go_to(
                x=BRIDGE_ENTRY_POSITION['x'],
                y=BRIDGE_ENTRY_POSITION['y'],
                yaw=BRIDGE_ENTRY_POSITION['yaw']
            )
            if success:
                self.get_logger().info('已抵達橋樑入口！切換慢速上橋模式...')
                self.state = BridgeMissionState.CROSSING_BRIDGE
            else:
                self.get_logger().error('無法抵達橋樑入口，重試...')

        elif self.state == BridgeMissionState.CROSSING_BRIDGE:
            # 【TODO - 組員 B】：
            # 上橋是最關鍵的步驟！這裡有幾個選擇：
            # 方案一（簡單）：直接用 Nav2 導航到橋中點，信任 Nav2 的路徑規劃。
            # 方案二（精確）：切換成慢速直線行驶，同時用雷達偵測橋邊緣距離保持置中。
            #
            # 以下示範方案一：
            self.get_logger().info('上橋中，導航至橋中點...')
            success = self.nav.go_to(
                x=self.BRIDGE_MIDPOINT['x'],
                y=self.BRIDGE_MIDPOINT['y'],
                yaw=self.BRIDGE_MIDPOINT['yaw']
            )
            if success:
                self.get_logger().info('已抵達橋中點！（+5分！）搜尋橋上的熊...')
                self.state = BridgeMissionState.REACHING_MIDPOINT
            else:
                self.get_logger().error('上橋失敗！')

        elif self.state == BridgeMissionState.REACHING_MIDPOINT:
            # 已在橋中點，開始搜尋熊
            self.state = BridgeMissionState.SEARCHING_BEAR_ON_BRIDGE

        elif self.state == BridgeMissionState.SEARCHING_BEAR_ON_BRIDGE:
            if self.current_bear_position is not None:
                self.get_logger().info(f'在橋上找到熊！位置：{self.current_bear_position}')
                self.state = BridgeMissionState.GRABBING
            else:
                self.get_logger().info('橋上搜尋中...')
                # 【TODO - 組員 B】：如果找不到，考慮原地小幅旋轉

        elif self.state == BridgeMissionState.GRABBING:
            self._stop()  # 夾取前先停止
            self.gripper.grab_sequence()
            self.current_bear_position = None
            self.get_logger().info('橋上夾熊完成！（+5分！）準備從另一側下橋...')
            self.state = BridgeMissionState.DESCENDING_BRIDGE

        elif self.state == BridgeMissionState.DESCENDING_BRIDGE:
            # 【TODO - 組員 B】：
            # 從橋的「另一側」下橋，不能走回頭路！
            # 導航至橋另一側的出口座標。
            self.get_logger().info('從另一側下橋中...')
            success = self.nav.go_to(
                x=self.BRIDGE_EXIT['x'],
                y=self.BRIDGE_EXIT['y'],
                yaw=self.BRIDGE_EXIT['yaw']
            )
            if success:
                self.get_logger().info('已下橋！返回基地...')
                self.state = BridgeMissionState.RETURNING_TO_BASE
            else:
                self.get_logger().error('下橋失敗！')

        elif self.state == BridgeMissionState.RETURNING_TO_BASE:
            success = self.nav.go_to(
                x=BASE_POSITION['x'],
                y=BASE_POSITION['y'],
                yaw=BASE_POSITION['yaw']
            )
            if success:
                self.get_logger().info('已回到基地！放下熊...')
                self.state = BridgeMissionState.RELEASING

        elif self.state == BridgeMissionState.RELEASING:
            self.gripper.release_sequence()
            self.get_logger().info('放下橋上的熊！（+5分！）任務完成！')
            self.state = BridgeMissionState.DONE

        elif self.state == BridgeMissionState.DONE:
            self.get_logger().info('上橋任務全部完成！')
            # 停止計時器，任務結束
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('節點被手動停止。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
