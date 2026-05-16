"""
door_opener / door_opener_node.py
【負責人：組員 C】

任務目標（決賽第二/三階段，每隊 2.5 分鐘）：
    1. 從起始點導航至門前。
    2. 用視覺辨識定位門把的精確位置。
    3. 操作機械臂/機械爪「解鎖」門把（+5分）。
    4. 持續施力，將門推開至規定角度（+5分）。

注意事項：
    - 碰到牆壁或障礙物 = 扣分！導航到門前要精確。
    - 門把的視覺辨識是最大挑戰，需要和 YOLO 模型確認如何偵測。
    - 規則中「規定的角度」需要向主辦方確認。

啟動方式（在 Docker 容器內）：
    ros2 run door_opener door_opener_node
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from wildbot_common.navigation_client import NavigationClient
from wildbot_common.gripper_controller import GripperController
from wildbot_common.robot_state import DoorOpenerState, BASE_POSITION, DOOR_POSITION


class DoorOpenerNode(Node):
    """
    開門任務節點。

    執行流程：導航至門前 → 視覺定位門把 → 對準 → 解鎖 → 推門。
    """

    # 【TODO - 組員 C】：確認 YOLO 偵測門把的 Topic 名稱
    # 這可能需要另外訓練一個「門把」的 YOLO 模型，或使用深度攝影機
    DOOR_HANDLE_TOPIC = '/yolo/door_handle'

    # 推門的速度（m/s）：慢速穩定地推
    PUSH_SPEED = 0.08

    # 【TODO - 組員 C】：向主辦方確認門需要推開的角度
    # 然後根據角度換算車子需要推進的距離
    PUSH_DURATION_SEC = 3.0  # 持續推門的秒數（暫定）

    def __init__(self):
        super().__init__('door_opener_node')
        self.get_logger().info('=== 開門任務節點啟動 ===')

        # 共用模組
        self.nav = NavigationClient(self)
        self.gripper = GripperController(self)

        # 直接速度控制（推門用）
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 狀態機
        self.state = DoorOpenerState.IDLE
        self.door_handle_position = None  # 門把的世界座標

        # 訂閱 YOLO 偵測結果
        self.handle_sub = self.create_subscription(
            Float32MultiArray,
            self.DOOR_HANDLE_TOPIC,
            self._on_handle_detected,
            10
        )

        # 主控制迴圈
        self.timer = self.create_timer(0.5, self._state_machine_loop)
        self.get_logger().info('初始化完成，等待任務開始...')

    def _on_handle_detected(self, msg):
        """接收 YOLO 偵測到的門把位置。"""
        # 【TODO - 組員 C】：解析出門把的世界座標或像素座標
        if len(msg.data) >= 2:
            self.door_handle_position = (msg.data[0], msg.data[1])

    def _push_door(self, duration_sec: float):
        """
        持續向前推門一段時間。
        推門時不使用 Nav2（避免 Nav2 認為前方有障礙物而停止）。
        """
        self.get_logger().info(f'開始推門，持續 {duration_sec} 秒...')
        twist = Twist()
        twist.linear.x = self.PUSH_SPEED
        end_time = time.time() + duration_sec
        # 注意：這裡使用 time.sleep 會阻塞節點，
        # 【TODO - 組員 C】：實際使用時應改成非阻塞的計時器控制
        while time.time() < end_time:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        # 停止
        twist.linear.x = 0.0
        self._cmd_vel_pub.publish(twist)
        self.get_logger().info('推門動作完成。')

    def _state_machine_loop(self):
        """主狀態機迴圈。"""

        if self.state == DoorOpenerState.IDLE:
            self.get_logger().info('狀態：IDLE → 準備導航至門前...')
            self.state = DoorOpenerState.NAVIGATING_TO_DOOR

        elif self.state == DoorOpenerState.NAVIGATING_TO_DOOR:
            self.get_logger().info('導航至門前...')
            success = self.nav.go_to(
                x=DOOR_POSITION['x'],
                y=DOOR_POSITION['y'],
                yaw=DOOR_POSITION['yaw']
            )
            if success:
                self.get_logger().info('已抵達門前！開始偵測門把...')
                self.state = DoorOpenerState.DETECTING_HANDLE
            else:
                self.get_logger().error('無法抵達門前，重試...')

        elif self.state == DoorOpenerState.DETECTING_HANDLE:
            if self.door_handle_position is not None:
                self.get_logger().info(f'偵測到門把！位置：{self.door_handle_position}')
                self.state = DoorOpenerState.ALIGNING_TO_HANDLE
            else:
                self.get_logger().info('偵測門把中...')
                # 【TODO - 組員 C】：如果找不到，考慮微調車子角度

        elif self.state == DoorOpenerState.ALIGNING_TO_HANDLE:
            # 【TODO - 組員 C】：
            # 這是開門任務最關鍵的一步！
            # 需要精確對準門把，讓機械爪能夠正確扣住並轉動。
            # 建議方案：
            # - 使用深度攝影機計算門把的 3D 位置
            # - 微調車子的 X/Y 位置和朝向
            # - 確認機械爪的高度與門把齊平（可能需要調整爪子高度）
            self.get_logger().info('對準門把中（TODO：實作精確對準）...')
            self.state = DoorOpenerState.UNLOCKING

        elif self.state == DoorOpenerState.UNLOCKING:
            # 【TODO - 組員 C】：
            # 解鎖動作取決於門把的類型：
            # - 如果是旋轉式門把：控制機械爪旋轉
            # - 如果是按壓式門把：控制機械爪下壓
            # 目前先用 close（夾緊）模擬解鎖動作
            self.get_logger().info('執行解鎖動作...')
            self.gripper.close()  # 夾住門把
            time.sleep(1.0)
            # 【TODO - 組員 C】：加入旋轉/按壓的控制指令
            self.get_logger().info('解鎖完成！（+5分！）開始推門...')
            self.state = DoorOpenerState.PUSHING_DOOR

        elif self.state == DoorOpenerState.PUSHING_DOOR:
            self._push_door(self.PUSH_DURATION_SEC)
            self.get_logger().info('推門完成！（+5分！）任務結束！')
            self.state = DoorOpenerState.DONE

        elif self.state == DoorOpenerState.DONE:
            self.get_logger().info('開門任務全部完成！')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = DoorOpenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('節點被手動停止。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
