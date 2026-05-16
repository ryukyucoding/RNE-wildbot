"""
bear_grabber / grab_and_return_node.py
【負責人：組員 A】

任務目標：
    在第一階段（8分鐘），反覆執行以下迴圈：
    1. 用 YOLO 辨識找到場地上的熊。
    2. 導航過去，夾起熊（+5分）。
    3. 帶熊回基地放下（+5分）。
    4. 重複，盡量多抓幾隻。

計分：
    - 每次成功夾起：+5 分
    - 每次成功送回：+5 分
    - 碰到障礙物：-1 分（一定要靠 Nav2 避障！）
    - 掉落物件：-1 分（最多 -3 分）

啟動方式（在 Docker 容器內）：
    ros2 run bear_grabber bear_grabber_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from wildbot_common.navigation_client import NavigationClient
from wildbot_common.gripper_controller import GripperController
from wildbot_common.robot_state import BearGrabberState, BASE_POSITION


class BearGrabberNode(Node):
    """
    平地夾熊任務節點。

    訂閱 YOLO 偵測結果，找到熊的位置後導航過去並夾取，
    再帶回基地放下。重複執行直到時間結束。
    """

    # 【TODO - 組員 A】：確認 YOLO 輸出的 Topic 名稱
    BEAR_DETECTION_TOPIC = '/yolo/detections'

    def __init__(self):
        super().__init__('bear_grabber_node')
        self.get_logger().info('=== 平地夾熊任務節點啟動 ===')

        # 共用模組初始化
        self.nav = NavigationClient(self)
        self.gripper = GripperController(self)

        # 狀態機初始狀態
        self.state = BearGrabberState.IDLE
        self.current_bear_position = None  # (x, y) 或 None

        # 訂閱 YOLO 偵測結果
        # 【TODO - 組員 A】：確認訊息格式後修改 msg type
        self.bear_sub = self.create_subscription(
            Float32MultiArray,
            self.BEAR_DETECTION_TOPIC,
            self._on_bear_detected,
            10
        )

        # 主控制迴圈（每 0.5 秒執行一次狀態機）
        self.timer = self.create_timer(0.5, self._state_machine_loop)

        self.get_logger().info('初始化完成，等待任務開始...')

    def _on_bear_detected(self, msg):
        """
        接收 YOLO 偵測到的熊的位置。

        【TODO - 組員 A】：
            根據 ros2_yolo_integration 實際發布的訊息格式，
            解析出熊的世界座標 (x, y) 並存入 self.current_bear_position。

            目前使用 Float32MultiArray 作為佔位符，
            假設格式為 [x, y, confidence]。
        """
        if len(msg.data) >= 2:
            x, y = msg.data[0], msg.data[1]
            self.current_bear_position = (x, y)
            # self.get_logger().debug(f'偵測到熊：({x:.2f}, {y:.2f})')

    def _state_machine_loop(self):
        """主狀態機迴圈，每 0.5 秒執行一次。"""

        if self.state == BearGrabberState.IDLE:
            self.get_logger().info('狀態：IDLE → 開始搜尋熊...')
            self.state = BearGrabberState.SEARCHING_BEAR

        elif self.state == BearGrabberState.SEARCHING_BEAR:
            if self.current_bear_position is not None:
                self.get_logger().info(
                    f'找到熊！位置：{self.current_bear_position}，開始導航...'
                )
                self.state = BearGrabberState.NAVIGATING_TO_BEAR
            else:
                self.get_logger().info('搜尋中...尚未偵測到熊')
                # 【TODO - 組員 A】：
                # 如果一直找不到，可以讓車子緩慢旋轉以擴大搜尋範圍。
                # 範例：self.nav.go_to(x=0.5, y=0.0, yaw=0.5)

        elif self.state == BearGrabberState.NAVIGATING_TO_BEAR:
            if self.current_bear_position is None:
                self.get_logger().warn('導航中途失去熊的位置，重新搜尋')
                self.state = BearGrabberState.SEARCHING_BEAR
                return

            x, y = self.current_bear_position
            success = self.nav.go_to(x=x, y=y)

            if success:
                self.get_logger().info('已抵達熊的附近，準備對準...')
                self.state = BearGrabberState.ALIGNING_TO_BEAR
            else:
                self.get_logger().error('導航失敗，重試...')
                self.state = BearGrabberState.SEARCHING_BEAR

        elif self.state == BearGrabberState.ALIGNING_TO_BEAR:
            # 【TODO - 組員 A】：
            # 在這裡實作更精確的對準邏輯，例如：
            # - 用攝影機深度資訊微調距離
            # - 確保機械爪正對著熊
            self.get_logger().info('對準完成（TODO：實作精確對準），執行夾取...')
            self.state = BearGrabberState.GRABBING

        elif self.state == BearGrabberState.GRABBING:
            self.gripper.grab_sequence()
            self.current_bear_position = None  # 清除位置，下次重新搜尋
            self.get_logger().info('夾取完成！（+5分！）導航回基地...')
            self.state = BearGrabberState.RETURNING_TO_BASE

        elif self.state == BearGrabberState.RETURNING_TO_BASE:
            success = self.nav.go_to(
                x=BASE_POSITION['x'],
                y=BASE_POSITION['y'],
                yaw=BASE_POSITION['yaw']
            )
            if success:
                self.get_logger().info('已回到基地，準備放下熊...')
                self.state = BearGrabberState.RELEASING
            else:
                self.get_logger().error('回基地失敗，重試...')

        elif self.state == BearGrabberState.RELEASING:
            self.gripper.release_sequence()
            self.get_logger().info('放下熊完成！（+5分！）準備下一輪...')
            self.state = BearGrabberState.DONE

        elif self.state == BearGrabberState.DONE:
            self.get_logger().info('本輪完成，重新開始搜尋...')
            self.state = BearGrabberState.SEARCHING_BEAR


def main(args=None):
    rclpy.init(args=args)
    node = BearGrabberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('節點被手動停止。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
