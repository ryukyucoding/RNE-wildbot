"""
wildbot_common / gripper_controller.py

封裝機械爪的控制邏輯。
透過發布標準 ROS 2 訊息來控制機械爪，與底層硬體實作解耦。

使用方式（在你的 Node 裡）：
    from wildbot_common.gripper_controller import GripperController
    self.gripper = GripperController(self)
    self.gripper.open()
    self.gripper.close()
    self.gripper.lift()
    self.gripper.lower()

【TODO - 組員 D（硬體整合）】：
    確認機械爪實際使用的 Topic 名稱與訊息格式後，
    修改下方的 GRIPPER_TOPIC 與訊息類型。
    目前使用 std_msgs/String 作為佔位符。
"""

import time
from rclpy.node import Node
from std_msgs.msg import String


# 【TODO - 組員 D】：確認後修改此 Topic 名稱
GRIPPER_TOPIC = '/wildbot/gripper/command'

# 控制指令字串（與韌體端約定好的格式）
# 【TODO - 組員 D】：確認後修改這些指令字串
CMD_OPEN  = 'OPEN'
CMD_CLOSE = 'CLOSE'
CMD_LIFT  = 'LIFT'
CMD_LOWER = 'LOWER'


class GripperController:
    """
    機械爪控制器。

    Args:
        node: 你的 ROS 2 Node 實例 (self)
    """

    def __init__(self, node: Node):
        self.node = node
        self._pub = node.create_publisher(String, GRIPPER_TOPIC, 10)
        node.get_logger().info(f'[GripperController] 初始化完成，發布至 {GRIPPER_TOPIC}')

    def _send(self, cmd: str, delay_sec: float = 1.5):
        """發送指令並等待機構動作完成。"""
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)
        self.node.get_logger().info(f'[GripperController] 發送指令: {cmd}')
        # 等待機構動作完成
        time.sleep(delay_sec)

    def open(self):
        """張開機械爪（準備夾取）。"""
        self._send(CMD_OPEN)

    def close(self):
        """夾緊機械爪（夾住物件）。"""
        self._send(CMD_CLOSE)

    def lift(self):
        """舉起機械爪（將物件抬離地面）。"""
        self._send(CMD_LIFT)

    def lower(self):
        """放下機械爪。"""
        self._send(CMD_LOWER)

    def grab_sequence(self):
        """
        完整的夾取動作序列：張開 → 夾緊 → 舉起。
        呼叫前請確保車子已停在目標物正前方。
        """
        self.node.get_logger().info('[GripperController] 開始夾取序列...')
        self.open()
        self.close()
        self.lift()
        self.node.get_logger().info('[GripperController] 夾取序列完成！')

    def release_sequence(self):
        """
        完整的放下動作序列：放下 → 張開。
        呼叫前請確保車子已回到目標放置位置。
        """
        self.node.get_logger().info('[GripperController] 開始放置序列...')
        self.lower()
        self.open()
        self.node.get_logger().info('[GripperController] 放置序列完成！')
