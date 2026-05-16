"""
wildbot_common / navigation_client.py

封裝 Nav2 SimpleNavigator，提供簡單的「導航到指定座標」功能。
所有任務節點（bear_grabber, bridge_mission, door_opener）都應該透過此模組控制車子移動，
而不是直接發 /cmd_vel，這樣才能享有 Nav2 的避障功能（碰到障礙物 = 扣分！）。

使用方式（在你的 Node 裡）：
    from wildbot_common.navigation_client import NavigationClient
    self.nav = NavigationClient(self)
    self.nav.go_to(x=1.0, y=2.0, yaw=0.0)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    """將 yaw 角度（弧度）轉換為四元數。"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class NavigationClient:
    """
    簡化版的 Nav2 導航客戶端。

    Args:
        node: 你的 ROS 2 Node 實例 (self)
    """

    def __init__(self, node: Node):
        self.node = node
        self._action_client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose'
        )
        node.get_logger().info('[NavigationClient] 初始化完成，等待 Nav2 服務...')

    def go_to(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map') -> bool:
        """
        導航到指定的地圖座標。

        Args:
            x:        目標 X 座標（公尺）
            y:        目標 Y 座標（公尺）
            yaw:      抵達後的朝向角度（弧度，0 = 面向 X 軸正方向）
            frame_id: 座標系，通常為 'map'

        Returns:
            True 表示成功到達，False 表示失敗或被中途取消。
        """
        # 等待 Nav2 Action Server 就緒
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('[NavigationClient] Nav2 Action Server 未就緒！請確認 pros_app 已啟動。')
            return False

        # 建立目標
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        goal_msg.pose = pose

        self.node.get_logger().info(f'[NavigationClient] 導航至 ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°')

        # 發送目標並同步等待結果
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('[NavigationClient] 目標被 Nav2 拒絕！')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        self.node.get_logger().info('[NavigationClient] 已抵達目標！')
        return True

    def cancel(self):
        """取消目前的導航任務。"""
        self.node.get_logger().info('[NavigationClient] 取消導航。')
        self._action_client._cancel_goal_async()
