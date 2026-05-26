from rclpy.node import Node
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from std_msgs.msg import String, Header
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan, Imu, CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped
import orjson
import time
from pros_car_py.ros_communicator_config import ACTION_MAPPINGS
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
try:
    from nav2_msgs.srv import ClearEntireCostmap
    from nav2_msgs.action import NavigateToPose
except ImportError:
    ClearEntireCostmap = None
    NavigateToPose = None
from rclpy.action import ActionClient
import rclpy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class RosCommunicator(Node):
    def __init__(self, node_name: str = "RosCommunicator"):
        super().__init__(node_name)

        self.declare_parameter("amcl_pose_topic", "/amcl_pose")

        # subscribe amcl_pose (topic 可用 launch / ros-args remap 或參數覆寫)
        self.latest_amcl_pose = None
        amcl_topic = (
            self.get_parameter("amcl_pose_topic").get_parameter_value().string_value
        )
        # Nav2 AMCL 通常為 RELIABLE + TRANSIENT_LOCAL（latched pose）。
        # 若用 BEST_EFFORT + VOLATILE 會與 AMCL 不相容，訂閱端永遠收不到 pose。
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscriber_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self.subscriber_amcl_callback,
            amcl_qos,
        )
        self.get_logger().info(
            f"Subscribing PoseWithCovarianceStamped on '{amcl_topic}' (localization AMCL)."
        )

        # subscribe /odom（wheel odometry，用於記錄起點與回程導航）
        self.latest_odom = None
        self.subscriber_odom = self.create_subscription(
            Odometry, "/base_controller/odom", self._odom_callback, 10
        )

        # subscribe goal_pose
        self.latest_goal_pose = None
        self.target_pose = None
        self.subscriber_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.subscriber_goal_callback, 10
        )

        # subscribe lidar
        self.latest_lidar = None
        self._lidar_missing_warn_time = 0.0
        self.subscriber_lidar = self.create_subscription(
            LaserScan, "/scan", self.subscriber_lidar_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # subscribe global_plan
        self.latest_received_global_plan = None
        self.subscriber_received_global_plan = self.create_subscription(
            Path, "/received_global_plan", self.received_global_plan_callback, 1
        )

        # Subscribe to YOLO detected object coordinates
        self.latest_yolo_coordinates = None
        self.subscriber_yolo_detection_position = self.create_subscription(
            PointStamped,
            "/yolo/detection/position",
            self.yolo_detection_position_callback,
            10,
        )

        # Subscribe to YOLO detected object coordinates
        self.latest_yolo_offset = None
        self.subscriber_yolo_offset = self.create_subscription(
            PointStamped,
            "/yolo/detection/offset",
            self.yolo_detection_offset_callback,
            10,
        )

        self.latest_yolo_detection_status = None
        self.subscriber_yolo_detection_status = self.create_subscription(
            Bool, "/yolo/detection/status", self.yolo_detection_status_callback, 10
        )

        self.latest_imu_data = None
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.imu_data_callback, 10
        )

        self.latest_mediapipe_data = None
        self.mediapipe_sub = self.create_subscription(
            Point, "/mediapipe_data", self.mediapipe_data_callback, 10
        )

        self.latest_yolo_target_info = None
        self.yolo_target_info_sub = self.create_subscription(
            Float32MultiArray, "/yolo/target_info", self.yolo_target_info_callback, 1
        )

        self.latest_camera_x_multi_depth = None
        self.camera_x_multi_depth_sub = self.create_subscription(
            Float32MultiArray,
            "/camera/x_multi_depth_values",
            self.camera_x_multi_depth_callback,
            10,
        )

        self.latest_obstacle_sector_depth = None
        self.obstacle_sector_depth_sub = self.create_subscription(
            Float32MultiArray,
            "/obstacle/sector_min_depth",
            self.obstacle_sector_depth_callback,
            10,
        )

        self.latest_cmd_vel = None
        self.subscriber_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self.subscriber_cmd_vel_callback, 1
        )

        # publish cmd_vel to real robot base controller
        self.publisher_cmd_vel = self.create_publisher(
            TwistStamped, "/base_controller/cmd_vel", 1
        )

        # publish goal_pose
        self.publisher_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # publish joint trajectory to real robot arm controller
        self.publisher_joint_trajectory = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

        self.publisher_coordinates = self.create_publisher(
            PointStamped, "/coordinates", 10
        )

        self.publisher_target_label = self.create_publisher(String, "/target_label", 10)

        self.crane_state_publisher = self.create_publisher(String, "crane_state", 10)

        self.publisher_confirmed_path = self.create_publisher(
            Path, "/confirmed_initial_plan", 10
        )

        self.publisher_target_marker = self.create_publisher(
            Marker, "/selected_target_marker", 10
        )

        # 創清除 costmap Service（Nav2 不可用時跳過）
        if ClearEntireCostmap is not None:
            self.clear_global_costmap_client = self.create_client(
                ClearEntireCostmap, "/global_costmap/clear"
            )
            self.clear_local_costmap_client = self.create_client(
                ClearEntireCostmap, "/local_costmap/clear"
            )
        else:
            self.clear_global_costmap_client = None
            self.clear_local_costmap_client = None

        self.publisher_received_global_plan = self.create_publisher(
            Path, "/received_global_plan", 10
        )
        self.publisher_plan = self.create_publisher(Path, "/plan", 10)

        self.navigate_to_pose_action_client = (
            ActionClient(self, NavigateToPose, "/navigate_to_pose")
            if NavigateToPose is not None
            else None
        )

        # ======== 在 __init__ 裡面新增 ========
        # 訂閱 YOLO 算出的目標 3D 位置 Marker
        self.latest_yolo_marker = None
        self.subscriber_yolo_marker = self.create_subscription(
            Marker, "/yolo/target_marker", self.yolo_target_marker_callback, 10
        )

        self.cv_image = None
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_callback, 1
        )
        
        # 發布手臂關節視覺化線條
        self.publisher_arm_visual = self.create_publisher(
            Marker, "/arm_visual_lines", 10
        )
        
        self.subscriber_clicked_point = self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_point_callback, 10
        )

        self.marker_pub = self.create_publisher(Marker, "/clicked_point_marker", 1)
    
    def image_callback(self, msg):
        """接收影像並進行物體檢測"""
        # 將 ROS 影像消息轉換為 OpenCV 格式
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

    # 新增 callback
    def clicked_point_callback(self, msg):
        # 為了偷懶，我們直接把它偽裝成 YOLO marker 塞給系統
        mock_marker = Marker()
        mock_marker.header = msg.header
        mock_marker.header.stamp = self.get_clock().now().to_msg()
        mock_marker.ns = 'yolo_target'
        mock_marker.id = 0
        mock_marker.type = Marker.SPHERE
        mock_marker.action = Marker.ADD
        mock_marker.pose.position = msg.point
        mock_marker.pose.orientation.w = 1.0
        mock_marker.scale.x = 0.08  # 網球大小約 8 公分
        mock_marker.scale.y = 0.08
        mock_marker.scale.z = 0.08
        mock_marker.color.a = 1.0   # 不透明度
        mock_marker.color.r = 0.8   # 螢光黃/綠色
        mock_marker.color.g = 1.0
        mock_marker.color.b = 0.0
        self.latest_yolo_marker = mock_marker
        self.marker_pub.publish(mock_marker)

    # ======== 在 class 內新增這兩個函式 ========
    def yolo_target_marker_callback(self, msg):
        self.latest_yolo_marker = msg

    def publish_arm_visual_lines(self, marker_msg):
        self.publisher_arm_visual.publish(marker_msg)

    def clear_received_global_plan(self):
        """
        清空 /received_global_plan 话题
        """
        empty_path = Path()
        empty_path.header.frame_id = "map"
        self.publisher_received_global_plan.publish(empty_path)
        self.get_logger().info("Published empty Path to /received_global_plan")

    def clear_plan(self):
        """
        清空 /plan 话题
        """
        empty_path = Path()
        empty_path.header.frame_id = "map"
        self.publisher_plan.publish(empty_path)
        self.get_logger().info("Published empty Path to /plan")

    def reset_nav2(self):
        """
        clear plan
        """
        self.clear_received_global_plan()
        self.clear_plan()
        self.get_logger().info("Nav2 Reset Completed")

    # amcl_pose callback and get_latest_amcl_pose
    def subscriber_amcl_callback(self, msg):
        self.latest_amcl_pose = msg

    def get_latest_amcl_pose(self):
        # 勿在此對每次呼叫 warn（bear_mission 輪詢會洗版）；缺資料由上層限頻記錄
        return self.latest_amcl_pose

    # odom callback and getter
    def _odom_callback(self, msg):
        self.latest_odom = msg

    def get_latest_odom(self):
        return self.latest_odom

    # goal callback and get_latest_goal
    def subscriber_goal_callback(self, msg):
        self.latest_goal_pose = msg.pose
        position = msg.pose.position
        target = [position.x, position.y, position.z]
        self.target_pose = target

    def get_goal_pose(self):
        """提供給 nav_processing 索取完整的目標姿態"""
        return self.latest_goal_pose

    def get_latest_goal(self):
        if self.target_pose is None:
            self.get_logger().warn("No goal pose data received yet.")
        return self.target_pose

    # lidar callback and get_latest_lidar
    def subscriber_lidar_callback(self, msg):
        self.latest_lidar = msg

    def get_latest_lidar(self):
        if self.latest_lidar is None:
            now = time.monotonic()
            if now - self._lidar_missing_warn_time >= 5.0:
                self._lidar_missing_warn_time = now
                self.get_logger().warn(
                    "No LiDAR on /scan yet — check localization stack, "
                    "ROS_DOMAIN_ID, and Docker bridge."
                )
        return self.latest_lidar

    # received_global_plan callback and get_latest_received_global_plan
    def received_global_plan_callback(self, msg):
        self.latest_received_global_plan = msg

    def get_latest_received_global_plan(self):
        if self.latest_received_global_plan is None:
            self.get_logger().warn("No received global plan data received yet.")
            return None
        return self.latest_received_global_plan
    
    def subscriber_cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def get_latest_cmd_vel(self):
        return self.latest_cmd_vel

    # 4. 新增一個「直接發布數值」的方法 (繞過 ACTION_MAPPINGS)
    def publish_raw_car_control(self, velocities, publish_rear=True, publish_front=True):
        """
        直接發布速度指令。
        支援兩種格式：
          - 4 元素 [RL, RR, FL, FR]：舊版四輪速度，自動換算成 TwistStamped
          - 2 元素 (linear_x, angular_z)：直接填入 TwistStamped
        """
        if len(velocities) == 4:
            # 舊格式換算：左輪 = 前後左平均，右輪 = 前後右平均
            # [RL, RR, FL, FR]
            left = (float(velocities[0]) + float(velocities[2])) / 2.0
            right = (float(velocities[1]) + float(velocities[3])) / 2.0
            # 以舊系統最大輪速 450（= 9.0 * speed_ratio 50）為基準正規化
            OLD_MAX = 450.0
            linear_x = (left + right) / 2.0 / OLD_MAX * 0.546
            angular_z = (right - left) / OLD_MAX * 3.983
        else:
            linear_x = float(velocities[0])
            angular_z = float(velocities[1])

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.publisher_cmd_vel.publish(msg)

    def publish_car_control(self, action_key, publish_rear=True, publish_front=True):
        if action_key not in ACTION_MAPPINGS:
            return
        linear_x, angular_z = ACTION_MAPPINGS[action_key]
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher_cmd_vel.publish(msg)

    # publish goal_pose
    def publish_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.publisher_goal_pose.publish(goal_pose)

    # publish robot arm angle
    def publish_robot_arm_angle(self, angle):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["arm_1_joint", "arm_2_joint", "gripper_joint"]
        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angle]
        point.time_from_start.sec = 1
        msg.points = [point]
        self.publisher_joint_trajectory.publish(msg)

    def publish_coordinates(self, x, y, z, frame_id="map"):
        coordinate_msg = PointStamped()
        coordinate_msg.header.stamp = self.get_clock().now().to_msg()
        coordinate_msg.header.frame_id = frame_id
        coordinate_msg.point.x = x
        coordinate_msg.point.y = y
        coordinate_msg.point.z = z
        self.publisher_coordinates.publish(coordinate_msg)

    def mediapipe_data_callback(self, msg):
        self.latest_mediapipe_data = msg

    def get_latest_mediapipe_data(self):
        if self.latest_mediapipe_data is None:
            self.get_logger().warn("No Mediapipe data received yet.")
            return None
        return self.latest_mediapipe_data

    def yolo_target_info_callback(self, msg):
        self.latest_yolo_target_info = msg

    def get_latest_yolo_target_info(self):
        if self.latest_yolo_target_info is None:
            return None
        return self.latest_yolo_target_info

    def camera_x_multi_depth_callback(self, msg):
        self.latest_camera_x_multi_depth = msg

    def get_latest_camera_x_multi_depth(self):
        if self.latest_camera_x_multi_depth is None:
            return None
        return self.latest_camera_x_multi_depth

    def obstacle_sector_depth_callback(self, msg):
        self.latest_obstacle_sector_depth = msg

    def get_latest_obstacle_sector_depth(self):
        if self.latest_obstacle_sector_depth is None:
            return None
        return self.latest_obstacle_sector_depth

    # YOLO coordinates callback
    def yolo_detection_position_callback(self, msg):
        """Callback to receive YOLO detected object coordinates."""
        self.latest_yolo_coordinates = msg

    def get_latest_yolo_detection_position(self):
        """Getter for the latest YOLO detected object coordinates."""
        if self.latest_yolo_coordinates is None:
            return None
        return self.latest_yolo_coordinates

    def yolo_detection_offset_callback(self, msg):
        self.latest_yolo_offset = msg

    def get_latest_yolo_detection_offset(self):
        if self.latest_yolo_offset is None:
            return None
        return self.latest_yolo_offset

    def publish_target_label(self, label):
        target_label_msg = String()
        target_label_msg.data = label
        self.publisher_target_label.publish(target_label_msg)

    # 天車
    def publish_crane_state(self, state):
        control_signal = {"type": "crane", "data": dict(crane_state=state)}
        crane_state_msg = String()
        crane_state_msg.data = orjson.dumps(control_signal).decode()
        self.crane_state_publisher.publish(crane_state_msg)

    def yolo_detection_status_callback(self, msg):
        self.latest_yolo_detection_status = msg

    def get_latest_yolo_detection_status(self):
        if self.latest_yolo_detection_status is None:
            return None
        return self.latest_yolo_detection_status

    def imu_data_callback(self, msg):
        self.latest_imu_data = msg

    def get_latest_imu_data(self):
        if self.latest_imu_data is None:
            return None
        return self.latest_imu_data

    def publish_confirmed_initial_plan(self, path_msg: Path):
        """
        確認路徑使用
        """
        self.publisher_confirmed_path.publish(path_msg)

    def publish_selected_target_marker(self, x, y, z=0.0):
        """
        在 foxglove 畫紅點
        """
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.2  # 球體大小
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # 透明度
        marker.color.r = 1.0  # 顏色
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.publisher_target_marker.publish(marker)
