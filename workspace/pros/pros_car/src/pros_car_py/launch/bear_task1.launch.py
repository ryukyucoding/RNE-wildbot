"""
僅啟動 bear_mission（不依賴 yolo_example_pkg）。

請先在 ros2_yolo_integration 容器／環境執行 yolo_node，並確認 ROS_DOMAIN_ID 與網路一致。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_unity_nav = LaunchConfiguration("use_unity_camera_nav")
    auto_delay = LaunchConfiguration("auto_start_delay_sec")
    amcl_wait = LaunchConfiguration("amcl_wait_timeout_sec")
    amcl_topic = LaunchConfiguration("amcl_pose_topic")
    visual_servo_enabled = LaunchConfiguration("visual_servo_enabled")
    visual_servo_target_depth_m = LaunchConfiguration("visual_servo_target_depth_m")
    align_pixel_thresh = LaunchConfiguration("align_pixel_thresh")
    obstacle_guard_enabled = LaunchConfiguration("obstacle_guard_enabled")
    obstacle_stop_m = LaunchConfiguration("obstacle_stop_m")
    obstacle_slow_m = LaunchConfiguration("obstacle_slow_m")
    nav_obstacle_guard_enabled = LaunchConfiguration("nav_obstacle_guard_enabled")
    unity_stow_elbow_deg = LaunchConfiguration("unity_stow_elbow_deg")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_unity_camera_nav",
                default_value="true",
                description="Unity=true 使用較遠障礙閾值；實車請設 false",
            ),
            DeclareLaunchArgument(
                "auto_start_delay_sec",
                default_value="5.0",
                description="延遲再開始任務（等 localization／對側 YOLO）",
            ),
            DeclareLaunchArgument(
                "amcl_wait_timeout_sec",
                default_value="120.0",
                description="等待第一筆 AMCL/pose 的最長秒數",
            ),
            DeclareLaunchArgument(
                "amcl_pose_topic",
                default_value="/amcl_pose",
                description="須與 localization 發布的 PoseWithCovarianceStamped 話題一致",
            ),
            DeclareLaunchArgument(
                "visual_servo_enabled",
                default_value="true",
                description="接近目標時使用像素誤差+深度 PID 視覺伺服",
            ),
            DeclareLaunchArgument(
                "visual_servo_target_depth_m",
                default_value="0.55",
                description="視覺伺服目標距離（公尺）；Unity 建議略遠避免 YOLO 太近失效",
            ),
            DeclareLaunchArgument(
                "align_pixel_thresh",
                default_value="40.0",
                description="夾取前允許的最大水平像素偏差（越小越要求置中）",
            ),
            DeclareLaunchArgument(
                "obstacle_guard_enabled",
                default_value="true",
                description="接近目標時啟用 LiDAR+深度避障護欄",
            ),
            DeclareLaunchArgument(
                "obstacle_stop_m",
                default_value="-1.0",
                description="急停距離（公尺）；-1 依 use_unity_camera_nav 自動",
            ),
            DeclareLaunchArgument(
                "obstacle_slow_m",
                default_value="-1.0",
                description="減速帶距離（公尺）；-1 自動",
            ),
            DeclareLaunchArgument(
                "nav_obstacle_guard_enabled",
                default_value="true",
                description="Nav2 回程時監看前方障礙並取消 goal",
            ),
            DeclareLaunchArgument(
                "unity_stow_elbow_deg",
                default_value="180.0",
                description="Unity 任務開始前 Elbow 角度（最低約 180°，避免夾爪擋住相機）",
            ),
            Node(
                package="pros_car_py",
                executable="bear_mission",
                name="bear_mission",
                output="screen",
                parameters=[
                    {
                        "auto_start": True,
                        "auto_start_delay_sec": auto_delay,
                        "use_unity_camera_nav": use_unity_nav,
                        "amcl_wait_timeout_sec": amcl_wait,
                        "amcl_pose_topic": amcl_topic,
                        "visual_servo_enabled": visual_servo_enabled,
                        "visual_servo_target_depth_m": visual_servo_target_depth_m,
                        "align_pixel_thresh": align_pixel_thresh,
                        "align_stable_frames": 8,
                        "visual_servo_far_distance_m": 0.90,
                        "visual_servo_max_forward_speed_far": 300.0,
                        "visual_servo_max_yaw_near": 175.0,
                        "visual_servo_max_yaw_far": 300.0,
                        "visual_servo_search_spin_speed": 130.0,
                        "visual_servo_min_yaw_large_px": 155.0,
                        "visual_servo_yaw_deadband_px": 12.0,
                        "visual_servo_center_deadband_px": 32.0,
                        "align_pixel_bias_px": 0.0,
                        "obstacle_guard_enabled": obstacle_guard_enabled,
                        "obstacle_stop_m": obstacle_stop_m,
                        "obstacle_slow_m": obstacle_slow_m,
                        "nav_obstacle_guard_enabled": nav_obstacle_guard_enabled,
                        "unity_stow_elbow_enabled": use_unity_nav,
                        "unity_stow_elbow_deg": unity_stow_elbow_deg,
                    }
                ],
            ),
        ]
    )
