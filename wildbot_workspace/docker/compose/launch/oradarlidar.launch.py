from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory  # 加入這一行
from ament_index_python.packages import get_package_share_directory  # 加入這一行
import os

def generate_launch_description():
    # 找到 oradar_lidar package 內的 launch 檔案
    ms200_scan_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch', 'ms200_scan.launch.py'
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ms200_scan_launch),
        launch_arguments={
            'angle_min': '0.1',
            'angle_max': '350.9',
            'range_max': '12.0',
            'scan_topic': '/scan_tmp'
        }.items()
    )

    # 設定 laser 到 base_link 的靜態 TF
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.165', '0', '0.18', '0', '0', '0', 'base_link','laser'],
    )

    return LaunchDescription([
        lidar_driver,
        tf2_node,
    ])
