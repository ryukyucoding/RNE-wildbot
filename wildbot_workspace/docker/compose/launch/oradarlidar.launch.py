from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ms200_scan_launch = os.path.join(
        get_package_share_directory("oradar_lidar"),
        "launch",
        "ms200_scan.launch.py",
    )

    # MS200 driver publishes /scan by default; remap to /scan_tmp so only
    # scan_sanitize publishes /scan (slam_toolbox needs fixed-length scans).
    lidar_driver = GroupAction(
        actions=[
            SetRemap(src="/scan", dst="/scan_tmp"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ms200_scan_launch),
                launch_arguments={
                    "angle_min": "0.1",
                    "angle_max": "350.9",
                    "range_max": "12.0",
                    "scan_topic": "/scan_tmp",
                }.items(),
            ),
        ]
    )

    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_lidar",
        arguments=["0.165", "0", "0.18", "0", "0", "0", "base_link", "lidar"],
    )

    return LaunchDescription([lidar_driver, tf2_node])
