"""Launch the real-car door_open task."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    length_m = LaunchConfiguration("length_m")
    width_m = LaunchConfiguration("width_m")
    turn_deg = LaunchConfiguration("turn_deg")
    yolo_target_label = LaunchConfiguration("yolo_target_label")
    vs_stop_distance = LaunchConfiguration("vs_stop_distance")
    press_elbow_drop_deg = LaunchConfiguration("press_elbow_drop_deg")
    open_door_duration = LaunchConfiguration("open_door_duration")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "length_m",
                default_value="0.85",
                description="Rectangle-drive length reference; pre-approach moves 2x this distance.",
            ),
            DeclareLaunchArgument(
                "width_m",
                default_value="2.95",
                description="Rectangle-drive width reference; pre-approach moves 0.5x this distance.",
            ),
            DeclareLaunchArgument(
                "turn_deg",
                default_value="48.5",
                description="Odom-based turn angle used for the right then left pre-approach turns.",
            ),
            DeclareLaunchArgument(
                "yolo_target_label",
                default_value="knob",
                description="YOLO class requested through /target_label.",
            ),
            DeclareLaunchArgument(
                "vs_stop_distance",
                default_value="0.30",
                description="Distance in meters where visual servoing stops before arm motion.",
            ),
            DeclareLaunchArgument(
                "press_elbow_drop_deg",
                default_value="30.0",
                description="Internal elbow-angle drop used to press the handle down.",
            ),
            DeclareLaunchArgument(
                "open_door_duration",
                default_value="2.5",
                description="Seconds to push forward after pressing the handle.",
            ),
            Node(
                package="pros_car_py",
                executable="door_open",
                name="door_open",
                output="screen",
                parameters=[
                    {
                        "length_m": ParameterValue(length_m, value_type=float),
                        "width_m": ParameterValue(width_m, value_type=float),
                        "turn_deg": ParameterValue(turn_deg, value_type=float),
                        "yolo_target_label": ParameterValue(
                            yolo_target_label, value_type=str
                        ),
                        "vs_stop_distance": ParameterValue(
                            vs_stop_distance, value_type=float
                        ),
                        "press_elbow_drop_deg": ParameterValue(
                            press_elbow_drop_deg, value_type=float
                        ),
                        "open_door_duration": ParameterValue(
                            open_door_duration, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
