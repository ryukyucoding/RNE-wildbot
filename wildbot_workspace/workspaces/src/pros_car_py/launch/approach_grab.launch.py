"""Launch the approach_grab scripted drive + multi-angle grasp task."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    forward1_m = LaunchConfiguration("forward1_m")
    forward2_m = LaunchConfiguration("forward2_m")
    turn_deg = LaunchConfiguration("turn_deg")
    grab_turn1_deg = LaunchConfiguration("grab_turn1_deg")
    grab_turn2_deg = LaunchConfiguration("grab_turn2_deg")
    grab_turn3_deg = LaunchConfiguration("grab_turn3_deg")
    grab_turn_direction = LaunchConfiguration("grab_turn_direction")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "forward1_m",
                default_value="0.85",
                description="First forward segment (meters, odom closed-loop).",
            ),
            DeclareLaunchArgument(
                "forward2_m",
                default_value="0.55",
                description="Second forward segment after corner turn (meters).",
            ),
            DeclareLaunchArgument(
                "turn_deg",
                default_value="48.5",
                description="Right turn between forward segments (degrees).",
            ),
            DeclareLaunchArgument(
                "grab_turn1_deg",
                default_value="0.0",
                description="Incremental turn before 1st grasp (0 = grasp at leg-2 heading).",
            ),
            DeclareLaunchArgument(
                "grab_turn2_deg",
                default_value="120.0",
                description="Incremental turn before 2nd grasp.",
            ),
            DeclareLaunchArgument(
                "grab_turn3_deg",
                default_value="120.0",
                description="Incremental turn before 3rd grasp.",
            ),
            DeclareLaunchArgument(
                "grab_turn_direction",
                default_value="right",
                description="Turn direction for grab sweeps: right or left.",
            ),
            Node(
                package="pros_car_py",
                executable="approach_grab",
                name="approach_grab",
                output="screen",
                parameters=[
                    {
                        "forward1_m": ParameterValue(forward1_m, value_type=float),
                        "forward2_m": ParameterValue(forward2_m, value_type=float),
                        "turn_deg": ParameterValue(turn_deg, value_type=float),
                        "grab_turn1_deg": ParameterValue(
                            grab_turn1_deg, value_type=float
                        ),
                        "grab_turn2_deg": ParameterValue(
                            grab_turn2_deg, value_type=float
                        ),
                        "grab_turn3_deg": ParameterValue(
                            grab_turn3_deg, value_type=float
                        ),
                        "grab_turn_direction": ParameterValue(
                            grab_turn_direction, value_type=str
                        ),
                    }
                ],
            ),
        ]
    )
