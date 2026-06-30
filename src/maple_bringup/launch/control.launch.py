"""Bring up the Maple control subsystem (Dynamixel motor controller).

Launches the C++ ``robot_move_node`` from ``maple_control`` which subscribes to
``/motion_command`` and ``/interaction_control`` and drives the hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("maple_control"),
        "config",
        "params.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    device = LaunchConfiguration("device")
    use_control = LaunchConfiguration("use_control")
    motion_dir = os.path.join(
        get_package_share_directory("maple_control"),
        "MotionLib"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the robot_move_node parameters YAML file.",
    )
    declare_device = DeclareLaunchArgument(
        "device",
        default_value="/dev/ttyUSB0",
        description="Serial device the Dynamixel U2D2 is connected to.",
    )
    declare_use_control = DeclareLaunchArgument(
        "use_control",
        default_value="true",
        description="Whether to start the control node.",
    )

    robot_node = Node(
        condition=IfCondition(use_control),
        package="maple_control",
        executable="robot_move_node",
        name="robot_move_node",
        output="screen",
        parameters=[params_file, {"device": device, "motion_dir": motion_dir}],
    )

    return LaunchDescription([
        declare_params_file,
        declare_device,
        declare_use_control,
        robot_node,
    ])
