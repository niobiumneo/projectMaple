"""Bring up the Maple core subsystem (orchestrator + PyLips face).

Launches the ``maple_orchestrator`` node from ``maple_core`` which drives the
PyLips face, subscribes to ``/maple_action``, ``/maple_expression`` and
``/maple_appearance`` and publishes ``/motion_command``.
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
        get_package_share_directory("maple_bringup"),
        "config",
        "orchestrator.yaml",
    )

    params_file = LaunchConfiguration("orchestrator_params_file")
    server_ip = LaunchConfiguration("server_ip")
    robot_name = LaunchConfiguration("robot_name")
    use_core = LaunchConfiguration("use_core")

    declare_params_file = DeclareLaunchArgument(
        "orchestrator_params_file",
        default_value=default_params,
        description="Path to the maple_orchestrator parameters YAML file.",
    )
    declare_server_ip = DeclareLaunchArgument(
        "server_ip",
        default_value="http://127.0.0.1:8000",
        description="Address of the PyLips face server.",
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="maple",
        description="PyLips robot/face name.",
    )
    declare_use_core = DeclareLaunchArgument(
        "use_core",
        default_value="true",
        description="Whether to start the orchestrator node.",
    )

    orchestrator_node = Node(
        condition=IfCondition(use_core),
        package="maple_core",
        executable="maple_orchestrator",
        name="maple_orchestrator",
        output="screen",
        parameters=[
            params_file,
            {"server_ip": server_ip, "robot_name": robot_name},
        ],
    )

    return LaunchDescription([
        declare_params_file,
        declare_server_ip,
        declare_robot_name,
        declare_use_core,
        orchestrator_node,
    ])
