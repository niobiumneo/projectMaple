import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('maple_control'),
        'config',
        'params.yaml'
    )
    motion_dir = os.path.join(
        get_package_share_directory("maple_control"),
        "MotionLib"
    )

    robot_node = Node(
        package="maple_control",
        executable="robot_move_node",
        name="robot_move_node",
        output="screen",
        parameters=[config_file, {"motion_dir": motion_dir}],
    )

    return LaunchDescription([robot_node])
