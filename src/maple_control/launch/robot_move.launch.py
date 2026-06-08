import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('maple_control'),
        'config',
        'params.yaml'
    )

    robot_node = Node(
        package="maple_control",
        executable="robot_move_node",
        name="robot_move_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([robot_node])
