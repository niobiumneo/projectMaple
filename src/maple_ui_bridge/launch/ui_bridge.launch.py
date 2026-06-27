"""Bring up the websocket bridge that connects the Maple web UI to ROS 2.

This is the ROS 2 port of the ROS 1 setup, where the React UI (roslib) talked to
``rosbridge_server`` via ``roslaunch rosbridge_server rosbridge_websocket.launch``.
Here we include the rosbridge_server websocket launch on the same port (9090) the
UI's ``ROSLIB.Ros({ url: 'ws://localhost:9090' })`` connects to.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration('port')
    address = LaunchConfiguration('address')

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Websocket port the web UI connects to (ws://<host>:<port>).')

    declare_address = DeclareLaunchArgument(
        'address',
        default_value='',
        description='Address to bind the websocket server to (empty = all interfaces).')

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml',
            ])
        ),
        launch_arguments={
            'port': port,
            'address': address,
        }.items(),
    )

    return LaunchDescription([
        declare_port,
        declare_address,
        rosbridge,
    ])
