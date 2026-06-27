"""Bring up the Maple UI bridge (rosbridge WebSocket server).

The Maple web UI (``maple_ui-main``, a React app) talks to ROS over a rosbridge
WebSocket connection (default ``ws://<host>:9090``). This launch file starts the
``rosbridge_websocket`` server and the ``rosapi`` node so the browser can
publish/subscribe to ``/maple_action``, ``/maple_expression``,
``/maple_appearance`` and ``/interaction_control``.

Requires the ``rosbridge_suite`` packages (``rosbridge_server`` + ``rosapi``):
    sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("ws_port")
    address = LaunchConfiguration("ws_address")
    use_ui = LaunchConfiguration("use_ui")

    declare_port = DeclareLaunchArgument(
        "ws_port",
        default_value="9090",
        description="Port the rosbridge WebSocket server listens on.",
    )
    declare_address = DeclareLaunchArgument(
        "ws_address",
        default_value="",
        description="Address to bind the WebSocket server to "
                    "(empty = all interfaces).",
    )
    declare_use_ui = DeclareLaunchArgument(
        "use_ui",
        default_value="true",
        description="Whether to start the rosbridge WebSocket server.",
    )

    rosbridge_node = Node(
        condition=IfCondition(use_ui),
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": port,
            "address": address,
        }],
    )

    rosapi_node = Node(
        condition=IfCondition(use_ui),
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    return LaunchDescription([
        declare_port,
        declare_address,
        declare_use_ui,
        rosbridge_node,
        rosapi_node,
    ])
