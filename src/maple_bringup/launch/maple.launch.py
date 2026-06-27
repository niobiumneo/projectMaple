"""Top-level Maple bringup.

Brings up the full Maple robot stack by including the per-subsystem launch
files in this package:

  * control -> ``robot_move_node`` (maple_control), Dynamixel motor control
  * core    -> ``maple_orchestrator`` (maple_core), PyLips face + motion routing
  * ui      -> ``rosbridge_websocket`` + ``rosapi``, bridge for the React web UI

Each subsystem can be toggled on/off, e.g. to run without hardware:

    ros2 launch maple_bringup maple.launch.py use_control:=false
    ros2 launch maple_bringup maple.launch.py use_ui:=false server_ip:=http://10.0.0.5:8000
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_launch_dir = os.path.join(
        get_package_share_directory("maple_bringup"), "launch"
    )

    # --- shared launch arguments -------------------------------------------
    use_control = LaunchConfiguration("use_control")
    use_core = LaunchConfiguration("use_core")
    use_ui = LaunchConfiguration("use_ui")

    declared_args = [
        DeclareLaunchArgument(
            "use_control", default_value="true",
            description="Start the maple_control motor node."),
        DeclareLaunchArgument(
            "use_core", default_value="true",
            description="Start the maple_core orchestrator node."),
        DeclareLaunchArgument(
            "use_ui", default_value="true",
            description="Start the rosbridge WebSocket server for the web UI."),

        # control args (forwarded)
        DeclareLaunchArgument(
            "device", default_value="/dev/ttyUSB0",
            description="Serial device for the Dynamixel controller."),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("maple_control"), "config", "params.yaml"]),
            description="maple_control parameters YAML file."),

        # core args (forwarded)
        DeclareLaunchArgument(
            "server_ip", default_value="http://127.0.0.1:8000",
            description="PyLips face server address."),
        DeclareLaunchArgument(
            "robot_name", default_value="maple",
            description="PyLips robot/face name."),
        DeclareLaunchArgument(
            "orchestrator_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("maple_bringup"), "config",
                "orchestrator.yaml"]),
            description="maple_core orchestrator parameters YAML file."),

        # ui args (forwarded)
        DeclareLaunchArgument(
            "ws_port", default_value="9090",
            description="rosbridge WebSocket port."),
        DeclareLaunchArgument(
            "ws_address", default_value="",
            description="rosbridge bind address (empty = all interfaces)."),
    ]

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, "control.launch.py")),
        launch_arguments={
            "use_control": use_control,
            "device": LaunchConfiguration("device"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, "core.launch.py")),
        launch_arguments={
            "use_core": use_core,
            "server_ip": LaunchConfiguration("server_ip"),
            "robot_name": LaunchConfiguration("robot_name"),
            "orchestrator_params_file":
                LaunchConfiguration("orchestrator_params_file"),
        }.items(),
    )

    ui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, "ui.launch.py")),
        launch_arguments={
            "use_ui": use_ui,
            "ws_port": LaunchConfiguration("ws_port"),
            "ws_address": LaunchConfiguration("ws_address"),
        }.items(),
    )

    return LaunchDescription([
        # Container-friendly defaults (no real audio hardware required).
        SetEnvironmentVariable("SDL_AUDIODRIVER", "dummy"),
        SetEnvironmentVariable("PYGAME_HIDE_SUPPORT_PROMPT", "1"),
    ] + declared_args + [control, core, ui])
