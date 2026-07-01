#!/usr/bin/env python3
import sys
import tty
import termios
import json
import threading

from dynamixel_sdk import *
import dr_r_config
import rospy
from std_msgs.msg import String
from utils import motor_angle2value

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# -----------------------------
# Pause / Resume control
# -----------------------------
# RUN_EVENT set   => running
# RUN_EVENT clear => paused
RUN_EVENT = threading.Event()
RUN_EVENT.set()

STOP_EVENT = threading.Event()

_state_lock = threading.Lock()
_active_motion = None
_active_pose_index = None


def wait_until_running():
    """Block while paused. Returns False if ROS is shutting down."""
    while not RUN_EVENT.is_set():
        if rospy.is_shutdown():
            return False
        rospy.sleep(0.05)
    return not rospy.is_shutdown()


def interruptible_sleep(seconds):
    """Sleep for up to `seconds`, but if paused, wait for resume and request a restart.

    Returns:
      - "DONE" when full sleep completed
      - "RESTART" if a pause happened during the sleep (after resume we repeat the pose)
      - "STOP" if a stop was requested
    """
    if seconds <= 0:
        return "DONE"

    start = rospy.Time.now()
    remaining = seconds

    while remaining > 0:
        if rospy.is_shutdown():
            return "STOP"
        if STOP_EVENT.is_set():
            return "STOP"

        # If we get paused during the wait, block until resume, then ask caller to restart pose.
        if not RUN_EVENT.is_set():
            ok = wait_until_running()
            if not ok:
                return "STOP"
            return "RESTART"

        step = min(0.05, remaining)
        rospy.sleep(step)

        elapsed = (rospy.Time.now() - start).to_sec()
        remaining = seconds - elapsed

    return "DONE"


def handle_interaction_control(msg: String):
    """Listen for pause/resume/stop commands over ROS.

    Topic: interaction_control (std_msgs/String)
    Payloads:
      - "pause"  : pause motion execution
      - "resume" : resume motion execution (restarts current pose)
      - "stop"   : abort current motion
    """
    cmd = (msg.data or "").strip().lower()
    if cmd == "pause":
        RUN_EVENT.clear()
        rospy.loginfo("robot_ctr: PAUSE received")
    elif cmd == "resume":
        RUN_EVENT.set()
        rospy.loginfo("robot_ctr: RESUME received")
    elif cmd == "stop":
        STOP_EVENT.set()
        RUN_EVENT.set()  # ensure we unblock if paused
        rospy.loginfo("robot_ctr: STOP received")
    else:
        rospy.logwarn(f"robot_ctr: Unknown interaction_control command: '{cmd}'")


# -----------------------------
# Dynamixel setup
# -----------------------------
portHandler = PortHandler(dr_r_config.DEVICENAME)
packetHandler = PacketHandler(dr_r_config.PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(
    portHandler,
    packetHandler,
    dr_r_config.ADDR_GOAL_POSITION,
    dr_r_config.LEN_GOAL_POSITION
)
groupSyncRead = GroupSyncRead(
    portHandler,
    packetHandler,
    dr_r_config.ADDR_PRESENT_POSITION,
    dr_r_config.LEN_PRESENT_POSITION
)


def move_robot(data: String):
    """Execute a named motion from MotionLib/<name>.json.

    Supports pausing and resuming:
      - On pause: stops progressing to the next pose and stops waiting timers.
      - On resume: repeats the current pose from the beginning, then continues.
    """
    motion_name = (data.data or "").strip()
    if not motion_name:
        return

    # Clear any old stop request for this new motion.
    STOP_EVENT.clear()

    with _state_lock:
        global _active_motion, _active_pose_index
        _active_motion = motion_name
        _active_pose_index = 0

    motion_config = f"MotionLib/{motion_name}.json"
    rospy.loginfo(f"robot_ctr: Loading motion '{motion_name}' from {motion_config}")

    try:
        with open(motion_config, "r") as f:
            config = json.load(f)
    except Exception as e:
        rospy.logerr(f"robot_ctr: Failed to load motion config '{motion_config}': {e}")
        with _state_lock:
            _active_motion = None
            _active_pose_index = None
        return

    motors = config.get("motors", [])
    pause_durations = config.get("pause_durations", [])

    if not motors:
        rospy.logwarn(f"robot_ctr: Motion '{motion_name}' has no motors defined")
        with _state_lock:
            _active_motion = None
            _active_pose_index = None
        return

    num_states = len(pause_durations) + 1
    pose_index = 0

    while pose_index < num_states and not rospy.is_shutdown():
        if STOP_EVENT.is_set():
            rospy.loginfo("robot_ctr: Stop requested, aborting motion")
            break

        # Block here if paused
        if not wait_until_running():
            break

        with _state_lock:
            _active_pose_index = pose_index

        rospy.loginfo(f"robot_ctr: Moving to pose {pose_index + 1} of {num_states}")

        # Build a SyncWrite packet for this pose
        for motor in motors:
            if STOP_EVENT.is_set() or rospy.is_shutdown():
                break

            motor_id = motor["id"]
            profile_acceleration = motor["profile_acceleration"]
            profile_velocity = motor["profile_speed"]
            goal_position = int(motor_angle2value(motor["goal_positions"][pose_index]))

            # Set profile acceleration
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler,
                motor_id,
                dr_r_config.ADDR_PRO_PROFILE_ACCELERATION,
                profile_acceleration
            )
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logwarn(packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.logwarn(packetHandler.getRxPacketError(dxl_error))

            # Set profile velocity
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
                portHandler,
                motor_id,
                dr_r_config.ADDR_PRO_PROFILE_VELOCITY,
                profile_velocity
            )
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logwarn(packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                rospy.logwarn(packetHandler.getRxPacketError(dxl_error))

            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(goal_position)),
                DXL_HIBYTE(DXL_LOWORD(goal_position)),
                DXL_LOBYTE(DXL_HIWORD(goal_position)),
                DXL_HIBYTE(DXL_HIWORD(goal_position)),
            ]

            dxl_addparam_result = groupSyncWrite.addParam(motor_id, param_goal_position)
            if dxl_addparam_result != True:
                rospy.logerr(f"[ID:{motor_id:03d}] groupSyncWrite addparam failed")
                groupSyncWrite.clearParam()
                return

        if STOP_EVENT.is_set() or rospy.is_shutdown():
            groupSyncWrite.clearParam()
            break

        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn(packetHandler.getTxRxResult(dxl_comm_result))

        groupSyncWrite.clearParam()

        # Wait after this pose, but if a pause happens during the wait,
        # resume by repeating THIS pose from the beginning.
        if pose_index < len(pause_durations):
            pause_duration = float(pause_durations[pose_index])
            rospy.loginfo(f"robot_ctr: Waiting {pause_duration:.2f}s after pose {pose_index + 1}")
            sleep_result = interruptible_sleep(pause_duration)

            if sleep_result == "STOP":
                rospy.loginfo("robot_ctr: Stop requested during wait, aborting motion")
                break
            if sleep_result == "RESTART":
                rospy.loginfo("robot_ctr: Resumed after pause, repeating current pose")
                continue

        pose_index += 1

    with _state_lock:
        _active_motion = None
        _active_pose_index = None

    STOP_EVENT.clear()
    rospy.loginfo("robot_ctr: Motion complete")


def initialize_robot():
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    if portHandler.setBaudRate(dr_r_config.BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    for i in range(12):
        motor_id = i

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler,
            motor_id,
            dr_r_config.ADDR_TORQUE_ENABLE,
            dr_r_config.TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel#{motor_id} has been successfully connected")

        dxl_addparam_result = groupSyncRead.addParam(motor_id)
        if dxl_addparam_result != True:
            print(f"[ID:{motor_id:03d}] groupSyncRead addparam failed")
            quit()


def shutdown_robot():
    groupSyncRead.clearParam()

    for i in range(12):
        motor_id = i

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler,
            motor_id,
            dr_r_config.ADDR_TORQUE_ENABLE,
            dr_r_config.TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    portHandler.closePort()


if __name__ == "__main__":
    rospy.init_node("robot_move_node", anonymous=True)

    # Existing motion commands
    rospy.Subscriber("/motion_command", String, move_robot)

    # New pause/resume/stop control
    rospy.Subscriber("/interaction_control", String, handle_interaction_control)

    initialize_robot()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    RUN_EVENT.set()
    STOP_EVENT.set()
    shutdown_robot()