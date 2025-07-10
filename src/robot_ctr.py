#!/usr/bin/env python3
import sys
import tty
import termios
import json
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

# Initialize PortHandler and PacketHandler instances
portHandler = PortHandler(dr_r_config.DEVICENAME)
packetHandler = PacketHandler(dr_r_config.PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, dr_r_config.ADDR_GOAL_POSITION, dr_r_config.LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, dr_r_config.ADDR_PRESENT_POSITION, dr_r_config.LEN_PRESENT_POSITION)

def move_robot(data):
    motion_config = "MotionLib/" + data.data + ".json"
    with open(motion_config, 'r') as f:
        config = json.load(f)

    motors = config['motors']
    pause_durations = config['pause_durations']

    if data:
        num_states = len(pause_durations) + 1  
        for pose_index in range(num_states):
            print(f"Moving to pose {pose_index + 1} of {num_states}")
            
            for motor in motors:
                motor_id = motor['id']
                profile_acceleration = motor['profile_acceleration']
                profile_velocity = motor['profile_speed']
                goal_position = int(motor_angle2value(motor['goal_positions'][pose_index]))

                # Set profile acceleration
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, dr_r_config.ADDR_PRO_PROFILE_ACCELERATION, profile_acceleration)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print(f"Dynamixel#{motor_id} profile acceleration has been set successfully")

                # Set profile velocity
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, dr_r_config.ADDR_PRO_PROFILE_VELOCITY, profile_velocity)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print(f"Dynamixel#{motor_id} profile velocity has been set successfully")

                # Allocate goal position value into byte array
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)), DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]

                # Add goal position value to the Syncwrite parameter storage
                dxl_addparam_result = groupSyncWrite.addParam(motor_id, param_goal_position)
                if dxl_addparam_result != True:
                    print(f"[ID:{motor_id:03d}] groupSyncWrite addparam failed")
                    return

            # Syncwrite goal position
            dxl_comm_result = groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Clear parameter storage
            groupSyncWrite.clearParam()

            # Pause after each state except the last one
            if pose_index < len(pause_durations):
                pause_duration = pause_durations[pose_index]
                print(f"Pausing for {pause_duration} seconds after state {pose_index + 1}")
                rospy.sleep(pause_duration)

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

        # Enable Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, dr_r_config.ADDR_TORQUE_ENABLE, dr_r_config.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Dynamixel#{motor_id} has been successfully connected")

        # Add parameter storage for present position value
        dxl_addparam_result = groupSyncRead.addParam(motor_id)
        if dxl_addparam_result != True:
            print(f"[ID:{motor_id:03d}] groupSyncRead addparam failed")
            quit()

def shutdown_robot():
    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    for i in range(12):
        motor_id = i

        # Disable Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, dr_r_config.ADDR_TORQUE_ENABLE, dr_r_config.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    portHandler.closePort()

if __name__ == '__main__':
    rospy.init_node('robot_move_node', anonymous=True)
    rospy.Subscriber('motion_command', String, move_robot)

    initialize_robot()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    shutdown_robot()