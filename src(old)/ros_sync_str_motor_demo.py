#!/usr/bin/env python3
import sys
import tty
import termios
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import dr_r_config
import rospy
from std_msgs.msg import String
if sys.version_info < (3, 9):
    import collections.abc, typing
    collections.abc.Mapping = typing.Mapping
from pylips.speech import RobotFace
from pylips.face import FacePresets

face = RobotFace(robot_name='Agent')

face.set_appearance(FacePresets.default)

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

index = 0
dxl_goal_position = [dr_r_config.DXL_MINIMUM_POSITION_VALUE, dr_r_config.DXL_MAXIMUM_POSITION_VALUE]  # Goal position

def move_robot(data):
    global index

    face.say("receoved motion cammond")
    face.wait()

    if data:  # If the received message is True

        # Set profile acceleration for both Dynamixels
        profile_acceleration = 100  
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dr_r_config.DXL1_ID, dr_r_config.ADDR_PRO_PROFILE_ACCELERATION, profile_acceleration)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d profile acceleration has been set successfully" % dr_r_config.DXL1_ID)

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dr_r_config.DXL2_ID, dr_r_config.ADDR_PRO_PROFILE_ACCELERATION, profile_acceleration)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d profile acceleration has been set successfully" % dr_r_config.DXL2_ID)

        # Set profile velocity for both Dynamixels
        profile_velocity = 500 
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dr_r_config.DXL1_ID, dr_r_config.ADDR_PRO_PROFILE_VELOCITY, profile_velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d profile velocity has been set successfully" % dr_r_config.DXL1_ID)

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dr_r_config.DXL2_ID, dr_r_config.ADDR_PRO_PROFILE_VELOCITY, profile_velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d profile velocity has been set successfully" % dr_r_config.DXL2_ID)

        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(dr_r_config.DXL1_ID, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % dr_r_config.DXL1_ID)
            return

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(dr_r_config.DXL2_ID, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % dr_r_config.DXL2_ID)
            return

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        while True:
            print("reading")
            # Syncread present position
            dxl_comm_result = groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(dr_r_config.DXL1_ID, dr_r_config.ADDR_PRESENT_POSITION, dr_r_config.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % dr_r_config.DXL1_ID)
                return

            # Check if groupsyncread data of Dynamixel#2 is available
            dxl_getdata_result = groupSyncRead.isAvailable(dr_r_config.DXL2_ID, dr_r_config.ADDR_PRESENT_POSITION, dr_r_config.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % dr_r_config.DXL2_ID)
                return

            # Get Dynamixel#1 present position value
            dxl1_present_position = groupSyncRead.getData(dr_r_config.DXL1_ID, dr_r_config.ADDR_PRESENT_POSITION, dr_r_config.LEN_PRESENT_POSITION)

            # Get Dynamixel#2 present position value
            dxl2_present_position = groupSyncRead.getData(dr_r_config.DXL2_ID, dr_r_config.ADDR_PRESENT_POSITION, dr_r_config.LEN_PRESENT_POSITION)

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (dr_r_config.DXL1_ID, dxl_goal_position[index], dxl1_present_position, dr_r_config.DXL2_ID, dxl_goal_position[index], dxl2_present_position))

            if not ((abs(dxl_goal_position[index] - dxl1_present_position) > dr_r_config.DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position[index] - dxl2_present_position) > dr_r_config.DXL_MOVING_STATUS_THRESHOLD)):
                break
        
        face.set_appearance(FacePresets.smile)
        face.say("pose completed")
        face.wait()

        # Change goal position
        if index == 0:
            index = 1
        else:
            index = 0

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

    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dr_r_config.DXL1_ID, dr_r_config.ADDR_TORQUE_ENABLE, dr_r_config.TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % dr_r_config.DXL1_ID)

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dr_r_config.DXL2_ID, dr_r_config.ADDR_TORQUE_ENABLE, dr_r_config.TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % dr_r_config.DXL2_ID)

    # Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(dr_r_config.DXL1_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % dr_r_config.DXL1_ID)
        quit()

    # Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncRead.addParam(dr_r_config.DXL2_ID)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % dr_r_config.DXL2_ID)
        quit()

def shutdown_robot():
    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dr_r_config.DXL1_ID, dr_r_config.ADDR_TORQUE_ENABLE, dr_r_config.TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dr_r_config.DXL2_ID, dr_r_config.ADDR_TORQUE_ENABLE, dr_r_config.TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()

if __name__ == '__main__':
    rospy.init_node('robot_move_node', anonymous=True)
    rospy.Subscriber('motion_command_topic', String, move_robot)
    
    initialize_robot()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    shutdown_robot()
