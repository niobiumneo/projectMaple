import time

from dynamixel_sdk import *  # Import SDK

# Initialize handlers
portHandler = PortHandler("/dev/ttyUSB0")  # Change port name as needed
packetHandler = PacketHandler(2.0)  # Protocol 2.0

# Open port
portHandler.openPort()
portHandler.setBaudRate(1000000)  # Set your baud rate

# Initialize GroupSyncRead
ADDR_PRESENT_POSITION = 132  # Address for Present Position
LEN_PRESENT_POSITION = 4     # Data length
groupSyncRead = group_sync_read.GroupSyncRead(portHandler, packetHandler, 
                             ADDR_PRESENT_POSITION, 
                             LEN_PRESENT_POSITION)

# Add motors to read from
motor_ids = [11]  # Add your motor IDs
for motor_id in motor_ids:
    result = groupSyncRead.addParam(motor_id)
    if result != True:
        print(f"Failed to add motor {motor_id}")

try:
    while True:
        result = groupSyncRead.txRxPacket()
        for motor_id in motor_ids:
            if groupSyncRead.isAvailable(motor_id, 
                                         ADDR_PRESENT_POSITION, 
                                         LEN_PRESENT_POSITION):
                position = groupSyncRead.getData(motor_id, 
                                                 ADDR_PRESENT_POSITION, 
                                                 LEN_PRESENT_POSITION)
                print(f"Motor {motor_id} position: {position}")
        time.sleep(0.1)  # Adjust the delay as needed
except KeyboardInterrupt:
    print("Exiting...")

# Optionally close the port when done
portHandler.closePort()
