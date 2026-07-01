# Maple Robot Motion Control Script

This script is designed to control the Maple robot with multiple Dynamixel motors using ROS (Robot Operating System). The script listens to a ROS topic (`/motion_command`) for commands to move the robot based on predefined motion configurations stored in JSON files.

## Prerequisites

Before running this script, ensure that the following are installed and configured on your system:

1. **ROS (Robot Operating System)**
   - Ensure ROS is installed and properly configured on the system. This repository has been tested on ROS Noetic.

2. **Dynamixel SDK**
   - The script relies on the Dynamixel SDK to interface with the motors. For more details, refer to the official documentation: [Dynamixel SDK Download](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/).
   
3. **Configuration Files**
   - `dr_r_config.py`: This file defines the configuration parameters for the Maple robot.
   - `MotionLib`: This directory contains JSON files with motion configurations. To add a new motion, follow the JSON file template to create a new file under the `MotionLib` directory. For file format details, refer to `motion.txt` under the `MotionLib` directory.

## DEMO

To see a demo of the Maple robot executing a motion:

Run the following command in your terminal:

``rostopic pub /motion_command std_msgs/String "data: 'wave'"``

This will trigger the robot to execute the motion defined in wave.json located in the MotionLib directory.


# Launch the ROS Nodes (will move this later to be a bash script)
```bash
# Terminal #1
python3 -m pylips.face.start --host 0.0.0.0 --port 8000 --skip-audio-unlock
```

```bash
# Terminal #2
cd /workspaces/projectMaple
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch maple_bringup maple.launch.py
```


To run with serial port
```bash
ls -l /dev/ttyUSB*
```

```bash
ros2 topic pub /maple_action std_msgs/msg/String \
  "{data: '{\"motion\": \"wave\", \"tts\": \"maplehi\", \"expression\": \"happy\", \"sync\": \"speech_then_motion\"}'}" --once
```

run on linux machine (can make this into a service as well)
```bash
./maple up
./maple up -v # for logs
./maple down
```