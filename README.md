# Maple

Maple is a tutor-mediated social robot platform designed for English language and Canadian cultural learning activities with young newcomer children. The system coordinates Dynamixel motor motions, speech, facial expressions, and a scenario-based web interface.

> **This `main` branch contains the ROS 1 implementation.**

## Implementations

| Implementation | Branch | Environment |
| --- | --- | --- |
| ROS 1 | [`main`](https://github.com/niobiumneo/projectMaple/tree/main) | ROS Noetic, tested on Ubuntu 22.04 |
| ROS 2 | [`ros2code`](https://github.com/niobiumneo/projectMaple/tree/ros2code) | ROS 2 Jazzy in Docker. A Linux host, including Ubuntu 24.04, can run the container |

The ROS 2 branch includes its own Docker setup and instructions. See [ROS 2 and Docker](#ros-2-and-docker) below.

## Repository overview

- `src/robot_ctr.py`: executes JSON-defined Dynamixel motions and listens on `/motion_command`
- `src/maple_orchestrator.py`: coordinates motion, PyLips speech, and facial expressions
- `src/MotionLib/`: motion definitions stored as JSON files
- `src/dr_r_config.py`: Dynamixel communication and motor configuration
- `maple_ui-main/`: React scenario interface connected to ROS through rosbridge

## Requirements

For the ROS 1 implementation on this branch:

- ROS Noetic
- Python 3
- [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/)
- [PyLips](https://github.com/interaction-lab/PyLips)
- A Dynamixel USB adapter, configured as `/dev/ttyUSB0` by default
- `rosbridge_server`, Node.js, and npm if using the web interface

## Build the ROS 1 package

Clone the repository into a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/niobiumneo/projectMaple.git

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Before connecting the robot, review `src/dr_r_config.py` and confirm the serial device, baud rate, protocol version, motor IDs, and position limits match the hardware.

## Run a motion

Start the ROS master:

```bash
roscore
```

In another terminal, start the motor controller from the `src` directory:

```bash
cd ~/catkin_ws/src/projectMaple/src
python3 robot_ctr.py
```

Publish a named motion. The name must match a JSON file in `src/MotionLib/`:

```bash
rostopic pub --once /motion_command std_msgs/String "data: 'wave'"
```

Motion execution can also be controlled through `/interaction_control` using `pause`, `resume`, or `stop`:

```bash
rostopic pub --once /interaction_control std_msgs/String "data: 'stop'"
```

## Web interface

The React interface uses rosbridge to exchange commands with ROS.

Start rosbridge:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

Then start the interface:

```bash
cd maple_ui-main
npm install
npm start
```

The development interface is available at [http://localhost:3000](http://localhost:3000). See [`maple_ui-main/README.md`](maple_ui-main/README.md) for scenario and media configuration.

## Adding motions

Add a JSON motion file to `src/MotionLib/`. Each motion defines motor IDs, goal positions, speed, acceleration, and timing between poses. Use `src/MotionLib/motion.txt` as the format reference.

## ROS 2 and Docker

A ROS 2 equivalent is maintained on the [`ros2code` branch](https://github.com/niobiumneo/projectMaple/tree/ros2code). It uses ROS 2 Jazzy in Docker and can be run from an Ubuntu 24.04 host without installing ROS 2 directly on the host.

```bash
git switch ros2code
chmod +x ./maple
./maple up
```

Refer to the README on that branch for the complete ROS 2 setup, Docker commands, and service endpoints.

## Hardware safety

Running the motor controller enables actuator torque. Confirm the robot has a clear workspace, verify all configured limits, and keep an emergency stop or immediate power disconnect available during testing.
