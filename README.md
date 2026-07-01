# Maple

ROS 2 stack for the Maple classroom robot: Dynamixel motion control, PyLips face/expressions, and a React web UI over rosbridge.

## Quick start

On a Linux machine with [Docker](https://docs.docker.com/get-docker/) installed:

```bash
chmod +x ./maple
./maple up
```


| Command         | Description                                              |
| --------------- | -------------------------------------------------------- |
| `./maple up`    | Build image (if needed), start container, run full stack |
| `./maple up -v` | Same as above, stream all container logs                 |
| `./maple down`  | Stop container and web UI                                |


**Endpoints when running:**


| Service     | URL                                                                       |
| ----------- | ------------------------------------------------------------------------- |
| PyLips face | [http://localhost:8000/face/maple](http://localhost:8000/face/maple)      |
| rosbridge   | ws://localhost:9090                                                       |
| Web UI      | [http://localhost:3000](http://localhost:3000) (requires Node.js on host) |


Connect the Dynamixel USB adapter before `./maple up` so motor control is enabled (`/dev/ttyUSB0`). Without it, the stack still runs face + orchestrator + rosbridge, but skips `robot_move_node`.

## ROS 2 packages


| Package           | Role                                                 |
| ----------------- | ---------------------------------------------------- |
| `maple_bringup`   | Top-level launch files                               |
| `maple_core`      | Orchestrator — routes UI actions to face and motions |
| `maple_control`   | C++ motor controller (`robot_move_node`)             |
| `maple_ui_bridge` | rosbridge launch helpers                             |


Legacy ROS 1 code lives in `archive/src(old)/` and is not built.

## Manual launch (inside container or dev environment)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Terminal 1 — PyLips face server
python3 -m pylips.server

# Terminal 2 — full stack
ros2 launch maple_bringup maple.launch.py

# Without hardware
ros2 launch maple_bringup maple.launch.py use_control:=false
```

Build the workspace:

```bash
colcon build --symlink-install
```



## Test a motion from the CLI

```bash
ros2 topic pub /maple_action std_msgs/msg/String \
  "{data: '{\"motion\": \"wave\", \"tts\": \"maplehi\", \"expression\": \"happy\", \"sync\": \"speech_then_motion\"}'}" --once
```

Motion definitions are JSON files in `src/maple_control/MotionLib/`.

## Docker layout

All container files live under `docker/`:

- `Dockerfile` — ROS 2 Jazzy image with build tools and PyLips
- `docker-compose.yml` — runtime stack
- `entrypoint.sh` — builds workspace, starts PyLips + `ros2 launch`

VS Code devcontainer config is in `.devcontainer/` and uses the same Dockerfile.

## Prerequisites

- **Robot runtime:** Docker, Linux recommended for USB serial passthrough
- **Development:** ROS 2 Jazzy, colcon, Dynamixel SDK (included in Docker image)
- **Python deps:** see `requirements.txt` (PyLips and related packages)
- **Web UI:** Node.js — run from `maple_ui-main/` with `npm install && npm start`

Ensure the web UI rosbridge URL matches your host (`ws://localhost:9090` for local dev).