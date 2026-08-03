# Maple

ROS 2 stack for the Maple classroom robot: Dynamixel motion control, PyLips face/expressions, and a React web UI over rosbridge.

## Linux desktop app (recommended)

On a Linux machine with [Docker](https://docs.docker.com/get-docker/) installed, install Maple once so you can launch it by double-click — no terminal required:

```bash
chmod +x packaging/install-linux-app.sh
./packaging/install-linux-app.sh
```

That adds **Maple** and **Stop Maple** to your app menu, and a **Maple** shortcut on your Desktop when `~/Desktop` exists.

Then double-click **Maple**. It will:

1. Build the Docker image if needed (first run only)
2. Start the ROS stack inside the container (PyLips + orchestrator + rosbridge + motors when connected)
3. Start the web UI (if Node.js is installed)
4. Open the face / UI in your browser

Use **Stop Maple** when you are finished.

You can also double-click `packaging/Maple.desktop` directly from the project folder (after install, or after right-click → Allow Launching on GNOME).

## Quick start (command line)

```bash
chmod +x ./maple
./maple app          # full stack + open browser
# or:
./maple start        # full stack, no browser
./maple down         # stop everything
```

| Command | Description |
| --- | --- |
| `./maple app` | Start full stack and open the face / UI in a browser |
| `./maple start` | Start full stack without opening a browser |
| `./maple start --open` | Same as `app` |
| `./maple up` | Start container and open an interactive shell |
| `./maple build` | Build the Docker image only |
| `./maple down` | Stop container and web UI |

**Endpoints when running:**

| Service | URL |
| --- | --- |
| PyLips face | [http://localhost:8000/face/maple](http://localhost:8000/face/maple) |
| rosbridge | ws://localhost:9090 |
| Web UI | [http://localhost:3000](http://localhost:3000) (requires Node.js on host) |

Connect the Dynamixel USB adapter before launching so motor control is enabled (`/dev/ttyUSB0`). Without it, the stack still runs face + orchestrator + rosbridge, but skips `robot_move_node`.

## ROS 2 packages

| Package | Role |
| --- | --- |
| `maple_bringup` | Top-level launch files |
| `maple_core` | Orchestrator — routes UI actions to face and motions |
| `maple_control` | C++ motor controller (`robot_move_node`) |
| `maple_ui_bridge` | rosbridge launch helpers |

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

Linux desktop packaging lives under `packaging/`:

- `install-linux-app.sh` — one-time install into the app menu / Desktop
- `maple-app.sh` — double-click launcher (notifications, no terminal)
- `Maple.desktop` / `Stop-Maple.desktop` — desktop entries

VS Code devcontainer config is in `.devcontainer/` and uses the same Dockerfile.

## Prerequisites

- **Robot runtime:** Docker, Linux recommended for USB serial passthrough
- **Desktop app:** same as above; optional `libnotify` / `zenity` for status popups
- **Development:** ROS 2 Jazzy, colcon, Dynamixel SDK (included in Docker image)
- **Python deps:** see `requirements.txt` (PyLips and related packages)
- **Web UI:** Node.js — run from `maple_ui-main/` with `npm install && npm start`

Ensure the web UI rosbridge URL matches your host (`ws://localhost:9090` for local dev).
