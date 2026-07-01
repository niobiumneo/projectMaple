#!/usr/bin/env bash
# Container entrypoint — build workspace, start PyLips, launch ROS stack.
set -euo pipefail

log() { echo "[maple] $*"; }

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
WORKSPACE="${WORKSPACE:-/ws}"

cd "${WORKSPACE}"

# shellcheck source=/dev/null
source "${ROS_SETUP}"

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  log "Building ROS workspace (first run)..."
  colcon build --symlink-install
else
  log "ROS workspace ready"
fi

# shellcheck source=/dev/null
source "${WORKSPACE}/install/setup.bash"

log "Starting PyLips server on port 8000..."
python3 -m pylips.server &
PYLIPS_PID=$!

cleanup() {
  log "Shutting down stack..."
  if [[ -n "${PYLIPS_PID:-}" ]]; then
    kill "${PYLIPS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

sleep 2
if kill -0 "${PYLIPS_PID}" 2>/dev/null; then
  log "PyLips server running — http://127.0.0.1:8000/face/maple"
else
  log "WARNING: PyLips server may have failed to start"
fi

MOTOR_DEVICE="/dev/ttyUSB"
LAUNCH_ARGS=()

if [[ -e "${MOTOR_DEVICE}" || -e /dev/ttyUSB0 ]]; then
  DEVICE="${MOTOR_DEVICE}"
  [[ -e /dev/ttyUSB0 ]] && DEVICE="/dev/ttyUSB0"
  log "Motor device found (${DEVICE}) — control node enabled"
  LAUNCH_ARGS+=(device:="${DEVICE}")
else
  log "No motor device — starting without control (use_control:=false)"
  LAUNCH_ARGS+=(use_control:=false)
fi

log "Starting ROS stack (orchestrator + rosbridge)..."
log "Stack ready — rosbridge ws://127.0.0.1:9090"
exec ros2 launch maple_bringup maple.launch.py "${LAUNCH_ARGS[@]}"
