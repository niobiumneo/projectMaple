#!/usr/bin/env bash
# GUI-friendly Maple launcher for Linux desktops (double-click / app menu).
# Starts the Docker ROS stack and opens the face / web UI — no terminal required.
set -euo pipefail

PACKAGING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${PACKAGING_DIR}/.." && pwd)"
MAPLE_DIR="${ROOT}/.maple"
LOG_FILE="${MAPLE_DIR}/app.log"
ACTION="${1:-start}"

mkdir -p "${MAPLE_DIR}"

notify() {
  local title="$1"
  local body="$2"
  if command -v notify-send >/dev/null 2>&1; then
    notify-send --app-name=Maple "${title}" "${body}" || true
  fi
}

show_error() {
  local message="$1"
  notify "Maple failed" "${message}"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Maple" --width=420 --text="${message}" 2>/dev/null || true
  elif command -v kdialog >/dev/null 2>&1; then
    kdialog --error "${message}" 2>/dev/null || true
  fi
}

require_linux_tools() {
  if ! command -v docker >/dev/null 2>&1; then
    show_error "Docker is not installed.\n\nInstall Docker Engine, then run packaging/install-linux-app.sh again."
    exit 1
  fi
  if ! docker info >/dev/null 2>&1; then
    show_error "Docker is installed but not running, or your user cannot access it.\n\nStart Docker and make sure your user is in the 'docker' group."
    exit 1
  fi
}

case "${ACTION}" in
  start|app|"")
    require_linux_tools
    notify "Maple" "Starting robot stack…"
    {
      echo "===== $(date -Iseconds) maple app start ====="
      cd "${ROOT}"
      chmod +x "${ROOT}/maple" "${ROOT}/docker/entrypoint.sh" "${PACKAGING_DIR}/maple-app.sh" 2>/dev/null || true
      "${ROOT}/maple" app
      echo "===== $(date -Iseconds) maple app ready ====="
    } >> "${LOG_FILE}" 2>&1 || {
      show_error "Could not start Maple.\n\nSee log:\n${LOG_FILE}"
      exit 1
    }
    notify "Maple is running" "Face: http://localhost:8000/face/maple\nUI: http://localhost:3000"
    ;;
  stop|down)
    {
      echo "===== $(date -Iseconds) maple app stop ====="
      cd "${ROOT}"
      "${ROOT}/maple" down
    } >> "${LOG_FILE}" 2>&1 || true
    notify "Maple stopped" "Container and web UI are shut down."
    ;;
  *)
    echo "Usage: $0 {start|stop}" >&2
    exit 1
    ;;
esac
