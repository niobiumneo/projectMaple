#!/usr/bin/env bash
# Install Maple as a double-clickable Linux desktop application.
#
# Creates:
#   ~/.local/share/applications/maple.desktop      (Start)
#   ~/.local/share/applications/maple-stop.desktop (Stop)
#   ~/Desktop/Maple.desktop                        (optional shortcut)
#
# Prerequisites: Docker Engine. Optional: Node.js (for the React web UI).
set -euo pipefail

PACKAGING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${PACKAGING_DIR}/.." && pwd)"
ICON_SRC="${PACKAGING_DIR}/icons/maple.svg"
APPS_DIR="${HOME}/.local/share/applications"
ICONS_DIR="${HOME}/.local/share/icons/hicolor/scalable/apps"
DESKTOP_DIR="${HOME}/Desktop"
LAUNCHER="${PACKAGING_DIR}/maple-app.sh"

echo "[maple] Installing Linux desktop application…"
echo "[maple] Project: ${ROOT}"

chmod +x \
  "${ROOT}/maple" \
  "${ROOT}/docker/entrypoint.sh" \
  "${LAUNCHER}" \
  "${PACKAGING_DIR}/install-linux-app.sh"

mkdir -p "${APPS_DIR}" "${ICONS_DIR}"

ICON_PATH="${ICONS_DIR}/maple.svg"
cp "${ICON_SRC}" "${ICON_PATH}"

write_desktop() {
  local path="$1"
  local name="$2"
  local comment="$3"
  local exec_cmd="$4"
  cat > "${path}" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=${name}
Comment=${comment}
Exec=${exec_cmd}
Icon=${ICON_PATH}
Path=${ROOT}
Terminal=false
Categories=Education;Science;Robotics;
StartupNotify=true
EOF
  chmod +x "${path}"
}

write_desktop \
  "${APPS_DIR}/maple.desktop" \
  "Maple" \
  "Start the Maple classroom robot (ROS + face + UI)" \
  "\"${LAUNCHER}\" start"

write_desktop \
  "${APPS_DIR}/maple-stop.desktop" \
  "Stop Maple" \
  "Stop the Maple robot stack and web UI" \
  "\"${LAUNCHER}\" stop"

# Mark as trusted for GNOME / Nautilus (ignore failures on other DEs)
if command -v gio >/dev/null 2>&1; then
  gio set "${APPS_DIR}/maple.desktop" metadata::trusted true 2>/dev/null || true
  gio set "${APPS_DIR}/maple-stop.desktop" metadata::trusted true 2>/dev/null || true
fi

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${APPS_DIR}" 2>/dev/null || true
fi

# Desktop shortcut (optional — only if ~/Desktop exists)
if [[ -d "${DESKTOP_DIR}" ]]; then
  write_desktop \
    "${DESKTOP_DIR}/Maple.desktop" \
    "Maple" \
    "Start the Maple classroom robot (ROS + face + UI)" \
    "\"${LAUNCHER}\" start"
  if command -v gio >/dev/null 2>&1; then
    gio set "${DESKTOP_DIR}/Maple.desktop" metadata::trusted true 2>/dev/null || true
  fi
  echo "[maple] Desktop shortcut: ${DESKTOP_DIR}/Maple.desktop"
fi

# Trust in-repo desktop files for double-click from the project folder.
# Icon name "maple" resolves via the hicolor theme copy installed above.
chmod +x "${PACKAGING_DIR}/Maple.desktop" "${PACKAGING_DIR}/Stop-Maple.desktop"
if command -v gio >/dev/null 2>&1; then
  gio set "${PACKAGING_DIR}/Maple.desktop" metadata::trusted true 2>/dev/null || true
  gio set "${PACKAGING_DIR}/Stop-Maple.desktop" metadata::trusted true 2>/dev/null || true
fi

echo "[maple] App menu entries installed:"
echo "         ${APPS_DIR}/maple.desktop"
echo "         ${APPS_DIR}/maple-stop.desktop"
echo ""
echo "[maple] First launch may take a few minutes while Docker builds the image."
echo "[maple] After that, double-click Maple (or search for it in your app menu)."
echo "[maple] Use 'Stop Maple' when you are done."

if ! command -v docker >/dev/null 2>&1; then
  echo ""
  echo "[maple] WARNING: Docker is not installed yet — install it before launching."
fi
