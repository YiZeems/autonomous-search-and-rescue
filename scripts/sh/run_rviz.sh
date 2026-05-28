#!/usr/bin/env bash
# Launch RViz2 with the project configuration.
# On Parallels ARM64 / Wayland sessions, forces QT_QPA_PLATFORM=xcb to avoid
# the "Ignoring XDG_SESSION_TYPE=wayland" freeze.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

# Force X11/XCB backend when running under Wayland (Parallels ARM64, WSLg, GNOME Wayland)
if [ "${XDG_SESSION_TYPE:-}" = "wayland" ] || [ "${WAYLAND_DISPLAY:-}" != "" ]; then
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export GDK_BACKEND="${GDK_BACKEND:-x11}"
    echo "[INFO] Wayland session detected. Forcing QT_QPA_PLATFORM=xcb for RViz2."
fi

RVIZ_CONFIG="${WS_DIR}/src/${PKG_NAME}/rviz/project_view.rviz"

if [ ! -f "${RVIZ_CONFIG}" ]; then
    echo "[WARN] RViz config not found at ${RVIZ_CONFIG}. Launching RViz2 without config."
    ros2 run rviz2 rviz2
else
    echo "[INFO] Launching RViz2 with config: ${RVIZ_CONFIG}"
    ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}"
fi
