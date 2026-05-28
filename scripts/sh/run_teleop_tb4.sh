#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"

source_turtlebot4_overlay_if_configured

echo "[INFO] Starting generic cmd_vel keyboard teleop for TurtleBot4."
echo "[INFO] If teleop_twist_keyboard is missing, install ros-humble-teleop-twist-keyboard."
ros2 run teleop_twist_keyboard teleop_twist_keyboard
