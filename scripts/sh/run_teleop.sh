#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot3_overlay.sh"
ensure_turtlebot3_gazebo
if ! ros2 pkg list | grep -qx "turtlebot3_teleop"; then
  echo "[ERROR] turtlebot3_teleop is not available in the current ROS environment." >&2
  echo "Install/source TurtleBot3 packages before using teleop." >&2
  exit 1
fi
ros2 run turtlebot3_teleop teleop_keyboard
