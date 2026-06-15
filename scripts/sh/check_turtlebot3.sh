#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot3_overlay.sh"
ensure_turtlebot3_gazebo

echo "TurtleBot3 packages"
ros2 pkg list | grep -E "^turtlebot3" || true

echo "TurtleBot3 executables"
ros2 pkg executables turtlebot3_teleop 2>/dev/null || true

echo "TURTLEBOT3_MODEL"
echo "${TURTLEBOT3_MODEL:-<unset>}"
