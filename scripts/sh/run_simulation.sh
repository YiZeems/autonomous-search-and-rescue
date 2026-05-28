#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_gazebo_graphics.sh"
source "${SCRIPT_DIR}/_turtlebot3_overlay.sh"
apply_gazebo_safe_graphics_if_requested
ensure_turtlebot3_gazebo
MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
WORLD="${IA712_SIM_WORLD:-house}"
echo "[INFO] Launching TurtleBot3 simulation: model=${MODEL}, world=${WORLD}"
ros2 launch "${PKG_NAME}" simulation.launch.py model:="${MODEL}" world:="${WORLD}"
