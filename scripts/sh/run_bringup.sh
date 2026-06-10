#!/usr/bin/env bash
# Launch the full rescue stack.
# Self-contained: sources ROS 2 Humble + workspace + TB3 overlay automatically.
# No .bashrc/.zshrc modification needed — works on any machine.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot3_overlay.sh"

ensure_turtlebot3_gazebo

exec ros2 launch rescue_bringup bringup.launch.py "$@"
