#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"

source_turtlebot4_overlay_if_configured

echo "TurtleBot4 ROS packages"
ros2 pkg list | grep -E "turtlebot4|irobot_create" | sort || echo "[WARN] No turtlebot4 packages found."

echo ""
echo "Ignition Gazebo"
if dpkg -l libignition-gazebo6 2>/dev/null | grep -q "^ii"; then
  dpkg -l libignition-gazebo6 | awk '/^ii/{print "[OK] libignition-gazebo6 " $3}'
else
  echo "[WARN] libignition-gazebo6 not installed. Run: ./scripts/run.sh install-apt"
fi

echo ""
echo "Available TB4 worlds"
worlds_dir="$(ros2 pkg prefix turtlebot4_ignition_bringup 2>/dev/null)/share/turtlebot4_ignition_bringup/worlds"
if [ -d "${worlds_dir}" ]; then
  find "${worlds_dir}" -name '*.sdf' -printf '  %f\n' | sed 's/\.sdf//'
else
  echo "  [WARN] worlds directory not found"
fi

echo ""
if pkg="$(find_turtlebot4_bringup_package 2>/dev/null)"; then
  echo "[OK] TurtleBot4 bringup package : ${pkg}"
  echo "[OK] Launch file                : turtlebot4_ignition.launch.py"
  echo ""
  echo "Ready to launch:"
  echo "  ./scripts/run.sh demo-tb4"
  echo "  ./scripts/run.sh demo-tb4 false lite    # TB4 lite model"
  echo "  IA712_TB4_WORLD=depot ./scripts/run.sh demo-tb4"
else
  echo "[WARN] No TurtleBot4 simulation bringup package found."
  echo "       Run: ./scripts/run.sh install-apt"
fi
