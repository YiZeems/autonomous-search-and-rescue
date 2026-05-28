#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"

source_turtlebot4_overlay_if_configured

echo "[INFO] Checking TurtleBot4 packages..."
ros2 pkg list | grep -E "turtlebot4|irobot|create3|ignition|gz" || true

if pkg="$(find_turtlebot4_bringup_package 2>/dev/null)"; then
  launch_file="$(find_turtlebot4_launch_file "${pkg}")"
  echo "[OK] TurtleBot4 bringup package: ${pkg}"
  echo "[OK] Selected launch file: ${launch_file}"
else
  echo "[WARN] No TurtleBot4 simulation bringup package found."
  echo "      Try installing ros-humble-turtlebot4-simulator on AMD64,"
  echo "      or configure config/local_env.sh with a TurtleBot4 source overlay."
fi
