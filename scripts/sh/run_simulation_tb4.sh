#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"

source_turtlebot4_overlay_if_configured
ensure_turtlebot4_simulator

PKG="$(find_turtlebot4_bringup_package)"
LAUNCH_FILE="$(find_turtlebot4_launch_file "${PKG}")"
MODEL="${IA712_TB4_MODEL:-standard}"
WORLD="${IA712_TB4_WORLD:-depot}"

cat <<INFO
[INFO] Launching TurtleBot4 simulation.
[INFO] Package: ${PKG}
[INFO] Launch:  ${LAUNCH_FILE}
[INFO] Model:   ${MODEL}
[INFO] World:   ${WORLD}

If this launch file does not support model/world arguments on your install,
set IA712_TB4_LAUNCH_FILE in config/local_env.sh or run the upstream launch manually.
INFO

ros2 launch "${PKG}" "${LAUNCH_FILE}" model:="${MODEL}" world:="${WORLD}"
