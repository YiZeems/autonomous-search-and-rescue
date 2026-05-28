#!/usr/bin/env bash
# Common helper for ROS-facing scripts.
# Important: do NOT use `set -u` here. ROS 2 setup scripts may reference
# optional variables such as AMENT_TRACE_SETUP_FILES before defining them.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"
PKG_NAME="ia712_search_rescue"

# By default, protect the project scripts from stale overlays sourced by old TPs
# or previous workspaces. This avoids errors such as:
#   /opt/ros/humble/setup.bash: ... no such file or directory: .../setup.sh
# If an advanced user really wants to keep their current overlay stack, run with:
#   IA712_KEEP_ROS_OVERLAY=1 ./scripts/run.sh <command>
if [ "${IA712_KEEP_ROS_OVERLAY:-0}" != "1" ]; then
  unset COLCON_PREFIX_PATH
  unset AMENT_PREFIX_PATH
  unset CMAKE_PREFIX_PATH
  unset PYTHONPATH
  unset AMENT_TRACE_SETUP_FILES
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble/setup.bash" >&2
  echo "Install ROS 2 Humble or run non-ROS scripts directly from scripts/*.py." >&2
  exit 1
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

# Keep generated outputs in the repository root (for example ./results),
# no matter from where the user invoked scripts/run.sh.
cd "${REPO_ROOT}"
