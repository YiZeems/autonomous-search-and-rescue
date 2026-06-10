#!/usr/bin/env bash
# Common helper for ROS-facing scripts.
# Important: do NOT use `set -u` here. ROS 2 setup scripts may reference
# optional variables such as AMENT_TRACE_SETUP_FILES before defining them.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"
PKG_NAME="rescue_robot"

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

# Source ROS 2 Humble — always use .bash even under zsh because the scripts
# run in a bash subprocess; the environment is passed to child processes via exec.
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[ERROR] ROS 2 Humble not found at /opt/ros/humble/setup.bash" >&2
  echo "Install ROS 2 Humble then retry." >&2
  exit 1
fi

# Source the project workspace overlay if it has been built.
# Prefer local_setup over full setup to avoid re-sourcing ROS 2 base twice
# (which causes the "no such file or directory: local_setup.sh" warning on
# Parallels/NFS paths where symlink resolution differs).
if [ -f "${WS_DIR}/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/local_setup.bash"
elif [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
fi

# Keep generated outputs in the repository root (for example ./results),
# no matter from where the user invoked scripts/run.sh.
cd "${REPO_ROOT}"
