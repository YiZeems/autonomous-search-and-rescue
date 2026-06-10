#!/usr/bin/env bash
# Helper sourced by simulation/teleop scripts after _common.sh.
# Purpose: make turtlebot3_gazebo available in two situations:
#   1) AMD64/WSL2/Ubuntu: package is installed in /opt/ros/humble.
#   2) ARM64: package is provided by a local source overlay.
set -eo pipefail

ensure_turtlebot3_gazebo() {
  export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"

  if ros2 pkg list | grep -qx "turtlebot3_gazebo"; then
    echo "[OK] turtlebot3_gazebo found in current ROS environment."
    return 0
  fi

  echo "[WARN] turtlebot3_gazebo not found in clean ROS environment."

  local overlay_setup=""

  # Highest priority: explicit setup file.
  if [ -n "${IA712_TURTLEBOT3_OVERLAY_SETUP:-}" ]; then
    overlay_setup="${IA712_TURTLEBOT3_OVERLAY_SETUP}"
  fi

  # Second priority: explicit install directory.
  if [ -z "${overlay_setup}" ] && [ -n "${IA712_TURTLEBOT3_OVERLAY_INSTALL:-}" ]; then
    if [ -f "${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.bash" ]; then
      overlay_setup="${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.bash"
    elif [ -f "${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.zsh" ]; then
      overlay_setup="${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.zsh"
    fi
  fi

  # Third priority: project-local optional env file.
  # This file is intentionally ignored by Git if users create it locally.
  if [ -z "${overlay_setup}" ] && [ -f "${REPO_ROOT}/config/local_env.sh" ]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/config/local_env.sh"
    if [ -n "${IA712_TURTLEBOT3_OVERLAY_SETUP:-}" ]; then
      overlay_setup="${IA712_TURTLEBOT3_OVERLAY_SETUP}"
    elif [ -n "${IA712_TURTLEBOT3_OVERLAY_INSTALL:-}" ]; then
      if [ -f "${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.bash" ]; then
        overlay_setup="${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.bash"
      elif [ -f "${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.zsh" ]; then
        overlay_setup="${IA712_TURTLEBOT3_OVERLAY_INSTALL}/setup.zsh"
      fi
    fi
  fi

  # ARM64 convenience auto-detect for the IA712 TP workspace layout used in class.
  # This does not affect AMD64 users unless the folder exists on their machine.
  if [ -z "${overlay_setup}" ]; then
    local candidate="${REPO_ROOT}/../TP/Phase_1/test-easy/ros2_ws/install/setup.bash"
    if [ -f "${candidate}" ]; then
      overlay_setup="${candidate}"
    fi
  fi

  if [ -z "${overlay_setup}" ]; then
    local candidate="${REPO_ROOT}/../TP/Phase_1/test-easy/ros2_ws/install/setup.zsh"
    if [ -f "${candidate}" ]; then
      overlay_setup="${candidate}"
    fi
  fi

  if [ -z "${overlay_setup}" ] || [ ! -f "${overlay_setup}" ]; then
    cat >&2 <<ERR
[ERROR] turtlebot3_gazebo is not available.

AMD64/WSL2 users usually fix this with apt:
  sudo apt update
  sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-simulations

ARM64 users may need a source overlay. Set one of these before running simulation:
  export IA712_TURTLEBOT3_OVERLAY_SETUP=/absolute/path/to/ros2_ws/install/setup.bash
  export IA712_TURTLEBOT3_OVERLAY_INSTALL=/absolute/path/to/ros2_ws/install

Then run:
  ./scripts/run.sh simulation
ERR
    exit 1
  fi

  echo "[INFO] Sourcing TurtleBot3 overlay: ${overlay_setup}"
  # shellcheck disable=SC1090
  source "${overlay_setup}"

  # Re-source this project after the dependency overlay so this package stays visible.
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
  fi

  if ros2 pkg list | grep -qx "turtlebot3_gazebo"; then
    echo "[OK] turtlebot3_gazebo found after overlay."
  else
    echo "[ERROR] Overlay was sourced, but turtlebot3_gazebo is still missing." >&2
    echo "Checked overlay: ${overlay_setup}" >&2
    exit 1
  fi
}
