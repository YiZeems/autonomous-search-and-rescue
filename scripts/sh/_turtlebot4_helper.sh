#!/usr/bin/env bash
# Helper sourced by TurtleBot4 simulation scripts after _common.sh.
# TurtleBot4 simulation packages vary by ROS/Gazebo generation. This helper
# supports the common Humble Ignition stack first and leaves explicit override
# hooks for team members.
set -eo pipefail

load_local_env_if_present() {
  if [ -f "${REPO_ROOT}/config/local_env.sh" ]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/config/local_env.sh"
  fi
}

find_turtlebot4_bringup_package() {
  if [ -n "${IA712_TB4_BRINGUP_PACKAGE:-}" ]; then
    echo "${IA712_TB4_BRINGUP_PACKAGE}"
    return 0
  fi

  for pkg in turtlebot4_ignition_bringup turtlebot4_gz_bringup; do
    if ros2 pkg list | grep -qx "${pkg}"; then
      echo "${pkg}"
      return 0
    fi
  done

  return 1
}

find_turtlebot4_launch_file() {
  local package="$1"

  if [ -n "${IA712_TB4_LAUNCH_FILE:-}" ]; then
    echo "${IA712_TB4_LAUNCH_FILE}"
    return 0
  fi

  case "${package}" in
    turtlebot4_ignition_bringup)
      echo "turtlebot4_ignition.launch.py"
      ;;
    turtlebot4_gz_bringup)
      # Package names differ between Gazebo generations. Users can override
      # IA712_TB4_LAUNCH_FILE if their installed package uses another filename.
      echo "turtlebot4_gz.launch.py"
      ;;
    *)
      echo "turtlebot4_ignition.launch.py"
      ;;
  esac
}

ensure_turtlebot4_simulator() {
  load_local_env_if_present

  if find_turtlebot4_bringup_package >/dev/null 2>&1; then
    local pkg
    pkg="$(find_turtlebot4_bringup_package)"
    echo "[OK] TurtleBot4 bringup package found: ${pkg}"
    return 0
  fi

  cat >&2 <<'ERR'
[ERROR] TurtleBot4 simulation bringup was not found in this ROS environment.

Expected one of these packages:
  - turtlebot4_ignition_bringup  (common ROS 2 Humble TurtleBot4 simulator stack)
  - turtlebot4_gz_bringup        (newer Gazebo naming, if available)

AMD64/WSL2 users can usually try:
  sudo apt update
  sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop

ARM64 users may need a source build/overlay. If you have one, create:
  config/local_env.sh

and set either:
  export IA712_TB4_OVERLAY_SETUP=/absolute/path/to/ros2_ws/install/setup.bash
or:
  export IA712_TB4_OVERLAY_INSTALL=/absolute/path/to/ros2_ws/install

Then run again:
  ./scripts/run.sh simulation-tb4
ERR
  exit 1
}

source_turtlebot4_overlay_if_configured() {
  load_local_env_if_present

  local overlay_setup=""

  if [ -n "${IA712_TB4_OVERLAY_SETUP:-}" ]; then
    overlay_setup="${IA712_TB4_OVERLAY_SETUP}"
  elif [ -n "${IA712_TB4_OVERLAY_INSTALL:-}" ]; then
    if [ -f "${IA712_TB4_OVERLAY_INSTALL}/setup.bash" ]; then
      overlay_setup="${IA712_TB4_OVERLAY_INSTALL}/setup.bash"
    elif [ -f "${IA712_TB4_OVERLAY_INSTALL}/setup.zsh" ]; then
      overlay_setup="${IA712_TB4_OVERLAY_INSTALL}/setup.zsh"
    fi
  fi

  if [ -n "${overlay_setup}" ]; then
    if [ ! -f "${overlay_setup}" ]; then
      echo "[ERROR] IA712_TB4 overlay setup not found: ${overlay_setup}" >&2
      exit 1
    fi
    echo "[INFO] Sourcing TurtleBot4 overlay: ${overlay_setup}"
    # shellcheck disable=SC1090
    source "${overlay_setup}"
    if [ -f "${WS_DIR}/install/setup.bash" ]; then
      source "${WS_DIR}/install/setup.bash"
    fi
  fi
}
