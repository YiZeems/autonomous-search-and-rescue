#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

echo "System"
echo "Architecture: $(uname -m)"
echo "Kernel: $(uname -sr)"
echo "Shell: ${SHELL:-unknown}"

echo
echo "ROS 2"
echo "ROS_DISTRO=${ROS_DISTRO:-<not set>}"
if command -v ros2 >/dev/null 2>&1; then
  echo "ros2: $(command -v ros2)"
else
  echo "ros2: not found in PATH"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  echo "/opt/ros/humble/setup.bash: found"
else
  echo "/opt/ros/humble/setup.bash: missing"
fi

echo
echo "Gazebo"
if command -v gazebo >/dev/null 2>&1; then
  gazebo --version || true
else
  echo "gazebo: not found"
fi
if command -v gzserver >/dev/null 2>&1; then
  gzserver --version || true
else
  echo "gzserver: not found"
fi
if command -v gzclient >/dev/null 2>&1; then
  gzclient --version || true
else
  echo "gzclient: not found"
fi

echo
echo "Current ROS overlay variables"
for var in COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH; do
  echo "-- $var --"
  value="${!var:-}"
  if [ -z "$value" ]; then
    echo "<empty>"
  else
    echo "$value" | tr ':' '\n'
  fi
 done

echo
echo "Potential stale workspace paths"
stale_found=0
for var in COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH; do
  value="${!var:-}"
  if echo "$value" | grep -E "test-easy|/TP/|old|deprecated" >/dev/null 2>&1; then
    echo "[WARN] $var contains possible stale paths:"
    echo "$value" | tr ':' '\n' | grep -E "test-easy|/TP/|old|deprecated" || true
    stale_found=1
  fi
 done
if [ "$stale_found" = "0" ]; then
  echo "No obvious stale TP/workspace paths detected."
fi

echo
echo "Recommended clean test"
echo "unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH AMENT_TRACE_SETUP_FILES"
echo "source /opt/ros/humble/setup.bash"
echo "./scripts/run.sh build"
echo "./scripts/run.sh mock"
