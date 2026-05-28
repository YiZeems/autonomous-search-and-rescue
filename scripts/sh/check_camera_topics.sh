#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot3_overlay.sh"
ensure_turtlebot3_gazebo

echo "== TurtleBot3 camera topic check =="
echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-<unset>}"
echo

echo "== Camera/image/depth topics currently visible =="
if ros2 topic list | grep -Ei "camera|image|depth|rgb|compressed|camera_info"; then
  echo
  echo "[OK] At least one camera-related topic is visible."
else
  cat <<'MSG'
[WARN] No camera-related topic is visible right now.

Possible explanations:
- The Gazebo simulation is not currently running.
- The current TurtleBot3 model does not publish a camera in this world.
- The camera plugin/topic names differ from the expected names.

Recommended next steps:
1. In another terminal, run:
   ./scripts/run.sh simulation
2. Wait until Gazebo has spawned the robot.
3. Run again:
   ./scripts/run.sh camera-check
4. If still no camera topic appears, inspect the Waffle Pi model SDF/URDF in the TurtleBot3 overlay.
MSG
fi

echo

echo "== Core robot topics expected for SLAM/Nav2 =="
ros2 topic list | grep -E "^/scan$|^/odom$|^/tf$|^/tf_static$|^/cmd_vel$" || true
