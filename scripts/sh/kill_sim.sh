#!/usr/bin/env bash
# Kill every simulation / ROS 2 process the TB4 demo can spawn, and clear stale
# Fast-DDS shared-memory segments.
#
# Why this exists: `turtlebot4_spawn.launch.py` pulls in ~30 create3/turtlebot4
# nodes (wheel_status, hazards, kidnap_estimator, motion_control, sensors, ros_gz
# bridges, ruby Ignition clients, …) that are NOT children of run_demo_tb4.sh, so
# they survive a stopped run. Over several runs they accumulate, the CPU load
# climbs past 500, the sim crawls and Nav2 stops activating (looks like flaky DDS
# but it is CPU starvation). Run this between runs if a demo ever feels stuck.
#
#   ./scripts/run.sh kill-sim
set -eo pipefail

echo "[kill-sim] load avant : $(uptime | grep -oE 'load average:.*')"

# Patterns live in this array (never on a bare command line) so pkill cannot
# match this script's own arguments.
_PATTERNS=(
  run_demo_tb4 "ign gazebo" ign_gazebo_server ruby rviz2
  async_slam_toolbox slam_toolbox parameter_bridge ros_gz_bridge ros_ign_bridge
  controller_server planner_server bt_navigator behavior_server
  lifecycle_manager velocity_smoother smoother_server
  waypoint_follower frontier_explorer
  robot_state_publisher static_transform_publisher tf2_echo
  tf_relay cmd_vel_relay scan_throttle
  coverage_evaluator result_exporter rviz_marker bt_supervisor
  mock_map_publisher mock_coverage mock_victim
  turtlebot4_spawn turtlebot4_ignition turtlebot4_node turtlebot4_viz
  irobot_create create3 wheel_status ui_mgr sensors_node pose_republisher
  kidnap_estimator joint_state_pub ir_intensity interface_button
  hazards_vector motion_control spawner "rescue_robot/lib"
)
for _p in "${_PATTERNS[@]}"; do
  pkill -9 -f -- "${_p}" 2>/dev/null || true
done

# Clear stale Fast-DDS SHM (both the port locks and the segment/semaphore files).
find /dev/shm -maxdepth 1 \( -name 'fastrtps*' -o -name 'sem.fastrtps*' \) -delete 2>/dev/null || true

sleep 2
_left=$(pgrep -cf "ign gazebo|slam_toolbox|turtlebot4|create3|wheel_status|hazards_vector" 2>/dev/null || true)
echo "[kill-sim] processus sim restants : ${_left:-0}"
echo "[kill-sim] load après : $(uptime | grep -oE 'load average:.*')"
echo "[OK] Simulation processes cleaned."
