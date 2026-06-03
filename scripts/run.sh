#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<USAGE
Usage: ./scripts/run.sh <command>

Commands:
  install-apt  Install apt dependencies after ROS 2 Humble is installed
  doctor-env   Print ROS/Gazebo/overlay diagnostics
  build        Build the ROS 2 workspace
  clean        Remove build/install/log
  mock         Launch mock system for parallel development
  bringup      Launch current bringup entrypoint
  simulation   Launch real TurtleBot3 Waffle Pi Gazebo simulation, default house world
  simulation-tb4 Launch optional TurtleBot4 simulation if installed/configured
  simulation-house Launch TurtleBot3 house world for rescue-like smoke test
  simulation-house-safe Launch house world with safe graphics settings for VM/Parallels/WSLg
  simulation-base  Launch TurtleBot3 basic demo world
  simulation-empty Launch TurtleBot3 empty world
  teleop       Drive TurtleBot3 with keyboard in a second terminal
  teleop-tb4   Drive TurtleBot4 with generic cmd_vel keyboard teleop
  check-tb3    Check TurtleBot3 Gazebo availability, including optional overlay
  check-tb4    Check TurtleBot4 simulator availability, including optional overlay
  camera-check Check whether Waffle Pi camera/image topics are visible
  demo         Launch FULL demo in one command: Gazebo+SLAM+Nav2+RViz2+waypoints [headless]
  demo-tb4     Launch FULL TurtleBot4 demo: Ignition Gazebo+SLAM+Nav2+RViz2 [headless] [model]
  waypoint     Send predefined waypoint path to Nav2 (Nav2 must be running) [file] [loop]
  rviz         Launch RViz2 with the project configuration (run in a separate terminal)
  navigation   Launch navigation module
  exploration  Launch exploration module
  detection    Launch detection module
  results      Launch results/visualization module
  bt           Launch Behavior Tree supervisor module
  uv-sync      Create/update the Python 3.10 uv development venv
  uv-test      Run lightweight Python/project tests with uv
  uv-lint      Run ruff checks with uv
USAGE
}

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

case "$1" in
  demo) "${SCRIPT_DIR}/sh/run_demo.sh" "${2:-false}" ;;
  demo-tb4) "${SCRIPT_DIR}/sh/run_demo_tb4.sh" "${2:-standard}" ;;
  waypoint) "${SCRIPT_DIR}/sh/run_waypoint.sh" "${2:-}" "${3:-false}" ;;
  install-apt) "${SCRIPT_DIR}/sh/install_apt_dependencies.sh" ;;
  doctor-env) "${SCRIPT_DIR}/sh/doctor_env.sh" ;;
  build) "${SCRIPT_DIR}/sh/build.sh" ;;
  clean) "${SCRIPT_DIR}/sh/clean.sh" ;;
  mock) "${SCRIPT_DIR}/sh/run_mock.sh" ;;
  bringup) "${SCRIPT_DIR}/sh/run_bringup.sh" ;;
  simulation) "${SCRIPT_DIR}/sh/run_simulation.sh" ;;
  simulation-tb4) "${SCRIPT_DIR}/sh/run_simulation_tb4.sh" ;;
  simulation-house) "${SCRIPT_DIR}/sh/run_simulation_house.sh" ;;
  simulation-house-safe) "${SCRIPT_DIR}/sh/run_simulation_house_safe.sh" ;;
  simulation-base) "${SCRIPT_DIR}/sh/run_simulation_base.sh" ;;
  simulation-empty) "${SCRIPT_DIR}/sh/run_simulation_empty.sh" ;;
  teleop) "${SCRIPT_DIR}/sh/run_teleop.sh" ;;
  teleop-tb4) "${SCRIPT_DIR}/sh/run_teleop_tb4.sh" ;;
  check-tb3) "${SCRIPT_DIR}/sh/check_turtlebot3.sh" ;;
  check-tb4) "${SCRIPT_DIR}/sh/check_turtlebot4.sh" ;;
  camera-check) "${SCRIPT_DIR}/sh/check_camera_topics.sh" ;;
  rviz) "${SCRIPT_DIR}/sh/run_rviz.sh" ;;
  navigation) "${SCRIPT_DIR}/sh/run_navigation.sh" ;;
  exploration) "${SCRIPT_DIR}/sh/run_exploration.sh" ;;
  detection) "${SCRIPT_DIR}/sh/run_detection.sh" ;;
  results) "${SCRIPT_DIR}/sh/run_results.sh" ;;
  bt) "${SCRIPT_DIR}/sh/run_bt.sh" ;;
  uv-sync) "${SCRIPT_DIR}/sh/uv_sync.sh" ;;
  uv-test) "${SCRIPT_DIR}/sh/uv_test.sh" ;;
  uv-lint) "${SCRIPT_DIR}/sh/uv_lint.sh" ;;
  *) usage; exit 1 ;;
esac
