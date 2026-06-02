#!/usr/bin/env bash
# Send a predefined waypoint path to Nav2 (Nav2 must already be running).
#
# Usage:
#   ./scripts/run.sh waypoint                              # default path
#   ./scripts/run.sh waypoint /abs/path/to/waypoints.yaml # custom file
#   ./scripts/run.sh waypoint "" true                      # loop indefinitely
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

WAYPOINTS_FILE="${1:-}"
LOOP="${2:-false}"

_cleanup() {
    echo ""
    echo "[waypoint] Stopping waypoint follower..."
    [ -n "${LAUNCH_PID:-}" ] && kill -- "-${LAUNCH_PID}" 2>/dev/null || true
    pkill -f waypoint_follower_node 2>/dev/null || true
    echo "[waypoint] Stopped."
}
trap _cleanup INT TERM EXIT

if [ -n "${WAYPOINTS_FILE}" ]; then
    ros2 launch rescue_robot waypoint.launch.py \
        waypoints_file:="${WAYPOINTS_FILE}" \
        loop:="${LOOP}" &
else
    ros2 launch rescue_robot waypoint.launch.py \
        loop:="${LOOP}" &
fi
LAUNCH_PID=$!
wait "${LAUNCH_PID}"
