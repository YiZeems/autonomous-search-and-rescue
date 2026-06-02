#!/usr/bin/env bash
# Launch the full demo in ONE terminal: Gazebo + SLAM + Nav2 + RViz2 + waypoint follower.
# Ctrl+C cleanly shuts down ALL child processes (gzserver, gzclient, rviz2, nav2...).
#
# Usage:
#   ./scripts/run.sh demo              # full GUI demo
#   ./scripts/run.sh demo headless     # no Gazebo/RViz GUI (CI mode)
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_gazebo_graphics.sh"
apply_gazebo_safe_graphics_if_requested

HEADLESS="${1:-false}"

# --- Cleanup trap: SIGTERM then SIGKILL for stubborn processes.
_cleanup() {
    [ "${_CLEANUP_DONE:-0}" = "1" ] && return
    _CLEANUP_DONE=1

    echo ""
    echo "[demo] Shutting down — sending SIGTERM to process group..."
    [ -n "${LAUNCH_PID:-}" ] && kill -- "-${LAUNCH_PID}" 2>/dev/null || true

    for name in \
        "gzserver" "gzclient" "rviz2" \
        "controller_server" "bt_navigator" "slam_toolbox" \
        "waypoint_follower_node" "lifecycle_manager"; do
        pkill -f "${name}" 2>/dev/null || true
    done

    sleep 2

    for name in "gzserver" "controller_server" "slam_toolbox"; do
        pkill -9 -f "${name}" 2>/dev/null || true
    done

    echo "[demo] All processes stopped."
}
trap _cleanup INT TERM EXIT

echo "[demo] Launching full demo (headless=${HEADLESS}) — one command, all components."
echo "[demo] Gazebo + SLAM Toolbox + Nav2 + RViz2 + Waypoint Follower"
echo "[demo] Waypoint follower starts 15s after Nav2 (nav2_ready_delay:=N to adjust)"
echo "[demo] Press Ctrl+C to stop everything cleanly."
echo ""

# Launch in background to capture PID for the cleanup trap
ros2 launch rescue_bringup demo.launch.py \
    headless:="${HEADLESS}" \
    use_sim_time:=true &
LAUNCH_PID=$!

# Wait for the launch process — this blocks until Ctrl+C or natural exit
wait "${LAUNCH_PID}"
