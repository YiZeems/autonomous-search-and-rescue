#!/usr/bin/env bash
# Full TurtleBot4 demo: Ignition Gazebo + SLAM + Nav2 + RViz2 + waypoint follower.
# Mirrors the TurtleBot3 demo (run.sh demo) but for TurtleBot4 + Ignition Gazebo.
# Ctrl+C kills ALL child processes cleanly (ign-gazebo, nav2, rviz2, waypoints...).
#
# Requires TurtleBot4 packages to be installed first:
#   ./scripts/run.sh install-apt
#
# Usage:
#   ./scripts/run.sh demo-tb4                    # full GUI, warehouse world
#   ./scripts/run.sh demo-tb4 true               # headless (no GUI)
#   ./scripts/run.sh demo-tb4 false lite         # TB4 lite model
#   IA712_TB4_WORLD=depot ./scripts/run.sh demo-tb4   # depot world (lighter)
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"

# lite model = simpler URDF, fewer joints, lighter on VM
MODEL="${1:-lite}"
# depot = open space, fewest meshes/objects, lightest world for Ignition
WORLD="${IA712_TB4_WORLD:-depot}"

# --- Stratégie Parallels ARM64 : split server / GUI client
#
# Problème : ignition.launch.py construit ses propres ign_args avec -r et son
# propre --gui-config. On ne peut pas y injecter --render-engine ogre depuis
# l'extérieur. Ogre2 (défaut) crashe sur le renderer software de Parallels.
#
# Solution en 2 étapes :
#   1. La simulation tourne en --headless-rendering (server stable, physique OK)
#   2. On lance un client GUI séparé (ign gazebo -g) avec --render-engine ogre
#      → fenêtre Gazebo visible, rendu Ogre1 stable
#
# Pour désactiver la fenêtre GUI (CI / RAM insuffisante) :
#   IA712_TB4_HEADLESS=1 ./scripts/run.sh demo-tb4

# Software GL pour la fenêtre GUI (Ogre1 a besoin de Mesa)
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"

# Pas de gz_args passé au launch TB4 — ignition.launch.py construit
# ses propres ign_args en interne. Le headless est géré côté client (voir ci-dessous).

# --- Cleanup trap: SIGTERM first, then SIGKILL after 3s for stubborn processes
_cleanup() {
    # Guard against double-invocation (EXIT fires after INT/TERM)
    [ "${_CLEANUP_DONE:-0}" = "1" ] && return
    _CLEANUP_DONE=1

    echo ""
    echo "[demo-tb4] Shutting down — sending SIGTERM to process group..."

    # 1. SIGTERM to the entire ros2 launch process group
    [ -n "${LAUNCH_PID:-}" ] && kill -- "-${LAUNCH_PID}" 2>/dev/null || true

    # 2. Kill separately-started processes
    [ -n "${GUI_PID:-}" ] && kill "${GUI_PID}" 2>/dev/null || true
    [ -n "${NAV_PID:-}" ] && kill "${NAV_PID}" 2>/dev/null || true
    [ -n "${TF_RELAY_PID:-}" ] && kill "${TF_RELAY_PID}" 2>/dev/null || true

    # 3. SIGTERM to known ROS/Ignition processes by name
    for name in \
        "ign_gazebo_server" "ign gazebo" "ruby.*ign" \
        "spawner_joint_state" "spawner_diff_drive" "spawner_" \
        "controller_manager" "controller_server" "bt_navigator" \
        "slam_toolbox" "turtlebot4_node" "rviz2" \
        "waypoint_follower_node" "lifecycle_manager"; do
        pkill -f "${name}" 2>/dev/null || true
    done

    sleep 2

    # 3. SIGKILL anything still alive
    for name in \
        "ign_gazebo_server" "spawner_" "controller_manager" \
        "controller_server" "slam_toolbox" "turtlebot4_node"; do
        pkill -9 -f "${name}" 2>/dev/null || true
    done

    echo "[demo-tb4] All processes stopped."
}
trap _cleanup INT TERM EXIT

source_turtlebot4_overlay_if_configured
ensure_turtlebot4_simulator

PKG="$(find_turtlebot4_bringup_package)"

# Waypoints: use world-specific file if it exists, otherwise generic fallback
WAYPOINTS_FILE="${REPO_ROOT}/ros2_ws/src/rescue_robot/config/waypoints_tb4_${WORLD}.yaml"
if [ ! -f "${WAYPOINTS_FILE}" ]; then
    WAYPOINTS_FILE="${REPO_ROOT}/ros2_ws/src/rescue_robot/config/waypoints.yaml"
fi

echo "[demo-tb4] TurtleBot4 demo — model=${MODEL} world=${WORLD}"
echo "[demo-tb4] Étapes : Ignition+robot → SLAM+Nav2 → Waypoints"
echo "[demo-tb4] Waypoints: ${WAYPOINTS_FILE}"
echo "[demo-tb4] Press Ctrl+C to stop everything cleanly."
echo ""

# Source workspace
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

# 1. Ignition Gazebo + robot TB4
#    NOTE : slam:=true nav2:=true sont ignorés par turtlebot4_ignition.launch.py
#    (ancienne version apt sans ces args). SLAM et Nav2 sont lancés séparément.
#    Une seule fenêtre Gazebo s'ouvre (celle du TB4 launch, avec Ogre2).
ros2 launch "${PKG}" turtlebot4_ignition.launch.py \
    model:="${MODEL}" \
    world:="${WORLD}" \
    rviz:=false \
    use_sim_time:=true &
LAUNCH_PID=$!

# Attendre que le robot soit spawné avant de lancer Nav2
echo "[demo-tb4] Attente 15s pour que Ignition + robot soient prêts..."
sleep 15

# 1b. TF relay: diffdrive_controller in /turtlebot4 namespace publishes
#     transforms to /turtlebot4/tf instead of global /tf. SLAM Toolbox needs
#     global /tf or LaserScan MessageFilter drops every scan.
#     QoS: BEST_EFFORT sub (matches diffdrive) + RELIABLE pub (matches slam_toolbox).
echo "[demo-tb4] Démarrage tf_relay_node (/turtlebot4/tf → /tf)..."
ros2 run rescue_robot tf_relay_node &
TF_RELAY_PID=$!
sleep 1

# 2. SLAM + Nav2 via notre propre launch (qui lui supporte use_sim_time correctement)
echo "[demo-tb4] Démarrage SLAM + Nav2..."
ros2 launch rescue_robot navigation.launch.py use_sim_time:=true &
NAV_PID=$!

# 3. RViz2 avec notre config projet
echo "[demo-tb4] Démarrage RViz2..."
RVIZ_CONFIG="${WS_DIR}/src/rescue_robot/rviz/project_view.rviz"
if [ -f "${RVIZ_CONFIG}" ]; then
    QT_QPA_PLATFORM=xcb ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" &
else
    QT_QPA_PLATFORM=xcb ros2 run rviz2 rviz2 &
fi

# 4. Waypoint follower après initialisation Nav2
echo "[demo-tb4] Attente 25s pour Nav2 lifecycle..."
sleep 25
echo "[demo-tb4] Démarrage waypoint follower..."
ros2 launch rescue_robot waypoint.launch.py \
    waypoints_file:="${WAYPOINTS_FILE}" \
    loop:=false \
    use_sim_time:=true &

wait "${LAUNCH_PID}"
