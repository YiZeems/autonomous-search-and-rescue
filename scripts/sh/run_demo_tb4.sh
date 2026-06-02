#!/usr/bin/env bash
# TurtleBot4 full demo — Ignition headless + SLAM + Nav2 + waypoints + results.
#
# ARM64 / Parallels constraints handled automatically:
#   - Ignition runs headless (Ogre2 crashes on virtual GPU 1ab8:0010)
#   - ign_ros2_control race: RSP starts 6 s before Ignition + robot spawn
#   - TF namespace split (turtlebot4/odom vs odom): static identity bridges + relay
#   - cmd_vel split (Nav2 /cmd_vel vs /turtlebot4/diffdrive_controller/…): relay
#
# Usage:
#   ./scripts/run.sh demo-tb4                       # standard model, maze world
#   MODEL=lite WORLD=depot ./scripts/run.sh demo-tb4
#   IA712_TB4_HEADLESS=1 ./scripts/run.sh demo-tb4  # skip Ignition GUI attempt
#
# Waypoints: config/waypoints_tb4_<world>.yaml if it exists, else waypoints.yaml
# Results:   results/run_summary.json + results/coverage_over_time.csv

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"

MODEL="${MODEL:-lite}"
WORLD="${IA712_TB4_WORLD:-depot}"
NAMESPACE="turtlebot4"

WS_INSTALL="${WS_DIR}/install"
SLAM_PARAMS="${WS_INSTALL}/rescue_robot/share/rescue_robot/config/slam_params_tb4.yaml"
RVIZ_CFG="${WS_INSTALL}/rescue_robot/share/rescue_robot/rviz/project_view.rviz"
WORLD_SDF="/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/${WORLD}.sdf"

WAYPOINTS="${REPO_ROOT}/ros2_ws/src/rescue_robot/config/waypoints_tb4_${WORLD}.yaml"
[ -f "${WAYPOINTS}" ] || WAYPOINTS="${REPO_ROOT}/ros2_ws/src/rescue_robot/config/waypoints.yaml"

LOGDIR=$(mktemp -d /tmp/ia712_tb4_XXXX)
echo "[demo-tb4] Logs: ${LOGDIR}"

# ── Software GL pour Mesa / Parallels ARM64
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"
export DISPLAY="${DISPLAY:-:0}"

# ── Ignition plugin path (nécessaire hors ros2 launch)
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds:/opt/ros/humble/share/irobot_create_ignition_bringup/worlds:/opt/ros/humble/share:${IGN_GAZEBO_RESOURCE_PATH:-}"

# ────────────────────────────────────────────────
_CLEANUP_DONE=0
_cleanup() {
    [ "${_CLEANUP_DONE}" = "1" ] && return
    _CLEANUP_DONE=1
    echo ""
    echo "[demo-tb4] Arrêt — envoi SIGTERM..."
    for p in \
        "${RVIZ_PID:-}" "${WAYPOINT_PID:-}" "${NAV2_PID:-}" \
        "${SLAM_PID:-}" "${CMD_VEL_RELAY_PID:-}" "${TF_RELAY_PID:-}" \
        "${STATIC_TF1_PID:-}" "${STATIC_TF2_PID:-}" \
        "${SENSOR_BRIDGE_PID:-}" "${CTRL_DIFF_PID:-}" "${CTRL_JSB_PID:-}" \
        "${CLOCK_BRIDGE_PID:-}" "${IGN_PID:-}" "${RSP_PID:-}" \
        "${COVERAGE_PID:-}" "${MARKER_PID:-}" "${EXPORTER_PID:-}"; do
        [ -n "${p}" ] && kill "${p}" 2>/dev/null || true
    done
    sleep 2
    pkill -9 -f "ign_gazebo_server|async_slam_toolbox|rviz2|tf_relay|cmd_vel_relay|waypoint_follower|parameter_bridge|robot_state_publisher|static_transform_publisher|coverage_evaluator|result_exporter|rviz_marker" 2>/dev/null || true
    echo "[demo-tb4] Arrêt terminé."
}
trap _cleanup INT TERM EXIT

source_turtlebot4_overlay_if_configured
ensure_turtlebot4_simulator

# Build check
if [ ! -f "${SLAM_PARAMS}" ]; then
    echo "[demo-tb4] Rebuild nécessaire (config TB4 absente)..."
    (cd "${WS_DIR}" && colcon build --symlink-install --packages-select rescue_robot 2>&1 | tail -3)
fi

echo ""
echo "[demo-tb4] ═══════════════════════════════════════════"
echo "[demo-tb4] Modèle: ${MODEL}  Monde: ${WORLD}"
echo "[demo-tb4] Waypoints: ${WAYPOINTS}"
echo "[demo-tb4] ═══════════════════════════════════════════"
echo ""

# ── ÉTAPE 1 : robot_state_publisher
echo "[1/9] robot_state_publisher..."
XACRO="/opt/ros/humble/share/turtlebot4_description/urdf/${MODEL}/turtlebot4.urdf.xacro"
URDF="$(xacro "${XACRO}" gazebo:=ignition namespace:=${NAMESPACE} 2>/dev/null)"
[ -z "${URDF}" ] && echo "[ERR] xacro a échoué" && exit 1

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -r __ns:=/${NAMESPACE} \
    -p use_sim_time:=true \
    -p "robot_description:=${URDF}" \
    >"${LOGDIR}/rsp.log" 2>&1 &
RSP_PID=$!
echo "  PID=${RSP_PID} — attente 6 s..."
sleep 6
kill -0 "${RSP_PID}" 2>/dev/null || { echo "[ERR] RSP mort"; cat "${LOGDIR}/rsp.log" | tail -5; exit 1; }

# ── ÉTAPE 2 : Ignition Gazebo server headless
echo "[2/9] Ignition Gazebo server (headless)..."
ign gazebo -s -r -v 2 "${WORLD_SDF}" >"${LOGDIR}/ign.log" 2>&1 &
IGN_PID=$!
echo "  PID=${IGN_PID} — attente 12 s..."
sleep 12
kill -0 "${IGN_PID}" 2>/dev/null || { echo "[ERR] Ignition mort"; tail -8 "${LOGDIR}/ign.log"; exit 1; }

# ── ÉTAPE 3 : Clock bridge + spawn
echo "[3/9] Clock bridge + spawn robot..."
ros2 run ros_gz_bridge parameter_bridge \
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock" \
    >"${LOGDIR}/clock.log" 2>&1 &
CLOCK_BRIDGE_PID=$!
sleep 2

ros2 run ros_ign_gazebo create \
    -name "${NAMESPACE}" -x 0.0 -y 0.0 -z 0.05 -Y 0.0 \
    -topic "/${NAMESPACE}/robot_description" 2>&1 | tee "${LOGDIR}/spawn.log"
sleep 3

# ── ÉTAPE 4 : Bridges capteurs (LiDAR → /scan, caméra → /camera/image_raw)
echo "[4/9] Bridges capteurs..."
# LiDAR : chemin Ignition complet remappé vers /scan
ros2 run ros_gz_bridge parameter_bridge \
    "/world/${WORLD}/model/${NAMESPACE}/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan" \
    --ros-args \
    -r "/world/${WORLD}/model/${NAMESPACE}/link/rplidar_link/sensor/rplidar/scan:=/scan" \
    >"${LOGDIR}/lidar_bridge.log" 2>&1 &

# Caméra OAK-D
ros2 run ros_gz_bridge parameter_bridge \
    "/world/${WORLD}/model/${NAMESPACE}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image" \
    --ros-args \
    -r "/world/${WORLD}/model/${NAMESPACE}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image:=/camera/image_raw" \
    >"${LOGDIR}/cam_bridge.log" 2>&1 &

# Odométrie + TF Ignition
ros2 run ros_gz_bridge parameter_bridge \
    "/${NAMESPACE}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
    >"${LOGDIR}/odom_bridge.log" 2>&1 &

ros2 run ros_gz_bridge parameter_bridge \
    "/${NAMESPACE}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V" \
    >"${LOGDIR}/tf_bridge.log" 2>&1 &

sleep 3
SENSOR_BRIDGE_PID=$!

# ── ÉTAPE 5 : Contrôleurs
echo "[5/9] Contrôleurs diffdrive + joint_state..."
ros2 run controller_manager spawner diffdrive_controller \
    --controller-manager /${NAMESPACE}/controller_manager \
    >"${LOGDIR}/ctrl_diff.log" 2>&1 &
CTRL_DIFF_PID=$!

ros2 run controller_manager spawner joint_state_broadcaster \
    --controller-manager /${NAMESPACE}/controller_manager \
    >"${LOGDIR}/ctrl_jsb.log" 2>&1 &
CTRL_JSB_PID=$!
sleep 6

# ── ÉTAPE 6 : Static TF bridges + relays
echo "[6/9] TF bridges + relays..."

# a) Connecte les deux arbres TF (namespaced ↔ non-namespaced)
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id "${NAMESPACE}/base_link" --child-frame-id base_link \
    >"${LOGDIR}/static_tf1.log" 2>&1 &
STATIC_TF1_PID=$!

# b) Connecte le frame capteur Ignition au frame URDF
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id rplidar_link --child-frame-id "${NAMESPACE}/rplidar_link/rplidar" \
    >"${LOGDIR}/static_tf2.log" 2>&1 &
STATIC_TF2_PID=$!

# c) Relay /turtlebot4/tf → /tf global
RELAY_SRC="${WS_DIR}/src/rescue_robot/rescue_robot/utils/tf_relay_node.py"
PYTHONPATH="${WS_DIR}/src/rescue_robot:${PYTHONPATH:-}" \
    python3 "${RELAY_SRC}" >"${LOGDIR}/tf_relay.log" 2>&1 &
TF_RELAY_PID=$!

# d) Relay /cmd_vel → /turtlebot4/diffdrive_controller/cmd_vel_unstamped
CMD_RELAY_SRC="${WS_DIR}/src/rescue_robot/rescue_robot/utils/cmd_vel_relay_node.py"
PYTHONPATH="${WS_DIR}/src/rescue_robot:${PYTHONPATH:-}" \
    python3 "${CMD_RELAY_SRC}" >"${LOGDIR}/cmd_relay.log" 2>&1 &
CMD_VEL_RELAY_PID=$!

sleep 4

# Vérification chaîne TF complète
echo "  Vérif TF: ${NAMESPACE}/rplidar_link/rplidar → ${NAMESPACE}/odom"
TF_OK=0
for _ in 1 2 3 4 5; do
    if timeout 3 ros2 run tf2_ros tf2_echo "${NAMESPACE}/rplidar_link/rplidar" "${NAMESPACE}/odom" 2>/dev/null | grep -q "Translation"; then
        TF_OK=1
        break
    fi
    sleep 2
done
[ "${TF_OK}" = "1" ] && echo "  TF OK" || echo "  [WARN] TF chain pas encore complète, SLAM attend..."

# ── ÉTAPE 7 : SLAM + Nav2
echo "[7/9] SLAM Toolbox + Nav2 (TB4 params)..."
START_TIME=$(date +%s)

bash -c "
source /opt/ros/humble/setup.bash 2>/dev/null
source '${WS_INSTALL}/local_setup.bash' 2>/dev/null || source '${WS_INSTALL}/setup.bash' 2>/dev/null || true
ros2 launch rescue_robot navigation_tb4.launch.py use_sim_time:=true
" >"${LOGDIR}/nav2.log" 2>&1 &
NAV2_PID=$!

echo "  Attente Nav2 lifecycle (35 s)..."
sleep 35
kill -0 "${NAV2_PID}" 2>/dev/null || { echo "[ERR] Nav2 mort"; tail -10 "${LOGDIR}/nav2.log"; }

# ── ÉTAPE 8 : RViz2 + nœuds résultats
echo "[8/9] RViz2 + nœuds résultats..."

# RViz2
bash -c "
source /opt/ros/humble/setup.bash 2>/dev/null
export DISPLAY=${DISPLAY} LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 QT_QPA_PLATFORM=xcb
rviz2 -d '${RVIZ_CFG}' --ros-args -p use_sim_time:=true
" >"${LOGDIR}/rviz2.log" 2>&1 &
RVIZ_PID=$!

# Nœuds de résultats
mkdir -p "${REPO_ROOT}/results"
PYTHONPATH="${WS_DIR}/src/rescue_robot:${PYTHONPATH:-}" \
    python3 -c "
import sys; sys.path.insert(0,'${WS_DIR}/src/rescue_robot')
from rescue_robot.results.coverage_evaluator_node import main; main()
" >"${LOGDIR}/coverage.log" 2>&1 &
COVERAGE_PID=$!

PYTHONPATH="${WS_DIR}/src/rescue_robot:${PYTHONPATH:-}" \
    python3 -c "
import sys; sys.path.insert(0,'${WS_DIR}/src/rescue_robot')
from rescue_robot.results.result_exporter_node import main; main()
" >"${LOGDIR}/exporter.log" 2>&1 &
EXPORTER_PID=$!

PYTHONPATH="${WS_DIR}/src/rescue_robot:${PYTHONPATH:-}" \
    python3 -c "
import sys; sys.path.insert(0,'${WS_DIR}/src/rescue_robot')
from rescue_robot.results.rviz_marker_node import main; main()
" >"${LOGDIR}/markers.log" 2>&1 &
MARKER_PID=$!

sleep 5

# ── ÉTAPE 9 : Waypoint follower
echo "[9/9] Waypoint follower → ${WAYPOINTS}"
PYTHONPATH="${WS_DIR}/src/rescue_robot:${PYTHONPATH:-}" \
    python3 -c "
import sys; sys.path.insert(0,'${WS_DIR}/src/rescue_robot')
import rclpy, yaml, math, time
from rescue_robot.navigation.waypoint_follower_node import WaypointFollowerNode
import rclpy.parameter
" 2>/dev/null

# Lancer waypoint_follower_node et attendre sa fin
bash -c "
source /opt/ros/humble/setup.bash 2>/dev/null
source '${WS_INSTALL}/local_setup.bash' 2>/dev/null || source '${WS_INSTALL}/setup.bash' 2>/dev/null || true
ros2 launch rescue_robot waypoint.launch.py \
    waypoints_file:='${WAYPOINTS}' \
    loop:=false \
    use_sim_time:=true
" >"${LOGDIR}/waypoints.log" 2>&1 &
WAYPOINT_PID=$!

echo ""
echo "[demo-tb4] ╔══════════════════════════════════════════╗"
echo "[demo-tb4] ║   Navigation en cours — suivre RViz2     ║"
echo "[demo-tb4] ║   Ctrl+C pour arrêter                    ║"
echo "[demo-tb4] ╚══════════════════════════════════════════╝"
echo ""

# Attendre la fin du waypoint follower
wait "${WAYPOINT_PID}" 2>/dev/null || true

END_TIME=$(date +%s)
DURATION=$(( END_TIME - START_TIME ))

# ── RÉSULTATS
echo ""
echo "[demo-tb4] ════════ RÉSULTATS DE MISSION ════════"
echo ""

SUMMARY="${REPO_ROOT}/results/run_summary.json"
COVERAGE_CSV="${REPO_ROOT}/results/coverage_over_time.csv"

if [ -f "${SUMMARY}" ]; then
    python3 -c "
import json, sys
with open('${SUMMARY}') as f:
    s = json.load(f)
coverage_pct = s.get('final_coverage', 0) * 100
victims = s.get('victims_detected', 0)
success = s.get('success_coverage_90', False)
print(f'  Couverture finale  : {coverage_pct:.1f}%')
print(f'  Victimes détectées : {victims}')
print(f'  Objectif 90% atteint : {\"OUI\" if success else \"NON\"}')
" 2>/dev/null || echo "  (résultats JSON non disponibles)"
else
    # Lire la dernière ligne du CSV coverage
    if [ -f "${COVERAGE_CSV}" ]; then
        LAST_COV=$(tail -1 "${COVERAGE_CSV}" | cut -d, -f2)
        printf "  Couverture finale  : %.1f%%\n" "$(python3 -c "print(float('${LAST_COV}') * 100)" 2>/dev/null || echo 0)"
    fi
fi

printf "  Durée totale       : %d min %02d s\n" $(( DURATION / 60 )) $(( DURATION % 60 ))
echo ""
echo "[demo-tb4] Waypoints: $(cat "${LOGDIR}/waypoints.log" 2>/dev/null | grep -c "reached\|Waypoint.*reached" || echo 0) atteints"
echo "[demo-tb4] Logs dans : ${LOGDIR}"
echo "[demo-tb4] Résultats : ${REPO_ROOT}/results/"
echo ""
echo "[demo-tb4] Ces données seront utilisées pour l'apprentissage du robot."
echo "[demo-tb4] ════════════════════════════════════"
