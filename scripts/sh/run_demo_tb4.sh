#!/usr/bin/env bash
# TurtleBot4 full demo — Ignition headless + SLAM + Nav2 + waypoints + results.
#
# ARM64 / Parallels constraints handled:
#   - Ignition runs headless (-s, no GUI) — Ogre2 crashes on vGPU 1ab8:0010
#   - ign_ros2_control race: official turtlebot4_spawn.launch.py waits for
#     robot_description before injecting SDF into Ignition
#   - TF namespace split: tf_relay_node (TRANSIENT_LOCAL QoS) bridges
#     /turtlebot4/tf → /tf
#   - cmd_vel split: cmd_vel_relay_node bridges Nav2 /cmd_vel →
#     /turtlebot4/diffdrive_controller/cmd_vel_unstamped
#   - NFS local_setup.bash failure: workspace sourced via AMENT_PREFIX_PATH +
#     PYTHONPATH (build dir for egg-info) instead of colcon overlay script
#   - Nav2 recovery (spin/backup) freezes Ignition under load → disabled,
#     only "wait" recovery kept
#
# Usage:
#   ./scripts/run.sh demo-tb4                       # standard model, maze world
#   MODEL=standard WORLD=depot ./scripts/run.sh demo-tb4
#   IA712_TB4_HEADLESS=1 ./scripts/run.sh demo-tb4  # same (headless is always on)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"

# Model: positional arg ($1) wins, then MODEL env, else 'standard'.
# NOTE: the 'lite' model has an RPLIDAR self-hit bug in Ignition (all rays
# return range_min) — 'standard' is required for SLAM to build a real map.
MODEL="${1:-${MODEL:-standard}}"
WORLD="${IA712_TB4_WORLD:-maze}"
NAMESPACE="turtlebot4"

WS_INSTALL="${WS_DIR}/install"

# ── Config files (absolute paths bypass NFS local_setup.bash failure)
SLAM_PARAMS="${WS_INSTALL}/rescue_robot/share/rescue_robot/config/slam_params_tb4.yaml"
NAV2_PARAMS="${WS_INSTALL}/rescue_robot/share/rescue_robot/config/nav2_params_tb4.yaml"
RVIZ_CFG="${WS_INSTALL}/rescue_robot/share/rescue_robot/rviz/project_view.rviz"
NAV_LAUNCH="${WS_INSTALL}/rescue_robot/share/rescue_robot/launch/navigation_tb4.launch.py"
WP_LAUNCH="${WS_INSTALL}/rescue_robot/share/rescue_robot/launch/waypoint.launch.py"
WORLD_SDF="/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/${WORLD}.sdf"

# ── Waypoints: world-specific file if present, else generic fallback
WAYPOINTS_FILE="${WS_DIR}/src/rescue_robot/config/waypoints_tb4_${WORLD}.yaml"
[ -f "${WAYPOINTS_FILE}" ] || WAYPOINTS_FILE="${WS_DIR}/src/rescue_robot/config/waypoints.yaml"

LOGDIR=$(mktemp -d /tmp/ia712_demo_XXXX)

# ── Fast-DDS UDP-only transport — disables shared memory (SHM) transport.
#    Stale /dev/shm/fastrtps_port* files from previous sessions block the
#    ign_ros2_control controller_manager from being reachable via service
#    calls (service is listed but never responds). UDP is slower but stable.
export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_ROOT}/config/fastdds_udp_only.xml"
[ -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ] || \
    FASTRTPS_DEFAULT_PROFILES_FILE="/tmp/fastdds_udp_only.xml"
# Write fallback profile if needed
if [ ! -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
cat >"${FASTRTPS_DEFAULT_PROFILES_FILE}" <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_only</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>udp_only</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
XML
fi

# ── Clean stale SHM files from previous sessions (must happen before any
#    ROS 2 process starts to avoid zombie controller_manager services)
find /dev/shm -name "fastrtps_port*" -delete 2>/dev/null || true

# ── Workspace overlay — AMENT_PREFIX_PATH + PYTHONPATH avoids the NFS
#    symlink issue in colcon's local_setup.bash on Parallels shared folders.
export AMENT_PREFIX_PATH="${WS_INSTALL}/rescue_robot:${WS_INSTALL}/rescue_bringup:${WS_INSTALL}/rescue_world:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}"

# ── Needed by navigation_tb4.launch.py to find config files without package lookup
export IA712_SLAM_PARAMS="${SLAM_PARAMS}"
export IA712_NAV2_PARAMS="${NAV2_PARAMS}"

# ── Software GL for Mesa/Parallels ARM64 (RViz2)
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"
export DISPLAY="${DISPLAY:-:0}"

# ── Ignition plugin path (required outside ros2 launch context)
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds:/opt/ros/humble/share/irobot_create_ignition_bringup/worlds:/opt/ros/humble/share:${IGN_GAZEBO_RESOURCE_PATH:-}"

# ────────────────────────────────────────────────────────────────────
_CLEANUP_DONE=0
_cleanup() {
    [ "${_CLEANUP_DONE}" = "1" ] && return
    _CLEANUP_DONE=1
    echo ""
    echo "[demo-tb4] Arrêt en cours..."
    for pid in \
        "${WP_PID:-}" "${RVIZ_PID:-}" "${NAV2_PID:-}" \
        "${CMD_VEL_RELAY_PID:-}" "${TF_RELAY_PID:-}" \
        "${STF2_PID:-}" "${STF1_PID:-}" \
        "${SPAWN_PID:-}" "${BRIDGE_PID:-}" \
        "${IGN_PID:-}" "${RSP_PID:-}" \
        "${COV_PID:-}" "${EXP_PID:-}" "${MRK_PID:-}"; do
        [ -n "${pid}" ] && kill "${pid}" 2>/dev/null || true
    done
    sleep 2
    pkill -9 -f "ign_gazebo_server|async_slam_toolbox|rviz2|tf_relay|cmd_vel_relay|waypoint_follower|parameter_bridge|robot_state_publisher|static_transform_publisher|coverage_evaluator|result_exporter|rviz_marker|bt_navigator|controller_server|planner_server|behavior_server|lifecycle_manager|velocity_smoother" 2>/dev/null || true
    echo "[demo-tb4] Arrêt terminé. Logs: ${LOGDIR}"
}
trap _cleanup INT TERM EXIT

source_turtlebot4_overlay_if_configured
ensure_turtlebot4_simulator

echo "[demo-tb4] ══════════════════════════════════════════"
echo "[demo-tb4] Modèle : ${MODEL}  |  Monde : ${WORLD}"
echo "[demo-tb4] Waypoints : ${WAYPOINTS_FILE##*/}"
echo "[demo-tb4] Logs : ${LOGDIR}"
echo "[demo-tb4] ══════════════════════════════════════════"
echo ""

# ── ÉTAPE 1 : robot_state_publisher (doit être prêt avant le spawn pour
#    éviter la race condition ign_ros2_control::getURDF())
echo "[1/8] robot_state_publisher..."
XACRO="/opt/ros/humble/share/turtlebot4_description/urdf/${MODEL}/turtlebot4.urdf.xacro"
URDF="$(xacro "${XACRO}" gazebo:=ignition namespace:=${NAMESPACE} 2>/dev/null)"
[ -z "${URDF}" ] && echo "[ERR] xacro a échoué pour ${XACRO}" && exit 1

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -r __ns:=/${NAMESPACE} \
    -p use_sim_time:=true \
    -p "robot_description:=${URDF}" \
    >"${LOGDIR}/rsp.log" 2>&1 &
RSP_PID=$!
echo "  PID=${RSP_PID} — attente 6 s"
sleep 6
kill -0 "${RSP_PID}" 2>/dev/null \
    || { echo "[ERR] RSP mort"; tail -3 "${LOGDIR}/rsp.log"; exit 1; }

# ── ÉTAPE 2 : Ignition server headless
echo "[2/8] Ignition Gazebo server (headless, monde=${WORLD})..."
ign gazebo -s -r -v 2 "${WORLD_SDF}" >"${LOGDIR}/ign.log" 2>&1 &
IGN_PID=$!
echo "  PID=${IGN_PID} — attente 12 s"
sleep 12
kill -0 "${IGN_PID}" 2>/dev/null \
    || { echo "[ERR] Ignition mort"; tail -5 "${LOGDIR}/ign.log"; exit 1; }

# ── ÉTAPE 3 : Clock bridge + spawn officiel
echo "[3/8] Clock bridge + spawn via turtlebot4_spawn.launch.py..."
ros2 run ros_gz_bridge parameter_bridge \
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock" \
    >"${LOGDIR}/clock.log" 2>&1 &
sleep 3

# Vérifier clock avant spawn
CLOCK_OK=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | head -1)
[ -z "${CLOCK_OK}" ] && echo "[WARN] /clock muet, Ignition peut être lent"

# spawn officiel : gère RSP wait + bridges (lidar, camera, odom, TF, controllers)
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py \
    namespace:="${NAMESPACE}" \
    model:="${MODEL}" \
    x:="0.0" y:="0.0" z:="0.05" yaw:="0.0" \
    >"${LOGDIR}/spawn.log" 2>&1 &
SPAWN_PID=$!
echo "  spawn PID=${SPAWN_PID} — attente 20 s (bridges + controllers)"
sleep 20
kill -0 "${SPAWN_PID}" 2>/dev/null \
    || { echo "[WARN] spawn terminé (normal si sync)"; }

# ── ÉTAPE 4 : Bridges capteurs manquants (lidar → /scan, camera → /camera/image_raw)
#    Le spawn officiel bridge vers /${NAMESPACE}/scan et oakd/rgb/… — on rebridge
#    vers les topics standards attendus par notre SLAM et RViz2.
echo "[4/8] Bridges capteurs standards..."
ros2 run ros_gz_bridge parameter_bridge \
    "/world/${WORLD}/model/${NAMESPACE}/link/rplidar_link/sensor/rplidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan" \
    --ros-args \
    -r "/world/${WORLD}/model/${NAMESPACE}/link/rplidar_link/sensor/rplidar/scan:=/scan" \
    >"${LOGDIR}/lidar.log" 2>&1 &

ros2 run ros_gz_bridge parameter_bridge \
    "/world/${WORLD}/model/${NAMESPACE}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image" \
    --ros-args \
    -r "/world/${WORLD}/model/${NAMESPACE}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image:=/camera/image_raw" \
    >"${LOGDIR}/cam.log" 2>&1 &

BRIDGE_PID=$!
sleep 4

echo "  Topics: $(for t in /clock /scan /camera/image_raw /${NAMESPACE}/odom; do
    P=$(ros2 topic info "$t" 2>/dev/null | grep 'Publisher count:' | awk '{print $3}')
    printf "%s(%s) " "$t" "${P:-0}"; done)"

# ── ÉTAPE 5 : Static TF bridges + relays TF et cmd_vel
echo "[5/8] Static TF bridges + relays..."

# a) turtlebot4/base_link ≡ base_link (connecte les deux arbres TF namespaced/non-namespaced)
#    use_sim_time:=true est requis : sans lui les TF statiques portent l'horloge
#    murale et le buffer tf2 (horloge sim) ne résout pas scan→odom → SLAM ne
#    publie jamais map→odom (cf. ERRORS_AND_FIXES #7).
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id "${NAMESPACE}/base_link" --child-frame-id base_link \
    --ros-args -p use_sim_time:=true \
    >"${LOGDIR}/stf1.log" 2>&1 &
STF1_PID=$!

# b) rplidar_link ≡ turtlebot4/rplidar_link/rplidar (frame capteur Ignition → frame URDF)
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id rplidar_link --child-frame-id "${NAMESPACE}/rplidar_link/rplidar" \
    --ros-args -p use_sim_time:=true \
    >"${LOGDIR}/stf2.log" 2>&1 &
STF2_PID=$!

# c) tf_relay : /turtlebot4/tf → /tf global
#    QoS TRANSIENT_LOCAL subscriber requis pour matcher le publisher diffdrive_controller
PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 "${WS_DIR}/src/rescue_robot/rescue_robot/utils/tf_relay_node.py" \
    >"${LOGDIR}/tf_relay.log" 2>&1 &
TF_RELAY_PID=$!

# d) cmd_vel_relay : /cmd_vel (Nav2) → /turtlebot4/diffdrive_controller/cmd_vel_unstamped
PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 "${WS_DIR}/src/rescue_robot/rescue_robot/utils/cmd_vel_relay_node.py" \
    >"${LOGDIR}/cmd_relay.log" 2>&1 &
CMD_VEL_RELAY_PID=$!

sleep 5

# Vérification chaîne TF complète
_TF_OK=0
for _i in 1 2 3 4 5; do
    if timeout 3 ros2 run tf2_ros tf2_echo \
            "${NAMESPACE}/rplidar_link/rplidar" "${NAMESPACE}/odom" \
            2>/dev/null | grep -q "Translation"; then
        _TF_OK=1; break
    fi
    sleep 2
done
[ "${_TF_OK}" = "1" ] && echo "  TF chain: COMPLÈTE" \
                       || echo "  TF chain: [WARN] incomplète, SLAM attendra"

# ── ÉTAPE 6 : SLAM + Nav2 (paramètres TB4)
echo "[6/8] SLAM Toolbox + Nav2 (params TB4, recovery=wait uniquement)..."
START_WALL=$(date +%s)

ros2 launch "${NAV_LAUNCH}" use_sim_time:=true >"${LOGDIR}/nav2.log" 2>&1 &
NAV2_PID=$!
echo "  PID=${NAV2_PID} — attente 50 s lifecycle"
sleep 50

if grep -q "Managed nodes are active" "${LOGDIR}/nav2.log" 2>/dev/null; then
    echo "  Nav2 ACTIF"
else
    echo "  [WARN] lifecycle pas encore actif, on continue..."
    tail -5 "${LOGDIR}/nav2.log" 2>/dev/null
fi

MAP_W=$(timeout 4 ros2 topic echo /map --once 2>/dev/null | awk '/width/{print $2}')
echo "  Map SLAM: ${MAP_W:-?} cellules"

# ── ÉTAPE 7 : RViz2 + nœuds résultats
echo "[7/8] RViz2 + nœuds résultats..."
mkdir -p "${REPO_ROOT}/results"

rviz2 -d "${RVIZ_CFG}" --ros-args -p use_sim_time:=true \
    >"${LOGDIR}/rviz2.log" 2>&1 &
RVIZ_PID=$!

PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 -c "from rescue_robot.results.coverage_evaluator_node import main; main()" \
    >"${LOGDIR}/cov.log" 2>&1 &
COV_PID=$!

PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 -c "from rescue_robot.results.result_exporter_node import main; main()" \
    >"${LOGDIR}/exp.log" 2>&1 &
EXP_PID=$!

PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 -c "from rescue_robot.results.rviz_marker_node import main; main()" \
    >"${LOGDIR}/mrk.log" 2>&1 &
MRK_PID=$!

sleep 5
kill -0 "${RVIZ_PID}" 2>/dev/null \
    && echo "  RViz2 PID=${RVIZ_PID} VIVANT" \
    || echo "  [WARN] RViz2 mort — voir ${LOGDIR}/rviz2.log"

# ── ÉTAPE 8 : Waypoint follower (parcours + résultats)
echo "[8/8] Waypoint follower → ${WAYPOINTS_FILE##*/}"
echo ""
echo "[demo-tb4] ╔══════════════════════════════════════════╗"
echo "[demo-tb4] ║   Navigation en cours — suivre RViz2     ║"
echo "[demo-tb4] ║   Ctrl+C pour arrêter proprement         ║"
echo "[demo-tb4] ╚══════════════════════════════════════════╝"
echo ""

ros2 launch "${WP_LAUNCH}" \
    waypoints_file:="${WAYPOINTS_FILE}" \
    loop:=false \
    use_sim_time:=true \
    >"${LOGDIR}/wp.log" 2>&1 &
WP_PID=$!

# Attendre la fin du waypoint follower
wait "${WP_PID}" 2>/dev/null || true

END_WALL=$(date +%s)
DURATION=$(( END_WALL - START_WALL ))

# ── AFFICHAGE RÉSULTATS ──────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════"
echo "         RÉSULTATS DE MISSION"
echo "══════════════════════════════════════════"
printf "Durée totale (depuis Nav2) : %dm%02ds\n" \
    $(( DURATION / 60 )) $(( DURATION % 60 ))

SUMMARY="${REPO_ROOT}/results/run_summary.json"
if [ -f "${SUMMARY}" ]; then
    python3 -c "
import json
with open('${SUMMARY}') as f:
    s = json.load(f)
cov = s.get('final_coverage', 0) * 100
vic = s.get('victims_detected', 0)
ok  = s.get('success_coverage_90', False)
print(f'Couverture finale   : {cov:.1f}%')
print(f'Victimes détectées  : {vic}')
print(f'Objectif 90%        : {\"OUI\" if ok else \"NON\"}')
" 2>/dev/null || echo "  (résultats JSON non disponibles)"
fi

echo ""
echo "Waypoints:"
WP_REACHED=$(grep -c "reached\."          "${LOGDIR}/wp.log" 2>/dev/null || echo 0)
WP_FAILED=$(grep -c "failed or timed out" "${LOGDIR}/wp.log" 2>/dev/null || echo 0)
echo "  Atteints : ${WP_REACHED}"
echo "  Échecs   : ${WP_FAILED}"

echo ""
echo "Fichiers résultats : ${REPO_ROOT}/results/"
echo "Logs               : ${LOGDIR}/"
echo "══════════════════════════════════════════"
echo ""
echo "Ces données serviront à l'apprentissage du robot."
