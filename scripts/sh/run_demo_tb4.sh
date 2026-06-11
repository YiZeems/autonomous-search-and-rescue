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
#   - Nav2 recovery (spin/backup/drive_on_heading) RE-ENABLED so a wedged robot
#     can free itself (was wait-only; see nav2_params_tb4.yaml behavior_server)
#
# Usage:
#   ./scripts/run.sh demo-tb4                       # standard model, maze world
#   MODEL=standard WORLD=depot ./scripts/run.sh demo-tb4
#   IA712_TB4_HEADLESS=1 ./scripts/run.sh demo-tb4  # same (headless is always on)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
source "${SCRIPT_DIR}/_turtlebot4_helper.sh"
source "${SCRIPT_DIR}/_platform.sh"

# Detect Mac/Parallels (ARM64) vs Win/WSL2 (x86) and load the matching profile
# (render engine, software-GL, DDS transport). Override with IA712_PLATFORM=mac|win.
source_platform_profile "${REPO_ROOT}"

# Model: positional arg ($1) wins, then MODEL env, else 'standard'.
# NOTE: both 'lite' and 'standard' work — the RPLIDAR only returned range_min
# because Ogre2's gpu_lidar fails under software GL; the platform profile uses
# Ogre v1 on Mac/Parallels so the lidar sees the environment (ERRORS_AND_FIXES #10).
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
EXP_LAUNCH="${WS_INSTALL}/rescue_robot/share/rescue_robot/launch/exploration.launch.py"

# World SDF: project worlds (rescue_world) take priority, else upstream TB4 worlds.
PROJECT_WORLD="${WS_INSTALL}/rescue_world/share/rescue_world/worlds/${WORLD}.sdf"
if [ -f "${PROJECT_WORLD}" ]; then
    WORLD_SDF="${PROJECT_WORLD}"
else
    WORLD_SDF="/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/${WORLD}.sdf"
fi

# ── Waypoints: world-specific file if present, else generic fallback
WAYPOINTS_FILE="${WS_DIR}/src/rescue_robot/config/waypoints_tb4_${WORLD}.yaml"
[ -f "${WAYPOINTS_FILE}" ] || WAYPOINTS_FILE="${WS_DIR}/src/rescue_robot/config/waypoints.yaml"

LOGDIR=$(mktemp -d /tmp/ia712_demo_XXXX)

# ── Fast-DDS UDP-only transport (Mac/Parallels: IA712_USE_UDP_DDS=1).
#    Stale /dev/shm/fastrtps_port* files from previous sessions block the
#    ign_ros2_control controller_manager from being reachable via service
#    calls (service is listed but never responds). UDP is slower but stable.
#    On WSL2 (IA712_USE_UDP_DDS=0) the default transport is kept.
if [ "${IA712_USE_UDP_DDS:-0}" = "1" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_ROOT}/config/fastdds_udp_only.xml"
    [ -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ] || FASTRTPS_DEFAULT_PROFILES_FILE="/tmp/fastdds_udp_only.xml"
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
fi

# ── Clean stale SHM files from previous sessions (harmless everywhere; must
#    happen before any ROS 2 process starts to avoid zombie services).
#    Fast-DDS leaves BOTH fastrtps_port<N> (lock) and fastrtps_<hash> (segment)
#    plus sem.* semaphores. They pile up over runs and eventually the SHM
#    transport fails with "open_and_lock_file failed" (ERRORS_AND_FIXES #21).
#    Stopping the ros2 daemon first releases the handles it holds.
if [ "${IA712_CLEAN_SHM:-1}" = "1" ]; then
    ros2 daemon stop >/dev/null 2>&1 || true
    find /dev/shm -maxdepth 1 \( -name 'fastrtps*' -o -name 'sem.fastrtps*' \) -delete 2>/dev/null || true
fi

# ── Workspace overlay — AMENT_PREFIX_PATH + PYTHONPATH avoids the NFS
#    symlink issue in colcon's local_setup.bash on Parallels shared folders.
export AMENT_PREFIX_PATH="${WS_INSTALL}/rescue_robot:${WS_INSTALL}/rescue_bringup:${WS_INSTALL}/rescue_world:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}"

# ── Needed by navigation_tb4.launch.py to find config files without package lookup
export IA712_SLAM_PARAMS="${SLAM_PARAMS}"
export IA712_NAV2_PARAMS="${NAV2_PARAMS}"

# ── GL / Qt / DISPLAY come from the platform profile (config/platform_*.sh):
#    Mac/Parallels forces software GL; WSL2 keeps the hardware WSLg GPU.
#    Only set safe fallbacks here if no profile was sourced.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export DISPLAY="${DISPLAY:-:0}"

# ── Ignition plugin path (required outside ros2 launch context)
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${WS_DIR}/src/rescue_world/models:${WS_INSTALL}/rescue_world/share/rescue_world/models:${WS_INSTALL}/rescue_world/share/rescue_world/worlds:/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds:/opt/ros/humble/share/irobot_create_ignition_bringup/worlds:/opt/ros/humble/share:${IGN_GAZEBO_RESOURCE_PATH:-}"

# ────────────────────────────────────────────────────────────────────
_CLEANUP_DONE=0
_cleanup() {
    [ "${_CLEANUP_DONE}" = "1" ] && return
    _CLEANUP_DONE=1
    echo ""
    echo "[demo-tb4] Arrêt en cours..."
    for pid in \
        "${WP_PID:-}" "${RVIZ_PID:-}" "${NAV2_PID:-}" \
        "${CMD_VEL_RELAY_PID:-}" "${TF_RELAY_PID:-}" "${TF_STATIC_RELAY_PID:-}" "${THROTTLE_PID:-}" \
        "${STF2_PID:-}" "${STF1_PID:-}" "${STF_ODOM_PID:-}" \
        "${SPAWN_PID:-}" "${BRIDGE_PID:-}" "${GUI_PID:-}" \
        "${IGN_PID:-}" "${RSP_PID:-}" \
        "${COV_PID:-}" "${EXP_PID:-}" "${MRK_PID:-}" \
        "${VREG_PID:-}" "${BT_PID:-}" "${APRILTAG_PID:-}" "${CAM_STF_PID:-}"; do
        [ -n "${pid}" ] && kill "${pid}" 2>/dev/null || true
    done
    sleep 2
    pkill -9 -f "ign gazebo|ign_gazebo_server|async_slam_toolbox|rviz2|tf_relay|cmd_vel_relay|scan_throttle|waypoint_follower|frontier_explorer|bt_runner|victim_registry|apriltag_node|parameter_bridge|robot_state_publisher|static_transform_publisher|coverage_evaluator|result_exporter|rviz_marker|bt_navigator|controller_server|planner_server|behavior_server|lifecycle_manager|velocity_smoother" 2>/dev/null || true
    # turtlebot4_spawn.launch.py pulls in dozens of create3/turtlebot4 nodes
    # (wheel_status, hazards, kidnap_estimator, motion_control, sensors, ui_mgr,
    # ir_intensity, ros_gz bridges, ruby ign clients…). They are NOT children of
    # this script, so they survive unless killed explicitly — otherwise they pile
    # up across runs and starve the CPU (load > 500), which then makes Nav2
    # lifecycle activation flaky. Kill them too.
    pkill -9 -f "turtlebot4_spawn|turtlebot4_ignition|turtlebot4_node|irobot_create|create3|wheel_status|ui_mgr|sensors_node|pose_republisher|kidnap_estimator|ir_intensity|interface_button|hazards_vector|motion_control|ros_gz_bridge|ros_ign_bridge|spawner" 2>/dev/null || true
    pkill -9 -f "[r]uby" 2>/dev/null || true
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
#    éviter la race condition ign_ros2_control::getURDF()).
#    ATTENTION : le turtlebot4_spawn.launch.py installé inclut déjà
#    robot_description.launch.py, qui lance SON PROPRE /turtlebot4/robot_state_publisher.
#    Lancer un second nœud du même nom ici crée un DOUBLON : la découverte du
#    service get_parameters devient ambiguë et gz_ros2_control boucle sur
#    "robot_state_publisher service not available" -> contrôleurs jamais chargés,
#    /turtlebot4/odom jamais publié. Par défaut on laisse donc le spawn fournir le
#    RSP (IA712_DEMO_OWN_RSP=0). Mettre IA712_DEMO_OWN_RSP=1 pour l'ancien comportement
#    (utile si votre turtlebot4_spawn n'inclut pas robot_description.launch.py).
if [ "${IA712_DEMO_OWN_RSP:-0}" = "1" ]; then
    echo "[1/8] robot_state_publisher (lancé par le demo)..."
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
else
    echo "[1/8] robot_state_publisher : fourni par turtlebot4_spawn (pas de doublon)"
fi

# ── ÉTAPE 2 : Ignition server (headless: physique + CAPTEURS).
#    Le moteur de rendu vient du profil plateforme :
#      Mac/Parallels -> ogre (v1)  : Ogre2 gpu_lidar échoue en software GL et
#                                    renvoie range_min sur tous les rayons (#10).
#      WSL2/x86      -> ogre2      : GPU matériel via WSLg, lidar OK nativement.
RENDER_ENGINE="${IA712_RENDER_ENGINE:-ogre2}"
echo "[2/8] Ignition Gazebo server (-s, monde=${WORLD}, render=${RENDER_ENGINE})..."
ign gazebo -s -r -v 2 --render-engine "${RENDER_ENGINE}" "${WORLD_SDF}" >"${LOGDIR}/ign.log" 2>&1 &
IGN_PID=$!
echo "  PID=${IGN_PID} — attente 12 s"
sleep 12
kill -0 "${IGN_PID}" 2>/dev/null \
    || { echo "[ERR] Ignition mort"; tail -5 "${LOGDIR}/ign.log"; exit 1; }

# ── ÉTAPE 2b : Client GUI Gazebo (optionnel) — même moteur que le serveur.
#    Mac/Parallels : Ogre2 crashe → on attache un client Ogre v1 (fenêtre OK).
#    WSL2/x86 : Ogre2 via WSLg (fenêtre native).
#    Désactiver avec IA712_TB4_GUI=0 (CI / RAM limitée).
if [ "${IA712_TB4_GUI:-1}" != "0" ]; then
    echo "[2b/8] Client GUI Gazebo (render=${RENDER_ENGINE})..."
    ign gazebo -g --render-engine "${RENDER_ENGINE}" \
        >"${LOGDIR}/ign_gui.log" 2>&1 &
    GUI_PID=$!
    sleep 6
    kill -0 "${GUI_PID}" 2>/dev/null \
        && echo "  GUI PID=${GUI_PID} — fenêtre Gazebo ouverte" \
        || echo "  [WARN] GUI fermée (voir ${LOGDIR}/ign_gui.log) — la sim continue en headless"
fi

# ── ÉTAPE 3 : Clock bridge + spawn officiel
echo "[3/8] Clock bridge + spawn via turtlebot4_spawn.launch.py..."
ros2 run ros_gz_bridge parameter_bridge \
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock" \
    >"${LOGDIR}/clock.log" 2>&1 &
sleep 3

# Vérifier clock avant spawn
CLOCK_OK=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | head -1 || true)
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

# ── ÉTAPE 4 : Bridges capteurs (lidar → /scan_raw, camera → /camera/image_raw)
#    Le lidar Ignition publie à ~300 Hz ; on bridge vers /scan_raw puis on
#    throttle à 10 Hz vers /scan (sinon le MessageFilter de SLAM déborde et la
#    carte reste 7×7 — cf. ERRORS_AND_FIXES #25).
echo "[4/8] Bridges capteurs (lidar → /scan_raw → throttle → /scan)..."
# Auto-detect the real Ignition sensor topics. Depending on the installed
# turtlebot4_description version (and whether the robot is spawned with the
# standard dock), the model link path is either
#   /world/<w>/model/<ns>/link/...                  (flat)
# or
#   /world/<w>/model/<ns>/<ns>/link/...             (robot nested under the
#                                                    namespace model + dock)
# Querying `ign topic -l` instead of hard-coding the namespace depth makes the
# bridge work on both. Retry for a few seconds because the sensor topics can
# register a bit after the spawn. `|| true` keeps `set -e`/pipefail from aborting
# the demo when grep finds nothing on an early iteration.
LIDAR_IGN=""
for _t in $(seq 1 20); do
    LIDAR_IGN="$(ign topic -l 2>/dev/null | grep -E '/rplidar/scan$' | head -1 || true)"
    [ -n "${LIDAR_IGN}" ] && break
    sleep 1
done
[ -n "${LIDAR_IGN}" ] || LIDAR_IGN="/world/${WORLD}/model/${NAMESPACE}/link/rplidar_link/sensor/rplidar/scan"
CAM_IGN="$(ign topic -l 2>/dev/null | grep -E '/rgbd_camera/image$' | head -1 || true)"
[ -n "${CAM_IGN}" ] || CAM_IGN="/world/${WORLD}/model/${NAMESPACE}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image"
# camera_info sits next to the image topic. apriltag_ros needs it to estimate the
# tag pose (otherwise no camera→tag TF, no victim registration).
CAM_INFO_IGN="${CAM_IGN%/image}/camera_info"
# Camera optical frame (scoped Ignition sensor name) — same derivation as SCAN_FRAME.
# apriltag publishes victim_<id> relative to this frame; a static TF (step 5b)
# connects it to the robot's URDF optical frame so victim_registry can reach map.
CAM_FRAME="$(printf '%s' "${CAM_IGN}" | sed -E 's#^/world/[^/]+/model/##; s#/link/#/#; s#/sensor/#/#; s#/image$##')"
[ -n "${CAM_FRAME}" ] || CAM_FRAME="${NAMESPACE}/oakd_rgb_camera_frame/rgbd_camera"
# Derive the scan's actual frame_id from the Ignition topic path. The bridged
# LaserScan keeps Ignition's frame_id, which is the scoped sensor name:
#   /world/<w>/model/<a>/<b>/link/rplidar_link/sensor/rplidar/scan
#     -> frame_id  <a>/<b>/rplidar_link/rplidar   (e.g. turtlebot4/turtlebot4/...)
# slam_toolbox needs a static TF to THIS exact frame or it drops every scan with
# "Message Filter dropping ... queue is full" and never publishes map->odom.
SCAN_FRAME="$(printf '%s' "${LIDAR_IGN}" | sed -E 's#^/world/[^/]+/model/##; s#/link/#/#; s#/sensor/#/#; s#/scan$##')"
[ -n "${SCAN_FRAME}" ] || SCAN_FRAME="${NAMESPACE}/rplidar_link/rplidar"
echo "  lidar  Ignition: ${LIDAR_IGN}"
echo "  camera Ignition: ${CAM_IGN}"
echo "  scan frame_id  : ${SCAN_FRAME}"

ros2 run ros_gz_bridge parameter_bridge \
    "${LIDAR_IGN}@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan" \
    --ros-args \
    -r "${LIDAR_IGN}:=/scan_raw" \
    >"${LOGDIR}/lidar.log" 2>&1 &

ros2 run ros_gz_bridge parameter_bridge \
    "${CAM_IGN}@sensor_msgs/msg/Image[ignition.msgs.Image" \
    "${CAM_INFO_IGN}@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo" \
    --ros-args \
    -r "${CAM_IGN}:=/camera/image_raw" \
    -r "${CAM_INFO_IGN}:=/camera/camera_info" \
    >"${LOGDIR}/cam.log" 2>&1 &

BRIDGE_PID=$!
sleep 2

# Throttle /scan_raw (~300 Hz) → /scan (10 Hz)
PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 "${WS_DIR}/src/rescue_robot/rescue_robot/utils/scan_throttle_node.py" \
    --ros-args -p use_sim_time:=true \
    -p in_topic:=/scan_raw -p out_topic:=/scan -p rate_hz:=10.0 \
    >"${LOGDIR}/scan_throttle.log" 2>&1 &
THROTTLE_PID=$!
sleep 3

echo "  Topics: $(for t in /clock /scan /camera/image_raw /${NAMESPACE}/odom; do
    P=$(ros2 topic info "$t" 2>/dev/null | grep 'Publisher count:' | awk '{print $3}' || true)
    printf "%s(%s) " "$t" "${P:-0}"; done)"

# ── ÉTAPE 5 : Static TF bridges + relays TF et cmd_vel
echo "[5/8] Static TF bridges + relays..."

# a) Connecter les frames RÉELS du diffdrive (irobot_create_control: odom_frame_id=odom,
#    base_frame_id=base_link, NON namespacés) aux frames namespacés attendus par SLAM/Nav2
#    (turtlebot4/odom, turtlebot4/base_link). Le diffdrive publie `odom → base_link`, donc :
#      - turtlebot4/odom → odom        : rattache l'odom_frame de SLAM à l'arbre réel.
#      - base_link → turtlebot4/base_link : rattache le base_frame de SLAM SANS donner un
#        2e parent à base_link (dont le parent est déjà `odom` côté diffdrive — un double
#        parent casserait l'arbre TF).
#    Arbre final : map → turtlebot4/odom → odom → base_link → {turtlebot4/base_link,
#    rplidar_link → scan_frame}. use_sim_time:=true requis (buffer tf2 en horloge sim, #7).
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id "${NAMESPACE}/odom" --child-frame-id odom \
    --ros-args -p use_sim_time:=true \
    >"${LOGDIR}/stf_odom.log" 2>&1 &
STF_ODOM_PID=$!

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_link --child-frame-id "${NAMESPACE}/base_link" \
    --ros-args -p use_sim_time:=true \
    >"${LOGDIR}/stf1.log" 2>&1 &
STF1_PID=$!

# b) rplidar_link ≡ <scan frame_id> (frame capteur Ignition → frame URDF)
#    child-frame-id dérivé du topic détecté (gère le namespace simple OU doublé).
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id rplidar_link --child-frame-id "${SCAN_FRAME}" \
    --ros-args -p use_sim_time:=true \
    >"${LOGDIR}/stf2.log" 2>&1 &
STF2_PID=$!

# c) tf_relay : /turtlebot4/tf → /tf global (dynamique, odom→base_link)
PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 "${WS_DIR}/src/rescue_robot/rescue_robot/utils/tf_relay_node.py" \
    >"${LOGDIR}/tf_relay.log" 2>&1 &
TF_RELAY_PID=$!

# c-bis) tf_static relay : /turtlebot4/tf_static → /tf_static global.
#    Le RSP du spawn publie base_link→rplidar_link (et les autres frames URDF) sur
#    /turtlebot4/tf_static NAMESPACÉ. Sans ce relais ils n'atteignent pas le
#    /tf_static global → la chaîne scan_frame→…→base_link→odom est cassée → SLAM
#    jette tous les scans (queue full) et ne crée jamais de carte.
PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
    python3 "${WS_DIR}/src/rescue_robot/rescue_robot/utils/tf_relay_node.py" \
    --ros-args -p src_topic:=/turtlebot4/tf_static -p dst_topic:=/tf_static -p static:=true \
    >"${LOGDIR}/tf_static_relay.log" 2>&1 &
TF_STATIC_RELAY_PID=$!

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
            "${SCAN_FRAME}" "${NAMESPACE}/odom" \
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

MAP_W=$(timeout 4 ros2 topic echo /map --once 2>/dev/null | awk '/width/{print $2}' || true)
echo "  Map SLAM: ${MAP_W:-?} cellules"

# ── ÉTAPE 7 : RViz2 + nœuds résultats
echo "[7/8] RViz2 + nœuds résultats..."
mkdir -p "${REPO_ROOT}/results"

# RViz gated like the Gazebo GUI client: IA712_RVIZ=0 for a fully headless run
# (no GPU rendering at all — useful when the WSLg GPU driver crashes under load).
if [ "${IA712_RVIZ:-1}" != "0" ]; then
    rviz2 -d "${RVIZ_CFG}" --ros-args -p use_sim_time:=true \
        >"${LOGDIR}/rviz2.log" 2>&1 &
    RVIZ_PID=$!
else
    echo "  RViz désactivé (IA712_RVIZ=0) — run sans rendu"
fi

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

# ── ÉTAPE 7b : Perception (AprilTags + registry + apriltag_ros) + Behavior Tree.
#    Tous PASSIFS (ne pilotent pas le robot) — sûrs à laisser ON.
#    IA712_APRILTAG=0         -> ne spawn pas les tags ni apriltag_ros
#    IA712_VICTIM_REGISTRY=0  -> désactive le registre de victimes
#    IA712_BT=0               -> désactive le superviseur Behavior Tree (BT.CPP)
echo "[7b/8] Perception + Behavior Tree (passifs)..."

# ---- AprilTag "victimes" : spawn des 5 tags + TF caméra + apriltag_ros ----
# NOTE: la détection AprilTag dans Ignition dépend de frames namespacés (comme le
# lidar, cf. ERRORS_AND_FIXES #26) ; les POSITIONS des tags et la TF caméra
# peuvent demander un ajustement après une passe visuelle dans Gazebo. Si rien
# n'est détecté, victim_registry publie simplement 0 victime (comportement
# d'origine) — ça ne casse rien.
if [ "${IA712_APRILTAG:-1}" = "1" ]; then
    # Tags already placed in rescue_arena.sdf via <include> blocks — no dynamic spawn.
    # We only need the camera TF bridge and the apriltag_ros detector.

    # Static TF: link the URDF optical frame to the scoped Ignition sensor frame so
    # that the apriltag camera→victim_<id> TF chain reaches map via tf2.
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
        --frame-id oakd_rgb_camera_optical_frame --child-frame-id "${CAM_FRAME}" \
        --ros-args -p use_sim_time:=true \
        >"${LOGDIR}/cam_stf.log" 2>&1 &
    CAM_STF_PID=$!

    # apriltag_ros : /camera/image_raw + /camera/camera_info -> /detections + TF
    _ATAG_PARAMS="${WS_INSTALL}/rescue_bringup/share/rescue_bringup/config/apriltag_tags.yaml"
    [ -f "${_ATAG_PARAMS}" ] || _ATAG_PARAMS="${WS_DIR}/src/rescue_bringup/config/apriltag_tags.yaml"
    ros2 run apriltag_ros apriltag_node --ros-args \
        --params-file "${_ATAG_PARAMS}" \
        -p use_sim_time:=true \
        -r image_rect:=/camera/image_raw \
        -r camera_info:=/camera/camera_info \
        >"${LOGDIR}/apriltag.log" 2>&1 &
    APRILTAG_PID=$!
    sleep 1
    kill -0 "${APRILTAG_PID}" 2>/dev/null \
        && echo "  apriltag_node PID=${APRILTAG_PID} VIVANT (tag36h11 → /detections)" \
        || echo "  [WARN] apriltag_node mort — voir ${LOGDIR}/apriltag.log"
fi

if [ "${IA712_VICTIM_REGISTRY:-1}" = "1" ]; then
    # AprilTag→map projection via TF2 + dedup + persistance results/victims.json.
    # Reçoit /detections d'apriltag_ros (étape AprilTag ci-dessus). Sans tags vus,
    # publie /victims_map vide. Même PYTHONPATH NFS workaround que les nœuds résultats.
    # workaround que les nœuds résultats ci-dessus.
    PYTHONPATH="${WS_DIR}/build/rescue_robot:${PYTHONPATH:-}" \
        python3 -c "from rescue_robot.detection.victim_registry_node import main; main()" \
        >"${LOGDIR}/victim_registry.log" 2>&1 &
    VREG_PID=$!
    sleep 1
    kill -0 "${VREG_PID}" 2>/dev/null \
        && echo "  victim_registry PID=${VREG_PID} VIVANT" \
        || echo "  [WARN] victim_registry mort — voir ${LOGDIR}/victim_registry.log"
fi

if [ "${IA712_BT:-1}" = "1" ]; then
    # Mission Behavior Tree (BehaviorTree.CPP v3). Monitor passif : attend /map,
    # supervise /coverage >= 90%, publie /mission_done (latched). Binaire en
    # chemin absolu pour contourner les soucis d'overlay NFS (cf. #8/#26).
    BT_BIN="${WS_INSTALL}/rescue_decision/lib/rescue_decision/bt_runner"
    BT_XML="${WS_INSTALL}/rescue_decision/share/rescue_decision/bt_xml/mission.xml"
    if [ -x "${BT_BIN}" ] && [ -f "${BT_XML}" ]; then
        "${BT_BIN}" --ros-args \
            -p use_sim_time:=true \
            -p bt_xml:="${BT_XML}" \
            -p tick_rate_hz:=2.0 \
            -p groot_zmq:=true \
            >"${LOGDIR}/bt.log" 2>&1 &
        BT_PID=$!
        sleep 1
        kill -0 "${BT_PID}" 2>/dev/null \
            && echo "  bt_runner PID=${BT_PID} VIVANT (Groot Monitor: ZMQ localhost:1666)" \
            || echo "  [WARN] bt_runner mort — voir ${LOGDIR}/bt.log"
    else
        echo "  [WARN] bt_runner introuvable (${BT_BIN}) — paquet rescue_decision non buildé ?"
    fi
fi

# ── ÉTAPE 8 : Navigation — exploration autonome (frontière) OU waypoints.
#    IA712_EXPLORE=1  -> le robot explore tout seul (frontier_explorer_node)
#    sinon            -> suit le parcours de waypoints prédéfini.
echo ""
echo "[demo-tb4] ╔══════════════════════════════════════════╗"
echo "[demo-tb4] ║   Navigation en cours — suivre RViz2     ║"
echo "[demo-tb4] ║   Ctrl+C pour arrêter proprement         ║"
echo "[demo-tb4] ╚══════════════════════════════════════════╝"
echo ""

if [ "${IA712_EXPLORE:-0}" = "1" ]; then
    echo "[8/8] Exploration autonome (frontier_explorer_node)"
    ros2 launch "${EXP_LAUNCH}" \
        use_sim_time:=true \
        base_frame:="${NAMESPACE}/base_link" \
        >"${LOGDIR}/exploration.log" 2>&1 &
    WP_PID=$!
else
    echo "[8/8] Waypoint follower → ${WAYPOINTS_FILE##*/}"
    ros2 launch "${WP_LAUNCH}" \
        waypoints_file:="${WAYPOINTS_FILE}" \
        loop:=false \
        use_sim_time:=true \
        >"${LOGDIR}/wp.log" 2>&1 &
    WP_PID=$!
fi

# Attendre la fin de la navigation (Ctrl+C pour exploration qui tourne en continu)
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
