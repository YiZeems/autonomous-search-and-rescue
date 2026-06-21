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
#   ./scripts/run.sh demo-tb4                       # standard model, rescue_arena world
#   MODEL=standard IA712_TB4_WORLD=maze ./scripts/run.sh demo-tb4   # override world
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
WORLD="${IA712_TB4_WORLD:-rescue_arena}"
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
# Headless (cluster / CI): do NOT force DISPLAY=:0 — with no X server, Ogre2 tries
# GLX on :0 and segfaults. Only set a display when a GUI is actually wanted.
if [ "${IA712_TB4_GUI:-1}" != "0" ]; then
    export DISPLAY="${DISPLAY:-:0}"
fi

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
    # Finalise the Gazebo video BEFORE killing the GUI, else the .mp4 is left
    # unencoded/corrupt. Stop the recorder and give it a moment to flush.
    if [ -n "${GAZEBO_VIDEO:-}" ] && kill -0 "${GUI_PID:-0}" 2>/dev/null; then
        echo "[demo-tb4] Finalisation de la vidéo Gazebo (${GAZEBO_VIDEO##*/})..."
        ign service -s /gui/record_video \
            --reqtype ignition.msgs.VideoRecord \
            --reptype ignition.msgs.Boolean --timeout 4000 \
            --req 'stop: true' >/dev/null 2>&1 || true
        sleep 6
        [ -f "${GAZEBO_VIDEO}" ] \
            && echo "[demo-tb4] Vidéo Gazebo enregistrée : ${GAZEBO_VIDEO}" \
            || echo "[demo-tb4] [WARN] vidéo Gazebo absente (recorder non démarré ?)"
    fi
    # Finalise the RViz screen recording (ffmpeg) BEFORE killing RViz: SIGINT lets
    # ffmpeg flush the moov atom so the .mp4 is playable.
    if [ -n "${RVIZ_FFMPEG_PID:-}" ] && kill -0 "${RVIZ_FFMPEG_PID}" 2>/dev/null; then
        echo "[demo-tb4] Finalisation de la vidéo RViz..."
        kill -INT "${RVIZ_FFMPEG_PID}" 2>/dev/null || true
        for _w in 1 2 3 4 5 6 7 8; do kill -0 "${RVIZ_FFMPEG_PID}" 2>/dev/null || break; sleep 1; done
        kill -9 "${RVIZ_FFMPEG_PID}" 2>/dev/null || true
        [ -n "${RVIZ_VIDEO:-}" ] && [ -f "${RVIZ_VIDEO}" ] \
            && echo "[demo-tb4] Vidéo RViz enregistrée : ${RVIZ_VIDEO}"
    fi
    # Tear down the headless recording RViz + its Xvfb.
    [ -n "${RVIZ_REC_PID:-}" ] && kill "${RVIZ_REC_PID}" 2>/dev/null || true
    [ -n "${XVFB_PID:-}" ] && kill "${XVFB_PID}" 2>/dev/null || true
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
    # Custom GUI config: zoomed-out overhead camera (whole arena visible) + a
    # VideoRecorder plugin so we can capture the Gazebo 3D scene to .mp4.
    # Override the config with IA712_GZ_GUI_CONFIG=<file> (empty = stock default).
    GUI_CONFIG="${IA712_GZ_GUI_CONFIG-${REPO_ROOT}/config/gazebo_gui_record.config}"
    _gui_cfg_arg=()
    if [ -n "${GUI_CONFIG}" ] && [ -f "${GUI_CONFIG}" ]; then
        _gui_cfg_arg=(--gui-config "${GUI_CONFIG}")
        echo "  GUI config : ${GUI_CONFIG##*/} (vue dézoomée + enregistreur vidéo)"
    fi
    ign gazebo -g --render-engine "${RENDER_ENGINE}" "${_gui_cfg_arg[@]}" \
        >"${LOGDIR}/ign_gui.log" 2>&1 &
    GUI_PID=$!
    sleep 6
    if kill -0 "${GUI_PID}" 2>/dev/null; then
        echo "  GUI PID=${GUI_PID} — fenêtre Gazebo ouverte"
        # ── Enregistrement vidéo de la scène Gazebo (best-effort) ───────────
        #    Désactiver avec IA712_GZ_RECORD=0. Sortie persistante dans results/.
        if [ "${IA712_GZ_RECORD:-1}" != "0" ]; then
            mkdir -p "${REPO_ROOT}/results"
            GAZEBO_VIDEO="${IA712_GZ_VIDEO:-${REPO_ROOT}/results/gazebo_capture.mp4}"
            # The VideoRecorder plugin advertises /gui/record_video once the GUI
            # scene is rendering; retry a few times while it comes up.
            ( for _try in 1 2 3 4 5 6; do
                if ign service -s /gui/record_video \
                       --reqtype ignition.msgs.VideoRecord \
                       --reptype ignition.msgs.Boolean --timeout 4000 \
                       --req "start: true, format: \"mp4\", save_filename: \"${GAZEBO_VIDEO}\"" \
                       >/dev/null 2>&1; then
                    echo "[2b/8] Enregistrement Gazebo démarré -> ${GAZEBO_VIDEO}" \
                        >>"${LOGDIR}/gz_record.log"
                    exit 0
                fi
                sleep 3
              done
              echo "[2b/8] [WARN] service /gui/record_video indisponible — pas d'enregistrement Gazebo" \
                  >>"${LOGDIR}/gz_record.log" ) &
            echo "  Enregistrement vidéo Gazebo -> ${GAZEBO_VIDEO} (démarrage en tâche de fond)"
        fi
    else
        echo "  [WARN] GUI fermée (voir ${LOGDIR}/ign_gui.log) — la sim continue en headless"
    fi
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
    x:="${IA712_SPAWN_X:-0.0}" y:="${IA712_SPAWN_Y:-0.0}" z:="0.05" yaw:="${IA712_SPAWN_YAW:-0.0}" \
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

# ── Enregistrement vidéo de RViz via Xvfb (framebuffer LISIBLE) ──────────────
#    Sur WSLg, x11grab de l'écran :0 ne capture que du noir (Xwayland rootless +
#    RViz rend en OpenGL). On lance donc un RViz HEADLESS dédié dans un
#    framebuffer virtuel Xvfb (rendu logiciel llvmpipe), dont l'image EST
#    capturable par ffmpeg. La fenêtre RViz :0 reste visible pour l'opérateur ;
#    celle-ci, headless, sert UNIQUEMENT à l'enregistrement (mêmes topics, donc
#    mêmes frontières/carte/victimes). Désactiver avec IA712_RVIZ_RECORD=0.
#    Sortie persistante dans results/rviz_capture.mp4.
if [ "${IA712_RVIZ_RECORD:-1}" != "0" ] && command -v ffmpeg >/dev/null 2>&1 \
   && command -v Xvfb >/dev/null 2>&1; then
    RVIZ_VIDEO="${IA712_RVIZ_VIDEO:-${REPO_ROOT}/results/rviz_capture.mp4}"
    RVIZ_REC_DISPLAY="${IA712_RVIZ_REC_DISPLAY:-:99}"
    RVIZ_REC_GEO="${IA712_RVIZ_REC_GEO:-1600x1000}"
    Xvfb "${RVIZ_REC_DISPLAY}" -screen 0 "${RVIZ_REC_GEO}x24" >"${LOGDIR}/xvfb_rviz.log" 2>&1 &
    XVFB_PID=$!
    sleep 3
    DISPLAY="${RVIZ_REC_DISPLAY}" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
        rviz2 -d "${RVIZ_CFG}" --ros-args -p use_sim_time:=true \
        >"${LOGDIR}/rviz_rec.log" 2>&1 &
    RVIZ_REC_PID=$!
    sleep 12   # laisser RViz (GL logiciel) démarrer et peindre la scène
    DISPLAY="${RVIZ_REC_DISPLAY}" ffmpeg -hide_banner -loglevel error -nostdin \
        -f x11grab -framerate 10 -video_size "${RVIZ_REC_GEO}" -i "${RVIZ_REC_DISPLAY}.0" \
        -vcodec libx264 -pix_fmt yuv420p -preset ultrafast -y "${RVIZ_VIDEO}" \
        >"${LOGDIR}/rviz_record.log" 2>&1 &
    RVIZ_FFMPEG_PID=$!
    echo "  Enregistrement vidéo RViz (Xvfb ${RVIZ_REC_DISPLAY}, ${RVIZ_REC_GEO}) -> ${RVIZ_VIDEO}"
fi

# ── ÉTAPE 7b : Perception (AprilTags + registry + apriltag_ros) + Behavior Tree.
#    Tous PASSIFS (ne pilotent pas le robot) — sûrs à laisser ON.
#    IA712_APRILTAG=0         -> ne spawn pas les tags ni apriltag_ros
#    IA712_VICTIM_REGISTRY=0  -> désactive le registre de victimes
#    IA712_BT=0               -> désactive le superviseur Behavior Tree (BT.CPP)
echo "[7b/8] Perception + Behavior Tree (passifs)..."

# ---- AprilTag "victimes" : spawn des 5 tags + TF caméra + apriltag_ros ----
# AprilTag detection depends on the camera TF chain being available.
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
    # Mission Behavior Tree (BehaviorTree.CPP v3). ORCHESTRATEUR : Sequence
    # WaitForMap → ExplorePhase (gère /mission/explore_enable jusqu'à couverture)
    # → InspectPhase (gère /mission/inspect_enable jusqu'à /mission/inspect_done)
    # → VictimsFound → /mission_done (latched). La décision (quand explorer / quand
    # inspecter / mission finie) est dans le BT. Binaire en chemin absolu (overlay NFS).
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

# ── Create3 safety_override : la base TB4 a des "reflexes" de sécurité qui
#    stoppent le robot (REFLEX_STUCK) et font échouer Nav2 ("Failed to make
#    progress") sur les goals serrés (doorways, poses près des murs). On passe
#    safety_override=full pour laisser Nav2 piloter librement. Le node peut être
#    /motion_control ou /turtlebot4/motion_control selon le bringup -> on tente les deux.
for mc in /motion_control /turtlebot4/motion_control; do
    ros2 param set "${mc}" safety_override full >/dev/null 2>&1 \
        && { echo "  safety_override=full sur ${mc}"; break; }
done

# ── ÉTAPE 8 : Navigation.
#    (défaut)            -> MISSION ORCHESTRÉE PAR LE BEHAVIOR TREE (L18, conforme énoncé) :
#                           le BT décide exploration → inspection → mission_done. Le robot
#                           découvre seul un environnement INCONNU et localise les victimes
#                           AprilTag, SANS connaître leurs positions (entrée = sa propre carte).
#    IA712_BT_MISSION=0  -> même mission 2 phases mais orchestrée par le script (repli sans BT).
#    IA712_WAYPOINTS=1   -> suit une route de waypoints (mode manuel optionnel / repli démo).
#    IA712_HYBRID=1      -> explore puis balaie une route de couverture GÉNÉRIQUE.
echo ""
echo "[demo-tb4] ╔══════════════════════════════════════════╗"
echo "[demo-tb4] ║   Navigation en cours — suivre RViz2     ║"
echo "[demo-tb4] ║   Ctrl+C pour arrêter proprement         ║"
echo "[demo-tb4] ╚══════════════════════════════════════════╝"
echo ""

if [ "${IA712_HYBRID:-0}" = "1" ]; then
    # ── Mode HYBRIDE (optionnel) : exploration autonome pour la couverture, PUIS
    #    balayage d'une route de couverture GÉNÉRIQUE (centres de pièces / portes,
    #    AUCUNE coordonnée de victime — cf. waypoints_*.yaml) pour re-balayer la caméra.
    #    N.B. le mode L18 conforme est l'exploration autonome SEULE (branche par défaut).
    EXPLORE_SECS="${IA712_EXPLORE_SECS:-300}"
    COV_TARGET="${IA712_EXPLORE_COV:-0.85}"
    _RES="${IA712_RESULTS_DIR:-${REPO_ROOT}/results}"
    echo "[8/8] HYBRIDE phase 1/2 : exploration autonome (frontier) jusqu'à cov≥${COV_TARGET} ou ${EXPLORE_SECS}s"
    # Caméra always_on (détection fiable à chaque pose victime). La fuite mémoire du
    # rendu GPU est contenue à la source par le LIDAR ramené à 10 Hz (le leaker dominant,
    # cf. parcours.md #10) → plus besoin de couper la caméra (le faire la rendait fragile
    # au re-spawn du bridge → 0 image → détection ratée).
    ros2 launch "${EXP_LAUNCH}" \
        use_sim_time:=true \
        base_frame:="${NAMESPACE}/base_link" \
        >"${LOGDIR}/exploration.log" 2>&1 &
    EXP_PID=$!
    _s=0
    while kill -0 "${EXP_PID}" 2>/dev/null && [ "${_s}" -lt "${EXPLORE_SECS}" ]; do
        _cov=$(tail -1 "${_RES}/coverage_over_time.csv" 2>/dev/null | cut -d, -f2)
        if [ -n "${_cov}" ] && awk "BEGIN{exit !(${_cov} >= ${COV_TARGET})}" 2>/dev/null; then
            echo "  couverture ${_cov} ≥ ${COV_TARGET} atteinte (${_s}s) → patrouille"
            break
        fi
        sleep 5; _s=$(( _s + 5 ))
    done
    echo "[8/8] HYBRIDE : fin phase 1 (couverture) → arrêt de l'explorateur"
    pkill -9 -f "frontier_explorer" 2>/dev/null || true
    kill -9 "${EXP_PID}" 2>/dev/null || true
    sleep 3
    echo "[8/8] HYBRIDE phase 2/2 : balayage couverture générique → ${WAYPOINTS_FILE##*/}"
    ros2 launch "${WP_LAUNCH}" \
        waypoints_file:="${WAYPOINTS_FILE}" \
        loop:=false \
        use_sim_time:=true \
        >"${LOGDIR}/wp.log" 2>&1 &
    WP_PID=$!
elif [ "${IA712_WAYPOINTS:-0}" = "1" ]; then
    echo "[8/8] Waypoint follower (mode manuel optionnel) → ${WAYPOINTS_FILE##*/}"
    ros2 launch "${WP_LAUNCH}" \
        waypoints_file:="${WAYPOINTS_FILE}" \
        loop:=false \
        use_sim_time:=true \
        >"${LOGDIR}/wp.log" 2>&1 &
    WP_PID=$!
elif [ "${IA712_BT_MISSION:-1}" = "1" ] && [ "${IA712_BT:-1}" = "1" ] && [ -n "${BT_PID:-}" ]; then
    # DÉFAUT (L18, conforme énoncé) : MISSION ORCHESTRÉE PAR LE BEHAVIOR TREE.
    #   Le bt_runner (lancé en 7b) décide : ExplorePhase → InspectPhase → /mission_done.
    #   Ici on ne fait que LANCER les nœuds gated et attendre le signal du BT :
    #     - frontier_explorer : exploration, démarrée/arrêtée par /mission/explore_enable ;
    #     - inspection_node   : inactif jusqu'à /mission/inspect_enable, dérive les poses
    #       de la carte (zéro coordonnée victime) et les balaie face-mur.
    #   La séquence des phases est dans le BT, pas dans ce script (conformité « décision = BT »).
    echo "[8/8] Mission orchestrée par le Behavior Tree (explore → inspection → done)"
    _RES="${IA712_RESULTS_DIR:-${REPO_ROOT}/results}"
    ros2 launch "${EXP_LAUNCH}" \
        use_sim_time:=true \
        base_frame:="${NAMESPACE}/base_link" \
        >"${LOGDIR}/exploration.log" 2>&1 &
    EXP_PID=$!
    ros2 run rescue_robot inspection_node --ros-args \
        -p use_sim_time:=true \
        -p dwell_sec:=8.0 -p dwell_spin_speed:=0.4 \
        >"${LOGDIR}/inspection.log" 2>&1 &
    INSPECT_PID=$!
    # Attendre que le BT latch /mission_done (il le logge) ; garde-fou temps.
    MISSION_MAX="${IA712_MISSION_MAX_SECS:-600}"
    _s=0
    while [ "${_s}" -lt "${MISSION_MAX}" ]; do
        if grep -q "mission done published" "${LOGDIR}/bt.log" 2>/dev/null; then
            echo "[8/8] BT : /mission_done → finalisation"; break
        fi
        kill -0 "${BT_PID}" 2>/dev/null || { echo "[8/8] bt_runner arrêté → finalisation"; break; }
        sleep 5; _s=$(( _s + 5 ))
    done
    [ "${_s}" -ge "${MISSION_MAX}" ] && echo "[8/8] temps max mission ${MISSION_MAX}s → finalisation"
    pkill -9 -f "frontier_explorer" 2>/dev/null || true
    pkill -9 -f "inspection_node" 2>/dev/null || true
    kill -9 "${EXP_PID}" "${INSPECT_PID}" 2>/dev/null || true
    sleep 2
    WP_PID=""
else
    # REPLI (IA712_BT_MISSION=0, ou BT absent) : MÊME mission 2 phases mais orchestrée
    # par CE SCRIPT au lieu du BT (utile si rescue_decision n'est pas buildé). Identique
    # côté robot : exploration par frontières puis inspection des pièces dérivées de la
    # carte. Le mode L18 par défaut reste la mission orchestrée par le BT (branche ci-dessus).
    echo "[8/8] Phase 1/2 : Exploration autonome par frontières (repli shell, sans BT)"
    ros2 launch "${EXP_LAUNCH}" \
        use_sim_time:=true \
        base_frame:="${NAMESPACE}/base_link" \
        >"${LOGDIR}/exploration.log" 2>&1 &
    WP_PID=$!
    # ── Watchdog d'arrêt ── L'explorateur logge "EXPLORATION_DONE" quand il a fini
    #   (couverture cible OU plus de frontières) mais ne se termine pas lui-même
    #   (rclpy.shutdown depuis un callback ne débloque pas spin). On surveille donc
    #   ce marqueur — plus un plafond de couverture et un temps max de sécurité —
    #   puis on tue l'explorateur pour que le script finalise (carte + annotation).
    EXPLORE_MAX_SECS="${IA712_EXPLORE_MAX_SECS:-300}"   # borne le run AVANT que SLAM ne dérive
    EXPLORE_COV_CAP="${IA712_EXPLORE_COV_CAP:-0.95}"
    _RES="${IA712_RESULTS_DIR:-${REPO_ROOT}/results}"
    _s=0
    while kill -0 "${WP_PID}" 2>/dev/null && [ "${_s}" -lt "${EXPLORE_MAX_SECS}" ]; do
        if grep -q "EXPLORATION_DONE" "${LOGDIR}/exploration.log" 2>/dev/null; then
            echo "[8/8] EXPLORATION_DONE détecté (${_s}s) → finalisation"
            break
        fi
        _cov=$(tail -1 "${_RES}/coverage_over_time.csv" 2>/dev/null | cut -d, -f2)
        if [ -n "${_cov}" ] && awk "BEGIN{exit !(${_cov} >= ${EXPLORE_COV_CAP})}" 2>/dev/null; then
            echo "[8/8] plafond couverture ${_cov} ≥ ${EXPLORE_COV_CAP} (${_s}s) → finalisation"
            break
        fi
        sleep 5; _s=$(( _s + 5 ))
    done
    [ "${_s}" -ge "${EXPLORE_MAX_SECS}" ] && echo "[8/8] temps max ${EXPLORE_MAX_SECS}s atteint → fin exploration"
    pkill -9 -f "frontier_explorer" 2>/dev/null || true
    kill -9 "${WP_PID}" 2>/dev/null || true
    sleep 2

    # ── PHASE 2 (L18) : INSPECTION AUTONOME DES PIÈCES DÉCOUVERTES ──────────────
    #   La portée caméra (~2 m) ≪ portée LIDAR (12 m) : l'exploration cartographie les
    #   pièces d'angle depuis leurs portes sans amener la caméra à <2 m des tags muraux
    #   (cf. parcours §7ter → ~2 victimes en explo pure). On ajoute une 2ᵉ phase 100 %
    #   autonome : on dérive de LA CARTE que le robot a construite (zéro coordonnée de
    #   victime, zéro waypoint écrit à la main) une pose d'inspection par pièce extérieure,
    #   et on la balaie au spin 360°. = recherche systématique du périmètre découvert.
    if [ "${IA712_INSPECT:-1}" != "0" ]; then
        echo "[8/8] Phase 2 : sauvegarde carte → génération des poses d'inspection DEPUIS la carte"
        ros2 run nav2_map_server map_saver_cli -f "${_RES}/final_map" \
            --ros-args -p save_map_timeout:=10.0 >/dev/null 2>&1 \
            || echo "  [WARN] map_saver_cli (carte d'inspection) a échoué"
        INSPECT_WP="${_RES}/inspection_waypoints.yaml"
        python3 "${REPO_ROOT}/scripts/generate_inspection_waypoints.py" "${_RES}" "${INSPECT_WP}" 2>&1 \
            | sed 's/^/  /' || true
        if grep -q '{' "${INSPECT_WP}" 2>/dev/null; then
            echo "[8/8] Phase 2 : patrouille d'inspection (spin 360° par pièce) — SLAM/Nav2/caméra toujours actifs"
            ros2 launch "${WP_LAUNCH}" \
                waypoints_file:="${INSPECT_WP}" \
                loop:=false use_sim_time:=true \
                dwell_sec:=8.0 dwell_spin_speed:=0.4 \
                >"${LOGDIR}/inspection.log" 2>&1 &
            INSPECT_PID=$!
            # Le node waypoint s'arrête seul après la patrouille (rclpy.shutdown depuis son
            # thread de nav fonctionne, contrairement à l'explorateur) ; garde-fou temps.
            _is=0
            while kill -0 "${INSPECT_PID}" 2>/dev/null && [ "${_is}" -lt "${IA712_INSPECT_MAX_SECS:-300}" ]; do
                sleep 5; _is=$(( _is + 5 ))
            done
            kill -9 "${INSPECT_PID}" 2>/dev/null || true
            pkill -9 -f "waypoint_follower" 2>/dev/null || true
            echo "[8/8] Phase 2 terminée (${_is}s)"
        else
            echo "  [WARN] aucune pose d'inspection générée → on garde les victimes de l'exploration"
        fi
    fi
    WP_PID=""   # déjà arrêté ; on saute le `wait` ci-dessous
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

# ── Carte finale annotée avec les victimes (livrable L18) ────────────
# SLAM/Nav2 sont encore vivants ici → /map publié. On sauve la carte puis on
# l'annote depuis victims.json (vrais IDs AprilTag) via scripts/annotate_map.py.
RESULTS_DIR="${IA712_RESULTS_DIR:-${REPO_ROOT}/results}"
echo ""
echo "[final] Carte finale + annotation victimes → ${RESULTS_DIR}/"
ros2 run nav2_map_server map_saver_cli -f "${RESULTS_DIR}/final_map" \
    --ros-args -p save_map_timeout:=10.0 >/dev/null 2>&1 \
    && echo "  carte: final_map.{yaml,pgm}" \
    || echo "  [WARN] map_saver_cli échoué (SLAM déjà arrêté ?)"
python3 "${REPO_ROOT}/scripts/annotate_map.py" "${RESULTS_DIR}" 2>&1 | sed 's/^/  /' || true

echo ""
echo "Fichiers résultats : ${REPO_ROOT}/results/"
echo "Logs               : ${LOGDIR}/"
echo "══════════════════════════════════════════"
echo ""
