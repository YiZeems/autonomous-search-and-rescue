#!/usr/bin/env bash
# Profileur mémoire : échantillonne le RSS par CATÉGORIE de process toutes les N s
# pendant un run, pour identifier le node qui FUIT (RSS qui croît dans le temps).
# Le run long faisait OOM (~0.6 GB/min) ; ce CSV dit lequel grossit (SLAM ? bridges ?).
#
#   scripts/sh/mem_profile.sh [interval_s=30] [out=/tmp/mem_profile.csv]
# Puis :  column -s, -t /tmp/mem_profile.csv   (ou regarder quelle colonne monte)
set -o pipefail
INT="${1:-30}"
OUT="${2:-/tmp/mem_profile.csv}"

# RSS total (MB) des process dont la cmdline matche le motif $1
_rss() { ps -eo rss,args 2>/dev/null | grep -E -- "$1" | grep -v grep | awk '{s+=$1} END{printf "%d", s/1024}'; }

echo "t_s,mem_used_mb,swap_mb,gazebo_mb,slam_mb,nav2_mb,bridges_mb,rescue_nodes_mb,tb4_create3_mb,top1" > "$OUT"
T0=$(date +%s)
echo "[mem_profile] échantillonnage toutes les ${INT}s → ${OUT}"
while true; do
  t=$(( $(date +%s) - T0 ))
  used=$(free -m | awk '/Mem/{print $3}')
  sw=$(free -m | awk '/Swap/{print $3}')
  gz=$(_rss 'ign gazebo|gz sim|ign_gazebo')
  slam=$(_rss 'slam_toolbox')
  nav2=$(_rss 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|lifecycle_manager|waypoint_follower')
  br=$(_rss 'parameter_bridge|ros_gz_bridge')
  resc=$(_rss 'rescue_robot/lib')
  tb4=$(_rss 'irobot_create|create3|wheel_status|motion_control|hazards_vector|sensors_node|turtlebot4')
  top=$(ps -eo rss,comm --sort=-rss 2>/dev/null | awk 'NR==2{printf "%s=%dMB", $2, $1/1024}')
  echo "${t},${used},${sw},${gz},${slam},${nav2},${br},${resc},${tb4},${top}" >> "$OUT"
  sleep "${INT}"
done
