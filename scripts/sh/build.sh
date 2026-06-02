#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
cd "${WS_DIR}"
colcon build --symlink-install
# Use local_setup to avoid the "no such file: local_setup.sh" warning on
# Parallels/NFS mounts where the full setup re-sources the ROS 2 base chain.
if [ -f "${WS_DIR}/install/local_setup.bash" ]; then
  source "${WS_DIR}/install/local_setup.bash"
fi
echo "[OK] Workspace built. Run in your terminal: source ros2_ws/install/local_setup.bash"
