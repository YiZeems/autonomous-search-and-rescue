#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"
cd "${WS_DIR}"
colcon build --symlink-install
source "${WS_DIR}/install/setup.bash"
echo "[OK] Workspace built and sourced."
