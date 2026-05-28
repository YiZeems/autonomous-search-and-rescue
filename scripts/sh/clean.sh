#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"
rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
echo "[OK] Removed ros2_ws/build, ros2_ws/install and ros2_ws/log."
