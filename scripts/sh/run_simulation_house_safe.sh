#!/usr/bin/env bash
set -eo pipefail
export IA712_SIM_WORLD=house
export IA712_GAZEBO_SAFE_GRAPHICS=1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_simulation.sh"
