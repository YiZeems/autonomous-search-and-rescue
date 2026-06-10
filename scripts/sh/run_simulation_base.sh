#!/usr/bin/env bash
set -eo pipefail
export IA712_SIM_WORLD=world
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_simulation.sh"
