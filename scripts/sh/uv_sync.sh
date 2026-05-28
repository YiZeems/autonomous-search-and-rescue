#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is not installed. Install it with:"
  echo "  curl -LsSf https://astral.sh/uv/install.sh | sh"
  exit 1
fi

cd "${REPO_ROOT}"
echo "[uv] Syncing Python 3.10 development environment"
echo "[uv] This installs only the dev group. The optional vision group is documented in docs/python_uv_setup.md."
uv sync --group dev
