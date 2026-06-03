# Testing Guide

This document explains how to test the repository in a simple way.

## From repository root

Always run scripts from the repository root.

## Build

```bash
./scripts/run.sh install-apt
./scripts/run.sh doctor-env
./scripts/run.sh build
```

This calls:

```text
scripts/sh/build.sh
```

It sources ROS 2 Humble and builds `ros2_ws`.

## Mock system

```bash
./scripts/run.sh mock
```

This launches fake publishers for early development:

- `/map`
- `/victims_map`
- `/coverage`

It also launches result/visualization nodes.

## Module tests

Each module can be tested independently:

```bash
./scripts/run.sh simulation
./scripts/run.sh navigation
./scripts/run.sh exploration
./scripts/run.sh detection
./scripts/run.sh results
./scripts/run.sh bt
```

Some modules are still skeletons. A skeleton that starts cleanly is acceptable at the base-validation stage.

## Clean generated files

```bash
./scripts/run.sh clean
```

This removes:

```text
ros2_ws/build
ros2_ws/install
ros2_ws/log
```

## Python syntax check without ROS

If ROS is not installed, you can still check Python syntax:

```bash
python3 - <<'PY'
from pathlib import Path
import ast
for path in list(Path("ros2_ws/src/rescue_robot/rescue_robot").rglob("*.py")) + list(Path("scripts").glob("*.py")):
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
print("Python syntax OK")
PY
```

This does not validate ROS imports at runtime, but it catches syntax errors without creating `__pycache__` files.

## Shell script syntax check

```bash
bash -n scripts/run.sh
for f in scripts/sh/*.sh; do bash -n "$f"; done
```

## CI checks

GitHub Actions runs `.github/workflows/basic-checks.yml` on PRs to `dev` and `main`.

The CI checks:

- no generated ROS folders are committed;
- required files exist;
- Python syntax is valid.

## Lightweight uv tests

Before pushing a branch, members can run:

```bash
./scripts/run.sh uv-sync
./scripts/run.sh uv-test
```

These tests verify structure, Python syntax and basic project hygiene without launching ROS 2. They are useful for B/C/D even when they do not have the full ROS/Gazebo stack running yet.

For linting:

```bash
./scripts/run.sh uv-lint
```

These commands are not a substitute for the final ROS test:

```bash
source /opt/ros/humble/setup.bash
./scripts/run.sh build
./scripts/run.sh mock
```
