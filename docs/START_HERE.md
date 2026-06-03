# Start Here

Read this file first.

## For the professor

The repository is a scaffold for Project B — Autonomous Search and Rescue. It is designed to be validated before full implementation.

Important files:

- `docs/INSTALLATION.md`
- `docs/project.md`
- `docs/full_architecture.md`
- `docs/ros2_package_structure.md`
- `docs/interfaces.md`
- `docs/validation.md`

## For A

Read:

- `docs/team_roles.md`
- `docs/INSTALLATION.md`

Then configure GitHub protection and push the initial branches.

## For B

Read:

- `docs/team_roles.md`
- `docs/interfaces.md`

Start from the mock `/map`.

## For C

Read:

- `docs/team_roles.md`
- `docs/simulation_setup.md`

Start with `test_world_small.world` before the final disaster world.

## For D

Read:

- `docs/team_roles.md`
- `docs/testing_guide.md`

Start with mock `/coverage` and `/victims_map`.

## First commands

From the repository root:

```bash
./scripts/run.sh build
./scripts/run.sh mock
```

If ROS 2 is not installed, use syntax checks only:

```bash
python3 - <<'PY'
from pathlib import Path
import ast
for path in list(Path("ros2_ws/src/rescue_robot/rescue_robot").rglob("*.py")) + list(Path("scripts").glob("*.py")):
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
print("Python syntax OK")
PY
bash -n scripts/run.sh
for f in scripts/sh/*.sh; do bash -n "$f"; done
```

- `docs/validation.md` — last scaffold audit and remaining real ROS test.
- `AGENTS.md` — instructions for AI coding agents.

## Optional but recommended: uv development setup

For offline scripts and lightweight tests:

```bash
./scripts/run.sh uv-sync
./scripts/run.sh uv-test
```

This does not replace ROS 2 Humble. It only creates a Python 3.10 development venv for tests, result scripts and linting. See `docs/INSTALLATION.md`.

## First real robot test

Once `./scripts/run.sh mock` works, test the real TurtleBot3 Gazebo interface:

```bash
./scripts/run.sh simulation
```

Then open a second terminal and run:

```bash
./scripts/run.sh teleop
```

Read `docs/real_simulation_smoke_test.md` before integrating SLAM/Nav2.

## TurtleBot3 Gazebo on AMD64 vs ARM64

See `docs/simulation_setup.md` before running the real simulation. AMD64/WSL2 users usually install TurtleBot3 Gazebo through apt. ARM64 users may need a source overlay.

## If Gazebo freezes on the house world

Use:

```bash
./scripts/run.sh simulation-house-safe
```

Read `docs/simulation_setup.md` for the explanation.
