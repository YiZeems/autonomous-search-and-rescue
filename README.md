# IA712 Search and Rescue

Project B — Autonomous Search and Rescue.

## Goal
Build a ROS 2/Gazebo system where a mobile robot explores an unknown disaster-like environment, builds a map, detects victims, localizes them in the `map` frame, and exports final results.

## Reference stack
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Gazebo Classic 11
- TurtleBot3
- slam_toolbox
- Nav2
- RViz2


## Documentation map

Start with these files:

- `docs/START_HERE.md` — first file to read for everyone.
- `docs/INSTALLATION.md` — full environment setup: ROS 2, Gazebo, uv, Git workflow, branch protection.
- `AGENTS.md` — safety rules for AI coding agents.

Then by topic:

- `docs/project.md` — project summary, goals, constraints, and roadmap.
- `docs/team_roles.md` — roles A/B/C/D, file ownership, and onboarding for each member.
- `docs/full_architecture.md` — complete layer-by-layer system architecture.
- `docs/interfaces.md` — all ROS topics and file interfaces.
- `docs/architecture_questions_and_decisions.md` — design decisions and professor critique.
- `docs/ros2_package_structure.md` — explains the `rescue_robot/rescue_robot` path structure.
- `docs/simulation_setup.md` — AMD64/WSL2 vs ARM64 Gazebo setup, Waffle Pi vs Burger, TurtleBot4.
- `docs/testing_guide.md` — how to test the scaffold and each module.
- `docs/validation.md` — checklist before professor validation.

## Branch workflow
Simple workflow for the team:

- `main`: stable demo only.
- `dev`: integration branch, managed by A.
- `a-integration`: base, interfaces, launch, integration.
- `b-exploration`: exploration module.
- `c-simulation`: Gazebo world, models, robot setup.
- `d-results`: metrics, visualization, export.

Team members should work only in their own module folders. Integration files are owned by A.

## Installation

Follow the complete setup guide first:

```text
docs/INSTALLATION.md
```

## Build
```bash
source /opt/ros/humble/setup.bash
./scripts/run.sh build
```



## Easy shell scripts
The professor asked for simple executable scripts. Use the orchestrator below from the repository root:

```bash
./scripts/run.sh build
./scripts/run.sh mock
./scripts/run.sh bringup
```

Module-level commands are also available:

```bash
./scripts/run.sh check-tb3
./scripts/run.sh simulation
./scripts/run.sh navigation
./scripts/run.sh exploration
./scripts/run.sh detection
./scripts/run.sh results
./scripts/run.sh bt
```

Each command calls a small executable script in `scripts/sh/`. This keeps testing simple while still allowing each module to be launched independently.

## Behavior Tree
The project-level Behavior Tree scaffold is located in:

```text
ros2_ws/src/rescue_robot/behavior_trees/search_and_rescue_bt.xml
ros2_ws/src/rescue_robot/rescue_robot/bt/bt_supervisor_node.py
ros2_ws/src/rescue_robot/launch/bt.launch.py
```

## Mock system launch
This launch lets everyone test without waiting for Gazebo/Nav2/SLAM.

```bash
cd ros2_ws
source install/setup.bash
ros2 launch rescue_robot mock_system.launch.py
```

Expected topics:

```bash
ros2 topic list
# /map
# /victims_map
# /coverage
# /visualization_marker_array
```

## Final target launch
```bash
ros2 launch rescue_bringup bringup.launch.py
```

At the beginning, `bringup.launch.py` may only include mock modules. It will progressively include simulation, SLAM, Nav2, exploration, detection, results, and Behavior Tree supervision.


## Before pushing to GitHub

`.github/CODEOWNERS` is already configured for `@YiZeems`. If A changes, replace `@YiZeems` with the new integration lead GitHub username. Then create the branches and branch-protection rules described in `docs/INSTALLATION.md`.

## Python dev environment with uv

The project keeps ROS 2 runtime separate from the Python development venv.

Use uv for lightweight tests, result scripts and linting:

```bash
./scripts/run.sh uv-sync
./scripts/run.sh uv-test
./scripts/run.sh uv-lint
```

The uv environment is pinned to Python 3.10 through `.python-version` and `pyproject.toml`, matching Ubuntu 22.04 / ROS 2 Humble. Do not install ROS 2 packages such as `rclpy` through uv or pip; they come from the ROS 2 Humble installation.

More details: [`docs/INSTALLATION.md`](docs/INSTALLATION.md).

## Real simulation smoke test

After the mock system is validated, run the first real robot/Gazebo test:

```bash
./scripts/run.sh build
./scripts/run.sh simulation
```

In a second terminal, drive the robot manually:

```bash
./scripts/run.sh teleop
```

See `docs/real_simulation_smoke_test.md` for the full procedure.

- [Robot model decision](docs/simulation_setup.md) — why Waffle Pi is the default instead of Burger.

- [Gazebo world selection](docs/simulation_setup.md)

- [TurtleBot4 experimental setup](docs/simulation_setup.md)

## Gazebo safe graphics mode

If Gazebo freezes on the house world with a message such as `gazebo is not responding` or `Preparing your world`, use the safe graphics launcher:

```bash
./scripts/run.sh simulation-house-safe
```

This is especially useful on macOS + Parallels + Ubuntu ARM64 and can also help on WSL2/WSLg. See `docs/simulation_setup.md`.
