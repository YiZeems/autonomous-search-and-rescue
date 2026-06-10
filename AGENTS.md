# AGENTS.md — Safety Rules for AI Coding Agents

This file is for AI coding agents used on this repository, for example Cursor, Copilot Agent, Codex, Claude Code or ChatGPT-generated patches.

The goal is to prevent an agent from breaking the ROS 2 project, changing shared interfaces without approval, or creating Git conflicts.

## Repository context

This repository is for the IA712 Mobile Robotics final project, Project B: autonomous search and rescue.

The final system should use ROS 2, Gazebo, SLAM, Nav2, victim detection, `tf2`, a Behavior Tree, a one-command bringup, and GitHub version control.

## Hard rule

Do not make broad changes. Prefer small, local, reviewable edits.

Before editing, identify which role/module the requested task belongs to:

- A: integration, base ROS 2 package, interfaces, GitHub, launch orchestration, Behavior Tree scaffold.
- B: exploration module.
- C: simulation module.
- D: results, metrics, visualization and export module.

## Path convention

All paths below are relative to the repository root.

Package root:

```text
ros2_ws/src/rescue_robot/
```

Python package root:

```text
ros2_ws/src/rescue_robot/rescue_robot/
```

Do not rename or remove the inner Python package folder. The repeated path below is intentional for an `ament_python` ROS 2 package:

```text
ros2_ws/src/rescue_robot/rescue_robot/
```

For the full explanation, read:

```text
docs/ros2_package_structure.md
```

## Files that are dangerous to modify

Do not modify these files unless the task explicitly says to change integration/shared files:

```text
README.md
CONTRIBUTING.md
AGENTS.md
.github/
pyproject.toml
.python-version
ros2_ws/src/rescue_robot/package.xml
ros2_ws/src/rescue_robot/setup.py
ros2_ws/src/rescue_robot/setup.cfg
ros2_ws/src/rescue_bringup/launch/bringup.launch.py
ros2_ws/src/rescue_robot/launch/mock_system.launch.py
ros2_ws/src/rescue_robot/config/nav2_params.yaml
ros2_ws/src/rescue_robot/config/slam_params.yaml
docs/interfaces.md
scripts/run.sh
scripts/sh/_common.sh
```

If changing one of these is necessary, explain why in the commit/PR message.

## Module ownership

### A — Integration

Allowed paths:

```text
README.md
CONTRIBUTING.md
AGENTS.md
.github/
docs/interfaces.md
docs/INSTALLATION.md
docs/INSTALLATION.md
docs/team_roles.md
ros2_ws/src/rescue_robot/package.xml
ros2_ws/src/rescue_robot/setup.py
ros2_ws/src/rescue_bringup/launch/bringup.launch.py
ros2_ws/src/rescue_robot/launch/mock_system.launch.py
ros2_ws/src/rescue_robot/launch/bt.launch.py
ros2_ws/src/rescue_robot/config/bt_params.yaml
ros2_ws/src/rescue_robot/behavior_trees/
ros2_ws/src/rescue_robot/rescue_robot/bt/
ros2_ws/src/rescue_robot/rescue_robot/mocks/
scripts/run.sh
scripts/sh/
```

### B — Exploration

Allowed paths:

```text
ros2_ws/src/rescue_robot/rescue_robot/exploration/
ros2_ws/src/rescue_robot/launch/exploration.launch.py
ros2_ws/src/rescue_robot/config/explorer_params.yaml
docs/team_roles.md
docs/team_roles.md
```

### C — Simulation

Allowed paths:

```text
ros2_ws/src/rescue_robot/worlds/
ros2_ws/src/rescue_robot/models/
ros2_ws/src/rescue_robot/launch/simulation.launch.py
ros2_ws/src/rescue_robot/config/simulation_params.yaml
docs/team_roles.md
docs/team_roles.md
```

### D — Results and visualization

Allowed paths:

```text
ros2_ws/src/rescue_robot/rescue_robot/results/
ros2_ws/src/rescue_robot/launch/results.launch.py
ros2_ws/src/rescue_robot/config/results_params.yaml
scripts/annotate_map.py
scripts/generate_run_summary.py
scripts/plot_coverage.py
docs/team_roles.md
docs/team_roles.md
```

## Interfaces that must remain stable

Do not rename these topics without explicitly updating `docs/interfaces.md`, launch files, config files and all dependent nodes:

```text
/map
/victims_map
/coverage
/visualization_marker_array
/camera/image_raw
/camera/camera_info
/tf
/tf_static
```

Do not change these output file names without updating docs and result scripts:

```text
results/coverage_over_time.csv
results/victims_detected.csv
results/run_summary.json
results/final_map_annotated.png
```

## ROS 2 and uv separation

ROS 2 runtime dependencies must come from Ubuntu/ROS packages, not from pip or uv.

Do not add `rclpy`, `tf2_ros`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `visualization_msgs` or other ROS packages to `pyproject.toml`.

Use uv only for:

```text
pytest
ruff
offline scripts
non-ROS Python tooling
```

## Dependency rules

Do not add heavy dependencies unless they are clearly needed.

Avoid adding:

```text
YOLO frameworks
PyTorch
TensorFlow
large datasets
large binary assets
```

The project statement allows simple victim markers such as AprilTag, ArUco, QR code or colored objects. A heavy human detector is not needed for the MVP.

## Generated files must not be committed

Never commit:

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
.venv/
__pycache__/
*.pyc
*.bag
*.db3
*.sqlite3
results/* except results/README.md and curated examples
```

## Required checks after editing

After non-ROS edits:

```bash
./scripts/run.sh uv-test
./scripts/run.sh uv-lint
```

After shell-script edits:

```bash
bash -n scripts/run.sh
for f in scripts/sh/*.sh; do bash -n "$f"; done
```

After ROS package edits on a ROS 2 Humble machine:

```bash
source /opt/ros/humble/setup.bash
./scripts/run.sh build
./scripts/run.sh mock
```

## When to stop and ask

Stop and ask before doing any of the following:

- renaming package `rescue_robot`;
- changing branch/GitHub workflow rules;
- changing `setup.py` entry points;
- changing shared topic names;
- changing `bringup.launch.py` architecture;
- deleting mocks;
- replacing the single-package structure with multiple ROS packages;
- adding heavy dependencies;
- changing Python version away from 3.10;
- changing ROS distribution away from Humble.

## Good agent behavior

A good patch should:

- touch only the files needed for the requested module;
- keep changes small;
- preserve existing interfaces;
- update docs if it changes behavior;
- explain what was changed and how to test it;
- avoid speculative rewrites.

## Real simulation launch rule

`simulation.launch.py` is intentionally connected to the upstream
`turtlebot3_gazebo` launch file. Do not replace it with mocks. The mock path is
`mock_system.launch.py`; the real Gazebo path is `simulation.launch.py`.

Do not make `bringup.launch.py` point back to the mock system unless explicitly
asked. The current bringup validates the real simulation interface first.


## TurtleBot3 Gazebo architecture-specific rule

Do not assume that `turtlebot3_gazebo` is installed through apt on every machine. AMD64/WSL2 users may have it from apt; ARM64 users may need a source overlay. Use `./scripts/run.sh check-tb3` and follow `docs/simulation_setup.md`. Do not hardcode a personal overlay path in shared scripts unless it is only used as an optional auto-detected fallback.

## Robot model rule (updated 2026-06-09 after team merge)

**The project default robot is now TurtleBot 4 Standard (Create 3 + RPLIDAR A1M8 + OAK-D-Pro)**
under Ignition Gazebo Fortress, validated runtime on 2026-06-09 (60 nodes alive,
topics publishing, single Ignition instance — see `docs/MERGE_NOTES.md`).

The legacy TurtleBot 3 Waffle Pi + Gazebo Classic 11 path remains supported as a
fallback (useful on macOS Parallels ARM64 where Ignition rendering is unstable).

Rules:
- **TB4 default:** use `bringup_tb4.launch.py` for any new feature, demo, benchmark.
- **TB3 fallback:** use `bringup.launch.py` only if Ignition fails on a teammate's
  machine, or for the `simulation-house-safe` Mac Parallels scenario.
- **Never revert** to Burger as default (no camera → no victim detection).
- Never silently rename `bringup_tb4.launch.py` or break its public arguments
  (`world`, `model`, `launch_slam`, `launch_nav2`, `launch_apriltag`, `launch_rviz`,
  `headless`, `use_sim_time`, `algo`, `run_id`).

## Gazebo / Ignition world policy

- **TB4 default worlds (Ignition Fortress):** `warehouse` (default), `depot`, `maze`.
  Select via `world:=<name>` to `bringup_tb4.launch.py`. For the maze/loop-closure
  scenario, prefer `world:=maze`.
- **Custom Ignition world:** `rescue_world/worlds/rescue_arena.sdf` (auto-generated
  by `scripts/generate_rescue_arena.py`, reproducible with a seed — use for the
  bonus comparative benchmark greedy vs info-gain).
- **TB3 fallback worlds (Gazebo Classic):** `turtlebot3_house.world` (default
  if running `bringup.launch.py`), `disaster_world.world`.
- Do not hard-code a new world path in multiple files; preserve the `world:=` interface.

## Ignition / TurtleBot 4 safety rule

Do not silently remove the TB4 stack now that it is the default. If you must
disable Ignition (e.g. on macOS Parallels where `Ogre2 gpu_lidar` returns
`range_min` everywhere — see ERRORS_AND_FIXES #10), use `bringup.launch.py`
(TB3) and document the reason in `docs/ERRORS_AND_FIXES.md`.

Do not commit machine-specific overlay paths.


## Gazebo safe graphics rule for agents

Do not remove `simulation-house-safe` or `IA712_GAZEBO_SAFE_GRAPHICS`. The house world may freeze on Parallels ARM64 or WSLg unless Gazebo is launched with safe graphics settings. Any changes to simulation scripts must preserve:

```bash
./scripts/run.sh simulation-house-safe
IA712_GAZEBO_SAFE_GRAPHICS=1 ./scripts/run.sh simulation
```
