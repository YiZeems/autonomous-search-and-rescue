# Team Roles and File Ownership

This document describes who does what and who owns which files. The rule is simple: **if a file is not in your area, do not modify it without asking the owner (A).**

---

## Ownership summary table

| Area | Owner | Key files |
|---|---|---|
| Integration, BT, base | **A** | `README.md`, `.github/`, `launch/bringup.launch.py`, `mocks/`, `bt/` |
| Exploration | **B** | `exploration/`, `launch/exploration.launch.py` |
| Simulation | **C** | `worlds/`, `models/`, `launch/simulation.launch.py` |
| Results / metrics | **D** | `results/`, `launch/results.launch.py`, `scripts/*.py` |

Generated outputs go to `results/`. Temporary outputs go to `results/tmp/` (ignored by Git).

---

## Role A — Integration, Base, Interfaces, Behavior Tree

### Main responsibility

A is responsible for the common project base and integration. A does not write everyone's code, but makes sure the project remains buildable, organized, and easy to launch.

### Files owned by A

```text
README.md
CONTRIBUTING.md
.github/
docs/interfaces.md
docs/architecture.md  →  now merged into docs/full_architecture.md
docs/github_protection.md  →  now in docs/INSTALLATION.md
ros2_ws/src/ia712_search_rescue/package.xml
ros2_ws/src/ia712_search_rescue/setup.py
ros2_ws/src/ia712_search_rescue/setup.cfg
ros2_ws/src/ia712_search_rescue/launch/bringup.launch.py
ros2_ws/src/ia712_search_rescue/launch/mock_system.launch.py
ros2_ws/src/ia712_search_rescue/launch/navigation.launch.py
ros2_ws/src/ia712_search_rescue/launch/bt.launch.py
ros2_ws/src/ia712_search_rescue/config/slam_params.yaml
ros2_ws/src/ia712_search_rescue/config/nav2_params.yaml
ros2_ws/src/ia712_search_rescue/config/bt_params.yaml
ros2_ws/src/ia712_search_rescue/behavior_trees/
ros2_ws/src/ia712_search_rescue/ia712_search_rescue/mocks/
ros2_ws/src/ia712_search_rescue/ia712_search_rescue/bt/
scripts/run.sh
scripts/sh/
```

### What A must know

- ROS 2 package structure and `colcon build`
- Launch files and `rclpy` basics
- Git branches and merges
- GitHub branch protection
- How to debug topics with `ros2 topic list`, `ros2 topic echo`
- How to run and test the mock system

### First tasks

1. Push the scaffold to GitHub.
2. Check that `.github/CODEOWNERS` contains the integration lead username (`@YiZeems`).
3. Create branches: `dev`, `a-integration`, `b-exploration`, `c-simulation`, `d-results`.
4. Configure GitHub protection for `main` and `dev`.
5. Check that CI passes.
6. Ask each member to work only in their folder.
7. Keep `dev` buildable.

### Integration policy

A merges modules into `dev` one by one, in this order:
1. simulation → 2. navigation/SLAM → 3. exploration → 4. detection → 5. results → 6. BT/bringup

After each merge:

```bash
./scripts/run.sh build
./scripts/run.sh mock
```

### Commands

```bash
./scripts/run.sh build
./scripts/run.sh mock
./scripts/run.sh bringup
./scripts/run.sh bt
```

---

## Role B — Exploration Autonomous Module

### Main responsibility

B is responsible for autonomous exploration. The module reads the map, finds frontiers, selects exploration goals, and sends them to Nav2.

### Files owned by B

```text
ros2_ws/src/ia712_search_rescue/ia712_search_rescue/exploration/
ros2_ws/src/ia712_search_rescue/launch/exploration.launch.py
ros2_ws/src/ia712_search_rescue/config/explorer_params.yaml
```

### What B should not modify

`setup.py`, `package.xml`, `bringup.launch.py`, `README.md`, `.github/`, `docs/interfaces.md` — ask A for those.

### What B must know

- `nav_msgs/msg/OccupancyGrid` — map values: `-1` unknown, `0` free, `100` occupied
- Frontier-based exploration
- Basic Nav2 goal sending using `NavigateToPose` action
- Coverage metric basics

### Development phases

| Phase | Goal |
|---|---|
| B1 | Parse occupancy grid from `/map` |
| B2 | Detect frontiers (boundary between free and unknown cells) |
| B3 | Select exploration goal — start with nearest, then `score = info_gain / distance` |
| B4 | Send goal to Nav2 via `NavigateToPose` action |
| B5 | Stop when coverage ≥ 90% or no frontier remains |

### Commands

```bash
./scripts/run.sh mock
./scripts/run.sh exploration
```

### Definition of done

- Works with mock `/map` and real `/map` from SLAM
- Sends goals to Nav2 automatically without RViz clicks
- Map coverage can reach the 90% target

---

## Role C — Gazebo Simulation, World, Robot, Victims

### Main responsibility

C is responsible for the simulation environment: world, robot spawn, sensors, and victim markers.

### Files owned by C

```text
ros2_ws/src/ia712_search_rescue/worlds/
ros2_ws/src/ia712_search_rescue/models/
ros2_ws/src/ia712_search_rescue/launch/simulation.launch.py
ros2_ws/src/ia712_search_rescue/config/simulation_params.yaml
```

### What C should not modify

`setup.py`, `package.xml`, `bringup.launch.py`, `frontier_explorer_node.py`, `victim_detector_node.py`, `result_exporter_node.py` — ask A for those.

### What C must know

- Gazebo world files
- How TurtleBot3 is spawned
- Robot sensors: LiDAR, camera, odometry
- How to verify ROS topics from simulation
- How to place simple victim markers or colored cylinders

### Development phases

| Phase | Goal |
|---|---|
| C1 | Small test world (`worlds/test_world_small.world`) with walls, spawn, 1 victim |
| C2 | Validate sensor topics: `/scan`, `/odom`, `/tf`, `/camera/image_raw`, `/camera/camera_info` |
| C3 | Final disaster world (`worlds/disaster_world.world`) with rooms, corridors, 3–5 victims |
| C4 | Document victim ground truth positions (`victim_1: x, y`, …) |

### Commands

```bash
./scripts/run.sh simulation
```

### Definition of done

- Small and final worlds launch correctly
- Robot spawns, LiDAR and camera topics exist
- Victims are visible and ground truth is documented

---

## Role D — Metrics, Visualization, Results Export

### Main responsibility

D turns the robot run into measurable and presentable results. The module must work with mock topics before the full stack is ready.

### Files owned by D

```text
ros2_ws/src/ia712_search_rescue/ia712_search_rescue/results/
ros2_ws/src/ia712_search_rescue/launch/results.launch.py
ros2_ws/src/ia712_search_rescue/config/results_params.yaml
scripts/plot_coverage.py
scripts/annotate_map.py
scripts/generate_run_summary.py
results/
```

### What D should not modify

`setup.py`, `package.xml`, `bringup.launch.py`, `frontier_explorer_node.py`, `victim_detector_node.py`, `worlds/` — ask A for those.

### What D must know

- `/map` as `nav_msgs/msg/OccupancyGrid`
- `/coverage` as `std_msgs/msg/Float32`
- `/victims_map` as `geometry_msgs/msg/PoseArray`
- RViz `MarkerArray` basics
- CSV and JSON export

### Development phases

| Phase | Output file | Columns |
|---|---|---|
| D1 | Mock testing with `./scripts/run.sh mock` + `./scripts/run.sh results` | — |
| D2 | `results/coverage_over_time.csv` | `time, coverage` |
| D3 | `results/victims_detected.csv` | `id, x, y` (+ ground truth columns if available) |
| D4 | `results/run_summary.json` | `final_coverage`, `victims_detected`, `success_coverage_90` |
| D5 | Coverage curve + annotated map figures for report | — |

### Commands

```bash
./scripts/run.sh mock
./scripts/run.sh results
```

### Definition of done

- Mock data and real run data can be exported
- RViz markers show victims
- Final summary and figures exist
