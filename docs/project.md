# Project — IA712 Autonomous Search and Rescue

## Official project

We selected **Project B — Autonomous Search and Rescue**.

The goal is to build a ROS 2/Gazebo system where a mobile robot explores an unknown disaster-like simulated environment, builds a map, detects victims represented by simple markers, localizes them in the global `map` frame, and exports final results.

---

## Final expected behavior

The final demonstration should show the following sequence:

1. One command launches the system.
2. Gazebo starts with the disaster environment and the robot.
3. SLAM starts and publishes `/map`.
4. Nav2 starts and accepts navigation goals.
5. The exploration node chooses goals automatically.
6. The robot explores the unknown environment without teleoperation.
7. The victim detector identifies markers or colored objects.
8. Victim positions are transformed into the `map` frame with `tf2`.
9. The results module exports coverage, victims, and summary files.
10. RViz displays the robot, map, goals, victims, and useful markers.

---

## Minimal viable project

- TurtleBot3 in Gazebo
- SLAM with `slam_toolbox`
- Nav2 navigation
- Frontier-based autonomous exploration
- Coverage evaluation with target above 90%
- Simple victim detection using colored cylinders or ArUco/AprilTag
- Victim coordinates in `map` frame
- Final annotated map and CSV/JSON results
- Behavior Tree scaffold used for high-level orchestration
- Single launch entrypoint

## Optional bonus

Compare quantitatively two exploration strategies (e.g. greedy frontier vs information-gain). Metrics: coverage over time, time to 90%, distance travelled, victims found, localization error.

---

## Architecture principle

Modular architecture — each component has a clear owner and interface. Two modes:

- **Mock mode:** fake `/map`, `/victims_map`, and `/coverage` for early module development.
- **Real mode:** Gazebo, SLAM, Nav2, exploration, detection, results integrated progressively.

### Main data flow

```text
Gazebo world
  -> robot sensors: /scan, /camera/image_raw, /odom, /tf
  -> slam_toolbox: /map
  -> frontier_explorer_node: navigation goals
  -> Nav2: /cmd_vel
  -> victim_detector_node: victim detections
  -> tf2: camera frame to map frame -> /victims_map
  -> result_exporter_node and rviz_marker_node
```

---

## Constraints

### Official constraints

- Use **ROS 2**
- Use **Gazebo** simulation
- Use **Behavior Trees** rather than a classical FSM
- Provide a **one-command launch** for the full stack
- Use **GitHub** for version control
- Provide custom nodes, launch files, Gazebo worlds, and documentation
- Provide a final report and presentation

### Project B specific constraints

- Explore unknown disaster-like environment autonomously
- Build a map during exploration
- Map at least **90%** of the explorable area
- Detect victims (simple markers or colored objects)
- Estimate victim positions in the global `map` frame using `tf2`
- Produce a final map with victim locations and quantitative results

### Technical stack

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Python | 3.10 |
| Simulator | Gazebo Classic 11 |
| Robot | TurtleBot3 Waffle Pi |
| SLAM | `slam_toolbox` |
| Navigation | Nav2 |
| Visualization | RViz2 |

### Intentionally avoided at first

- YOLO or heavy deep learning detection
- Custom robot model from scratch
- Multi-robot coordination
- Reinforcement learning
- Complex disaster world before small test world works
- CI pipeline that launches Gazebo/Nav2

### Base acceptance criteria

- `colcon build` works on Ubuntu 22.04 + ROS 2 Humble
- `./scripts/run.sh mock` launches mock publishers
- Module launch files exist and are separate
- Each role has a clearly separated folder
- GitHub rules protect `main` and `dev`
- Generated folders are ignored by Git
- Architecture and interfaces are documented

---

## Roadmap

| Session | Milestone |
|---|---|
| **L13** | Team roles fixed, repository created, interfaces fixed, mock system available |
| **L14** | Package structure stable, launch files split by module, small Gazebo world started |
| **L15–L16** | Exploration, simulation, detection, results developed in parallel — mocks gradually replaced |
| **L17** | Full bringup launch, debug and repeated test runs |
| **L18** | Demo, report, slides, final tagged version |
