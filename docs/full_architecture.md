# Architecture — IA712 Search and Rescue

> **Quick overview** — mission flow in one block:
>
> ```text
> Gazebo simulation
>   -> TurtleBot3 simulated robot
>   -> sensors: /scan, /odom, /camera/image_raw, /camera/camera_info, /tf
>   -> slam_toolbox builds /map
>   -> Nav2 receives exploration goals and publishes /cmd_vel
>   -> frontier_explorer_node reads /map and selects goals
>   -> victim_detector_node detects markers and localizes victims in map with tf2
>   -> coverage_evaluator_node computes /coverage
>   -> bt_supervisor_node monitors mission-level conditions
>   -> rviz_marker_node visualizes victims
>   -> result_exporter_node writes CSV/JSON outputs
> ```
>
> For the ROS 2/Python folder layout see `docs/ros2_package_structure.md`.
> For team roles and file ownership see `docs/team_roles.md`.

---

This document explains the complete intended architecture of the IA712 Project B scaffold.

The repository currently contains a working scaffold with mocks. The final project will progressively replace the mocks with real Gazebo, SLAM, Nav2, camera and `tf2` components.

---

## 1. Final mission

The robot must:

```text
1. start in a simulated disaster-like environment;
2. explore autonomously without teleoperation;
3. build a map with SLAM;
4. detect victims represented by simple markers;
5. transform victim positions into the global map frame;
6. produce a final annotated map and result files.
```

The target final command is:

```bash
ros2 launch rescue_bringup bringup.launch.py
```

The shell wrapper is:

```bash
./scripts/run.sh bringup
```

---

## 2. High-level architecture

```text
Gazebo simulation
    |
    v
TurtleBot3 robot model
    |
    +-- /scan ----------------------+
    +-- /odom ----------------------+|
    +-- /camera/image_raw ----------+|
    +-- /camera/camera_info --------+|
    +-- /tf, /tf_static ------------+|
                                      |
                                      v
                              slam_toolbox
                                      |
                                      v
                                    /map
                                      |
             +------------------------+------------------------+
             |                                                 |
             v                                                 v
frontier_explorer_node                              coverage_evaluator_node
             |                                                 |
             v                                                 v
     Nav2 NavigateToPose                                  /coverage
             |                                                 |
             v                                                 v
          /cmd_vel                                  bt_supervisor_node
             |
             v
      robot movement

Camera stream
    |
    v
victim_detector_node
    |
    v
tf2 camera_frame -> map
    |
    v
/victims_map
    |
    +--> rviz_marker_node -> /visualization_marker_array
    |
    +--> result_exporter_node -> results/*.csv, results/*.json
```

---

## 3. Simulation layer

Owned by C.

Relevant paths:

```text
ros2_ws/src/rescue_robot/worlds/
ros2_ws/src/rescue_robot/models/
ros2_ws/src/rescue_robot/launch/simulation.launch.py
ros2_ws/src/rescue_robot/config/simulation_params.yaml
```

Responsibilities:

```text
- create a small test world;
- create the final disaster world;
- place obstacles, corridors and rooms;
- place victim markers;
- ensure TurtleBot3 has LiDAR and camera sensors;
- document ground-truth victim positions.
```

Development command:

```bash
./scripts/run.sh simulation
```

---

## 4. SLAM and navigation layer

Mostly owned by A during integration, then connected to B's exploration.

Relevant paths:

```text
ros2_ws/src/rescue_robot/launch/navigation.launch.py
ros2_ws/src/rescue_robot/config/slam_params.yaml
ros2_ws/src/rescue_robot/config/nav2_params.yaml
```

Responsibilities:

```text
- start slam_toolbox;
- expose /map;
- start Nav2;
- accept navigation goals;
- publish /cmd_vel to move the robot.
```

Expected frame tree:

```text
map
└── odom
    └── base_link
        ├── base_scan
        └── camera_link
            └── camera_optical_frame
```

Validation command when implemented:

```bash
ros2 run tf2_tools view_frames
```

---

## 5. Exploration layer

Owned by B.

Relevant paths:

```text
ros2_ws/src/rescue_robot/rescue_robot/exploration/
ros2_ws/src/rescue_robot/launch/exploration.launch.py
ros2_ws/src/rescue_robot/config/explorer_params.yaml
```

Current scaffold:

```text
frontier_explorer_node.py subscribes to /map and logs map statistics.
```

Final behavior:

```text
/map
  -> detect frontier cells
  -> cluster frontiers
  -> score candidate goals
  -> send selected goal to Nav2
  -> repeat until coverage >= 0.90 or no reachable frontier remains
```

Initial strategy:

```text
greedy frontier exploration
```

Possible bonus:

```text
information-gain exploration comparison
```

Development command:

```bash
./scripts/run.sh exploration
```

---

## 6. Victim detection and localization layer

Owned by the detection responsible person. If the team keeps four roles only, this work should be coordinated between B and A or assigned explicitly.

Relevant paths:

```text
ros2_ws/src/rescue_robot/rescue_robot/detection/
ros2_ws/src/rescue_robot/launch/detection.launch.py
ros2_ws/src/rescue_robot/config/detector_params.yaml
```

Current scaffold:

```text
victim_detector_node.py publishes an empty /victims_map PoseArray.
```

Final behavior:

```text
/camera/image_raw
/camera/camera_info
/tf
  -> detect colored object, ArUco or AprilTag
  -> estimate victim pose in camera frame
  -> transform pose into map frame using tf2
  -> publish /victims_map
```

Important rule:

```text
A detection is not final until it has a valid pose in the map frame.
```

Development command:

```bash
./scripts/run.sh detection
```

---

## 7. Coverage, results and visualization layer

Owned by D.

Relevant paths:

```text
ros2_ws/src/rescue_robot/rescue_robot/results/
ros2_ws/src/rescue_robot/launch/results.launch.py
ros2_ws/src/rescue_robot/config/results_params.yaml
scripts/plot_coverage.py
scripts/annotate_map.py
scripts/generate_run_summary.py
results/
```

Responsibilities:

```text
- compute /coverage from /map;
- export coverage_over_time.csv;
- export victims_detected.csv;
- export run_summary.json;
- publish RViz markers for victims;
- generate final figures for the report.
```

Output files:

```text
results/coverage_over_time.csv
results/victims_detected.csv
results/run_summary.json
results/final_map_annotated.png
```

Development command:

```bash
./scripts/run.sh results
```

---

## 8. Behavior Tree layer

Owned by A.

Relevant paths:

```text
ros2_ws/src/rescue_robot/behavior_trees/search_and_rescue_bt.xml
ros2_ws/src/rescue_robot/rescue_robot/bt/bt_supervisor_node.py
ros2_ws/src/rescue_robot/launch/bt.launch.py
ros2_ws/src/rescue_robot/config/bt_params.yaml
```

Current scaffold:

```text
bt_supervisor_node.py monitors /coverage and detects when coverage >= 0.90.
```

Final behavior should include real mission-level checks and actions:

```text
CheckSystemReady
StartExploration
MonitorCoverage
ExportVictims
SaveMap
FinalizeMission
```

Nav2 also uses Behavior Trees internally for navigation. This project-level BT is for mission supervision.

Development command:

```bash
./scripts/run.sh bt
```

---

## 9. Mock layer

Owned by A.

Relevant paths:

```text
ros2_ws/src/rescue_robot/rescue_robot/mocks/
ros2_ws/src/rescue_robot/launch/mock_system.launch.py
```

Mocks publish:

```text
/map
/victims_map
/coverage
```

Purpose:

```text
- allow B to test exploration without real SLAM;
- allow D to test visualization and exports without real detection;
- allow A to test BT supervision without the final stack.
```

Mock command:

```bash
./scripts/run.sh mock
```

Mocks are not final validation. They are only for development and interface testing.

---

## 10. Launch architecture

```text
bringup.launch.py
  ├── simulation.launch.py
  ├── navigation.launch.py
  ├── exploration.launch.py
  ├── detection.launch.py
  ├── results.launch.py
  └── bt.launch.py
```

During the scaffold phase, `bringup.launch.py` may include `mock_system.launch.py` only.

Final requirement:

```text
One command must launch the full system.
```

---

## 11. Shell scripts

The shell scripts are wrappers around ROS 2 and Python commands.

Main orchestrator:

```text
scripts/run.sh
```

Module scripts:

```text
scripts/sh/run_mock.sh
scripts/sh/run_simulation.sh
scripts/sh/run_navigation.sh
scripts/sh/run_exploration.sh
scripts/sh/run_detection.sh
scripts/sh/run_results.sh
scripts/sh/run_bt.sh
scripts/sh/build.sh
scripts/sh/doctor_env.sh
```

These scripts exist to make testing easy. They do not replace the ROS 2 launch files.

---

## 12. Development order

Recommended order:

```text
1. Validate environment and mock system.
2. Make simulation.launch.py start Gazebo/TurtleBot3.
3. Make navigation.launch.py start SLAM and Nav2.
4. Make exploration read real /map and send Nav2 goals.
5. Add camera-based victim detection.
6. Transform victim positions to map using tf2.
7. Export results and annotated map.
8. Replace mock bringup with full bringup.
9. Tune and test the final demo.
```

---

## 13. What is currently validated

The scaffold validates:

```text
- ROS 2 package layout;
- colcon build;
- mock publishers;
- /coverage monitoring by BT supervisor;
- result export files;
- shell-script workflow;
- environment cleanup for stale ROS overlays.
```

The scaffold does not yet validate:

```text
- real Gazebo project world;
- real slam_toolbox mapping;
- real Nav2 goals;
- real frontier exploration;
- real camera detection;
- real tf2 victim localization.
```

Those are the next development tasks.

## Current real-interface validation layer

The scaffold contains two complementary execution paths.

The mock path validates project interfaces without Gazebo:

```bash
./scripts/run.sh mock
```

The real simulation path validates Gazebo, TurtleBot3 and sensor topics:

```bash
./scripts/run.sh simulation
./scripts/run.sh teleop
```

At this stage, the real simulation path proves that the robot can be launched
and moved in Gazebo. SLAM, Nav2, exploration, detection and tf2 localization are
then integrated progressively on top of this real interface.

## Robot model choice

The default project robot is TurtleBot3 Waffle Pi, not Burger, because Project B needs a simulated camera for victim detection and later tf2 projection from camera frame to map. Burger remains useful only as a quick LiDAR/SLAM smoke test. See `docs/simulation_setup.md`.

The default smoke-test Gazebo world is the TurtleBot3 house world, selected through `IA712_SIM_WORLD=house`, because it better represents rooms/corridors/occlusions for Project B.

## Optional TurtleBot4 path

The stable robot baseline is TurtleBot3 Waffle Pi. The repository also includes optional TurtleBot4 scripts for professor validation and possible migration. See `docs/simulation_setup.md`. TurtleBot4 must not silently replace Waffle Pi unless the team has validated it on the target machines.
