# Interfaces

These interfaces are fixed early so each member can work independently with mock data.

## Map
- Topic: `/map`
- Type: `nav_msgs/msg/OccupancyGrid`
- Real publisher: `slam_toolbox`
- Mock publisher: `mock_map_publisher`
- Subscribers: `frontier_explorer_node`, `coverage_evaluator_node`

## Victims in map
- Topic: `/victims_map`
- Type: `geometry_msgs/msg/PoseArray`
- Real publisher: `victim_registry_node` (aggregates per-detection results into the full PoseArray)
- Mock publisher: `mock_victim_publisher`
- Subscribers: `rviz_marker_node`, `result_exporter_node`

## Coverage
- Topic: `/coverage`
- Type: `std_msgs/msg/Float32`
- Real publisher: `coverage_evaluator_node`
- Mock publisher: `mock_coverage_publisher`
- Subscribers: `result_exporter_node`, optional BT supervisor

## Visualization
- Topic: `/visualization_marker_array`
- Type: `visualization_msgs/msg/MarkerArray`
- Publisher: `rviz_marker_node`
- Subscriber: RViz2

## Result files
- `results/coverage_over_time.csv`
- `results/victims_detected.csv`
- `results/run_summary.json`
- `results/final_map_annotated.png`

## Mission orchestration (Behavior Tree)
The mission is driven by the C++ Behavior Tree `rescue_decision` (`bt_runner.cpp`,
tree `bt_xml/mission.xml`), launched via `rescue_decision bt_mission.launch.py`.
It runs **two sequential phases** and gates the other nodes through latched
(`TRANSIENT_LOCAL`) `/mission/*` topics:

- **Phase 1 — ExplorePhase**: raises `/mission/explore_enable=true`, stays RUNNING
  until `/coverage >= threshold`, then lowers the gate.
- **Phase 2 — InspectPhase**: raises `/mission/inspect_enable=true`, stays RUNNING
  until `inspection_node` latches `/mission/inspect_done=true`.
- On completion the BT latches `/mission_done=true` and idles (keeping it latched).

| Topic | Type | Publisher | Subscriber | Purpose |
| --- | --- | --- | --- | --- |
| `/mission/explore_enable` | `std_msgs/msg/Bool` | `rescue_decision` (BT) | `frontier_explorer_node` | Gate Phase 1 exploration on/off |
| `/mission/inspect_enable` | `std_msgs/msg/Bool` | `rescue_decision` (BT) | `inspection_node` | Gate Phase 2 inspection on/off |
| `/mission/inspect_done` | `std_msgs/msg/Bool` | `inspection_node` | `rescue_decision` (BT) | Signal Phase 2 finished |
| `/mission_done` | `std_msgs/msg/Bool` | `rescue_decision` (BT) | (latched end-of-mission flag) | Mission complete |
| `/coverage` | `std_msgs/msg/Float32` | `coverage_evaluator_node` | `rescue_decision` (BT) | ExplorePhase coverage threshold |
| `/victims_map` | `geometry_msgs/msg/PoseArray` | `victim_registry_node` | `rescue_decision` (BT) | VictimsFound condition (pose count) |

(`/mission/*` and `/mission_done` use latched `TRANSIENT_LOCAL` QoS; `/coverage`
and `/victims_map` use default QoS depth 10.)

## Algorithm visualization (RViz)
| Topic | Type | Publisher | Subscriber | Purpose |
| --- | --- | --- | --- | --- |
| `/exploration/frontiers` | `visualization_msgs/msg/MarkerArray` | `frontier_explorer_node` | RViz2 | Show frontiers + selected goal |
| `/inspection/poses` | `visualization_msgs/msg/MarkerArray` | `inspection_node` | RViz2 | Show map-derived inspection poses |

## Navigation (Nav2)
- Action: `/navigate_to_pose`
- Type: `nav2_msgs/action/NavigateToPose`
- Server: Nav2 `bt_navigator`
- Client: `waypoint_follower_node` (sends the predefined path goal-by-goal)
- Client (Phase 2): `inspection_node` — gated by `/mission/inspect_enable`, reads
  `/map` to derive one inspection pose per room (`inspection_planner.poses_from_grid`,
  no victim coords), drives each via `NavigateToPose`, publishes `/inspection/poses`
  (`visualization_msgs/msg/MarkerArray`) for RViz and latches `/mission/inspect_done`
  (`std_msgs/msg/Bool`) when finished.
- Velocity output: `/cmd_vel` (`geometry_msgs/msg/Twist`) from `controller_server`

## TurtleBot4 + Ignition integration bridges
These nodes exist only to bridge the namespaced TB4 simulation onto the global
topics the rest of the stack expects (see `docs/ERRORS_AND_FIXES.md` #5, #7).

- **LiDAR**: Ignition `/world/<world>/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan`
  → `/scan` (`sensor_msgs/msg/LaserScan`), via `ros_gz_bridge`.
- **Camera**: Ignition `.../oakd_rgb_camera_frame/sensor/rgbd_camera/image`
  → `/camera/image_raw` (`sensor_msgs/msg/Image`).
- **TF relay** (`tf_relay_node`): `/turtlebot4/tf` → `/tf` (`tf2_msgs/msg/TFMessage`),
  QoS RELIABLE + TRANSIENT_LOCAL on the subscriber.
- **cmd_vel relay** (`cmd_vel_relay_node`): `/cmd_vel` →
  `/turtlebot4/diffdrive_controller/cmd_vel_unstamped` (`geometry_msgs/msg/Twist`).
- **Static TF** (identity): `turtlebot4/base_link` ≡ `base_link`,
  `rplidar_link` ≡ `turtlebot4/rplidar_link/rplidar`.

## Shell scripts
- `./scripts/run.sh build` — build workspace.
- `./scripts/run.sh mock` — launch mock integration system.
- `./scripts/run.sh exploration` — launch exploration module.
- `./scripts/run.sh detection` — launch detection module.
- `./scripts/run.sh results` — launch results module.
- `./scripts/run.sh bt` — launch BT supervisor module.
- `./scripts/run.sh bringup` — launch current global entrypoint.

The per-module scripts above are for independent/mock development. The **real
end-to-end mission flow** is the 2-phase BT and runs through the TurtleBot4
bringup, not these per-module scripts:

- `ros2 launch rescue_bringup bringup_tb4.launch.py` — Ignition + SLAM + Nav2 + RViz2 bringup.
- `./scripts/run.sh demo-tb4` (`scripts/sh/run_demo_tb4.sh`) — validated full demo
  orchestrating the 2-phase BT (explore then inspect) end-to-end.
