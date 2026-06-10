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
- Real publisher: `victim_detector_node` / `victim_localizer_node`
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

## Behavior Tree supervisor
Topic: `/coverage`  
Type: `std_msgs/msg/Float32`  
Subscriber: `bt_supervisor_node`  
Purpose: decide when the exploration success condition has been reached.

## Navigation (Nav2)
- Action: `/navigate_to_pose`
- Type: `nav2_msgs/action/NavigateToPose`
- Server: Nav2 `bt_navigator`
- Client: `waypoint_follower_node` (sends the predefined path goal-by-goal)
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
