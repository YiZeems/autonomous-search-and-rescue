# Maps

SLAM occupancy maps (`.pgm` + `.yaml`) are **generated at runtime** by driving
the robot in a world and saving with `nav2_map_server map_saver_cli`; they are
not committed (they depend on the run and the host GPU).

Generate one (after the TB4 demo is running and SLAM has mapped an area):

```bash
ros2 run nav2_map_server map_saver_cli -f ros2_ws/src/rescue_world/maps/rescue_arena \
    --ros-args -p use_sim_time:=true
```

## Platform caveat (important)

The quality of the SLAM map depends on the simulated lidar, which depends on the
GPU (see `docs/ERRORS_AND_FIXES.md` #10/#25):

- **Windows/WSL2 (hardware GPU via WSLg)** — the `gpu_lidar` returns dense, clean
  scans → SLAM builds a proper occupancy map of `rescue_arena`.
- **macOS/Parallels (ARM64, software GL)** — Ogre v1 makes the lidar return *real*
  ranges (obstacles are visible in RViz), but ~80% of rays come back invalid
  (0.0) under software rendering, which is too sparse for a clean occupancy grid.
  Use a WSL2 host (or the TurtleBot3 + Gazebo Classic path, whose CPU ray sensor
  is dense) to generate the project map.
