# Real simulation smoke test

This document explains the first non-mock test of the project.

The mock system is useful for parallel development, but it does not prove that
Gazebo, TurtleBot3 and the real ROS topics work. The real simulation smoke test
launches the upstream TurtleBot3 Gazebo world through this project package.

## What this test validates

It validates that the machine can run:

- Gazebo Classic 11;
- TurtleBot3 Gazebo;
- the simulated TurtleBot3 robot model;
- the simulated LiDAR interface;
- the ROS 2 launch system from this package.

It does not validate yet:

- SLAM;
- Nav2 autonomous navigation;
- frontier exploration;
- camera victim detection;
- tf2 camera-to-map localization.

Those will be added progressively after this smoke test is stable.

## Terminal 1: launch the real simulation

From the repository root:

```bash
./scripts/run.sh build
./scripts/run.sh simulation
```

This runs:

```bash
ros2 launch ia712_search_rescue simulation.launch.py
```

The launch file includes:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Expected result:

- Gazebo opens;
- the TurtleBot3 appears in the world;
- the blue LiDAR rays are visible;
- the simulation keeps running until `Ctrl+C`.

## Terminal 2: drive the robot manually

Open a second terminal from the repository root:

```bash
./scripts/run.sh teleop
```

Expected result:

- keyboard teleoperation starts;
- the robot moves in Gazebo;
- this confirms the simulated robot command interface works.

## Terminal 3: inspect ROS topics

Open a third terminal from the repository root:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic list
```

Useful topics to check:

```text
/cmd_vel
/scan
/odom
/tf
/joint_states
```

To check the LiDAR rate:

```bash
ros2 topic hz /scan
```

To inspect odometry:

```bash
ros2 topic echo /odom --once
```

## Difference between `mock`, `simulation` and `bringup`

```bash
./scripts/run.sh mock
```

starts fake publishers and project scaffolds. It is for parallel development.

```bash
./scripts/run.sh simulation
```

starts the real TurtleBot3 Gazebo simulation.

```bash
./scripts/run.sh bringup
```

currently starts the real simulation too. Later, `bringup.launch.py` will become
the full one-click entrypoint for simulation, navigation, exploration, detection,
results and Behavior Tree supervision.

## If Gazebo does not open

Check:

```bash
./scripts/run.sh doctor-env
ros2 pkg list | grep turtlebot3_gazebo
```

Also verify that the TurtleBot3 model is set:

```bash
echo $TURTLEBOT3_MODEL
```

The scripts default to:

```text
waffle_pi
```


## TurtleBot3 Gazebo on AMD64 vs ARM64

See `docs/simulation_setup.md` before running the real simulation. AMD64/WSL2 users usually install TurtleBot3 Gazebo through apt. ARM64 users may need a source overlay.

The default real simulation now uses the TurtleBot3 house world, which is closer to the rescue scenario than the basic world. Use `./scripts/run.sh simulation-base` for the original basic TurtleBot3 world.


## Direct `ros2 launch` note

The project launch file sets `TURTLEBOT3_MODEL` from the `model` launch argument. This means both commands are valid:

```bash
./scripts/run.sh simulation
ros2 launch ia712_search_rescue simulation.launch.py model:=waffle_pi world:=house
```

For normal team use, prefer the script because it also handles TurtleBot3 overlays on ARM64.


## Safe graphics variant

If Gazebo GUI freezes on the house world, run:

```bash
./scripts/run.sh simulation-house-safe
```

This is the recommended mode for Parallels ARM64 when testing the house world.
