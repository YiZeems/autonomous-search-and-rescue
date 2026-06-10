# Shell Scripts and Launch Commands

The professor asked for simple executable scripts. We provide one orchestrator:

```bash
./scripts/run.sh <command>
```

## Commands

```bash
./scripts/run.sh install-apt
./scripts/run.sh doctor-env
./scripts/run.sh build
./scripts/run.sh clean
./scripts/run.sh mock
./scripts/run.sh bringup
./scripts/run.sh simulation
./scripts/run.sh navigation
./scripts/run.sh exploration
./scripts/run.sh detection
./scripts/run.sh results
./scripts/run.sh bt
```

## Rule

Shell scripts are only wrappers. The canonical ROS 2 entrypoints remain the launch files in:

```text
ros2_ws/src/rescue_robot/launch/
```

This prevents the scripts from hiding the architecture. Each script simply sources ROS 2 / the workspace and calls the matching launch file.

## uv commands

The root wrapper also supports Python development commands:

```bash
./scripts/run.sh uv-sync
./scripts/run.sh uv-test
./scripts/run.sh uv-lint
```

They call:

```text
scripts/sh/uv_sync.sh
scripts/sh/uv_test.sh
scripts/sh/uv_lint.sh
```

These commands are intentionally separate from the ROS 2 launch commands. They help check code structure and offline tools before integration.

## Real simulation commands

```bash
./scripts/run.sh simulation
```

Launches the real TurtleBot3 Gazebo world through `simulation.launch.py`.

```bash
./scripts/run.sh teleop
```

Runs TurtleBot3 keyboard teleoperation. Use it in a second terminal while the
simulation is running.


## TurtleBot3 check

Before launching the real simulation, run:

```bash
./scripts/run.sh check-tb3
```

This verifies whether `turtlebot3_gazebo` is available directly from apt/ROS or through a local source overlay. See `docs/simulation_setup.md`.

## Camera topic check

```bash
./scripts/run.sh camera-check
```

Checks whether camera/image/depth topics are visible while the Waffle Pi simulation is running. This is important for Project B victim detection.

`./scripts/run.sh simulation` launches the default Project B house world. Use `simulation-base` for the basic TurtleBot3 demo world, and `simulation-empty` for an empty world.

## TurtleBot4 optional commands

```bash
./scripts/run.sh check-tb4
./scripts/run.sh simulation-tb4
./scripts/run.sh teleop-tb4
```

These commands are optional. They are for machines where TurtleBot4 simulator packages are installed or configured through `config/local_env.sh`.


## Safe graphics simulation command

For virtual machines or fragile OpenGL setups, especially macOS + Parallels + Ubuntu ARM64, use:

```bash
./scripts/run.sh simulation-house-safe
```

This launches the TurtleBot3 Waffle Pi in the house world with safer Gazebo GUI settings. It is equivalent to setting `IA712_GAZEBO_SAFE_GRAPHICS=1` before launching simulation.
