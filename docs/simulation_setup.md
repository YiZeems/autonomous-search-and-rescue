# Simulation Setup — Robot, Worlds, and Gazebo Configuration

---

## Robot model decision: Burger vs Waffle Pi vs TurtleBot4

### Final recommendation

Use **TurtleBot3 Waffle Pi** as the main project robot.

Use **TurtleBot3 Burger** only as a low-level smoke-test robot for quick LiDAR/teleop/SLAM debugging.

Keep **TurtleBot4** as a possible professor-recommended extension — see the [TurtleBot4 section](#turtlebot4-experimental) below.

### Why not Burger for the final project?

Burger is light and reliable for first tests, but Project B requires victim detection using a simulated camera and `tf2` projection from camera to `map`. Burger has no camera model.

### Why Waffle Pi is the best default

- Stays in the TurtleBot3 ecosystem already tested in this repository
- Adds a camera-oriented robot model for victim detection
- Keeps LiDAR, odometry, and TF for SLAM/Nav2
- Avoids a full migration to a different robot stack
- Compatible across ARM64 and AMD64

---

## Gazebo world selection

### Why not use only the default TurtleBot3 world?

The basic TurtleBot3 demo world is too clean for Project B. For the project smoke test, the default simulation world is:

```bash
./scripts/run.sh simulation
```

which launches **TurtleBot3 Waffle Pi + TurtleBot3 house world** — a built-in world with rooms, corridors, walls and occlusions, closer to the rescue scenario.

### Available commands

```bash
./scripts/run.sh simulation          # default = house world
./scripts/run.sh simulation-house    # same as above
./scripts/run.sh simulation-base     # basic TurtleBot3 world
./scripts/run.sh simulation-empty    # empty world (spawn/model debugging)
./scripts/run.sh simulation-house-safe  # house world + safe graphics (see below)
```

| Command | Purpose |
|---|---|
| `simulation-house` | Preferred smoke test. Rooms/corridors/occlusions. |
| `simulation-base` | Basic TurtleBot3 world. Quick LiDAR/teleop tests. |
| `simulation-empty` | Empty world. Isolate spawn/model issues. |

### Environment variable overrides

```bash
export TURTLEBOT3_MODEL=waffle_pi
export IA712_SIM_WORLD=house
./scripts/run.sh simulation
```

World aliases: `house`, `base`/`world`, `empty`. You can also pass an upstream TurtleBot3 launch filename directly via `IA712_SIM_WORLD=turtlebot3_house`.

Direct `ros2 launch` equivalent:

```bash
ros2 launch rescue_robot simulation.launch.py model:=waffle_pi world:=house
```

### What the house-world smoke test validates

```text
✅ Gazebo Classic 11
✅ TurtleBot3 Waffle Pi model
✅ LiDAR /scan
✅ Odometry /odom
✅ TF /tf and /tf_static
✅ /cmd_vel teleoperation
✅ Realistic walls/corridors
```

Not yet validated at this stage: SLAM map quality, Nav2, exploration, victim detection, tf2 localization.

---

## TurtleBot3 simulation setup (AMD64 and ARM64)

### Check your architecture

```bash
uname -m
# x86_64 = AMD64/WSL2
# aarch64 = ARM64 Linux
```

### AMD64 / WSL2 setup

TurtleBot3 Gazebo packages are usually available through apt:

```bash
./scripts/run.sh install-apt
./scripts/run.sh build
./scripts/run.sh check-tb3
./scripts/run.sh simulation
```

### ARM64 setup with source overlay

On ARM64, `ros-humble-turtlebot3-gazebo` may not be available through apt. The scripts auto-detect a known overlay path:

```text
../TP/Phase_1/test-easy/ros2_ws/install
```

Manual equivalent:

```bash
source /opt/ros/humble/setup.zsh
source /media/psf/.../TP/Phase_1/test-easy/ros2_ws/install/setup.zsh
ros2 pkg list | grep turtlebot3_gazebo
```

### Explicit overlay configuration (any machine)

```bash
cp config/local_env.example.sh config/local_env.sh
```

Edit `config/local_env.sh`:

```bash
export IA712_TURTLEBOT3_OVERLAY_SETUP="/absolute/path/to/ros2_ws/install/setup.bash"
# or
export IA712_TURTLEBOT3_OVERLAY_INSTALL="/absolute/path/to/ros2_ws/install"
```

`config/local_env.sh` is ignored by Git (machine-specific).

### Teleoperation test

```bash
# Terminal 1
./scripts/run.sh simulation
# Terminal 2
./scripts/run.sh teleop
```

---

## Gazebo safe graphics mode

### When to use it

On some machines (macOS + Parallels ARM64, WSL2/WSLg, Wayland), Gazebo Classic can freeze the GUI while loading the house world:

```text
"gazebo" is not responding — Preparing your world ...
```

This does **not** mean the robot or ROS 2 are broken — it is a rendering issue.

### Recommended command for Parallels ARM64

```bash
./scripts/run.sh simulation-house-safe
```

Equivalent manual invocation:

```bash
IA712_GAZEBO_SAFE_GRAPHICS=1 TURTLEBOT3_MODEL=waffle_pi IA712_SIM_WORLD=house ./scripts/run.sh simulation
```

### What safe graphics mode changes

| Variable | Effect |
|---|---|
| `QT_QPA_PLATFORM=xcb` | Prefer X11/XCB over Wayland for Qt |
| `GDK_BACKEND=x11` | Prefer X11 for GTK GUI |
| `LIBGL_ALWAYS_SOFTWARE=1` | Software OpenGL (slower but stable in VMs) |
| `MESA_GL_VERSION_OVERRIDE=3.3` | Expose compatible OpenGL version |
| `MESA_GLSL_VERSION_OVERRIDE=330` | Expose compatible GLSL version |

### Recommended testing order

```bash
./scripts/run.sh doctor-env
./scripts/run.sh build
./scripts/run.sh check-tb3
./scripts/run.sh simulation-base          # quick baseline
./scripts/run.sh simulation-house-safe    # house world smoke test
```

In another terminal:

```bash
./scripts/run.sh teleop
./scripts/run.sh camera-check
```

### Usage guidelines by machine

| Machine | Recommended command |
|---|---|
| Native AMD64 Linux | `simulation-house` first, then safe if frozen |
| AMD64 WSL2 | `simulation-house` first, then safe if frozen |
| ARM64 Parallels | `simulation-house-safe` (default) |

Do not remove safe graphics support — it is required for reliable cross-machine testing.

---

## TurtleBot4 experimental {#turtlebot4-experimental}

TurtleBot4 is kept as an optional experimental path. **The stable baseline remains TurtleBot3 Waffle Pi.**

### Why not switch immediately?

TurtleBot4 usually uses a different simulation stack (Ignition/newer Gazebo). Binary packages may be unavailable on ARM64. Switching too early wastes integration time.

### Commands

```bash
./scripts/run.sh check-tb4
./scripts/run.sh simulation-tb4
./scripts/run.sh teleop-tb4
```

### AMD64 / WSL2 setup (Windows 11 + WSLg)

Install the full TurtleBot4 + Ignition Fortress stack (idempotent — `install-apt`
already lists all of these):

```bash
./scripts/run.sh install-apt          # installs turtlebot4-simulator, ros-gz, ignition-gazebo6, irobot-create…
# or the minimal TB4 subset:
sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \
    ros-humble-ros-gz-bridge ros-humble-ros-gz-sim libignition-gazebo6 ros-humble-xacro
./scripts/run.sh build
./scripts/run.sh check-tb4            # should list turtlebot4_* + irobot_create_* + Ignition 6.x + worlds
./scripts/run.sh demo-tb4            # full demo: Ignition + SLAM + Nav2 + RViz2
```

#### WSLg OpenGL: Ogre2 vs software GL

`demo-tb4`/`platform_win.sh` default to **Ogre2** on the WSLg hardware GPU. That
works only if your WSLg GL driver is complete. Some setups expose OpenGL only
through the Mesa **D3D12** driver (`ls /dev/dri` empty, only `/dev/dxg`), which
**cannot** run Ogre2's GL3Plus path — `ign gazebo` then aborts in the Sensors
RenderThread (`Ogre2Material::SetTextureMapImpl`, see
[ERRORS_AND_FIXES.md](ERRORS_AND_FIXES.md) #10).

If Gazebo crashes immediately on `demo-tb4`, switch to the Ogre v1 + llvmpipe
software path:

```bash
IA712_WSL_SOFTWARE_GL=1 ./scripts/run.sh demo-tb4
```

To make it permanent on this machine, put the same settings in
`config/local_env.sh` (git-ignored), which is sourced after `platform_win.sh`:

```bash
export IA712_RENDER_ENGINE=ogre        # Ogre v1, not ogre2
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export IA712_USE_UDP_DDS=1             # controller_manager reachability (#21)
export IA712_TB4_GUI=1                 # headless drops the controllers on WSLg
```

Notes verified on a WSLg/D3D12 box:
- The Gazebo **and** RViz2 windows open fine under Ogre v1.
- `demo-tb4` auto-detects the Ignition lidar/camera topics with `ign topic -l`,
  so it works whether the model path is `model/<ns>/link/…` or the nested
  `model/<ns>/<ns>/link/…` (the latter appears when the robot spawns with the dock).
- Software GL is CPU-bound: SLAM mapping and Nav2 are slow, and the
  `ign_ros2_control` controller_manager can be intermittently unreachable over
  DDS (#21) — keep the GUI on and UDP DDS enabled. A real GPU host gives a
  cleaner map.

### TurtleBot4 — legacy minimal path

```bash
./scripts/run.sh check-tb4
./scripts/run.sh simulation-tb4       # upstream turtlebot4_ignition.launch.py only
./scripts/run.sh teleop-tb4
```

### ARM64 setup

Add to `config/local_env.sh`:

```bash
export IA712_TB4_OVERLAY_SETUP="/absolute/path/to/turtlebot4_ws/install/setup.bash"
```

### Launch package overrides

Default launcher targets `turtlebot4_ignition_bringup` / `turtlebot4_ignition.launch.py` / model `standard` / world `depot`. Override in `config/local_env.sh`:

```bash
export IA712_TB4_BRINGUP_PACKAGE="turtlebot4_ignition_bringup"
export IA712_TB4_LAUNCH_FILE="turtlebot4_ignition.launch.py"
export IA712_TB4_MODEL="standard"
export IA712_TB4_WORLD="depot"
```

### Recommendation

Use TurtleBot3 Waffle Pi as the stable baseline until TurtleBot4 works on at least one AMD64 machine and one team member can reproduce the setup. If TurtleBot4 takes too long, keep it as a documented professor-facing experiment.
