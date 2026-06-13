# Running the TB4 demo on Windows + WSL 2 (GPU acceleration & stability)

The full `demo-tb4` (Ignition Gazebo + SLAM + Nav2 + perception) is heavy. On
Windows/WSL 2 it runs fine **and on the GPU**, but two host-side settings make the
difference between "reboots after a few minutes" and "stable 40-minute runs".
This page collects everything specific to that environment.

> TL;DR — do these once: (1) write `.wslconfig` and `wsl --shutdown`; (2) run the
> demo **headless** without forcing software GL; (3) build/run in a **conda-free**
> shell. Then long autonomous runs are stable on the RTX-class GPU.

---

## 1. GPU acceleration (do NOT force software GL)

WSLg exposes the discrete GPU to Linux through the **Mesa D3D12 driver**
(`/dev/dxg`). Ignition's **Ogre2** renders on it once the GL version is overridden
to 4.5 — this is already done by [`config/platform_win.sh`](../config/platform_win.sh)
(its **default** branch: `unset LIBGL_ALWAYS_SOFTWARE`, `MESA_GL_VERSION_OVERRIDE=4.5`,
NVIDIA adapter pinned).

Measured here (RTX 4070 Laptop): camera renders at **~7 Hz on the GPU vs ~0.3 Hz**
in software (`llvmpipe`) — a **~23× speed-up**, and the CPU stays free (load ~4
instead of pegging every core). Software rendering is what made long runs overheat
and reboot the machine.

**Rule:** never put `export LIBGL_ALWAYS_SOFTWARE=1` in your run command — that
disables the GPU and forces the slow CPU path. Software GL is a *fallback only*,
selected with `IA712_WSL_SOFTWARE_GL=1` for machines with no usable GPU.

What WSL/NVIDIA does **not** give you (so don't chase it): there is **no native
NVIDIA OpenGL/Vulkan driver on WSL** (only CUDA + D3D12 + video). Hardware Vulkan
(for a Zink→GL 4.6 path) would need Mesa 24's `dzn` driver, absent on Ubuntu 22.04.
CUDA is compute-only and does **not** accelerate the Ogre renderer.

---

## 2. Stability: `.wslconfig` (prevents reboots on long runs)

Even on the GPU, a sustained ~15-minute run could reboot the host (thermal / GPU
passthrough stress) because by default WSL is uncapped. Cap it so Windows keeps
headroom.

Create **`C:\Users\<you>\.wslconfig`** (Windows side):

```ini
# Stabilité des longs runs Gazebo/GPU.
# Host = 31.5 GB / 22 logical CPU here — leave margin for Windows + swap cushion.
[wsl2]
memory=20GB
swap=8GB
processors=16
gpuSupport=true
```

Tune to your machine: keep `processors` a few below your logical-core count (less
sustained heat), `memory` well under the host total (leave Windows several GB),
keep `gpuSupport=true`.

**Apply it** (from a Windows PowerShell / CMD — this restarts WSL, so close work
first):

```powershell
wsl --shutdown
```

WSL restarts with the new limits on the next command. Verify inside WSL:

```bash
nproc                 # should match `processors`
free -g | grep Mem    # total should match `memory`
```

With this applied, autonomous runs have stayed stable for **40+ minutes** with no
reboot.

---

## 3. Run the demo (headless, GPU, conda-free)

ROS Humble needs the **system** Python 3.10, not conda's 3.13 — conda breaks the
build (catkin_pkg / ncurses link) and the runtime (`rclpy` / numpy). Run inside a
clean environment (`env -i …`), or `conda config --set auto_activate_base false`
once. See [`ERRORS_AND_FIXES.md`](ERRORS_AND_FIXES.md) #31.

```bash
# Headless autonomous mission, GPU (no software-GL override):
env -i HOME="$HOME" PATH=/usr/bin:/bin TERM=xterm DISPLAY=:0 bash -lc '
  source /opt/ros/humble/setup.bash
  cd ros2_ws ; [ -d install ] || colcon build --symlink-install ; cd ..
  IA712_TB4_WORLD=rescue_arena IA712_TB4_GUI=0 IA712_RVIZ=0 IA712_EXPLORE=1 \
  ./scripts/run.sh demo-tb4'
```

Useful env vars:

| Var | Effect |
|---|---|
| `IA712_TB4_GUI=0`, `IA712_RVIZ=0` | fully headless (always use these for long runs) |
| `IA712_EXPLORE=1` | autonomous frontier exploration (else waypoint mode) |
| `IA712_SPAWN_X/Y/YAW` | spawn the robot at a chosen pose (e.g. facing a victim, to validate detection without a long mission) |
| `IA712_WSL_SOFTWARE_GL=1` | force the software-GL fallback (only if the GPU path fails) |

Outputs land in `results/` (`run_summary.json`, `victims.json`,
`coverage_over_time.csv`) and are regenerated on every run.

---

## 4. Required WSL package

`ros-humble-rmw-cyclonedds-cpp` is **mandatory** on WSL (Fast-RTPS discovery is
flaky here). Without it every node dies at startup. See `ERRORS_AND_FIXES.md` #32
and the apt list in the [README](../README.md).
