# Installation Guide

This guide explains how to install the development environment for the IA712 Search and Rescue project.

## Target environment

Use this reference setup whenever possible:

```text
Ubuntu 22.04 LTS
ROS 2 Humble
Python 3.10
Gazebo Classic 11
TurtleBot3
Nav2
slam_toolbox
uv for Python development tests only
```

ROS 2 Humble on Ubuntu 22.04 is the reference because it matches the course stack and avoids unnecessary compatibility problems.

## Repository paths

All commands assume the repository root is:

```text
ia712-search-and-rescue/
```

The ROS 2 workspace is:

```text
ia712-search-and-rescue/ros2_ws/
```

The ROS 2 package is:

```text
ia712-search-and-rescue/ros2_ws/src/rescue_robot/
```

## 1. Install ROS 2 Humble

Install ROS 2 Humble Desktop on Ubuntu 22.04 using the official ROS 2 documentation.

After installation, this file must exist:

```text
/opt/ros/humble/setup.bash
```

Test:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

## 2. Install project apt dependencies

After ROS 2 Humble is installed, run from the repository root:

```bash
./scripts/run.sh install-apt
./scripts/run.sh doctor-env
```

This installs the common packages used by the scaffold:

```text
python3-colcon-common-extensions
python3-rosdep
python3-vcstool
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-slam-toolbox
ros-humble-turtlebot3
ros-humble-tf2-tools
ros-humble-rviz2
ros-humble-cv-bridge
ros-humble-image-transport
```

It also tries to install TurtleBot3 Gazebo packages when they are available from apt:

```text
ros-humble-turtlebot3-gazebo
ros-humble-turtlebot3-simulations
ros-humble-turtlebot3-bringup
ros-humble-turtlebot3-description
```

On AMD64/WSL2 these packages are usually available. On ARM64 they may be missing from apt and must then come from a source overlay. See `docs/simulation_setup.md`.

If this script fails because ROS apt sources are missing, finish the ROS 2 Humble installation first, then retry.

## 3. Configure TurtleBot3 model

This project requires **Waffle Pi** (not Burger) because the victim detection pipeline needs a simulated camera.

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

To make it persistent, add to your shell config (`~/.bashrc` or `~/.zshrc`):

```bash
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
# or for zsh:
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.zshrc
```

## 4. Build the ROS 2 workspace

From repository root:

```bash
source /opt/ros/humble/setup.bash
./scripts/run.sh build
```

Equivalent manual command:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Launch the mock system

The mock system is the first test. It does not need Gazebo/Nav2/SLAM to be fully configured.

```bash
./scripts/run.sh mock
```

Expected topics:

```bash
ros2 topic list
```

You should see at least:

```text
/map
/victims_map
/coverage
/visualization_marker_array
```

## 6. Install uv

uv is used only for Python development tests, linting and offline scripts.

Install uv:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Restart the terminal or source your shell profile if needed.

Then run:

```bash
./scripts/run.sh uv-sync
./scripts/run.sh uv-test
./scripts/run.sh uv-lint
```

Do not install ROS 2 packages such as `rclpy` with uv. ROS dependencies come from ROS 2 Humble and apt packages.

## 7. Useful commands

From repository root:

```bash
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
./scripts/run.sh uv-sync
./scripts/run.sh uv-test
./scripts/run.sh uv-lint
```

## 8. Troubleshooting

### ROS 2 Humble not found

Error:

```text
ROS 2 Humble not found at /opt/ros/humble/setup.bash
```

Fix:

```bash
source /opt/ros/humble/setup.bash
```

If the file does not exist, ROS 2 Humble is not installed correctly.

### colcon command not found

Fix:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

### TurtleBot3 Gazebo packages missing

First check:

```bash
./scripts/run.sh check-tb3
```

On AMD64/WSL2, try apt:

```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-simulations
```

On ARM64, these apt packages may not exist. Use a source overlay instead. See:

```text
docs/simulation_setup.md
```

### uv cannot find Python 3.10

On Ubuntu 22.04, Python 3.10 is normally installed. Check:

```bash
python3 --version
```

If needed, let uv install managed Python 3.10 automatically, or install Python 3.10 manually.

### Do not commit generated files

Never commit:

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
.venv/
__pycache__/
*.pyc
```

## Troubleshooting: stale ROS workspace paths

If `source /opt/ros/humble/setup.bash` or `./scripts/run.sh build` prints an error like:

```text
setup.bash: no such file or directory: .../old_workspace.../setup.sh
AMENT_TRACE_SETUP_FILES: unbound variable
```

then your terminal probably contains paths from an old ROS workspace. This often happens after sourcing a previous TP workspace in `.bashrc`.

Use the project diagnostic command:

```bash
./scripts/run.sh doctor-env
```

For a clean temporary session, run:

```bash
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH AMENT_TRACE_SETUP_FILES
source /opt/ros/humble/setup.bash
./scripts/run.sh build
./scripts/run.sh mock
```

The project scripts now clean stale ROS overlay variables by default before sourcing ROS 2 Humble. Advanced users can preserve their current overlay stack with:

```bash
IA712_KEEP_ROS_OVERLAY=1 ./scripts/run.sh build
```

Recommended `.bashrc` policy:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

Do not automatically source old project workspaces in `.bashrc`; source each workspace only when working on it.

## TurtleBot3 Gazebo on AMD64 vs ARM64

See `docs/simulation_setup.md` before running the real simulation. AMD64/WSL2 users usually install TurtleBot3 Gazebo through apt. ARM64 users may need a source overlay.

### Project robot default

For this rescue project, the default TurtleBot3 model is `waffle_pi`, not `burger`, because the final perception pipeline needs a simulated camera for victim detection. Burger can still be used for quick LiDAR-only smoke tests.

See `docs/simulation_setup.md` for details.

## Gazebo freezes on `Preparing your world`

On Parallels ARM64 or WSLg, Gazebo Classic may freeze while loading the house world. Use safe graphics mode:

```bash
./scripts/run.sh simulation-house-safe
```

Or manually:

```bash
IA712_GAZEBO_SAFE_GRAPHICS=1 ./scripts/run.sh simulation-house
```

See the [Gazebo safe graphics section](simulation_setup.md#gazebo-safe-graphics-mode) in `docs/simulation_setup.md`.

---

## Python dev environment with uv

The project uses two Python contexts:

1. **ROS 2 runtime Python** — provided by Ubuntu 22.04 + ROS 2 Humble (`rclpy`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, etc.).
2. **Development venv with uv** — used for offline scripts, formatting, lightweight tests, and result-generation tools.

> Do not install ROS 2 packages such as `rclpy` with `pip` or `uv`. They must come from the ROS 2 Humble installation.

### Required Python version

Python 3.10 — matches Ubuntu 22.04 and ROS 2 Humble.

```bash
cat .python-version  # → 3.10
```

### Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Restart the shell or source your shell config afterwards.

### Create/sync the venv

```bash
./scripts/run.sh uv-sync
# or: uv sync --group dev
```

### Run tests

```bash
./scripts/run.sh uv-test
# or: uv run pytest
```

These tests are intentionally lightweight (structure, syntax, file hygiene). They do not replace `colcon build`.

### Run lint

```bash
./scripts/run.sh uv-lint
# or: uv run ruff check .
```

### Run offline result scripts

```bash
uv run python scripts/generate_run_summary.py
uv run python scripts/plot_coverage.py
uv run python scripts/annotate_map.py
```

### If `uv sync` says Python 3.10 is missing

```bash
uv python install 3.10
./scripts/run.sh uv-sync
```

---

## Git workflow

### Branches

```text
main           stable demo only — protected, PR required
dev            integration branch — protected, PR required
dev/yimou      Yimou's working branch
dev/julien     Julien's working branch
dev/hugo       Hugo's working branch
dev/paul       Paul's working branch
```

### Clone and set up your branch

```bash
git clone https://github.com/YiZeems/autonomous-search-and-rescue.git
cd autonomous-search-and-rescue
git checkout dev/<your-name>
```

### Daily workflow

```bash
git pull origin dev/<your-name>
# ... make changes in your module folder ...
git add .
git commit -m "feat: describe what you did"
git push origin dev/<your-name>
```

Then open a Pull Request from `dev/<your-name>` → `dev` on GitHub.

### Integration workflow (Yimou / A role)

```bash
git checkout dev
git pull origin dev
git merge dev/julien     # after PR is approved
./scripts/run.sh build
./scripts/run.sh mock
git push origin dev
```

Repeat for each member's branch.

### When to merge to dev

Merge when a module has a stable small update: skeleton starts, launch runs, mock works. Merge `dev` → `main` only at stable milestones.

### Files only A (Yimou) should touch

`README.md`, `CONTRIBUTING.md`, `.github/`, `package.xml`, `setup.py`, `launch/bringup.launch.py`, `launch/mock_system.launch.py`, `docs/interfaces.md`.

---

## GitHub Protection Rules

### Protect `main`

Settings: require PR before merging, ≥1 approval, Code Owners review required, status checks pass, block force pushes and deletions.

Nobody pushes directly to `main`, not even A.

### Protect `dev`

Settings: require PR, status checks pass, Code Owners review, block force pushes and deletions.

A manages merges into `dev`.

### Personal branches

`a-integration`, `b-exploration`, `c-simulation`, `d-results` — push freely without PR overhead.

### CODEOWNERS

File: `.github/CODEOWNERS`. Replace `@YiZeems` with A's real GitHub username before using it.

### CI policy

The first CI is intentionally light:

- checks that `build/`, `install/`, `log/` are not committed;
- checks that required files exist;
- checks Python syntax.

We avoid Gazebo/Nav2 CI because it is heavy and fragile for a student project.
