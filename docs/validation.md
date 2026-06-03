# Validation — Professor Checklist and Audit Notes

---

## Professor Validation Checklist

Use this checklist before presenting the repository to the professor.

### Project scope

- [ ] The repository clearly states we selected Project B.
- [ ] The README explains the goal of autonomous search and rescue.
- [ ] `docs/full_architecture.md` explains Gazebo, SLAM, Nav2, exploration, detection, tf2, results, and Behavior Tree.
- [ ] `docs/architecture_questions_and_decisions.md` explains choices, justifications, and likely professor critiques.
- [ ] `docs/project.md` lists ROS 2, Gazebo, Behavior Trees, GitHub, one-command launch, and final results.

### ROS 2 structure

- [ ] Package exists under `ros2_ws/src/rescue_robot`.
- [ ] `package.xml` exists.
- [ ] `setup.py` exists.
- [ ] Launch files exist.
- [ ] Config files exist.
- [ ] Clear Python module structure.

### Architecture

- [ ] `bringup.launch.py` exists as the final single entrypoint.
- [ ] `mock_system.launch.py` exists for parallel development.
- [ ] Module launch files exist: simulation, navigation, exploration, detection, results, BT.
- [ ] Interfaces documented in `docs/interfaces.md`.
- [ ] Behavior Tree files exist.

### Team workflow

- [ ] Each person has a role section in `docs/team_roles.md`.
- [ ] File ownership is documented in `docs/team_roles.md`.
- [ ] Git workflow documented in `docs/INSTALLATION.md`.
- [ ] GitHub protection documented in `docs/INSTALLATION.md`.
- [ ] CODEOWNERS configured with A's real GitHub username.

### Scripts

- [ ] `scripts/run.sh` is executable.
- [ ] `scripts/sh/*.sh` are executable.
- [ ] `./scripts/run.sh build` is documented.
- [ ] `./scripts/run.sh mock` is documented.
- [ ] Module commands are documented.

### Clean repository

- [ ] No `__pycache__` folder committed.
- [ ] No `.pyc` file committed.
- [ ] No `build/`, `install/`, or `log/` folder committed.
- [ ] `.gitignore` exists and is correct.

### Minimum local test

On a ROS 2 Humble machine:

```bash
./scripts/run.sh build
./scripts/run.sh mock
```

Expected topics:

```text
/map
/victims_map
/coverage
/visualization_marker_array
```

---

## Scaffold Audit Notes

This scaffold has been checked for the following non-ROS issues:

- Required project files exist.
- No generated ROS folders are committed.
- No Python cache files are committed.
- Python source files parse successfully without generating `__pycache__`.
- Shell scripts are executable and pass `bash -n`.
- `setup.py` console entry points point to existing modules.
- `scripts/run.sh` dispatches only to existing shell scripts.
- Module launch files load their module parameter files where relevant.
- `results/` generated outputs are ignored by Git except curated examples and README.

The remaining mandatory test is on the real target environment:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rescue_robot mock_system.launch.py
```

This cannot be fully validated outside Ubuntu 22.04 + ROS 2 Humble.

### uv setup (added after v5)

The project now includes:

```text
pyproject.toml
.python-version
.github/workflows/uv-checks.yml
tests/test_project_structure.py
scripts/sh/uv_sync.sh
scripts/sh/uv_test.sh
scripts/sh/uv_lint.sh
```

Purpose: consistent Python 3.10, allow `uv sync --group dev` / `uv run pytest` / `uv run ruff check .`, keep ROS 2 runtime deps separate from pip/uv, fast non-ROS check before pushing.

> **Limitation:** `uv-test` does not validate Gazebo, Nav2, SLAM Toolbox or `rclpy` runtime behavior. Those still require Ubuntu 22.04 + ROS 2 Humble and `colcon build`.

### v10 audit correction

The GitHub `uv-checks` workflow uses `ruff check .`. Import-order enforcement (`I`) was removed from the Ruff rule set to keep CI focused on real Python errors without blocking students on formatting-only details.
