# ROS 2 Package Structure Explained

> **Current layout (3 packages).** The project is split into three ROS 2
> packages under `ros2_ws/src/`:
>
> - **`rescue_robot`** (`ament_python`) — all nodes, configs, launch files, RViz config.
> - **`rescue_bringup`** (`ament_cmake`) — integration entry points (`bringup.launch.py`, `demo.launch.py`).
> - **`rescue_world`** (`ament_cmake`) — Gazebo worlds/models.
>
> The single launch file that lives in `rescue_bringup` (not `rescue_robot`) is
> `bringup.launch.py`: run it with `ros2 launch rescue_bringup bringup.launch.py`.

This file explains the repository tree and, especially, why the path below contains the same name twice:

```text
ros2_ws/src/rescue_robot/rescue_robot/
```

This is normal for a ROS 2 Python package.

---

## 1. Two folders, two different roles

### ROS 2 package folder

```text
ros2_ws/src/rescue_robot/
```

This is the **ROS 2 package root**. It is the folder that `colcon` builds.

It contains files used by ROS 2, `ament_python`, launch, Gazebo and configuration:

```text
ros2_ws/src/rescue_robot/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── launch/
├── config/
├── behavior_trees/
├── worlds/
├── models/
├── maps/
├── rviz/
├── test_data/
└── rescue_robot/
```

This level answers:

```text
How is the ROS 2 package built, installed and launched?
```

### Python module folder

```text
ros2_ws/src/rescue_robot/rescue_robot/
```

This is the **Python importable module**. It contains the Python code for the ROS 2 nodes.

```text
ros2_ws/src/rescue_robot/rescue_robot/
├── __init__.py
├── bt/
├── detection/
├── exploration/
├── mocks/
└── results/
```

This level answers:

```text
Where is the Python code imported by the ROS 2 executables?
```

---

## 2. Why the repeated name is required

The repeated name is the standard layout for an `ament_python` package:

```text
package_name/
└── package_name/
```

In this project:

```text
rescue_robot/                 <- ROS 2 package root
└── rescue_robot/             <- Python module
```

The `setup.py` file defines entry points such as:

```python
'frontier_explorer_node = rescue_robot.exploration.frontier_explorer_node:main'
```

This means Python must be able to import:

```python
rescue_robot.exploration.frontier_explorer_node
```

So the inner folder below must exist:

```text
ros2_ws/src/rescue_robot/rescue_robot/
```

Do **not** rename it, move it or delete it unless `setup.py`, imports, tests and launch files are updated accordingly.

---

## 3. Meaning of the main folders

### `launch/`

```text
ros2_ws/src/rescue_robot/launch/
```

Contains ROS 2 launch files.

```text
bringup.launch.py       final one-command entrypoint
mock_system.launch.py   mock development system
simulation.launch.py    Gazebo simulation layer
navigation.launch.py    SLAM/Nav2 layer
exploration.launch.py   exploration module
detection.launch.py     victim detection module
results.launch.py       metrics/export/visualization module
bt.launch.py            Behavior Tree supervisor
```

The final target is still one command:

```bash
ros2 launch rescue_bringup bringup.launch.py
```

The other launch files exist so that each module can be tested independently.

### `config/`

```text
ros2_ws/src/rescue_robot/config/
```

Contains YAML parameter files.

```text
bt_params.yaml
explorer_params.yaml
detector_params.yaml
results_params.yaml
simulation_params.yaml
slam_params.yaml
nav2_params.yaml
```

Each module should use its own config file to avoid Git conflicts.

### `behavior_trees/`

```text
ros2_ws/src/rescue_robot/behavior_trees/
```

Contains XML Behavior Tree definitions.

The project requires Behavior Trees rather than finite-state machines. This folder makes that design choice explicit.

### `worlds/` and `models/`

```text
ros2_ws/src/rescue_robot/worlds/
ros2_ws/src/rescue_robot/models/
```

Contain Gazebo world files and custom Gazebo models such as obstacles or victim markers.

### `maps/`

```text
ros2_ws/src/rescue_robot/maps/
```

Contains maps saved from SLAM or reference/test maps.

### `rviz/`

```text
ros2_ws/src/rescue_robot/rviz/
```

Contains RViz configuration files.

### `test_data/`

```text
ros2_ws/src/rescue_robot/test_data/
```

Contains small fake data used for development and tests.

---

## 4. Meaning of the Python submodules

### `mocks/`

```text
ros2_ws/src/rescue_robot/rescue_robot/mocks/
```

Contains fake publishers used before the real Gazebo/SLAM/Nav2 stack is ready.

Examples:

```text
mock_map_publisher.py       publishes /map
mock_victim_publisher.py    publishes /victims_map
mock_coverage_publisher.py  publishes /coverage
```

### `exploration/`

```text
ros2_ws/src/rescue_robot/rescue_robot/exploration/
```

Contains the autonomous exploration logic.

Final goal:

```text
/map -> frontier detection -> exploration goal -> Nav2 NavigateToPose
```

### `detection/`

```text
ros2_ws/src/rescue_robot/rescue_robot/detection/
```

Contains camera-based victim detection and future `tf2` localization.

Final goal:

```text
camera image -> marker detection -> camera frame pose -> tf2 -> map frame pose -> /victims_map
```

### `results/`

```text
ros2_ws/src/rescue_robot/rescue_robot/results/
```

Contains metrics, exports and visualization nodes.

Examples:

```text
coverage_evaluator_node.py
result_exporter_node.py
rviz_marker_node.py
```

### `bt/`

```text
ros2_ws/src/rescue_robot/rescue_robot/bt/
```

Contains the project-level Behavior Tree supervisor node.

---

## 5. Common confusion: code folder vs output folder

There are two different `results` locations:

```text
ros2_ws/src/rescue_robot/rescue_robot/results/
```

This is Python code.

```text
results/
```

This is output generated at runtime: CSV, JSON and annotated maps.

Do not confuse them.

---

## 6. Generated files

Python may create:

```text
__pycache__/
*.pyc
```

ROS 2 may create:

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
```

These are generated files. They should not be committed to Git.

To clean them:

```bash
find . -type d -name "__pycache__" -prune -exec rm -rf {} +
find . -name "*.pyc" -delete
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log
```

---

## 7. Rule for humans and AI agents

Do not simplify the tree by removing the inner package folder.

This path is intentional and required:

```text
ros2_ws/src/rescue_robot/rescue_robot/
```

If an AI coding agent suggests deleting or renaming it, reject that suggestion.
