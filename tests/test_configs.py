"""Tests for YAML configuration files — structure, required keys, valid values."""
from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "rescue_robot"
CONFIG = PKG / "config"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


# ---------------------------------------------------------------------------
# nav2_params.yaml
# ---------------------------------------------------------------------------

def test_nav2_params_required_sections() -> None:
    data = load_yaml(CONFIG / "nav2_params.yaml")
    required = [
        "bt_navigator",
        "controller_server",
        "planner_server",
        "local_costmap",
        "global_costmap",
        "lifecycle_manager_navigation",
    ]
    missing = [s for s in required if s not in data]
    assert not missing, f"nav2_params.yaml missing sections: {missing}"


def test_nav2_params_use_sim_time_true() -> None:
    data = load_yaml(CONFIG / "nav2_params.yaml")
    for section, content in data.items():
        if isinstance(content, dict) and "ros__parameters" in content:
            params = content["ros__parameters"]
            if "use_sim_time" in params:
                assert params["use_sim_time"] is True, (
                    f"nav2_params.yaml [{section}] use_sim_time must be true"
                )


def test_nav2_params_no_amcl() -> None:
    """SLAM toolbox provides localisation — amcl YAML section must not be present."""
    data = load_yaml(CONFIG / "nav2_params.yaml")
    amcl_keys = [k for k in data if "amcl" in k.lower()]
    assert not amcl_keys, (
        f"nav2_params.yaml must not contain AMCL sections, found: {amcl_keys}"
    )


def test_nav2_params_controller_velocity_limits() -> None:
    data = load_yaml(CONFIG / "nav2_params.yaml")
    params = data["controller_server"]["ros__parameters"]
    follow = params.get("FollowPath", {})
    max_vel_x = follow.get("max_vel_x", 0)
    assert 0 < max_vel_x <= 0.26, (
        f"max_vel_x={max_vel_x} — TurtleBot3 Waffle Pi hardware limit is 0.26 m/s"
    )


def test_nav2_params_lifecycle_nodes_complete() -> None:
    data = load_yaml(CONFIG / "nav2_params.yaml")
    node_names = (
        data["lifecycle_manager_navigation"]["ros__parameters"]["node_names"]
    )
    required_lifecycle = {"controller_server", "planner_server", "bt_navigator"}
    missing = required_lifecycle - set(node_names)
    assert not missing, f"lifecycle_manager_navigation missing nodes: {missing}"


# ---------------------------------------------------------------------------
# slam_params.yaml
# ---------------------------------------------------------------------------

def test_slam_params_required_keys() -> None:
    data = load_yaml(CONFIG / "slam_params.yaml")
    params = data["slam_toolbox"]["ros__parameters"]
    required = ["odom_frame", "map_frame", "base_frame", "scan_topic",
                "use_sim_time", "mode", "resolution"]
    missing = [k for k in required if k not in params]
    assert not missing, f"slam_params.yaml missing keys: {missing}"


def test_slam_params_use_sim_time() -> None:
    data = load_yaml(CONFIG / "slam_params.yaml")
    params = data["slam_toolbox"]["ros__parameters"]
    assert params["use_sim_time"] is True, "slam_params.yaml: use_sim_time must be true"


def test_slam_params_scan_topic() -> None:
    data = load_yaml(CONFIG / "slam_params.yaml")
    topic = data["slam_toolbox"]["ros__parameters"]["scan_topic"]
    assert topic == "/scan", f"slam_params.yaml scan_topic must be /scan, got {topic!r}"


def test_slam_params_mode_mapping() -> None:
    data = load_yaml(CONFIG / "slam_params.yaml")
    mode = data["slam_toolbox"]["ros__parameters"]["mode"]
    assert mode == "mapping", f"slam_params.yaml mode must be 'mapping', got {mode!r}"


# ---------------------------------------------------------------------------
# waypoints YAML files
# ---------------------------------------------------------------------------

def _check_waypoints_file(path: Path) -> None:
    data = load_yaml(path)
    assert "waypoints" in data, f"{path.name}: missing 'waypoints' key"
    wps = data["waypoints"]
    assert isinstance(wps, list) and len(wps) > 0, f"{path.name}: waypoints list is empty"
    for i, wp in enumerate(wps):
        for field in ("x", "y"):
            assert field in wp, f"{path.name}: waypoint[{i}] missing field '{field}'"
        assert isinstance(wp["x"], (int, float)), f"{path.name}: waypoint[{i}].x not a number"
        assert isinstance(wp["y"], (int, float)), f"{path.name}: waypoint[{i}].y not a number"


def test_waypoints_tb3_house() -> None:
    _check_waypoints_file(CONFIG / "waypoints.yaml")


def test_waypoints_tb4_warehouse() -> None:
    _check_waypoints_file(CONFIG / "waypoints_tb4_warehouse.yaml")


# ---------------------------------------------------------------------------
# RViz config
# ---------------------------------------------------------------------------

def test_rviz_config_valid_yaml() -> None:
    rviz_path = PKG / "rviz" / "project_view.rviz"
    assert rviz_path.exists(), "project_view.rviz not found"
    data = load_yaml(rviz_path)
    assert "Visualization Manager" in data, "project_view.rviz: missing Visualization Manager"
    vm = data["Visualization Manager"]
    assert "Global Options" in vm, "project_view.rviz: missing Global Options"
    assert vm["Global Options"]["Fixed Frame"] == "map", (
        "project_view.rviz: Fixed Frame must be 'map'"
    )


def test_rviz_config_has_required_displays() -> None:
    data = load_yaml(PKG / "rviz" / "project_view.rviz")
    displays = data["Visualization Manager"]["Displays"]
    class_names = [d.get("Class", "") for d in displays]
    required = [
        "rviz_default_plugins/Map",
        "rviz_default_plugins/LaserScan",
        "rviz_default_plugins/RobotModel",
        "rviz_default_plugins/MarkerArray",
        "rviz_default_plugins/Image",
    ]
    missing = [c for c in required if c not in class_names]
    assert not missing, f"project_view.rviz missing displays: {missing}"


def test_rviz_camera_display_is_image_not_camera() -> None:
    """Camera feed must use Image display (no TF sync) to avoid queue overflow.

    The Camera display type uses a MessageFilter that syncs image timestamps with TF.
    Under sim-time, this causes the queue to fill up and all camera messages are
    dropped (verified in rviz2_17429_1779285372836.log).  Image display shows the
    latest frame directly without TF, which is the correct approach.
    """
    data = load_yaml(PKG / "rviz" / "project_view.rviz")
    displays = data["Visualization Manager"]["Displays"]
    camera_display = next(
        (d for d in displays if d.get("Class") == "rviz_default_plugins/Image"), None
    )
    assert camera_display is not None, (
        "project_view.rviz: must have an Image display for camera feed. "
        "Do NOT use Camera display — it causes TF queue overflow under sim time."
    )
    topic = camera_display.get("Topic", {}).get("Value", "")
    assert topic == "/camera/image_raw", (
        f"Image display topic must be /camera/image_raw, got {topic!r}"
    )
    assert camera_display.get("Enabled") is True, (
        "Camera Image display must be Enabled: true"
    )


def test_rviz_laserscan_display_is_enabled() -> None:
    data = load_yaml(PKG / "rviz" / "project_view.rviz")
    displays = data["Visualization Manager"]["Displays"]
    scan = next(
        (d for d in displays if d.get("Class") == "rviz_default_plugins/LaserScan"), None
    )
    assert scan is not None, "project_view.rviz: missing LaserScan display"
    assert scan.get("Value") is True, "LaserScan display must be Value: true (enabled)"
    topic = scan.get("Topic", {}).get("Value", "")
    assert topic == "/scan", f"LaserScan topic must be /scan, got {topic!r}"


def test_rviz_local_costmap_is_enabled() -> None:
    data = load_yaml(PKG / "rviz" / "project_view.rviz")
    displays = data["Visualization Manager"]["Displays"]
    costmap = next(
        (d for d in displays
         if d.get("Class") == "rviz_default_plugins/Map"
         and d.get("Name") == "Local Costmap"),
        None,
    )
    assert costmap is not None, "project_view.rviz: missing Local Costmap display"
    assert costmap.get("Enabled") is True, "Local Costmap: Enabled must be true"
    assert costmap.get("Value") is True, (
        "Local Costmap: Value must be true — Enabled:true + Value:false leaves the "
        "display in a broken state where it is registered but never renders"
    )


# ---------------------------------------------------------------------------
# package.xml
# ---------------------------------------------------------------------------

def test_package_xml_has_required_deps() -> None:
    import xml.etree.ElementTree as ET
    tree = ET.parse(PKG / "package.xml")
    root = tree.getroot()
    deps = {el.text for el in root.iter() if el.tag.endswith("depend")}
    required = {"rclpy", "nav2_msgs", "nav_msgs", "geometry_msgs",
                "visualization_msgs", "sensor_msgs"}
    missing = required - deps
    assert not missing, f"package.xml missing dependencies: {missing}"
