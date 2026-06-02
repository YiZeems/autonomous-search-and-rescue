"""Inter-module interface consistency tests.

Verifies that the topic names published by mock nodes match the topics
subscribed by real nodes, as defined in docs/interfaces.md.
Topics must be consistent across the full pipeline so each member can work
independently with mock data.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY_PKG = ROOT / "ros2_ws" / "src" / "rescue_robot" / "rescue_robot"


# ---------------------------------------------------------------------------
# Helpers — extract topic strings from source without importing ROS
# ---------------------------------------------------------------------------

def _source(rel: str) -> str:
    return (PY_PKG / rel).read_text(encoding="utf-8")


def _topics(source: str, method: str) -> set[str]:
    """Return all string literals passed as second arg to create_publisher/subscription."""
    pattern = rf"{method}\s*\([^,]+,\s*['\"]([^'\"]+)['\"]"
    return set(re.findall(pattern, source))


def _published(rel: str) -> set[str]:
    return _topics(_source(rel), "create_publisher")


def _subscribed(rel: str) -> set[str]:
    return _topics(_source(rel), "create_subscription")


# ---------------------------------------------------------------------------
# /map  (nav_msgs/OccupancyGrid)
# ---------------------------------------------------------------------------

def test_mock_map_publishes_on_map_topic() -> None:
    topics = _published("mocks/mock_map_publisher.py")
    assert "/map" in topics, f"mock_map_publisher must publish /map, found: {topics}"


def test_frontier_explorer_subscribes_to_map() -> None:
    topics = _subscribed("exploration/frontier_explorer_node.py")
    assert "/map" in topics, (
        f"frontier_explorer_node must subscribe to /map, found: {topics}"
    )


def test_coverage_evaluator_subscribes_to_map() -> None:
    topics = _subscribed("results/coverage_evaluator_node.py")
    assert "/map" in topics, (
        f"coverage_evaluator_node must subscribe to /map, found: {topics}"
    )


# ---------------------------------------------------------------------------
# /victims_map  (geometry_msgs/PoseArray)
# ---------------------------------------------------------------------------

def test_mock_victim_publishes_on_victims_map_topic() -> None:
    topics = _published("mocks/mock_victim_publisher.py")
    assert "/victims_map" in topics, (
        f"mock_victim_publisher must publish /victims_map, found: {topics}"
    )


def test_victim_detector_publishes_on_victims_map_topic() -> None:
    # Topic is set via declare_parameter default — check the default value string
    source = _source("detection/victim_detector_node.py")
    assert "/victims_map" in source, (
        "victim_detector_node must use /victims_map as default output_topic"
    )


def test_rviz_marker_subscribes_to_victims_map() -> None:
    topics = _subscribed("results/rviz_marker_node.py")
    assert "/victims_map" in topics, (
        f"rviz_marker_node must subscribe to /victims_map, found: {topics}"
    )


# ---------------------------------------------------------------------------
# /coverage  (std_msgs/Float32)
# ---------------------------------------------------------------------------

def test_mock_coverage_publishes_on_coverage_topic() -> None:
    topics = _published("mocks/mock_coverage_publisher.py")
    assert "/coverage" in topics, (
        f"mock_coverage_publisher must publish /coverage, found: {topics}"
    )


def test_coverage_evaluator_publishes_on_coverage_topic() -> None:
    topics = _published("results/coverage_evaluator_node.py")
    assert "/coverage" in topics, (
        f"coverage_evaluator_node must publish /coverage, found: {topics}"
    )


# ---------------------------------------------------------------------------
# /visualization_marker_array  (visualization_msgs/MarkerArray)
# ---------------------------------------------------------------------------

def test_rviz_marker_publishes_marker_array() -> None:
    # Topic is set via declare_parameter default — check the default value string
    source = _source("results/rviz_marker_node.py")
    assert "/visualization_marker_array" in source, (
        "rviz_marker_node must default to /visualization_marker_array"
    )


# ---------------------------------------------------------------------------
# Entrypoint contracts — every node module has main() and a Node subclass
# ---------------------------------------------------------------------------

NODE_MODULES = [
    "exploration/frontier_explorer_node.py",
    "detection/victim_detector_node.py",
    "results/coverage_evaluator_node.py",
    "results/result_exporter_node.py",
    "results/rviz_marker_node.py",
    "mocks/mock_map_publisher.py",
    "mocks/mock_victim_publisher.py",
    "mocks/mock_coverage_publisher.py",
    "bt/bt_supervisor_node.py",
    "navigation/waypoint_follower_node.py",
]


def _has_main(source: str) -> bool:
    tree = ast.parse(source)
    return any(
        isinstance(n, ast.FunctionDef) and n.name == "main"
        for n in ast.walk(tree)
    )


def _has_node_subclass(source: str) -> bool:
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            for base in node.bases:
                base_str = ast.unparse(base) if hasattr(ast, "unparse") else ""
                if "Node" in base_str:
                    return True
    return False


def test_all_nodes_have_main_function() -> None:
    missing = []
    for rel in NODE_MODULES:
        src = _source(rel)
        if not _has_main(src):
            missing.append(rel)
    assert not missing, f"These node modules are missing main(): {missing}"


def test_all_nodes_have_node_subclass() -> None:
    missing = []
    for rel in NODE_MODULES:
        src = _source(rel)
        if not _has_node_subclass(src):
            missing.append(rel)
    assert not missing, f"These modules have no Node subclass: {missing}"


# ---------------------------------------------------------------------------
# setup.py entrypoints — every console_script points to a real module + main
# ---------------------------------------------------------------------------

def test_setup_py_entrypoints_resolve() -> None:
    setup_py = (ROOT / "ros2_ws" / "src" / "rescue_robot" / "setup.py")
    source = setup_py.read_text()
    # Extract  'name = pkg.module:main'
    pattern = r"'(\w+)\s*=\s*([\w.]+):(\w+)'"
    entries = re.findall(pattern, source)
    pkg_root = ROOT / "ros2_ws" / "src" / "rescue_robot"
    errors = []
    for _name, module_path, func in entries:
        # Convert dotted module path to file path
        rel_path = module_path.replace(".", "/") + ".py"
        full_path = pkg_root / rel_path
        if not full_path.exists():
            errors.append(f"{module_path} → {rel_path} not found")
            continue
        src = full_path.read_text()
        if f"def {func}(" not in src:
            errors.append(f"{module_path}:{func}() not found in {rel_path}")
    assert not errors, "setup.py entrypoint errors:\n" + "\n".join(errors)


# ---------------------------------------------------------------------------
# Launch files — importable as Python modules (no missing imports at parse)
# ---------------------------------------------------------------------------

# rescue_robot: module-level launches
RESCUE_ROBOT_LAUNCHES = [
    "launch/mock_system.launch.py",
    "launch/navigation.launch.py",
    "launch/exploration.launch.py",
    "launch/detection.launch.py",
    "launch/results.launch.py",
    "launch/waypoint.launch.py",
    "launch/simulation.launch.py",
    "launch/bt.launch.py",
]

# rescue_bringup: integration launches (main entry points)
RESCUE_BRINGUP_LAUNCHES = [
    "launch/bringup.launch.py",
    "launch/demo.launch.py",
    "launch/exploration.launch.py",
]


def test_launch_files_parse_and_have_generate_function() -> None:
    checks = (
        [(ROOT / "ros2_ws" / "src" / "rescue_robot", rel) for rel in RESCUE_ROBOT_LAUNCHES]
        + [(ROOT / "ros2_ws" / "src" / "rescue_bringup", rel) for rel in RESCUE_BRINGUP_LAUNCHES]
    )
    errors = []
    for pkg, rel in checks:
        path = pkg / rel
        if not path.exists():
            errors.append(f"{rel}: file not found")
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError as e:
            errors.append(f"{rel}: SyntaxError — {e}")
            continue
        has_generate = any(
            isinstance(n, ast.FunctionDef) and n.name == "generate_launch_description"
            for n in ast.walk(tree)
        )
        if not has_generate:
            errors.append(f"{rel}: missing generate_launch_description()")
    assert not errors, "Launch file errors:\n" + "\n".join(errors)
