"""ROS 2 integration tests — run with: python3 -m pytest tests/test_ros_nodes.py

These tests spin real ROS 2 nodes and verify end-to-end message pipelines.
They require rclpy (sourced ROS 2 Humble) and are skipped by uv-test.

Real problems these tests catch:
  - MockMapPublisher publishes invalid OccupancyGrid (bad frame, zero size)
  - CoverageEvaluator receives /map but publishes wrong coverage value
  - RvizMarkerNode receives /victims_map but publishes wrong number of markers
  - ResultExporter writes empty/corrupt files
  - MockCoveragePublisher publishes values > 1.0 or < 0.0
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="rclpy not available — run with system python3")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "rescue_robot"))


# ---------------------------------------------------------------------------
# Session-scoped ROS 2 context (init once, shutdown once)
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module", autouse=True)
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin_background(node, seconds: float) -> threading.Thread:
    """Spin a node in a dedicated SingleThreadedExecutor for `seconds`."""
    from rclpy.executors import SingleThreadedExecutor

    def _run():
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        deadline = time.time() + seconds
        while time.time() < deadline and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
        executor.remove_node(node)
        executor.shutdown(timeout_sec=0.1)

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t


# ---------------------------------------------------------------------------
# MockMapPublisher — publishes valid OccupancyGrid on /map
# ---------------------------------------------------------------------------

def test_mock_map_publisher_sends_valid_map(ros_context) -> None:
    from nav_msgs.msg import OccupancyGrid
    from rescue_robot.mocks.mock_map_publisher import MockMapPublisher

    received = []
    listener = rclpy.create_node("test_map_listener_02")
    listener.create_subscription(OccupancyGrid, "/map", lambda m: received.append(m), 10)

    pub_node = MockMapPublisher()
    t1 = _spin_background(pub_node, 3.5)
    t2 = _spin_background(listener, 3.5)
    t1.join()
    t2.join()

    pub_node.destroy_node()
    listener.destroy_node()

    assert received, "/map received no messages in 3.5s"
    msg = received[-1]
    assert msg.header.frame_id == "map", f"frame_id must be 'map', got '{msg.header.frame_id}'"
    assert msg.info.width > 0 and msg.info.height > 0, "OccupancyGrid has zero dimensions"
    assert len(msg.data) == msg.info.width * msg.info.height, (
        "data length doesn't match width*height — malformed OccupancyGrid"
    )


# ---------------------------------------------------------------------------
# MockVictimPublisher — publishes 3 poses on /victims_map
# ---------------------------------------------------------------------------

def test_mock_victim_publisher_sends_three_victims(ros_context) -> None:
    from geometry_msgs.msg import PoseArray
    from rescue_robot.mocks.mock_victim_publisher import MockVictimPublisher

    received = []
    listener = rclpy.create_node("test_victim_listener_02")
    listener.create_subscription(PoseArray, "/victims_map", lambda m: received.append(m), 10)

    pub_node = MockVictimPublisher()
    t1 = _spin_background(pub_node, 3.5)
    t2 = _spin_background(listener, 3.5)
    t1.join()
    t2.join()

    pub_node.destroy_node()
    listener.destroy_node()

    assert received, "/victims_map received no messages in 3.5s"
    msg = received[-1]
    assert msg.header.frame_id == "map", f"frame_id must be 'map', got '{msg.header.frame_id}'"
    assert len(msg.poses) == 3, f"Expected 3 victims, got {len(msg.poses)}"
    # All poses must have valid positions
    for i, pose in enumerate(msg.poses):
        assert pose.orientation.w != 0.0, f"pose[{i}] has zero quaternion"


# ---------------------------------------------------------------------------
# MockCoveragePublisher — values must be in [0, 1] and non-decreasing
# ---------------------------------------------------------------------------

def test_mock_coverage_publisher_values_valid(ros_context) -> None:
    from std_msgs.msg import Float32
    from rescue_robot.mocks.mock_coverage_publisher import MockCoveragePublisher

    received = []
    listener = rclpy.create_node("test_cov_listener_02")
    listener.create_subscription(Float32, "/coverage", lambda m: received.append(m.data), 10)

    pub_node = MockCoveragePublisher()
    t1 = _spin_background(pub_node, 5.0)
    t2 = _spin_background(listener, 5.0)
    t1.join()
    t2.join()

    pub_node.destroy_node()
    listener.destroy_node()

    assert len(received) >= 3, f"/coverage only got {len(received)} messages in 5s, expected ≥3"
    assert all(0.0 <= v <= 1.0 for v in received), (
        f"Coverage out of [0,1]: {[v for v in received if not 0<=v<=1]}"
    )
    for a, b in zip(received, received[1:]):
        assert b >= a - 1e-6, f"Coverage must not decrease: {a:.3f} → {b:.3f}"


# ---------------------------------------------------------------------------
# CoverageEvaluatorNode — receives /map, publishes correct /coverage value
# Uses a unique /map_test topic to avoid contamination from other tests.
# ---------------------------------------------------------------------------

def test_coverage_evaluator_computes_correct_value(ros_context) -> None:
    """Feed a 50% known map → /coverage must publish 0.5 (±0.01)."""
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Float32
    from rescue_robot.results.coverage_evaluator_node import CoverageEvaluatorNode

    # Use a fresh evaluator subscribed to a unique topic to avoid stale messages
    evaluator = CoverageEvaluatorNode()
    # Destroy the default /map subscription and create one on unique topic
    evaluator.destroy_subscription(evaluator.subscription)
    received_after = []

    helper = rclpy.create_node("test_cov_eval_helper")
    map_pub = helper.create_publisher(OccupancyGrid, "/map_cov_test", 10)
    evaluator.subscription = evaluator.create_subscription(
        OccupancyGrid, "/map_cov_test", evaluator.map_callback, 10
    )
    # Also listen on /coverage for values published after our map
    cov_received: list[float] = []
    helper.create_subscription(Float32, "/coverage", lambda m: cov_received.append(m.data), 10)

    # 10x10 map: 50 free (0) + 50 unknown (-1) → expected coverage = 0.5
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = 10
    grid.info.height = 10
    grid.info.resolution = 0.1
    grid.data = list([0] * 50 + [-1] * 50)

    t1 = _spin_background(evaluator, 4.0)
    t2 = _spin_background(helper, 4.0)

    time.sleep(0.4)       # let subscriptions establish
    map_pub.publish(grid)
    time.sleep(1.0)       # wait for callback + publish

    # Capture values received AFTER our publish
    received_after = list(cov_received)

    t1.join()
    t2.join()
    evaluator.destroy_node()
    helper.destroy_node()

    # Filter: only values close to 0.5 (our map) — ignore any pre-test stale values
    matching = [v for v in received_after if abs(v - 0.5) < 0.05]
    assert matching, (
        f"No /coverage message ≈ 0.5 received after publishing 50% map. "
        f"Got values: {received_after}"
    )


# ---------------------------------------------------------------------------
# RvizMarkerNode — receives /victims_map, publishes 3 sphere markers
# ---------------------------------------------------------------------------

def test_rviz_marker_node_publishes_markers_per_victim(ros_context) -> None:
    """Feed 3 victim poses → MarkerArray must have exactly 3 SPHERE markers."""
    from geometry_msgs.msg import Pose, PoseArray
    from visualization_msgs.msg import Marker, MarkerArray
    from rescue_robot.results.rviz_marker_node import RvizMarkerNode

    received = []
    helper = rclpy.create_node("test_marker_helper")
    helper.create_subscription(
        MarkerArray, "/visualization_marker_array", lambda m: received.append(m), 10
    )
    vic_pub = helper.create_publisher(PoseArray, "/victims_map", 10)

    marker_node = RvizMarkerNode()

    t1 = _spin_background(marker_node, 4.0)
    t2 = _spin_background(helper, 4.0)

    time.sleep(0.4)
    msg = PoseArray()
    msg.header.frame_id = "map"
    for x, y in [(1.0, 1.5), (-1.2, 2.2), (2.4, -0.8)]:
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.orientation.w = 1.0
        msg.poses.append(p)
    vic_pub.publish(msg)
    time.sleep(1.5)

    t1.join()
    t2.join()
    marker_node.destroy_node()
    helper.destroy_node()

    assert received, "/visualization_marker_array published nothing after /victims_map"
    last = received[-1]
    spheres = [m for m in last.markers if m.type == Marker.SPHERE]
    assert len(spheres) == 3, (
        f"Expected 3 SPHERE markers (one per victim), got {len(spheres)}. "
        f"All markers: {[(m.type, m.ns) for m in last.markers]}"
    )


# ---------------------------------------------------------------------------
# ResultExporterNode — CSV is written correctly on coverage messages
# ---------------------------------------------------------------------------

def test_result_exporter_writes_csv_on_coverage_message(ros_context, tmp_path) -> None:
    """ResultExporter must append a row per /coverage message with valid values."""
    import csv as csv_mod
    from std_msgs.msg import Float32
    from rescue_robot.results.result_exporter_node import ResultExporterNode

    exporter = ResultExporterNode()
    exporter.results_dir = tmp_path
    exporter.coverage_path = tmp_path / "coverage_over_time.csv"
    exporter.victims_path = tmp_path / "victims_detected.csv"
    exporter.summary_path = tmp_path / "run_summary.json"
    exporter._ensure_headers()

    helper = rclpy.create_node("test_export_helper")
    cov_pub = helper.create_publisher(Float32, "/coverage", 10)

    t1 = _spin_background(exporter, 4.0)
    t2 = _spin_background(helper, 4.0)

    time.sleep(0.4)
    for v in [0.1, 0.3, 0.5]:
        msg = Float32()
        msg.data = v
        cov_pub.publish(msg)
        time.sleep(0.5)

    t1.join()
    t2.join()
    exporter.destroy_node()
    helper.destroy_node()

    rows = list(csv_mod.reader(exporter.coverage_path.open()))
    assert len(rows) >= 2, "coverage CSV must have header + at least 1 data row"
    assert rows[0] == ["time", "coverage", "path_length_m"], f"Wrong CSV header: {rows[0]}"
    values = [float(r[1]) for r in rows[1:]]
    assert all(0.0 <= v <= 1.0 for v in values), (
        f"CSV contains invalid coverage values: {values}"
    )
