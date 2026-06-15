"""Autonomous room-inspection node — phase 2 of the BT-orchestrated mission.

Idle until the Behavior Tree raises ``/mission/inspect_enable`` (Bool). Then it reads
the robot's own ``/map``, derives one inspection pose per discovered outer room
(`inspection_planner.poses_from_grid` — NO victim coordinates), drives each via Nav2
``NavigateToPose`` (snapped to a free cell, with a short in-place camera sweep at the
pose so the wall AprilTag is seen), and finally latches ``/mission/inspect_done`` (Bool)
so the BT can advance. This is the action behind the BT's ``InspectPhase`` node.
"""
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from rescue_robot.exploration.frontier_search import nearest_free_cell
from rescue_robot.navigation.inspection_planner import poses_from_grid
from rescue_robot.utils.node_runner import run_node


class InspectionNode(Node):
    def __init__(self):
        super().__init__("inspection_node")

        self.declare_parameter("goal_timeout_sec", 150.0)
        self.declare_parameter("goal_snap_radius", 16)
        self.declare_parameter("dwell_sec", 8.0)
        self.declare_parameter("dwell_spin_speed", 0.4)   # rad/s, short wall sweep
        self.declare_parameter("inspect_grid", 2)
        self.declare_parameter("inspect_offset", 1.3)
        self.declare_parameter("inspect_min_cells", 80)

        self._timeout = float(self.get_parameter("goal_timeout_sec").value)
        self._snap_radius = int(self.get_parameter("goal_snap_radius").value)
        self._dwell = float(self.get_parameter("dwell_sec").value)
        self._spin = float(self.get_parameter("dwell_spin_speed").value)
        self._grid = int(self.get_parameter("inspect_grid").value)
        self._offset = float(self.get_parameter("inspect_offset").value)
        self._min_cells = int(self.get_parameter("inspect_min_cells").value)

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # RViz visualisation of the inspection algorithm: the map-derived poses as
        # arrows + the pose currently being inspected highlighted.
        self._marker_pub = self.create_publisher(MarkerArray, "/inspection/poses", 10)

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self._done_pub = self.create_publisher(Bool, "/mission/inspect_done", latched)
        self.create_subscription(Bool, "/mission/inspect_enable", self._on_enable, latched)

        self._started = False
        self.get_logger().info("Inspection node ready — waiting for /mission/inspect_enable.")

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_enable(self, msg: Bool):
        if msg.data and not self._started:
            self._started = True
            threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        self.get_logger().info("Inspection enabled by BT — planning from /map.")
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Nav2 NavigateToPose not available — aborting inspection.")
            self._publish_done()
            return
        # Give SLAM a moment to publish the latest map after exploration stops.
        for _ in range(40):
            if self._map is not None:
                break
            time.sleep(0.25)
        m = self._map
        if m is None:
            self.get_logger().error("No /map received — aborting inspection.")
            self._publish_done()
            return

        poses = poses_from_grid(
            m.data, m.info.width, m.info.height, m.info.resolution,
            m.info.origin.position.x, m.info.origin.position.y,
            grid=self._grid, offset=self._offset, min_cells=self._min_cells,
        )
        self.get_logger().info(f"Inspecting {len(poses)} discovered room(s) (map-derived poses).")
        failed = []
        for i, wp in enumerate(poses):
            self.get_logger().info(
                f"Inspect pose {i+1}/{len(poses)}: x={wp['x']:.2f} y={wp['y']:.2f} yaw={wp['yaw']:.2f}")
            self._publish_markers(poses, i)
            if not self._goto(wp):
                failed.append((i, wp))
        for i, wp in failed:  # second pass — robot is elsewhere now, often succeeds
            self.get_logger().info(f"Retry inspect pose {i+1} (2nd pass)...")
            self._publish_markers(poses, i)
            self._goto(wp)

        self.get_logger().info("Inspection complete — latching /mission/inspect_done.")
        self._publish_markers(poses, -1)
        self._publish_done()

    def _publish_markers(self, poses, current):
        """RViz: each inspection pose as an arrow (blue), the one being inspected in
        orange. Lets you watch the inspection algorithm visit each discovered room."""
        arr = MarkerArray()
        for i, wp in enumerate(poses):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "inspection"; m.id = i; m.type = Marker.ARROW; m.action = Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = 0.6, 0.12, 0.12
            if i == current:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.12, 0.45, 0.95, 0.85
            m.pose.position.x = float(wp["x"]); m.pose.position.y = float(wp["y"]); m.pose.position.z = 0.1
            yaw = float(wp.get("yaw", 0.0))
            m.pose.orientation.z = math.sin(yaw / 2.0); m.pose.orientation.w = math.cos(yaw / 2.0)
            arr.markers.append(m)
        self._marker_pub.publish(arr)

    def _publish_done(self):
        self._cmd_pub.publish(Twist())
        msg = Bool()
        msg.data = True
        self._done_pub.publish(msg)

    def _goto(self, wp: dict) -> bool:
        if not self._send_goal(wp):
            return False
        if self._dwell > 0.0:
            self.get_logger().info(
                f"  sweep {self._dwell:.0f}s @ {self._spin:.1f} rad/s (face the wall, catch the tag)")
            self._spin_in_place(self._dwell, self._spin)
        return True

    def _spin_in_place(self, duration, speed):
        twist = Twist()
        twist.angular.z = speed
        end = time.time() + duration
        while time.time() < end:
            self._cmd_pub.publish(twist)
            time.sleep(0.1)
        self._cmd_pub.publish(Twist())

    def _snap(self, x, y):
        m = self._map
        if m is None or self._snap_radius <= 0 or m.info.resolution <= 0.0:
            return x, y
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        free = nearest_free_cell(m.data, m.info.width, m.info.height,
                                 (x - ox) / res, (y - oy) / res, self._snap_radius)
        if free is None:
            return x, y
        fx, fy = free
        return ox + (fx + 0.5) * res, oy + (fy + 0.5) * res

    def _send_goal(self, wp: dict) -> bool:
        sx, sy = self._snap(float(wp["x"]), float(wp["y"]))
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(sx, sy, wp.get("yaw", 0.0))
        future = self._client.send_goal_async(goal)
        deadline = time.time() + 30.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn("Goal acceptance timed out.")
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            return False
        result_future = handle.get_result_async()
        deadline = time.time() + self._timeout
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            self.get_logger().warn(f"Goal timed out after {self._timeout}s.")
            handle.cancel_goal_async()
            return False
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def _make_pose(self, x, y, yaw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose


def main(args=None):
    run_node(InspectionNode, args=args, executor_factory=MultiThreadedExecutor)


if __name__ == "__main__":
    main()
