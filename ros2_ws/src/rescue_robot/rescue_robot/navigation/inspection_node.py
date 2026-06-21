"""Autonomous room-inspection node — phase 2 of the BT-orchestrated mission.

Idle until the Behavior Tree raises ``/mission/inspect_enable`` (Bool). Then it reads
the robot's own ``/map``, derives one inspection pose per discovered outer room
(`inspection_planner.poses_from_grid`), drives each via Nav2
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
        self.declare_parameter("dwell_sec", 13.0)
        self.declare_parameter("dwell_spin_speed", 0.5)    # rad/s; 13 s * 0.5 = ~6.5 rad > full turn
        self.declare_parameter("inspect_grid", 2)
        self.declare_parameter("inspect_offset", 1.3)
        self.declare_parameter("inspect_min_cells", 80)
        # Robustness (cross-machine): when Nav2 rejects/aborts a pose (it fell in the
        # inflation layer or near unknown space on a slightly different map), DON'T just
        # re-submit immediately — wait for the costmap to settle, then try fallback poses
        # pulled toward the room interior (out of the wall inflation). See _goto.
        self.declare_parameter("retry_wait_sec", 4.0)
        self.declare_parameter("interior_pulls", [0.5, 0.9, 1.3])  # m, pull toward room centre
        # Last-resort robustness: if the standoff pose AND every straight interior pull are
        # unreachable (Nav2 won't plan into that corner — inflated/rough map), try a FAN of
        # vantage points on rings around the standoff pose. They stay within camera range
        # (<= ~max radius), so the robot can approach the corner FROM THE SIDE and still
        # sweep the wall tag. This is what recovers a room a single straight line of pulls
        # cannot (observed: NW room skipped -> victim missed). Radii kept <= 1.1 m so the
        # vantage is comfortably inside the ~2 m AprilTag detection range of the wall.
        self.declare_parameter("vantage_radii", [0.7, 1.1])  # m, rings around the standoff pose
        self.declare_parameter("vantage_count", 8)           # angular samples per ring

        self._timeout = float(self.get_parameter("goal_timeout_sec").value)
        self._snap_radius = int(self.get_parameter("goal_snap_radius").value)
        self._dwell = float(self.get_parameter("dwell_sec").value)
        self._spin = float(self.get_parameter("dwell_spin_speed").value)
        self._grid = int(self.get_parameter("inspect_grid").value)
        self._offset = float(self.get_parameter("inspect_offset").value)
        self._min_cells = int(self.get_parameter("inspect_min_cells").value)
        self._retry_wait = float(self.get_parameter("retry_wait_sec").value)
        self._pulls = [float(p) for p in self.get_parameter("interior_pulls").value]
        self._vantage_radii = [float(r) for r in self.get_parameter("vantage_radii").value]
        self._vantage_count = int(self.get_parameter("vantage_count").value)

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
        if failed:
            self.get_logger().info(
                f"{len(failed)} room(s) unreached — settling the costmap before a 2nd pass.")
            self._wait_settle(self._retry_wait)
        for i, wp in failed:  # second pass — robot is elsewhere now, costmap settled
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
        """Reach the room and sweep the camera. Tries the snapped pose, then fallback poses
        pulled toward the room interior (out of the wall inflation layer) — with a costmap-settle
        WAIT before each retry, not an immediate re-submission. A reached pose then triggers a
        guaranteed FULL-turn sweep so the tag is caught whatever the arrival orientation."""
        yaw = float(wp.get("yaw", 0.0))
        candidates = list(self._candidate_poses(wp))
        for k, (cx, cy) in enumerate(candidates):
            if k > 0:
                self.get_logger().info(
                    f"  goal failed — waiting {self._retry_wait:.0f}s for the costmap to settle, "
                    f"then fallback {k}/{len(candidates)-1} (interior pull / side vantage near the room)")
                self._wait_settle(self._retry_wait)
            if self._send_goal_xy(cx, cy, yaw):
                self._sweep()
                return True
        self.get_logger().warn(
            f"  pose unreachable after all {len(candidates)} candidates (pulls + vantage fan) "
            f"— skipping this room.")
        return False

    def _candidate_poses(self, wp: dict):
        """Yield (x, y) candidates in increasing order of desperation:
          1. the snapped original standoff pose;
          2. the same pose pulled straight toward the room interior (opposite the wall-facing
             yaw) so it leaves the wall inflation layer where Nav2 refuses to plan;
          3. a FAN of vantage points on rings around the standoff pose, so when the whole
             straight-back line is blocked the robot can still reach the corner from the side.
        Each candidate is snapped to the nearest free cell; duplicates are skipped."""
        yaw = float(wp.get("yaw", 0.0))
        x0, y0 = float(wp["x"]), float(wp["y"])
        seen = set()

        def _emit(x, y):
            sx, sy = self._snap(x, y)
            key = (round(sx, 2), round(sy, 2))
            if key in seen:
                return None
            seen.add(key)
            return (sx, sy)

        c = _emit(x0, y0)
        if c:
            yield c
        for pull in self._pulls:
            c = _emit(x0 - pull * math.cos(yaw), y0 - pull * math.sin(yaw))
            if c:
                yield c
        # Fan: rings around the standoff pose, angles swept around the full circle. Reaching
        # any one of these and doing the 360 deg sweep still catches the wall tag (in range).
        n = max(1, self._vantage_count)
        for r in self._vantage_radii:
            for i in range(n):
                a = 2.0 * math.pi * i / n
                c = _emit(x0 + r * math.cos(a), y0 + r * math.sin(a))
                if c:
                    yield c

    def _sweep(self):
        """Spin in place for at least a FULL turn (+15 %) so every wall is swept regardless of
        the arrival orientation — this is what catches a tag the robot would otherwise face away
        from (the cause of a victim missed at a reached pose)."""
        if self._spin <= 0.0 or (self._dwell <= 0.0):
            return
        full_turn = 2.0 * math.pi * 1.15 / max(self._spin, 0.1)
        dur = max(self._dwell, full_turn)
        self.get_logger().info(
            f"  sweep {dur:.0f}s @ {self._spin:.1f} rad/s (full turn, catch the tag)")
        self._spin_in_place(dur, self._spin)

    def _wait_settle(self, secs):
        """Hold still while the Nav2 costmap updates/clears before a retry."""
        end = time.time() + secs
        while time.time() < end:
            self._cmd_pub.publish(Twist())
            time.sleep(0.2)

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

    def _send_goal_xy(self, x: float, y: float, yaw: float) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)
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
