"""Waypoint follower — sends a predefined sequence of poses to Nav2 NavigateToPose.

Usage:
    ros2 launch rescue_robot waypoint.launch.py
    ros2 launch rescue_robot waypoint.launch.py waypoints_file:=/abs/path/to/waypoints.yaml
    ros2 launch rescue_robot waypoint.launch.py loop:=true

Waypoints YAML format (config/waypoints.yaml):
    waypoints:
      - {x: 0.0, y: 0.0, yaw: 0.0}
      - {x: 1.5, y: 0.5, yaw: 1.57}
"""

import math
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rescue_robot.exploration.frontier_search import nearest_free_cell
from rescue_robot.utils.node_runner import run_node


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower_node")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("loop", False)
        self.declare_parameter("goal_timeout_sec", 150.0)
        # Snap each waypoint to the nearest FREE map cell within this radius (cells)
        # before sending it to Nav2. A pose deep in a room corner often lands in
        # still-unknown / wall-inflated space and Nav2 rejects it instantly; the
        # snap pulls it onto the closest reachable free cell (same trick as the
        # frontier explorer). 0 disables snapping.
        self.declare_parameter("goal_snap_radius", 16)
        # Pause facing the wall at each reached waypoint so apriltag_ros has time to
        # lock the tag (the waypoint already faces the wall). 0 disables.
        self.declare_parameter("dwell_sec", 4.0)
        # During the dwell, ROTATE IN PLACE at this rate (rad/s) so the camera sweeps
        # the whole room — the inspection pose only approximates the wall direction, so a
        # full turn guarantees the wall AprilTag enters the frame. 0 = dwell without spin.
        self.declare_parameter("dwell_spin_speed", 0.6)

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._loop = self.get_parameter("loop").value
        self._timeout = self.get_parameter("goal_timeout_sec").value
        self._snap_radius = int(self.get_parameter("goal_snap_radius").value)
        self._dwell = float(self.get_parameter("dwell_sec").value)
        self._dwell_spin = float(self.get_parameter("dwell_spin_speed").value)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Latest map for goal snapping (SLAM publishes /map latched: transient-local).
        self._map = None
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        waypoints_file = self.get_parameter("waypoints_file").value
        self._waypoints = self._load_waypoints(waypoints_file)

        self.get_logger().info(
            f"WaypointFollower ready — {len(self._waypoints)} waypoints, "
            f"loop={self._loop}, timeout={self._timeout}s"
        )

        # Run navigation in a background thread so the executor stays free
        # to process action client callbacks (spin_until_future_complete from
        # within a timer callback blocks the executor in single-threaded mode).
        self._nav_thread = threading.Thread(target=self._run, daemon=True)
        self._nav_thread.start()

    def _load_waypoints(self, path: str) -> list:
        if not path:
            return [
                {"x": 0.0,  "y": 0.0,  "yaw": 0.0},
                {"x": 1.5,  "y": 0.0,  "yaw": 0.0},
                {"x": 1.5,  "y": 1.5,  "yaw": 1.57},
                {"x": 0.0,  "y": 1.5,  "yaw": 3.14},
                {"x": -1.5, "y": 1.5,  "yaw": 3.14},
                {"x": -1.5, "y": 0.0,  "yaw": -1.57},
                {"x": -1.5, "y": -1.5, "yaw": -1.57},
                {"x": 0.0,  "y": -1.5, "yaw": 0.0},
            ]
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            return data.get("waypoints", [])
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from {path}: {e}")
            return []

    def _run(self):
        if not self._waypoints:
            self.get_logger().warn("No waypoints to follow.")
            return

        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        if not self._client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("NavigateToPose server not available after 60s. Aborting.")
            return

        while True:
            n = len(self._waypoints)
            failed = []
            for i, wp in enumerate(self._waypoints):
                self.get_logger().info(
                    f"Navigating to waypoint {i+1}/{n}: "
                    f"x={wp['x']:.2f} y={wp['y']:.2f} yaw={wp.get('yaw', 0.0):.2f}"
                )
                if self._goto(wp):
                    self.get_logger().info(f"Waypoint {i+1} reached.")
                else:
                    self.get_logger().warn(f"Waypoint {i+1} failed — retry at end.")
                    failed.append((i, wp))

            # 2nd pass on the failures: the robot is now elsewhere (often closer / less
            # boxed-in), so a goal unreachable on pass 1 frequently succeeds on pass 2.
            # Retry failed goals once after the robot has moved and the costmap has updated.
            for i, wp in failed:
                self.get_logger().info(f"Retry waypoint {i+1}/{n} (2nd pass)...")
                if self._goto(wp):
                    self.get_logger().info(f"Waypoint {i+1} reached on retry.")
                else:
                    self.get_logger().warn(f"Waypoint {i+1} failed on retry too.")

            self.get_logger().info("All waypoints done.")
            if not self._loop:
                break
            self.get_logger().info("Looping back to first waypoint...")
            time.sleep(2.0)

        # Patrol complete (loop=false) → shut the node down so `ros2 launch` exits and
        # run_demo_tb4.sh proceeds to the final map-save/annotate PROMPTLY, instead of
        # lingering until the global timeout (extra minutes during which memory keeps
        # growing → OOM that starved SLAM before the map could be saved).
        self.get_logger().info("Patrol complete — shutting down for clean finalization.")
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _goto(self, wp: dict) -> bool:
        """Navigate to wp; on success, dwell (so apriltag can lock the wall tag) only
        if the waypoint asks for it. Staging waypoints set `dwell: false` — they only
        guide the robot doorway-by-doorway and must not waste time facing a wall."""
        if not self._send_goal(wp):
            return False
        if self._dwell > 0.0 and wp.get("dwell", True):
            if self._dwell_spin > 0.0:
                self.get_logger().info(
                    f"  inspect: spin {self._dwell:.0f}s @ {self._dwell_spin:.1f} rad/s "
                    f"(sweep the room walls for AprilTags)"
                )
                self._spin_in_place(self._dwell, self._dwell_spin)
            else:
                self.get_logger().info(f"  dwell {self._dwell:.0f}s (let apriltag lock the tag)")
                time.sleep(self._dwell)
        return True

    def _spin_in_place(self, duration: float, speed: float) -> None:
        """Rotate in place for `duration` s, then stop — sweeps the camera over the
        whole room so a wall AprilTag is seen whatever the approach heading was."""
        twist = Twist()
        twist.angular.z = speed
        end = time.time() + duration
        while time.time() < end:
            self._cmd_pub.publish(twist)
            time.sleep(0.1)
        self._cmd_pub.publish(Twist())  # stop

    def _send_goal(self, wp: dict) -> bool:
        sx, sy = self._snap(float(wp["x"]), float(wp["y"]))
        if abs(sx - float(wp["x"])) > 1e-3 or abs(sy - float(wp["y"])) > 1e-3:
            self.get_logger().info(
                f"  goal snapped ({wp['x']:.2f},{wp['y']:.2f}) → ({sx:.2f},{sy:.2f})"
            )
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose({"x": sx, "y": sy, "yaw": wp.get("yaw", 0.0)})

        future = self._client.send_goal_async(goal)

        # Block the background thread until the goal acceptance comes back.
        # The executor (MultiThreadedExecutor in main) continues processing
        # other callbacks concurrently, so action client callbacks are delivered.
        deadline = time.time() + 30.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().warn("Goal acceptance timed out (30s).")
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            return False

        result_future = goal_handle.get_result_async()

        deadline = time.time() + self._timeout
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.05)

        if not result_future.done():
            self.get_logger().warn(f"Goal timed out after {self._timeout}s.")
            goal_handle.cancel_goal_async()
            return False

        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _snap(self, x: float, y: float):
        """Return (x, y) snapped to the nearest free map cell (or unchanged)."""
        m = self._map
        if m is None or self._snap_radius <= 0 or m.info.resolution <= 0.0:
            return x, y
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        free = nearest_free_cell(
            m.data, m.info.width, m.info.height,
            (x - ox) / res, (y - oy) / res, self._snap_radius,
        )
        if free is None:
            return x, y
        fx, fy = free
        return ox + (fx + 0.5) * res, oy + (fy + 0.5) * res

    def _make_pose(self, wp: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp["x"])
        pose.pose.position.y = float(wp["y"])
        pose.pose.position.z = 0.0
        yaw = float(wp.get("yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args=None):
    # MultiThreadedExecutor keeps the executor free to deliver action-client
    # callbacks while the navigation loop runs in its background thread.
    run_node(WaypointFollowerNode, args=args, executor_factory=MultiThreadedExecutor)


if __name__ == "__main__":
    main()
