"""Autonomous frontier exploration.

Subscribes to /map, finds the boundary between explored and unexplored space
(frontiers), and repeatedly sends the best frontier as a NavigateToPose goal so
the robot explores the environment on its own — the "search" half of search and
rescue. Stops when coverage reaches the threshold or no frontiers remain.

The frontier maths lives in frontier_search.py (pure, unit-tested). This node
only does the ROS plumbing: map subscription, robot pose via TF, and the Nav2
action client (fully callback-driven, so it never blocks the executor).

Frontier blacklisting (cf. CM8 "Inaccessible Frontiers" / "blacklist"): a
frontier is blacklisted — and never re-selected — when either
  * Nav2 returns a non-SUCCEEDED result for it (planner/controller can't reach
    it: seen-through-a-window, behind inflation, etc.), or
  * the same frontier is re-selected ``stall_repeats`` times without coverage
    improving by ``coverage_epsilon`` (a goal Nav2 reports done but that doesn't
    actually grow the map).
Without this, the explorer loops forever on one unreachable frontier and
coverage plateaus (observed: stuck at 53.8% re-sending the same goal).

    ros2 run rescue_robot frontier_explorer_node
"""
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

from rescue_robot.exploration import frontier_search as fs
from rescue_robot.utils.node_runner import run_node


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__("frontier_explorer_node")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("coverage_stop_threshold", 0.90)
        self.declare_parameter("min_frontier_size", 5)
        self.declare_parameter("replan_period_sec", 3.0)
        # Blacklist tuning (cf. CM8).
        self.declare_parameter("blacklist_quantum_m", 0.5)
        self.declare_parameter("stall_repeats", 3)
        self.declare_parameter("coverage_epsilon", 0.005)
        self.declare_parameter("max_blacklist_clears", 3)

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._cov_stop = float(self.get_parameter("coverage_stop_threshold").value)
        self._min_size = int(self.get_parameter("min_frontier_size").value)
        self._quantum = float(self.get_parameter("blacklist_quantum_m").value)
        self._stall_repeats = int(self.get_parameter("stall_repeats").value)
        self._cov_eps = float(self.get_parameter("coverage_epsilon").value)
        self._max_clears = int(self.get_parameter("max_blacklist_clears").value)

        self._map: OccupancyGrid | None = None
        self._navigating = False
        self._done = False

        # Blacklist state (world-quantised keys).
        self._blacklist: set[tuple[int, int]] = set()
        self._current_goal_world: tuple[float, float] | None = None
        self._last_goal_key: tuple[int, int] | None = None
        self._same_goal_count = 0
        self._coverage_at_last_goal = 0.0
        self._blacklist_clears = 0

        self.subscription = self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        period = float(self.get_parameter("replan_period_sec").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Frontier explorer started — stop at {self._cov_stop:.0%} coverage, "
            f"min frontier size {self._min_size}, blacklist quantum {self._quantum} m."
        )

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _coverage(self, data) -> float:
        total = len(data)
        if not total:
            return 0.0
        known = sum(1 for v in data if v != fs.UNKNOWN)
        return known / total

    def _robot_cell(self, info) -> tuple[float, float]:
        """Robot position in grid cells, falling back to the map centre."""
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
            wx = tf.transform.translation.x
            wy = tf.transform.translation.y
            cx = (wx - info.origin.position.x) / info.resolution
            cy = (wy - info.origin.position.y) / info.resolution
            return cx, cy
        except Exception:  # noqa: BLE001 — TF may not be ready yet
            return info.width / 2.0, info.height / 2.0

    def _tick(self) -> None:
        if self._done or self._navigating or self._map is None:
            return
        msg = self._map
        info = msg.info

        coverage = self._coverage(msg.data)
        if coverage >= self._cov_stop:
            self.get_logger().info(
                f"Exploration complete: coverage {coverage:.1%} >= {self._cov_stop:.0%}."
            )
            self._done = True
            return

        cells = fs.find_frontier_cells(msg.data, info.width, info.height)
        centroids = fs.cluster_centroids(fs.cluster_frontiers(cells), min_size=self._min_size)
        reachable = fs.filter_blacklisted(
            centroids, info.resolution, info.origin.position.x, info.origin.position.y,
            self._blacklist, self._quantum,
        )
        goal = fs.choose_frontier(reachable, self._robot_cell(info), min_size=self._min_size)

        if goal is None:
            # Every frontier is either too small or blacklisted. Frontiers may
            # become reachable as the map grows, so clear the blacklist and retry
            # a few times before declaring the map done.
            if self._blacklist and self._blacklist_clears < self._max_clears:
                self._blacklist_clears += 1
                self.get_logger().warn(
                    f"All {len(self._blacklist)} frontiers blacklisted at coverage "
                    f"{coverage:.1%} — clearing blacklist (attempt "
                    f"{self._blacklist_clears}/{self._max_clears})."
                )
                self._blacklist.clear()
                return
            self.get_logger().info(
                f"No reachable frontiers left (coverage {coverage:.1%}). Exploration finished."
            )
            self._done = True
            return

        if not self._client.server_is_ready():
            self.get_logger().warn("Nav2 navigate_to_pose not ready yet; retrying...",
                                   throttle_duration_sec=5.0)
            return

        wx, wy = fs.cell_to_world(goal[0], goal[1], info.resolution,
                                  info.origin.position.x, info.origin.position.y)
        key = fs.blacklist_key(wx, wy, self._quantum)

        # Stall detection: same goal re-chosen with no coverage gain -> blacklist.
        if key == self._last_goal_key and (coverage - self._coverage_at_last_goal) < self._cov_eps:
            self._same_goal_count += 1
            if self._same_goal_count >= self._stall_repeats:
                self._blacklist.add(key)
                self.get_logger().warn(
                    f"Frontier ({wx:.2f}, {wy:.2f}) stalled "
                    f"({self._same_goal_count}x, coverage flat at {coverage:.1%}) "
                    f"-> blacklisted. {len(self._blacklist)} frontier(s) blacklisted."
                )
                self._same_goal_count = 0
                self._last_goal_key = None
                return  # re-pick a different frontier next tick
        else:
            self._same_goal_count = 0
            self._last_goal_key = key
            self._coverage_at_last_goal = coverage

        self.get_logger().info(
            f"Frontier goal -> ({wx:.2f}, {wy:.2f})  [cluster {goal[2]} cells, "
            f"coverage {coverage:.1%}]"
        )
        self._send_goal(wx, wy)

    def _send_goal(self, wx: float, wy: float) -> None:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        self._navigating = True
        self._current_goal_world = (wx, wy)
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("Frontier goal rejected by Nav2.")
            self._blacklist_current_goal("rejected")
            self._navigating = False
            return
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        # Blacklist any frontier Nav2 could not actually reach (ABORTED/CANCELED).
        status = getattr(future.result(), "status", GoalStatus.STATUS_UNKNOWN)
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._blacklist_current_goal(f"status={status}")
        self._navigating = False

    def _blacklist_current_goal(self, reason: str) -> None:
        if self._current_goal_world is None:
            return
        wx, wy = self._current_goal_world
        key = fs.blacklist_key(wx, wy, self._quantum)
        if key not in self._blacklist:
            self._blacklist.add(key)
            self.get_logger().warn(
                f"Frontier ({wx:.2f}, {wy:.2f}) unreachable ({reason}) -> blacklisted. "
                f"{len(self._blacklist)} frontier(s) blacklisted."
            )
        self._current_goal_world = None


def main(args=None):
    run_node(FrontierExplorerNode, args=args)


if __name__ == "__main__":
    main()
