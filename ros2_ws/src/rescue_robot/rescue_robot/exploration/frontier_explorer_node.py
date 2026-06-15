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
import os

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
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
        # L17 bonus: exploration strategy (greedy | info_gain | size_dist).
        # IA712_EXPLORE_STRATEGY env overrides the param (used by the benchmark).
        self.declare_parameter("strategy", "info_gain")
        self.declare_parameter("info_gain_lambda", 1.0)
        self.declare_parameter("info_gain_radius_m", 1.5)
        # SpinAndScan: rotate in place after each reached goal so the OAK-D sweeps
        # the walls — frontier goals point the camera along travel, missing the
        # wall-mounted victim tags otherwise (fixes "explores but sees no victim").
        self.declare_parameter("spin_and_scan", True)
        self.declare_parameter("spin_scan_speed", 0.6)      # rad/s
        self.declare_parameter("spin_scan_duration", 7.0)   # s (~ one full turn)
        # Goal refinement (cf. codex §5): snap the goal to the nearest FREE cell so
        # Nav2 doesn't reject a centroid sitting in unknown/occupied space; skip IG
        # frontiers revealing fewer than info_gain_min_gain unknown cells.
        self.declare_parameter("goal_snap_radius", 8)       # cells
        self.declare_parameter("info_gain_min_gain", 0)
        # Reachability pre-check (cf. codex §5/§6): ask Nav2 ComputePathToPose for
        # a path BEFORE committing a NavigateToPose goal; if there is no path,
        # blacklist + re-pick immediately instead of wasting a ~60 s timeout.
        self.declare_parameter("precheck_reachable", True)

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._cov_stop = float(self.get_parameter("coverage_stop_threshold").value)
        self._min_size = int(self.get_parameter("min_frontier_size").value)
        self._quantum = float(self.get_parameter("blacklist_quantum_m").value)
        self._stall_repeats = int(self.get_parameter("stall_repeats").value)
        self._cov_eps = float(self.get_parameter("coverage_epsilon").value)
        self._max_clears = int(self.get_parameter("max_blacklist_clears").value)
        self._strategy = os.environ.get(
            "IA712_EXPLORE_STRATEGY", str(self.get_parameter("strategy").value)
        ).strip()
        self._ig_lambda = float(self.get_parameter("info_gain_lambda").value)
        self._ig_radius_m = float(self.get_parameter("info_gain_radius_m").value)
        self._spin_scan = bool(self.get_parameter("spin_and_scan").value)
        self._spin_speed = float(self.get_parameter("spin_scan_speed").value)
        self._spin_duration = float(self.get_parameter("spin_scan_duration").value)
        self._spin_ticks_left = 0
        self._snap_radius = int(self.get_parameter("goal_snap_radius").value)
        self._ig_min_gain = int(self.get_parameter("info_gain_min_gain").value)
        self._precheck = bool(self.get_parameter("precheck_reachable").value)

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
        self._path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        period = float(self.get_parameter("replan_period_sec").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Frontier explorer started — strategy={self._strategy} "
            f"(λ={self._ig_lambda}, r={self._ig_radius_m} m), spin_scan={self._spin_scan}, "
            f"stop at {self._cov_stop:.0%} coverage, min frontier size {self._min_size}."
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
            self._finish(f"Exploration complete: coverage {coverage:.1%} >= {self._cov_stop:.0%}.")
            return

        cells = fs.find_frontier_cells(msg.data, info.width, info.height)
        centroids = fs.cluster_centroids(fs.cluster_frontiers(cells), min_size=self._min_size)
        reachable = fs.filter_blacklisted(
            centroids, info.resolution, info.origin.position.x, info.origin.position.y,
            self._blacklist, self._quantum,
        )
        robot = self._robot_cell(info)
        radius_cells = max(1, int(round(self._ig_radius_m / info.resolution))) if info.resolution else 1
        goal = fs.select_frontier(
            self._strategy, reachable, robot,
            data=msg.data, width=info.width, height=info.height,
            radius_cells=radius_cells, lam=self._ig_lambda, min_size=self._min_size,
            min_gain=self._ig_min_gain,
        )

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
            self._finish(
                f"No reachable frontiers left (coverage {coverage:.1%}). Exploration finished."
            )
            return

        if not self._client.server_is_ready():
            self.get_logger().warn("Nav2 navigate_to_pose not ready yet; retrying...",
                                   throttle_duration_sec=5.0)
            return

        # Snap the chosen centroid to the nearest free cell so Nav2 accepts it.
        snap = fs.nearest_free_cell(msg.data, info.width, info.height,
                                    goal[0], goal[1], self._snap_radius)
        gx, gy = snap if snap is not None else (goal[0], goal[1])
        wx, wy = fs.cell_to_world(gx, gy, info.resolution,
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

        extra = ""
        if self._strategy == "info_gain":
            gain, cost, score = fs.infogain_score(
                goal, robot, msg.data, info.width, info.height,
                radius_cells, self._ig_lambda,
            )
            extra = f", gain={gain} cost={cost:.1f} score={score:.1f}"
        self.get_logger().info(
            f"Frontier goal [{self._strategy}] -> ({wx:.2f}, {wy:.2f})  "
            f"[cluster {goal[2]} cells{extra}, coverage {coverage:.1%}]"
        )
        self._go_to(wx, wy)

    def _pose(self, wx: float, wy: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.w = 1.0
        return pose

    def _go_to(self, wx: float, wy: float) -> None:
        self._navigating = True
        self._current_goal_world = (wx, wy)
        if self._precheck and self._path_client.server_is_ready():
            self._request_path(wx, wy)
        else:
            self._send_nav_goal(wx, wy)

    # -- reachability pre-check (ComputePathToPose) -------------------------------

    def _request_path(self, wx: float, wy: float) -> None:
        g = ComputePathToPose.Goal()
        g.goal = self._pose(wx, wy)
        g.use_start = False
        future = self._path_client.send_goal_async(g)
        future.add_done_callback(lambda f: self._on_path_response(f, wx, wy))

    def _on_path_response(self, future, wx: float, wy: float) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._send_nav_goal(wx, wy)        # can't pre-check -> just try it
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._on_path_result(f, wx, wy)
        )

    def _on_path_result(self, future, wx: float, wy: float) -> None:
        result = future.result()
        status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
        path = getattr(getattr(result, "result", None), "path", None)
        n = len(path.poses) if path is not None else 0
        if status != GoalStatus.STATUS_SUCCEEDED or n == 0:
            self.get_logger().info(
                f"Pre-check: no path to ({wx:.2f}, {wy:.2f}) -> blacklisted, re-picking."
            )
            self._blacklist_current_goal("no-path")
            self._navigating = False
            return
        self._send_nav_goal(wx, wy)

    # -- NavigateToPose ----------------------------------------------------------

    def _send_nav_goal(self, wx: float, wy: float) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(wx, wy)
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
            return
        # Reached the frontier. Optionally spin in place so the camera sweeps the
        # walls (victim tags) before picking the next goal. Keep _navigating=True
        # until the spin finishes so _tick doesn't grab a new goal mid-spin.
        self._current_goal_world = None
        if self._spin_scan:
            self._start_spin()
        else:
            self._navigating = False

    def _start_spin(self) -> None:
        self._spin_steps_left = max(1, int(self._spin_duration / 0.1))
        self._spin_timer = self.create_timer(0.1, self._spin_step)
        self.get_logger().info(
            f"SpinAndScan: rotating ~{self._spin_duration:.0f}s to sweep the camera."
        )

    def _spin_step(self) -> None:
        if self._spin_steps_left <= 0:
            self._cmd_pub.publish(Twist())          # stop
            self._spin_timer.cancel()
            self._navigating = False
            return
        twist = Twist()
        twist.angular.z = self._spin_speed
        self._cmd_pub.publish(twist)
        self._spin_steps_left -= 1

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

    def _finish(self, reason: str) -> None:
        """Exploration is over (coverage target reached or no frontiers left).

        Stop the robot, mark done and emit a clear log line. We deliberately do
        NOT call rclpy.shutdown() here: invoked from inside a timer callback it
        does not unblock rclpy.spin() (the process hangs). Instead the launcher
        (run_demo_tb4.sh, EXPLORE branch) watches for this 'EXPLORATION_DONE'
        marker, then stops the explorer and proceeds to the L18 finalization
        (map save + victim annotation)."""
        if self._done:
            return
        self._done = True
        try:
            self._cmd_pub.publish(Twist())  # stop any residual motion
        except Exception:
            pass
        self.get_logger().info(f"EXPLORATION_DONE — {reason}")


def main(args=None):
    run_node(FrontierExplorerNode, args=args)


if __name__ == "__main__":
    main()
