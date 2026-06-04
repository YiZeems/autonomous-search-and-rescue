"""Autonomous frontier exploration.

Subscribes to /map, finds the boundary between explored and unexplored space
(frontiers), and repeatedly sends the best frontier as a NavigateToPose goal so
the robot explores the environment on its own — the "search" half of search and
rescue. Stops when coverage reaches the threshold or no frontiers remain.

The frontier maths lives in frontier_search.py (pure, unit-tested). This node
only does the ROS plumbing: map subscription, robot pose via TF, and the Nav2
action client (fully callback-driven, so it never blocks the executor).

    ros2 run rescue_robot frontier_explorer_node
"""
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

        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._cov_stop = float(self.get_parameter("coverage_stop_threshold").value)
        self._min_size = int(self.get_parameter("min_frontier_size").value)

        self._map: OccupancyGrid | None = None
        self._navigating = False
        self._done = False

        self.subscription = self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        period = float(self.get_parameter("replan_period_sec").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Frontier explorer started — stop at {self._cov_stop:.0%} coverage, "
            f"min frontier size {self._min_size}."
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
        goal = fs.choose_frontier(centroids, self._robot_cell(info), min_size=self._min_size)
        if goal is None:
            self.get_logger().info(
                f"No frontiers left (coverage {coverage:.1%}). Exploration finished."
            )
            self._done = True
            return

        if not self._client.server_is_ready():
            self.get_logger().warn("Nav2 navigate_to_pose not ready yet; retrying...",
                                   throttle_duration_sec=5.0)
            return

        wx, wy = fs.cell_to_world(goal[0], goal[1], info.resolution,
                                  info.origin.position.x, info.origin.position.y)
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
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("Frontier goal rejected by Nav2.")
            self._navigating = False
            return
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, _future) -> None:
        # Whatever the outcome (reached / aborted), free up for the next frontier.
        self._navigating = False


def main(args=None):
    run_node(FrontierExplorerNode, args=args)


if __name__ == "__main__":
    main()
