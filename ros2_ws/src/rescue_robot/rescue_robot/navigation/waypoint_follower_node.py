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
import time

import rclpy
import rclpy.clock
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower_node")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("loop", False)
        self.declare_parameter("goal_timeout_sec", 60.0)

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._loop = self.get_parameter("loop").value
        self._timeout = self.get_parameter("goal_timeout_sec").value

        waypoints_file = self.get_parameter("waypoints_file").value
        self._waypoints = self._load_waypoints(waypoints_file)

        self.get_logger().info(
            f"WaypointFollower ready — {len(self._waypoints)} waypoints, "
            f"loop={self._loop}, timeout={self._timeout}s"
        )

        # Start after Nav2 is ready — wall-clock timer so it fires regardless
        # of whether use_sim_time is enabled (sim clock may be paused at startup).
        self._start_timer = self.create_timer(
            1.0, self._start_once,
            clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME),
        )
        self._started = False

    def _load_waypoints(self, path: str) -> list:
        if not path:
            # Fallback: predefined turtlebot3_house coverage path (map frame)
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

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self.destroy_timer(self._start_timer)
        self._run()

    def _run(self):
        if not self._waypoints:
            self.get_logger().warn("No waypoints to follow.")
            return

        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("NavigateToPose server not available after 30s. Aborting.")
            return

        while True:
            for i, wp in enumerate(self._waypoints):
                self.get_logger().info(
                    f"Navigating to waypoint {i+1}/{len(self._waypoints)}: "
                    f"x={wp['x']:.2f} y={wp['y']:.2f} yaw={wp.get('yaw', 0.0):.2f}"
                )
                success = self._send_goal(wp)
                if success:
                    self.get_logger().info(f"Waypoint {i+1} reached.")
                else:
                    self.get_logger().warn(f"Waypoint {i+1} failed or timed out — continuing.")

            self.get_logger().info("All waypoints done.")
            if not self._loop:
                break
            self.get_logger().info("Looping back to first waypoint...")
            time.sleep(2.0)

    def _send_goal(self, wp: dict) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(wp)

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self._timeout)

        if not result_future.done():
            self.get_logger().warn(f"Goal timed out after {self._timeout}s.")
            goal_handle.cancel_goal_async()
            return False

        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

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
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
