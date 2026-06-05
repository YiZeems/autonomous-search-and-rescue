"""Relay Nav2's /cmd_vel onto the TurtleBot4/create3 velocity input.

Nav2's controller_server publishes Twist on /cmd_vel. The create3 base (driven by
ign_ros2_control) takes its velocity command on the namespaced topic
**/turtlebot4/cmd_vel** — that goes through the create3 motion_control/safety layer
which then commands the diffdrive_controller. Publishing straight to the
diffdrive_controller's cmd_vel_unstamped bypasses motion_control, which keeps
sending zero, so the robot never moves. Relaying to /turtlebot4/cmd_vel instead
lets Nav2 actually drive the robot.

Parameters:
  src_topic (str) : default /cmd_vel
  dst_topic (str) : default /turtlebot4/cmd_vel

Run::

    ros2 run rescue_robot cmd_vel_relay_node
    ros2 run rescue_robot cmd_vel_relay_node --ros-args -p dst_topic:=/turtlebot4/cmd_vel
"""
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rescue_robot.utils.node_runner import run_node

_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class CmdVelRelayNode(Node):
    """Relay Nav2 /cmd_vel to the create3 velocity input."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_relay_node")
        self.declare_parameter("src_topic", "/cmd_vel")
        self.declare_parameter("dst_topic", "/turtlebot4/cmd_vel")
        src = self.get_parameter("src_topic").value
        dst = self.get_parameter("dst_topic").value
        self.pub = self.create_publisher(Twist, dst, _QOS)
        self.sub = self.create_subscription(Twist, src, self._relay, _QOS)
        self.get_logger().info(f"cmd_vel_relay_node: {src} -> {dst}")

    def _relay(self, msg: Twist) -> None:
        self.pub.publish(msg)


def main(args=None) -> None:
    run_node(CmdVelRelayNode, args=args)


if __name__ == "__main__":
    main()
