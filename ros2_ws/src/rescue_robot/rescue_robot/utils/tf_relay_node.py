"""Relay a namespaced TF topic onto the global one.

TurtleBot4's nodes run inside the /turtlebot4 namespace and publish transforms to
*namespaced* topics:
  - dynamic  : /turtlebot4/tf         (diffdrive_controller odom->base_link)
  - static   : /turtlebot4/tf_static  (robot_state_publisher base_link->sensors)
SLAM Toolbox and RViz2 listen on the global /tf and /tf_static, so BOTH must be
relayed or the LaserScan MessageFilter can never resolve scan->odom (every scan is
dropped with "queue is full" and no map is built).

Parameters:
  src_topic (str)  : source topic            (default /turtlebot4/tf)
  dst_topic (str)  : destination topic       (default /tf)
  static    (bool) : use TRANSIENT_LOCAL QoS  (default false → VOLATILE, for /tf)

QoS:
  - dynamic /tf : VOLATILE subscriber — compatible with BOTH a VOLATILE publisher
    (CycloneDDS) and a TRANSIENT_LOCAL one (Fast-DDS). A TRANSIENT_LOCAL *subscriber*
    would reject CycloneDDS' VOLATILE /tf publisher ("incompatible QoS … DURABILITY").
  - static /tf_static : TRANSIENT_LOCAL on both sides (latched), matching RSP.

Run::

    ros2 run rescue_robot tf_relay_node                                   # /turtlebot4/tf -> /tf
    ros2 run rescue_robot tf_relay_node --ros-args \\
        -p src_topic:=/turtlebot4/tf_static -p dst_topic:=/tf_static -p static:=true
"""
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

from rescue_robot.utils.node_runner import run_node


def _qos(static: bool) -> QoSProfile:
    return QoSProfile(
        depth=200,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=(
            QoSDurabilityPolicy.TRANSIENT_LOCAL if static else QoSDurabilityPolicy.VOLATILE
        ),
    )


class TfRelayNode(Node):
    """Relay TFMessages from a namespaced topic to the global one."""

    def __init__(self) -> None:
        super().__init__("tf_relay_node")
        self.declare_parameter("src_topic", "/turtlebot4/tf")
        self.declare_parameter("dst_topic", "/tf")
        self.declare_parameter("static", False)

        src = self.get_parameter("src_topic").value
        dst = self.get_parameter("dst_topic").value
        static = bool(self.get_parameter("static").value)
        qos = _qos(static)

        self.pub = self.create_publisher(TFMessage, dst, qos)
        self.sub = self.create_subscription(TFMessage, src, self._relay, qos)
        self.get_logger().info(
            f"tf_relay_node: relaying {src} -> {dst} "
            f"({'static/TRANSIENT_LOCAL' if static else 'dynamic/VOLATILE'})"
        )

    def _relay(self, msg: TFMessage) -> None:
        self.pub.publish(msg)


def main(args=None) -> None:
    run_node(TfRelayNode, args=args)


if __name__ == "__main__":
    main()
