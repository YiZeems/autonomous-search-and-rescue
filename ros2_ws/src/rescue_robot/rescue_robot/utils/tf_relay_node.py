"""Relay /turtlebot4/tf → /tf (global).

TurtleBot4's diffdrive_controller runs inside the /turtlebot4 namespace and
publishes transforms to /turtlebot4/tf instead of the global /tf topic.
SLAM Toolbox and RViz2 listen on /tf, so LaserScan MessageFilter fails.

QoS chosen deliberately:
  subscriber  = BEST_EFFORT — accepts both VOLATILE and TRANSIENT_LOCAL
                              publishers without triggering Fast-DDS
                              INCOMPATIBLE_QOS (RMW pitfall: RELIABLE sub +
                              BEST_EFFORT pub = INCOMPATIBLE in Fast-DDS,
                              the reverse of the DDS spec).
  publisher   = RELIABLE + VOLATILE — matches the QoS profile expected by
                              SLAM Toolbox and tf2_ros::TransformListener.

Run alongside the TB4 simulation when using headless ARM64 / sequenced launch:
    ros2 run rescue_robot tf_relay_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


# BEST_EFFORT subscriber avoids Fast-DDS INCOMPATIBLE_QOS when the upstream
# diffdrive_controller uses VOLATILE/BEST_EFFORT.
_SUB_QOS = QoSProfile(
    depth=100,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# RELIABLE+VOLATILE is what tf2_ros::TransformListener expects on /tf.
_PUB_QOS = QoSProfile(
    depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class TfRelayNode(Node):
    """Relay transforms from /turtlebot4/tf to the global /tf topic."""

    def __init__(self) -> None:
        super().__init__('tf_relay_node')
        self.pub = self.create_publisher(TFMessage, '/tf', _PUB_QOS)
        self.sub = self.create_subscription(
            TFMessage, '/turtlebot4/tf', self._relay, _SUB_QOS
        )
        self.get_logger().info(
            'tf_relay_node: relaying /turtlebot4/tf → /tf'
        )

    def _relay(self, msg: TFMessage) -> None:
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
