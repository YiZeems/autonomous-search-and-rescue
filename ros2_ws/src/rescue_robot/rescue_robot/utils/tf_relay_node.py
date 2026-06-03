"""Relay /turtlebot4/tf -> /tf (global).

TurtleBot4's diffdrive_controller runs inside the /turtlebot4 namespace and
publishes transforms to /turtlebot4/tf instead of the global /tf topic.
SLAM Toolbox and RViz2 listen on /tf, so the LaserScan MessageFilter fails.

QoS: subscriber uses RELIABLE + TRANSIENT_LOCAL to match diffdrive_controller's
publisher QoS exactly (Fast-DDS enforces durability matching).  Publisher uses
RELIABLE + VOLATILE to match tf2_ros::TransformListener on /tf.

Run alongside the TB4 simulation (headless ARM64 / sequenced launch)::

    ros2 run rescue_robot tf_relay_node
"""
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

from rescue_robot.utils.node_runner import run_node
from rescue_robot.utils.topic_relay import TopicRelay

# Match diffdrive_controller's exact QoS: RELIABLE + TRANSIENT_LOCAL
# (Fast-DDS requires durability to match for TRANSIENT_LOCAL publishers).
_SUB_QOS = QoSProfile(
    depth=100,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# RELIABLE + VOLATILE is what tf2_ros::TransformListener expects on /tf.
_PUB_QOS = QoSProfile(
    depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class TfRelayNode(TopicRelay):
    """Relay transforms from /turtlebot4/tf to the global /tf topic."""

    def __init__(self) -> None:
        super().__init__(
            "tf_relay_node",
            TFMessage,
            "/turtlebot4/tf",
            "/tf",
            sub_qos=_SUB_QOS,
            pub_qos=_PUB_QOS,
            description="tf_relay_node: relaying /turtlebot4/tf -> /tf",
        )


def main(args=None) -> None:
    run_node(TfRelayNode, args=args)


if __name__ == "__main__":
    main()
