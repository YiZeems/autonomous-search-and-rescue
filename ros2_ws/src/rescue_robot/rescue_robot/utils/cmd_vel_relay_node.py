"""Relay /cmd_vel -> /turtlebot4/diffdrive_controller/cmd_vel_unstamped.

Nav2's controller_server publishes Twist on /cmd_vel.  TurtleBot4's
diffdrive_controller runs in the /turtlebot4 namespace and subscribes to
cmd_vel_unstamped (Twist, not TwistStamped).  A direct relay bridges the gap
without touching either node's configuration.

Run alongside the TB4 simulation::

    ros2 run rescue_robot cmd_vel_relay_node
"""
from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rescue_robot.utils.node_runner import run_node
from rescue_robot.utils.topic_relay import TopicRelay

_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class CmdVelRelayNode(TopicRelay):
    """Relay Nav2 /cmd_vel to the namespaced diffdrive controller input."""

    def __init__(self) -> None:
        super().__init__(
            "cmd_vel_relay_node",
            Twist,
            "/cmd_vel",
            "/turtlebot4/diffdrive_controller/cmd_vel_unstamped",
            sub_qos=_QOS,
            pub_qos=_QOS,
            description=(
                "cmd_vel_relay_node: /cmd_vel -> "
                "/turtlebot4/diffdrive_controller/cmd_vel_unstamped"
            ),
        )


def main(args=None) -> None:
    run_node(CmdVelRelayNode, args=args)


if __name__ == "__main__":
    main()
