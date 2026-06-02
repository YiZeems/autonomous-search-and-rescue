"""Relay /cmd_vel → /turtlebot4/diffdrive_controller/cmd_vel_unstamped.

Nav2 controller_server publishes Twist on /cmd_vel.  TurtleBot4's
diffdrive_controller runs in the /turtlebot4 namespace and subscribes to
cmd_vel_unstamped (Twist, not TwistStamped).  A direct relay bridges the gap
without touching either node's configuration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist

_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class CmdVelRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_relay_node')
        self.pub = self.create_publisher(
            Twist, '/turtlebot4/diffdrive_controller/cmd_vel_unstamped', _QOS
        )
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._relay, _QOS)
        self.get_logger().info(
            'cmd_vel_relay_node: /cmd_vel → /turtlebot4/diffdrive_controller/cmd_vel_unstamped'
        )

    def _relay(self, msg: Twist) -> None:
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelayNode()
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
