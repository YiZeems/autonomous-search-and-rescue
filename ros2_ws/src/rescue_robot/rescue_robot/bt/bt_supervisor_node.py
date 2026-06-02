import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class BtSupervisorNode(Node):
    """Lightweight project-level Behavior Tree supervisor scaffold.

    The final implementation can be replaced by BehaviorTree.CPP integration or
    by a richer ROS 2 supervisor. The scaffold is intentionally connected to a
    real condition (/coverage) so the BT requirement is not purely decorative.
    """

    def __init__(self):
        super().__init__('bt_supervisor_node')
        self.declare_parameter('coverage_threshold', 0.90)
        self.coverage = 0.0
        self.create_subscription(Float32, '/coverage', self.coverage_callback, 10)
        self.timer = self.create_timer(2.0, self.tick)
        self.get_logger().info('BT supervisor scaffold started. Monitoring /coverage.')

    def coverage_callback(self, msg: Float32):
        self.coverage = float(msg.data)

    def tick(self):
        threshold = float(self.get_parameter('coverage_threshold').value)
        if self.coverage >= threshold:
            self.get_logger().info(
                f'BT condition reached: coverage >= {threshold:.2f}. Ready to export/finalize.',
                throttle_duration_sec=10.0,
            )
        else:
            self.get_logger().info(
                f'BT tick: coverage={self.coverage:.3f}', throttle_duration_sec=10.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = BtSupervisorNode()
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
