import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MockCoveragePublisher(Node):
    def __init__(self):
        super().__init__('mock_coverage_publisher')
        self.publisher = self.create_publisher(Float32, '/coverage', 10)
        self.timer = self.create_timer(1.0, self.publish_coverage)
        self.coverage = 0.05
        self.get_logger().info('Publishing mock /coverage')

    def publish_coverage(self):
        msg = Float32()
        self.coverage = min(0.95, self.coverage + 0.03)
        msg.data = float(self.coverage)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCoveragePublisher()
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
