import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MockMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.width = 40
        self.height = 40
        self.resolution = 0.2
        self.get_logger().info('Publishing mock /map')

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = -4.0
        msg.info.origin.position.y = -4.0
        msg.info.origin.orientation.w = 1.0

        data = []
        for y in range(self.height):
            for x in range(self.width):
                if x in (0, self.width - 1) or y in (0, self.height - 1):
                    data.append(100)
                elif 12 < x < 28 and 12 < y < 28:
                    data.append(0)
                elif 5 < x < 16 and 5 < y < 16:
                    data.append(0)
                else:
                    data.append(-1)
        msg.data = data
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockMapPublisher()
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
