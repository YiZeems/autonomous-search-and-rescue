from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

from rescue_robot.utils.node_runner import run_node

# SLAM toolbox publishes /map with Transient Local durability so late subscribers
# (including RViz2) receive the last map on connect.  Mock must match or RViz
# logs "incompatible QoS" and never renders the map.
_MAP_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class MockMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', _MAP_QOS)
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
    run_node(MockMapPublisher, args=args)


if __name__ == '__main__':
    main()
