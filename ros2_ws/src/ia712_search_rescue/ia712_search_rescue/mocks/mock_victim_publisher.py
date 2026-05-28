import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


class MockVictimPublisher(Node):
    def __init__(self):
        super().__init__('mock_victim_publisher')
        self.publisher = self.create_publisher(PoseArray, '/victims_map', 10)
        self.timer = self.create_timer(1.0, self.publish_victims)
        self.get_logger().info('Publishing mock /victims_map')

    def publish_victims(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in [(1.0, 1.5), (-1.2, 2.2), (2.4, -0.8)]:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockVictimPublisher()
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
