from rclpy.node import Node
from geometry_msgs.msg import PoseArray

from rescue_robot.utils.node_runner import run_node


class VictimDetectorNode(Node):
    """Victim detection scaffold — SUPERSEDED by victim_registry_node.

    The real victim pipeline is implemented in
    ``rescue_robot.detection.victim_registry_node`` (apriltag_ros → TF2
    projection camera→victim_<id>→map → /victims_map + results/victims.json,
    validated end-to-end at L16). This node is kept only as a lightweight test
    fixture: it publishes an empty PoseArray periodically so the downstream
    results/visualization modules can be exercised without the full perception
    stack (Gazebo + apriltag_ros) running.
    """

    def __init__(self):
        super().__init__('victim_detector_node')
        self.declare_parameter('output_topic', '/victims_map')
        self.declare_parameter('map_frame', 'map')
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(PoseArray, output_topic, 10)
        self.timer = self.create_timer(2.0, self.publish_empty)
        self.get_logger().info(
            'Victim detector scaffold (superseded by victim_registry_node) — '
            'publishing empty /victims_map for downstream tests only.'
        )

    def publish_empty(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').value
        self.publisher.publish(msg)


def main(args=None):
    run_node(VictimDetectorNode, args=args)


if __name__ == '__main__':
    main()
