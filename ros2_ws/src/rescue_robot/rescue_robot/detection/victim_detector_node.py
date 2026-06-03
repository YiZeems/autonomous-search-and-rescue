from rclpy.node import Node
from geometry_msgs.msg import PoseArray

from rescue_robot.utils.node_runner import run_node


class VictimDetectorNode(Node):
    """Victim detection scaffold.

    MVP plan:
    - subscribe to /camera/image_raw and /camera/camera_info;
    - detect a colored cylinder or ArUco/AprilTag marker;
    - estimate pose in the camera frame;
    - transform camera_frame -> map with tf2;
    - publish /victims_map as PoseArray.

    Current scaffold publishes an empty PoseArray periodically so downstream
    results/visualization modules can be tested before the detector is ready.
    """

    def __init__(self):
        super().__init__('victim_detector_node')
        self.declare_parameter('output_topic', '/victims_map')
        self.declare_parameter('map_frame', 'map')
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(PoseArray, output_topic, 10)
        self.timer = self.create_timer(2.0, self.publish_empty)
        self.get_logger().info(
            'Victim detector scaffold started. TODO: implement camera + tf2 detection pipeline.'
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
