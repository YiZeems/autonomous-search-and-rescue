from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

from rescue_robot.utils.node_runner import run_node


class RvizMarkerNode(Node):
    def __init__(self):
        super().__init__('rviz_marker_node')
        self.declare_parameter('marker_topic', '/visualization_marker_array')
        marker_topic = self.get_parameter('marker_topic').value
        self.subscription = self.create_subscription(
            PoseArray, '/victims_map', self.victims_callback, 10
        )
        self.publisher = self.create_publisher(MarkerArray, marker_topic, 10)
        self.get_logger().info('RViz marker node started. Waiting for /victims_map...')

    def victims_callback(self, msg: PoseArray):
        markers = MarkerArray()

        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, pose in enumerate(msg.poses):
            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = 'victims'
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            markers.markers.append(sphere)

            label = Marker()
            label.header = msg.header
            label.ns = 'victim_labels'
            label.id = index + 1000
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose
            label.pose.position.z += 0.45
            label.scale.z = 0.25
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'victim_{index + 1}'
            markers.markers.append(label)

        self.publisher.publish(markers)


def main(args=None):
    run_node(RvizMarkerNode, args=args)


if __name__ == '__main__':
    main()
