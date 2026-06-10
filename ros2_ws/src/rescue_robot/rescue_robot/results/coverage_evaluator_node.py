from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

from rescue_robot.utils.node_runner import run_node


class CoverageEvaluatorNode(Node):
    """Compute a first-pass coverage ratio from an occupancy grid.

    MVP definition:
    - unknown cells are -1;
    - known cells are any value different from -1.

    Later improvement:
    - compare with a known explorable mask or exclude occupied walls from the
      denominator when the final world is fixed.
    """

    def __init__(self):
        super().__init__('coverage_evaluator_node')
        self.declare_parameter('exclude_occupied_from_denominator', False)
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.publisher = self.create_publisher(Float32, '/coverage', 10)
        self.get_logger().info('Coverage evaluator started. Waiting for /map...')

    def map_callback(self, msg: OccupancyGrid):
        exclude_occupied = bool(self.get_parameter('exclude_occupied_from_denominator').value)
        if exclude_occupied:
            candidate = [value for value in msg.data if value <= 50]
            known = sum(1 for value in candidate if value != -1)
            total = len(candidate)
        else:
            known = sum(1 for value in msg.data if value != -1)
            total = len(msg.data)
        coverage = float(known) / float(total) if total else 0.0
        out = Float32()
        out.data = coverage
        self.publisher.publish(out)
        self.get_logger().info(f'Coverage={coverage:.3f}', throttle_duration_sec=5.0)


def main(args=None):
    run_node(CoverageEvaluatorNode, args=args)


if __name__ == '__main__':
    main()
