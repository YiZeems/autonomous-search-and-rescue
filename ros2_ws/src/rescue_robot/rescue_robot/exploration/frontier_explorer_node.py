from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from rescue_robot.utils.node_runner import run_node


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.get_logger().info('Frontier explorer skeleton started. Waiting for /map...')

    def map_callback(self, msg: OccupancyGrid):
        unknown = sum(1 for value in msg.data if value == -1)
        free = sum(1 for value in msg.data if value == 0)
        occupied = sum(1 for value in msg.data if value > 50)
        self.get_logger().info(
            f'Map received: free={free}, occupied={occupied}, unknown={unknown}',
            throttle_duration_sec=5.0,
        )
        # TODO(B): detect frontiers and send NavigateToPose goals to Nav2.


def main(args=None):
    run_node(FrontierExplorerNode, args=args)


if __name__ == '__main__':
    main()
