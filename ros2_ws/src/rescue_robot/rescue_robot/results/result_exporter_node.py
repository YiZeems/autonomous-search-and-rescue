import csv
import json
from pathlib import Path

from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32

from rescue_robot.utils.node_runner import run_node


class ResultExporterNode(Node):
    """Export run results from standard project topics.

    Inputs:
    - /coverage: std_msgs/Float32 in [0, 1]
    - /victims_map: geometry_msgs/PoseArray in map frame

    Outputs are written under the repository-level results/ directory when launched
    from the repository root. During final integration, A may replace this with an
    absolute configured output directory.
    """

    def __init__(self):
        super().__init__('result_exporter_node')
        self.declare_parameter('output_dir', 'results')
        self.results_dir = Path(str(self.get_parameter('output_dir').value))
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.coverage_path = self.results_dir / 'coverage_over_time.csv'
        self.victims_path = self.results_dir / 'victims_detected.csv'
        self.summary_path = self.results_dir / 'run_summary.json'
        self.coverage = 0.0
        self.victims = []
        self._ensure_headers()
        self.create_subscription(Float32, '/coverage', self.coverage_callback, 10)
        self.create_subscription(PoseArray, '/victims_map', self.victims_callback, 10)
        self.timer = self.create_timer(5.0, self.export_summary)
        self.get_logger().info('Result exporter started. Writing to results/.')

    def _ensure_headers(self):
        if not self.coverage_path.exists() or self.coverage_path.stat().st_size == 0:
            with self.coverage_path.open('w', newline='') as f:
                csv.writer(f).writerow(['time', 'coverage'])
        if not self.victims_path.exists() or self.victims_path.stat().st_size == 0:
            with self.victims_path.open('w', newline='') as f:
                csv.writer(f).writerow(['id', 'x', 'y'])

    def coverage_callback(self, msg: Float32):
        self.coverage = float(msg.data)
        with self.coverage_path.open('a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.get_clock().now().nanoseconds / 1e9, self.coverage])

    def victims_callback(self, msg: PoseArray):
        self.victims = [(p.position.x, p.position.y) for p in msg.poses]
        with self.victims_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['id', 'x', 'y'])
            for i, (x, y) in enumerate(self.victims, start=1):
                writer.writerow([f'victim_{i}', x, y])

    def export_summary(self):
        summary = {
            'final_coverage': self.coverage,
            'victims_detected': len(self.victims),
            'success_coverage_90': self.coverage >= 0.90,
        }
        with self.summary_path.open('w') as f:
            json.dump(summary, f, indent=2)
            f.write('\n')


def main(args=None):
    # Flush a final run_summary.json before the node is destroyed.
    run_node(ResultExporterNode, args=args, on_shutdown=lambda node: node.export_summary())


if __name__ == '__main__':
    main()
