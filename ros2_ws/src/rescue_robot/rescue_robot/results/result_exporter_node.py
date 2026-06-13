import csv
import json
import math
import os
from pathlib import Path

from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from rescue_robot.utils.node_runner import run_node


class ResultExporterNode(Node):
    """Export run results + exploration metrics from standard project topics.

    Inputs:
    - /coverage      (std_msgs/Float32, [0, 1])
    - /victims_map   (geometry_msgs/PoseArray, map frame)
    - <odom_topic>   (nav_msgs/Odometry) — integrated into total path length

    Outputs (under ``output_dir``, env IA712_RESULTS_DIR overrides the param so the
    L17 benchmark can point each run at experiments/<algo>_run<n>/):
    - coverage_over_time.csv : time, coverage, path_length_m
    - victims_detected.csv   : id, x, y
    - run_summary.json       : final_coverage, victims_detected, success_coverage_90,
                               path_length_m, duration_s, time_to_50/75/90_s, strategy
    These are exactly the metrics the bonus comparison (greedy vs info-gain) needs.
    """

    def __init__(self):
        super().__init__('result_exporter_node')
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('odom_topic', '/turtlebot4/odom')
        out = os.environ.get('IA712_RESULTS_DIR', str(self.get_parameter('output_dir').value))
        self.results_dir = Path(out)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.coverage_path = self.results_dir / 'coverage_over_time.csv'
        self.victims_path = self.results_dir / 'victims_detected.csv'
        self.summary_path = self.results_dir / 'run_summary.json'

        self.strategy = os.environ.get('IA712_EXPLORE_STRATEGY', 'default')
        self.coverage = 0.0
        self.victims = []
        self.path_length = 0.0
        self._last_xy = None
        self._t0 = None                       # sim time of first coverage sample
        self._time_to = {0.5: None, 0.75: None, 0.90: None}

        self._ensure_headers()
        self.create_subscription(Float32, '/coverage', self.coverage_callback, 10)
        self.create_subscription(PoseArray, '/victims_map', self.victims_callback, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.odom_callback, 10
        )
        self.timer = self.create_timer(5.0, self.export_summary)
        self.get_logger().info(
            f"Result exporter started (strategy={self.strategy}). Writing to {self.results_dir}/."
        )

    # -- helpers ----------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _ensure_headers(self):
        if not self.coverage_path.exists() or self.coverage_path.stat().st_size == 0:
            with self.coverage_path.open('w', newline='') as f:
                csv.writer(f).writerow(['time', 'coverage', 'path_length_m'])
        if not self.victims_path.exists() or self.victims_path.stat().st_size == 0:
            with self.victims_path.open('w', newline='') as f:
                csv.writer(f).writerow(['id', 'x', 'y'])

    # -- callbacks --------------------------------------------------------------

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_xy is not None:
            self.path_length += math.dist(self._last_xy, (x, y))
        self._last_xy = (x, y)

    def coverage_callback(self, msg: Float32):
        self.coverage = float(msg.data)
        now = self._now()
        if self._t0 is None:
            self._t0 = now
        elapsed = now - self._t0
        for thr in self._time_to:
            if self._time_to[thr] is None and self.coverage >= thr:
                self._time_to[thr] = round(elapsed, 1)
        with self.coverage_path.open('a', newline='') as f:
            csv.writer(f).writerow([round(elapsed, 2), round(self.coverage, 4),
                                    round(self.path_length, 3)])

    def victims_callback(self, msg: PoseArray):
        self.victims = [(p.position.x, p.position.y) for p in msg.poses]
        with self.victims_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['id', 'x', 'y'])
            for i, (x, y) in enumerate(self.victims, start=1):
                writer.writerow([f'victim_{i}', x, y])

    # -- summary ----------------------------------------------------------------

    def export_summary(self):
        duration = (self._now() - self._t0) if self._t0 is not None else 0.0
        summary = {
            'strategy': self.strategy,
            'final_coverage': round(self.coverage, 4),
            'victims_detected': len(self.victims),
            'success_coverage_90': self.coverage >= 0.90,
            'path_length_m': round(self.path_length, 3),
            'duration_s': round(duration, 1),
            'time_to_50_s': self._time_to[0.5],
            'time_to_75_s': self._time_to[0.75],
            'time_to_90_s': self._time_to[0.90],
        }
        with self.summary_path.open('w') as f:
            json.dump(summary, f, indent=2)
            f.write('\n')


def main(args=None):
    # Flush a final run_summary.json before the node is destroyed.
    run_node(ResultExporterNode, args=args, on_shutdown=lambda node: node.export_summary())


if __name__ == '__main__':
    main()
