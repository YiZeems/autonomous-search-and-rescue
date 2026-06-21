import csv
import json
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from rescue_robot.utils.node_runner import run_node


class ResultExporterNode(Node):
    """Export run results + exploration metrics from standard project topics.

    Inputs:
    - /coverage      (std_msgs/Float32, [0, 1])
    - /victims_map   (geometry_msgs/PoseArray, map frame)
    - <odom_topic>   (nav_msgs/Odometry) — integrated into total path length

    Outputs (under ``output_dir``, env IA712_RESULTS_DIR overrides the param so the
    the benchmark can point each run at experiments/<algo>_run<n>/):
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
        # Frames for the MAP-frame robot pose (the trajectory plotted on the map must be
        # in 'map', not 'odom' — odom drifts vs map as SLAM corrects, which made the path
        # look like it crossed walls). base_frame is namespaced on the TB4.
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'turtlebot4/base_link')
        out = os.environ.get('IA712_RESULTS_DIR', str(self.get_parameter('output_dir').value))
        self.results_dir = Path(out)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.coverage_path = self.results_dir / 'coverage_over_time.csv'
        self.victims_path = self.results_dir / 'victims_detected.csv'
        self.victims_timeline_path = self.results_dir / 'victims_over_time.csv'
        self.trajectory_path = self.results_dir / 'trajectory.csv'   # dense 2 Hz map-frame path
        self.summary_path = self.results_dir / 'run_summary.json'

        self.strategy = os.environ.get('IA712_EXPLORE_STRATEGY', 'default')
        self.coverage = 0.0
        self.victims = []
        self.path_length = 0.0
        self._last_xy = None
        self._t0 = None                       # sim time of first coverage sample
        self._time_to = {0.5: None, 0.75: None, 0.90: None}
        self._victim_count = 0

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._ensure_headers()
        self.create_subscription(Float32, '/coverage', self.coverage_callback, 10)
        self.create_subscription(PoseArray, '/victims_map', self.victims_callback, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.odom_callback, 10
        )
        self.timer = self.create_timer(5.0, self.export_summary)
        self.create_timer(0.5, self._log_trajectory)   # dense, smooth map-frame trajectory
        self.get_logger().info(
            f"Result exporter started (strategy={self.strategy}). Writing to {self.results_dir}/."
        )

    # -- helpers ----------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _ensure_headers(self):
        # These are PER-RUN time-series logs (one node instance = one mission), appended
        # to as the run progresses. They MUST start fresh each run: truncate + write the
        # header unconditionally. Otherwise a leftover CSV from a previous run keeps its
        # rows and the new run appends to them → the report figures show several runs'
        # curves/trajectories superimposed (the "courbes qui se cumulent" bug).
        # robot_x/robot_y are logged so the figures can draw the real path taken.
        for path, hdr in (
            (self.coverage_path, ['time', 'coverage', 'path_length_m', 'robot_x', 'robot_y']),
            # victims_over_time.csv: (sim time, cumulative count) → the replay reveals each
            # victim at the moment it was actually detected, not a guessed threshold.
            (self.victims_timeline_path, ['time', 'count']),
            (self.trajectory_path, ['time', 'x', 'y']),          # dense 2 Hz map-frame path
        ):
            with path.open('w', newline='') as f:
                csv.writer(f).writerow(hdr)
        # victims.json's CSV twin is a final snapshot (rewritten wholesale on each update),
        # so it only needs a header when missing/empty — no per-run truncation required.
        if not self.victims_path.exists() or self.victims_path.stat().st_size == 0:
            with self.victims_path.open('w', newline='') as f:
                csv.writer(f).writerow(['id', 'x', 'y'])

    # -- callbacks --------------------------------------------------------------

    def _log_trajectory(self):
        mxy = self._map_xy()
        if mxy is None or self._t0 is None:
            return
        t = self._now() - self._t0
        with self.trajectory_path.open('a', newline='') as f:
            csv.writer(f).writerow([round(t, 2), round(mxy[0], 3), round(mxy[1], 3)])

    def _map_xy(self):
        """Robot (x, y) in the MAP frame via TF (map → base). None if TF not ready yet."""
        try:
            tf = self._tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

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
        # MAP-frame pose for the plotted trajectory (fall back to odom if TF not ready).
        mxy = self._map_xy()
        rx, ry = mxy if mxy is not None else (self._last_xy if self._last_xy is not None else ('', ''))
        with self.coverage_path.open('a', newline='') as f:
            csv.writer(f).writerow([round(elapsed, 2), round(self.coverage, 4),
                                    round(self.path_length, 3),
                                    round(rx, 3) if rx != '' else '',
                                    round(ry, 3) if ry != '' else ''])

    def victims_callback(self, msg: PoseArray):
        self.victims = [(p.position.x, p.position.y) for p in msg.poses]
        with self.victims_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['id', 'x', 'y'])
            for i, (x, y) in enumerate(self.victims, start=1):
                writer.writerow([f'victim_{i}', x, y])
        # Record the detection moment when the count grows (for the replay video).
        if len(self.victims) > self._victim_count:
            self._victim_count = len(self.victims)
            t = (self._now() - self._t0) if self._t0 is not None else 0.0
            with self.victims_timeline_path.open('a', newline='') as f:
                csv.writer(f).writerow([round(t, 2), self._victim_count])

    # -- summary ----------------------------------------------------------------

    def _real_victims(self):
        """Real AprilTag IDs + map positions, from victim_registry's victims.json
        (the /victims_map PoseArray carries poses only, no IDs). Looked up in the
        run's results dir first, then the repo default."""
        for p in (self.results_dir / 'victims.json', Path('results/victims.json')):
            try:
                vics = json.loads(p.read_text()).get('victims', [])
                if vics:
                    return vics
            except (OSError, json.JSONDecodeError):
                continue
        return []

    def export_summary(self):
        duration = (self._now() - self._t0) if self._t0 is not None else 0.0
        real = self._real_victims()
        ids = sorted({v['id'] for v in real if 'id' in v})
        summary = {
            'strategy': self.strategy,
            'final_coverage': round(self.coverage, 4),
            'victims_detected': max(len(self.victims), len(ids)),
            'success_coverage_90': self.coverage >= 0.90,
            'path_length_m': round(self.path_length, 3),
            'duration_s': round(duration, 1),
            'time_to_50_s': self._time_to[0.5],
            'time_to_75_s': self._time_to[0.75],
            'time_to_90_s': self._time_to[0.90],
            # Real AprilTag victim IDs (so the report can say "ids 0,1,2,3"):
            'victim_ids': ids,
            'victim_positions': [
                {'id': v.get('id'), 'x': round(v['x'], 3), 'y': round(v['y'], 3)}
                for v in real if 'x' in v and 'y' in v
            ],
            'success_victims_3': len(ids) >= 3,
            'success_victims_all': len(ids) >= 4,
        }
        with self.summary_path.open('w') as f:
            json.dump(summary, f, indent=2)
            f.write('\n')


def main(args=None):
    # Flush a final run_summary.json before the node is destroyed.
    run_node(ResultExporterNode, args=args, on_shutdown=lambda node: node.export_summary())


if __name__ == '__main__':
    main()
