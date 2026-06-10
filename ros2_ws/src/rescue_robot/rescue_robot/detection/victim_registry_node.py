"""Victim registry — AprilTag detections projected into the map frame.

Real implementation of the victim pipeline (upgrades the victim_detector
scaffold). apriltag_ros publishes a TF for every detected tag
(camera_optical_frame -> victim_<id>, frame names from apriltag_tags.yaml).
This node:

1. listens to /detections (apriltag_msgs/AprilTagDetectionArray);
2. for each detected tag id, looks up TF map -> victim_<id> via tf2
   (the full chain map -> odom -> base_link -> camera -> tag must be up,
   which is exactly the "projection TF" requirement of the assignment);
3. de-duplicates by tag id (first valid registration wins; re-registers
   only if the tag apparently moved more than duplicate_distance_threshold_m);
4. publishes ALL registered victims as a PoseArray on /victims_map
   (map frame) — the interface already consumed by result_exporter_node
   and rviz_marker_node, so downstream needs no change;
5. persists results/victims.json after every new registration.

Run::

    ros2 run rescue_robot victim_registry_node
    # TB4 (namespaced camera TF chain works the same once tf_relay bridges it)
    ros2 run rescue_robot victim_registry_node --ros-args -p map_frame:=map
"""
import json
import math
from pathlib import Path

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from rescue_robot.utils.node_runner import run_node


class VictimRegistryNode(Node):
    def __init__(self):
        super().__init__('victim_registry_node')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('output_topic', '/victims_map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tag_frame_prefix', 'victim_')
        self.declare_parameter('duplicate_distance_threshold_m', 0.5)
        self.declare_parameter('persist_path', 'results/victims.json')

        self._map_frame = self.get_parameter('map_frame').value
        self._prefix = self.get_parameter('tag_frame_prefix').value
        self._dup_thresh = float(self.get_parameter('duplicate_distance_threshold_m').value)
        self._persist_path = Path(str(self.get_parameter('persist_path').value))

        # id (int) -> dict(x=, y=, z=)
        self._victims: dict[int, dict] = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray,
            self.get_parameter('detections_topic').value,
            self._detections_cb, 10,
        )
        self._pub = self.create_publisher(
            PoseArray, self.get_parameter('output_topic').value, 10
        )
        # Republish the registry at 1 Hz so late subscribers always get it.
        self.create_timer(1.0, self._publish_registry)
        self.get_logger().info(
            f'Victim registry started — projecting {self._prefix}<id> TF frames '
            f'into {self._map_frame}, dedup threshold {self._dup_thresh} m.'
        )

    # -- detections ---------------------------------------------------------

    def _detections_cb(self, msg: AprilTagDetectionArray) -> None:
        for det in msg.detections:
            tag_id = int(det.id)
            frame = f'{self._prefix}{tag_id}'
            try:
                tf = self._tf_buffer.lookup_transform(self._map_frame, frame, Time())
            except Exception:  # noqa: BLE001 — chain not complete yet, retry on next msg
                continue
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            self._register(tag_id, x, y, z)

    def _register(self, tag_id: int, x: float, y: float, z: float) -> None:
        known = self._victims.get(tag_id)
        if known is not None:
            moved = math.dist((known['x'], known['y']), (x, y))
            if moved <= self._dup_thresh:
                return  # same physical tag, already registered
            self.get_logger().warning(
                f'victim {tag_id} re-registered {moved:.2f} m away '
                f'(old ({known["x"]:.2f},{known["y"]:.2f}) -> new ({x:.2f},{y:.2f}))'
            )
        else:
            self.get_logger().info(
                f'NEW victim {tag_id} registered at ({x:.2f}, {y:.2f}) in {self._map_frame}'
            )
        self._victims[tag_id] = {'x': x, 'y': y, 'z': z}
        self._persist()
        self._publish_registry()

    # -- outputs ------------------------------------------------------------

    def _publish_registry(self) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        for tag_id in sorted(self._victims):
            v = self._victims[tag_id]
            pose = Pose()
            pose.position.x = v['x']
            pose.position.y = v['y']
            pose.position.z = v['z']
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._pub.publish(msg)

    def _persist(self) -> None:
        try:
            self._persist_path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                'frame': self._map_frame,
                'victims': [
                    {'id': tag_id, **self._victims[tag_id]}
                    for tag_id in sorted(self._victims)
                ],
            }
            self._persist_path.write_text(json.dumps(payload, indent=2))
        except OSError as exc:
            self.get_logger().warning(f'could not persist victims.json: {exc}')


def main(args=None):
    run_node(VictimRegistryNode, args=args)


if __name__ == '__main__':
    main()
