"""Throttle a LaserScan topic to a fixed rate.

The Ignition RPLIDAR bridge publishes /scan_raw far faster than the real sensor
(observed ~300 Hz vs the 10 Hz of an RPLIDAR A2).  At that rate slam_toolbox's
tf2 MessageFilter overflows ("queue is full") and integrates no scan, so the map
never grows past its initial 7x7 cell (cf. docs/ERRORS_AND_FIXES.md #25).

This node forwards at most ``rate_hz`` messages per second from ``in_topic`` to
``out_topic``, so SLAM and RViz consume a sane scan rate.

    ros2 run rescue_robot scan_throttle_node \
        --ros-args -p in_topic:=/scan_raw -p out_topic:=/scan -p rate_hz:=10.0
"""
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from rescue_robot.utils.node_runner import run_node

# Input QoS: BEST_EFFORT matches the ros_gz LaserScan bridge on /scan_raw.
_SUB_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
# Output QoS: RELIABLE. slam_toolbox subscribes to /scan as RELIABLE; a
# BEST_EFFORT publisher is rejected under CycloneDDS ("incompatible QoS ...
# RELIABILITY") and SLAM then receives no scans (no map). A RELIABLE publisher
# is compatible with BOTH a RELIABLE subscriber (slam_toolbox) and a BEST_EFFORT
# one (RViz), and works under Fast-DDS and CycloneDDS alike.
_PUB_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ScanThrottleNode(Node):
    """Republish ``in_topic`` onto ``out_topic`` at no more than ``rate_hz``."""

    def __init__(self) -> None:
        super().__init__("scan_throttle_node")
        self.declare_parameter("in_topic", "/scan_raw")
        self.declare_parameter("out_topic", "/scan")
        self.declare_parameter("rate_hz", 10.0)
        # The Ignition lidar bridge stamps scans slightly ahead of /clock; SLAM
        # then can't look up the TF at that future stamp and never accumulates
        # the map. Re-stamping the forwarded scan with the current sim clock
        # keeps the lookup inside the TF buffer (ERRORS_AND_FIXES #25).
        self.declare_parameter("restamp", True)
        # Re-stamp slightly in the PAST. Stamping at exactly "now" can land just
        # ahead of the latest odom->base_link transform the diffdrive_controller
        # has published, so slam_toolbox's tf2 MessageFilter can't resolve
        # scan->odom and drops every scan ("queue is full") -> no map. A small
        # backdate guarantees the odom TF for that stamp already exists.
        self.declare_parameter("restamp_back_sec", 0.15)

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        self._period = 1.0 / float(self.get_parameter("rate_hz").value)
        self._restamp = bool(self.get_parameter("restamp").value)
        self._restamp_back = float(self.get_parameter("restamp_back_sec").value)
        self._last = 0.0

        self.pub = self.create_publisher(LaserScan, out_topic, _PUB_QOS)
        self.sub = self.create_subscription(LaserScan, in_topic, self._on_scan, _SUB_QOS)
        self.get_logger().info(
            f"scan_throttle_node: {in_topic} -> {out_topic} "
            f"@ {self.get_parameter('rate_hz').value} Hz"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last >= self._period:
            self._last = now
            if self._restamp:
                stamp = self.get_clock().now() - Duration(seconds=self._restamp_back)
                msg.header.stamp = stamp.to_msg()
            self.pub.publish(msg)


def main(args=None) -> None:
    run_node(ScanThrottleNode, args=args)


if __name__ == "__main__":
    main()
