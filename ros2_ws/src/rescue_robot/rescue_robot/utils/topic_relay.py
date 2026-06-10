"""Generic one-to-one topic relay base class.

Several TurtleBot4 + Ignition integration problems are solved by re-publishing a
message verbatim from one topic onto another (namespace bridging, QoS bridging).
``TopicRelay`` captures that pattern so each concrete relay only declares its
message type, source/destination topics and QoS profiles.
"""
from __future__ import annotations


from rclpy.node import Node
from rclpy.qos import QoSProfile


class TopicRelay(Node):
    """Subscribe on ``src_topic`` and re-publish each message on ``dst_topic``."""

    def __init__(
        self,
        node_name: str,
        msg_type,
        src_topic: str,
        dst_topic: str,
        *,
        sub_qos: QoSProfile,
        pub_qos: QoSProfile,
        description: str | None = None,
    ) -> None:
        super().__init__(node_name)
        self._dst_topic = dst_topic
        self._src_topic = src_topic
        self.pub = self.create_publisher(msg_type, dst_topic, pub_qos)
        self.sub = self.create_subscription(msg_type, src_topic, self._relay, sub_qos)
        self.get_logger().info(description or f"{node_name}: {src_topic} -> {dst_topic}")

    def _relay(self, msg) -> None:
        self.pub.publish(msg)
