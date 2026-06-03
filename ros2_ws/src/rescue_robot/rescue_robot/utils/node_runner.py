"""Shared entry-point helper for rescue_robot ROS 2 nodes.

Every node has the same lifecycle: ``rclpy.init`` → spin → ``destroy_node`` →
``rclpy.shutdown``.  ``run_node`` centralises that boilerplate so each node's
``main()`` stays a one-liner and the shutdown path (KeyboardInterrupt swallowed,
double-shutdown guarded) is identical everywhere.

Example::

    from rescue_robot.utils.node_runner import run_node

    def main(args=None):
        run_node(MyNode, args=args)
"""
from __future__ import annotations

from collections.abc import Callable

import rclpy
from rclpy.executors import Executor
from rclpy.node import Node


def run_node(
    node_factory: Callable[[], Node],
    *,
    args=None,
    executor_factory: Callable[[], Executor] | None = None,
    on_shutdown: Callable[[Node], None] | None = None,
) -> None:
    """Initialise rclpy, spin ``node_factory()`` and clean up on exit.

    Args:
        node_factory: zero-argument callable returning the Node to spin
            (usually the Node subclass itself).
        args: forwarded to ``rclpy.init``.
        executor_factory: optional zero-argument callable returning an
            Executor (e.g. ``MultiThreadedExecutor``).  When omitted the node
            is spun with the default single-threaded ``rclpy.spin``.
        on_shutdown: optional callback invoked with the node just before it is
            destroyed (e.g. to flush results to disk).  Exceptions raised by
            the callback are swallowed so shutdown always completes.
    """
    rclpy.init(args=args)
    node = node_factory()

    executor: Executor | None = executor_factory() if executor_factory else None
    if executor is not None:
        executor.add_node(node)

    try:
        if executor is not None:
            executor.spin()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if on_shutdown is not None:
            try:
                on_shutdown(node)
            except Exception:  # noqa: BLE001 — shutdown must never raise
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
