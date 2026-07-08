#!/usr/bin/env python3
"""Publish a simple 21-value joint target array for ROS2 desktop testing."""

from __future__ import annotations

import argparse
import math
import time

from desktop.core.protocol import ENCODER_COUNT
from desktop.ros2.joint_targets import DEFAULT_JOINT_TARGET_TOPIC


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish mmhand joint target arrays over ROS2.")
    parser.add_argument("--topic", default=DEFAULT_JOINT_TARGET_TOPIC)
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate in Hz.")
    parser.add_argument("--amplitude", type=float, default=10.0, help="Sine amplitude in degrees.")
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

    class _Publisher(Node):
        def __init__(self) -> None:
            super().__init__("mmhand_joint_target_example_sender")
            self.publisher = self.create_publisher(Float32MultiArray, args.topic, 10)
            self.t0 = time.time()
            self.timer = self.create_timer(1.0 / max(0.1, float(args.rate)), self._tick)
            self.get_logger().info(f"Publishing {ENCODER_COUNT} degree targets on {args.topic}")

        def _tick(self) -> None:
            t = time.time() - self.t0
            value = float(args.amplitude) * math.sin(t)
            data = [value] * ENCODER_COUNT
            msg = Float32MultiArray()
            msg.data = data
            dim = MultiArrayDimension()
            dim.label = "mmhand_joints_deg"
            dim.size = ENCODER_COUNT
            dim.stride = ENCODER_COUNT
            layout = MultiArrayLayout()
            layout.dim = [dim]
            layout.data_offset = 0
            msg.layout = layout
            self.publisher.publish(msg)

    rclpy.init()
    node = _Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
