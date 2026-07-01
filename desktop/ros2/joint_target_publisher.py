"""ROS2 publisher for validated 21-DOF mmhand joint targets."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

from desktop.core.protocol import ENCODER_COUNT
from desktop.ros2.joint_targets import DEFAULT_JOINT_TARGET_TOPIC


class Ros2JointTargetPublisher:
    """Publish J00-J20 degree targets as ``Float32MultiArray`` messages.

    A publisher does not need an executor to send messages. This lets it share
    the rclpy context with :class:`Ros2JointTargetBridge`, whose executor spins
    in the desktop background thread.
    """

    def __init__(
        self,
        *,
        topic: str = DEFAULT_JOINT_TARGET_TOPIC,
        node_name: str = "mmhand_manus_joint_target_publisher",
        rclpy_module=None,
        message_types: Optional[Tuple[type, type, type]] = None,
    ) -> None:
        self.topic = str(topic)
        self.node_name = str(node_name)
        self._rclpy_override = rclpy_module
        self._message_types_override = message_types
        self._rclpy = None
        self._node = None
        self._publisher = None
        self._message_types: Optional[Tuple[type, type, type]] = None
        self._owns_rclpy_context = False

    @property
    def started(self) -> bool:
        return self._publisher is not None

    def start(self) -> None:
        if self.started:
            return

        if self._rclpy_override is None:
            try:
                import rclpy
            except Exception as exc:  # pragma: no cover - environment dependent
                raise RuntimeError("ROS2 publishing requires rclpy") from exc
        else:
            rclpy = self._rclpy_override

        if self._message_types_override is None:
            try:
                from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
            except Exception as exc:  # pragma: no cover - environment dependent
                raise RuntimeError("ROS2 publishing requires std_msgs") from exc
            message_types = (Float32MultiArray, MultiArrayDimension, MultiArrayLayout)
        else:
            message_types = self._message_types_override

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        try:
            node = rclpy.create_node(self.node_name)
            publisher = node.create_publisher(message_types[0], self.topic, 10)
        except Exception:
            if self._owns_rclpy_context and rclpy.ok():
                rclpy.shutdown()
            self._owns_rclpy_context = False
            raise

        self._rclpy = rclpy
        self._node = node
        self._publisher = publisher
        self._message_types = message_types

    def publish_values(self, values: Iterable[float]) -> None:
        targets = self.validate_values(values)
        if not self.started:
            self.start()

        Float32MultiArray, MultiArrayDimension, MultiArrayLayout = self._message_types
        msg = Float32MultiArray()
        msg.data = targets

        dim = MultiArrayDimension()
        dim.label = "mmhand_joints_deg"
        dim.size = ENCODER_COUNT
        dim.stride = ENCODER_COUNT
        layout = MultiArrayLayout()
        layout.dim = [dim]
        layout.data_offset = 0
        msg.layout = layout
        self._publisher.publish(msg)

    def subscription_count(self) -> int:
        if self._publisher is None:
            return 0
        try:
            return int(self._publisher.get_subscription_count())
        except Exception:
            return 0

    def stop(self) -> None:
        node = self._node
        rclpy = self._rclpy
        self._publisher = None
        self._node = None
        self._message_types = None
        self._rclpy = None

        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if self._owns_rclpy_context and rclpy is not None:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
        self._owns_rclpy_context = False

    @staticmethod
    def validate_values(values: Iterable[float]) -> list[float]:
        try:
            targets = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise ValueError("joint targets must be numeric") from exc
        if len(targets) != ENCODER_COUNT:
            raise ValueError(f"expected {ENCODER_COUNT} joint targets, got {len(targets)}")
        if not all(math.isfinite(value) for value in targets):
            raise ValueError("joint targets must be finite")
        return targets

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.stop()


__all__ = ["Ros2JointTargetPublisher"]
