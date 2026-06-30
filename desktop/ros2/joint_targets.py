"""Optional ROS2 joint-target subscriber for desktop backends."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Iterable, Optional

from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import ControlMode, ENCODER_COUNT

DEFAULT_JOINT_TARGET_TOPIC = "/mmhand/joint_targets"


@dataclass(frozen=True)
class Ros2JointTargetConfig:
    topic: str = DEFAULT_JOINT_TARGET_TOPIC
    auto_start: bool = False


class JointTargetCommandHandler:
    """Validate Float32MultiArray-like payloads and send them through backend."""

    def __init__(
        self,
        backend: HandBackend,
        *,
        topic: str = DEFAULT_JOINT_TARGET_TOPIC,
        auto_start: bool = False,
        warning_sink: Optional[Callable[[str], None]] = None,
        warn_interval_s: float = 1.0,
    ) -> None:
        self.backend = backend
        self.topic = topic
        self.auto_start = bool(auto_start)
        self.warning_sink = warning_sink or print
        self.warn_interval_s = float(warn_interval_s)
        self._last_warn_ts = 0.0
        self._auto_started = False

    def handle_message(self, msg: object) -> bool:
        return self.handle_values(getattr(msg, "data", []))

    def handle_values(self, values: Iterable[float]) -> bool:
        try:
            targets = [float(v) for v in values]
        except (TypeError, ValueError):
            self._warn("ignored ROS2 joint target: data must be numeric")
            return False
        if len(targets) != ENCODER_COUNT:
            self._warn(
                f"ignored ROS2 joint target: expected {ENCODER_COUNT} values, got {len(targets)}"
            )
            return False
        if not all(math.isfinite(v) for v in targets):
            self._warn("ignored ROS2 joint target: values must be finite")
            return False

        self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
        if self.auto_start and not self._auto_started:
            self.backend.start()
            self._auto_started = True
        self.backend.send_joint_targets(
            targets,
            source=f"ROS2:{self.topic}",
            live=True,
        )
        return True

    def _warn(self, text: str) -> None:
        now = time.time()
        if now - self._last_warn_ts < self.warn_interval_s:
            return
        self._last_warn_ts = now
        self.warning_sink(text)


class Ros2JointTargetBridge:
    """Background ROS2 subscriber that feeds joint targets into HandBackend."""

    def __init__(
        self,
        backend: HandBackend,
        *,
        topic: str = DEFAULT_JOINT_TARGET_TOPIC,
        auto_start: bool = False,
    ) -> None:
        self.config = Ros2JointTargetConfig(topic=topic, auto_start=auto_start)
        self.handler = JointTargetCommandHandler(
            backend,
            topic=self.config.topic,
            auto_start=self.config.auto_start,
            warning_sink=self._warn,
        )
        self._thread: Optional[threading.Thread] = None
        self._node = None
        self._rclpy = None
        self._shutdown_rclpy = False
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float32MultiArray
        except Exception as exc:  # pragma: no cover - environment dependent
            raise RuntimeError(
                "ROS2 support requires rclpy and std_msgs in the active Python environment"
            ) from exc

        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_rclpy = True

        bridge = self

        class _JointTargetNode(Node):
            def __init__(self) -> None:
                super().__init__("mmhand_desktop_joint_target_subscriber")
                self.create_subscription(
                    Float32MultiArray,
                    bridge.config.topic,
                    bridge.handler.handle_message,
                    10,
                )
                self.get_logger().info(
                    f"Listening for Float32MultiArray joint targets on {bridge.config.topic}"
                )

        self._rclpy = rclpy
        self._node = _JointTargetNode()
        self._thread = threading.Thread(target=self._spin, name="mmhand-ros2-joint-targets", daemon=True)
        self._started = True
        self._thread.start()

    def stop(self) -> None:
        if not self._started:
            return
        rclpy = self._rclpy
        node = self._node
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if self._shutdown_rclpy and rclpy is not None:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._node = None
        self._rclpy = None
        self._started = False
        self._shutdown_rclpy = False

    def _spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except Exception as exc:
            self._warn(f"ROS2 joint target subscriber stopped: {exc}")

    def _warn(self, text: str) -> None:
        node = self._node
        try:
            if node is not None:
                node.get_logger().warn(text)
                return
        except Exception:
            pass
        print(text)
