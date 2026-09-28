"""joint_targets.py - MMHand Desktop 模块契约。

职责：订阅最终关节目标 Topic 并经 HandBackend 下发 21 关节角。
线程：ROS2 executor 线程执行；与 Qt/串口线程通过 HandBackend 边界协作。
输入：ROS2 Topic 消息、节点参数和活动模型契约。
输出：校验后的 21 关节目标或明确拒绝诊断。
状态所有权：仅拥有 ROS2 节点、订阅器和最近消息元数据，不拥有设备状态。
禁止依赖：禁止依赖 Qt/UI，禁止直接写串口或绕过 HandBackend 的控制门禁。
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable, Iterable, Optional

from desktop.core.hand_backend import HandBackend
from desktop.core.joint_target_adapter import URDF_DEG, adapt_joint_targets

DEFAULT_JOINT_TARGET_TOPIC = "/mmhand/joint_targets"


@dataclass(frozen=True)
class Ros2JointTargetConfig:
    topic: str = DEFAULT_JOINT_TARGET_TOPIC
    auto_start: bool = False
    input_space: str = URDF_DEG


class JointTargetCommandHandler:
    """Validate Float32MultiArray-like payloads and send them through backend."""

    def __init__(
        self,
        backend: HandBackend,
        *,
        topic: str = DEFAULT_JOINT_TARGET_TOPIC,
        auto_start: bool = False,
        input_space: str = URDF_DEG,
        warning_sink: Optional[Callable[[str], None]] = None,
        warn_interval_s: float = 1.0,
    ) -> None:
        self.backend = backend
        self.topic = topic
        self.auto_start = bool(auto_start)
        self.input_space = str(input_space)
        self.warning_sink = warning_sink or print
        self.warn_interval_s = float(warn_interval_s)
        self._last_warn_ts = 0.0
        self._auto_started = False

    def handle_message(self, msg: object) -> bool:
        return self.handle_values(getattr(msg, "data", []))

    def handle_values(self, values: Iterable[float]) -> bool:
        try:
            adaptation = adapt_joint_targets(values, source_space=self.input_space)
        except (TypeError, ValueError) as exc:
            self._warn(f"ignored ROS2 joint target: {exc}")
            return False

        # 两个 ROS2 输入 topic 统一经过 backend 的授权门。这里不能直接调用
        # send_joint_targets，否则 E-STOP 后的下一帧会再次发送 RUN_FEEDBACK。
        result = self.backend.send_ros2_joint_targets(
            adaptation.urdf_targets_deg,
            source=f"ROS2:{self.topic}",
            input_space=adaptation.source_space,
            raw_targets=adaptation.source_targets,
        )
        sent = bool(getattr(result, "sent", True))
        message = str(getattr(result, "message", "") or "")
        if message:
            self._warn(message)
        if not sent and not message:
            self._warn(f"ignored ROS2 joint target on {self.topic}: backend rejected frame")
        if sent and self.auto_start and not self._auto_started:
            self.backend.start()
            self._auto_started = True
        return sent

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
        self.backend = backend
        self.config = Ros2JointTargetConfig(topic=topic, auto_start=auto_start, input_space=URDF_DEG)
        self.handler = JointTargetCommandHandler(
            backend,
            topic=self.config.topic,
            auto_start=self.config.auto_start,
            input_space=self.config.input_space,
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
        self.backend.set_ros2_output_armed(False, reason="ros2_bridge_stop")
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
