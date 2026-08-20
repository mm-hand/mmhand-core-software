"""raw_ik_target.py - MMHand Desktop 模块契约。

职责：订阅原始 IK Topic，完成目标适配后交给统一关节命令处理器。
线程：ROS2 executor 线程执行；与 Qt/串口线程通过 HandBackend 边界协作。
输入：ROS2 Topic 消息、节点参数和活动模型契约。
输出：校验后的 21 关节目标或明确拒绝诊断。
状态所有权：仅拥有 ROS2 节点、订阅器和最近消息元数据，不拥有设备状态。
禁止依赖：禁止依赖 Qt/UI，禁止直接写串口或绕过 HandBackend 的控制门禁。
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Optional

from desktop.core.hand_backend import HandBackend
from desktop.ros2.joint_targets import JointTargetCommandHandler
from desktop.core.joint_target_adapter import LEGACY_CALIBRATED_DEG

DEFAULT_RAW_IK_TARGET_TOPIC = "/raw_ik_target"


@dataclass(frozen=True)
class Ros2RawIkTargetConfig:
    topic: str = DEFAULT_RAW_IK_TARGET_TOPIC
    auto_start: bool = False
    input_space: str = LEGACY_CALIBRATED_DEG


class RawIkTargetBridge:
    """Background ROS2 subscriber for external raw-IK 21-angle output."""

    def __init__(
        self,
        backend: HandBackend,
        *,
        topic: str = DEFAULT_RAW_IK_TARGET_TOPIC,
        auto_start: bool = False,
        input_space: str = LEGACY_CALIBRATED_DEG,
    ) -> None:
        self.backend = backend
        self.config = Ros2RawIkTargetConfig(
            topic=topic,
            auto_start=auto_start,
            input_space=input_space,
        )
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

        class _RawIkTargetNode(Node):
            def __init__(self) -> None:
                super().__init__("mmhand_desktop_raw_ik_target_subscriber")
                self.create_subscription(
                    Float32MultiArray,
                    bridge.config.topic,
                    bridge.handler.handle_message,
                    10,
                )
                self.get_logger().info(
                    f"Listening for Float32MultiArray raw IK targets on {bridge.config.topic}"
                )

        self._rclpy = rclpy
        self._node = _RawIkTargetNode()
        self._thread = threading.Thread(target=self._spin, name="mmhand-ros2-raw-ik-target", daemon=True)
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
            self._warn(f"ROS2 raw IK target subscriber stopped: {exc}")

    def _warn(self, text: str) -> None:
        node = self._node
        try:
            if node is not None:
                node.get_logger().warn(text)
                return
        except Exception:
            pass
        print(text)


__all__ = ["DEFAULT_RAW_IK_TARGET_TOPIC", "RawIkTargetBridge", "Ros2RawIkTargetConfig"]
