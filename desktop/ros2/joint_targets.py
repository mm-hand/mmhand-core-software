"""joint_targets.py - MMHand Desktop 模块契约。

职责：按固定 Topic 订阅目标、发布命令镜像和关节反馈，并经 HandBackend 下发 21 关节角。
线程：ROS2 executor 线程执行；与 Qt/串口线程通过 HandBackend 边界协作。
输入：ROS2 Topic 消息、节点参数和活动模型契约。
输出：校验后的 21 关节目标、JointState 命令镜像/反馈或明确拒绝诊断。
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
DEFAULT_JOINT_STATE_TOPIC = "/mmhand/joint_states"
DEFAULT_JOINT_COMMAND_TOPIC = "/mmhand/joint_cmd"
P4_TIME_FRAME_ID_PREFIX = "mmhand:p4_time_ms="


def format_p4_time_frame_id(p4_time_ms: int) -> str:
    """Encode the mapped-angle packet's P4 uint32 millis in JointState metadata."""
    value = int(p4_time_ms)
    if value != p4_time_ms or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("P4 time must be a uint32 millisecond value")
    return f"{P4_TIME_FRAME_ID_PREFIX}{value}"


@dataclass(frozen=True)
class Ros2JointTargetConfig:
    topic: Optional[str] = DEFAULT_JOINT_TARGET_TOPIC
    auto_start: bool = False
    input_space: str = URDF_DEG


class JointTargetCommandHandler:
    """Validate Float32MultiArray-like payloads and send them through backend."""

    def __init__(
        self,
        backend: HandBackend,
        *,
        topic: Optional[str] = DEFAULT_JOINT_TARGET_TOPIC,
        auto_start: bool = False,
        input_space: str = URDF_DEG,
        warning_sink: Optional[Callable[[str], None]] = None,
        command_sink: Optional[Callable[[int, tuple[float, ...], str], None]] = None,
        warn_interval_s: float = 1.0,
    ) -> None:
        self.backend = backend
        self.topic = topic
        self.auto_start = bool(auto_start)
        self.input_space = str(input_space)
        self.warning_sink = warning_sink or print
        self.command_sink = command_sink
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

        if self.command_sink is not None:
            self.command_sink(time.time_ns(), adaptation.source_targets, adaptation.source_space)

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
        topic: Optional[str] = DEFAULT_JOINT_TARGET_TOPIC,
        auto_start: bool = False,
        input_space: str = URDF_DEG,
        feedback_mode: str = "armed",
        output: str = "all",
    ) -> None:
        self.backend = backend
        self.config = Ros2JointTargetConfig(topic=topic, auto_start=auto_start, input_space=input_space)
        self.feedback_mode = feedback_mode
        if output not in ("off", "joint_states", "joint_cmd", "all"):
            raise ValueError(f"unsupported ROS2 output: {output}")
        self.output = output
        self.publish_joint_states = output in ("joint_states", "all")
        self.publish_joint_cmd = output in ("joint_cmd", "all")
        self._command_publisher = None
        self.handler = JointTargetCommandHandler(
            backend,
            topic=self.config.topic,
            auto_start=self.config.auto_start,
            input_space=self.config.input_space,
            warning_sink=self._warn,
            command_sink=self._publish_joint_command if self.publish_joint_cmd else None,
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
            from sensor_msgs.msg import JointState
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from rclpy.clock import Clock, ClockType
        except Exception as exc:  # pragma: no cover - environment dependent
            raise RuntimeError(
                "ROS2 support requires rclpy, sensor_msgs and std_msgs in the active Python environment"
            ) from exc

        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_rclpy = True

        bridge = self
        self.backend.configure_ros2_feedback(
            self.feedback_mode if self.publish_joint_states else "off"
        )

        class _JointTargetNode(Node):
            def __init__(self) -> None:
                super().__init__("mmhand_desktop_joint_target_subscriber")
                if bridge.config.topic:
                    self.create_subscription(
                        Float32MultiArray,
                        bridge.config.topic,
                        bridge.handler.handle_message,
                        1,
                    )
                if bridge.publish_joint_cmd:
                    bridge._command_publisher = self.create_publisher(
                        JointState, DEFAULT_JOINT_COMMAND_TOPIC, 1
                    )
                if bridge.publish_joint_states and bridge.feedback_mode != "off":
                    publisher = self.create_publisher(JointState, DEFAULT_JOINT_STATE_TOPIC,
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE))

                    def publish(stamp_ns, positions, p4_time_ms):
                        msg = JointState()
                        msg.header.stamp.sec = stamp_ns // 1_000_000_000
                        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
                        msg.header.frame_id = format_p4_time_frame_id(p4_time_ms)
                        msg.name = [f"J{i:02d}" for i in range(21)]
                        msg.position = positions
                        publisher.publish(msg)

                    self.create_timer(0.01, lambda: bridge.backend.publish_ros2_feedback(publish),
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
                if bridge.feedback_mode == "always":
                    self.get_logger().warning("always feedback: HORA continues inference while ROS2 INPUT is off")
                if bridge.config.topic:
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
        self._command_publisher = None

    def _spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except Exception as exc:
            self._warn(f"ROS2 joint target subscriber stopped: {exc}")
        finally:
            self.backend.set_ros2_output_armed(False, reason="ros2_executor_exit")

    def _publish_joint_command(
        self, stamp_ns: int, positions: tuple[float, ...], input_space: str
    ) -> None:
        publisher = self._command_publisher
        if publisher is None:
            return
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = f"mmhand:input={self.config.topic};space={input_space}"
        msg.name = [f"J{i:02d}" for i in range(21)]
        msg.position = list(positions)
        publisher.publish(msg)

    def _warn(self, text: str) -> None:
        node = self._node
        try:
            if node is not None:
                node.get_logger().warn(text)
                return
        except Exception:
            pass
        print(text)
