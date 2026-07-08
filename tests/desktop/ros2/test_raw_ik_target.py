from desktop.core.protocol import ControlMode, ENCODER_COUNT
from desktop.ros2.raw_ik_target import DEFAULT_RAW_IK_TARGET_TOPIC, RawIkTargetBridge


class FakeBackend:
    def __init__(self) -> None:
        self.control_modes = []
        self.sent = []

    def set_control_mode(self, mode) -> None:
        self.control_modes.append(mode)

    def send_joint_targets(self, targets, *, source="", live=False):
        self.sent.append((list(targets), source, live))


def test_raw_ik_bridge_defaults_to_raw_ik_topic() -> None:
    backend = FakeBackend()
    bridge = RawIkTargetBridge(backend)

    assert bridge.config.topic == DEFAULT_RAW_IK_TARGET_TOPIC


def test_raw_ik_bridge_handler_sends_directly_to_backend() -> None:
    backend = FakeBackend()
    bridge = RawIkTargetBridge(backend)

    assert bridge.handler.handle_values([2.0] * ENCODER_COUNT) is True

    assert backend.control_modes == [ControlMode.JOINT_ANGLE]
    assert backend.sent == [([2.0] * ENCODER_COUNT, "ROS2:/raw_ik_target", True)]
