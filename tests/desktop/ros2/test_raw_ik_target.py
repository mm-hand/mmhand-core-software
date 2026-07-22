from desktop.core.protocol import ENCODER_COUNT
from desktop.ros2.raw_ik_target import DEFAULT_RAW_IK_TARGET_TOPIC, RawIkTargetBridge
from desktop.teleop.joint_target_adapter import LEGACY_CALIBRATED_DEG, URDF_DEG, adapt_joint_targets


class FakeBackend:
    def __init__(self) -> None:
        self.control_modes = []
        self.sent = []
        self.armed = True

    def set_control_mode(self, mode) -> None:
        self.control_modes.append(mode)

    def send_ros2_joint_targets(self, targets, *, source="", input_space="", raw_targets=None):
        self.sent.append((list(targets), source, input_space, list(raw_targets or [])))

    def set_ros2_output_armed(self, armed, *, reason=""):
        self.armed = bool(armed)
        return True, reason


def test_raw_ik_bridge_defaults_to_raw_ik_topic() -> None:
    backend = FakeBackend()
    bridge = RawIkTargetBridge(backend)

    assert bridge.config.topic == DEFAULT_RAW_IK_TARGET_TOPIC


def test_raw_ik_bridge_handler_sends_directly_to_backend() -> None:
    backend = FakeBackend()
    bridge = RawIkTargetBridge(backend)

    assert bridge.handler.handle_values([2.0] * ENCODER_COUNT) is True

    assert backend.control_modes == []
    raw = [2.0] * ENCODER_COUNT
    expected = list(adapt_joint_targets(raw, source_space=LEGACY_CALIBRATED_DEG).urdf_targets_deg)
    assert backend.sent == [
        (expected, "ROS2:/raw_ik_target", LEGACY_CALIBRATED_DEG, raw)
    ]


def test_raw_ik_bridge_can_accept_active_urdf_degrees_explicitly() -> None:
    backend = FakeBackend()
    bridge = RawIkTargetBridge(backend, input_space=URDF_DEG)
    raw = [3.0] * ENCODER_COUNT

    assert bridge.handler.handle_values(raw) is True
    assert backend.sent == [(raw, "ROS2:/raw_ik_target", URDF_DEG, raw)]


def test_raw_ik_bridge_stop_revokes_output_even_before_start() -> None:
    backend = FakeBackend()
    bridge = RawIkTargetBridge(backend)

    bridge.stop()

    assert backend.armed is False
