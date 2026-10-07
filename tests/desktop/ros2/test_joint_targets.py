import math
from types import SimpleNamespace

from desktop.core.protocol import ENCODER_COUNT
from desktop.ros2.joint_targets import JointTargetCommandHandler


class FakeBackend:
    def __init__(self, result=None) -> None:
        self.control_modes = []
        self.sent = []
        self.starts = 0
        self.result = result
        self.armed = True

    def set_control_mode(self, mode) -> None:
        self.control_modes.append(mode)

    def start(self) -> None:
        self.starts += 1

    def set_ros2_output_armed(self, armed, *, reason=""):
        self.armed = bool(armed)
        return True, reason

    def send_ros2_joint_targets(self, targets, *, source="", input_space="", raw_targets=None):
        self.sent.append((list(targets), source, input_space, list(raw_targets or [])))
        return self.result


def test_ros2_handler_sends_valid_21_degree_targets() -> None:
    backend = FakeBackend()
    handler = JointTargetCommandHandler(backend)

    ok = handler.handle_message(SimpleNamespace(data=[1.5] * ENCODER_COUNT))

    assert ok is True
    assert backend.control_modes == []
    assert backend.sent == [
        ([1.5] * ENCODER_COUNT, "ROS2:/mmhand/joint_targets", "urdf_deg", [1.5] * ENCODER_COUNT)
    ]


def test_ros2_handler_rejects_wrong_length_and_non_finite_values() -> None:
    backend = FakeBackend()
    warnings = []
    handler = JointTargetCommandHandler(
        backend,
        warning_sink=warnings.append,
        warn_interval_s=0.0,
    )

    assert handler.handle_values([0.0] * (ENCODER_COUNT - 1)) is False
    assert handler.handle_values([math.nan] * ENCODER_COUNT) is False

    assert backend.sent == []
    assert len(warnings) == 2


def test_ros2_handler_never_starts_or_enables_device() -> None:
    backend = FakeBackend()
    handler = JointTargetCommandHandler(backend)

    assert handler.handle_values([0.0] * ENCODER_COUNT) is True
    assert handler.handle_values([1.0] * ENCODER_COUNT) is True

    assert backend.starts == 0
    assert len(backend.sent) == 2


def test_ros2_handler_reports_backend_workspace_rejection() -> None:
    warnings = []
    backend = FakeBackend(
        SimpleNamespace(
            sent=False,
            message="Joint command blocked: J00 Manual reference missing",
        )
    )
    handler = JointTargetCommandHandler(
        backend,
        warning_sink=warnings.append,
        warn_interval_s=0.0,
    )

    assert handler.handle_values([0.0] * ENCODER_COUNT) is False
    assert warnings == ["Joint command blocked: J00 Manual reference missing"]


def test_joint_target_bridge_stop_revokes_output_even_before_start() -> None:
    from desktop.ros2.joint_targets import Ros2JointTargetBridge

    backend = FakeBackend()
    bridge = Ros2JointTargetBridge(backend)

    bridge.stop()

    assert backend.armed is False
