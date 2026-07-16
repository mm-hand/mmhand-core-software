import math
from types import SimpleNamespace

from desktop.core.protocol import ControlMode, ENCODER_COUNT
from desktop.ros2.joint_targets import JointTargetCommandHandler


class FakeBackend:
    def __init__(self, result=None) -> None:
        self.control_modes = []
        self.sent = []
        self.starts = 0
        self.result = result

    def set_control_mode(self, mode) -> None:
        self.control_modes.append(mode)

    def start(self) -> None:
        self.starts += 1

    def send_joint_targets(self, targets, *, source="", live=False, input_space="", raw_targets=None):
        self.sent.append((list(targets), source, live, input_space, list(raw_targets or [])))
        return self.result


def test_ros2_handler_sends_valid_21_degree_targets() -> None:
    backend = FakeBackend()
    handler = JointTargetCommandHandler(backend, topic="/mmhand/joint_targets")

    ok = handler.handle_message(SimpleNamespace(data=[1.5] * ENCODER_COUNT))

    assert ok is True
    assert backend.control_modes == [ControlMode.JOINT_ANGLE]
    assert backend.sent == [
        ([1.5] * ENCODER_COUNT, "ROS2:/mmhand/joint_targets", True, "urdf_deg", [1.5] * ENCODER_COUNT)
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


def test_ros2_handler_auto_start_only_once() -> None:
    backend = FakeBackend()
    handler = JointTargetCommandHandler(backend, auto_start=True)

    assert handler.handle_values([0.0] * ENCODER_COUNT) is True
    assert handler.handle_values([1.0] * ENCODER_COUNT) is True

    assert backend.starts == 1
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
