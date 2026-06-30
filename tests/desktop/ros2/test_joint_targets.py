import math
from types import SimpleNamespace

from desktop.core.protocol import ControlMode, ENCODER_COUNT
from desktop.ros2.joint_targets import JointTargetCommandHandler


class FakeBackend:
    def __init__(self) -> None:
        self.control_modes = []
        self.sent = []
        self.starts = 0

    def set_control_mode(self, mode) -> None:
        self.control_modes.append(mode)

    def start(self) -> None:
        self.starts += 1

    def send_joint_targets(self, targets, *, source="", live=False):
        self.sent.append((list(targets), source, live))


def test_ros2_handler_sends_valid_21_degree_targets() -> None:
    backend = FakeBackend()
    handler = JointTargetCommandHandler(backend, topic="/mmhand/joint_targets")

    ok = handler.handle_message(SimpleNamespace(data=[1.5] * ENCODER_COUNT))

    assert ok is True
    assert backend.control_modes == [ControlMode.JOINT_ANGLE]
    assert backend.sent == [([1.5] * ENCODER_COUNT, "ROS2:/mmhand/joint_targets", True)]


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
