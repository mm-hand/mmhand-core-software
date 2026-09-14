import math
from types import SimpleNamespace

from desktop.core.protocol import ENCODER_COUNT
from desktop.ros2.joint_targets import (
    JointTargetCommandHandler,
    Ros2JointTargetBridge,
    format_p4_time_frame_id,
)


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


def test_p4_time_uses_parseable_joint_state_frame_id_metadata() -> None:
    assert format_p4_time_frame_id(0) == "mmhand:p4_time_ms=0"
    assert format_p4_time_frame_id(0xFFFFFFFF) == "mmhand:p4_time_ms=4294967295"


def test_ros2_handler_sends_valid_21_degree_targets() -> None:
    backend = FakeBackend()
    handler = JointTargetCommandHandler(backend, topic="/mmhand/joint_targets")

    ok = handler.handle_message(SimpleNamespace(data=[1.5] * ENCODER_COUNT))

    assert ok is True
    assert backend.control_modes == []
    assert backend.sent == [
        ([1.5] * ENCODER_COUNT, "ROS2:/mmhand/joint_targets", "urdf_deg", [1.5] * ENCODER_COUNT)
    ]


def test_joint_cmd_reports_the_valid_frame_exactly_as_received() -> None:
    backend = FakeBackend(SimpleNamespace(sent=False, message="not armed"))
    commands = []
    handler = JointTargetCommandHandler(
        backend,
        topic="/raw_ik_target",
        command_sink=lambda stamp, values, space: commands.append((stamp, values, space)),
        warn_interval_s=0.0,
    )
    received = [0.01 * index for index in range(ENCODER_COUNT)]

    assert handler.handle_values(received) is False

    assert len(commands) == 1
    stamp, values, space = commands[0]
    assert stamp > 0
    assert values == tuple(received)
    assert space == "urdf_deg"


def test_bridge_output_mode_selects_fixed_publishers() -> None:
    backend = FakeBackend()

    states = Ros2JointTargetBridge(backend, output="joint_states")
    command = Ros2JointTargetBridge(backend, output="joint_cmd")

    assert (states.publish_joint_states, states.publish_joint_cmd) == (True, False)
    assert (command.publish_joint_states, command.publish_joint_cmd) == (False, True)


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


def test_joint_target_bridge_stop_revokes_output_even_before_start() -> None:
    from desktop.ros2.joint_targets import Ros2JointTargetBridge

    backend = FakeBackend()
    bridge = Ros2JointTargetBridge(backend)

    bridge.stop()

    assert backend.armed is False
