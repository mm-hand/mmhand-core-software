"""Host calibration must own the backend before external streams can enqueue."""

import threading
from unittest.mock import MagicMock

import pytest

from desktop.core.data_models import HandModel
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import ControlMode, ENCODER_COUNT, MOTOR_COUNT


class _Controller:
    def __init__(self):
        self.current_state = HandModel()
        self.current_state.protocol_compatible = True
        self.current_state.active_motor_mask = 1
        self.current_state.active_joint_mask = 1
        self.state_lock = threading.RLock()
        self.update_callbacks = []
        self.stop_requests = 0
        self.reset_requests = 0
        self.sent = []

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)

    def stop(self):
        self.stop_requests += 1

    def reset(self):
        self.reset_requests += 1

    def is_started(self):
        return True


class _Workflow:
    def __init__(self):
        self.active = False
        self.accept = True
        self.begin_hook = None
        self.calls = []
        self.cancel_requests = 0

    def is_host_calibration_active(self):
        return self.active

    def is_import_active(self):
        return False

    def begin_calibration(self, **kwargs):
        if self.begin_hook is not None:
            self.begin_hook()
        self.calls.append(kwargs)
        self.active = self.accept
        return self.accept

    def cancel(self):
        # The real service keeps its active-worker gate closed until cleanup
        # finishes, even after cancel has invalidated the transaction epoch.
        self.cancel_requests += 1


@pytest.fixture
def backend(monkeypatch):
    monkeypatch.setattr("desktop.core.hand_backend.log_session", lambda *_args: None)
    result = HandBackend(controller=_Controller())
    result._calibration_workflow = _Workflow()
    result.is_comm_connected = lambda: True
    return result


def _begin(backend):
    return backend.begin_calibration_workflow(
        joint_mask=1, motor_mask=1
    )


def test_expected_unknown_policy_is_owned_by_actual_calibration_io(backend):
    assert not backend.is_expected_calibration_turn_unknown(HandModel())
    io = MagicMock()
    state = HandModel()
    backend._controller._host_calibration_io = io
    io.is_expected_calibration_turn_unknown.return_value = False
    assert not backend.is_expected_calibration_turn_unknown(state)
    io.is_expected_calibration_turn_unknown.return_value = True
    assert backend.is_expected_calibration_turn_unknown(state)
    io.is_expected_calibration_turn_unknown.assert_called_with(state)


def test_calibration_entry_revokes_external_sources_before_workflow_submission(backend):
    claimed, token, _ = backend.begin_replay_motion()
    assert claimed
    assert backend.set_ros2_output_armed(True, reason="test")[0]

    def at_submission():
        assert not backend.replay_motion_token_valid(token)
        assert not backend.ros2_output_armed()

    backend._calibration_workflow.begin_hook = at_submission

    assert _begin(backend)
    assert backend._calibration_workflow.calls[0]["joint_mask"] == 1
    assert backend._calibration_workflow.calls[0]["motor_mask"] == 1
    assert isinstance(backend._calibration_workflow.calls[0]["baseline_state"], HandModel)


def test_rejected_calibration_submission_keeps_external_sources_revoked(backend):
    claimed, token, _ = backend.begin_replay_motion()
    assert claimed
    assert backend.set_ros2_output_armed(True, reason="test")[0]
    backend._calibration_workflow.accept = False

    assert not _begin(backend)
    assert not backend.replay_motion_token_valid(token)
    assert not backend.ros2_output_armed()


@pytest.mark.parametrize("enabled_api", ["set_ros2_output_armed", "set_ros2_output_enabled"])
def test_ros2_cannot_be_rearmed_during_host_calibration(backend, enabled_api):
    assert _begin(backend)

    ok, message = getattr(backend, enabled_api)(True)

    assert not ok
    assert "host calibration is active" in message
    assert not backend.ros2_output_armed()
    assert getattr(backend, enabled_api)(False)[0]


def test_replay_cannot_claim_control_during_host_calibration(backend):
    assert _begin(backend)

    ok, token, message = backend.begin_replay_motion()

    assert not ok
    assert token == 0
    assert "host calibration is active" in message


@pytest.mark.parametrize("source", ["UI target", "Replay:old", "ROS2:/joint_targets"])
def test_external_joint_commands_cannot_replace_host_calibration_targets(backend, source):
    assert _begin(backend)

    result = backend.send_joint_targets(
        [0.0] * ENCODER_COUNT,
        source=source,
        live=True,
        command_joint_mask=1,
        motion_token=123,
    )

    assert not result.sent
    assert "host calibration is active" in result.message
    assert backend._controller.sent == []


def test_ros2_receive_gate_rejects_frames_even_if_authorization_is_stale(backend):
    assert _begin(backend)
    # A stale authorization must not bypass the transaction gate.
    backend._ros2_output_armed = True
    backend._ros2_armed_scope = (1, 1, False)

    result = backend.send_ros2_joint_targets(
        [0.0] * ENCODER_COUNT, source="ROS2:/joint_targets"
    )

    assert not result.sent
    assert result.message.endswith("host_calibration_active")
    assert backend._controller.sent == []


def test_replay_claim_waiting_at_submission_cannot_enter_transaction(backend):
    attempting = threading.Event()
    results = []
    workers = []

    def claim():
        attempting.set()
        results.append(backend.begin_replay_motion())

    def at_submission():
        worker = threading.Thread(target=claim)
        workers.append(worker)
        worker.start()
        assert attempting.wait(1)

    backend._calibration_workflow.begin_hook = at_submission
    assert _begin(backend)
    for worker in workers:
        worker.join(1)
        assert not worker.is_alive()

    assert len(results) == 1
    assert not results[0][0]
    assert "host calibration is active" in results[0][2]


def test_stop_and_reset_remain_available_during_host_calibration(backend):
    assert _begin(backend)

    backend.stop()
    backend.reset()

    assert backend._controller.stop_requests == 1
    assert backend._controller.reset_requests == 1


def test_finishing_workflow_does_not_implicitly_restore_external_authorization(backend):
    assert _begin(backend)
    backend._calibration_workflow.active = False

    assert not backend.ros2_output_armed()
    assert not backend._replay_motion_active
    assert backend.set_ros2_output_armed(True, reason="explicit resume")[0]


def test_host_direct_mode_does_not_present_as_an_enabled_user_controller(backend):
    assert backend.is_started()
    assert _begin(backend)

    assert backend._controller.is_started()
    assert not backend.is_started()

    backend.cancel_calibration_workflow()
    assert not backend.is_started()
    backend._calibration_workflow.active = False
    assert backend.is_started()


_USER_OUTPUT_CASES = [
    ("hold_current_motor_positions", (0.0,), False),
    ("invalidate_direct_commands", (), None),
    ("enable_direct_hold", (1,), (False, "host calibration is active", [])),
    ("set_control_mode", (ControlMode.JOINT_ANGLE,), None),
    ("invalidate_joint_commands", (), 0),
    ("set_motor_positions_absolute", ([0] * MOTOR_COUNT,), None),
    ("set_single_motor_position_absolute", (0, 1000), None),
    ("request_run_feedback", (), False),
    ("set_active_scope", (1, 1, True), False),
    ("set_manual_encoder_references", ([1000] * ENCODER_COUNT, [5000] * ENCODER_COUNT), False),
    ("set_manual_encoder_reference", (0, 1000, 5000), False),
    ("clear_manual_encoder_references", (), False),
    ("rope_init", (1,), False),
    ("start", (), None),
]


@pytest.mark.parametrize("method,args,blocked_result", _USER_OUTPUT_CASES)
@pytest.mark.parametrize("cancelled", [False, True], ids=["running", "cancel_cleanup"])
def test_user_outputs_cannot_enter_host_motion_or_cancel_cleanup(
    backend, method, args, blocked_result, cancelled
):
    controller_call = MagicMock(return_value=object())
    setattr(backend._controller, method, controller_call)
    assert _begin(backend)
    if cancelled:
        backend.cancel_calibration_workflow()

    returned = getattr(backend, method)(*args)

    assert returned == blocked_result
    controller_call.assert_not_called()
    if method == "set_active_scope":
        assert backend._requested_active_scope is None


def test_manual_prepare_cannot_reset_coordinate_system_during_host_calibration(backend):
    controller_call = MagicMock()
    backend._controller.manual_prepare_all_motors = controller_call
    assert _begin(backend)

    returned = backend.manual_prepare_all_motors()

    assert not returned.ok
    assert "host calibration is active" in returned.message
    assert returned.reset_motor_mask == 0
    assert returned.invalidated_joint_mask == 0
    controller_call.assert_not_called()


def test_stop_cancels_host_worker_and_pause_resume_are_removed(backend):
    backend._controller.stop = MagicMock()
    assert _begin(backend)
    backend.stop_motion()
    assert backend._calibration_workflow.cancel_requests == 1
    backend._controller.stop.assert_called_once_with()
    assert not hasattr(backend, 'pause') and not hasattr(backend, 'resume')


def test_direct_user_target_waiting_at_submission_cannot_enter_host_transaction(backend):
    controller_call = MagicMock()
    backend._controller.set_single_motor_position_absolute = controller_call
    attempting = threading.Event()
    workers = []

    def send():
        attempting.set()
        backend.set_single_motor_position_absolute(0, 1200)

    def at_submission():
        worker = threading.Thread(target=send)
        workers.append(worker)
        worker.start()
        assert attempting.wait(1)

    backend._calibration_workflow.begin_hook = at_submission
    assert _begin(backend)
    for worker in workers:
        worker.join(1)
        assert not worker.is_alive()

    controller_call.assert_not_called()


@pytest.mark.parametrize("method,args,_blocked_result", _USER_OUTPUT_CASES)
def test_user_output_proxy_remains_available_after_host_cleanup(backend, method, args, _blocked_result):
    controller_result = True
    controller_call = MagicMock(return_value=controller_result)
    setattr(backend._controller, method, controller_call)
    assert _begin(backend)
    backend._calibration_workflow.active = False

    returned = getattr(backend, method)(*args)

    controller_call.assert_called_once()
    if method in {"hold_current_motor_positions", "enable_direct_hold", "invalidate_joint_commands",
                  "request_run_feedback", "set_active_scope",
                  "set_manual_encoder_references", "set_manual_encoder_reference",
                  "clear_manual_encoder_references", "rope_init"}:
        assert returned is controller_result
