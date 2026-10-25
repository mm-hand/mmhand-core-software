"""One desktop request per recovery operation; the P4 owns hardware execution."""
from _clock_helpers import stamp
import queue
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
import desktop.core.core_logic as core_logic
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.core_logic import HandController
from desktop.core.protocol import (HOST_PROTOCOL_VERSION, MOTOR_COUNT, CMD_RESTORE_MULTITURN,
    STATE_SYNC_MULTITURN_STATE, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_UNSUPPORTED_MODE)

@pytest.fixture
def restore(monkeypatch):
    clock = {"wall": 100.0, "mono": 20.0, "connected": True}
    monkeypatch.setattr(core_logic.time, "time", lambda: clock["wall"])
    monkeypatch.setattr(core_logic.time, "monotonic", lambda: clock["mono"])
    controller = HandController()
    comm = LowerComputerComm("")
    comm._protocol_compatible = True
    monkeypatch.setattr(comm, "is_connected", lambda: clock["connected"])
    controller.comm = comm
    state = controller.current_state
    state.system_state = SYSTEM_STATE_IDLE
    state.has_system_status_data = True
    state.system_status_last_rx_ts = clock["wall"]
    stamp(state, 'system_status_last_rx_ts')
    state.servo_abs_last_rx_ts = clock["wall"]
    stamp(state, 'servo_abs_last_rx_ts')
    state.system_status_p4_time_ms = state.servo_feedback_p4_time_ms = 1000
    state.has_p4_time = True
    state.p4_time_ms = 1000
    state.has_product_fingerprint = True
    state.product_fingerprint = {
        "protocol_version": HOST_PROTOCOL_VERSION,
        "device_id": "0000123456789ABC",
    }
    state.servo_online_bitmap = 7
    state.multi_turn_unknown_bitmap = 7
    state.has_servo_route_status = True
    state.servo_route_controllable_bitmap = 7
    state.servo_route_scan_state = 2
    state.has_servo_angle_data = True
    state.has_servo_online_status = True
    state.servo_online_check = [1] * MOTOR_COUNT
    state.has_multiturn_restore_result = True
    state.multiturn_restore_result_token = 7
    state.multiturn_restore_result_p4_time_ms = 900
    anchors = (7, [8192, 4096, 12288] + [0] * (MOTOR_COUNT - 3),
               [0] * MOTOR_COUNT, True)
    monkeypatch.setattr(controller, "_multiturn_recovery_anchors", lambda _state: anchors)
    return controller, comm, state, clock


def _advance(restore, seconds=2):
    _controller, _comm, state, clock = restore
    clock["mono"] += seconds
    clock["wall"] += seconds
    state.system_status_last_rx_ts = clock["wall"]
    stamp(state, 'system_status_last_rx_ts')
    state.servo_abs_last_rx_ts = clock["wall"]
    stamp(state, 'servo_abs_last_rx_ts')
    state.servo_feedback_p4_time_ms = state.system_status_p4_time_ms


def _drain(comm):
    commands = []
    while True:
        try:
            commands.append(comm.tx_queue.get_nowait())
        except queue.Empty:
            return commands


def _result(restore, restored, rejected, *, token=None):
    controller, _comm, state, clock = restore
    state.multiturn_restore_result_token = (
        controller._multiturn_request.token if token is None else token
    )
    state.multiturn_restore_restored_mask = restored
    state.multiturn_restore_rejected_mask = rejected
    state.multiturn_restore_result_last_rx_ts = clock["wall"] + 0.01
    stamp(state, 'multiturn_restore_result_last_rx_ts')
    state.multiturn_restore_result_p4_time_ms = state.p4_time_ms + 20
    state.has_servo_online_status = True
    state.servo_online_check = [1] * MOTOR_COUNT
    state.has_multiturn_restore_result = True



def test_request_sends_saved_native_abs_without_desktop_unwrapping(restore):
    controller, comm, state, _ = restore
    controller._service_multiturn_restore(state)
    request = _drain(comm)
    assert request[0][:2] == ('restore_multiturn', 7)
    assert request[0][2][:3] == [8192, 4096, 12288]
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []


def test_explicit_failure_is_reported_without_desktop_hardware_retry(restore):
    controller, comm, state, _ = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    _result(restore, 3, 4)
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert controller._multiturn_failed_mask == 4
    assert controller._multiturn_pending_restore_mask == 0
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []
    ok, message = controller.wait_for_multiturn_recovery(4, timeout_s=1)
    assert not ok and 'M02' in message


def test_import_rearms_only_its_scope_and_preserves_stop(restore):
    controller, comm, state, _ = restore
    controller._service_multiturn_restore(state); _drain(comm)
    _result(restore, 0, 7); _advance(restore)
    controller._service_multiturn_restore(state)
    controller.activate_calibration_snapshot({'active_motor_mask': 4}, Mock())
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][:2] == ('restore_multiturn', 4)
    assert controller._multiturn_failed_mask == 3
    controller.stop(); _drain(comm)
    controller.activate_calibration_snapshot({'active_motor_mask': 7}, Mock())
    assert controller._multiturn_restore_cancelled


def test_unknown_result_is_queried_once_without_another_write_and_can_arrive_late(restore):
    controller, comm, state, _ = restore
    controller._service_multiturn_restore(state); _drain(comm)
    controller.activate_calibration_snapshot({'active_motor_mask': 7}, Mock())
    _advance(restore, 9)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == [('state_sync', STATE_SYNC_MULTITURN_STATE)]
    for _ in range(4):
        _advance(restore)
        controller._service_multiturn_restore(state)
        assert _drain(comm) == []
    assert controller._multiturn_pending_restore_mask == 7
    _result(restore, 7, 0)
    controller._service_multiturn_restore(state)
    assert controller._multiturn_pending_restore_mask == 0


@pytest.mark.parametrize('invalid', ['old_token','jumped_token','old_host_time','old_p4_time','incomplete','overlap'])
def test_stale_or_invalid_final_result_cannot_complete_request(restore, invalid):
    controller, comm, state, clock = restore
    controller._service_multiturn_restore(state); _drain(comm)
    _result(restore, 7, 0)
    if invalid == 'old_token': state.multiturn_restore_result_token -= 1
    if invalid == 'jumped_token': state.multiturn_restore_result_token += 1
    if invalid == 'old_host_time':
        state.monotonic_received['multiturn_restore_result_last_rx_ts'] = controller._multiturn_request.sent_at
    if invalid == 'old_p4_time': state.multiturn_restore_result_p4_time_ms = 900
    if invalid == 'incomplete': state.multiturn_restore_restored_mask = 3
    if invalid == 'overlap': state.multiturn_restore_rejected_mask = 1
    controller._service_multiturn_restore(state)
    assert controller._multiturn_pending_restore_mask == 7
    assert _drain(comm) == []


@pytest.mark.parametrize('guard', ['control','owner','stale','missing_abs','calibration','rebase','conflict','blocked'])
def test_coordinate_and_feedback_guards(restore, guard):
    controller, comm, state, clock = restore
    if guard == 'control': state.control_enabled = True
    if guard == 'owner': state.servo_target_owner = 1
    if guard == 'stale': state.monotonic_received['system_status_last_rx_ts'] = clock['mono']-2
    if guard == 'missing_abs': state.has_servo_angle_data = False
    if guard == 'calibration': controller._host_calibration_io = SimpleNamespace(_session_finished=False,is_cancelled=lambda:False)
    if guard == 'rebase': controller._servo_rebase_zero_unresolved_mask = 1
    if guard == 'conflict': state.servo_route_conflict_bitmap = 7
    if guard == 'blocked': controller._block_multiturn_anchor_capture(7)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []


def test_negative_ack_completes_failed_operation_without_rewriting(restore):
    controller, comm, state, clock = restore
    controller._service_multiturn_restore(state); _drain(comm)
    event = dict(command=CMD_RESTORE_MULTITURN, sequence=1, received_monotonic=clock['mono'],
        p4_time_ms=1001, status=PROTO_ACK_STATUS_UNSUPPORTED_MODE)
    controller._service_multiturn_restore(state, [event])
    assert controller._multiturn_pending_restore_mask == 0
    assert controller._multiturn_failed_mask == 7
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []


def test_new_connection_does_not_match_previous_result(restore):
    controller, comm, state, _ = restore
    controller._service_multiturn_restore(state); _drain(comm)
    _result(restore, 7, 0)
    controller._servo_abs_zero_connection_generation += 1
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][:2] == ('restore_multiturn', 7)
    assert controller._multiturn_request.token == state.multiturn_restore_result_token + 1


def test_calibration_can_restore_only_its_explicit_preserved_helper_scope(restore):
    controller, comm, state, _clock = restore
    io = SimpleNamespace(_session_finished=False, is_cancelled=lambda: False, _helper_restore_mask=0)
    controller._host_calibration_io = io
    controller._service_multiturn_restore(state)
    assert not _drain(comm)
    io._helper_restore_mask = 2
    controller._service_multiturn_restore(state)
    commands = _drain(comm)
    assert len(commands) == 1 and commands[0][0:2] == ('restore_multiturn', 2)
    assert controller._multiturn_pending_restore_mask == 2
