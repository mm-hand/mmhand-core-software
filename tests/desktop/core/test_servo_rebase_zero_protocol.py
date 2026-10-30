"""设零协议、异步最终结果和新坐标锚点验证；不连接真实硬件。"""

from _clock_helpers import stamp

import struct
from unittest.mock import MagicMock, call

import pytest

import desktop.core.core_logic as core_logic
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.core_logic import HandController
from desktop.core.calibration_snapshot import (
    _empty_v7_snapshot, atomic_write_snapshot, read_snapshot, validate_snapshot,
)
from desktop.core.data_models import HandModel
from _bundle_helpers import SeededBundleStore
from desktop.core.protocol import (
    CMD_MOTOR_ABS_ZERO,
    HOST_PROTOCOL_VERSION,
    MOTOR_COUNT,
    PACKET_TYPE_PROTO_ACK,
    PACKET_TYPE_SERVO_REBASE_ZERO_RESULT,
    PROTO_ACK_STATUS_QUEUED,
    STATE_SYNC_MULTITURN_STATE,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RUN_DIRECT,
    build_servo_rebase_zero_cmd,
    build_motor_abs_zero_cmd,
    parse_servo_rebase_zero_result_packet,
    split_p4_timestamp,
)


def _result_payload(token=8, success=3, failed=0):
    return struct.pack(">III", token, success, failed) + bytes(MOTOR_COUNT * 4)


def test_command_encodes_selected_channels_with_fixed_big_endian_mask():
    mask = (1 << 0) | (1 << 26)
    assert build_servo_rebase_zero_cmd(mask) == (
        bytes((0xFE, 6, CMD_MOTOR_ABS_ZERO)) + struct.pack(">I", mask) + b"\xFF"
    )
    comm = LowerComputerComm("")
    assert comm._encode_command(("servo_rebase_zero", mask)) == build_servo_rebase_zero_cmd(mask)


def test_direct_zero_encodes_a_distinct_command_with_only_selected_motors():
    mask = (1 << 9) | (1 << 26)
    expected = bytes((0xFE, 6, CMD_MOTOR_ABS_ZERO)) + struct.pack(">I", mask) + b"\xFF"
    assert CMD_MOTOR_ABS_ZERO == 0xF1
    assert build_motor_abs_zero_cmd(mask) == expected
    assert LowerComputerComm("")._encode_command(("motor_abs_zero", mask)) == expected
    assert expected == build_servo_rebase_zero_cmd(mask)


@pytest.mark.parametrize("mask", [0, -1, 1 << 27, (1 << 31) | 1])
def test_direct_zero_rejects_invalid_motor_scope(mask):
    with pytest.raises(ValueError, match="non-zero valid motor mask"):
        build_motor_abs_zero_cmd(mask)


@pytest.mark.parametrize("mask", [0, -1, 1 << 27, (1 << 31) | 1, 1 << 32])
def test_command_rejects_empty_negative_or_outside_hardware_mask(mask):
    with pytest.raises(ValueError, match="non-zero valid motor mask"):
        build_servo_rebase_zero_cmd(mask)


def test_result_decodes_masks_with_zero_reserved_area_and_exact_length():
    payload = _result_payload(success=0x0F)
    assert len(payload) == 120
    assert parse_servo_rebase_zero_result_packet(payload) == (8, 0x0F, 0)
    assert parse_servo_rebase_zero_result_packet(payload[:-1]) is None
    assert parse_servo_rebase_zero_result_packet(payload + b"\x00") is None
    assert parse_servo_rebase_zero_result_packet(_result_payload(success=1, failed=1)) is None
    assert parse_servo_rebase_zero_result_packet(_result_payload(success=1 << 27)) is None


@pytest.mark.parametrize("index", [12, 15, 116, 119])
@pytest.mark.parametrize("success,failed", [(3, 0), (0, 3)])
def test_nonzero_reserved_area_cannot_publish_a_rebase_result(index, success, failed):
    payload = bytearray(_result_payload(success=success, failed=failed))
    payload[index] = 0xFF
    assert parse_servo_rebase_zero_result_packet(payload) is None


def test_result_routes_timestamp_and_preserves_independent_result_snapshot():
    comm = LowerComputerComm("")
    wire_payload = _result_payload() + struct.pack(">I", 1000)
    business, timestamp = split_p4_timestamp(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, wire_payload)
    assert business == _result_payload()
    assert timestamp == 1000
    updated = comm._process_packets([(PACKET_TYPE_SERVO_REBASE_ZERO_RESULT, wire_payload)])
    assert updated is not None
    assert updated.servo_rebase_zero_result_token == 8
    assert updated.servo_rebase_zero_success_mask == 3
    assert updated.servo_rebase_zero_failed_mask == 0
    assert updated.servo_rebase_zero_result_p4_time_ms == 1000
    assert updated.has_servo_rebase_zero_result
    snapshot = HandModel()
    snapshot.copy_state_from(updated)
    updated.servo_rebase_zero_success_mask = 1
    assert snapshot.servo_rebase_zero_success_mask == 3
    assert not hasattr(updated, "servo_rebase_zero_logical_offsets")


def test_queued_ack_leaves_result_and_coordinate_trust_unchanged():
    comm = LowerComputerComm("")
    previous = HandModel()
    previous.multi_turn_unknown_bitmap = 3
    previous.servo_rebase_zero_result_token = 7
    comm._state_reducer.last_model = previous
    updated = comm._process_packets([(
        PACKET_TYPE_PROTO_ACK,
        bytes((CMD_MOTOR_ABS_ZERO, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED))
        + struct.pack(">I", 1000),
    )])
    assert updated is not None
    assert updated.proto_ack_cmd == CMD_MOTOR_ABS_ZERO
    assert updated.servo_rebase_zero_result_token == 7
    assert updated.servo_rebase_zero_success_mask == 0
    assert updated.multi_turn_unknown_bitmap == 3


def _controller(monkeypatch):
    controller = HandController()
    controller.comm = MagicMock()
    controller.comm.send_command.return_value = True
    controller._invalidate_servo_rebase_zero_anchors = MagicMock()
    controller._persist_multiturn_state = MagicMock(return_value=True)
    state = controller.current_state
    state.system_state = SYSTEM_STATE_IDLE
    state.has_system_status_data = True
    state.system_status_last_rx_ts = core_logic.time.time()
    stamp(state, 'system_status_last_rx_ts')
    state.has_servo_route_status = True
    state.servo_route_controllable_bitmap = 3
    state.has_servo_rebase_zero_result = True
    state.servo_rebase_zero_result_token = 7
    state.servo_online_bitmap = 3
    state.has_product_fingerprint = True
    state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION, "device_id": "0000123456789ABC"}
    controller._servo_rebase_zero_device_id = state.product_fingerprint["device_id"]
    monkeypatch.setattr(core_logic, "SERVO_REBASE_ZERO_RESULT_TIMEOUT_S", 0.02)
    return controller


def _publish_final_result(controller, success=3, failed=0):
    state = controller.current_state
    state.servo_rebase_zero_result_token = 8
    state.servo_rebase_zero_success_mask = success
    state.servo_rebase_zero_failed_mask = failed
    state.servo_rebase_zero_result_last_rx_ts = core_logic.time.time()
    stamp(state, 'servo_rebase_zero_result_last_rx_ts')
    state.servo_rebase_zero_result_p4_time_ms = 1000


def test_api_waits_for_final_result_without_maintenance_reset(monkeypatch):
    controller = _controller(monkeypatch)

    def ack(cmd, _timeout, received_after_monotonic):
        assert received_after_monotonic > 0
        _publish_final_result(controller)
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, core_logic.time.time()

    controller.comm.wait_for_ack.side_effect = ack
    result = controller.rebase_servo_abs_zero(3)
    assert result.ok
    assert result.result_token == 8
    assert result.success_motor_mask == 3
    assert result.failed_motor_mask == 0
    assert not hasattr(result, "logical_offsets")
    controller.comm.send_command.assert_called_once_with(("servo_rebase_zero", 3))
    controller._invalidate_servo_rebase_zero_anchors.assert_called_once_with(3)
    assert controller._multiturn_anchor_capture_blocked_mask == 3


def test_api_accepts_final_result_even_when_ack_was_lost(monkeypatch):
    controller = _controller(monkeypatch)

    def ack(*_args, **_kwargs):
        _publish_final_result(controller)
        return None

    controller.comm.wait_for_ack.side_effect = ack
    assert controller.rebase_servo_abs_zero(3).ok
    assert controller.comm.send_command.call_args_list == [
        call(("servo_rebase_zero", 3)), call(("state_sync", STATE_SYNC_MULTITURN_STATE)),
    ]


def test_direct_zero_uses_f1_ack_and_the_shared_hardware_result(monkeypatch):
    controller = _controller(monkeypatch)

    def ack(cmd, _timeout, received_after_monotonic):
        assert cmd == CMD_MOTOR_ABS_ZERO
        _publish_final_result(controller)
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    assert controller.zero_direct_motor_abs(3).ok
    controller.comm.send_command.assert_called_once_with(("motor_abs_zero", 3))
    controller._invalidate_servo_rebase_zero_anchors.assert_called_once_with(3)


def test_direct_zero_cancelled_before_submission_preserves_feedback_and_anchors(monkeypatch):
    controller = _controller(monkeypatch)
    controller.current_state.servo_angles[0] = 8000
    assert not controller.zero_direct_motor_abs(1, cancelled=lambda: True).ok
    assert controller.current_state.servo_angles[0] == 8000
    assert controller._servo_rebase_zero_unresolved_mask == 0
    controller.comm.send_command.assert_not_called()
    controller._invalidate_servo_rebase_zero_anchors.assert_not_called()


def test_direct_zero_cancel_gate_restores_capture_state_without_sending(monkeypatch):
    controller = _controller(monkeypatch)
    controller._multiturn_anchor_capture_blocked_mask = 2
    controller._multiturn_anchor_capture_pending_mask = 1
    assert not controller.zero_direct_motor_abs(1, send_gate=lambda _: False).ok
    assert controller._servo_rebase_zero_unresolved_mask == 0
    assert controller._multiturn_anchor_capture_blocked_mask == 2
    assert controller._multiturn_anchor_capture_pending_mask == 1
    controller.comm.send_command.assert_not_called()


def test_direct_zero_timeout_also_blocks_calibration_rebase_retry(monkeypatch):
    controller = _controller(monkeypatch)
    monkeypatch.setattr(core_logic, "SERVO_REBASE_ZERO_RESULT_TIMEOUT_S", 0)
    controller.comm.wait_for_ack.return_value = (
        CMD_MOTOR_ABS_ZERO, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, 1.0,
    )
    assert not controller.zero_direct_motor_abs(3).ok
    assert not controller.rebase_servo_abs_zero(3).ok
    assert controller.comm.send_command.call_args_list.count(call(("motor_abs_zero", 3))) == 1
    assert call(("servo_rebase_zero", 3)) not in controller.comm.send_command.call_args_list


def test_direct_zero_anchor_saved_requires_runtime_validity_without_pending_capture(monkeypatch):
    controller = _controller(monkeypatch)
    controller._multiturn_store_payload = {"valid_motor_mask": "0x00000003"}
    assert controller.servo_abs_zero_anchor_saved(3)
    for attr in ("_servo_rebase_zero_unresolved_mask", "_servo_rebase_zero_capture_mask",
                 "_multiturn_anchor_capture_blocked_mask"):
        setattr(controller, attr, 1)
        assert not controller.servo_abs_zero_anchor_saved(3)
        assert controller.servo_abs_zero_anchor_saved(2)
        setattr(controller, attr, 0)
    assert not controller.servo_abs_zero_anchor_saved(4)


def _fresh_zero_feedback(controller):
    state = controller.current_state
    state.has_servo_angle_data = True
    state.has_servo_route_status = True
    state.servo_route_controllable_bitmap = 3
    state.servo_feedback_p4_time_ms = 1001
    if not state.has_product_fingerprint:
        state.has_product_fingerprint = True
        state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION, "device_id": "0000123456789ABC"}
    return state


def test_direct_zero_late_result_after_timeout_advances_only_saved_success(monkeypatch):
    controller = _controller(monkeypatch)
    monkeypatch.setattr(core_logic, "SERVO_REBASE_ZERO_RESULT_TIMEOUT_S", 0)
    controller.comm.wait_for_ack.return_value = (
        CMD_MOTOR_ABS_ZERO, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, 1.0,
    )
    assert not controller.zero_direct_motor_abs(3).ok
    assert not any(controller.get_servo_abs_zero_generations())
    _publish_final_result(controller, success=1, failed=2)
    state = _fresh_zero_feedback(controller)
    controller._service_servo_rebase_zero_capture(state)
    generations, feedback_ms, devices, connections, current_connection = controller.get_servo_abs_zero_generation_state()
    assert generations[:2] == (1, 0)
    assert feedback_ms[:2] == (1001, 0)
    assert devices[0] == "0000123456789ABC"
    assert connections[0] == current_connection
    controller._service_servo_rebase_zero_capture(state)
    assert controller.get_servo_abs_zero_generations() == generations


def test_direct_zero_cancel_after_submission_still_advances_when_late_anchor_is_saved(monkeypatch):
    controller = _controller(monkeypatch)
    cancelled = [False]

    def ack(cmd, _timeout, received_after_monotonic):
        cancelled[0] = True
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    assert not controller.zero_direct_motor_abs(3, cancelled=lambda: cancelled[0]).ok
    assert controller._servo_rebase_zero_unresolved_mask == 3
    assert not any(controller.get_servo_abs_zero_generations())
    _publish_final_result(controller)
    controller._service_servo_rebase_zero_capture(_fresh_zero_feedback(controller))
    assert controller.get_servo_abs_zero_generations()[:2] == (1, 1)


def test_direct_zero_generation_waits_for_save_and_does_not_repeat_on_more_feedback(monkeypatch):
    controller = _controller(monkeypatch)
    controller._servo_rebase_zero_capture_mask = 3
    controller._servo_rebase_zero_capture_after_ms = 1000
    controller._block_multiturn_anchor_capture(3)
    controller._persist_multiturn_state.side_effect = [False, True]
    state = _fresh_zero_feedback(controller)
    controller._service_servo_rebase_zero_capture(state)
    assert not any(controller.get_servo_abs_zero_generations())
    assert controller._servo_rebase_zero_capture_mask == 3
    controller._service_servo_rebase_zero_capture(state)
    assert controller.get_servo_abs_zero_generations()[:2] == (1, 1)
    state.servo_feedback_p4_time_ms = 1002
    controller._service_servo_rebase_zero_capture(state)
    assert controller.get_servo_abs_zero_generations()[:2] == (1, 1)
    assert controller._persist_multiturn_state.call_count == 2


def test_initial_or_reconnected_historical_result_does_not_advance_zero_generation(monkeypatch):
    controller = _controller(monkeypatch)
    for reconnect in (False, True):
        if reconnect:
            controller._reset_local_runtime_state("SYNCING")
        _publish_final_result(controller)
        state = _fresh_zero_feedback(controller)
        state.servo_online_bitmap = 3
        controller._service_servo_rebase_zero_capture(state)
        assert not any(controller.get_servo_abs_zero_generations())
    controller._persist_multiturn_state.assert_not_called()


def test_saved_zero_generation_remains_monotonic_across_local_runtime_reset(monkeypatch):
    controller = _controller(monkeypatch)
    controller._servo_rebase_zero_capture_mask = 3
    controller._servo_rebase_zero_capture_after_ms = 1000
    controller._service_servo_rebase_zero_capture(_fresh_zero_feedback(controller))
    generations = controller.get_servo_abs_zero_generations()
    controller._reset_local_runtime_state("SYNCING")
    assert isinstance(generations, tuple)
    assert controller.get_servo_abs_zero_generations() == generations


def test_direct_zero_different_device_result_cannot_complete_or_invalidate_original_request(monkeypatch):
    controller = _controller(monkeypatch)
    original_device = controller.current_state.product_fingerprint["device_id"]
    monkeypatch.setattr(core_logic, "SERVO_REBASE_ZERO_RESULT_TIMEOUT_S", 0)

    def other_device_ack(cmd, _timeout, received_after_monotonic):
        controller.current_state.product_fingerprint["device_id"] = "0000FEDCBA987654"
        _publish_final_result(controller)
        _fresh_zero_feedback(controller)
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = other_device_ack
    assert not controller.zero_direct_motor_abs(3).ok
    state = controller.current_state
    controller._service_servo_rebase_zero_capture(state)
    assert controller._servo_rebase_zero_unresolved_mask == 3
    assert controller._servo_rebase_zero_device_id == original_device
    assert not any(controller.get_servo_abs_zero_generations())
    controller._invalidate_servo_rebase_zero_anchors.assert_not_called()
    controller._persist_multiturn_state.assert_not_called()
    state.product_fingerprint["device_id"] = original_device
    controller._service_servo_rebase_zero_capture(state)
    assert controller._servo_rebase_zero_unresolved_mask == 0
    assert controller.get_servo_abs_zero_generations()[:2] == (1, 1)


def test_direct_zero_requires_a_verified_device_identity_before_submission(monkeypatch):
    controller = _controller(monkeypatch)
    controller.current_state.has_product_fingerprint = False
    result = controller.zero_direct_motor_abs(3)
    assert not result.ok and 'identity' in result.message
    controller.comm.send_command.assert_not_called()
    assert controller._servo_rebase_zero_unresolved_mask == 0


@pytest.mark.parametrize('token,success,failed', [(0, 3, 0), (8, 1, 0), (8, 7, 0)])
def test_reboot_history_or_incomplete_result_keeps_local_zero_request_unresolved(monkeypatch, token, success, failed):
    controller = _controller(monkeypatch)
    controller._servo_rebase_zero_unresolved_mask = 3
    controller._servo_rebase_zero_baseline_token = 7
    controller._servo_rebase_zero_sent_at = 0
    _publish_final_result(controller, success=success, failed=failed)
    state = _fresh_zero_feedback(controller)
    state.servo_rebase_zero_result_token = token
    controller._service_servo_rebase_zero_capture(state)
    assert controller._servo_rebase_zero_unresolved_mask == 3
    controller._invalidate_servo_rebase_zero_anchors.assert_not_called()
    controller._persist_multiturn_state.assert_not_called()


def test_api_partial_failure_exposes_success_and_failure_masks(monkeypatch):
    controller = _controller(monkeypatch)

    def ack(cmd, _timeout, received_after_monotonic):
        _publish_final_result(controller, success=1, failed=2)
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    result = controller.rebase_servo_abs_zero(3)
    assert not result.ok
    assert result.success_motor_mask == 1
    assert result.failed_motor_mask == 2
    assert not hasattr(result, "logical_offsets")


def test_timeout_queries_existing_transaction_and_blocks_blind_reexecution(monkeypatch):
    controller = _controller(monkeypatch)
    monkeypatch.setattr(core_logic, "SERVO_REBASE_ZERO_RESULT_TIMEOUT_S", 0)
    controller.comm.wait_for_ack.return_value = (
        CMD_MOTOR_ABS_ZERO, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, 1.0,
    )
    result = controller.rebase_servo_abs_zero(3)
    assert not result.ok
    assert controller._servo_rebase_zero_unresolved_mask == 3
    assert not controller.rebase_servo_abs_zero(3).ok
    sent = controller.comm.send_command.call_args_list
    assert sent.count(call(("servo_rebase_zero", 3))) == 1
    assert call(("state_sync", STATE_SYNC_MULTITURN_STATE)) in sent
    assert call("reset") not in sent


@pytest.mark.parametrize("state,enabled,owner", [
    (SYSTEM_STATE_RUN_DIRECT, True, 1), (SYSTEM_STATE_IDLE, False, 1),
])
def test_api_requires_stopped_state_without_target_owner(monkeypatch, state, enabled, owner):
    controller = _controller(monkeypatch)
    controller.current_state.system_state = state
    controller.current_state.control_enabled = enabled
    controller.current_state.servo_target_owner = owner
    assert not controller.rebase_servo_abs_zero(3).ok
    controller.comm.send_command.assert_not_called()


def test_new_anchor_uses_fresh_native_abs_after_final_result(monkeypatch):
    controller = _controller(monkeypatch)
    controller._servo_rebase_zero_unresolved_mask = 3
    controller._servo_rebase_zero_baseline_token = 7
    controller._servo_rebase_zero_sent_at = 0
    controller._block_multiturn_anchor_capture(3)
    _publish_final_result(controller)
    state = controller.current_state
    state.has_servo_angle_data = True
    state.servo_feedback_p4_time_ms = 999
    controller._service_servo_rebase_zero_capture(state)
    assert controller._multiturn_anchor_capture_blocked_mask == 3
    controller._persist_multiturn_state.assert_not_called()
    state.servo_feedback_p4_time_ms = 1001
    controller._service_servo_rebase_zero_capture(state)
    assert controller._multiturn_anchor_capture_blocked_mask == 0
    assert controller._multiturn_anchor_capture_pending_mask == 0
    controller._persist_multiturn_state.assert_called_once_with(state, force=True)


def test_blocked_rebase_scope_cannot_restore_old_json_anchor(monkeypatch):
    controller = _controller(monkeypatch)
    controller._multiturn_recovery_anchors = lambda _state: (
        3, [8192] * MOTOR_COUNT, [0] * MOTOR_COUNT, True,
    )
    state = controller.current_state
    state.multi_turn_unknown_bitmap = 3
    controller._block_multiturn_anchor_capture(3)
    controller.reconcile_multiturn_frame(3)
    controller.comm.send_command.assert_not_called()


def test_reconcile_loads_resealed_shifted_snapshot(tmp_path, monkeypatch):
    controller = _controller(monkeypatch)
    controller.current_state.multi_turn_unknown_bitmap = 1
    path = tmp_path / 'snapshot.json'
    snapshot = _empty_v7_snapshot()
    snapshot['multiturn_anchor_valid_bitmap'] = '0x1'
    snapshot['multiturn_anchor_logical_abs'][0] = 4096
    atomic_write_snapshot(path, snapshot)
    controller._multiturn_store = MagicMock(path=path)
    controller._multiturn_recovery_anchors = lambda _state: (
        1, [4096] + [0] * (MOTOR_COUNT - 1), [0] * MOTOR_COUNT, True,
    )
    def load(payload):
        validate_snapshot(payload)
        assert payload == read_snapshot(path)
        return True, 'loaded'
    controller.load_calibration_snapshot = load

    ok, _ = controller.reconcile_multiturn_frame(1)
    assert ok
    controller.comm.send_command.assert_called_once_with(('trust_multiturn', 1))


def test_reconcile_shifts_only_unknown_motors(tmp_path, monkeypatch):
    controller = _controller(monkeypatch)
    controller.current_state.multi_turn_unknown_bitmap = 1
    controller.current_state.servo_angles[:2] = [200, 2600]
    path = tmp_path / 'snapshot.json'
    snapshot = _empty_v7_snapshot()
    snapshot['multiturn_anchor_valid_bitmap'] = '0x3'
    snapshot['multiturn_anchor_logical_abs'][:2] = [5402, 5402]
    atomic_write_snapshot(path, snapshot)
    controller._multiturn_store = MagicMock(path=path)
    controller._multiturn_recovery_anchors = lambda _state: (
        3, read_snapshot(path)['multiturn_anchor_logical_abs'], [0] * MOTOR_COUNT, True,
    )
    controller.load_calibration_snapshot = MagicMock(return_value=(True, 'loaded'))
    assert controller.reconcile_multiturn_frame(3)[0]
    assert read_snapshot(path)['multiturn_anchor_logical_abs'][:2] == [1306, 5402]
    controller.comm.send_command.assert_called_once_with(('trust_multiturn', 1))


def test_new_anchor_save_failure_keeps_scope_blocked_and_retries(monkeypatch):
    controller = _controller(monkeypatch)
    controller._persist_multiturn_state = (
        HandController._persist_multiturn_state.__get__(controller)
    )
    controller._multiturn_identity = lambda _state: {"device": "same-hand"}
    controller._load_multiturn_store = MagicMock(return_value=None)
    controller._multiturn_store = MagicMock()
    controller._multiturn_store.save.side_effect = OSError("disk full")
    controller._servo_rebase_zero_capture_mask = 3
    controller._servo_rebase_zero_capture_after_ms = 1000
    controller._servo_rebase_zero_rebased_mask = 3
    controller._block_multiturn_anchor_capture(3)
    state = controller.current_state
    state.has_servo_angle_data = True
    state.servo_feedback_p4_time_ms = 1001

    controller._service_servo_rebase_zero_capture(state)
    assert state.host_multiturn_status == "save failed: disk full"
    assert controller._servo_rebase_zero_capture_mask == 3
    assert controller._multiturn_anchor_capture_blocked_mask == 3

    controller._multiturn_store.save.side_effect = None
    controller._multiturn_store.save.return_value = {"logical_abs": list(state.servo_angles)}
    controller._service_servo_rebase_zero_capture(state)
    assert controller._servo_rebase_zero_capture_mask == 0
    assert controller._multiturn_anchor_capture_blocked_mask == 0
    assert controller._multiturn_store.save.call_count == 2
    assert controller._multiturn_store.save.call_args.kwargs["logical_abs"][:2] == [0, 0]
    assert "raw" not in controller._multiturn_store.save.call_args.kwargs


@pytest.mark.parametrize("waiting_for_result", [True, False])
def test_reconnect_preserves_pending_rebase_and_blocks_old_anchor_restore(
    monkeypatch, waiting_for_result,
):
    controller = _controller(monkeypatch)
    controller._servo_rebase_zero_unresolved_mask = 3 if waiting_for_result else 0
    controller._servo_rebase_zero_capture_mask = 0 if waiting_for_result else 3
    controller._servo_rebase_zero_baseline_token = 7
    controller._servo_rebase_zero_sent_at = 123.0
    controller._servo_rebase_zero_capture_after_ms = 1000
    controller._servo_rebase_zero_rebased_mask = 0 if waiting_for_result else 3
    controller._block_multiturn_anchor_capture(3)

    controller._reset_local_runtime_state("SYNCING")
    assert controller._servo_rebase_zero_unresolved_mask == (3 if waiting_for_result else 0)
    assert controller._servo_rebase_zero_capture_mask == (0 if waiting_for_result else 3)
    assert controller._servo_rebase_zero_baseline_token == 7
    assert controller._servo_rebase_zero_sent_at == 123.0
    assert controller._servo_rebase_zero_capture_after_ms == 1000
    assert controller._multiturn_anchor_capture_blocked_mask == 3

    controller._multiturn_recovery_anchors = lambda _state: (
        3, [8192] * MOTOR_COUNT, [0] * MOTOR_COUNT, True,
    )
    state = controller.current_state
    state.servo_online_bitmap = state.multi_turn_unknown_bitmap = 3
    controller.reconcile_multiturn_frame(3)
    controller.comm.send_command.assert_not_called()


def test_rebase_invalidates_only_old_scope_and_captures_runtime_before_new_auto(
    tmp_path, monkeypatch,
):
    controller = HandController()
    controller._multiturn_store = SeededBundleStore(tmp_path / "runtime.json")
    identity = {
        "device_id": "0000123456789ABC",
        "urdf_repo_path": "model/hand.urdf",
        "product_fingerprint": {
            "schema_version": 2,
            "feedback_mapping_version": 3,
            "hardware_map_crc": 0x1234,
        },
    }
    controller._multiturn_identity = lambda _state: identity
    other = 1 << 26
    old_abs = [5000, 6000] + [0] * (MOTOR_COUNT - 3) + [9000]
    old_raw = [value % 4096 for value in old_abs]
    controller._multiturn_store_payload = controller._multiturn_store.save(
        identity=identity, valid_motor_mask=3 | other,
        logical_abs=old_abs, raw=old_raw, p4_time_ms=99,
    )
    snapshot_path = tmp_path / "calibration.mmcal.json"
    results = [None] * 21
    results[0] = {"joint": 0, "success": True}
    results[20] = {"joint": 20, "success": True}
    manual_min, manual_max = [123] * 21, [456] * 21
    atomic_write_snapshot(snapshot_path, {
        "joint_calib_results": results,
        "motor_abs_limit_min": [100] * MOTOR_COUNT,
        "motor_abs_limit_max": [1000] * MOTOR_COUNT,
        "motor_abs_limit_valid_bitmap": hex(3 | other),
        "manual_reference_min_raw": manual_min,
        "manual_reference_max_raw": manual_max,
        "manual_reference_valid_bitmap": "0x001FFFFF",
        "active_joint_mask": hex(1 | (1 << 20)),
        "active_motor_mask": hex(3 | other),
        "multiturn_anchor_valid_bitmap": hex(3 | other),
        "multiturn_anchor_logical_abs": old_abs,
        "multiturn_anchor_raw": old_raw,
    })
    monkeypatch.setattr(core_logic, "CALIB_ACTIVE_SNAPSHOT_PATH", snapshot_path)
    controller._block_multiturn_anchor_capture(3)
    controller._invalidate_servo_rebase_zero_anchors(3)
    runtime = controller._multiturn_store.load(identity=identity)
    snapshot = read_snapshot(snapshot_path)
    assert int(runtime["valid_motor_mask"], 0) == other
    assert snapshot["manual_reference_min_raw"] == manual_min
    assert snapshot["manual_reference_max_raw"] == manual_max
    assert snapshot["joint_calib_results"][0] is None
    assert snapshot["joint_calib_results"][20]["success"]
    assert int(snapshot["motor_abs_limit_valid_bitmap"], 0) == other
    assert int(snapshot["multiturn_anchor_valid_bitmap"], 0) == other

    state = controller.current_state
    state.has_servo_angle_data = True
    state.servo_online_bitmap = 3
    state.servo_angles[:2] = [2047, 2048]
    state.has_product_fingerprint = True
    state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION, "device_id": identity["device_id"]}
    controller._servo_rebase_zero_device_id = identity["device_id"]
    state.servo_feedback_p4_time_ms = 101
    state.p4_time_ms = 101
    controller._servo_rebase_zero_capture_mask = 3
    controller._servo_rebase_zero_capture_after_ms = 100
    controller._servo_rebase_zero_rebased_mask = 3
    controller._service_servo_rebase_zero_capture(state)
    runtime = controller._multiturn_store.load(identity=identity)
    assert int(runtime["valid_motor_mask"], 0) == 3 | other
    assert runtime["logical_abs"][:2] == [2047, 2048]
    assert runtime["raw"][:2] == [2047, 2048]
    assert runtime["logical_abs"][26] == 9000
    # 新 Auto 尚未生成，不提前把运行原点写成有效 Auto/LUT 快照。
    assert int(read_snapshot(snapshot_path)["multiturn_anchor_valid_bitmap"], 0) == other
    assert controller._multiturn_anchor_capture_pending_mask == 0
    stale = HandModel()
    stale.copy_state_from(state)
    stale.servo_angles[:2] = [5000, 6000]
    stale.servo_feedback_p4_time_ms = 99
    controller._persist_multiturn_state(stale, force=True)
    runtime = controller._multiturn_store.load(identity=identity)
    assert runtime["logical_abs"][:2] == [2047, 2048]
    assert runtime["raw"][:2] == [2047, 2048]


def test_reconcile_retries_load_without_shifting_twice(tmp_path, monkeypatch):
    controller = _controller(monkeypatch)
    controller.current_state.multi_turn_unknown_bitmap = 1
    path = tmp_path / 'snapshot.json'
    snapshot = _empty_v7_snapshot()
    snapshot['multiturn_anchor_valid_bitmap'] = '0x1'
    snapshot['multiturn_anchor_logical_abs'][0] = 4096
    atomic_write_snapshot(path, snapshot)
    controller._multiturn_store = MagicMock(path=path)
    controller._multiturn_recovery_anchors = lambda _state: (
        1, read_snapshot(path)['multiturn_anchor_logical_abs'], [0] * MOTOR_COUNT, True,
    )
    controller.load_calibration_snapshot = MagicMock(side_effect=[(False, 'timeout'), (True, 'loaded')])
    assert not controller.reconcile_multiturn_frame(1)[0]
    controller.comm.send_command.assert_not_called()
    assert controller.reconcile_multiturn_frame(1)[0]
    assert read_snapshot(path)['multiturn_anchor_logical_abs'][0] == 0
    assert controller.load_calibration_snapshot.call_count == 2
