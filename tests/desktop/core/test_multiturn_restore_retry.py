"""DA 恢复只重试已确认失败的电机；真实通信队列不连接硬件。"""

from _clock_helpers import stamp

import queue
import struct
from types import SimpleNamespace

import pytest

import desktop.core.core_logic as core_logic


def test_no_background_coordinate_write_during_calibration_preparation(restore):
    controller,comm,state,_=restore
    controller._host_calibration_io=SimpleNamespace(_session_finished=False,is_cancelled=lambda:False)
    controller._service_multiturn_restore(state)
    assert comm.tx_queue.empty()
    controller._host_calibration_io._session_finished=True
    controller._service_multiturn_restore(state)
    assert not comm.tx_queue.empty()
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.core_logic import HandController
from desktop.core.data_models import HandModel
from desktop.core.protocol import (
    HOST_PROTOCOL_VERSION,
    CMD_RESTORE_MULTITURN,
    MOTOR_COUNT,
    PACKET_TYPE_MULTITURN_RESTORE_RESULT,
    STATE_SYNC_MULTITURN_STATE,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RUN_DIRECT,
    PROTO_ACK_STATUS_UNSUPPORTED_MODE,
    parse_multiturn_restore_result_packet,
)


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
    state.system_status_p4_time_ms = state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = 1000
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
    state.has_servo_raw_data = True
    state.has_servo_angle_data = True
    state.servo_raw_online = [i < 3 for i in range(MOTOR_COUNT)]
    state.servo_raw_positions = [12] * MOTOR_COUNT
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
    state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = state.system_status_p4_time_ms


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
        controller._multiturn_pending_restore_expected_token if token is None else token
    )
    state.multiturn_restore_restored_mask = restored
    state.multiturn_restore_rejected_mask = rejected
    state.multiturn_restore_result_last_rx_ts = clock["wall"] + 0.01
    stamp(state, 'multiturn_restore_result_last_rx_ts')
    state.multiturn_restore_result_p4_time_ms = state.p4_time_ms + 20
    state.has_multiturn_restore_result = True


@pytest.mark.parametrize("changes", [
    {"has_servo_angle_data": False},
    {"has_servo_raw_data": False},
    {"servo_raw_p4_time_ms": 999},
    {"servo_abs_last_rx_ts": 98.0},
    {"servo_abs_last_rx_ts": 101.0},
    {"servo_feedback_p4_time_ms": 10, "servo_raw_p4_time_ms": 10, "system_status_p4_time_ms": 2000},
    {"servo_feedback_p4_time_ms": 3000, "servo_raw_p4_time_ms": 3000, "system_status_p4_time_ms": 1000},
])
def test_restore_does_not_write_abs_from_stale_or_unpaired_raw_snapshot(restore, changes):
    controller, comm, state, _clock = restore
    for field, value in changes.items():
        setattr(state, field, value)
        if field.endswith('last_rx_ts'):
            stamp(state, field)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []
    assert controller._multiturn_pending_restore_mask == 0


def test_restore_accepts_fresh_paired_raw_abs_across_p4_clock_wrap(restore):
    controller, comm, state, _clock = restore
    state.system_status_p4_time_ms = 20
    state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = 0xfffffff0
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][:2] == ("restore_multiturn", 7)


def test_partial_results_retry_only_failed_subset_then_report_specific_exhaustion(restore):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    first = _drain(comm)
    assert first[0][0:2] == ("restore_multiturn", 7)
    assert first[0][2][:3] == [8204, 4108, 12300]

    _result(restore, 1, 6)
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 6)
    _result(restore, 2, 4)
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 4)
    _result(restore, 0, 4)
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []
    assert controller._multiturn_restore_attempts[:3] == [1, 2, 3]
    assert controller._multiturn_restore_confirmed_mask == 3
    assert controller._multiturn_restore_exhausted_mask == 4
    ok, message = controller.wait_for_multiturn_recovery(4, timeout_s=10)
    assert not ok and "3 requests" in message and "M02" in message


def test_unknown_result_is_queried_three_times_without_another_write_and_can_arrive_late(restore):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 7)
    _advance(restore, 9)
    for _attempt in range(3):
        controller._service_multiturn_restore(state)
        assert _drain(comm) == [("state_sync", STATE_SYNC_MULTITURN_STATE)]
        _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []

    assert controller._multiturn_pending_restore_mask == 7
    assert controller._multiturn_restore_attempts[:3] == [1, 1, 1]
    ok, message = controller.wait_for_multiturn_recovery(7, timeout_s=10)
    assert not ok and "unknown after 3 queries" in message and "M00,M01,M02" in message
    _result(restore, 7, 0)
    controller._service_multiturn_restore(state)
    assert controller._multiturn_pending_restore_mask == 0
    assert controller._multiturn_restore_confirmed_mask == 7
    assert _drain(comm) == []


def _nack(restore, sequence=1):
    controller, _comm, state, clock = restore
    state.proto_ack_sequence = sequence
    return {"sequence": sequence, "command": CMD_RESTORE_MULTITURN,
            "mode": state.system_state, "status": PROTO_ACK_STATUS_UNSUPPORTED_MODE,
            "received_at": clock["wall"], "received_monotonic": clock["mono"], "p4_time_ms": state.p4_time_ms + 10}


def test_explicit_nack_is_confirmed_failure_and_consumes_at_most_three_write_requests(restore):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    for sequence in range(1, 4):
        _advance(restore)
        controller._service_multiturn_restore(state, ack_events=[_nack(restore, sequence)])
        queued = _drain(comm)
        assert [cmd[0:2] for cmd in queued] == ([] if sequence == 3 else [("restore_multiturn", 7)])
    assert controller._multiturn_pending_restore_mask == 0
    assert controller._multiturn_restore_attempts[:3] == [3, 3, 3]
    assert controller._multiturn_restore_exhausted_mask == 7


@pytest.mark.parametrize("invalid", ["wrong_command", "old_sequence", "old_host_time", "old_p4_time"])
def test_unrelated_or_stale_nack_cannot_resolve_pending(restore, invalid):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    _advance(restore)
    event = _nack(restore)
    if invalid == "wrong_command":
        event["command"] = 0xF0
    elif invalid == "old_sequence":
        event["sequence"] = controller._multiturn_pending_restore_ack_sequence
    elif invalid == "old_host_time":
        event["received_monotonic"] = controller._multiturn_pending_restore_sent_at - 0.01
    else:
        event["p4_time_ms"] = 999
    controller._service_multiturn_restore(state, ack_events=[event])
    assert controller._multiturn_pending_restore_mask == 7
    assert controller._multiturn_restore_confirmed_mask == 0
    assert _drain(comm) == []


def test_full_final_result_takes_precedence_over_a_negative_ack(restore):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    _advance(restore)
    _result(restore, 7, 0)
    controller._service_multiturn_restore(state, ack_events=[_nack(restore)])
    assert controller._multiturn_pending_restore_mask == 0
    assert controller._multiturn_restore_confirmed_mask == 7
    assert _drain(comm) == []


@pytest.mark.parametrize("invalid", ["old_token", "jumped_token", "old_host_time", "old_p4_time",
                                     "missing_result", "incomplete_mask", "extra_mask", "overlap"])
def test_unmatched_final_result_never_clears_pending_or_retries(restore, invalid):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    _result(restore, 1, 6)
    if invalid == "old_token":
        state.multiturn_restore_result_token = 7
    elif invalid == "jumped_token":
        state.multiturn_restore_result_token = 9
    elif invalid == "old_host_time":
        state.multiturn_restore_result_last_rx_ts = controller._multiturn_pending_restore_sent_at
        stamp(state, 'multiturn_restore_result_last_rx_ts')
    elif invalid == "old_p4_time":
        state.multiturn_restore_result_p4_time_ms = 999
    elif invalid == "missing_result":
        state.has_multiturn_restore_result = False
    elif invalid == "incomplete_mask":
        state.multiturn_restore_rejected_mask = 2
    elif invalid == "extra_mask":
        state.multiturn_restore_rejected_mask = 14
    else:
        state.multiturn_restore_rejected_mask = 7
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []
    assert controller._multiturn_pending_restore_mask == 7
    assert controller._multiturn_restore_confirmed_mask == 0


def test_result_token_and_p4_clock_wrap_are_matched_modulo_u32(restore):
    controller, comm, state, _clock = restore
    state.multiturn_restore_result_token = 0xFFFFFFFF
    state.p4_time_ms = 0xFFFFFFF0
    controller._service_multiturn_restore(state)
    _drain(comm)
    _result(restore, 7, 0, token=0)
    state.multiturn_restore_result_p4_time_ms = 4
    controller._service_multiturn_restore(state)
    assert controller._multiturn_pending_restore_mask == 0
    assert controller._multiturn_restore_confirmed_mask == 7


@pytest.mark.parametrize("rejection", ["offline", "incompatible", "queue_full"])
def test_enqueue_rejection_does_not_consume_an_attempt_or_register_pending(restore, rejection):
    controller, comm, state, clock = restore
    if rejection == "offline":
        clock["connected"] = False
    elif rejection == "incompatible":
        comm._protocol_compatible = False
    else:
        comm.tx_queue = queue.Queue(maxsize=1)
        comm.tx_queue.put_nowait("occupied")
    controller._service_multiturn_restore(state)
    assert controller._multiturn_restore_attempts[:3] == [0, 0, 0]
    assert controller._multiturn_pending_restore_mask == 0


def test_rejected_queries_do_not_consume_successful_query_limit(restore):
    controller, comm, state, clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    clock["connected"] = False
    _advance(restore, 9)
    controller._service_multiturn_restore(state)
    assert controller._multiturn_pending_restore_query_count == 0
    assert controller._multiturn_pending_restore_mask == 7
    clock["connected"] = True
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == [("state_sync", STATE_SYNC_MULTITURN_STATE)]
    assert controller._multiturn_pending_restore_query_count == 1


def test_missing_baseline_is_queried_before_any_hardware_write(restore):
    controller, comm, state, _clock = restore
    state.has_multiturn_restore_result = False
    controller._service_multiturn_restore(state)
    assert _drain(comm) == [("state_sync", STATE_SYNC_MULTITURN_STATE)]
    assert controller._multiturn_restore_attempts[:3] == [0, 0, 0]
    state.has_multiturn_restore_result = True
    state.multiturn_restore_result_token = 41
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 7)
    assert controller._multiturn_pending_restore_expected_token == 42


@pytest.mark.parametrize("guard", ["no_status", "stale_status", "future_status", "no_routes",
                                   "no_raw", "no_p4_time", "route_scan", "active", "paused", "owner", "enabled",
                                   "f0_pending", "f0_capture", "ee_pending", "cancelled"])
def test_nonstationary_or_unresolved_coordinate_transaction_prevents_restore(restore, guard):
    controller, comm, state, clock = restore
    if guard == "no_status":
        state.has_system_status_data = False
    elif guard == "stale_status":
        state.system_status_last_rx_ts = clock["wall"] - 1.1
        stamp(state, 'system_status_last_rx_ts')
    elif guard == "future_status":
        state.system_status_last_rx_ts = clock["wall"] + 0.1
        stamp(state, 'system_status_last_rx_ts')
    elif guard == "no_routes":
        state.has_servo_route_status = False
    elif guard == "no_raw":
        state.has_servo_raw_data = False
    elif guard == "no_p4_time":
        state.has_p4_time = False
    elif guard == "route_scan":
        state.servo_route_scan_state = 1
    elif guard == "active":
        state.system_state = SYSTEM_STATE_RUN_DIRECT
    elif guard == "paused":
        state.system_state = 7  # Retired wire state is rejected; no pause compatibility.
    elif guard == "owner":
        state.servo_target_owner = 1
    elif guard == "enabled":
        state.control_enabled = True
    elif guard == "f0_pending":
        controller._servo_rebase_zero_unresolved_mask = 1
    elif guard == "f0_capture":
        controller._servo_rebase_zero_capture_mask = 1
    elif guard == "ee_pending":
        controller._servo_turn_zero_unresolved_mask = 1
    else:
        controller._multiturn_restore_cancelled = True
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []
    assert controller._multiturn_pending_restore_mask == 0


def test_candidate_excludes_unroutable_raw_offline_and_blocked_motors(restore):
    controller, comm, state, _clock = restore
    state.servo_route_controllable_bitmap = 3
    state.servo_raw_online[1] = False
    controller._multiturn_anchor_capture_blocked_mask = 4
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 1)


def test_conflicting_route_and_invalid_raw_are_never_restored(restore):
    controller, comm, state, _clock = restore
    state.servo_route_conflict_bitmap = 1
    state.servo_raw_positions[1] = -1
    controller._service_multiturn_restore(state)
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 4)


@pytest.mark.parametrize("change", ["device", "connection", "comm"])
def test_new_device_or_connection_cannot_accept_previous_context_as_completed(restore, change, monkeypatch):
    controller, comm, state, clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    old_context = controller._multiturn_pending_restore_context
    _result(restore, 7, 0)
    if change == "device":
        state.product_fingerprint["device_id"] = "0000123456789ABD"
    elif change == "connection":
        controller._servo_abs_zero_connection_generation += 1
    else:
        comm = LowerComputerComm("")
        comm._protocol_compatible = True
        monkeypatch.setattr(comm, "is_connected", lambda: clock["connected"])
        controller.comm = comm
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert controller._multiturn_restore_confirmed_mask == 0
    assert controller._multiturn_pending_restore_context != old_context
    assert controller._multiturn_pending_restore_expected_token == 9
    assert controller._multiturn_restore_attempts[:3] == [1, 1, 1]
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 7)


def test_offline_motor_resets_only_its_own_attempt_budget(restore):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    _result(restore, 0, 7)
    state.servo_online_bitmap = 6
    _advance(restore)
    controller._service_multiturn_restore(state)
    assert controller._multiturn_restore_attempts[:3] == [0, 2, 2]
    assert _drain(comm)[0][0:2] == ("restore_multiturn", 6)


def test_stop_halts_retry_and_queries_while_retaining_already_sent_result(restore):
    controller, comm, state, _clock = restore
    controller._service_multiturn_restore(state)
    _drain(comm)
    controller.stop()
    assert _drain(comm) == ["stop"]
    _advance(restore, 9)
    controller._service_multiturn_restore(state)
    assert _drain(comm) == []
    assert controller._multiturn_pending_restore_mask == 7
    _result(restore, 1, 6)
    controller._service_multiturn_restore(state)
    assert controller._multiturn_pending_restore_mask == 0
    assert controller._multiturn_restore_confirmed_mask == 1
    assert _drain(comm) == []


@pytest.mark.parametrize("restored,rejected", [(1, 1), (1 << MOTOR_COUNT, 0), (0, 1 << 31)])
def test_restore_parser_rejects_overlapping_or_nonexistent_motor_masks(restored, rejected):
    assert parse_multiturn_restore_result_packet(struct.pack(">III", 8, restored, rejected)) is None


def test_restore_reducer_keeps_p4_time_and_result_presence_in_copied_model():
    comm = LowerComputerComm("")
    updated = comm._process_packets([(
        PACKET_TYPE_MULTITURN_RESTORE_RESULT,
        struct.pack(">IIII", 8, 1, 6, 1200),
    )])
    assert updated is not None
    assert updated.has_multiturn_restore_result
    assert updated.multiturn_restore_result_p4_time_ms == 1200
    assert updated.multiturn_restore_result_last_rx_ts > 0
    snapshot = HandModel()
    snapshot.copy_state_from(updated)
    updated.multiturn_restore_restored_mask = 7
    assert snapshot.multiturn_restore_restored_mask == 1
    assert snapshot.has_multiturn_restore_result
    assert snapshot.multiturn_restore_result_p4_time_ms == 1200
