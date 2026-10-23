"""Upper-computer coordinate writes retry failed motors without repeating successes."""

from _clock_helpers import stamp

import time
from unittest.mock import Mock, call

import pytest

from desktop.core import core_logic
from desktop.core.core_logic import HandController
from desktop.core.calibration_snapshot import atomic_write_snapshot, read_snapshot
from desktop.core.device_state_store import MultiTurnRuntimeStore
from desktop.core.protocol import (
    CMD_RESET, CMD_MOTOR_ABS_ZERO, CMD_SERVO_RESET_ZERO,
    HOST_PROTOCOL_VERSION, PROTO_ACK_STATUS_QUEUED, STATE_SYNC_MULTITURN_STATE,
    SYSTEM_STATE_IDLE,
)


@pytest.fixture
def controller(monkeypatch):
    result = HandController()
    result.comm = Mock()
    result.comm.send_command.return_value = True
    state = result.current_state
    state.has_product_fingerprint = True
    state.product_fingerprint = {"device_id": "0000123456789ABC", "protocol_version": HOST_PROTOCOL_VERSION}
    state.has_system_status_data = state.has_servo_route_status = True
    state.system_state = SYSTEM_STATE_IDLE
    state.system_status_last_rx_ts = time.time()
    stamp(state, 'system_status_last_rx_ts')
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = 7
    state.has_servo_rebase_zero_result = state.has_servo_reset_zero_status = True
    state.servo_rebase_zero_result_token = 7
    state.servo_reset_zero_request_token = state.servo_reset_zero_result_token = 7
    state.has_servo_angle_data = state.has_servo_raw_data = True
    state.servo_raw_online[:3] = [True] * 3
    state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = 100
    result._invalidate_servo_rebase_zero_anchors = Mock()
    result._invalidate_multiturn_anchor_mask = Mock()
    result._persist_multiturn_state = Mock(return_value=True)
    monkeypatch.setattr(core_logic, "invalidate_calibration_snapshot_scope", Mock())
    monkeypatch.setattr(core_logic, "SERVO_REBASE_ZERO_RESULT_TIMEOUT_S", 0)
    monkeypatch.setattr(core_logic, "SERVO_RESET_ZERO_RESULT_TIMEOUT_S", 0)
    monkeypatch.setattr(core_logic, "CALIBRATION_TURN_RESET_SETTLE_S", 0)
    monkeypatch.setattr(core_logic, "MULTITURN_ANCHOR_CAPTURE_TIMEOUT_S", 0)
    return result


def publish(controller, mask, success, *, ee=False, token=None):
    state = controller.current_state
    state.system_status_last_rx_ts = time.time()
    stamp(state, 'system_status_last_rx_ts')
    if ee:
        token = state.servo_reset_zero_result_token + 1 if token is None else token
        state.servo_reset_zero_request_token = state.servo_reset_zero_result_token = token
        state.servo_reset_zero_success_mask = success
        state.servo_reset_zero_failed_mask = mask & ~success
        state.servo_reset_zero_status = 2 if success == mask else 3
    else:
        token = state.servo_rebase_zero_result_token + 1 if token is None else token
        state.servo_rebase_zero_result_token = token
        state.servo_rebase_zero_success_mask = success
        state.servo_rebase_zero_failed_mask = mask & ~success
        state.servo_rebase_zero_result_last_rx_ts = time.time()
        stamp(state, 'servo_rebase_zero_result_last_rx_ts')
        state.servo_rebase_zero_result_p4_time_ms = token * 100
        state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = token * 100 + 1
    state.multi_turn_unknown_bitmap = mask & ~success
    return state


def scripted_ack(controller, successes, *, ee=False):
    outcomes = iter(successes)
    command_name = "servo_reset_zero" if ee else "servo_rebase_zero"

    def ack(command, _timeout, received_after_monotonic):
        if command == CMD_RESET:
            return command, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic
        commands = [c.args[0] for c in controller.comm.send_command.call_args_list
                    if isinstance(c.args[0], tuple) and c.args[0][0] in (command_name, "motor_abs_zero")]
        mask = commands[-1][1]
        publish(controller, mask, next(outcomes), ee=ee)
        return command, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack


@pytest.mark.parametrize("direct", [False, True])
def test_retries_only_failed_subset_and_merges_successes(controller, direct):
    scripted_ack(controller, [1, 2, 4])
    result = controller.zero_direct_motor_abs(7) if direct else controller.rebase_servo_abs_zero(7)
    command = "motor_abs_zero" if direct else "servo_rebase_zero"
    assert result.ok and result.success_motor_mask == 7 and result.failed_motor_mask == 0
    assert result.result_token == 10
    assert controller.comm.send_command.call_args_list == [
        call((command, 7)), call((command, 6)), call((command, 4)),
    ]
    assert controller.get_servo_abs_zero_generations()[:3] == (1, 1, 1)
    assert controller._servo_rebase_zero_unresolved_mask == 0


def test_three_failed_writes_preserve_first_success_and_report_remaining_motor(controller):
    scripted_ack(controller, [1, 0, 0])
    result = controller.rebase_servo_abs_zero(3)
    assert not result.ok and result.success_motor_mask == 1 and result.failed_motor_mask == 2
    assert result.result_token == 10 and "3 attempts" in result.message and "M01" in result.message
    assert controller.comm.send_command.call_args_list == [
        call(("servo_rebase_zero", 3)), call(("servo_rebase_zero", 2)), call(("servo_rebase_zero", 2)),
    ]
    assert controller.get_servo_abs_zero_generations()[:2] == (1, 0)


@pytest.mark.parametrize("direct", [False, True])
def test_fresh_rejections_try_three_times_without_invalidating_old_coordinates(controller, direct):
    controller._multiturn_anchor_capture_blocked_mask = 2
    controller._multiturn_anchor_capture_pending_mask = 1
    controller.comm.wait_for_ack.side_effect = (
        lambda cmd, _timeout, received_after_monotonic: (cmd, SYSTEM_STATE_IDLE, 1, received_after_monotonic))
    result = controller.zero_direct_motor_abs(1) if direct else controller.rebase_servo_abs_zero(1)
    command = "motor_abs_zero" if direct else "servo_rebase_zero"
    assert not result.ok and "rejected after 3" in result.message
    assert controller.comm.send_command.call_args_list == [call((command, 1))] * 3
    assert controller._servo_rebase_zero_unresolved_mask == 0
    assert controller._multiturn_anchor_capture_blocked_mask == 2
    assert controller._multiturn_anchor_capture_pending_mask == 1
    controller._invalidate_servo_rebase_zero_anchors.assert_not_called()


@pytest.mark.parametrize("ack_kind", ["lost", "wrong_command", "old_timestamp"])
def test_unknown_result_queries_three_times_and_never_rewrites(controller, ack_kind):
    def ack(cmd, _timeout, received_after_monotonic):
        if ack_kind in ("lost", "old_timestamp"):  # Tracker filters pre-send ACKs.
            return None
        return (CMD_SERVO_RESET_ZERO if ack_kind == "wrong_command" else cmd,
                SYSTEM_STATE_IDLE, 1, received_after_monotonic - 1 if ack_kind == "old_timestamp" else received_after_monotonic)

    controller.comm.wait_for_ack.side_effect = ack
    result = controller.rebase_servo_abs_zero(3)
    assert not result.ok and controller._servo_rebase_zero_unresolved_mask == 3
    commands = controller.comm.send_command.call_args_list
    assert commands.count(call(("servo_rebase_zero", 3))) == 1
    assert commands.count(call(("state_sync", STATE_SYNC_MULTITURN_STATE))) == 3
    assert not controller.zero_direct_motor_abs(3).ok
    assert controller.comm.send_command.call_args_list.count(call(("servo_rebase_zero", 3))) == 1


def test_second_result_query_recovers_original_transaction_without_new_write(controller):
    controller.comm.wait_for_ack.return_value = None
    queries = 0

    def send(command):
        nonlocal queries
        if command == ("state_sync", STATE_SYNC_MULTITURN_STATE):
            queries += 1
            if queries == 2:
                publish(controller, 3, 3)
        return True

    controller.comm.send_command.side_effect = send
    assert controller.rebase_servo_abs_zero(3).ok
    assert queries == 2
    assert controller.comm.send_command.call_args_list.count(call(("servo_rebase_zero", 3))) == 1


@pytest.mark.parametrize("change", ["stop", "owner", "device", "connection"])
def test_cancellation_or_safety_change_after_result_prevents_next_write(controller, change):
    scripted_ack(controller, [1])
    capture = controller._service_servo_rebase_zero_capture

    def changed_capture(state):
        capture(state)
        if change == "stop":
            controller.stop()
        elif change == "owner":
            controller.current_state.servo_target_owner = 1
        elif change == "device":
            controller.current_state.product_fingerprint["device_id"] = "0000FEDCBA987654"
        else:
            controller._servo_abs_zero_connection_generation += 1

    controller._service_servo_rebase_zero_capture = changed_capture
    result = controller.rebase_servo_abs_zero(3)
    assert not result.ok and result.success_motor_mask == 1 and result.failed_motor_mask == 2
    commands = [c.args[0] for c in controller.comm.send_command.call_args_list
                if isinstance(c.args[0], tuple) and c.args[0][0] == "servo_rebase_zero"]
    assert commands == [("servo_rebase_zero", 3)]


def test_unsaved_success_blocks_retry_without_repeating_zero(controller):
    scripted_ack(controller, [1])
    controller._persist_multiturn_state.return_value = False
    result = controller.rebase_servo_abs_zero(3)
    assert not result.ok and result.success_motor_mask == 1 and "save unconfirmed" in result.message
    assert controller._servo_rebase_zero_capture_mask == 1
    controller.comm.send_command.assert_called_once_with(("servo_rebase_zero", 3))


@pytest.mark.parametrize("bad_result", ["old_token", "skipped_token", "missing_motor", "extra_motor", "old_rx"])
def test_unrelated_final_result_does_not_complete_or_allow_retry(controller, bad_result):
    def ack(cmd, _timeout, received_after_monotonic):
        state = publish(controller, 3, 3)
        if bad_result == "old_token":
            state.servo_rebase_zero_result_token = 7
        elif bad_result == "skipped_token":
            state.servo_rebase_zero_result_token = 9
        elif bad_result == "missing_motor":
            state.servo_rebase_zero_success_mask = 1
        elif bad_result == "extra_motor":
            state.servo_rebase_zero_success_mask = 7
        else:
            state.servo_rebase_zero_result_last_rx_ts = received_after_monotonic - 1
            stamp(state, 'servo_rebase_zero_result_last_rx_ts')
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    assert not controller.rebase_servo_abs_zero(3).ok
    assert controller._servo_rebase_zero_unresolved_mask == 3
    assert controller.comm.send_command.call_args_list.count(call(("servo_rebase_zero", 3))) == 1


def test_ee_retries_failed_motors_with_one_entry_reset(controller):
    scripted_ack(controller, [1, 2, 4], ee=True)
    ok, _message, success, failed = controller.prepare_calibration_turn_zero(7)
    assert ok and success == 7 and failed == 0
    assert controller.comm.send_command.call_args_list == [
        call("reset"), call(("servo_reset_zero", 7)),
        call(("servo_reset_zero", 6)), call(("servo_reset_zero", 4)),
    ]
    assert controller._servo_turn_zero_unresolved_mask == 0


def test_ee_unknown_result_is_queried_but_never_reset_or_written_again(controller):
    controller.comm.wait_for_ack.side_effect = (
        lambda cmd, _timeout, received_after_monotonic:
        (cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic))
    result = controller.prepare_calibration_turn_zero(3)
    assert not result[0] and controller._servo_turn_zero_unresolved_mask == 3
    assert not controller.prepare_calibration_turn_zero(3)[0]
    calls = controller.comm.send_command.call_args_list
    assert calls.count(call("reset")) == calls.count(call(("servo_reset_zero", 3))) == 1
    assert calls.count(call(("state_sync", STATE_SYNC_MULTITURN_STATE))) == 3
    assert not controller.rebase_servo_abs_zero(3).ok


def test_ee_fresh_nack_restores_anchor_gate_and_retries_without_another_reset(controller):
    controller._multiturn_anchor_capture_blocked_mask = 2
    controller._multiturn_anchor_capture_pending_mask = 1
    controller.comm.wait_for_ack.side_effect = (
        lambda cmd, _timeout, received_after_monotonic:
        (cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED if cmd == CMD_RESET else 1, received_after_monotonic))
    assert not controller.prepare_calibration_turn_zero(1)[0]
    assert controller.comm.send_command.call_args_list == [call("reset")] + [call(("servo_reset_zero", 1))] * 3
    assert controller._servo_turn_zero_unresolved_mask == 0
    assert controller._multiturn_anchor_capture_blocked_mask == 2
    assert controller._multiturn_anchor_capture_pending_mask == 1
    controller._invalidate_multiturn_anchor_mask.assert_not_called()


def test_stop_after_ee_reset_prevents_zero_write(controller):
    def ack(cmd, _timeout, received_after_monotonic):
        controller.stop()
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    assert not controller.prepare_calibration_turn_zero(3)[0]
    assert controller.comm.send_command.call_args_list == [call("reset"), call("stop")]


def test_late_ee_result_resolves_original_transaction_without_new_hardware_write(controller):
    controller.comm.wait_for_ack.side_effect = (
        lambda cmd, _timeout, received_after_monotonic:
        (cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic))
    assert not controller.prepare_calibration_turn_zero(3)[0]
    controller.comm.send_command.reset_mock()
    controller._service_servo_turn_zero_result(publish(controller, 3, 1, ee=True))
    assert controller._servo_turn_zero_unresolved_mask == 0
    controller.comm.send_command.assert_not_called()


@pytest.mark.parametrize("command_name", ["servo_rebase_zero", "motor_abs_zero", "servo_reset_zero"])
@pytest.mark.parametrize("ending", ["stop", "caller_cancel", "timeout"])
def test_unknown_or_cancelled_write_invalidates_runtime_and_device_auto_backup(
    controller, monkeypatch, tmp_path, command_name, ending,
):
    device_id = controller.current_state.product_fingerprint["device_id"]
    device_dir = tmp_path / device_id / "model"
    runtime = MultiTurnRuntimeStore(device_dir / "multiturn_runtime.json")
    identity = {"device_id": device_id, "urdf_repo_path": "models/hand.urdf",
                "product_fingerprint": {"schema_version": 2}}
    controller._multiturn_store = runtime
    controller._multiturn_identity = lambda _state: identity
    controller._multiturn_store_payload = runtime.save(
        identity=identity, valid_motor_mask=7, logical_abs=[9000] * 27,
        raw=[808] * 27, p4_time_ms=100,
    )
    snapshot_name = core_logic.CALIB_ACTIVE_SNAPSHOT_PATH.name
    snapshot_path = device_dir / snapshot_name
    foreign_snapshot = tmp_path / "0000FEDCBA987654" / "model" / snapshot_name
    snapshot = {"multiturn_anchor_valid_bitmap": "0x00000007", "multiturn_anchor_logical_abs": [9000] * 27,
                "multiturn_anchor_raw": [808] * 27, "active_motor_mask": "0x00000007",
                "active_joint_mask": "0x00000003", "motor_abs_limit_valid_bitmap": "0x00000007",
                "motor_abs_limit_min": [0] * 27, "motor_abs_limit_max": [20000] * 27,
                "device_id": device_id, "urdf_repo_path": identity["urdf_repo_path"],
                "joint_calib_results": [{"joint": 0, "success": True},
                                        {"joint": 1, "success": True}] + [None] * 19,
                "manual_reference_min_raw": [123] * 21,
                "manual_reference_max_raw": [456] * 21,
                "manual_reference_valid_bitmap": "0x001FFFFF"}
    atomic_write_snapshot(snapshot_path, snapshot)
    atomic_write_snapshot(foreign_snapshot, snapshot)
    # 全局旧路径和当前设备路径不同，不能错误失效全局文件或另一设备。
    legacy_path = tmp_path / "legacy" / snapshot_name
    atomic_write_snapshot(legacy_path, snapshot)
    monkeypatch.setattr(core_logic, "CALIB_ACTIVE_SNAPSHOT_PATH", legacy_path)
    from desktop.core.calibration_snapshot import invalidate_calibration_snapshot_scope
    monkeypatch.setattr(core_logic, "invalidate_calibration_snapshot_scope", invalidate_calibration_snapshot_scope)
    controller._invalidate_multiturn_anchor_mask = (
        HandController._invalidate_multiturn_anchor_mask.__get__(controller))
    controller._invalidate_servo_rebase_zero_anchors = (
        HandController._invalidate_servo_rebase_zero_anchors.__get__(controller))
    cancelled = [False]

    def ack(cmd, _timeout, received_after_monotonic):
        if cmd != CMD_RESET:
            if ending == "stop":
                controller.stop()
            elif ending == "caller_cancel":
                cancelled[0] = True
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    cancel_check = lambda: cancelled[0]
    if command_name == "servo_reset_zero":
        assert not controller.prepare_calibration_turn_zero(1, cancelled=cancel_check)[0]
    elif command_name == "motor_abs_zero":
        assert not controller.zero_direct_motor_abs(1, cancelled=cancel_check).ok
    else:
        assert not controller.rebase_servo_abs_zero(1, cancelled=cancel_check).ok
    assert int(runtime.load(identity=identity)["valid_motor_mask"], 0) == 6
    assert int(read_snapshot(snapshot_path)["multiturn_anchor_valid_bitmap"], 0) == 7
    assert int(read_snapshot(foreign_snapshot)["multiturn_anchor_valid_bitmap"], 0) == 7
    assert int(read_snapshot(legacy_path)["multiturn_anchor_valid_bitmap"], 0) == 6
    assert controller.comm.send_command.call_args_list.count(call((command_name, 1))) == 1


@pytest.mark.parametrize("command_name", ["servo_rebase_zero", "motor_abs_zero", "servo_reset_zero"])
def test_fresh_rejection_and_stop_preserve_unaccepted_anchor_and_prevent_retry(controller, command_name):
    controller._multiturn_anchor_capture_blocked_mask = 2
    controller._multiturn_anchor_capture_pending_mask = 1

    def ack(cmd, _timeout, received_after_monotonic):
        if cmd == CMD_RESET:
            return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic
        controller.stop()
        return cmd, SYSTEM_STATE_IDLE, 1, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    if command_name == "servo_reset_zero":
        assert not controller.prepare_calibration_turn_zero(1)[0]
    elif command_name == "motor_abs_zero":
        assert not controller.zero_direct_motor_abs(1).ok
    else:
        assert not controller.rebase_servo_abs_zero(1).ok
    assert controller._servo_rebase_zero_unresolved_mask == controller._servo_turn_zero_unresolved_mask == 0
    assert controller._multiturn_anchor_capture_blocked_mask == 2
    assert controller._multiturn_anchor_capture_pending_mask == 1
    controller._invalidate_servo_rebase_zero_anchors.assert_not_called()
    controller._invalidate_multiturn_anchor_mask.assert_not_called()
    core_logic.invalidate_calibration_snapshot_scope.assert_not_called()
    assert controller.comm.send_command.call_args_list.count(call((command_name, 1))) == 1


@pytest.mark.parametrize("command_name", ["servo_rebase_zero", "motor_abs_zero", "servo_reset_zero"])
def test_changed_device_unknown_write_never_invalidates_new_device_anchors(controller, command_name):
    def ack(cmd, _timeout, received_after_monotonic):
        if cmd != CMD_RESET:
            controller.current_state.product_fingerprint["device_id"] = "0000FEDCBA987654"
        return cmd, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack
    if command_name == "servo_reset_zero":
        assert not controller.prepare_calibration_turn_zero(1)[0]
    elif command_name == "motor_abs_zero":
        assert not controller.zero_direct_motor_abs(1).ok
    else:
        assert not controller.rebase_servo_abs_zero(1).ok
    controller._invalidate_servo_rebase_zero_anchors.assert_not_called()
    controller._invalidate_multiturn_anchor_mask.assert_not_called()
    core_logic.invalidate_calibration_snapshot_scope.assert_not_called()

