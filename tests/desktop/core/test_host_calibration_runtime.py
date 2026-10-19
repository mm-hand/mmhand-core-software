"""校准设备适配、执行确认及每关节一次 Manual/LUT 写回。"""

from _clock_helpers import stamp

import time
from unittest.mock import MagicMock

import pytest

from desktop.core.core_logic import HandController
from desktop.core.host_calibration import runtime
from desktop.core.communication.feedback_history import TimestampedFeedbackHistory
from desktop.core.host_calibration.time_alignment import TimeAlignmentError
from desktop.core.host_calibration.math import (
    TendonRawFitBuffer, build_calibration_result, build_tendon_lut,
)
from desktop.core.protocol import (
    CMD_CALIB_LOAD_JOINT, CMD_CALIB_LOAD_LIMITS, CMD_MANUAL_ENCODER_SINGLE,
    CMD_RESET, SYSTEM_STATE_IDLE, SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_UNCONTROLLABLE,
    SYSTEM_FAULT_MULTITURN_UNKNOWN, SYSTEM_FAULT_OVERLOAD,
)


@pytest.fixture
def io(monkeypatch):
    controller = HandController()
    controller.comm = MagicMock()
    state = controller.current_state
    state.has_system_status_data = state.has_p4_time = True
    state.system_state = SYSTEM_STATE_IDLE
    state.system_status_last_rx_ts = time.time()
    stamp(state, 'system_status_last_rx_ts')
    state.system_status_p4_time_ms = 1000
    state.p4_time_ms = state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = 1000
    state.servo_abs_last_rx_ts = time.time()
    stamp(state, 'servo_abs_last_rx_ts')
    state.has_servo_angle_data = state.has_servo_raw_data = True
    state.has_servo_route_status = True
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = (1 << 27) - 1
    state.servo_raw_online = [True] * 27
    state.servo_raw_positions = [128] * 27
    state.servo_logical_range_valid_bitmap = (1 << 27) - 1
    state.servo_logical_range_min = [-30000] * 27
    state.servo_logical_range_max = [30000] * 27
    state.can_link_online = state.has_encoder_raw_data = True
    state.encoder_raw_last_rx_ts = time.time()
    stamp(state, 'encoder_raw_last_rx_ts')
    state.encoder_raw_p4_time_ms = 1000
    state.encoder_online_bitmap = (1 << 21) - 1
    state.manual_reference_valid_bitmap = (1 << 21) - 1
    state.manual_reference_min_raw = [10000] * 21
    state.manual_reference_max_raw = [6000] * 21
    for encoder in state.encoders:
        encoder.raw_error = False
        encoder.raw_sensor = 10000
    store = MagicMock()
    store.path = 'unused.json'
    port = controller.create_host_calibration_io(lambda: False, 2, 4, snapshot_store=store)
    monkeypatch.setattr(runtime, 'log_session', lambda *_args: None)
    return port


def test_empty_batch_keeps_original_initial_state_behavior(io):
    assert io.publish({}, 400, 20)
    io.controller.comm.send_command.assert_not_called()


def _record_fit_raw(history, timestamp, value):
    values = [0] * 21
    values[1] = value
    history.record('encoder_raw', timestamp, values, 2, 100 + timestamp / 1000)


def _record_fit_abs(history, timestamp, value):
    values = [0] * 27
    values[2] = value
    history.record('servo_abs', timestamp, values, 4, 200 + timestamp / 1000)


def _fit_window(io):
    history = TimestampedFeedbackHistory()
    io.controller.comm.feedback_history = history
    _record_fit_raw(history, 0, 10000)
    _record_fit_abs(history, 10, 300)
    return history, io.begin_lut_sampling(1, (2,))


def test_lut_fit_pairs_p4_time_instead_of_latest_model_and_never_waits(io, monkeypatch):
    history, window = _fit_window(io)
    for timestamp in (20, 40, 60, 80, 100):
        _record_fit_raw(history, timestamp, 10000 - 10 * timestamp)
    for timestamp in (30, 50, 70, 90, 110):
        _record_fit_abs(history, timestamp, 100 + 20 * timestamp)
    # These independently updated getters deliberately describe another pair.
    io.controller.current_state.encoders[1].raw_sensor = 5000
    io.controller.current_state.servo_angles[2] = -9999
    monkeypatch.setattr(io, 'sleep_ms', lambda *_: pytest.fail('fit must not wait during release'))
    fit = io.finish_lut_sampling(window)
    assert fit.count >= 3
    for sample in fit.samples:
        assert sample.primary_abs == 100 + 2 * (sample.encoder - 6384)
        assert sample.secondary_abs == 0
    diagnostics = io._lut_sampling_diagnostics[1]
    assert (diagnostics['start_ms'], diagnostics['end_ms']) == (10, 100)
    assert diagnostics['raw_gap_ms'] == diagnostics['abs_gap_ms'] == 20
    assert diagnostics['fit_points'] == len(diagnostics['samples']) == fit.count
    assert fit.samples[0].encoder == 6484
    assert fit.samples[-1].encoder == 7384
    io.controller.comm.send_command.assert_not_called()


@pytest.mark.parametrize('change', ['reset', 'source', 'direction'])
def test_lut_fit_rejects_coordinate_or_source_change_without_fallback(io, change):
    history, window = _fit_window(io)
    _record_fit_raw(history, 100, 9000)
    _record_fit_abs(history, 110, 2300)
    if change == 'reset':
        history.reset()
    elif change == 'source':
        io.controller.comm.feedback_history = TimestampedFeedbackHistory()
    else:
        io.set_direction(1, 1)
    with pytest.raises(TimeAlignmentError):
        io.finish_lut_sampling(window)
    assert io.lut_sampling_error
    assert io.failure_message(16) == io.lut_sampling_error
    assert 1 not in io._lut_sampling_diagnostics
    io.controller.comm.send_command.assert_not_called()


def test_lut_fit_with_insufficient_common_travel_reports_counts(io):
    history, window = _fit_window(io)
    _record_fit_raw(history, 100, 9990)
    _record_fit_abs(history, 110, 310)
    with pytest.raises(TimeAlignmentError, match='insufficient common-time LUT samples'):
        io.finish_lut_sampling(window)
    assert 'fit=2' in io.lut_sampling_error
    assert io._lut_sampling_diagnostics[1]['fit_points'] == 2


def test_lut_fit_with_no_common_time_rejects_latest_value_pairing(io):
    history, window = _fit_window(io)
    history.reset()
    _record_fit_raw(history, 0, 10000)
    _record_fit_raw(history, 100, 9000)
    _record_fit_abs(history, 200, 4000)
    _record_fit_abs(history, 300, 5000)
    window = io.begin_lut_sampling(1, (2,))
    _record_fit_raw(history, 120, 8800)
    _record_fit_abs(history, 320, 5200)
    with pytest.raises(TimeAlignmentError, match='no common valid'):
        io.finish_lut_sampling(window)


def test_old_sampling_window_cannot_remove_new_active_window(io):
    history, old = _fit_window(io)
    active = io.begin_lut_sampling(1, (2,))
    with pytest.raises(TimeAlignmentError, match='not active'):
        io.finish_lut_sampling(old)
    _record_fit_raw(history, 50, 9500)
    _record_fit_raw(history, 100, 9000)
    _record_fit_abs(history, 110, 2300)
    assert io.finish_lut_sampling(active).count >= 3


def test_encoder_raw_uses_packet_age_and_fixed_installation_direction(io):
    assert io.encoder_raw(1) == 10000
    assert io.encoder(1) == 6384
    io.controller.current_state.encoder_raw_p4_time_ms = 699
    assert io.encoder_raw(1) is None


def test_stale_or_offline_telemetry_cannot_drive_endpoint_detection(io):
    state = io.controller.current_state
    state.servo_telem_online[2] = True
    state.servo_telem_p4_time_ms = 1000
    state.servo_telem_last_rx_ts = time.time()
    stamp(state, 'servo_telem_last_rx_ts')
    state.servo_load[2] = -250
    assert io.servo_load(2) == -250
    state.servo_telem_p4_time_ms = 499
    assert io.servo_load(2) is None


def test_historical_task_packet_does_not_make_fresh_feedback_49_days_old(io):
    state = io.controller.current_state
    state.p4_time_ms = 398488  # 实录 TaskRuntime 包携带历史窗口时间。
    state.system_status_p4_time_ms = 398997
    state.servo_feedback_p4_time_ms = 398986
    state.servo_raw_p4_time_ms = 398975
    state.encoder_raw_p4_time_ms = 398980
    state.servo_telem_p4_time_ms = 398986
    state.servo_telem_last_rx_ts = time.time()
    stamp(state, 'servo_telem_last_rx_ts')
    state.servo_telem_online[2] = True
    state.servo_load[2] = 250
    assert io._age(state, 398986, state.received_at('servo_abs_last_rx_ts')) < 100
    assert io.servo_abs(2) == 0
    assert io.encoder_raw(1) == 10000
    assert io.servo_load(2) == 250
    assert io._fresh_feedback(state, 4)
    assert not io._fresh_anchor(state, 4)  # 配对锚点仍须同戳。


def test_historical_packet_cannot_hide_genuinely_stale_device_feedback(io):
    state = io.controller.current_state
    state.p4_time_ms = 300
    state.system_status_p4_time_ms = 1501
    assert io.servo_abs(2) is None
    assert not io._fresh_feedback(state, 4)


def test_feedback_age_handles_p4_wrap_and_system_clock_extrapolation(io, monkeypatch):
    monkeypatch.setattr(time, 'monotonic', lambda: 1024.0)
    io.controller.current_state.monotonic_received['system_status_last_rx_ts'] = 1024.0
    state = io.controller.current_state
    state.system_status_p4_time_ms = 10
    assert 26 <= io._age(state, 0xFFFFFFF0, time.monotonic()) < 100
    state.system_status_p4_time_ms = 1000
    state.system_status_last_rx_ts = time.time() - 0.6
    stamp(state, 'system_status_last_rx_ts')
    assert io._age(state, 1000, time.monotonic()) == pytest.approx(600, abs=.1)


def _recovery_clock(io, monkeypatch, *, advance_feedback=True):
    clock = [0]
    monkeypatch.setattr(io, 'now_ms', lambda: clock[0])

    def sleep(milliseconds):
        clock[0] += milliseconds
        if advance_feedback:
            state = io.controller.current_state
            state.servo_feedback_p4_time_ms += 20
            state.servo_raw_p4_time_ms = state.servo_feedback_p4_time_ms - 11
            state.system_status_p4_time_ms = state.servo_feedback_p4_time_ms + 10
            state.p4_time_ms = state.servo_feedback_p4_time_ms - 498
            state.system_status_last_rx_ts = state.servo_abs_last_rx_ts = time.time()
            stamp(state, 'system_status_last_rx_ts')
            stamp(state, 'servo_abs_last_rx_ts')

    monkeypatch.setattr(io, 'sleep_ms', sleep)
    return clock


def test_uncontrollable_recovery_counts_new_abs_without_requiring_raw_pair(io, monkeypatch):
    state = io.controller.current_state
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    state.servo_raw_p4_time_ms = state.servo_feedback_p4_time_ms - 11
    io._initial_prepared = True
    clock = _recovery_clock(io, monkeypatch)
    assert io.wait_servo_recovery(4)
    assert clock[0] == 40  # 三个不同的新 ABS 快照，沿用原稳定帧门槛。
    assert not io._fresh_anchor(state, 4)
    io.controller.comm.send_command.assert_not_called()


def test_repeated_abs_sample_never_satisfies_recovery_stable_frames(io, monkeypatch):
    io._initial_prepared = True
    _recovery_clock(io, monkeypatch, advance_feedback=False)
    assert not io.wait_servo_recovery(4)
    assert 'stable=1/3' in io.feedback_recovery_error


@pytest.mark.parametrize('field', [
    'multi_turn_unknown_bitmap', 'servo_online_bitmap',
    'servo_route_controllable_bitmap', 'servo_logical_range_valid_bitmap',
])
def test_recovery_keeps_trust_online_route_and_range_gates(io, monkeypatch, field):
    state = io.controller.current_state
    setattr(state, field, 4 if field == 'multi_turn_unknown_bitmap' else 0)
    io._initial_prepared = True
    _recovery_clock(io, monkeypatch)
    assert not io.wait_servo_recovery(4)
    assert state.calib_diag_reason == runtime.CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT


def test_entry_resets_once_before_rebase_without_cancelling_host_session(io, monkeypatch):
    state = io.controller.current_state
    state.has_direct_target_result = True
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    calls = []
    _recovery_clock(io, monkeypatch)

    def ack(command, value):
        calls.append((command, value))
        state.system_status_last_rx_ts = time.time() + 0.01
        stamp(state, 'system_status_last_rx_ts')
        return True

    monkeypatch.setattr(io, '_ack', ack)
    monkeypatch.setattr(io, '_rebase', lambda mask: calls.append(('rebase', mask)) or True)
    io.controller.reset = MagicMock()
    assert io.wait_servo_recovery(4)
    assert io.wait_servo_recovery(4 | 8)  # 第二轮 workspace 恢复不重复 RESET。
    assert calls == [(CMD_RESET, 'reset'), ('rebase', 4)]
    io.controller.reset.assert_not_called()
    assert not io._cancel.is_set()


def test_failed_entry_reset_does_not_rebase_or_attempt_motion(io, monkeypatch):
    io.controller.current_state.has_direct_target_result = True
    monkeypatch.setattr(io, '_ack', lambda *_args: False)
    io.controller.rebase_servo_abs_zero = MagicMock()
    assert not io.wait_servo_recovery(4)
    assert io.feedback_recovery_error == 'maintenance RESET timeout/reject'
    io.controller.rebase_servo_abs_zero.assert_not_called()


def test_cancel_during_entry_reset_prevents_rebase(io, monkeypatch):
    io.controller.current_state.has_direct_target_result = True
    monkeypatch.setattr(io, '_ack', lambda *_args: True)
    monkeypatch.setattr(io, 'sleep_ms', lambda _milliseconds: io.cancel())
    io.controller.rebase_servo_abs_zero = MagicMock()
    assert not io.wait_servo_recovery(4)
    io.controller.rebase_servo_abs_zero.assert_not_called()


@pytest.mark.parametrize('status,applied,rejected,ok', [(2, 4, 0, True), (3, 0, 4, False), (1, 4, 0, False)])
def test_motion_waits_for_matching_execution_result_and_original_parameters(
    io, monkeypatch, status, applied, rejected, ok,
):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)

    def send(command):
        state = io.controller.current_state
        state.direct_target_token = command[1]
        state.direct_target_result_rx_ts = time.time()
        stamp(state, 'direct_target_result_rx_ts')
        state.direct_target_requested_mask = 4
        state.direct_target_applied_mask = applied
        state.direct_target_rejected_mask = rejected
        state.direct_target_status = status
        state.system_state = SYSTEM_STATE_RUN_DIRECT
        state.servo_target_owner = 1
        state.control_enabled = True
        return True

    io.controller.comm.send_command.side_effect = send
    assert io.publish({2: 200}, 400, 20) is ok
    command = io.controller.comm.send_command.call_args.args[0]
    assert command[0] == 'motor_pos_abs'
    assert command[2] == 4
    assert command[3][2] == 200
    assert command[4:] == (400, 20)


def test_queue_ack_without_execution_confirmation_does_not_advance_machine(io, monkeypatch):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)
    monkeypatch.setattr(io, '_wait', lambda *_args: False)
    assert not io.publish({2: 200}, 400, 20)
    assert io.controller.current_state.calib_diag_reason == runtime.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT


def test_remote_execution_and_owner_can_arrive_after_original_100ms(io, monkeypatch):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)
    clock = _recovery_clock(io, monkeypatch)
    tick = io.sleep_ms
    state = io.controller.current_state
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    io.controller.comm.wait_for_ack.return_value = None
    sent = []
    io.controller.comm.send_command.side_effect = lambda command: sent.append(command) or True

    def delayed(milliseconds):
        tick(milliseconds)
        if clock[0] >= 160:
            state.direct_target_token = sent[0][1]
            state.direct_target_result_rx_ts = time.time()
            stamp(state, 'direct_target_result_rx_ts')
            state.direct_target_requested_mask = 4
            state.direct_target_applied_mask = 4
            state.direct_target_status = 2
        if clock[0] >= 320:
            state.system_state = SYSTEM_STATE_RUN_DIRECT
            state.servo_target_owner = 1
            state.control_enabled = True

    monkeypatch.setattr(io, 'sleep_ms', delayed)
    assert io.publish({2: 200}, 400, 20)
    assert clock[0] >= 320
    assert len(sent) == 1  # 不自动重发运动指令。
    assert sent[0][4:] == (400, 20)


def test_queued_ack_without_final_result_fails_with_transport_diagnostic(io, monkeypatch):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)
    _recovery_clock(io, monkeypatch)
    io.controller.comm.wait_for_ack.return_value = (runtime.CMD_MOTOR_POS_ABS, 1, 2)
    assert not io.publish({2: 200}, 400, 20)
    assert io.controller.current_state.calib_diag_value_c == runtime.kHostTargetResultTimeoutMs
    assert 'execution result timeout' in io.failure_message(16)
    assert io.controller.comm.send_command.call_count == 2  # D3 + 超时后的只读同步查询。


def test_explicit_f1_rejection_reports_ack_without_waiting_for_nonexistent_result(io, monkeypatch):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)
    clock = _recovery_clock(io, monkeypatch)
    io.controller.comm.wait_for_ack.return_value = (runtime.CMD_MOTOR_POS_ABS, 1, 3)
    assert not io.publish({2: 200}, 400, 20)
    assert clock[0] == 0
    assert 'ACK status=3' in io.failure_message(16)
    assert io.controller.comm.send_command.call_count == 1


def test_expected_turn_unknown_only_covers_static_rebase_window(io, monkeypatch):
    state = io.controller.current_state
    seen = []

    def rebase(_mask, *, cancelled, send_gate):
        assert not cancelled() and send_gate(lambda: True)
        state.system_status_p4_time_ms = 1050
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        state.multi_turn_unknown_bitmap = 4
        state.system_fault_bitmap = SYSTEM_FAULT_MULTITURN_UNKNOWN
        during = io._state()
        assert io.is_expected_calibration_turn_unknown(during)
        seen.append(during)
        state.multi_turn_unknown_bitmap = state.system_fault_bitmap = 0
        state.servo_rebase_zero_result_p4_time_ms = 1100
        return MagicMock(ok=True)

    io.controller.rebase_servo_abs_zero = rebase
    monkeypatch.setattr(io, '_release_motion', lambda: True)

    def wait(predicate, timeout):
        assert timeout == runtime.kHostRebaseFeedbackTimeoutMs
        state.system_status_p4_time_ms = state.p4_time_ms = 1120
        state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = 1120
        state.servo_logical_range_p4_time_ms = 1120
        state.servo_logical_range_last_rx_ts = time.time() + 0.01
        stamp(state, 'servo_logical_range_last_rx_ts')
        return predicate()

    monkeypatch.setattr(io, '_wait', wait)
    assert io._rebase(4)
    during = seen[0]
    assert io.is_expected_calibration_turn_unknown(during)  # Qt 晚到的真实维护中间帧。
    during.system_status_p4_time_ms = 1121
    assert not io.is_expected_calibration_turn_unknown(during)
    during.system_status_p4_time_ms = 1050
    during.system_state = SYSTEM_STATE_RUN_DIRECT
    assert not io.is_expected_calibration_turn_unknown(during)
    during.system_state = SYSTEM_STATE_IDLE
    during.system_fault_bitmap |= SYSTEM_FAULT_OVERLOAD
    assert not io.is_expected_calibration_turn_unknown(during)
    during.system_fault_bitmap = SYSTEM_FAULT_MULTITURN_UNKNOWN
    during.multi_turn_unknown_bitmap = 4 | 8
    assert not io.is_expected_calibration_turn_unknown(during)
    during.multi_turn_unknown_bitmap = 4
    state.multi_turn_unknown_bitmap = 4
    assert not io.is_expected_calibration_turn_unknown(during)  # 真实持续 UNKNOWN 不屏蔽。


def test_failed_rebase_does_not_hide_persistent_unknown(io, monkeypatch):
    state = io.controller.current_state
    monkeypatch.setattr(io, '_release_motion', lambda: True)

    def failed(_mask, *, cancelled, send_gate):
        assert not cancelled() and send_gate(lambda: True)
        state.multi_turn_unknown_bitmap = 4
        state.system_fault_bitmap = SYSTEM_FAULT_MULTITURN_UNKNOWN
        state.system_status_p4_time_ms = 1050
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        assert io.is_expected_calibration_turn_unknown(io._state())
        return MagicMock(ok=False, message='hardware zero failed')

    io.controller.rebase_servo_abs_zero = failed
    assert not io._rebase(4)
    assert not io.is_expected_calibration_turn_unknown(io._state())


def test_calibration_passes_live_cancellation_and_owner_gate_into_rebase(io, monkeypatch):
    monkeypatch.setattr(io, '_release_motion', lambda: True)
    calls = []

    def rebase(mask, *, cancelled, send_gate):
        assert mask == 4 and not cancelled()
        assert send_gate(lambda: calls.append('first') or True)
        io.controller.current_state.servo_target_owner = 1
        assert not send_gate(lambda: calls.append('foreign owner') or True)
        io.controller.current_state.servo_target_owner = 0
        io.cancel()
        assert cancelled()
        assert not send_gate(lambda: calls.append('after stop') or True)
        return MagicMock(ok=False, message='cancelled')

    io.controller.rebase_servo_abs_zero = rebase
    assert not io._rebase(4)
    assert calls == ['first']


def test_new_session_diagnostics_stay_clear_when_old_p4_failure_arrives(io):
    from desktop.core.calibration_diagnostics import format_calibration_diagnostic

    io.diag(runtime.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
            runtime.CALIB_DIAG_PHASE_SEQUENCE, 0, 1, 2, 3, 4, 3)
    assert format_calibration_diagnostic(io._state())['active']
    controller = io.controller
    retry = controller.create_host_calibration_io(
        lambda: False, 2, 4, snapshot_store=io.snapshot_store)
    retry.session_state('PENDING')
    assert not format_calibration_diagnostic(retry._state())['active']
    # 原 P4 Auto 任务的旧失败包会跨 RESET 保留；它不拥有 host 本轮诊断。
    controller.current_state.calib_diag_valid = True
    controller.current_state.calib_diag_reason = runtime.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED
    controller._apply_host_calibration_overlay()
    assert not format_calibration_diagnostic(retry._state())['active']
    retry.diag(runtime.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT,
               runtime.CALIB_DIAG_PHASE_SEQUENCE, 1)
    assert retry._state().calib_diag_reason == runtime.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT
    assert format_calibration_diagnostic(retry._state())['active']


def test_normal_direct_done_allows_next_step_but_stop_and_foreign_token_do_not(io):
    state = io.controller.current_state
    io._motion_started = True
    io._target_token = state.direct_target_token = 9
    state.direct_target_status = 2
    assert io.owns_control()  # D3 正常到位，owner 已释放到 IDLE。
    state.direct_target_token = 10
    assert not io.owns_control()
    state.direct_target_token = 9
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    assert not io.owns_control()  # 用户 STOP 不等于普通 DONE。


def test_motor_calibration_faults_are_scoped_without_masking_selected_failures(io):
    from desktop.core.protocol import SYSTEM_FAULT_SERVO_OFFLINE
    state = io.controller.current_state
    io._motion_started = True
    io._motion_scope = 4
    state.system_state = SYSTEM_STATE_RUN_DIRECT
    state.control_enabled = True
    state.servo_target_owner = 1
    state.servo_online_bitmap = 4
    state.multi_turn_unknown_bitmap = ((1 << 27) - 1) & ~4
    state.system_fault_bitmap = SYSTEM_FAULT_SERVO_OFFLINE | SYSTEM_FAULT_MULTITURN_UNKNOWN
    assert io.owns_control()
    state.servo_online_bitmap = 0
    assert not io.owns_control()
    state.servo_online_bitmap = 4
    state.multi_turn_unknown_bitmap |= 4
    assert not io.owns_control()
    state.multi_turn_unknown_bitmap &= ~4
    state.system_fault_bitmap |= SYSTEM_FAULT_OVERLOAD
    assert not io.owns_control()


@pytest.mark.parametrize('unknown', [0, 1 << 5])
def test_core_ignores_unselected_unknown_helper_without_rebasing_it(io, unknown):
    io.selected_joint_mask = 1
    io.selected_motor_mask = io.preparation_reset_mask = 3
    io.controller.current_state.multi_turn_unknown_bitmap = unknown
    assert io._prepare_workspace_reset_scope()
    assert io.preparation_reset_mask == 3


def test_core_removes_now_offline_helper_from_old_preplanned_rebase_mask(io):
    io.selected_joint_mask = 1
    io.selected_motor_mask = 3
    pair = (1 << 5) | (1 << 6)
    io.preparation_reset_mask = 3 | pair
    io.controller.current_state.servo_online_bitmap &= ~pair
    assert io._prepare_workspace_reset_scope()
    assert io.preparation_reset_mask == 3


@pytest.mark.parametrize('motor_only', [False, True])
def test_j07_initial_recovery_requires_only_its_selected_motor(io, monkeypatch, motor_only):
    selected = 1 << 9
    io.selected_joint_mask = 0 if motor_only else 1 << 7
    io.selected_motor_mask = io.preparation_reset_mask = selected
    state = io.controller.current_state
    state.has_direct_target_result = True
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = selected
    state.servo_logical_range_valid_bitmap = selected
    state.multi_turn_unknown_bitmap = state.servo_route_conflict_bitmap = (
        ((1 << 27) - 1) & ~selected)
    state.servo_raw_online = [motor == 9 for motor in range(27)]
    # One helper is only partially online, and has unknown turns/conflicting routes.
    state.servo_online_bitmap |= 1 << 5
    monkeypatch.setattr(io, '_prepare_device', lambda: True)
    monkeypatch.setattr(io, '_rebase', MagicMock(return_value=True))
    _recovery_clock(io, monkeypatch)
    assert io.wait_servo_recovery(selected)
    assert io.preparation_reset_mask == selected
    assert io.feedback_recovery_error == ''
    io._rebase.assert_called_once_with(selected)
    io.controller.comm.send_command.assert_not_called()


@pytest.mark.parametrize('failure,reason', [
    ('partial', 'helper pair partially online'),
    ('unknown', 'helper multi-turn position unknown'),
    ('route_conflict', 'helper motor route conflict'),
])
def test_unselected_unusable_helper_never_blocks_or_enters_rebase(io, monkeypatch, failure, reason):
    io.selected_joint_mask = 1
    io.selected_motor_mask = io.preparation_reset_mask = 3
    state = io.controller.current_state
    state.has_direct_target_result = True
    if failure == 'partial':
        state.servo_online_bitmap &= ~(1 << 6)
    elif failure == 'unknown':
        state.multi_turn_unknown_bitmap = 1 << 5
    else:
        state.servo_route_conflict_bitmap = 1 << 5
    monkeypatch.setattr(io, '_prepare_device', lambda: True)
    monkeypatch.setattr(io, '_rebase', MagicMock(return_value=True))
    _recovery_clock(io, monkeypatch)
    assert io.wait_servo_recovery(3)
    io._rebase.assert_called_once_with(3)
    assert io.feedback_recovery_error == ''



def test_outside_logical_range_is_rejected_without_clamping(io, monkeypatch):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)
    assert not io.publish({2: 30001}, 400, 20)
    io.controller.comm.send_command.assert_not_called()


def test_external_stop_cancels_host_session_and_preserves_real_p4_state(io):
    io.session_state('RUNNING', completed=2)
    assert io.controller.current_state.calib_status == 'RUNNING'
    assert io.controller.current_state.system_state == SYSTEM_STATE_IDLE
    io.controller.stop()
    assert not io.owns_control()
    io.controller.comm.send_command.assert_called_once_with('stop')
    io.session_state('FAILED', completed=2)
    assert io.controller.current_state.calib_status == 'IDLE'


def test_stop_cancels_host_display_and_old_worker_cannot_restore_running(io):
    io.session_state('RUNNING')
    io.controller.stop()
    io.controller.comm.send_command.assert_called_once_with('stop')
    assert not io.owns_control()
    io.session_state('RUNNING')
    io.controller._apply_host_calibration_overlay()
    assert io.controller.current_state.calib_status == 'IDLE'


def test_overloaded_hold_motor_rejects_entire_batch_before_transmission(io, monkeypatch):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)
    monkeypatch.setattr(io, 'servo_load', lambda _motor: -800)
    assert not io.publish({2: 200}, 400, 20)
    io.controller.comm.send_command.assert_not_called()
    assert io.controller.current_state.calib_diag_reason == runtime.CALIB_DIAG_REASON_OVERLOAD


def test_rebased_scope_cannot_reappear_through_host_overlay(io):
    state = io.controller.current_state
    state.joint_calib_results[1] = {'success': True}
    state.joint_calib_results[3] = {'success': True}
    state.auto_calibration_valid_bitmap = 2 | 8
    state.motor_abs_limit_valid_bitmap = 4 | 16
    io._overlay(joint_calib_results=state.joint_calib_results,
                auto_calibration_valid_bitmap=state.auto_calibration_valid_bitmap,
                motor_abs_limit_valid_bitmap=state.motor_abs_limit_valid_bitmap)
    io._clear_result_scope(4)
    io.controller._apply_host_calibration_overlay()
    assert state.joint_calib_results[1] is None
    assert state.joint_calib_results[3]['success']
    assert state.auto_calibration_valid_bitmap == 8
    assert state.motor_abs_limit_valid_bitmap == 16
    assert 'joint_calib_results' not in io.controller._host_calibration_overlay
    assert 'auto_calibration_valid_bitmap' not in io.controller._host_calibration_overlay


def test_manual_queued_ack_waits_for_commit_before_dependent_lut(io):
    io.controller.comm.wait_for_ack.return_value = (0, 0, runtime.PROTO_ACK_STATUS_QUEUED)
    io.controller._wait_for_manual_reference_commit_result = MagicMock(return_value=False)
    assert not io._ack(CMD_MANUAL_ENCODER_SINGLE,
                       ('manual_encoder_single', 1, 10000, 6000))
    io.controller._wait_for_manual_reference_commit_result.assert_called_once()


def test_failure_restores_motion_scope_after_workspace_read_mask_cleared(io, monkeypatch):
    state = io.controller.current_state
    io._motion_scope = 4 | 8
    state.active_motor_mask = 4 | 8
    state.active_joint_mask = 2
    io.set_workspace_read_mask(0)  # flow 的 finally 首先清读反馈范围。
    io.controller.set_active_scope = MagicMock(return_value=True)
    io.finish_session(False, 0, 2)
    io.controller.comm.send_command.assert_called_once_with('stop')
    io.controller.set_active_scope.assert_called_once_with(4, 2, True)


def test_target_token_rollover_keeps_advancing(io, monkeypatch):
    monkeypatch.setattr(io, '_ensure_motion', lambda: True)
    state = io.controller.current_state
    state.direct_target_token = 0xFFFFFFFF

    def send(command):
        state.direct_target_token = command[1]
        state.direct_target_result_rx_ts = time.time()
        stamp(state, 'direct_target_result_rx_ts')
        state.direct_target_requested_mask = 4
        state.direct_target_applied_mask = 4
        state.direct_target_rejected_mask = 0
        state.direct_target_status = 2
        state.system_state = SYSTEM_STATE_RUN_DIRECT
        state.servo_target_owner = 1
        state.control_enabled = True
        return True

    io.controller.comm.send_command.side_effect = send
    assert io.publish({2: 200}, 400, 20)
    assert io.publish({2: 201}, 400, 20)
    assert [call.args[0][1] for call in io.controller.comm.send_command.call_args_list] == [1, 2]


def test_rebase_requires_feedback_and_range_after_result(io, monkeypatch):
    monkeypatch.setattr(io, '_release_motion', lambda: True)
    io.controller.rebase_servo_abs_zero = MagicMock(return_value=MagicMock(ok=True))
    state = io.controller.current_state
    state.servo_rebase_zero_result_p4_time_ms = 1100
    # 旧帧本身仍然新鲜、可信、同帧，不足以证明坐标更新。
    state.servo_logical_range_p4_time_ms = 1000
    state.servo_logical_range_last_rx_ts = time.time()
    stamp(state, 'servo_logical_range_last_rx_ts')
    checked = []

    def wait(predicate, _timeout):
        checked.append(predicate())
        state.p4_time_ms = state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = 1120
        state.system_status_p4_time_ms = 1120
        state.servo_logical_range_p4_time_ms = 1120
        state.servo_logical_range_last_rx_ts = time.time() + 0.01
        stamp(state, 'servo_logical_range_last_rx_ts')
        checked.append(predicate())
        return checked[-1]

    monkeypatch.setattr(io, '_wait', wait)
    assert io._rebase(4)
    assert checked == [False, True]


def _rebase_wait_clock(io, monkeypatch, *, on_sleep=None):
    clock = [0]
    monkeypatch.setattr(io, 'now_ms', lambda: clock[0])
    monkeypatch.setattr(runtime, 'kHostRebaseFeedbackTimeoutMs', 20)

    def sleep(milliseconds):
        clock[0] += milliseconds
        if on_sleep is not None:
            on_sleep()

    monkeypatch.setattr(io, 'sleep_ms', sleep)
    return clock


def _install_rebase_feedback(io, *, result_ms=1100, abs_ms=1120,
                             raw_ms=1120, range_ms=1120):
    state = io.controller.current_state
    state.servo_rebase_zero_result_p4_time_ms = result_ms
    io.controller.rebase_servo_abs_zero = MagicMock(return_value=MagicMock(ok=True))

    def sync():
        state.servo_feedback_p4_time_ms = abs_ms
        state.servo_raw_p4_time_ms = raw_ms
        state.servo_logical_range_p4_time_ms = range_ms
        state.servo_logical_range_last_rx_ts = time.time()
        stamp(state, 'servo_logical_range_last_rx_ts')

    io.controller.request_state_sync = MagicMock(side_effect=sync)


@pytest.mark.parametrize('cancel_source', ['io', 'workflow'])
def test_successful_zero_followed_by_cancel_is_not_reported_as_hardware_failure(
    io, monkeypatch, cancel_source,
):
    _install_rebase_feedback(io, raw_ms=1121)  # 等待尚未组成的同戳锚点。
    if cancel_source == 'io':
        clock = _rebase_wait_clock(io, monkeypatch, on_sleep=io.cancel)
    else:
        clock = _rebase_wait_clock(io, monkeypatch)
        io.cancel_check = lambda: clock[0] >= 5
    ok, message = io.prepare_joint(1, 4)
    assert not ok
    assert clock[0] == 5  # 真正超时为 20 ms，取消没有等到期限。
    assert io.wait_exit_reason == 'cancelled'
    assert 'logical ABS zero applied' in message
    assert 'feedback wait cancelled/stopped' in message
    assert 'zero failed' not in message
    assert 'timeout' not in message
    assert 'abs_raw_timestamp_mismatch' in message
    io.controller.rebase_servo_abs_zero.assert_called_once()
    assert io.controller.rebase_servo_abs_zero.call_args.args == (4,)
    assert io.controller.rebase_servo_abs_zero.call_args.kwargs['cancelled'] == io.is_cancelled
    assert io._rebase_windows[-1]['end_ms'] is None


def test_post_zero_feedback_timeout_identifies_the_same_timestamp_gate(io, monkeypatch):
    _install_rebase_feedback(io, raw_ms=1121)
    clock = _rebase_wait_clock(io, monkeypatch)
    ok, message = io.prepare_joint(1, 4)
    assert not ok
    assert clock[0] > runtime.kHostRebaseFeedbackTimeoutMs
    assert io.wait_exit_reason == 'timeout'
    assert 'logical ABS zero applied' in message
    assert 'feedback wait timeout' in message
    assert 'unmet=abs_raw_timestamp_mismatch' in message
    assert 'zero failed' not in message
    assert not io.is_cancelled()
    io.controller.rebase_servo_abs_zero.assert_called_once()
    assert io.controller.rebase_servo_abs_zero.call_args.args == (4,)
    assert io.controller.rebase_servo_abs_zero.call_args.kwargs['cancelled'] == io.is_cancelled


@pytest.mark.parametrize('field,value,expected', [
    ('has_servo_angle_data', False, 'missing_abs'),
    ('has_servo_raw_data', False, 'missing_raw'),
    ('servo_online_bitmap', 0, 'offline=0x00000004'),
    ('servo_route_controllable_bitmap', 0, 'unroutable=0x00000004'),
    ('multi_turn_unknown_bitmap', 4, 'unknown=0x00000004'),
    ('servo_abs_last_rx_ts', 0, 'stale_abs(age=infms)'),
    ('servo_logical_range_valid_bitmap', 0, 'missing_range=0x00000004'),
])
def test_post_zero_timeout_reports_the_missing_trust_gate(
    io, monkeypatch, field, value, expected,
):
    _install_rebase_feedback(io)
    setattr(io.controller.current_state, field, value)
    if field.endswith('last_rx_ts'):
        stamp(io.controller.current_state, field)
    _rebase_wait_clock(io, monkeypatch)
    assert not io.prepare_joint(1, 4)[0]
    assert expected in io.feedback_recovery_error
    assert io.wait_exit_reason == 'timeout'


def test_post_zero_timeout_reports_raw_offline_and_old_range_reply(io, monkeypatch):
    _install_rebase_feedback(io)
    state = io.controller.current_state
    state.servo_raw_online[2] = False
    state.servo_logical_range_last_rx_ts = time.time() - 1
    stamp(state, 'servo_logical_range_last_rx_ts')
    io.controller.request_state_sync = MagicMock()
    state.servo_feedback_p4_time_ms = state.servo_raw_p4_time_ms = 1120
    state.servo_logical_range_p4_time_ms = 1120
    _rebase_wait_clock(io, monkeypatch)
    assert not io.prepare_joint(1, 4)[0]
    assert 'raw_offline=0x00000004' in io.feedback_recovery_error
    assert 'range_before_sync_request' in io.feedback_recovery_error


@pytest.mark.parametrize('abs_ms,range_ms,expected', [
    (1100, 1120, 'abs_not_after_zero_result'),
    (1099, 1120, 'abs_not_after_zero_result'),
    (1120, 1100, 'range_not_after_zero_result'),
    (1120, 1099, 'range_not_after_zero_result'),
])
def test_post_zero_timeout_keeps_strict_post_result_timestamps(
    io, monkeypatch, abs_ms, range_ms, expected,
):
    _install_rebase_feedback(io, abs_ms=abs_ms, raw_ms=abs_ms, range_ms=range_ms)
    _rebase_wait_clock(io, monkeypatch)
    assert not io.prepare_joint(1, 4)[0]
    assert expected in io.feedback_recovery_error


@pytest.mark.parametrize('start_ms,result_ms,anchor_ms,intermediate_ms', [
    (1000, 1100, 1120, 1110),
    (0xFFFFFFD0, 0xFFFFFFF0, 20, 0xFFFFFFF8),
])
def test_completed_rebase_window_uses_known_anchor_even_when_system_status_lags(
    io, monkeypatch, start_ms, result_ms, anchor_ms, intermediate_ms,
):
    state = io.controller.current_state
    state.system_status_p4_time_ms = start_ms
    _install_rebase_feedback(io, result_ms=result_ms, abs_ms=anchor_ms,
                             raw_ms=anchor_ms, range_ms=anchor_ms)
    _rebase_wait_clock(io, monkeypatch)
    during = []

    def zero(_mask, *, cancelled, send_gate):
        assert not cancelled() and send_gate(lambda: True)
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        state.multi_turn_unknown_bitmap = 4
        state.system_fault_bitmap = SYSTEM_FAULT_MULTITURN_UNKNOWN
        intermediate = io._state()
        intermediate.system_status_p4_time_ms = intermediate_ms
        during.append(intermediate)
        state.multi_turn_unknown_bitmap = state.system_fault_bitmap = 0
        return MagicMock(ok=True)

    io.controller.rebase_servo_abs_zero.side_effect = zero
    assert io.prepare_joint(1, 4)[0]
    assert io.wait_exit_reason == 'ready'
    assert io._rebase_windows[-1]['end_ms'] == anchor_ms
    assert state.system_status_p4_time_ms == start_ms  # 旧状态包不能提前截断窗口。
    assert io.is_expected_calibration_turn_unknown(during[0])
    during[0].system_status_p4_time_ms = (anchor_ms + 1) & 0xFFFFFFFF
    assert not io.is_expected_calibration_turn_unknown(during[0])
    during[0].system_status_p4_time_ms = intermediate_ms
    state.multi_turn_unknown_bitmap = 4
    assert not io.is_expected_calibration_turn_unknown(during[0])


def test_each_joint_writes_manual_and_lut_once_then_limits_and_anchors(io, monkeypatch):
    result = build_calibration_result(io, 1, 6384, 0, 9384, 0, 100, 2000)
    assert result
    raw = TendonRawFitBuffer()
    for encoder, motor_abs in ((6384, 0), (7884, 1000), (9384, 2000)):
        raw.append(encoder, motor_abs, 0)
    assert build_tendon_lut(io, 1, result, 2, 255, 1, raw)
    monkeypatch.setattr(io, '_release_motion', lambda: True)
    sent = []
    monkeypatch.setattr(io, '_ack', lambda command, value: sent.append((command, value)) or True)
    io.controller._persist_multiturn_state = MagicMock(return_value=True)
    saved_anchors = []
    monkeypatch.setattr(runtime, 'update_snapshot_multiturn_anchors',
                        lambda path, **values: saved_anchors.append((path, values)))
    ok, _message = io.commit_joint(1, result, {2: (-500, 2900)})
    assert ok
    assert [command for command, _value in sent] == [
        CMD_MANUAL_ENCODER_SINGLE, CMD_CALIB_LOAD_JOINT, CMD_CALIB_LOAD_LIMITS,
    ]
    assert sent[0][1] == ('manual_encoder_single', 1, 10000, 6000)
    assert sent[1][1][1]['tendonLutMotor'] == [2, 255]
    assert sent[2][1][1:3] == (4, 4)
    io.snapshot_store.merge_calibration.assert_called_once()
    assert saved_anchors[0][1]['motor_mask'] == 4
    assert io.controller.current_state.joint_calib_results[1]['success']
    # P4 状态回显可稍后才到；host 收尾仍使用本轮自己的生成结果。
    io.controller.current_state.joint_calib_results[1] = None
    assert io.calibration_result(1)['tendonLutMotor'] == [2, 255]


def test_manual_failure_does_not_upload_lut_or_report_joint_complete(io, monkeypatch):
    result = build_calibration_result(io, 1, 6384, 0, 9384, 0, 100, 2000)
    raw = TendonRawFitBuffer()
    for encoder, motor_abs in ((6384, 0), (7884, 1000), (9384, 2000)):
        raw.append(encoder, motor_abs, 0)
    assert build_tendon_lut(io, 1, result, 2, 255, 1, raw)
    monkeypatch.setattr(io, '_release_motion', lambda: True)
    sent = []
    monkeypatch.setattr(io, '_ack', lambda command, value: sent.append(command) and False)
    ok, _message = io.commit_joint(1, result, {2: (-500, 2900)})
    assert not ok
    assert sent == [CMD_MANUAL_ENCODER_SINGLE]
    io.snapshot_store.merge_calibration.assert_not_called()


def test_invalid_lut_remains_host_diagnostic_and_is_not_uploaded(io, monkeypatch):
    result = build_calibration_result(io, 0, 6384, 0, 9384, 0, 100, 2000)
    assert not build_tendon_lut(io, 0, result, 0, 1, 2, TendonRawFitBuffer())
    assert result and not result['tendonLutValid']
    monkeypatch.setattr(io, '_release_motion', lambda: True)
    sent = []
    monkeypatch.setattr(io, '_ack', lambda command, value: sent.append((command, value)) or True)
    ok, message = io.commit_joint(0, result, {0: (-3000, 500), 1: (-500, 3000)})
    assert not ok and 'LUT build invalid, not uploaded' in message
    assert sent == []
    io.snapshot_store.merge_calibration.assert_not_called()


def test_host_zero_finish_failure_stops_ordinary_motion(io, monkeypatch):
    from desktop.core.host_calibration import finish

    state = io.controller.current_state
    monkeypatch.setattr(io, '_release_motion', lambda: True)
    io.controller.set_active_scope = MagicMock(return_value=True)

    def ack(command, _value):
        if command == runtime.CMD_ROPE_INIT:
            state.rope_init_sequence += 1
            state.rope_init_status = 2
        return True

    monkeypatch.setattr(io, '_ack', ack)
    monkeypatch.setattr(finish, 'finish_dual_zero', lambda *_args: (False, 'zero target rejected'))
    assert io.finish_session(True, 2, 0) == (False, 'zero target rejected')
    io.controller.comm.send_command.assert_called_once_with('stop')
