"""Release Rope wait/retry semantics: transient feedback must not abort, and
each repeated release on the same device resets zero counters again."""
import contextlib
import threading
import time
from unittest.mock import MagicMock, Mock

import pytest

from _clock_helpers import stamp
from desktop.core import core_logic
from desktop.core.core_logic import HandController, ServoRebaseZeroResult
from desktop.core.protocol import HOST_PROTOCOL_VERSION, MOTOR_COUNT, SYSTEM_STATE_IDLE, SYSTEM_STATE_RUN_DIRECT


class _Ticker:
    def __init__(self, state, field='servo_abs_last_rx_ts'):
        self.state = state
        self.field = field
        self.stop = threading.Event()
        self.paused = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.wait(0.01):
            if self.paused.is_set():
                continue
            setattr(self.state, self.field, time.time())
            stamp(self.state, self.field)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.0)


@contextlib.contextmanager
def feedback_ticker(state):
    ticker = _Ticker(state)
    try:
        yield ticker
    finally:
        ticker.close()


def _sent_reset(controller):
    return any(
        call.args and call.args[0] == 'reset'
        for call in controller.comm.send_command.call_args_list
    )


def make_controller(monkeypatch, motors=(3,), angle=3000):
    controller = HandController()
    controller.comm = MagicMock()
    state = controller.current_state
    state.has_product_fingerprint = True
    state.product_fingerprint = {
        'device_id': '0000123456789ABC',
        'protocol_version': HOST_PROTOCOL_VERSION,
    }
    state.system_state = SYSTEM_STATE_IDLE
    state.has_servo_angle_data = state.has_servo_route_status = True
    state.has_p4_time = True
    state.p4_time_ms = state.servo_feedback_p4_time_ms = 1000
    state.servo_abs_last_rx_ts = time.time()
    stamp(state, 'servo_abs_last_rx_ts')
    mask = sum(1 << motor for motor in motors)
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = mask
    state.multi_turn_unknown_bitmap = ((1 << MOTOR_COUNT) - 1) & ~mask
    for motor in motors:
        state.servo_angles[motor] = angle
    controller.rebase_servo_abs_zero = MagicMock(return_value=ServoRebaseZeroResult(True, 'ok', 1, mask, 0))
    controller.set_active_scope = MagicMock(return_value=True)
    monkeypatch.setattr(core_logic, 'invalidate_calibration_snapshot_scope', Mock(return_value='saved'))
    return controller, state, mask


def accepted_send(state, mask):
    """Command handler that records acceptance and moves all selected motors."""
    def send(command):
        if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
            targets = command[1]
            state.has_direct_target_result = True
            state.direct_target_status = 2
            state.direct_target_applied_mask = mask
            state.direct_target_rejected_mask = 0
            state.control_enabled = True
            for motor in range(MOTOR_COUNT):
                if mask & (1 << motor):
                    state.servo_angles[motor] = int(targets[motor])
        return True
    return send


def test_transient_feedback_gap_does_not_abort_release(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    monkeypatch.setattr(core_logic, 'MANUAL_PREPARE_TRANSIENT_GRACE_S', 0.4)
    with feedback_ticker(state) as ticker:
        def send(command):
            if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
                ticker.paused.set()
                threading.Timer(0.25, ticker.paused.clear).start()
            return accepted_send(state, mask)(command)
        controller.comm.send_command.side_effect = send
        result = controller.manual_prepare_all_motors()
    assert result.ok, result.message
    assert result.reached_motor_mask == mask


def test_transient_route_flap_does_not_abort_release(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    monkeypatch.setattr(core_logic, 'MANUAL_PREPARE_TRANSIENT_GRACE_S', 0.5)
    with feedback_ticker(state):
        def send(command):
            if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
                state.servo_route_controllable_bitmap = 0
                threading.Timer(0.25, lambda: setattr(state, 'servo_route_controllable_bitmap', mask)).start()
            return accepted_send(state, mask)(command)
        controller.comm.send_command.side_effect = send
        result = controller.manual_prepare_all_motors()
    assert result.ok, result.message
    assert result.reached_motor_mask == mask


def test_persistent_feedback_staleness_reports_transient_reason(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    monkeypatch.setattr(core_logic, 'MANUAL_PREPARE_READY_TIMEOUT_S', 0.05)
    monkeypatch.setattr(core_logic, 'MANUAL_PREPARE_TRANSIENT_GRACE_S', 0.2)
    with feedback_ticker(state) as ticker:
        def send(command):
            if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
                ticker.paused.set()  # never recover
                state.has_direct_target_result = True
                state.direct_target_status = 2
                state.direct_target_applied_mask = mask
                # command accepted but motors do not move
            return True
        controller.comm.send_command.side_effect = send
        result = controller.manual_prepare_all_motors()
    assert not result.ok
    assert 'servo ABS feedback stale' in result.message
    assert 'did not settle' in result.message
    assert result.reached_motor_mask == 0
    assert _sent_reset(controller)


def test_direct_target_rejection_fails_before_motion_wait(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    with feedback_ticker(state):
        def send(command):
            if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
                state.has_direct_target_result = True
                state.direct_target_status = 3
                state.direct_target_rejected_mask = mask
            return True
        controller.comm.send_command.side_effect = send
        started = time.monotonic()
        result = controller.manual_prepare_all_motors()
        elapsed = time.monotonic() - started
    assert not result.ok
    assert 'rejected' in result.message.lower()
    assert elapsed < 2.0
    assert _sent_reset(controller)


def test_motion_timeout_reports_reached_mask_and_diagnostics(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    monkeypatch.setattr(core_logic, 'MANUAL_PREPARE_READY_TIMEOUT_S', 0.05)
    monkeypatch.setattr(core_logic, 'MANUAL_PREPARE_MOTION_TIMEOUT_S', 0.25)
    with feedback_ticker(state):
        def send(command):
            if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
                state.has_direct_target_result = True
                state.direct_target_status = 2
                state.direct_target_applied_mask = mask
                state.system_state = SYSTEM_STATE_RUN_DIRECT
            return True  # command accepted but motors never move
        controller.comm.send_command.side_effect = send
        result = controller.manual_prepare_all_motors()
    assert not result.ok
    assert 'did not settle' in result.message
    assert 'reached=0x00000' in result.message
    assert 'M03:' in result.message  # per-motor target/actual diagnostic
    assert _sent_reset(controller)


def test_release_clears_runtime_calibration_but_keeps_manual(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    state.joint_calib_results[2] = {'joint': 2, 'success': True, 'manualSpanCounts': 4000}
    state.auto_calibration_valid_bitmap = 1 << 2
    state.motor_abs_limit_valid_bitmap = mask
    state.motor_abs_limit_min[3] = -1234
    state.motor_abs_limit_max[3] = 4321
    state.manual_reference_valid_bitmap = 1 << 2
    state.manual_reference_min_raw[2] = 8573
    state.manual_reference_max_raw[2] = 12000
    with feedback_ticker(state):
        controller.comm.send_command.side_effect = accepted_send(state, mask)
        result = controller.manual_prepare_all_motors()
    assert result.ok, result.message
    assert state.joint_calib_results[2] is None
    assert not state.auto_calibration_valid_bitmap & (1 << 2)
    assert not state.calibration_warning_bitmap & (1 << 2)
    assert not state.motor_abs_limit_valid_bitmap & mask
    assert state.motor_abs_limit_min[3] == 0
    assert state.motor_abs_limit_max[3] == 0
    assert state.manual_reference_valid_bitmap & (1 << 2)
    assert state.manual_reference_min_raw[2] == 8573
    assert state.manual_reference_max_raw[2] == 12000


def test_repeat_release_resets_zero_and_releases_again(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    with feedback_ticker(state):
        controller.comm.send_command.side_effect = accepted_send(state, mask)
        first = controller.manual_prepare_all_motors()
        second = controller.manual_prepare_all_motors()
    assert first.ok and second.ok, second.message
    assert controller.rebase_servo_abs_zero.call_count == 2
    assert core_logic.invalidate_calibration_snapshot_scope.call_count == 2
    motor_sends = [
        call for call in controller.comm.send_command.call_args_list
        if call.args and isinstance(call.args[0], tuple) and call.args[0][0] == 'motor_pos_abs'
    ]
    assert len(motor_sends) == 2
    # Each release rebuilds turn0 and unwinds another 1500 counts: 3000 -> 1500 -> 0.
    assert state.servo_angles[3] == 0


def test_cancel_during_release_motion_resets_without_marking_done(monkeypatch):
    controller, state, mask = make_controller(monkeypatch)
    cancel = threading.Event()
    monkeypatch.setattr(core_logic, 'MANUAL_PREPARE_MOTION_TIMEOUT_S', 5.0)
    with feedback_ticker(state):
        def send(command):
            if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
                state.has_direct_target_result = True
                state.direct_target_status = 2
                state.direct_target_applied_mask = mask
                # command accepted but intentionally never reaches the target
                threading.Timer(0.1, cancel.set).start()
            return True
        controller.comm.send_command.side_effect = send
        result = controller.manual_prepare_all_motors(cancelled=cancel.is_set)
    assert not result.ok
    assert 'cancelled during release motion' in result.message
    assert _sent_reset(controller)
    # not marked done: a later attempt still performs a reset
    with feedback_ticker(state):
        controller.comm.send_command.side_effect = accepted_send(state, mask)
        controller.rebase_servo_abs_zero.reset_mock()
        controller.manual_prepare_all_motors()
    assert controller.rebase_servo_abs_zero.called
