
from _clock_helpers import stamp
from desktop.core.protocol import HOST_PROTOCOL_VERSION
import time
from unittest.mock import MagicMock, call

import pytest

from desktop.core import core_logic
from desktop.core.core_logic import HandController
from desktop.core.protocol import MOTOR_COUNT, SYSTEM_STATE_IDLE


def controller_with_motor():
    controller = HandController()
    controller.comm = MagicMock()
    state = controller.current_state
    state.has_product_fingerprint = True
    state.product_fingerprint = {"device_id": "0000123456789ABC", "protocol_version": HOST_PROTOCOL_VERSION}
    state.system_state = SYSTEM_STATE_IDLE
    state.has_servo_angle_data = state.has_servo_route_status = True
    state.has_p4_time = True
    state.p4_time_ms = state.servo_feedback_p4_time_ms = 1000
    state.servo_abs_last_rx_ts = time.time()
    stamp(state, 'servo_abs_last_rx_ts')
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = 8
    state.multi_turn_unknown_bitmap = ((1 << MOTOR_COUNT) - 1) & ~8
    state.servo_angles[3] = 3000
    return controller


def test_direct_enable_holds_exact_abs_and_ignores_unselected_unknown():
    controller = controller_with_motor()
    state = controller.current_state
    state.active_motor_mask = 8
    def executed(command):
        if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
            _, token, mask, targets, _, _ = command
            from desktop.core.protocol import SYSTEM_STATE_RUN_DIRECT
            state.direct_target_token = token; state.direct_target_status = 2
            state.direct_target_applied_mask = mask; state.direct_target_rejected_mask = 0
            state.direct_target_result_rx_ts = state.system_status_last_rx_ts = time.time()
            stamp(state, 'direct_target_result_rx_ts')
            stamp(state, 'system_status_last_rx_ts')
            state.system_state = SYSTEM_STATE_RUN_DIRECT
            state.control_enabled = True; state.servo_target_owner = 1
        return True
    controller.comm.send_command.side_effect = executed
    ok, _, targets = controller.enable_direct_hold(8)
    assert ok and targets[3] == 3000
    controller.comm.invalidate_pending_direct_commands.assert_called_once()
    assert controller.comm.send_command.call_args_list == [
        call("start"), call(("motor_pos_abs", 1, 8, targets, 400, 20)),
    ]


@pytest.mark.parametrize("fault", ["stale", "unknown", "route", "raw_only", "range"])
def test_direct_enable_rejects_untrusted_or_clamped_hold(fault):
    controller = controller_with_motor()
    state = controller.current_state
    if fault == "stale":
        state.servo_abs_last_rx_ts -= 2
        stamp(state, 'servo_abs_last_rx_ts')
    elif fault == "unknown":
        state.multi_turn_unknown_bitmap |= 8
    elif fault == "route":
        state.servo_route_controllable_bitmap = 0
    elif fault == "raw_only":
        state.has_servo_angle_data = False
        state.has_servo_raw_data = True
    else:
        controller._clamp_motor_abs_for_current_session = lambda i, value: value - 1
    assert not controller.enable_direct_hold(8)[0]
    controller.comm.send_command.assert_not_called()


def test_release_only_connected_motor(monkeypatch):
    controller = controller_with_motor()
    state = controller.current_state
    state.servo_raw_positions[3] = 3000
    state.servo_raw_online[3] = True
    controller.prepare_calibration_turn_zero = MagicMock(return_value=(True, "ok", 8, 0))
    controller.set_active_scope = MagicMock(return_value=True)
    invalidate = MagicMock(return_value="saved")
    monkeypatch.setattr(core_logic, "invalidate_calibration_snapshot_scope", invalidate)

    def send(command):
        if isinstance(command, tuple) and command[0] == "motor_pos_abs":
            state.servo_angles[3] = command[1][3]
        return True

    controller.comm.send_command.side_effect = send
    result = controller.manual_prepare_all_motors()
    assert result.ok and result.reached_motor_mask == 8
    assert state.servo_angles[3] == 1500
    controller.set_active_scope.assert_called_once_with(8, 0, False)
    assert invalidate.call_args.kwargs["motor_mask"] == 8
    assert invalidate.call_args.kwargs["joint_mask"] == 4


def test_partial_reset_invalidates_only_success_without_motion(monkeypatch):
    controller = controller_with_motor()
    state = controller.current_state
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = 24
    controller.prepare_calibration_turn_zero = MagicMock(return_value=(False, "partial", 8, 16))
    invalidate = MagicMock(return_value="saved")
    monkeypatch.setattr(core_logic, "invalidate_calibration_snapshot_scope", invalidate)
    result = controller.manual_prepare_all_motors()
    assert not result.ok and result.reset_motor_mask == 8 and result.failed_motor_mask == 16
    assert invalidate.call_args.kwargs["motor_mask"] == 8
    controller.comm.send_command.assert_not_called()


def test_release_rejects_missing_identity_before_reset_or_motion():
    controller = controller_with_motor()
    controller.current_state.has_product_fingerprint = False
    controller.prepare_calibration_turn_zero = MagicMock()
    result = controller.manual_prepare_all_motors()
    assert not result.ok
    controller.prepare_calibration_turn_zero.assert_not_called()
    controller.comm.send_command.assert_not_called()


def test_selected_release_never_resets_invalidates_or_moves_other_motors(monkeypatch):
    controller = controller_with_motor()
    state = controller.current_state
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = 24
    state.servo_raw_positions[3] = state.servo_angles[3]
    state.servo_raw_online[3] = True
    state.servo_angles[4] = 2500
    controller.prepare_calibration_turn_zero = MagicMock(return_value=(True, 'ok', 8, 0))
    controller.set_active_scope = MagicMock(return_value=True)
    invalidate = MagicMock(return_value='saved')
    monkeypatch.setattr(core_logic, 'invalidate_calibration_snapshot_scope', invalidate)

    def send(command):
        if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
            assert command[1][4] == 2500
            state.servo_angles[3] = command[1][3]
        return True

    controller.comm.send_command.side_effect = send
    result = controller.manual_prepare_all_motors(motor_mask=8)
    assert result.ok and result.reset_motor_mask == result.reached_motor_mask == 8
    assert controller.prepare_calibration_turn_zero.call_args.args[0] == 8
    assert invalidate.call_args.kwargs['motor_mask'] == 8
    assert invalidate.call_args.kwargs['joint_mask'] == 4
    controller.set_active_scope.assert_called_once_with(8, 0, False)
    assert state.servo_angles[4] == 2500


def test_selected_release_requires_every_requested_motor_before_any_write():
    controller = controller_with_motor()
    controller.prepare_calibration_turn_zero = MagicMock()
    result = controller.manual_prepare_all_motors(motor_mask=24)
    assert not result.ok and 'offline or unroutable' in result.message
    controller.prepare_calibration_turn_zero.assert_not_called()
    controller.comm.send_command.assert_not_called()
