"""Release 只作废旧坐标锚点，可选记账失败不得挡住后续坐标事务。

2026-11-15 的现场：Release 归零后写新锚点被标定包范围拒绝，内存里留下
``_servo_rebase_zero_capture_mask``，之后每次归零都被 "previous servo
coordinate transaction is awaiting its result" 挡下，直到重启 App。
"""

from _clock_helpers import stamp

import time
from unittest.mock import Mock

import pytest

from desktop.core import core_logic
from desktop.core.core_logic import HandController, ServoRebaseZeroResult
from desktop.core.protocol import (
    CMD_RESET, HOST_PROTOCOL_VERSION, PROTO_ACK_STATUS_QUEUED, SYSTEM_STATE_IDLE,
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
    state.has_servo_rebase_zero_result = True
    state.servo_rebase_zero_result_token = 7
    state.has_servo_angle_data = True
    state.servo_feedback_p4_time_ms = 100
    result._invalidate_servo_rebase_zero_anchors = Mock()
    result._persist_multiturn_state = Mock(return_value=True)
    monkeypatch.setattr(core_logic, "invalidate_calibration_snapshot_scope", Mock(return_value="saved"))
    monkeypatch.setattr(core_logic, "SERVO_REBASE_ZERO_RESULT_TIMEOUT_S", 0)
    return result


def publish(controller, mask, success):
    state = controller.current_state
    token = state.servo_rebase_zero_result_token + 1
    state.servo_rebase_zero_result_token = token
    state.servo_rebase_zero_success_mask = success
    state.servo_rebase_zero_failed_mask = mask & ~success
    state.servo_rebase_zero_result_last_rx_ts = time.time()
    stamp(state, 'servo_rebase_zero_result_last_rx_ts')
    state.servo_rebase_zero_result_p4_time_ms = token * 100
    state.servo_feedback_p4_time_ms = token * 100 + 1
    state.multi_turn_unknown_bitmap = mask & ~success
    return state


def scripted_ack(controller, outcomes):
    pending = iter(outcomes)

    def ack(command, _timeout, received_after_monotonic):
        if command == CMD_RESET:
            return command, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic
        commands = [
            call.args[0] for call in controller.comm.send_command.call_args_list
            if isinstance(call.args[0], tuple)
            and call.args[0][0] in ("servo_rebase_zero", "motor_abs_zero")
        ]
        publish(controller, commands[-1][1], next(pending))
        return command, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack


def test_release_rebase_only_invalidates_anchors(controller):
    scripted_ack(controller, [7])
    result = controller.rebase_servo_abs_zero(7, capture_anchor=False)
    assert result.ok, result.message
    assert controller._servo_rebase_zero_unresolved_mask == 0
    assert controller._servo_rebase_zero_capture_mask == 0
    controller._invalidate_servo_rebase_zero_anchors.assert_called_once_with(7)
    controller._persist_multiturn_state.assert_not_called()


def test_failed_anchor_write_cannot_block_the_next_release(controller):
    # 复现现场：锚点写盘失败（标定包范围拒绝）不再影响 Release，也不留阻塞位。
    controller._persist_multiturn_state.return_value = False
    scripted_ack(controller, [7, 7])
    first = controller.rebase_servo_abs_zero(7, capture_anchor=False)
    second = controller.rebase_servo_abs_zero(7, capture_anchor=False)
    assert first.ok and second.ok, second.message
    assert "awaiting its result" not in second.message
    assert controller._servo_rebase_zero_capture_mask == 0
    commands = [call.args[0] for call in controller.comm.send_command.call_args_list]
    assert commands.count(("servo_rebase_zero", 7)) == 2
    controller._persist_multiturn_state.assert_not_called()


def test_manual_prepare_reset_requests_invalidate_only(controller):
    seen = {}

    def rebase(mask, **kwargs):
        seen["mask"] = mask
        seen.update(kwargs)
        return ServoRebaseZeroResult(True, "zero", 1, mask, 0)

    controller.rebase_servo_abs_zero = rebase
    controller.invalidate_runtime_calibration_scope = Mock()
    reset, failure = controller._manual_prepare_reset(7, lambda: False, lambda action: action())
    assert failure is None
    assert reset[0] is True
    assert seen["mask"] == 7 and seen["capture_anchor"] is False
