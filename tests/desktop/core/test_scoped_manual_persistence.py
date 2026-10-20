"""Exercise controller persistence against a partial P4 session without hardware."""

from _clock_helpers import live_servo_feedback, stamp
from copy import deepcopy
import time
from unittest.mock import Mock

import pytest

from desktop.core.calibration_snapshot import read_snapshot, parse_mask
from desktop.core.core_logic import HandController
from desktop.core.protocol import ENCODER_COUNT, SYSTEM_STATE_IDLE
from test_calibration_snapshot import two_finger_snapshot


@pytest.fixture
def controller(monkeypatch, tmp_path):
    path = tmp_path / 'hand.mmcal.json'
    two_finger_snapshot(path)
    monkeypatch.setattr(HandController, 'calibration_snapshot_path', property(lambda self: path))
    monkeypatch.setattr(HandController, 'calibration_data_identity', property(lambda self: {}))
    controller = HandController()
    controller.comm = Mock()
    controller.comm.wait_for_ack.return_value = (0, 0, 0, time.time())
    monkeypatch.setattr(controller, 'query_manual_encoder_references', lambda *args: True)
    monkeypatch.setattr(controller, '_verify_manual_encoder_readback', Mock(return_value=True))
    state = controller.current_state
    state.manual_reference_valid_bitmap = 1 << 15
    state.manual_reference_min_raw[15] = 100
    state.manual_reference_max_raw[15] = 5000
    return controller, path


@pytest.mark.parametrize('clear', [False, True])
def test_controller_saves_only_explicit_manual_scope(controller, clear):
    controller, path = controller
    before = read_snapshot(path)
    if clear:
        mins, maxs = [0] * ENCODER_COUNT, [0] * ENCODER_COUNT
        assert controller.set_manual_encoder_references(mins, maxs, updated_joint_mask=1 << 15)
    else:
        assert controller.set_manual_encoder_reference(15, 100, 5100)
    after = read_snapshot(path)
    assert after['joint_calib_results'][11] == before['joint_calib_results'][11]
    assert after['joint_calib_results'][15] is None
    assert after['manual_reference_min_raw'][11] == 100
    assert after['manual_reference_max_raw'][11] == 5000
    assert parse_mask(after['manual_reference_valid_bitmap']) & (1 << 11)
    assert parse_mask(after['multiturn_anchor_valid_bitmap']) & (1 << 14)


def test_release_preserves_other_finger_in_runtime_and_json(controller):
    controller, path = controller
    before = read_snapshot(path)
    state = controller.current_state
    state.system_state = SYSTEM_STATE_IDLE
    state.joint_calib_results = deepcopy(before['joint_calib_results'])
    state.motor_abs_limit_valid_bitmap = (1 << 14) | (1 << 19)
    state.has_servo_angle_data = state.has_servo_route_status = True
    state.servo_abs_last_rx_ts = time.time()
    stamp(state, 'servo_abs_last_rx_ts')
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = (1 << 14) | (1 << 19)
    state.servo_angles[14], state.servo_angles[19] = 2000, 3000
    state.servo_raw_positions[19] = 3000
    state.servo_raw_online[19] = True

    def reset_selected(mask, **kwargs):
        assert mask == 1 << 19
        # Emulate the P4's scoped reset-result invalidation.
        state.joint_calib_results[15] = None
        state.motor_abs_limit_valid_bitmap &= ~mask
        return True, 'reset', mask, 0

    controller.prepare_calibration_turn_zero = Mock(side_effect=reset_selected)
    controller.set_active_scope = Mock(return_value=True)

    def move(command):
        if isinstance(command, tuple) and command[0] == 'motor_pos_abs':
            assert command[1][14] == 2000
            state.servo_angles[19] = command[1][19]
            state.has_direct_target_result = True
            state.direct_target_status = 2
            state.direct_target_applied_mask = 1 << 19
            state.system_state = 6
            state.control_enabled = True
        return True

    controller.comm.send_command.side_effect = move
    with live_servo_feedback(state):
        result = controller.manual_prepare_all_motors(motor_mask=1 << 19)
    assert result.ok and result.invalidated_joint_mask == 1 << 15
    assert state.joint_calib_results[11] == before['joint_calib_results'][11]
    assert state.motor_abs_limit_valid_bitmap == 1 << 14
    assert state.servo_angles[14] == 2000
    after = read_snapshot(path)
    assert after['joint_calib_results'][11] == before['joint_calib_results'][11]
    assert after['joint_calib_results'][15] is None
    assert after['manual_reference_min_raw'] == before['manual_reference_min_raw']
    assert after['manual_reference_max_raw'] == before['manual_reference_max_raw']
    assert after['multiturn_anchor_logical_abs'][14] == before['multiturn_anchor_logical_abs'][14]
