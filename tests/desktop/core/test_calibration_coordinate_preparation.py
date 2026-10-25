"""One batch zero, preserving calibrated helper coordinate systems."""
from copy import deepcopy
from unittest.mock import Mock
import pytest
from test_host_calibration_runtime import io


@pytest.fixture
def prepared_io(io):
    io.wait_servo_recovery = Mock(return_value=True)
    io.preflight_finger_aa = Mock(return_value=True)
    io.snapshot_store.load.return_value = {}
    io._rebase = Mock(return_value=True)
    return io


def test_one_batch_zero_keeps_calibrated_helpers_and_no_per_joint_zero(prepared_io):
    io = prepared_io
    state = io.controller.current_state
    state.motor_abs_limit_valid_bitmap = 3  # J00 is a calibrated midpoint helper.
    state.auto_calibration_valid_bitmap = 1
    state.joint_calib_results[0] = {'success': True, 'unchanged': 'LUT'}
    state.motor_abs_limit_min[:2] = [-123, -456]
    state.motor_abs_limit_max[:2] = [12345, 23456]
    before = deepcopy(state.joint_calib_results[0])
    manual = list(state.manual_reference_min_raw)
    # M02 is being calibrated; M05/M06 are uncalibrated other-finger helpers.
    mask = 4 | 3 | 96
    def zero(scope):
        io._clear_result_scope(scope)
        return True
    io._rebase.side_effect = zero
    assert io.prepare_coordinates((1,), mask)
    io._rebase.assert_called_once_with(4 | 96)
    assert state.joint_calib_results[0] == before
    assert state.motor_abs_limit_min[:2] == [-123, -456]
    assert state.motor_abs_limit_max[:2] == [12345, 23456]
    assert state.manual_reference_min_raw == manual
    assert io.prepare_joint(1, 4)[0]
    assert io.prepare_joint(4, 96)[0]
    io._rebase.assert_called_once()
    assert not io.prepare_coordinates((1,), mask)  # No late scope expansion or repeat zero.
    io._rebase.assert_called_once()


def test_helper_calibration_in_json_is_preserved_even_if_not_loaded(prepared_io):
    io = prepared_io
    io.snapshot_store.load.return_value = {'motor_abs_limit_valid_bitmap': '0x3'}
    assert io.prepare_coordinates((1,), 7)
    io._rebase.assert_called_once_with(4)


def test_unknown_calibrated_helper_restores_before_batch_zero(prepared_io):
    io = prepared_io
    state = io.controller.current_state
    state.motor_abs_limit_valid_bitmap = state.multi_turn_unknown_bitmap = 3
    sequence = []
    def restore(mask, *, cancelled):
        assert mask == io._helper_restore_mask == 3
        io._rebase.assert_not_called()
        assert not cancelled()
        sequence.append('restore')
        state.multi_turn_unknown_bitmap = 0
        return True, 'restored'
    io.controller.wait_for_multiturn_recovery = Mock(side_effect=restore)
    io._rebase.side_effect = lambda mask: sequence.append(('zero', mask)) or True
    assert io.prepare_coordinates((1,), 7)
    assert sequence == ['restore', ('zero', 4)]
    assert io._helper_restore_mask == 0


@pytest.mark.parametrize('ending', ['missing_anchor', 'stale_result', 'cancelled'])
def test_failed_helper_recovery_never_falls_back_to_zero(prepared_io, ending):
    io = prepared_io
    state = io.controller.current_state
    state.motor_abs_limit_valid_bitmap = state.multi_turn_unknown_bitmap = 3
    io.controller.wait_for_multiturn_recovery = Mock(return_value=(ending == 'stale_result', ending))
    assert not io.prepare_coordinates((1,), 7)
    assert 'helper coordinate recovery failed' in io.feedback_recovery_error
    assert io._helper_restore_mask == 0
    io._rebase.assert_not_called()


def test_no_zero_when_any_member_fails_configuration_or_feedback(prepared_io):
    io = prepared_io
    io.wait_servo_recovery.return_value = False
    assert not io.prepare_coordinates((1,), 7)
    io._rebase.assert_not_called()
    io.snapshot_store.load.assert_not_called()


def test_helper_calibration_read_error_does_not_destroy_old_coordinates(prepared_io):
    io = prepared_io
    io.snapshot_store.load.side_effect = ValueError('corrupt JSON')
    assert not io.prepare_coordinates((1,), 7)
    io._rebase.assert_not_called()


def test_only_explicit_boundary_retry_can_zero_again(prepared_io):
    io = prepared_io
    assert io.prepare_coordinates((1,), 4)
    io.rebase_retry_requested = True
    assert io.prepare_joint(1, 4)[0]
    io._rebase.assert_called_once_with(4)
    assert io.rebase_joint_for_retry(1, 4)
    assert io._rebase.call_count == 2


def test_motor_only_selection_zeroes_the_whole_selected_joint(prepared_io):
    io = prepared_io
    io.selected_joint_mask = 0
    io.selected_motor_mask = 1
    io.controller.current_state.motor_abs_limit_valid_bitmap = 2
    assert io.prepare_coordinates((0,), 3)
    io._rebase.assert_called_once_with(3)
    assert io.selected_joint_mask == 1 and io.selected_motor_mask == 3
