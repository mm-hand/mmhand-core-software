"""Control badges reflect confirmed P4 execution, independently of UI selection."""

from _clock_helpers import stamp
import time
import pytest
from test_application_actions import hand
from desktop.core.protocol import (
    ControlMode, SERVO_TARGET_OWNER_CONTROL, SERVO_TARGET_OWNER_CALIBRATION,
    SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_RUN_DIRECT,
)


def running_control(hand, direct):
    state = hand._controller.current_state
    state.has_active_scope_status = True
    state.control_enabled = True
    state.servo_target_owner = SERVO_TARGET_OWNER_CONTROL
    state.control_mode = (ControlMode.DIRECT_MOTOR if direct else ControlMode.JOINT_ANGLE).value
    state.system_state = SYSTEM_STATE_RUN_DIRECT if direct else SYSTEM_STATE_RUN_FEEDBACK
    state.active_joint_mask = 1 << 7
    state.active_motor_mask = 1 << 11
    return state


@pytest.mark.parametrize('direct', [False, True])
def test_control_badges_use_confirmed_scope_even_when_selection_changes(hand, direct):
    running_control(hand, direct)
    hand._selected_joints = [6]
    hand._selected_motors = [10]
    snapshot = hand.application_snapshot()
    rows, key, enabled = (snapshot['motors'], 'direct_control_status', 11) if direct else (
        snapshot['joints'], 'feedback_control_status', 7)
    assert rows[enabled][key] == 'ENABLED'
    assert all(row[key] == 'DISABLED' for row in rows if row is not rows[enabled])
    inactive, inactive_key = (snapshot['joints'], 'feedback_control_status') if direct else (
        snapshot['motors'], 'direct_control_status')
    assert all(row[inactive_key] == 'DISABLED' for row in inactive)


@pytest.mark.parametrize('direct', [False, True])
@pytest.mark.parametrize('gate', ['preparation', 'calibration_owner', 'control_off', 'wrong_mode'])
def test_preparation_and_non_control_ownership_never_display_enabled(hand, direct, gate):
    state = running_control(hand, direct)
    if gate == 'preparation':
        state.system_state = SYSTEM_STATE_READY
    elif gate == 'calibration_owner':
        state.servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION
    elif gate == 'control_off':
        state.control_enabled = False
    else:
        state.control_mode = (ControlMode.JOINT_ANGLE if direct else ControlMode.DIRECT_MOTOR).value
    snapshot = hand.application_snapshot()
    rows, key = (snapshot['motors'], 'direct_control_status') if direct else (
        snapshot['joints'], 'feedback_control_status')
    assert all(row[key] == 'DISABLED' for row in rows)


@pytest.mark.parametrize('missing', ['scope', 'stale', 'disconnected'])
def test_unavailable_confirmation_displays_unknown_in_both_pages(hand, monkeypatch, missing):
    state = running_control(hand, True)
    if missing == 'scope':
        state.has_active_scope_status = False
    elif missing == 'stale':
        state.system_status_last_rx_ts = time.time() - 1
        stamp(state, 'system_status_last_rx_ts')
    else:
        monkeypatch.setattr(hand, 'is_comm_connected', lambda: False)
    snapshot = hand.application_snapshot()
    assert all(row['feedback_control_status'] == 'UNKNOWN' for row in snapshot['joints'])
    assert all(row['direct_control_status'] == 'UNKNOWN' for row in snapshot['motors'])
