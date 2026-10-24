"""AA positioning/holding uses the real runtime and simulated feedback only."""

from _clock_helpers import stamp
from copy import deepcopy
import math
import time
from unittest.mock import Mock

import pytest

from desktop.core.host_calibration.config import (
    default_auto_calibration_params, finger_aa_joint, joint_motors,
)
from desktop.core.host_calibration.dual_motor import WorkspaceCacheEntry, POSE_A, WORKSPACE_DUAL_JOINTS
from desktop.core.host_calibration.finish import dual_lut_targets
from desktop.core.host_calibration import flow, single_motor
from desktop.core.host_calibration.dependencies import (
    missing_aa_joint_mask,
    required_aa_joint_mask,
    usable_snapshot_aa_result,
)
from desktop.core.host_calibration.runtime import HostCalibrationIO
from desktop.core.protocol import manual_encoder_reference_entry_crc
from test_host_calibration_runtime import io


@pytest.fixture
def centered_io(io, monkeypatch):
    state = io.controller.current_state
    state.manual_reference_committed = True
    state.auto_calibration_valid_bitmap = sum(1 << aa for aa in range(0, 20, 4))
    state.motor_abs_limit_valid_bitmap = (1 << 27) - 1
    state.motor_abs_limit_min, state.motor_abs_limit_max = [-30000] * 27, [30000] * 27
    for aa in range(0, 20, 4):
        lo, hi = io.urdf_limits(aa)
        count = math.ceil((hi - lo) / 5) + 1
        state.joint_calib_results[aa] = {
            'success': True, 'scaled': True, 'hasManualScaledV4Fields': True,
            'mappingVersion': 4, 'urdfConfigCrc': io.urdf_config_crc(),
            'manualReferenceCrc': manual_encoder_reference_entry_crc(aa, 6000, 10000),
            'manualSpanCounts': 4000,
            'currentMinDeg': lo, 'currentMaxDeg': hi, 'angleMin': lo, 'angleMax': hi,
            'tendonLutValid': True, 'tendonLutCount': count,
            'tendonLutMotor': list(joint_motors(aa)),
            'tendonLutAngleStartDeg': lo, 'tendonLutAngleStepDeg': 5,
            'tendonLutMotorAbs': [[1000 + 100 * i for i in range(count)],
                                  [2000 - 50 * i for i in range(count)]],
        }
    clock = [0]
    monkeypatch.setattr(io, 'now_ms', lambda: clock[0])
    monkeypatch.setattr(io, 'owns_control', lambda: not io.is_cancelled())

    def sleep(ms):
        clock[0] += ms
        state.encoder_raw_p4_time_ms += 20
        state.servo_feedback_p4_time_ms += 20
        state.p4_time_ms = state.system_status_p4_time_ms = state.encoder_raw_p4_time_ms
        state.encoder_raw_last_rx_ts = state.servo_abs_last_rx_ts = time.time()
        stamp(state, 'encoder_raw_last_rx_ts')
        stamp(state, 'servo_abs_last_rx_ts')
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')

    def publish(targets, speed, acc):
        for motor, target in targets.items():
            state.servo_angles[motor] = target
        for aa in range(0, 20, 4):
            if all(m in targets for m in joint_motors(aa)):
                state.encoders[aa].raw_sensor = 8000
        return True

    monkeypatch.setattr(io, 'sleep_ms', sleep)
    monkeypatch.setattr(io, 'publish', Mock(side_effect=publish))
    return io


@pytest.mark.parametrize('aa', range(0, 20, 4))
@pytest.mark.parametrize('offset', [1, 2, 3])
def test_five_finger_midpoint_is_reached_then_held_at_fixed_targets(centered_io, aa, offset):
    io = centered_io
    joint = aa + offset
    assert finger_aa_joint(joint) == aa
    assert io.preflight_finger_aa((joint,))
    assert io.finger_aa_motor_mask((joint,)) == sum(1 << m for m in joint_motors(aa))
    before = deepcopy(io.controller.current_state.joint_calib_results)
    io._dual_workspace_cache = ({aa: WorkspaceCacheEntry(valid=True, current_pose=POSE_A)}
                                if aa in WORKSPACE_DUAL_JOINTS else {})
    assert io.prepare_finger_aa(joint) == (True, '')
    assert io.now_ms() >= 60  # Three new feedback pairs, not merely command acceptance.
    if aa in WORKSPACE_DUAL_JOINTS:
        assert not io._dual_workspace_cache[aa].valid
    assert io._aa_hold[2] == sum(io.urdf_limits(aa)) / 2
    fixed = dict(io._aa_hold[3])
    for motor in fixed:
        io.controller.current_state.servo_angles[motor] += 20
    held = io.hold_targets_except((aa, joint), joint, 0, default_auto_calibration_params())
    assert {m: held[m] for m in fixed} == fixed
    assert io.controller.current_state.joint_calib_results == before
    io.snapshot_store.invalidate_scope.assert_not_called()
    io.snapshot_store.merge_calibration.assert_not_called()


@pytest.mark.parametrize('fault', ['missing', 'manual_crc', 'urdf_crc', 'unknown', 'limits', 'lut', 'span'])
def test_untrusted_unselected_aa_is_rejected_before_reset_or_motion(centered_io, fault):
    io = centered_io
    state = io.controller.current_state
    if fault == 'missing':
        state.joint_calib_results[0] = None
    elif fault == 'manual_crc':
        state.joint_calib_results[0]['manualReferenceCrc'] ^= 1
    elif fault == 'urdf_crc':
        state.joint_calib_results[0]['urdfConfigCrc'] ^= 1
    elif fault == 'unknown':
        state.multi_turn_unknown_bitmap |= 1
    elif fault == 'limits':
        state.motor_abs_limit_valid_bitmap &= ~1
    elif fault == 'span':
        state.joint_calib_results[0]['manualSpanCounts'] = 0
    else:
        state.joint_calib_results[0]['tendonLutValid'] = False
    io.wait_servo_recovery = Mock(side_effect=AssertionError('must preflight before reset'))
    io.finish_session = Mock(return_value=(False, 'stopped'))
    result = flow.run(io, 2, 4, lambda *_: None)
    assert not result.success and 'J00' in result.message
    io.wait_servo_recovery.assert_not_called()
    io.publish.assert_not_called()


def test_selected_aa_can_be_calibrated_first_and_uses_committed_host_result(centered_io):
    io = centered_io
    io.selected_joint_mask |= 1
    state = io.controller.current_state
    result = deepcopy(state.joint_calib_results[0])
    state.joint_calib_results[0] = None
    state.auto_calibration_valid_bitmap &= ~1
    assert io.preflight_finger_aa((0, 1, 2, 3))
    assert not io.prepare_finger_aa(1)[0]  # Not ready until commit succeeds.
    io._committed_joint_results[0] = result
    assert io.prepare_finger_aa(1)[0]


@pytest.mark.parametrize('aa', range(0, 20, 4))
def test_whole_finger_sequence_centers_after_joint_prepare_before_each_search(centered_io, monkeypatch, aa):
    io = centered_io
    joints = tuple(range(aa, aa + 4))
    io.selected_joint_mask = sum(1 << j for j in joints)
    io.selected_motor_mask = sum(1 << m for j in joints for m in joint_motors(j))
    result = deepcopy(io.controller.current_state.joint_calib_results[aa])
    io.controller.current_state.joint_calib_results[aa] = None
    io.controller.current_state.auto_calibration_valid_bitmap &= ~(1 << aa)
    events = []
    io.wait_servo_recovery = Mock(return_value=True)
    io.wait_encoder_recovery = Mock(return_value=True)
    io.prepare_joint = lambda joint, mask: (events.append(('prepare', joint)) is None, '')
    io.finish_session = Mock(return_value=(True, 'done'))

    def dual(*args):
        events.append(('AA', aa))
        return dict(success=True, result=result, limits={})

    def commit(joint, payload, limits):
        if joint == aa:
            io._committed_joint_results[aa] = payload
        return True, ''

    def search(ctx, port, params, targets):
        assert io._aa_hold[0] == aa
        assert io.controller.current_state.encoders[aa].raw_sensor == 8000
        assert all(targets[m] == value for m, value in io._aa_hold[3].items())
        events.append(('search', ctx.joint))
        ctx.state = single_motor.AutoCalibState.DONE
        ctx.result = {'success': True}

    monkeypatch.setattr(flow, 'run_dual', dual)
    monkeypatch.setattr(io, 'commit_joint', commit)
    monkeypatch.setattr(single_motor, 'step_single', search)
    session = flow.run(io, io.selected_joint_mask, io.selected_motor_mask, lambda *_: None)
    assert session.success
    assert events == [('prepare', aa), ('AA', aa)] + [
        event for j in joints[1:] for event in [('prepare', j), ('search', j)]]
    assert io.publish.call_count > 6  # Repeated low-speed corrections, then ordinary search batches.


def test_midpoint_holds_abort_on_three_distinct_drift_frames(centered_io):
    io = centered_io
    assert io.prepare_finger_aa(1)[0]
    state = io.controller.current_state
    state.encoders[0].raw_sensor = 7000
    params = default_auto_calibration_params()
    for _ in range(2):
        io.sleep_ms(20)
        assert io.hold_targets_except((1,), 1, 0, params) is not None
    # Re-reading one packet does not count as another drift frame.
    assert io.hold_targets_except((1,), 1, 0, params) is not None
    io.sleep_ms(20)
    assert io.hold_targets_except((1,), 1, 0, params) is None
    assert 'drifted' in io.motion_publish_error


@pytest.mark.parametrize('fault', ['stall', 'frozen', 'stale', 'publish', 'cancel', 'load', 'limit'])
def test_midpoint_failures_never_enter_single_search(centered_io, monkeypatch, fault):
    io = centered_io
    if fault == 'stall':
        io.publish.side_effect = None
        io.publish.return_value = True  # Executed, but neither motors nor angle arrived.
    elif fault == 'frozen':
        clock = [0]
        monkeypatch.setattr(io, 'now_ms', lambda: clock[0])
        monkeypatch.setattr(io, 'sleep_ms', lambda ms: clock.__setitem__(0, clock[0] + ms))
    elif fault == 'stale':
        io.controller.current_state.encoder_raw_last_rx_ts -= 10
        stamp(io.controller.current_state, 'encoder_raw_last_rx_ts')
    elif fault == 'publish':
        io.publish.side_effect = None
        io.publish.return_value = False
    elif fault == 'load':
        monkeypatch.setattr(io, '_ensure_motion', lambda: True)
        monkeypatch.setattr(io, 'servo_load', lambda motor: 800)
        io.set_workspace_read_mask(io.finger_aa_motor_mask((1,)))
        io.publish.side_effect = lambda *args: HostCalibrationIO.publish(io, *args)
    elif fault == 'limit':
        io.controller.current_state.motor_abs_limit_max[0] = -40000
    else:
        io.cancel()
    monkeypatch.setattr(single_motor, 'step_single', Mock(side_effect=AssertionError('search started')))
    outcome = single_motor.run_single(io, 1, default_auto_calibration_params(), (1,), 0, 0, 2)
    assert not outcome['success'] and outcome['aborted']
    assert 'J00' in outcome['reason']
    assert io._aa_hold is None
    if fault == 'stall':
        assert 'did not follow' in outcome['reason']
        assert '20 seconds' in outcome['reason']
        assert 20000 <= io.now_ms() <= 20100
        assert max(abs(call.args[0][0]) for call in io.publish.call_args_list) <= 40
    if fault == 'limit':
        assert 'limits' in outcome['reason']
        io.publish.assert_not_called()


@pytest.mark.parametrize('aa', range(0, 20, 4))
@pytest.mark.parametrize('start_raw', [6384, 10384])
def test_encoder_driven_midpoint_handles_slack_from_either_side(centered_io, aa, start_raw):
    io = centered_io
    state = io.controller.current_state
    primary, secondary = joint_motors(aa)
    state.encoders[aa].raw_sensor = start_raw
    state.servo_angles[primary], state.servo_angles[secondary] = 5000, -4000
    slack = [120]

    def move(targets, speed, acc):
        assert speed <= 100
        delta = targets[primary] - state.servo_angles[primary]
        assert abs(delta) <= 10  # No large jump to a LUT motor position.
        assert targets[secondary] - state.servo_angles[secondary] == -delta
        for motor, value in targets.items():
            state.servo_angles[motor] = value
        takeup = min(slack[0], abs(delta))
        slack[0] -= takeup
        motion = (abs(delta) - takeup) * (1 if delta >= 0 else -1)
        state.encoders[aa].raw_sensor = (state.encoders[aa].raw_sensor + motion * 5) % 16384
        return True

    io.publish.side_effect = move
    assert io.prepare_finger_aa(aa + 1)[0]
    result, center, fixed = io._aa_hold[1:]
    angle = io._finger_aa_feedback(aa, result)[1]
    assert abs(angle - center) <= 2
    assert slack[0] == 0
    assert io.now_ms() > 1000
    assert fixed == {m: state.servo_angles[m] for m in (primary, secondary)}
    assert fixed != dual_lut_targets(result, (primary, secondary), center)


def test_midpoint_timeout_is_bounded_even_when_angle_keeps_moving(centered_io):
    io = centered_io
    state = io.controller.current_state

    def never_arrive(targets, speed, acc):
        for motor, value in targets.items():
            state.servo_angles[motor] = value
        state.encoders[0].raw_sensor = 9900 if io.now_ms() % 1000 < 500 else 10000
        return True

    io.publish.side_effect = never_arrive
    ok, message = io.prepare_finger_aa(1)
    assert not ok and 'timed out' in message
    assert 60000 <= io.now_ms() <= 60100


def test_auxiliary_aa_is_not_added_to_reset_scope(centered_io):
    io = centered_io
    selected, reset = io.selected_motor_mask, io.preparation_reset_mask
    helper = io.finger_aa_motor_mask((1,))
    io.set_workspace_read_mask(helper)
    assert io._prepare_workspace_reset_scope()
    assert io.selected_motor_mask == selected
    assert io.preparation_reset_mask == reset & selected
    assert not io.preparation_reset_mask & helper


def test_thumb_cmc_does_not_require_or_move_aa(centered_io):
    io = centered_io
    assert finger_aa_joint(20) is None
    assert io.finger_aa_motor_mask((20,)) == 0
    assert io.prepare_finger_aa(20) == (True, '')
    io.publish.assert_not_called()


def test_general_lut_interpolation_preserves_short_last_interval_and_rejects_extrapolation():
    result = dict(success=True, scaled=True, mappingVersion=4, tendonLutValid=True,
        tendonLutCount=3, tendonLutMotor=[0, 1], tendonLutMotorAbs=[[0, 10, 20], [0, -10, -20]],
        tendonLutAngleStartDeg=-8, tendonLutAngleStepDeg=5, angleMax=1)
    assert dual_lut_targets(result, (0, 1), 0) == {0: 18, 1: -18}
    assert dual_lut_targets(result, (0, 1), -3.5) == {0: 9, 1: -9}
    assert dual_lut_targets(result, (0, 1), 2) is None


def _snapshot_with_aa(io, aa):
    state = io.controller.current_state
    snapshot = {
        'joint_calib_results': [None] * 21,
        'active_joint_mask': hex(1 << aa),
        'motor_abs_limit_valid_bitmap': hex((1 << 27) - 1),
    }
    snapshot['joint_calib_results'][aa] = deepcopy(state.joint_calib_results[aa])
    io.snapshot_store.load.return_value = snapshot
    return snapshot


def test_required_aa_mask_only_adds_unselected_dependencies():
    assert required_aa_joint_mask(1 << 1) == 1 << 0
    assert required_aa_joint_mask((1 << 0) | (1 << 1)) == 0
    assert required_aa_joint_mask(1 << 13) == 1 << 12
    assert required_aa_joint_mask(1 << 20) == 0


def test_host_snapshot_aa_result_satisfies_cross_session_dependency(centered_io):
    io = centered_io
    state = io.controller.current_state
    snapshot = _snapshot_with_aa(io, 0)
    assert usable_snapshot_aa_result(snapshot, 0, state, io.urdf_config_crc()) is not None
    assert missing_aa_joint_mask(1 << 1, snapshot, state, io.urdf_config_crc()) == 0

    # The P4 readback result is absent, but the host snapshot still resolves it.
    state.joint_calib_results[0] = None
    state.auto_calibration_valid_bitmap &= ~1
    result, _center, _limits = io._finger_aa_plan(0)
    assert result['manualSpanCounts'] == 4000
    assert io._finger_aa_plan(0)[0]['manualSpanCounts'] == 4000


@pytest.mark.parametrize('fault', ['missing', 'manual_crc', 'manual_span', 'urdf_crc', 'inactive', 'limits'])
def test_unusable_host_snapshot_requests_aa_recalibration(centered_io, fault):
    io = centered_io
    state = io.controller.current_state
    snapshot = _snapshot_with_aa(io, 0)
    result = snapshot['joint_calib_results'][0]
    if fault == 'missing':
        snapshot['joint_calib_results'][0] = None
    elif fault == 'manual_crc':
        result['manualReferenceCrc'] ^= 1
    elif fault == 'manual_span':
        result['manualSpanCounts'] = 0
    elif fault == 'urdf_crc':
        result['urdfConfigCrc'] ^= 1
    elif fault == 'inactive':
        snapshot['active_joint_mask'] = '0x0'
    else:
        snapshot['motor_abs_limit_valid_bitmap'] = '0x0'
    assert usable_snapshot_aa_result(snapshot, 0, state, io.urdf_config_crc()) is None
    assert missing_aa_joint_mask(1 << 1, snapshot, state, io.urdf_config_crc()) == 1


def test_midpoint_may_start_outside_the_trusted_window_and_move_inward(centered_io):
    # 拟合直线的端部外推与实际 ABS 有偏差：起点在窗口外 120 counts 时，
    # 向窗口内运动必须允许，否则第一次目标下发就中止。
    io = centered_io
    state = io.controller.current_state
    state.encoders[0].raw_sensor = 7600
    primary, secondary = joint_motors(0)
    state.servo_angles[primary], state.servo_angles[secondary] = -120, 120
    state.motor_abs_limit_min[primary], state.motor_abs_limit_max[primary] = -60, 400
    state.motor_abs_limit_min[secondary], state.motor_abs_limit_max[secondary] = -400, 60
    assert io.prepare_finger_aa(1) == (True, '')


def test_midpoint_still_aborts_when_the_next_target_moves_further_outside(centered_io):
    io = centered_io
    state = io.controller.current_state
    state.encoders[0].raw_sensor = 7600
    primary, secondary = joint_motors(0)
    state.servo_angles[primary], state.servo_angles[secondary] = 120, -120
    state.motor_abs_limit_min[primary], state.motor_abs_limit_max[primary] = -400, 60
    state.motor_abs_limit_min[secondary], state.motor_abs_limit_max[secondary] = -60, 400
    ok, message = io.prepare_finger_aa(1)
    assert not ok and 'trusted motor limits' in message
