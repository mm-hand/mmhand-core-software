"""Quality presentation stays aligned with P4 warning boundaries, without hardware."""
from pathlib import Path
from types import SimpleNamespace
import pytest
from desktop.core.console_snapshot import (calibration_status_badge, direct_zero_required,
                                      joint_quality, motor_quality)
from desktop.core.protocol import DUAL_TENDON_JOINT_INDICES

@pytest.mark.parametrize('difference,status', [(360, 'READY'), (361, 'OFFSET')])
def test_joint_offset_boundary(difference, status):
    assert joint_quality(True, 4000 + difference, 4000) == [status]

@pytest.mark.parametrize('reason,expected', [(0, ['READY']), (1, ['WEAK RETURN']),
    (2, ['HIGH FRICTION']), (3, ['WEAK RETURN', 'HIGH FRICTION'])])
@pytest.mark.parametrize('measured', [0, 10000])
def test_motor_quality_uses_p4_bits_even_when_raw_measurements_disagree(reason, expected, measured):
    result = dict(success=True, hasAssemblyWarningFields=True, assemblyWarningReason=reason,
                  autoMinOffsetCounts=measured, autoMaxShortfallCounts=measured)
    assert motor_quality(True, result) == expected


@pytest.mark.parametrize('joint', DUAL_TENDON_JOINT_INDICES)
@pytest.mark.parametrize('reason', [1, 2, 3])
def test_antagonistic_warning_bits_describe_friction_not_spring_return(joint, reason):
    result = dict(success=True, hasAssemblyWarningFields=True, assemblyWarningReason=reason,
                  autoMinOffsetCounts=0, autoMaxShortfallCounts=0)
    assert motor_quality(True, result, joint=joint) == ['HIGH FRICTION']


@pytest.mark.parametrize('joint', [j for j in range(21) if j not in DUAL_TENDON_JOINT_INDICES])
def test_single_tendon_warning_bits_keep_weak_return_classification(joint):
    result = dict(success=True, hasAssemblyWarningFields=True, assemblyWarningReason=3,
                  autoMinOffsetCounts=0, autoMaxShortfallCounts=0)
    assert motor_quality(True, result, joint=joint) == ['WEAK RETURN', 'HIGH FRICTION']


def test_missing_stale_failed_and_pending_are_not_ready():
    assert motor_quality(True, {}) == ['UNKNOWN']
    assert motor_quality(True, dict(success=True, hasAssemblyWarningFields=True, autoMinOffsetCounts=500, autoMaxShortfallCounts=500)) == ['UNKNOWN']
    assert motor_quality(True, {'hasAssemblyWarningFields': True}) == ['UNKNOWN']
    assert motor_quality(True, {'success': False}) == ['FAILED']
    assert motor_quality(False, {}) == ['PENDING']
    assert motor_quality(True, {}, fresh=False) == ['UNKNOWN']
    assert joint_quality(True, 4000, 4000, 'INSTALLED_NOT_SAVED') == ['FAILED']
    assert joint_quality(True, 4000, 4000, 'DRAFT') == ['PENDING']
    assert joint_quality(True, 4000, 4000, fresh=False) == ['UNKNOWN']


def _badge_state(**overrides):
    values = dict(
        calib_status='READY',
        encoders=[object()] * 21,
        manual_reference_valid_bitmap=0x0000FF00,
        active_joint_mask=0x0000FF00,
        auto_calibration_valid_bitmap=0x00000F00,
        encoder_online_bitmap=0x00001FFF,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def _direct_zero_state(**overrides):
    values = dict(servo_online_bitmap=0b111, motor_abs_limit_valid_bitmap=0)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_direct_zero_required_only_without_any_motor_calibration():
    assert direct_zero_required(_direct_zero_state(), fresh=True)
    assert not direct_zero_required(_direct_zero_state(), fresh=False)
    # 载入过 motor calibration（有 ABS 限位）：不再引导归零。
    assert not direct_zero_required(
        _direct_zero_state(motor_abs_limit_valid_bitmap=0b011), fresh=True)
    # 没有在线电机：没有可归零的对象。
    assert not direct_zero_required(_direct_zero_state(servo_online_bitmap=0), fresh=True)


def test_calibration_badge_denominator_is_stable_not_encoder_online():
    state = _badge_state()
    assert calibration_status_badge(state) == 'NEED CALIBRATION 4/8'
    # Encoder telemetry fluctuation must not change the installed/manual scope.
    state.encoder_online_bitmap = 0x00000003
    assert calibration_status_badge(state) == 'NEED CALIBRATION 4/8'
    state.auto_calibration_valid_bitmap = 0x0000FF00
    assert calibration_status_badge(state) == 'READY 8/8'


def test_calibration_badge_without_known_scope_has_no_jumping_fraction():
    state = _badge_state(manual_reference_valid_bitmap=0, active_joint_mask=0)
    assert calibration_status_badge(state) == 'NEED CALIBRATION'
    state.active_joint_mask = 0x00000F00
    state.auto_calibration_valid_bitmap = 0
    assert calibration_status_badge(state) == 'NEED CALIBRATION 0/4'


def test_calibration_badge_keeps_calibrating_and_stale_semantics():
    assert calibration_status_badge(_badge_state(calib_status='PENDING')) == 'CALIBRATING'
    assert calibration_status_badge(_badge_state(), fresh=False) == 'UNKNOWN'


def test_warning_contract_matches_firmware():
    root = Path(__file__).resolve().parents[3]
    text = (root / 'ESP32-P4/src/safety/policies/CalibrationSafetyPolicy.h').read_text(encoding='utf-8')
    for name, value in [('AutoMinAssembly', 350), ('AutoMaxShortfall', 300), ('UrdfManualSpan', 360)]:
        assert f'kCalibration{name}WarnCounts = {value};' in text
    assert 'autoMinOffsetCounts >= kCalibrationAutoMinAssemblyWarnCounts' in text
    assert 'autoMaxShortfallCounts >= kCalibrationAutoMaxShortfallWarnCounts' in text
    assert 'urdfManualSpanDifferenceCounts >' in text


@pytest.mark.parametrize('code,expected', [
    (0, 'Stopped (0)'), (1, 'Idle (1)'), (2, 'Running (2)'), (4, 'Retired Calib (4)'),
    (5, 'Ready (5)'), (6, 'Direct (6)'), (8, 'Rope Init (8)'), (9, 'Recovery (9)')])
def test_p4_badge_speaks_plain_words(code, expected):
    """P4 的 7 种状态各自一个人话标签；四个状态不能再糊成同一个 'Disabled'。"""
    from desktop.core.console_snapshot import p4_state_badge
    # 编码器全部 angle_error=True，越界检查直接跳过，只验证标签本身。
    encoders = [SimpleNamespace(current_angle_deg=0.0, angle_error=True)] * 21
    state = SimpleNamespace(system_state=code, has_system_status_data=True, encoders=encoders)
    hand = SimpleNamespace(_workspace_mapping_version=0, _workspace_urdf_crc=0)
    assert p4_state_badge(state, hand, fresh=True) == expected


def test_p4_badge_unknown_without_fresh_status():
    from desktop.core.console_snapshot import p4_state_badge
    state = SimpleNamespace(system_state=5, has_system_status_data=False, encoders=[])
    hand = SimpleNamespace(_workspace_mapping_version=0, _workspace_urdf_crc=0)
    assert p4_state_badge(state, hand, fresh=False) == 'UNKNOWN'
