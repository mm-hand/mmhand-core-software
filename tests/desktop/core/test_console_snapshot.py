"""Quality presentation stays aligned with P4 warning boundaries, without hardware."""
from pathlib import Path
import pytest
from desktop.core.console_snapshot import joint_quality, motor_quality
from desktop.core.protocol import DUAL_TENDON_JOINT_INDICES

@pytest.mark.parametrize('difference,status', [(360, 'READY'), (361, 'OFFSET')])
def test_joint_offset_boundary(difference, status):
    assert joint_quality(True, 4000 + difference, 4000) == [status]

@pytest.mark.parametrize('offset,shortfall,expected', [
    (349, 299, ['READY']), (350, 299, ['WEAK RETURN']),
    (349, 300, ['HIGH FRICTION']), (350, 300, ['WEAK RETURN', 'HIGH FRICTION'])])
def test_motor_warning_boundaries(offset, shortfall, expected):
    result = dict(success=True, hasAssemblyWarningFields=True, autoMinOffsetCounts=offset, autoMaxShortfallCounts=shortfall)
    assert motor_quality(True, result) == expected


@pytest.mark.parametrize('joint', DUAL_TENDON_JOINT_INDICES)
@pytest.mark.parametrize('offset,shortfall,expected', [
    (349, 299, ['READY']), (350, 299, ['HIGH FRICTION']),
    (349, 300, ['HIGH FRICTION']), (350, 300, ['HIGH FRICTION'])])
def test_antagonistic_joints_classify_return_and_travel_as_friction(joint, offset, shortfall, expected):
    result = dict(success=True, hasAssemblyWarningFields=True,
                  autoMinOffsetCounts=offset, autoMaxShortfallCounts=shortfall)
    assert motor_quality(True, result, joint=joint) == expected


@pytest.mark.parametrize('joint', [j for j in range(21) if j not in DUAL_TENDON_JOINT_INDICES])
def test_single_tendon_joints_keep_weak_return_classification(joint):
    result = dict(success=True, hasAssemblyWarningFields=True,
                  autoMinOffsetCounts=350, autoMaxShortfallCounts=300)
    assert motor_quality(True, result, joint=joint) == ['WEAK RETURN', 'HIGH FRICTION']


@pytest.mark.parametrize('joint', DUAL_TENDON_JOINT_INDICES)
@pytest.mark.parametrize('reason', [1, 2, 3])
def test_antagonistic_warning_bits_keep_friction_classification_below_thresholds(joint, reason):
    result = dict(success=True, hasAssemblyWarningFields=True, assemblyWarningReason=reason,
                  autoMinOffsetCounts=0, autoMaxShortfallCounts=0)
    assert motor_quality(True, result, joint=joint) == ['HIGH FRICTION']
    assert result['assemblyWarningReason'] == reason

def test_missing_stale_failed_and_pending_are_not_ready():
    assert motor_quality(True, {}) == ['UNKNOWN']
    assert motor_quality(True, {'hasAssemblyWarningFields': True}) == ['UNKNOWN']
    assert motor_quality(True, {'success': False}) == ['FAILED']
    assert motor_quality(False, {}) == ['PENDING']
    assert motor_quality(True, {}, fresh=False) == ['UNKNOWN']
    assert joint_quality(True, 4000, 4000, 'INSTALLED_NOT_SAVED') == ['FAILED']
    assert joint_quality(True, 4000, 4000, 'DRAFT') == ['PENDING']
    assert joint_quality(True, 4000, 4000, fresh=False) == ['UNKNOWN']

def test_warning_contract_matches_firmware():
    root = Path(__file__).resolve().parents[3]
    text = (root / 'ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.h').read_text(encoding='utf-8')
    for name, value in [('AutoMinAssembly', 350), ('AutoMaxShortfall', 300), ('UrdfManualSpan', 360)]:
        assert f'kCalibration{name}WarnCounts = {value};' in text
    assert 'autoMinOffsetCounts >= kCalibrationAutoMinAssemblyWarnCounts' in text
    assert 'autoMaxShortfallCounts >= kCalibrationAutoMaxShortfallWarnCounts' in text
    assert 'urdfManualSpanDifferenceCounts >' in text
