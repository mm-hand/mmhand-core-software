from types import SimpleNamespace

import re
import pytest

from desktop.core.calibration_diagnostics import (
    CALIBRATION_DIAGNOSTICS, format_calibration_diagnostic, format_calibration_warning,
)
from desktop.core.protocol import (
    CALIB_WARN_AUTO_MIN_OFFSET,
    DUAL_TENDON_JOINT_INDICES,
    ENCODER_COUNT,
)


def test_retired_reason_codes_have_no_compatibility_catalog():
    assert not {2,3,4,7,9,10,11,12,19}.intersection(CALIBRATION_DIAGNOSTICS)


def test_calibration_reason_titles_are_unique_and_english():
    titles = [spec.title for spec in CALIBRATION_DIAGNOSTICS.values()]
    assert len(titles) == len(set(titles))
    assert not any(re.search(r"[\u4e00-\u9fff]", title) for title in titles)


def return_warning_state(joint, warning_fields):
    result = dict(assemblyWarningReason=CALIB_WARN_AUTO_MIN_OFFSET,
                  hasAssemblyWarningFields=warning_fields, autoMinOffsetCounts=350,
                  autoMaxShortfallCounts=0)
    results, offsets = [None] * ENCODER_COUNT, [0] * ENCODER_COUNT
    results[joint], offsets[joint] = result, 350
    return SimpleNamespace(joint_calib_results=results, calibration_auto_min_offset_counts=offsets)


@pytest.mark.parametrize('joint', DUAL_TENDON_JOINT_INDICES)
@pytest.mark.parametrize('warning_fields', [False, True], ids=['legacy', 'current'])
def test_antagonistic_return_warning_explains_friction_for_current_and_legacy_results(joint, warning_fields):
    state = return_warning_state(joint, warning_fields)
    summary = format_calibration_warning(state, 1 << joint)
    assert summary['ready']
    assert f'J{joint:02d}:' in summary['message']
    assert '350 counts' in summary['message']
    assert 'excessive tendon friction' in summary['message']
    assert 'weak spring return' not in summary['message']
    assert 'does not block READY or feedback control' in summary['message']
    assert f'J{joint:02d}:reason=0x01' in summary['event_detail']
    assert state.joint_calib_results[joint]['assemblyWarningReason'] == CALIB_WARN_AUTO_MIN_OFFSET


@pytest.mark.parametrize('joint', [j for j in range(ENCODER_COUNT) if j not in DUAL_TENDON_JOINT_INDICES])
def test_single_tendon_return_warning_still_explains_spring_return(joint):
    summary = format_calibration_warning(return_warning_state(joint, True), 1 << joint)
    assert summary['ready']
    assert 'weak spring return' in summary['message']
    assert 'excessive tendon friction' not in summary['message']
