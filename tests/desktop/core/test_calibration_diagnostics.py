from types import SimpleNamespace

import re

from desktop.core.calibration_diagnostics import CALIBRATION_DIAGNOSTICS, format_calibration_diagnostic
from desktop.core.protocol import (
    CALIB_DIAG_REASON_LEGACY_ANGLE_NOT_MOVING,
    CALIB_DIAG_REASON_LEGACY_CAN_FEEDBACK_TIMEOUT,
    CALIB_DIAG_REASON_LEGACY_INTERNAL_ERROR,
    CALIB_DIAG_REASON_LEGACY_LOAD_NOT_REACHED,
    CALIB_DIAG_REASON_LEGACY_SERVO_FEEDBACK_TIMEOUT,
    CALIB_DIAG_REASON_LEGACY_TELEMETRY_TIMEOUT,
    CALIB_DIAG_REASON_LEGACY_TURN_UNKNOWN,
    CALIB_DIAG_REASON_LEGACY_WRONG_DIRECTION,
    CALIB_DIAG_REASON_LEGACY_ZERO_RETURN_TIMEOUT,
)


def test_all_reserved_legacy_reasons_remain_readable():
    reasons = {
        CALIB_DIAG_REASON_LEGACY_CAN_FEEDBACK_TIMEOUT,
        CALIB_DIAG_REASON_LEGACY_SERVO_FEEDBACK_TIMEOUT,
        CALIB_DIAG_REASON_LEGACY_TELEMETRY_TIMEOUT,
        CALIB_DIAG_REASON_LEGACY_TURN_UNKNOWN,
        CALIB_DIAG_REASON_LEGACY_LOAD_NOT_REACHED,
        CALIB_DIAG_REASON_LEGACY_ANGLE_NOT_MOVING,
        CALIB_DIAG_REASON_LEGACY_WRONG_DIRECTION,
        CALIB_DIAG_REASON_LEGACY_ZERO_RETURN_TIMEOUT,
        CALIB_DIAG_REASON_LEGACY_INTERNAL_ERROR,
    }

    assert reasons == {2, 3, 4, 7, 9, 10, 11, 12, 19}
    assert all(CALIBRATION_DIAGNOSTICS[reason].legacy for reason in reasons)

    for reason in reasons:
        summary = format_calibration_diagnostic(
            SimpleNamespace(calib_diag_valid=True, calib_diag_reason=reason)
        )
        assert summary["active"] is True
        assert summary["title"].startswith("Legacy")
        assert summary["suggestions"]
        assert not re.search(r"[\u4e00-\u9fff]", " ".join(summary["lines"] + summary["suggestions"]))


def test_calibration_reason_titles_are_unique_and_english():
    titles = [spec.title for spec in CALIBRATION_DIAGNOSTICS.values()]
    assert len(titles) == len(set(titles))
    assert not any(re.search(r"[\u4e00-\u9fff]", title) for title in titles)
