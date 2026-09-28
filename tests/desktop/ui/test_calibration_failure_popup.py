from types import SimpleNamespace

from desktop.core.protocol import (
    CALIB_DIAG_PHASE_DUAL_EXCHANGE,
    CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
    CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
    CALIB_DIAG_REASON_CONTROL_LOST,
    CALIB_DIAG_REASON_DUPLICATE_JOINT,
    CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED,
    CALIB_DIAG_REASON_ENCODER_ERROR,
    CALIB_DIAG_REASON_FEEDBACK_LOST,
    CALIB_DIAG_REASON_HOLD_TARGET_FAILED,
    CALIB_DIAG_REASON_JOINT_LIST_INVALID,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
    CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
    CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED,
    CALIB_DIAG_REASON_OVERLOAD,
    CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS,
    CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
    CALIB_DIAG_REASON_RESULT_COMMIT_FAILED,
    CALIB_DIAG_REASON_RESULT_SPAN_INVALID,
    CALIB_DIAG_REASON_SERVO_OFFLINE,
    CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT,
    CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
    CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT,
    CALIB_DIAG_REASON_TASK_START_FAILED,
    CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
    CALIB_DIAG_REASON_WRAP_TOO_LARGE,
    CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
)
from desktop.ui.ui_app import HandGUI


def test_popup_explains_result_span_failure_instead_of_generic_step_failure():
    state = SimpleNamespace(
        calib_diag_valid=True,
        calib_diag_reason=CALIB_DIAG_REASON_RESULT_SPAN_INVALID,
        calib_diag_phase=CALIB_DIAG_PHASE_DUAL_EXCHANGE,
        calib_diag_joint=12,
        calib_diag_motor=16,
        calib_diag_value_a=0,
        calib_diag_value_b=6668,
        calib_diag_value_c=3242,
        calib_diag_bitmap=(1 << 16) | (1 << 17),
        calib_diag_timestamp_ms=123456,
        calib_diag_sequence=9,
        calib_diag_completed_joint_mask=(1 << 8),
        calib_diag_failed_joint_mask=(1 << 12),
    )

    summary = HandGUI._calib_diag_summary(state)

    assert summary["active"] is True
    assert summary["title"] == "Invalid magnetic encoder travel"
    assert any("J12, M16" in line for line in summary["lines"])
    assert any("completed=0x00000100" in line for line in summary["lines"])
    assert any("encoder_span=0" in line for line in summary["lines"])


def test_every_firmware_calibration_reason_has_a_dedicated_popup_explanation():
    reasons = (
        CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
        CALIB_DIAG_REASON_ENCODER_ERROR,
        CALIB_DIAG_REASON_SERVO_OFFLINE,
        CALIB_DIAG_REASON_OVERLOAD,
        CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
        CALIB_DIAG_REASON_WRAP_TOO_LARGE,
        CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
        CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
        CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT,
        CALIB_DIAG_REASON_FEEDBACK_LOST,
        CALIB_DIAG_REASON_TASK_START_FAILED,
        CALIB_DIAG_REASON_CONTROL_LOST,
        CALIB_DIAG_REASON_HOLD_TARGET_FAILED,
        CALIB_DIAG_REASON_JOINT_LIST_INVALID,
        CALIB_DIAG_REASON_DUPLICATE_JOINT,
        CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
        CALIB_DIAG_REASON_JOINT_STEP_FAILED,
        CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT,
        CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS,
        CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
        CALIB_DIAG_REASON_RESULT_SPAN_INVALID,
        CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
        CALIB_DIAG_REASON_RESULT_COMMIT_FAILED,
        CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED,
        CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED,
    )

    for sequence, reason in enumerate(reasons, start=1):
        state = SimpleNamespace(
            calib_diag_valid=True,
            calib_diag_reason=reason,
            calib_diag_phase=CALIB_DIAG_PHASE_DUAL_EXCHANGE,
            calib_diag_joint=12,
            calib_diag_motor=16,
            calib_diag_value_a=1,
            calib_diag_value_b=2,
            calib_diag_value_c=3,
            calib_diag_bitmap=(1 << 16) | (1 << 17),
            calib_diag_timestamp_ms=1000 + sequence,
            calib_diag_sequence=sequence,
            calib_diag_completed_joint_mask=0,
            calib_diag_failed_joint_mask=(1 << 12),
        )

        summary = HandGUI._calib_diag_summary(state)

        assert summary["active"] is True
        assert not summary["title"].startswith("Unknown calibration reason:")
        assert summary["suggestions"], f"reason={reason} has no recovery guidance"


def test_joint_step_fallback_uses_result_masks_not_payload_values():
    state = SimpleNamespace(
        calib_diag_valid=True,
        calib_diag_reason=CALIB_DIAG_REASON_JOINT_STEP_FAILED,
        calib_diag_phase=CALIB_DIAG_PHASE_DUAL_EXCHANGE,
        calib_diag_joint=14,
        calib_diag_motor=18,
        calib_diag_value_a=111,
        calib_diag_value_b=222,
        calib_diag_value_c=333,
        calib_diag_bitmap=0xFFFFFFFF,
        calib_diag_timestamp_ms=123,
        calib_diag_sequence=4,
        calib_diag_completed_joint_mask=(1 << 14),
        calib_diag_failed_joint_mask=(1 << 15),
    )

    summary = HandGUI._calib_diag_summary(state)

    assert any("completed=0x00004000" in line for line in summary["lines"])
    assert any("no specific mechanical cause" in line for line in summary["lines"])
    assert all("111" not in line and "222" not in line for line in summary["lines"])


def test_dual_exchange_stall_has_dedicated_payload_names():
    state = SimpleNamespace(
        calib_diag_valid=True,
        calib_diag_reason=CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED,
        calib_diag_phase=CALIB_DIAG_PHASE_DUAL_EXCHANGE,
        calib_diag_joint=12,
        calib_diag_motor=16,
        calib_diag_value_a=180,
        calib_diag_value_b=250,
        calib_diag_value_c=37,
        calib_diag_bitmap=(1 << 16) | (1 << 17),
        calib_diag_timestamp_ms=456,
        calib_diag_sequence=5,
        calib_diag_completed_joint_mask=0,
        calib_diag_failed_joint_mask=(1 << 12),
    )

    summary = HandGUI._calib_diag_summary(state)

    assert summary["title"] == "Joint stalled during dual-tendon exchange"
    assert any("load=180" in line and "encoder_delta=37" in line for line in summary["lines"])
