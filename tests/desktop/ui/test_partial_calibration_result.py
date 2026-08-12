from types import SimpleNamespace

from desktop.core.protocol import (
    CALIB_DIAG_PHASE_SEQUENCE,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
)
from desktop.ui.ui_control_calibration import CalibrationWorkflowController


def test_partial_calibration_summary_returns_only_completed_joints():
    state = SimpleNamespace(
        calib_diag_valid=True,
        calib_diag_completed_joint_mask=(1 << 3) | (1 << 4),
        calib_diag_failed_joint_mask=(1 << 5),
    )

    mask = CalibrationWorkflowController._partial_calibration_completed_joint_mask(
        SimpleNamespace(), state
    )

    assert mask == (1 << 3) | (1 << 4)


def test_process_failure_diag_is_not_treated_as_partial_summary():
    state = SimpleNamespace(
        calib_diag_valid=True,
        calib_diag_reason=CALIB_DIAG_REASON_JOINT_STEP_FAILED,
        calib_diag_phase=CALIB_DIAG_PHASE_SEQUENCE,
        calib_diag_motor=3,
        calib_diag_value_a=9,
        calib_diag_value_b=0,
        calib_diag_bitmap=(1 << 3),
        calib_diag_completed_joint_mask=0,
        calib_diag_failed_joint_mask=0,
    )

    mask = CalibrationWorkflowController._partial_calibration_completed_joint_mask(
        SimpleNamespace(), state
    )

    assert mask == 0


def test_incremental_result_mask_only_contains_changed_successful_joints():
    old_j03 = {"success": True, "joint": 3, "zeroEncoder": 100}
    new_j03 = {"success": True, "joint": 3, "zeroEncoder": 120}
    unchanged_j04 = {"success": True, "joint": 4, "zeroEncoder": 200}
    results = [None] * 21
    results[3] = new_j03
    results[4] = unchanged_j04
    panel = SimpleNamespace(
        _pending_calib_joint_mask=(1 << 3) | (1 << 4) | (1 << 5),
        _pending_calib_result_baselines={
            3: CalibrationWorkflowController._joint_calib_result_signature(old_j03),
            4: CalibrationWorkflowController._joint_calib_result_signature(unchanged_j04),
            5: None,
        },
    )

    mask = CalibrationWorkflowController._incremental_calibration_completed_joint_mask(
        panel,
        SimpleNamespace(joint_calib_results=results),
    )

    assert mask == (1 << 3)


def test_incremental_result_mask_ignores_failed_or_unselected_results():
    results = [None] * 21
    results[2] = {"success": False, "joint": 2}
    results[6] = {"success": True, "joint": 6, "zeroEncoder": 600}
    panel = SimpleNamespace(
        _pending_calib_joint_mask=(1 << 2),
        _pending_calib_result_baselines={2: None},
    )

    mask = CalibrationWorkflowController._incremental_calibration_completed_joint_mask(
        panel,
        SimpleNamespace(joint_calib_results=results),
    )

    assert mask == 0
