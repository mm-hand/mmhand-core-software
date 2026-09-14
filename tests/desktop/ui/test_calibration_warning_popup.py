from types import SimpleNamespace

from desktop.core.protocol import (
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
    ENCODER_COUNT,
)
from desktop.core.calibration_workflow import CalibrationWorkflowEvent
from desktop.core.calibration_diagnostics import format_calibration_warning
from desktop.ui.ui_control_calibration import CalibrationWorkflowController


class _WarningHarness(CalibrationWorkflowController):
    def __init__(self):
        """Capture warning popup and action side effects."""
        self._calibration_warning_popup_mask = 0
        self._calibration_batch_active = False
        self._calibration_batch_alerts = []
        self.actions = []
        self.warnings = []
        self.errors = []
        self.statuses = []
        self.context = SimpleNamespace(emit_action=self.actions.append)

    def _warn(self, title, text):
        """Capture warning popup content."""
        self.warnings.append((title, text))

    def _error(self, title, text):
        self.errors.append((title, text))

    def _set_status(self, text):
        self.statuses.append(text)


def _state(mask: int):
    results = [None] * ENCODER_COUNT
    results[12] = {
        "hasAssemblyWarningFields": True,
        "assemblyWarningReason": (
            CALIB_WARN_AUTO_MIN_OFFSET |
            CALIB_WARN_AUTO_MAX_SHORTFALL |
            CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH
        ),
        "autoMinOffsetCounts": 412,
        "autoMaxShortfallCounts": 326,
        "assemblyAutoSpanCounts": 2184,
        "assemblyManualSpanCounts": 3400,
        "assemblyUrdfSpanCounts": 2957,
        "assemblySpanDifferenceCounts": 443,
        "hasAssemblySpanComparisonFields": True,
    }
    return SimpleNamespace(
        has_calibration_warning_status=True,
        calibration_warning_bitmap=mask,
        calibration_auto_min_offset_counts=[0] * ENCODER_COUNT,
        joint_calib_results=results,
        calib_status="WARN",
    )


def test_calibration_warning_popup_explains_all_reasons_once():
    panel = _WarningHarness()
    mask = 1 << 12

    panel._maybe_show_calibration_assembly_warning(_state(mask))
    panel._maybe_show_calibration_assembly_warning(_state(mask))

    assert len(panel.warnings) == 1
    title, body = panel.warnings[0]
    assert title == "Calibration Assembly Warning"
    assert "J12" in body
    assert "412 counts" in body
    assert "326 counts" in body
    assert "3400 counts" in body
    assert "2957 counts" in body
    assert "443 counts" in body
    assert "limit: 360 counts" in body
    assert len(panel.actions) == 1


def test_calibration_warning_popup_episode_resets_after_warning_clears():
    panel = _WarningHarness()
    mask = 1 << 12

    panel._maybe_show_calibration_assembly_warning(_state(mask))
    panel._maybe_show_calibration_assembly_warning(_state(0))
    panel._maybe_show_calibration_assembly_warning(_state(mask))

    assert len(panel.warnings) == 2


def test_active_batch_defers_warning_until_single_final_popup():
    panel = _WarningHarness()
    panel._calibration_batch_active = True
    state = _state(1 << 12)
    summary = format_calibration_warning(state, 1 << 12)
    record = {
        "phase_index": 0,
        "phase_count": 2,
        "status": "WARN",
        "warning_summary": summary,
    }

    panel._maybe_show_calibration_assembly_warning(state)
    panel._on_calibration_workflow_event(
        CalibrationWorkflowEvent(
            1,
            "phase_complete",
            True,
            "dual phase complete",
            details={**record, "has_next_phase": True},
        )
    )
    assert panel.warnings == []

    panel._on_calibration_workflow_event(
        CalibrationWorkflowEvent(
            1,
            "calibration_complete",
            True,
            "all calibration phases completed",
            details={"records": [record]},
        )
    )

    assert len(panel.warnings) == 1
    assert panel.warnings[0][0] == "Calibration Completed with Warnings"
    assert "Phase 1/2" in panel.warnings[0][1]
    assert "J12" in panel.warnings[0][1]


def test_final_batch_includes_rope_init_warning_reason():
    panel = _WarningHarness()
    panel._calibration_batch_active = True
    record = {
        "phase_index": 0,
        "phase_count": 1,
        "status": "WARN",
        "rope_init_reason": 1,
    }

    panel._on_calibration_workflow_event(
        CalibrationWorkflowEvent(
            2,
            "calibration_complete",
            True,
            "all calibration phases completed",
            details={"records": [record]},
        )
    )

    assert len(panel.warnings) == 1
    assert "Automatic Rope Init warning" in panel.warnings[0][1]
