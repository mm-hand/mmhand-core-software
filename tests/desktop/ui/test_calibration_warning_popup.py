from types import SimpleNamespace

from desktop.core.protocol import (
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
    ENCODER_COUNT,
)
from desktop.ui.ui_control_calibration import CalibrationWorkflowController


class _WarningHarness(CalibrationWorkflowController):
    def __init__(self):
        """Capture warning popup and action side effects."""
        self._calibration_warning_popup_mask = 0
        self.actions = []
        self.warnings = []
        self.context = SimpleNamespace(emit_action=self.actions.append)

    def _warn(self, title, text):
        """Capture warning popup content."""
        self.warnings.append((title, text))


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

    panel.show_calibration_assembly_warning_if_needed(_state(mask))
    panel.show_calibration_assembly_warning_if_needed(_state(mask))

    assert len(panel.warnings) == 1
    title, body = panel.warnings[0]
    assert title == "标定装配警告"
    assert "J12" in body
    assert "412 counts" in body
    assert "326 counts" in body
    assert "3400 counts" in body
    assert "2957 counts" in body
    assert "443 counts" in body
    assert "超过 360 counts" in body
    assert len(panel.actions) == 1


def test_calibration_warning_popup_episode_resets_after_warning_clears():
    panel = _WarningHarness()
    mask = 1 << 12

    panel.show_calibration_assembly_warning_if_needed(_state(mask))
    panel.show_calibration_assembly_warning_if_needed(_state(0))
    panel.show_calibration_assembly_warning_if_needed(_state(mask))

    assert len(panel.warnings) == 2
