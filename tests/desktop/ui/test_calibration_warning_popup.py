from types import SimpleNamespace

from desktop.core.protocol import (
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_AUTO_SPAN_RANGE,
    ENCODER_COUNT,
)
from desktop.ui.ui_control_panels import ControlPanel


def _state(mask: int):
    results = [None] * ENCODER_COUNT
    results[12] = {
        "hasAssemblyWarningFields": True,
        "assemblyWarningReason": (
            CALIB_WARN_AUTO_MIN_OFFSET |
            CALIB_WARN_AUTO_MAX_SHORTFALL |
            CALIB_WARN_AUTO_SPAN_RANGE
        ),
        "autoMinOffsetCounts": 412,
        "autoMaxShortfallCounts": 326,
        "assemblyAutoSpanCounts": 2184,
    }
    return SimpleNamespace(
        has_calibration_warning_status=True,
        calibration_warning_bitmap=mask,
        calibration_auto_min_offset_counts=[0] * ENCODER_COUNT,
        joint_calib_results=results,
        calib_status="WARN",
    )


def test_calibration_warning_popup_explains_all_reasons_once():
    actions = []
    warnings = []
    panel = SimpleNamespace(
        _calibration_warning_popup_mask=0,
        context=SimpleNamespace(emit_action=actions.append),
        _warn=lambda title, text: warnings.append((title, text)),
    )
    mask = 1 << 12

    ControlPanel._maybe_show_calibration_assembly_warning(panel, _state(mask))
    ControlPanel._maybe_show_calibration_assembly_warning(panel, _state(mask))

    assert len(warnings) == 1
    title, body = warnings[0]
    assert title == "标定装配警告"
    assert "J12" in body
    assert "412 counts" in body
    assert "326 counts" in body
    assert "2184 counts" in body
    assert "2300～6300" in body
    assert len(actions) == 1


def test_calibration_warning_popup_episode_resets_after_warning_clears():
    warnings = []
    panel = SimpleNamespace(
        _calibration_warning_popup_mask=0,
        context=SimpleNamespace(emit_action=lambda _message: None),
        _warn=lambda title, text: warnings.append((title, text)),
    )
    mask = 1 << 12

    ControlPanel._maybe_show_calibration_assembly_warning(panel, _state(mask))
    ControlPanel._maybe_show_calibration_assembly_warning(panel, _state(0))
    ControlPanel._maybe_show_calibration_assembly_warning(panel, _state(mask))

    assert len(warnings) == 2
