from desktop.core.data_models import HandModel
from desktop.core.protocol import ENCODER_COUNT, VALID_MOTOR_MASK, manual_encoder_reference_entry_crc
from desktop.ui.ui_control_device import DeviceControlBar
from desktop.ui.ui_control_scope import ControlScopeModel, FINGER_JOINT_RANGES
from desktop.ui.ui_control_target_model import JointTargetModel
from desktop.ui.ui_qt_compat import ensure_app


def test_control_scope_model_orders_thumb_workflow_and_masks_motors() -> None:
    pinky_label, pinky_start, pinky_end = FINGER_JOINT_RANGES[0]
    thumb_label, thumb_start, thumb_end = FINGER_JOINT_RANGES[-1]

    pinky_joint_mask = ControlScopeModel.joint_group_mask(
        pinky_label,
        pinky_start,
        pinky_end,
    )
    pinky_motor_mask, scope_joint_mask, require_can = ControlScopeModel.scope_range(
        pinky_start,
        pinky_end,
        True,
    )
    assert scope_joint_mask == pinky_joint_mask
    assert pinky_motor_mask == ControlScopeModel.motor_mask_for_joint_mask(pinky_joint_mask)
    assert require_can is True

    assert ControlScopeModel.workflow_joint_indices_for_group(
        thumb_label,
        thumb_start,
        thumb_end,
    ) == [20, 16, 17, 18, 19]

    full_joint_mask = (1 << ENCODER_COUNT) - 1
    assert ControlScopeModel.motor_mask_for_joint_mask(full_joint_mask) == VALID_MOTOR_MASK


def test_joint_target_model_uses_calibrated_min_and_clamps_invalid_values() -> None:
    model = JointTargetModel(mapping_version=3, urdf_crc=0x12345678)
    state = HandModel()
    min_raw = 100
    max_raw = 2000
    state.manual_reference_nvs_valid = True
    state.manual_reference_valid_bitmap = 1
    state.auto_calibration_valid_bitmap = 1
    state.manual_reference_min_raw[0] = min_raw
    state.manual_reference_max_raw[0] = max_raw
    state.joint_calib_results[0] = {
        "success": True,
        "hasManualScaledV3Fields": True,
        "mappingVersion": 3,
        "urdfConfigCrc": 0x12345678,
        "manualReferenceCrc": manual_encoder_reference_entry_crc(0, min_raw, max_raw),
        "currentMinDeg": 5.0,
        "currentMaxDeg": model.default_max(0),
        "angleMin": 5.0,
        "angleMax": model.default_max(0),
    }

    assert model.calibrated_min(0, state) == 5.0
    assert model.target_min(0, state) == 5.0
    assert model.clamp(0, float("nan"), state) == 5.0
    assert model.clamp(0, model.default_max(0) + 100.0, state) == model.default_max(0)


def test_device_control_bar_exposes_expected_buttons() -> None:
    _app = ensure_app()
    bar = DeviceControlBar()
    events = []
    bar.start_requested.connect(lambda: events.append("start"))
    bar.stop_requested.connect(lambda: events.append("stop"))
    bar.pause_requested.connect(lambda: events.append("pause"))
    bar.raw_ik_toggled.connect(lambda enabled: events.append(("raw", enabled)))
    bar.clear_fault_requested.connect(lambda: events.append("clear"))
    bar.calibrate_requested.connect(lambda: events.append("calibrate"))
    bar.rope_init_requested.connect(lambda: events.append("rope"))
    bar.nvs_toggle_requested.connect(lambda: events.append("nvs"))

    assert bar.enable_btn.text() == "ENABLE"
    assert bar.stop_btn.text() == "E-STOP"
    assert bar.raw_ik_enable_btn.isCheckable()
    assert bar.nvs_control_btn.text() == "Flash ON"
    bar.enable_btn.click()
    bar.stop_btn.click()
    bar.pause_btn.click()
    bar.raw_ik_enable_btn.click()
    bar.reset_btn.click()
    bar.calib_btn.click()
    bar.rope_init_btn.click()
    bar.nvs_control_btn.click()
    assert events == [
        "start",
        "stop",
        "pause",
        ("raw", True),
        "clear",
        "calibrate",
        "rope",
        "nvs",
    ]
