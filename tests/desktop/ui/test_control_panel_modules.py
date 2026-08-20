from desktop.core.data_models import HandModel
from desktop.core.protocol import ENCODER_COUNT, VALID_MOTOR_MASK, manual_encoder_reference_entry_crc
from desktop.ui.ui_control_device import DeviceControlBar
from desktop.ui.ui_control_scope import ControlScopeModel, FINGER_JOINT_RANGES
from desktop.ui.ui_control_target_model import JointTargetModel
from desktop.ui.ui_app import HandGUI
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
    model = JointTargetModel(mapping_version=4, urdf_crc=0x12345678)
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
        "hasManualScaledV4Fields": True,
        "mappingVersion": 4,
        "urdfConfigCrc": 0x12345678,
        "manualReferenceCrc": manual_encoder_reference_entry_crc(0, min_raw, max_raw),
        "currentMinDeg": model.default_min(0),
        "currentMaxDeg": model.default_max(0),
        "angleMin": model.default_min(0),
        "angleMax": model.default_max(0),
    }

    assert model.calibrated_min(0, state) == model.default_min(0)
    assert model.target_min(0, state) == model.default_min(0)
    assert model.clamp(0, float("nan"), state) == model.default_min(0)
    assert model.clamp(0, model.default_max(0) + 100.0, state) == model.default_max(0)


def test_device_control_bar_exposes_expected_buttons() -> None:
    _app = ensure_app()
    bar = DeviceControlBar()
    events = []
    bar.start_requested.connect(lambda: events.append("start"))
    bar.stop_requested.connect(lambda: events.append("stop"))
    bar.pause_requested.connect(lambda: events.append("pause"))
    bar.ros2_output_toggled.connect(lambda enabled: events.append(("ros2", enabled)))
    bar.clear_fault_requested.connect(lambda: events.append("clear"))
    bar.calibrate_requested.connect(lambda: events.append("calibrate"))
    bar.rope_init_requested.connect(lambda: events.append("rope"))
    bar.nvs_toggle_requested.connect(lambda: events.append("nvs"))

    assert bar.enable_btn.text() == "ENABLE"
    assert bar.stop_btn.text() == "E-STOP"
    assert bar.ros2_output_btn.isCheckable()
    assert bar.nvs_control_btn.text() == "Flash ON"
    bar.enable_btn.click()
    bar.stop_btn.click()
    bar.pause_btn.click()
    bar.ros2_output_btn.click()
    bar.reset_btn.click()
    bar.calib_btn.click()
    bar.rope_init_btn.click()
    bar.nvs_control_btn.click()
    assert events == [
        "start",
        "stop",
        "pause",
        ("ros2", True),
        "clear",
        "calibrate",
        "rope",
        "nvs",
    ]


def test_joint_table_displays_ros2_targets_next_to_current() -> None:
    """Joint Channels 应在 Current 右侧展示实际送往 P4 的 ROS2 目标角。"""
    from desktop.ui.ui_control_panels import ControlPanel

    class _Backend:
        def ros2_output_enabled(self):
            return False

        def get_current_state(self):
            return HandModel()

        def set_control_mode(self, _mode):
            return None

    class _Context:
        backend = _Backend()
        ros2_enabled = True

        @staticmethod
        def is_comm_connected():
            return False

        @staticmethod
        def set_status(_text):
            return None

        @staticmethod
        def emit_action(_text):
            return None

        @staticmethod
        def connect_selected_port():
            return False

        @staticmethod
        def get_state():
            return HandModel()

        @staticmethod
        def cancel_replay(_reason):
            return None

    panel = ControlPanel(_Context())
    assert panel.joint_table.columnCount() == 13
    assert panel.joint_table.horizontalHeaderItem(4).text() == "Current"
    assert panel.joint_table.horizontalHeaderItem(5).text() == "ROS2 Target"

    inputs = [float(idx) for idx in range(ENCODER_COUNT)]
    outputs = list(inputs)
    outputs[0] = 5.5
    panel.update_ros2_joint_targets(
        {
            "source": "ROS2:/raw_ik_target",
            "sent": True,
            "input_targets": inputs,
            "output_targets": outputs,
            "command_joint_mask": 1,
            "clamped": [0],
            "message": "",
        }
    )

    assert panel.joint_ros2_target_items[0].text() == "5.50"
    assert "clamped from 0.00 deg to 5.50 deg" in panel.joint_ros2_target_items[0].toolTip()
    assert panel.joint_ros2_target_items[1].text() == "1.00"
    assert "was not sent" in panel.joint_ros2_target_items[1].toolTip()

    panel.update_ros2_joint_targets(
        {
            "source": "ROS2:/raw_ik_target",
            "sent": False,
            "message": "ROS2 joint target dropped: not_armed",
        }
    )
    assert panel.joint_ros2_target_items[0].text() == "5.50"
    assert "not_armed" in panel.joint_ros2_target_items[0].toolTip()

    panel.clear_ros2_joint_targets()
    assert all(item.text() == "-" for item in panel.joint_ros2_target_items)


def test_ros2_target_reaches_ui_when_save_data_is_disabled() -> None:
    """实时目标显示不得再被 Save Data 开关提前截断。"""
    emitted = []

    class _Signal:
        @staticmethod
        def emit(payload):
            emitted.append(payload)

    class _StoppedLogger:
        @staticmethod
        def is_running():
            return False

    owner = type(
        "JointTargetOwner",
        (),
        {
            "ros2_joint_target_received": _Signal(),
            "_data_logger": _StoppedLogger(),
        },
    )()
    payload = {
        "source": "ROS2:/raw_ik_target",
        "sent": True,
        "output_targets": [1.0] * ENCODER_COUNT,
    }

    HandGUI._on_joint_target_command(owner, payload)

    assert emitted == [payload]
