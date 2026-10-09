import os
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from desktop.core.data_models import HandModel
from desktop.core.protocol import ENCODER_COUNT, SYSTEM_STATE_UNCONTROLLABLE
from desktop.ui.ui_control_panels import ControlPanel
from desktop.ui.ui_qt_compat import QtCore, QtWidgets, ensure_app


_APP = ensure_app()


class _Backend:
    def __init__(self):
        self.calibration_snapshot_path = Path(
            "C:/temporary/calibration_bundle.mmcal.json"
        )

    def ros2_output_enabled(self):
        """Return the ROS2 output arm state for ControlPanel construction."""
        return False

    def get_current_state(self):
        """Return an empty state for ControlPanel construction."""
        return HandModel()

    def set_control_mode(self, _mode):
        """Accept control-mode updates during mode-switch tests."""
        return None

    def register_calibration_workflow_callback(self, _callback):
        """Accept calibration workflow subscriptions during construction."""
        return None


class _Context:
    def __init__(self):
        """Capture joint target packets sent by the panel."""
        self.backend = _Backend()
        self.ros2_enabled = False
        self.calls = []

    def is_comm_connected(self):
        """Report serial connection state for the UI."""
        return False

    def set_status(self, _text):
        """Accept status updates."""
        return None

    def emit_action(self, _text):
        """Accept action log updates."""
        return None

    def connect_selected_port(self):
        """Do not open serial ports in these UI tests."""
        return False

    def get_state(self):
        """Return an empty state for view refreshes."""
        return HandModel()

    def cancel_replay(self, _reason):
        """Accept replay cancellation requests."""
        return None

    def send_joint_targets(self, targets, **kwargs):
        """Capture outgoing joint target packets."""
        self.calls.append((list(targets), kwargs))


def _panel() -> ControlPanel:
    """Construct a ControlPanel with a minimal desktop context."""
    return ControlPanel(_Context())


def test_manual_workflow_buttons_keep_signals_after_module_split():
    """模块拆分后，工作流按钮仍应调用 ControlPanel 上的最新方法。"""
    panel = _panel()
    calls = []
    actions = (
        ("calib_file_load_btn", "load_calibration_from_file", "load-file"),
        ("release_half_turn_btn", "release_all_motors_half_turn", "release"),
        ("tighten_half_turn_btn", "tighten_all_motors_half_turn", "tighten"),
    )
    for button_name, method_name, marker in actions:
        setattr(panel, method_name, lambda value=marker: calls.append(value))
        button = getattr(panel, button_name)
        button.setEnabled(True)
        button.click()

    assert calls == [item[2] for item in actions]
    assert not hasattr(panel, "manual_check")
    assert not hasattr(panel, "calib_load_btn")
    assert not hasattr(panel, "calib_import_btn")
    assert not hasattr(panel, "calib_export_btn")


def test_pose_buttons_exist_only_in_test_panel_and_keep_signals():
    """预设姿态应归属 Test 页面，不能重新出现在 Manual Workflow。"""
    panel = _panel()
    calls = []
    panel.apply_open_pose = lambda: calls.append("open")
    panel.apply_fist_pose = lambda: calls.append("fist")

    test_buttons = panel.test_panel.findChildren(QtWidgets.QPushButton)
    workflow = next(
        group
        for group in panel.findChildren(QtWidgets.QGroupBox)
        if group.title() == "Manual Workflow"
    )
    workflow_buttons = workflow.findChildren(QtWidgets.QPushButton)
    assert panel.open_btn in test_buttons
    assert panel.fist_btn in test_buttons
    assert panel.open_btn not in workflow_buttons
    assert panel.fist_btn not in workflow_buttons
    panel.open_btn.setEnabled(True)
    panel.fist_btn.setEnabled(True)
    panel.open_btn.click()
    panel.fist_btn.click()

    assert calls == ["open", "fist"]


def test_apply_preset_only_updates_controls_without_opening_hand_pose():
    """动作按钮不能隐式创建 Hand Pose 窗口，否则会在发送前阻塞 GUI 线程。"""
    panel = _panel()
    assert not hasattr(panel.context, "draw_pose_angles")

    panel.apply_preset([12.0] + [0.0] * 20, joint_indices=[0])

    assert panel.joint_target_spins[0].value() == 12.0


def test_calibration_load_signal_survives_mode_body_rebuild():
    """切换模式后重建 Manual body，标定加载入口仍必须可用。"""
    panel = _panel()
    panel.sync_app_mode("monitor")
    panel.sync_app_mode("monitor")
    calls = []
    panel.load_calibration_from_file = lambda: calls.append("load-file")
    panel.calib_file_load_btn.setEnabled(True)

    panel.calib_file_load_btn.click()

    assert calls == ["load-file"]


def test_body_clear_drops_motor_route_items_before_old_table_destruction():
    panel = _panel()
    assert any(item is not None for item in panel.motor_bus_items)
    assert any(item is not None for item in panel.motor_route_items)

    panel._clear_body()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)

    assert panel.motor_bus_items == []
    assert panel.motor_route_items == []
    panel._refresh_motor_route_item(0, HandModel())


def test_load_calibration_opens_snapshot_directory_then_restores_selected_file(monkeypatch):
    """UI 只选文件并确认，验证、激活和下发交给核心事务。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    panel._state = HandModel()
    panel._manual_submode = lambda: "device_check"
    selected_path = Path("C:/temporary/external_hand.mmcal.json")
    snapshot = {"hand_profile": "v2"}
    calls = []
    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args: (
            calls.append(_args[2]) or str(selected_path),
            "MMHand Calibration (*.mmcal.json)",
        ),
    )
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: QtWidgets.QMessageBox.Yes,
    )
    panel.backend.inspect_calibration_snapshot = lambda path: (
        calls.append(("inspect", path)) or snapshot
    )
    panel.backend.begin_calibration_import = lambda payload, source: (
        calls.append(("import", payload, source)) or True
    )

    panel.load_calibration_from_file()

    assert calls[0] == str(panel.backend.calibration_snapshot_path.parent)
    assert calls[1] == ("inspect", selected_path)
    assert calls[2] == ("import", snapshot, str(selected_path))


def test_load_calibration_cancel_does_not_activate_or_send(monkeypatch):
    """取消文件选择时不得覆盖本机快照或向 P4 发送恢复帧。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    panel._state = HandModel()
    calls = []
    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args: ("", ""),
    )
    panel.backend.inspect_calibration_snapshot = lambda _path: calls.append("inspect")
    panel.backend.begin_calibration_import = lambda *_args: calls.append("import")

    panel.load_calibration_from_file()

    assert calls == []


def test_startup_restore_is_delegated_to_core_workflow():
    """UI 启动检查只提交选区，不在 Qt 线程读写 JSON 或等待 P4。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    calls = []
    panel._active_joint_mask_from_ui = lambda: (1 << 3) | (1 << 4)
    panel.backend.begin_startup_calibration_restore = lambda mask: (
        calls.append(mask) or True
    )

    panel.begin_startup_device_check()

    assert calls == [(1 << 3) | (1 << 4)]
    assert panel._startup_device_check_running




def test_calibration_stays_available_without_auto_snapshot():
    """没有 Auto/LUT/ABS Limit 时禁止负反馈，但必须允许用户重新执行标定。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    panel.tabs.setCurrentIndex(1)
    state = HandModel()
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    state.motor_abs_limit_valid_bitmap = 0
    state.joint_calib_results = [None] * 21
    panel._state = state

    panel._update_manual_action_buttons()

    assert panel.calib_btn.isEnabled()
    assert not panel.enable_btn.isEnabled()


def test_calibrate_skips_offline_workspace_helpers_before_starting():
    """全部离线的工作空间辅助关节由 P4 跳过，不能扩大复位范围。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    panel._manual_submode = lambda: "joint"
    panel._active_scope_manual_missing_indices = lambda: []
    panel._state = SimpleNamespace(system_state=SYSTEM_STATE_UNCONTROLLABLE)
    panel._active_joint_mask_from_ui = lambda: 1 << 4
    panel._active_motor_mask_from_ui = lambda: (1 << 5) | (1 << 6)
    calls = []
    panel.backend.begin_calibration_workflow = (
        lambda joint_mask, motor_mask, reset_motor_mask: (
            calls.append((joint_mask, motor_mask, reset_motor_mask)) or True
        )
    )

    panel.calibrate()

    selected_motor_mask = (1 << 5) | (1 << 6)
    assert calls == [(1 << 4, selected_motor_mask, selected_motor_mask)]


def test_calibrate_resets_online_unknown_workspace_helpers_before_starting():
    """在线但圈数未知的辅助双电机必须与目标电机一起 Reset Zero。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    panel._manual_submode = lambda: "joint"
    panel._active_scope_manual_missing_indices = lambda: []
    helper_motor_mask = (1 << 0) | (1 << 1) | (1 << 5) | (1 << 6) | (1 << 10) | (1 << 11)
    selected_motor_mask = sum(1 << motor_idx for motor_idx in range(15, 20))
    panel._state = SimpleNamespace(
        system_state=SYSTEM_STATE_UNCONTROLLABLE,
        servo_online_bitmap=helper_motor_mask | selected_motor_mask,
        multi_turn_unknown_bitmap=helper_motor_mask | selected_motor_mask,
    )
    panel._active_joint_mask_from_ui = lambda: sum(1 << joint_idx for joint_idx in range(12, 16))
    panel._active_motor_mask_from_ui = lambda: selected_motor_mask
    calls = []
    panel.context.emit_action = lambda message: calls.append(("event", message))
    panel.backend.begin_calibration_workflow = (
        lambda joint_mask, motor_mask, reset_motor_mask: (
            calls.append((joint_mask, motor_mask, reset_motor_mask)) or True
        )
    )

    panel.calibrate()

    final_motor_mask = selected_motor_mask | helper_motor_mask
    active_joint_mask = sum(1 << joint_idx for joint_idx in range(12, 16))
    assert calls[-1] == (active_joint_mask, selected_motor_mask, final_motor_mask)


def test_calibrate_rejects_partial_online_workspace_helper_before_reset():
    """辅助双电机关节仅一台在线时，必须在任何硬件复位前拒绝。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    panel._manual_submode = lambda: "joint"
    panel._active_scope_manual_missing_indices = lambda: []
    panel._state = SimpleNamespace(
        system_state=SYSTEM_STATE_UNCONTROLLABLE,
        servo_online_bitmap=(1 << 0) | (1 << 15) | (1 << 16),
        multi_turn_unknown_bitmap=(1 << 0),
    )
    panel._active_joint_mask_from_ui = lambda: 1 << 12
    panel._active_motor_mask_from_ui = lambda: (1 << 15) | (1 << 16)
    calls = []
    panel.backend.prepare_calibration_turn_zero = lambda motor_mask: calls.append(motor_mask)
    panel._warn = lambda title, message: calls.append(("warn", title, message))

    panel.calibrate()

    assert not any(isinstance(item, int) for item in calls)
    assert calls and calls[0][0] == "warn"




def test_spin_change_marks_target_before_state_refresh_can_resync_it():
    panel = _panel()

    panel.joint_target_spins[0].setValue(20.0)

    assert panel.resolved_joint_targets()[0] == 20.0
    assert panel.joint_sliders[0].value() == 2000


def test_slider_change_marks_target_before_state_refresh_can_resync_it():
    panel = _panel()

    panel.joint_sliders[0].setValue(2000)

    assert panel.resolved_joint_targets()[0] == 20.0
    assert panel.joint_target_spins[0].value() == 20.0


def test_single_joint_command_uses_live_feedback_for_other_joints():
    panel = _panel()
    panel._state = SimpleNamespace(
        angles=[30.0 + i for i in range(21)],
        target_angles=[10.1] * 21,
    )
    panel._controller_interaction_ready = lambda: True
    panel.context.is_comm_connected = lambda: True
    panel._guard_joint_command_started = lambda: True
    panel._joint_angle_display_calibrated = lambda _idx, _state: True
    panel._joint_encoder_display_valid = lambda _idx, _state: True
    panel._joint_angle_feedback_valid = lambda _idx, _state: True
    panel._set_row_running = lambda *_args: None
    panel._set_status = lambda _text: None

    panel.joint_target_spins[3].setValue(90.0)
    panel.send_joint_row(3)
    targets, kwargs = panel.context.calls[-1]

    assert kwargs["command_joint_mask"] == (1 << 3)
    assert targets[3] == 90.0
    assert targets[2] == 32.0
    assert targets[2] != 92.0


def test_single_joint_command_falls_back_to_last_target_when_feedback_invalid():
    panel = _panel()
    panel._state = SimpleNamespace(
        angles=[30.0] * 21,
        target_angles=[12.5] * 21,
    )
    panel._controller_interaction_ready = lambda: True
    panel.context.is_comm_connected = lambda: True
    panel._guard_joint_command_started = lambda: True
    panel._joint_angle_display_calibrated = lambda _idx, _state: True
    panel._joint_encoder_display_valid = lambda idx, _state: idx != 2
    panel._joint_angle_feedback_valid = lambda _idx, _state: True
    panel._set_row_running = lambda *_args: None
    panel._set_status = lambda _text: None

    panel.joint_target_spins[3].setValue(40.0)
    panel.send_joint_row(3)
    targets, _kwargs = panel.context.calls[-1]

    assert targets[2] == 12.5
    assert targets[3] == 40.0


def test_send_joint_row_passes_only_clicked_joint_mask():
    panel = _panel()
    panel._controller_interaction_ready = lambda: True
    panel.context.is_comm_connected = lambda: True
    panel._guard_joint_command_started = lambda: True
    panel._single_joint_command_targets = lambda _idx: [0.0] * 21
    panel._set_row_running = lambda *_args: None
    panel._set_status = lambda _text: None

    panel.send_joint_row(3)

    assert panel.context.calls[0][1]["command_joint_mask"] == (1 << 3)


def test_open_pose_sends_zero_to_all_joint_channels():
    panel = _panel()
    sent = []
    applied = []
    panel.apply_preset = lambda targets, joint_indices=None: applied.append(
        (list(targets), list(joint_indices or []))
    )
    panel._guard_joint_command_started = lambda: True
    panel._set_status = lambda _text: None
    panel.context.send_joint_targets = lambda targets, **kwargs: (
        sent.append((list(targets), kwargs)) or SimpleNamespace(sent=True)
    )

    panel.apply_open_pose()

    assert applied == [([0.0] * ENCODER_COUNT, list(range(ENCODER_COUNT)))]
    assert sent == [([0.0] * ENCODER_COUNT, {"source": "Open Pose", "live": False})]
