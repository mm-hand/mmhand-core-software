import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from desktop.core.data_models import HandModel, JointWorkspaceRange
from desktop.core.protocol import (
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_UNCONTROLLABLE,
)
from desktop.ui.ui_control_panels import ControlPanel
from desktop.ui.ui_qt_compat import QtWidgets, ensure_app


_APP = ensure_app()


class _Backend:
    def raw_ik_output_enabled(self):
        """Return the raw IK arm state for ControlPanel construction."""
        return False

    def get_current_state(self):
        """Return an empty state for ControlPanel construction."""
        return HandModel()

    def set_pid_control(self, _enabled):
        """Accept PID policy updates during ControlPanel construction."""
        return None

    def set_control_mode(self, _mode):
        """Accept control-mode updates during mode-switch tests."""
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


def _ready_workspace(joint: int, current_min: float, current_max: float) -> JointWorkspaceRange:
    """Build a ready workspace range for pose target tests."""
    return JointWorkspaceRange(
        joint_index=joint,
        urdf_min_deg=-90.0,
        urdf_max_deg=90.0,
        current_min_deg=current_min,
        current_max_deg=current_max,
        manual_valid=True,
        auto_valid=True,
        manual_crc_match=True,
        mapping_valid=True,
        ready=True,
    )


def test_manual_workflow_buttons_keep_signals_after_module_split():
    """模块拆分后，工作流按钮仍应调用 ControlPanel 上的最新方法。"""
    panel = _panel()
    calls = []
    actions = (
        ("new_device_reset_btn", "new_device_reset", "new-device"),
        ("calib_load_btn", "load_calibration_snapshot", "load"),
        ("calib_import_btn", "import_calibration_snapshot", "import"),
        ("calib_export_btn", "export_calibration_snapshot", "export"),
        ("open_btn", "apply_open_pose", "open"),
        ("release_half_turn_btn", "release_all_motors_half_turn", "release"),
        ("tighten_half_turn_btn", "tighten_all_motors_half_turn", "tighten"),
        ("fist_btn", "apply_fist_pose", "fist"),
    )
    for button_name, method_name, marker in actions:
        setattr(panel, method_name, lambda value=marker: calls.append(value))
        button = getattr(panel, button_name)
        button.setEnabled(True)
        button.click()

    assert calls == [item[2] for item in actions]


def test_manual_workflow_signal_survives_mode_body_rebuild():
    """切换页面并重建 Manual body 后，按钮必须重新连接到当前控制器。"""
    panel = _panel()
    panel.sync_app_mode("teleop")
    panel.sync_app_mode("monitor")
    calls = []
    panel.new_device_reset = lambda: calls.append("new-device")
    panel.new_device_reset_btn.setEnabled(True)

    panel.new_device_reset_btn.click()

    assert calls == ["new-device"]


def test_new_device_reset_available_while_feedback_is_running():
    """Turn Unknown 恢复可从负反馈状态发起，后端会先执行安全维护复位。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    state = HandModel()
    state.system_state = SYSTEM_STATE_RUN_FEEDBACK
    state.multi_turn_unknown_bitmap = 1
    state.has_servo_raw_data = True
    panel._state = state

    panel._update_manual_action_buttons()

    assert panel.new_device_reset_btn.isEnabled()
    assert "安全停止" in panel.new_device_reset_btn.toolTip()


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


def test_new_device_reset_stays_disabled_when_turns_are_trusted_or_task_busy():
    """可信圈数和标定任务运行时仍保持硬门禁。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    state = HandModel()
    state.multi_turn_unknown_bitmap = 0
    panel._state = state
    panel._update_manual_action_buttons()
    assert not panel.new_device_reset_btn.isEnabled()
    assert "圈数可信" in panel.new_device_reset_btn.toolTip()

    state.multi_turn_unknown_bitmap = 1
    state.system_state = SYSTEM_STATE_CALIBRATION_RUNNING
    panel._update_manual_action_buttons()
    assert not panel.new_device_reset_btn.isEnabled()
    assert "正在运行" in panel.new_device_reset_btn.toolTip()


def test_new_device_reset_reads_fresh_backend_snapshot():
    """点击恢复前必须绕过旧 UI 帧并读取后端的最新 Turn Unknown 状态。"""
    panel = _panel()
    stale = HandModel()
    stale.multi_turn_unknown_bitmap = 0
    fresh = HandModel()
    fresh.multi_turn_unknown_bitmap = 1 << 24
    panel._state = stale
    panel.backend.get_state_snapshot = lambda: fresh

    assert panel._latest_new_device_reset_state() is fresh
    assert panel._state is fresh


def test_new_device_reset_reaches_backend_after_confirmation(monkeypatch):
    """完整走过关节掩码转换，防止拆分后的缺失辅助函数再次漏测。"""
    panel = _panel()
    panel.context.is_comm_connected = lambda: True
    state = HandModel()
    state.multi_turn_unknown_bitmap = 1 << 4
    state.has_servo_raw_data = True
    state.servo_raw_online[4] = True
    panel.backend.get_state_snapshot = lambda: state
    calls = []
    panel.backend.reset_multiturn_baseline_for_new_device = lambda: (
        calls.append("reset") or (False, "test stop", 1 << 4, 1 << 3)
    )
    panel._warn = lambda *_args: None
    panel._error = lambda *_args: None
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: QtWidgets.QMessageBox.Yes,
    )

    panel.new_device_reset()

    assert calls == ["reset"]


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
    panel._interaction_enabled = True
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
    panel._interaction_enabled = True
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
    panel._interaction_enabled = True
    panel.context.is_comm_connected = lambda: True
    panel._guard_joint_command_started = lambda: True
    panel._single_joint_command_targets = lambda _idx: [0.0] * 21
    panel._set_row_running = lambda *_args: None
    panel._set_status = lambda _text: None

    panel.send_joint_row(3)

    assert panel.context.calls[0][1]["command_joint_mask"] == (1 << 3)


def test_open_pose_uses_dual_midpoint_and_single_min_plus_five():
    panel = _panel()
    panel._state = HandModel()
    panel._active_scope_joint_indices = lambda: [0, 1]
    panel.resolved_joint_targets = lambda: [0.0] * 21
    workspaces = {
        0: _ready_workspace(0, 10.0, 50.0),
        1: _ready_workspace(1, 20.0, 80.0),
    }
    panel._joint_workspace_range = lambda joint, _state: workspaces[joint]

    pose = panel._build_open_pose_targets()

    assert pose is not None
    targets, joints = pose
    assert joints == [0, 1]
    assert targets[0] == 30.0
    assert targets[1] == 25.0


def test_current_min_pose_builder_keeps_min_plus_five_for_dual_joint():
    panel = _panel()
    panel._state = HandModel()
    panel._active_scope_joint_indices = lambda: [0]
    panel.resolved_joint_targets = lambda: [0.0] * 21
    workspace = _ready_workspace(0, 10.0, 50.0)
    panel._joint_workspace_range = lambda _joint, _state: workspace

    pose = panel._build_current_min_open_pose_targets()

    assert pose is not None
    targets, joints = pose
    assert joints == [0]
    assert targets[0] == 15.0
