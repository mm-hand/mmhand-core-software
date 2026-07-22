from types import MethodType, SimpleNamespace

from desktop.ui.ui_control_panels import ControlPanel


class _ValueWidget:
    def __init__(self):
        self.value = None

    def blockSignals(self, _blocked):
        return None

    def setValue(self, value):
        self.value = value


class _ReadableSpin:
    def __init__(self, value):
        self._value = float(value)

    def value(self):
        return self._value


def _panel():
    panel = SimpleNamespace(
        _updating=False,
        _interaction_enabled=False,
        _joint_target_user_set=[False],
        joint_sliders=[_ValueWidget()],
        joint_target_spins=[_ValueWidget()],
        _clamp_joint_target_deg=lambda _idx, value: float(value),
    )
    panel._mark_joint_target_user_set = MethodType(
        ControlPanel._mark_joint_target_user_set, panel
    )
    return panel


def test_spin_change_marks_target_before_state_refresh_can_resync_it():
    panel = _panel()

    ControlPanel._joint_spin_changed(panel, 0, 20.0)

    assert panel._joint_target_user_set[0] is True
    assert panel.joint_sliders[0].value == 2000


def test_slider_change_marks_target_before_state_refresh_can_resync_it():
    panel = _panel()

    ControlPanel._joint_slider_changed(panel, 0, 2000)

    assert panel._joint_target_user_set[0] is True
    assert panel.joint_target_spins[0].value == 20.0


def test_single_joint_command_uses_live_feedback_for_other_joints():
    """单行发送不得夹带其他输入框中残留的历史目标。"""
    joint_count = 21
    state = SimpleNamespace(
        angles=[30.0 + i for i in range(joint_count)],
        target_angles=[10.1] * joint_count,
    )
    panel = SimpleNamespace(
        _state=state,
        joint_target_spins=[_ReadableSpin(90.0 + i) for i in range(joint_count)],
        _joint_angle_display_calibrated=lambda _idx, _state: True,
        _joint_encoder_display_valid=lambda _idx, _state: True,
        _joint_angle_feedback_valid=lambda _idx, _state: True,
        _clamp_joint_target_deg=lambda _idx, value: float(value),
    )

    targets = ControlPanel._single_joint_command_targets(panel, 3)

    assert targets[3] == 93.0
    assert targets[2] == 32.0
    assert targets[2] != 92.0


def test_single_joint_command_falls_back_to_last_target_when_feedback_invalid():
    joint_count = 21
    state = SimpleNamespace(
        angles=[30.0] * joint_count,
        target_angles=[12.5] * joint_count,
    )
    panel = SimpleNamespace(
        _state=state,
        joint_target_spins=[_ReadableSpin(40.0) for _ in range(joint_count)],
        _joint_angle_display_calibrated=lambda _idx, _state: True,
        _joint_encoder_display_valid=lambda idx, _state: idx != 2,
        _joint_angle_feedback_valid=lambda _idx, _state: True,
        _clamp_joint_target_deg=lambda _idx, value: float(value),
    )

    targets = ControlPanel._single_joint_command_targets(panel, 3)

    assert targets[2] == 12.5
    assert targets[3] == 40.0


def test_send_joint_row_passes_only_clicked_joint_mask():
    class _Context:
        def __init__(self):
            self.calls = []

        def send_joint_targets(self, targets, **kwargs):
            self.calls.append((list(targets), kwargs))

    context = _Context()
    panel = SimpleNamespace(
        _interaction_enabled=True,
        context=context,
        _mark_joint_target_user_set=lambda _idx: None,
        _guard_joint_command_started=lambda: True,
        _single_joint_command_targets=lambda _idx: [0.0] * 21,
        _set_row_running=lambda *_args: None,
        _set_status=lambda _text: None,
    )

    ControlPanel.send_joint_row(panel, 3)

    assert context.calls[0][1]["command_joint_mask"] == (1 << 3)
