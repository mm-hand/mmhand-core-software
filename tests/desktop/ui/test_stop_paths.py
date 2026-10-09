import threading
import time
from types import MethodType, SimpleNamespace

import pytest

from desktop.ui.ui_app import HandGUI
from desktop.core.data_models import HandModel
from desktop.core.protocol import (
    CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
    ControlMode,
    SYSTEM_FAULT_MULTITURN_UNKNOWN,
    SYSTEM_FAULT_SERVO_OFFLINE,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_ROPE_INIT_RUNNING,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_UNCONTROLLABLE,
)
from desktop.ui.ui_control_commands import ControlCommandController
from desktop.ui.ui_control_state import ControlPanelStateMixin


class _FakeSignal:
    def __init__(self):
        self.args = None

    def emit(self, *args):
        self.args = args


class _FakeCheckBox:
    def __init__(self):
        self.checked = True

    def blockSignals(self, _blocked):
        pass

    def setChecked(self, checked):
        self.checked = bool(checked)


def test_startup_fault_does_not_latch_estop_before_motion_starts():
    app = SimpleNamespace(_fault_previous_runtime_active=False)
    summary = {"active": True, "offline_only": False}

    assert not HandGUI._fault_stop_allowed(
        app, summary, SimpleNamespace(system_state=SYSTEM_STATE_UNCONTROLLABLE)
    )
    assert HandGUI._fault_stop_allowed(
        app, summary, SimpleNamespace(system_state=SYSTEM_STATE_RUN_DIRECT)
    )


def test_internal_p4_motion_fault_does_not_send_competing_host_estop():
    """标定和 Rope Init 由 P4 自己停机，上位机只展示诊断，不能抢断预检。"""
    app = SimpleNamespace(_fault_previous_runtime_active=False)
    summary = {"active": True, "offline_only": False}

    assert not HandGUI._fault_stop_allowed(
        app,
        summary,
        SimpleNamespace(system_state=SYSTEM_STATE_CALIBRATION_RUNNING),
    )
    assert not HandGUI._fault_stop_allowed(
        app,
        summary,
        SimpleNamespace(system_state=SYSTEM_STATE_ROPE_INIT_RUNNING),
    )


def test_expected_calibration_rebase_unknown_does_not_popup_or_stop():
    state = SimpleNamespace(
        system_state=SYSTEM_STATE_UNCONTROLLABLE,
        control_enabled=False,
        servo_target_owner=0,
    )
    calls = []
    app = SimpleNamespace(
        _fault_previous_runtime_active=True,
        backend=SimpleNamespace(
            is_expected_calibration_turn_unknown=lambda snapshot: calls.append(snapshot) or True,
        ),
    )
    summary = {"active": True, "offline_only": False, "only_turn_unknown": True}

    assert not HandGUI._fault_popup_allowed(app, summary, state)
    assert not HandGUI._fault_stop_allowed(app, summary, state)
    assert calls == [state, state]


def test_motion_unknown_is_not_suppressed_by_calibration_hook():
    state = SimpleNamespace(system_state=SYSTEM_STATE_RUN_DIRECT)
    app = SimpleNamespace(
        _fault_previous_runtime_active=False,
        backend=SimpleNamespace(
            is_expected_calibration_turn_unknown=lambda snapshot:
                snapshot.system_state == SYSTEM_STATE_UNCONTROLLABLE,
        ),
    )
    summary = {"active": True, "offline_only": False, "only_turn_unknown": True}

    assert HandGUI._fault_popup_allowed(app, summary, state)
    assert HandGUI._fault_stop_allowed(app, summary, state)


def test_other_faults_keep_popup_and_stop_during_calibration_rebase():
    calls = []
    state = SimpleNamespace(system_state=SYSTEM_STATE_UNCONTROLLABLE)
    app = SimpleNamespace(
        _fault_previous_runtime_active=True,
        backend=SimpleNamespace(
            is_expected_calibration_turn_unknown=lambda snapshot: calls.append(snapshot) or True,
        ),
    )
    summary = {"active": True, "offline_only": False, "only_turn_unknown": False}

    assert HandGUI._fault_popup_allowed(app, summary, state)
    assert HandGUI._fault_stop_allowed(app, summary, state)
    assert calls == []


def test_unknown_without_explicit_core_rebase_confirmation_keeps_fault_protection():
    def failed_predicate(_state):
        raise RuntimeError("backend unavailable")

    state = SimpleNamespace(system_state=SYSTEM_STATE_UNCONTROLLABLE)
    summary = {"active": True, "offline_only": False, "only_turn_unknown": True}
    apps = [
        SimpleNamespace(_fault_previous_runtime_active=True),
        SimpleNamespace(_fault_previous_runtime_active=True, backend=SimpleNamespace()),
        SimpleNamespace(
            _fault_previous_runtime_active=True,
            backend=SimpleNamespace(is_expected_calibration_turn_unknown=lambda _: False),
        ),
        SimpleNamespace(
            _fault_previous_runtime_active=True,
            backend=SimpleNamespace(is_expected_calibration_turn_unknown=failed_predicate),
        ),
    ]
    for app in apps:
        assert HandGUI._fault_popup_allowed(app, summary, state)
        assert HandGUI._fault_stop_allowed(app, summary, state)


def _fault_app(monkeypatch, predicate):
    events = []
    app = SimpleNamespace(
        _fault_previous_runtime_active=True,
        _fault_stop_sent=False,
        _fault_popup_episode_active=False,
        _fault_dialog_active=False,
        _calib_diag_wait_started_at=0.0,
        backend=SimpleNamespace(
            is_expected_calibration_turn_unknown=predicate,
            stop=lambda: events.append("stop"),
        ),
        control_panel=SimpleNamespace(
            stop_all_tests=lambda *_args, **_kwargs: events.append("stop_tests"),
            turn_unknown_popup_seen=lambda: False,
            mark_turn_unknown_popup_seen=lambda mask: events.append(("turn_popup", mask)),
        ),
        _set_status=lambda text: events.append(("status", text)),
        _latest_safety_event_text=lambda _state: "",
        _fault_bitmap_value=HandGUI._fault_bitmap_value,
        _overload_detail_items=HandGUI._overload_detail_items,
        _overload_mask_from_state=HandGUI._overload_mask_from_state,
        _calib_diag_summary=HandGUI._calib_diag_summary,
        _fault_body_text=HandGUI._fault_body_text,
    )
    for name in ("_build_fault_summary", "_fault_stop_allowed", "_fault_popup_allowed"):
        setattr(app, name, MethodType(getattr(HandGUI, name), app))
    monkeypatch.setattr("desktop.ui.ui_app.log_session", lambda *_args: None)
    monkeypatch.setattr(
        "desktop.ui.ui_app.show_scrollable_message",
        lambda *_args, **_kwargs: events.append("dialog"),
    )
    return app, events


def _unknown_after_prior_joint_failure():
    state = HandModel()
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    state.system_fault_bitmap = SYSTEM_FAULT_MULTITURN_UNKNOWN
    state.multi_turn_unknown_bitmap = 1 << 8
    state.calib_diag_valid = True
    state.calib_diag_reason = CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED
    state.calib_diag_joint = 5
    state.calib_diag_failed_joint_mask = 1 << 5
    return state


def test_prior_joint_diagnostic_does_not_interrupt_confirmed_rebase(monkeypatch):
    calls = []
    app, events = _fault_app(
        monkeypatch, lambda snapshot: calls.append(snapshot) or True
    )
    state = _unknown_after_prior_joint_failure()

    summary = app._build_fault_summary(state)
    assert summary["only_turn_unknown"]
    assert app._calib_diag_summary(state)["active"]
    HandGUI._handle_fault_side_effects(app, state)

    assert calls == [state]
    assert events == []
    assert not app._fault_stop_sent
    assert not app._fault_popup_episode_active
    assert state.calib_diag_reason == CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED


@pytest.mark.parametrize("extra_fault", ["servo_offline", "overload"])
def test_prior_joint_diagnostic_keeps_real_fault_protection(monkeypatch, extra_fault):
    calls = []
    app, events = _fault_app(
        monkeypatch, lambda snapshot: calls.append(snapshot) or True
    )
    state = _unknown_after_prior_joint_failure()
    if extra_fault == "servo_offline":
        state.system_fault_bitmap |= SYSTEM_FAULT_SERVO_OFFLINE
    else:
        state.servo_overload_fault[8] = True

    assert not app._build_fault_summary(state)["only_turn_unknown"]
    HandGUI._handle_fault_side_effects(app, state)

    assert calls == []
    assert "stop" in events
    assert "dialog" in events


@pytest.mark.parametrize("initial_confirmation", [True, False])
def test_fault_effects_use_one_core_confirmation_for_the_same_snapshot(
    monkeypatch, initial_confirmation
):
    calls = []

    def changing_confirmation(snapshot):
        calls.append(snapshot)
        return initial_confirmation if len(calls) == 1 else not initial_confirmation

    app, events = _fault_app(monkeypatch, changing_confirmation)
    state = _unknown_after_prior_joint_failure()
    HandGUI._handle_fault_side_effects(app, state)

    assert calls == [state]
    assert ("stop" in events) is not initial_confirmation
    assert ("dialog" in events) is not initial_confirmation


def test_estop_is_enqueued_before_test_and_recorder_cleanup():
    events = []
    panel = SimpleNamespace(
        _estop_in_progress=False,
        _estop_latched=False,
        _update_connection_lights=lambda: events.append("lights"),
        backend=SimpleNamespace(stop=lambda: events.append("backend.stop")),
        context=SimpleNamespace(
            cancel_replay=lambda _reason: events.append("cancel_replay"),
            emit_action=lambda _message: events.append("emit_action"),
        ),
        stop_all_tests=lambda *_args, **_kwargs: events.append("stop_all_tests"),
        _clear_run_states=lambda: events.append("clear_run_states"),
        clear_ros2_joint_targets=lambda: events.append("clear_ros2_targets"),
    )

    ControlCommandController.stop_controller(panel)

    assert events.index("backend.stop") < events.index("stop_all_tests")
    assert panel._estop_latched is True
    assert panel._estop_in_progress is False


def test_direct_result_reports_execution_only_after_p4_generation_applies():
    actions = []
    statuses = []
    panel = SimpleNamespace(
        _last_reported_direct_generation=None,
        context=SimpleNamespace(emit_action=actions.append),
        _set_status=statuses.append,
    )

    # 首帧只是重连基线，不能把历史结果当成本次命令重复提示。
    baseline = SimpleNamespace(
        has_motion_result_status=True,
        pending_direct_generation=7,
        applied_direct_generation=7,
        direct_reject_reason=0,
    )
    ControlPanelStateMixin._handle_direct_command_result(panel, baseline)
    assert actions == []

    # 已提交但尚未仲裁的短窗口不应提前显示“执行成功”。
    pending = SimpleNamespace(
        has_motion_result_status=True,
        pending_direct_generation=8,
        applied_direct_generation=7,
        direct_reject_reason=0,
    )
    ControlPanelStateMixin._handle_direct_command_result(panel, pending)
    assert actions == []

    applied = SimpleNamespace(
        has_motion_result_status=True,
        pending_direct_generation=8,
        applied_direct_generation=8,
        direct_reject_reason=0,
    )
    ControlPanelStateMixin._handle_direct_command_result(panel, applied)
    assert actions == ["Direct command executed by P4: generation=8"]
    assert statuses == []


def test_direct_result_surfaces_p4_rejection_reason():
    actions = []
    statuses = []
    panel = SimpleNamespace(
        _last_reported_direct_generation=8,
        context=SimpleNamespace(emit_action=actions.append),
        _set_status=statuses.append,
    )
    rejected = SimpleNamespace(
        has_motion_result_status=True,
        pending_direct_generation=9,
        applied_direct_generation=8,
        direct_reject_reason=1,
    )

    ControlPanelStateMixin._handle_direct_command_result(panel, rejected)

    assert len(actions) == 1
    assert "generation=9" in actions[0]
    # 断言稳定规则编号，不绑定可能随本地化更新的中英文说明。
    assert "DIR-3101" in actions[0]
    assert statuses == actions


def test_test_stop_returns_without_waiting_for_recording_finalize():
    stop_started = threading.Event()
    allow_stop_to_finish = threading.Event()

    class _SlowLogger:
        path = "session-test"

        def is_running(self):
            return True

        def stop(self):
            stop_started.set()
            allow_stop_to_finish.wait(timeout=2.0)

    app = SimpleNamespace(
        save_data_check=_FakeCheckBox(),
        _test_recording_stop_lock=threading.Lock(),
        _test_recording_stop_thread=None,
        _data_logger=_SlowLogger(),
        _record_event=lambda *_args, **_kwargs: None,
        _recording_status_note="recording",
        _recording_csv_path="session-test",
        _set_status=lambda _text: None,
        _stop_joint_debug_packet_log=lambda: None,
        _set_control_trace_recording_subscription=lambda _enabled: None,
        test_recording_stop_finished=_FakeSignal(),
    )
    app.finish_test_recording_stop = MethodType(HandGUI.finish_test_recording_stop, app)

    started_at = time.perf_counter()
    HandGUI.stop_recording_for_test(app, "feedforward stopped")
    elapsed = time.perf_counter() - started_at

    assert stop_started.wait(timeout=0.5)
    assert elapsed < 0.1
    assert app.save_data_check.checked is False

    allow_stop_to_finish.set()
    app._test_recording_stop_thread.join(timeout=1.0)
    assert app.test_recording_stop_finished.args == (
        "session-test",
        "feedforward stopped",
        "",
    )


def test_joint_enable_does_not_stage_a_direct_motor_command():
    events = []
    backend = SimpleNamespace(
        set_control_mode=lambda mode: events.append(("mode", mode)),
        set_motor_positions_absolute=lambda _targets: events.append("motor_abs"),
        reset=lambda: events.append("reset"),
        start=lambda: events.append("start"),
    )
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "joint",
        _active_scope_offline_motor_indices=lambda _state: [],
        _active_scope_unroutable_motor_indices=lambda _state: [],
        _active_scope_offline_encoder_indices=lambda _state: [],
        _active_scope_joint_calib_missing_indices=lambda: [],
        _apply_active_scope_for_current_mode=lambda: True,
        _state=SimpleNamespace(system_state=5),
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _show_turn_recovery_guide=lambda *_args: None,
        _sync_motor_targets_to_feedback=lambda: events.append("sync_motor") or True,
        _build_motor_abs_command_list=lambda: [0] * 27,
        _motor_last_sent_abs=[],
        _active_motor_mask_from_ui=lambda: 8,
        _motor_session_origin_abs=[None] * 27,
        _motor_abs_pos=[None] * 27,
        _motor_last_slider=[None] * 27,
        _motor_target_user_set=[False] * 27,
        _set_motor_target_widget=lambda *_args: None,
        _estop_latched=True,
        backend=backend,
        context=SimpleNamespace(
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
            emit_action=lambda text: events.append(("action", text)),
        ),
        _set_status=lambda text: events.append(("status", text)),
        _update_connection_lights=lambda: None,
    )

    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert ("mode", ControlMode.JOINT_ANGLE) in events
    assert events.index("reset") < events.index("start")
    assert "start" in events
    assert "motor_abs" not in events
    assert "sync_motor" not in events


def test_joint_enable_resets_uncontrollable_p4_after_desktop_restart():
    """UI 本地未记住 E-STOP 时，仍应依据 P4 真值完成 RESET 后再 START。"""
    events = []
    backend = SimpleNamespace(
        set_control_mode=lambda mode: events.append(("mode", mode)),
        reset=lambda: events.append("reset"),
        start=lambda: events.append("start"),
    )
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "joint",
        _active_scope_offline_motor_indices=lambda _state: [],
        _active_scope_unroutable_motor_indices=lambda _state: [],
        _active_scope_offline_encoder_indices=lambda _state: [],
        _apply_active_scope_for_current_mode=lambda: events.append("scope") or True,
        _state=SimpleNamespace(system_state=0),
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _show_turn_recovery_guide=lambda *_args: None,
        _estop_latched=False,
        backend=backend,
        context=SimpleNamespace(
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
            emit_action=lambda text: events.append(("action", text)),
        ),
        _set_status=lambda text: events.append(("status", text)),
        _update_connection_lights=lambda: None,
    )

    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert events.index("reset") < events.index("scope") < events.index("start")
    assert ("mode", ControlMode.JOINT_ANGLE) in events


def test_joint_enable_does_not_arm_ros2_output():
    events = []
    backend = SimpleNamespace(
        set_control_mode=lambda mode: events.append(("mode", mode)),
        reset=lambda: events.append("reset"),
        start=lambda: events.append("start"),
        set_ros2_output_armed=lambda armed, reason="": (
            events.append(("ros2_arm", bool(armed), reason)) or (True, "ROS2 armed")
        ),
    )
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "joint",
        _active_scope_offline_motor_indices=lambda _state: [],
        _active_scope_unroutable_motor_indices=lambda _state: [],
        _active_scope_offline_encoder_indices=lambda _state: [],
        _active_scope_joint_calib_missing_indices=lambda: [],
        _apply_active_scope_for_current_mode=lambda: True,
        _state=SimpleNamespace(system_state=5),
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _show_turn_recovery_guide=lambda *_args: None,
        _estop_latched=True,
        backend=backend,
        context=SimpleNamespace(
            ros2_enabled=True,
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
            emit_action=lambda text: events.append(("action", text)),
        ),
        _set_status=lambda text: events.append(("status", text)),
        _update_connection_lights=lambda: None,
    )

    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert not any(event[0] == "ros2_arm" for event in events if isinstance(event, tuple))
    assert events.index("reset") < events.index("start")
    assert "start" in events


class _ToggleButton:
    def __init__(self):
        self.checked = False
        self.text = ""

    def blockSignals(self, _blocked):
        pass

    def setChecked(self, checked):
        self.checked = bool(checked)

    def setText(self, text):
        self.text = str(text)


def test_ros2_output_button_is_the_only_ui_path_that_arms_external_targets():
    events = []
    state = {"enabled": False}

    def set_ros2_output(enabled):
        state["enabled"] = bool(enabled)
        events.append(("ros2_output", bool(enabled)))
        return True, "ROS2 output enabled" if enabled else "ROS2 output disabled"

    panel = SimpleNamespace(
        backend=SimpleNamespace(
            set_ros2_output_enabled=set_ros2_output,
            ros2_output_enabled=lambda: state["enabled"],
        ),
        ros2_output_btn=_ToggleButton(),
        context=SimpleNamespace(emit_action=lambda text: events.append(("action", text))),
        _set_status=lambda text: events.append(("status", text)),
    )

    ControlCommandController.toggle_ros2_output(panel, True)

    assert ("ros2_output", True) in events
    assert panel.ros2_output_btn.checked
    assert panel.ros2_output_btn.text == "ROS2 INPUT ON"


def test_motor_enable_stages_direct_hold_without_extra_start_event():
    events = []
    targets = [100 + index for index in range(27)]
    backend = SimpleNamespace(
        set_control_mode=lambda mode: events.append(("mode", mode)),
        set_motor_positions_absolute=lambda values: events.append(("motor_abs", list(values))),
        reset=lambda: events.append("reset"),
        start=lambda: events.append("start"),
    )
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "motor",
        _active_scope_offline_motor_indices=lambda _state: [],
        _active_scope_unroutable_motor_indices=lambda _state: [],
        _active_scope_offline_encoder_indices=lambda _state: [],
        _apply_active_scope_for_current_mode=lambda: True,
        _state=SimpleNamespace(system_state=5),
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _show_turn_recovery_guide=lambda *_args: None,
        _sync_motor_targets_to_feedback=lambda: True,
        _build_motor_abs_command_list=lambda: targets,
        _motor_last_sent_abs=[],
        _active_motor_mask_from_ui=lambda: 8,
        _motor_session_origin_abs=[None] * 27,
        _motor_abs_pos=[None] * 27,
        _motor_last_slider=[None] * 27,
        _motor_target_user_set=[False] * 27,
        _set_motor_target_widget=lambda *_args: None,
        _estop_latched=True,
        backend=backend,
        context=SimpleNamespace(
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
            emit_action=lambda text: events.append(("action", text)),
        ),
        _set_status=lambda text: events.append(("status", text)),
        _update_connection_lights=lambda: None,
    )

    def hold(mask):
        assert mask == 8
        events.append("hold")
        return True, "hold", targets
    backend.enable_direct_hold = hold
    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert "hold" in events
    assert panel._motor_session_origin_abs[3] == targets[3]
    assert panel._motor_abs_pos[3] == 0
    assert events.index("reset") < events.index("hold")
    assert "start" not in events


def test_motor_enable_checks_servo_online_but_not_encoder_online():
    events = []
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "motor",
        _state=SimpleNamespace(system_state=5),
        _active_scope_offline_motor_indices=lambda _state: [],
        _active_scope_unroutable_motor_indices=lambda _state: [],
        _active_scope_offline_encoder_indices=lambda _state: [3],
        _apply_active_scope_for_current_mode=lambda: True,
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _show_turn_recovery_guide=lambda *_args: None,
        _sync_motor_targets_to_feedback=lambda: True,
        _build_motor_abs_command_list=lambda: [0] * 27,
        _motor_last_sent_abs=[],
        _active_motor_mask_from_ui=lambda: 8,
        _motor_session_origin_abs=[None] * 27,
        _motor_abs_pos=[None] * 27,
        _motor_last_slider=[None] * 27,
        _motor_target_user_set=[False] * 27,
        _set_motor_target_widget=lambda *_args: None,
        _estop_latched=False,
        backend=SimpleNamespace(
            set_control_mode=lambda _mode: events.append("mode"),
            start=lambda: events.append("start"),
            set_motor_positions_absolute=lambda _targets: events.append("direct"),
            enable_direct_hold=lambda _mask: (events.append("direct") or True, "hold", [0] * 27),
        ),
        context=SimpleNamespace(
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
            emit_action=lambda _text: None,
        ),
        _set_status=lambda text: events.append(text),
        _update_connection_lights=lambda: None,
    )

    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert "direct" in events


def test_joint_enable_requires_corresponding_servo_online_before_start():
    events = []
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "joint",
        _state=SimpleNamespace(system_state=5),
        _active_scope_offline_motor_indices=lambda _state: [8],
        _active_scope_offline_encoder_indices=lambda _state: [],
        _format_motor_list=lambda indices: ",".join(f"M{idx:02d}" for idx in indices),
        _format_joint_list=lambda indices: ",".join(f"J{idx:02d}" for idx in indices),
        backend=SimpleNamespace(
            set_control_mode=lambda _mode: None,
            start=lambda: events.append("start"),
        ),
        context=SimpleNamespace(
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
            emit_action=lambda _text: None,
        ),
        _set_status=lambda text: events.append(text),
        _update_connection_lights=lambda: None,
    )

    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert "start" not in events
    assert any("servo offline" in str(event) for event in events)


def test_joint_enable_rejects_offline_encoder_before_start():
    events = []
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "joint",
        _state=SimpleNamespace(system_state=5),
        _active_scope_offline_motor_indices=lambda _state: [],
        _active_scope_unroutable_motor_indices=lambda _state: [],
        _active_scope_offline_encoder_indices=lambda _state: [3],
        _format_joint_list=lambda indices: ",".join(f"J{idx:02d}" for idx in indices),
        backend=SimpleNamespace(start=lambda: events.append("start")),
        context=SimpleNamespace(
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
        ),
        _set_status=lambda text: events.append(text),
        _update_connection_lights=lambda: None,
    )

    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert "start" not in events
    assert any("encoder offline" in str(event) for event in events)


def test_motor_enable_turn_unknown_guides_new_device_before_reset_or_start():
    events = []
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "motor",
        _state=SimpleNamespace(system_state=5),
        _active_scope_offline_motor_indices=lambda _state: [],
        _active_scope_unroutable_motor_indices=lambda _state: [],
        _active_scope_multiturn_unknown_mask=lambda: 1 << 4,
        _show_turn_recovery_guide=lambda mask, action: events.append(
            ("turn_guide", mask, action)
        ),
        _estop_latched=True,
        backend=SimpleNamespace(
            reset=lambda: events.append("reset"),
            start=lambda: events.append("start"),
        ),
        context=SimpleNamespace(
            is_comm_connected=lambda: True,
            connect_selected_port=lambda: True,
        ),
        _set_status=lambda text: events.append(text),
        _update_connection_lights=lambda: None,
    )

    panel._clear_motor_target_after_single_send = MethodType(
        ControlPanelStateMixin._clear_motor_target_after_single_send, panel)
    ControlCommandController.start_controller(panel)

    assert ("turn_guide", 1 << 4, "ENABLE") in events
    assert "reset" not in events
    assert "start" not in events
