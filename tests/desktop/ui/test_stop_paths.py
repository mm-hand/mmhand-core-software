import threading
import time
from types import MethodType, SimpleNamespace

from desktop.ui.ui_app import HandGUI
from desktop.core.protocol import ControlMode
from desktop.ui.ui_control_panels import ControlPanel


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
    )

    ControlPanel.stop_controller(panel)

    assert events.index("backend.stop") < events.index("stop_all_tests")
    assert panel._estop_latched is True
    assert panel._estop_in_progress is False


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
        test_recording_stop_finished=_FakeSignal(),
    )
    app._finish_test_recording_stop = MethodType(HandGUI._finish_test_recording_stop, app)

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
        set_pid_control=lambda enabled: events.append(("pid", enabled)),
        set_motor_positions_absolute=lambda _targets: events.append("motor_abs"),
        start=lambda: events.append("start"),
    )
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "joint",
        _active_scope_joint_calib_missing_indices=lambda: [],
        _apply_active_scope_for_current_mode=lambda: True,
        _state=SimpleNamespace(system_state=5),
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _show_turn_recovery_guide=lambda *_args: None,
        _sync_motor_targets_to_feedback=lambda: events.append("sync_motor") or True,
        _build_motor_abs_command_list=lambda: [0] * 27,
        _motor_last_sent_abs=[],
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

    ControlPanel.start_controller(panel)

    assert ("mode", ControlMode.JOINT_ANGLE) in events
    assert ("pid", True) in events
    assert "start" in events
    assert "motor_abs" not in events
    assert "sync_motor" not in events


def test_joint_enable_does_not_arm_raw_ik_output():
    events = []
    backend = SimpleNamespace(
        set_control_mode=lambda mode: events.append(("mode", mode)),
        set_pid_control=lambda enabled: events.append(("pid", enabled)),
        start=lambda: events.append("start"),
        set_ros2_output_armed=lambda armed, reason="": (
            events.append(("ros2_arm", bool(armed), reason)) or (True, "ROS2 armed")
        ),
    )
    panel = SimpleNamespace(
        _app_mode="teleop",
        _manual_submode=lambda: "joint",
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

    ControlPanel.start_controller(panel)

    assert not any(event[0] == "ros2_arm" for event in events if isinstance(event, tuple))
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


def test_raw_ik_button_is_the_only_ui_path_that_arms_teleoperation():
    events = []
    state = {"enabled": False}

    def set_raw_ik(enabled):
        state["enabled"] = bool(enabled)
        events.append(("raw_ik", bool(enabled)))
        return True, "RAW IK output enabled" if enabled else "RAW IK output disabled"

    panel = SimpleNamespace(
        backend=SimpleNamespace(
            set_raw_ik_output_enabled=set_raw_ik,
            raw_ik_output_enabled=lambda: state["enabled"],
        ),
        raw_ik_enable_btn=_ToggleButton(),
        context=SimpleNamespace(emit_action=lambda text: events.append(("action", text))),
        _set_status=lambda text: events.append(("status", text)),
    )

    ControlPanel.toggle_raw_ik_output(panel, True)

    assert ("raw_ik", True) in events
    assert panel.raw_ik_enable_btn.checked
    assert panel.raw_ik_enable_btn.text == "RAW IK ON"


def test_motor_enable_stages_direct_hold_without_extra_start_event():
    events = []
    targets = [100 + index for index in range(27)]
    backend = SimpleNamespace(
        set_control_mode=lambda mode: events.append(("mode", mode)),
        set_pid_control=lambda enabled: events.append(("pid", enabled)),
        set_motor_positions_absolute=lambda values: events.append(("motor_abs", list(values))),
        start=lambda: events.append("start"),
    )
    panel = SimpleNamespace(
        _app_mode="manual",
        _manual_submode=lambda: "motor",
        _apply_active_scope_for_current_mode=lambda: True,
        _state=SimpleNamespace(system_state=5),
        _active_scope_multiturn_unknown_mask=lambda: 0,
        _show_turn_recovery_guide=lambda *_args: None,
        _sync_motor_targets_to_feedback=lambda: True,
        _build_motor_abs_command_list=lambda: targets,
        _motor_last_sent_abs=[],
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

    ControlPanel.start_controller(panel)

    assert ("mode", ControlMode.DIRECT_MOTOR) in events
    assert ("pid", False) in events
    assert ("motor_abs", targets) in events
    assert "start" not in events
