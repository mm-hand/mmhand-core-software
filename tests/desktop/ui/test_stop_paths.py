import threading
import time
from types import MethodType, SimpleNamespace

from desktop.ui.ui_app import HandGUI
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
            cancel_algorithm=lambda _reason: events.append("cancel_algorithm"),
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
