import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from desktop.ui import ui_control_tests
from desktop.ui.ui_control_common import TEST_DEFAULT_DURATION_S, QtWidgets
from desktop.ui.ui_control_test_panel import ManualTestPanel
from desktop.ui.ui_control_tests import ManualTestPanelMixin


def _panel(elapsed_s: float):
    events = []
    sent = []
    panel = SimpleNamespace(
        _test_running=True,
        _test_t0=100.0,
        _test_hold_s=2.0,
        _test_duration_s=20.0,
        _test_active_rows=[],
        _test_feedback_start_pending=True,
        _manual_submode=lambda: "test",
        backend=SimpleNamespace(is_started=lambda: True),
        _on_stop_test=lambda reason: events.append(reason),
        resolved_joint_targets=lambda: [0.0] * 21,
        context=SimpleNamespace(
            send_joint_targets=lambda *args, **kwargs: (
                sent.append((args, kwargs)) or SimpleNamespace(sent=True)
            )
        ),
    )
    return panel, events, sent, 100.0 + elapsed_s


def test_sine_test_panel_exposes_duration_control():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = SimpleNamespace(
        _on_test_joint_changed=lambda _row: None,
        _on_start_test=lambda: None,
        _on_stop_test=lambda: None,
    )

    group = ManualTestPanel.sine_test_box(panel)

    assert panel.test_duration_spin.value() == TEST_DEFAULT_DURATION_S
    assert panel.test_duration_spin.minimum() == 0.1
    assert panel.test_duration_spin.maximum() == 3600.0
    assert "after the initial hold" in panel.test_duration_spin.toolTip()
    group.deleteLater()
    assert app is not None


def test_sine_test_keeps_running_before_hold_plus_duration(monkeypatch):
    panel, events, sent, now = _panel(21.999)
    monkeypatch.setattr(ui_control_tests.time, "time", lambda: now)

    ManualTestPanelMixin._test_tick(panel)

    assert events == []
    assert len(sent) == 1
    assert sent[0][1]["request_feedback_start"] is True
    assert panel._test_feedback_start_pending is False


def test_sine_test_requests_feedback_start_only_on_first_tick(monkeypatch):
    panel, events, sent, now = _panel(3.0)
    monkeypatch.setattr(ui_control_tests.time, "time", lambda: now)

    ManualTestPanelMixin._test_tick(panel)
    ManualTestPanelMixin._test_tick(panel)

    assert events == []
    assert [call[1]["request_feedback_start"] for call in sent] == [True, False]


def test_sine_test_stops_at_hold_plus_duration(monkeypatch):
    panel, events, sent, now = _panel(22.0)
    monkeypatch.setattr(ui_control_tests.time, "time", lambda: now)

    ManualTestPanelMixin._test_tick(panel)

    assert events == ["completed after 20 s sine duration"]
    assert sent == []
