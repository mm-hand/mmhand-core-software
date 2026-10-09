from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from desktop.app import ui_runner
from desktop.core.hand_backend import HandBackend


def test_replay_runner_forwards_ros2_configuration(monkeypatch) -> None:
    calls = []

    def fake_run_replay(backend, port, tactile_port, *, ros2_enabled, ros_topic):
        calls.append((backend, port, tactile_port, ros2_enabled, ros_topic))

    monkeypatch.setattr(ui_runner, "_run_replay_gui", fake_run_replay)
    backend = HandBackend()

    ui_runner.run(
        ui_runner.Mode.REPLAY,
        backend,
        port="PORT",
        tactile_port="TACTILE",
        ros2_enabled=True,
        ros_topic="/raw_ik_target",
    )

    assert calls == [(backend, "PORT", "TACTILE", True, "/raw_ik_target")]


@pytest.mark.parametrize("enabled,fails", [(True, False), (True, True), (False, False)])
@pytest.mark.parametrize("mode", ["monitor", "replay"])
def test_ros2_startup_and_manual_fallback(monkeypatch, enabled, fails, mode):
    backend = Mock()
    bridge = Mock()
    if fails:
        bridge.start.side_effect = RuntimeError("missing rclpy")
    factory, run, warning, log = Mock(return_value=bridge), Mock(), Mock(), Mock()
    monkeypatch.setattr(ui_runner, "_configure_gui_diagnostics", lambda: None)
    monkeypatch.setattr(ui_runner, "_make_ros2_bridge", factory)
    monkeypatch.setattr(ui_runner, "run", run)
    monkeypatch.setattr(ui_runner, "_schedule_ros2_warning", warning)
    monkeypatch.setattr(ui_runner, "log_session", log)

    assert ui_runner.run_ui(mode=mode, backend=backend, ros2_enabled=enabled) == 0
    assert run.call_args.kwargs["ros2_enabled"] == (enabled and not fails)
    backend.start.assert_not_called()
    backend.connect.assert_not_called()
    if enabled:
        assert factory.call_args.kwargs["auto_start"] is False
        bridge.start.assert_called_once()
    else:
        factory.assert_not_called()
    if fails:
        warning.assert_called_once_with("missing rclpy")
        log.assert_called_once_with("ROS2", "startup failed: missing rclpy")
    else:
        warning.assert_not_called()
    assert bridge.stop.call_count == int(enabled and not fails)


def test_ros2_warning_waits_for_gui_event_loop(monkeypatch):
    from desktop.ui import ui_qt_compat as qt

    pending, warnings = [], []
    app = object()
    monkeypatch.setattr(qt, "ensure_app", lambda: app)
    monkeypatch.setattr(qt, "QtCore", SimpleNamespace(
        QTimer=SimpleNamespace(singleShot=lambda delay, callback: pending.append(callback)),
    ))
    monkeypatch.setattr(qt, "QtWidgets", SimpleNamespace(
        QApplication=SimpleNamespace(activeWindow=lambda: "main-window"),
        QMessageBox=SimpleNamespace(warning=lambda *args: warnings.append(args)),
    ))
    assert ui_runner._schedule_ros2_warning("missing rclpy") is app
    assert warnings == []
    pending[0]()
    assert warnings[0][:2] == ("main-window", "ROS2 unavailable")
    assert "missing rclpy" in warnings[0][2]


def test_gui_failure_still_stops_ros2_bridge(monkeypatch):
    bridge = Mock()
    monkeypatch.setattr(ui_runner, "_configure_gui_diagnostics", lambda: None)
    monkeypatch.setattr(ui_runner, "_make_ros2_bridge", Mock(return_value=bridge))
    monkeypatch.setattr(ui_runner, "run", Mock(side_effect=RuntimeError("GUI failure")))
    with pytest.raises(RuntimeError, match="GUI failure"):
        ui_runner.run_ui(backend=Mock())
    bridge.stop.assert_called_once()


def test_gui_gc_is_installed_before_serial_workers_start(monkeypatch):
    from desktop.ui import ui_qt_compat as qt

    calls = []
    backend = Mock()
    app = object()
    monkeypatch.setattr(ui_runner, "_configure_gui_diagnostics",
                        lambda: calls.append("diagnostics"))
    monkeypatch.setattr(qt, "ensure_app", lambda: calls.append("gui_gc") or app)
    backend.connect.side_effect = lambda _port: calls.append("connect") or True
    monkeypatch.setattr(ui_runner, "run", lambda *_args, **_kwargs: calls.append("gui"))

    assert ui_runner.run_ui(port="PORT", backend=backend, ros2_enabled=False) == 0
    assert calls == ["diagnostics", "gui_gc", "connect", "gui"]
