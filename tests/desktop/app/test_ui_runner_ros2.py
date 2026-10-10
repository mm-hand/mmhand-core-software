from unittest.mock import Mock
import pytest
from desktop.app import ui_runner


@pytest.mark.parametrize('enabled,fails', [(True, False), (True, True), (False, False)])
def test_optional_ros_fallback_and_single_backend_lifecycle(monkeypatch, enabled, fails):
    from nicegui import app, ui
    hand, bridge = Mock(), Mock()
    if fails:
        bridge.start.side_effect = RuntimeError('missing rclpy')
    factory = Mock(return_value=bridge)
    monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', factory)
    monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
    monkeypatch.setattr(ui, 'run', Mock())
    monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
    monkeypatch.setattr(app, 'on_shutdown', Mock())
    monkeypatch.setattr(app, 'add_static_files', Mock())
    assert ui_runner.run_ui(backend=hand, ros2_enabled=enabled, show=False) == 0
    hand.open_application.assert_called_once()
    hand.close_application.assert_called_once()
    hand.start.assert_not_called()
    hand.connect.assert_not_called()
    if enabled:
        factory.assert_called_once_with(hand)
        bridge.stop.assert_called_once()
        assert hand.set_ros_availability.call_args.args[0] == (not fails)
    else:
        factory.assert_not_called()
        hand.set_ros_availability.assert_called_once_with(False, 'Disabled by --no-ros2')
    assert ui.run.call_args.kwargs['host'] == '127.0.0.1'
    assert not ui.run.call_args.kwargs['reload']
    app.add_static_files.assert_not_called()


def test_ui_failure_closes_backend_and_bridge(monkeypatch):
    from nicegui import app, ui
    hand, bridge = Mock(), Mock()
    monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', Mock(return_value=bridge))
    monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
    monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
    monkeypatch.setattr(ui, 'run', Mock(side_effect=RuntimeError('serve failed')))
    monkeypatch.setattr(app, 'on_shutdown', Mock())
    monkeypatch.setattr(app, 'add_static_files', Mock())
    with pytest.raises(RuntimeError, match='serve failed'):
        ui_runner.run_ui(backend=hand, show=False)
    bridge.stop.assert_called_once()
    hand.close_application.assert_called_once()


def test_bridge_stop_failure_still_closes_backend(monkeypatch):
    from nicegui import app, ui
    hand, bridge = Mock(), Mock()
    bridge.stop.side_effect = RuntimeError('bridge stop failed')
    monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', Mock(return_value=bridge))
    monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
    monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
    monkeypatch.setattr(ui, 'run', Mock())
    monkeypatch.setattr(app, 'on_shutdown', Mock())
    with pytest.raises(RuntimeError, match='bridge stop failed'):
        ui_runner.run_ui(backend=hand, show=False)
    hand.close_application.assert_called_once()
