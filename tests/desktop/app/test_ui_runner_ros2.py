from unittest.mock import Mock
import pytest
from desktop.app import ui_runner


@pytest.mark.parametrize('fails', [None, 'joint', 'raw'])
def test_optional_ros_fallback_and_single_backend_lifecycle(monkeypatch, fails):
    from nicegui import app, ui
    hand = Mock()
    joint_bridge, raw_bridge = Mock(), Mock()
    if fails == 'joint':
        joint_bridge.start.side_effect = RuntimeError('missing rclpy')
    if fails == 'raw':
        raw_bridge.start.side_effect = RuntimeError('missing rclpy')
    joint_factory = Mock(return_value=joint_bridge)
    raw_factory = Mock(return_value=raw_bridge)
    monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', joint_factory)
    monkeypatch.setattr(ui_runner, 'RawIkTargetBridge', raw_factory)
    monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
    monkeypatch.setattr(ui, 'run', Mock())
    monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
    monkeypatch.setattr(app, 'on_shutdown', Mock())
    monkeypatch.setattr(app, 'add_static_files', Mock())
    assert ui_runner.run_ui(backend=hand, ros2_enabled=True, show=False) == 0
    hand.open_application.assert_called_once()
    hand.close_application.assert_called_once()
    hand.start.assert_not_called()
    hand.connect.assert_not_called()
    joint_factory.assert_called_once_with(hand)
    raw_factory.assert_called_once_with(hand)
    joint_bridge.stop.assert_called_once()
    raw_bridge.stop.assert_called_once()
    if fails is None:
        hand.set_ros_availability.assert_called_once_with(True)
    else:
        available, reason = hand.set_ros_availability.call_args.args
        assert available is False
        assert 'ROS2 unavailable: ' in reason
        assert 'missing rclpy' in reason
    assert ui.run.call_args.kwargs['host'] == '127.0.0.1'
    assert not ui.run.call_args.kwargs['reload']
    app.add_static_files.assert_not_called()


def test_disabled_ros2_never_creates_bridges(monkeypatch):
    from nicegui import app, ui
    hand = Mock()
    monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', Mock())
    monkeypatch.setattr(ui_runner, 'RawIkTargetBridge', Mock())
    monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
    monkeypatch.setattr(ui, 'run', Mock())
    monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
    monkeypatch.setattr(app, 'on_shutdown', Mock())
    monkeypatch.setattr(app, 'add_static_files', Mock())
    assert ui_runner.run_ui(backend=hand, ros2_enabled=False, show=False) == 0
    ui_runner.Ros2JointTargetBridge.assert_not_called()
    ui_runner.RawIkTargetBridge.assert_not_called()
    hand.set_ros_availability.assert_called_once_with(False, 'Disabled by --no-ros2')


def test_ui_failure_closes_backend_and_bridges(monkeypatch):
    from nicegui import app, ui
    hand = Mock()
    joint_bridge, raw_bridge = Mock(), Mock()
    monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', Mock(return_value=joint_bridge))
    monkeypatch.setattr(ui_runner, 'RawIkTargetBridge', Mock(return_value=raw_bridge))
    monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
    monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
    monkeypatch.setattr(ui, 'run', Mock(side_effect=RuntimeError('serve failed')))
    monkeypatch.setattr(app, 'on_shutdown', Mock())
    monkeypatch.setattr(app, 'add_static_files', Mock())
    with pytest.raises(RuntimeError, match='serve failed'):
        ui_runner.run_ui(backend=hand, show=False)
    joint_bridge.stop.assert_called_once()
    raw_bridge.stop.assert_called_once()
    hand.close_application.assert_called_once()


def test_bridge_stop_failure_still_closes_backend_and_other_bridge(monkeypatch):
    from nicegui import app, ui
    hand = Mock()
    joint_bridge, raw_bridge = Mock(), Mock()
    raw_bridge.stop.side_effect = RuntimeError('bridge stop failed')
    monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', Mock(return_value=joint_bridge))
    monkeypatch.setattr(ui_runner, 'RawIkTargetBridge', Mock(return_value=raw_bridge))
    monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
    monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
    monkeypatch.setattr(ui, 'run', Mock())
    monkeypatch.setattr(app, 'on_shutdown', Mock())
    with pytest.raises(RuntimeError, match='bridge stop failed'):
        ui_runner.run_ui(backend=hand, show=False)
    joint_bridge.stop.assert_called_once()
    hand.close_application.assert_called_once()
