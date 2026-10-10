import socket
from unittest.mock import Mock

import pytest

from desktop.app import ui_runner


def occupy_local_port():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    return server, server.getsockname()[1]


def test_resolve_ui_port_returns_preferred_when_free():
    server, port = occupy_local_port()
    server.close()
    assert ui_runner.resolve_ui_port(preferred=port) == port


def test_resolve_ui_port_skips_occupied_port():
    server, port = occupy_local_port()
    try:
        chosen = ui_runner.resolve_ui_port(preferred=port)
        assert chosen in range(port + 1, port + 1 + ui_runner.UI_PORT_SCAN_SPAN)
    finally:
        server.close()


def test_resolve_ui_port_raises_when_span_exhausted():
    server, port = occupy_local_port()
    try:
        with pytest.raises(RuntimeError, match=f'{port}-{port}'):
            ui_runner.resolve_ui_port(preferred=port, span=1)
    finally:
        server.close()


def test_run_ui_serves_next_free_port_when_default_occupied(monkeypatch, capsys):
    from nicegui import app, ui

    server, port = occupy_local_port()
    try:
        hand = Mock()
        monkeypatch.setattr(ui_runner, 'Ros2JointTargetBridge', Mock(return_value=Mock()))
        monkeypatch.setattr(ui_runner, 'ReplayService', Mock())
        monkeypatch.setattr(ui, 'run', Mock())
        monkeypatch.setattr(ui, 'page', lambda _: lambda f: f)
        monkeypatch.setattr(app, 'on_shutdown', Mock())
        monkeypatch.setattr(app, 'add_static_files', Mock())
        assert ui_runner.run_ui(
            backend=hand, ros2_enabled=False, show=False, port=port
        ) == 0
        served = ui.run.call_args.kwargs['port']
        assert served in range(port + 1, port + 1 + ui_runner.UI_PORT_SCAN_SPAN)
        assert f'port {port} is occupied' in capsys.readouterr().out
    finally:
        server.close()
