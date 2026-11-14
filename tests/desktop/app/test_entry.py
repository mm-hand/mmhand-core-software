from unittest.mock import Mock
import pytest
from desktop.app.entry import build_parser, main


def test_default_entry_enables_optional_ros():
    assert not build_parser().parse_args([]).no_ros2
    assert build_parser().parse_args(['--no-ros2']).no_ros2


@pytest.mark.parametrize('arguments', [['ui'], ['cli'], ['--auto-start'], ['--ros-topic', '/raw_ik_target']])
def test_retired_modes_are_not_formal_entry(arguments):
    with pytest.raises(SystemExit):
        build_parser().parse_args(arguments)


def test_main_launches_one_local_ui(monkeypatch):
    from desktop.app import ui_runner
    runner = Mock(return_value=0)
    monkeypatch.setattr(ui_runner, 'run_ui', runner)
    assert main(['--no-ros2']) == 0
    runner.assert_called_once_with(ros2_enabled=False)


def test_run_script_is_the_entry(monkeypatch):
    import runpy
    import sys
    from pathlib import Path
    from desktop.app import entry

    calls = []
    monkeypatch.setattr(entry, 'main', lambda argv=None: calls.append(argv) or 0)
    monkeypatch.setattr(sys, 'argv', ['run.py'])
    with pytest.raises(SystemExit) as info:
        runpy.run_path(str(Path(__file__).resolve().parents[3] / 'run.py'), run_name='__main__')
    assert info.value.code == 0 and calls == [None]
