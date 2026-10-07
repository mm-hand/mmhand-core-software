import pytest

from desktop.app.entry import build_parser, main


def test_default_launch_opens_gui_without_authorizing_motion(monkeypatch):
    calls = []
    monkeypatch.setattr("desktop.app.ui_runner.run_ui", lambda **kw: calls.append(kw) or 0)
    assert main([]) == 0
    assert calls == [{"ros2_enabled": True}]


def test_no_ros2_launch(monkeypatch):
    calls = []
    monkeypatch.setattr("desktop.app.ui_runner.run_ui", lambda **kw: calls.append(kw) or 0)
    assert main(["--no-ros2"]) == 0
    assert calls == [{"ros2_enabled": False}]


@pytest.mark.parametrize("args", [
    ["ui"], ["cli"], ["--mode", "replay"], ["--port", "COM7"],
    ["--tactile-port", "COM8"], ["--ros2"], ["--ros2-input", "raw_ik"],
    ["--ros-topic", "/other"], ["--ros2-joint-target-topic", "/other"],
    ["--ros2-raw-ik-topic", "/other"], ["--ros2-raw-ik-space", "urdf_rad"],
    ["--auto-start"], ["--no-plot"],
])
def test_retired_arguments_are_explicitly_rejected(args):
    with pytest.raises(SystemExit) as exc:
        build_parser().parse_args(args)
    assert exc.value.code == 2
