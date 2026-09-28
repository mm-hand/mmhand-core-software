from desktop.app.cli_monitor import _print_console, run_cli_monitor
from desktop.core.data_models import HandModel


class FakeBackend:
    def __init__(self) -> None:
        self.calls = []

    def connect(self, port: str) -> bool:
        self.calls.append(("connect", port))
        return True

    def start(self) -> None:
        self.calls.append(("start",))

    def stop(self) -> None:
        self.calls.append(("stop",))

    def reset(self) -> None:
        self.calls.append(("reset",))

    def request_firmware_calibration(self) -> bool:
        self.calls.append(("calibrate",))
        return True

    def set_ros2_output_enabled(self, enabled: bool):
        self.calls.append(("ros2_output", bool(enabled)))
        return True, "ROS2 output enabled" if enabled else "ROS2 output disabled"

    def shutdown(self) -> None:
        self.calls.append(("shutdown",))


class _RenderBackend(FakeBackend):
    """为 CLI 渲染测试提供当前 HandModel 快照。"""

    def __init__(self) -> None:
        super().__init__()
        self.state = HandModel()
        self.state.host_rx_time = 1.0
        self.state.target_angles[0] = 12.5
        self.state.encoders[0].raw = 1024
        self.state.encoders[0].angle_error = False
        self.state.servo_online[26] = True
        self.state.servo_angles[26] = 4096

    def get_state_snapshot(self) -> HandModel:
        return self.state


class FakeKeyReader:
    def __init__(self, keys) -> None:
        self.keys = list(keys)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return None

    def read_key(self):
        if not self.keys:
            return "q"
        return self.keys.pop(0)


def test_cli_monitor_drives_backend_commands_only() -> None:
    backend = FakeBackend()

    code = run_cli_monitor(
        port="COM9",
        no_plot=True,
        backend=backend,
        key_reader_factory=lambda: FakeKeyReader(["s", "x", "r", "c", "q"]),
        render=False,
        sleep_s=0.0,
    )

    assert code == 0
    assert backend.calls == [
        ("connect", "COM9"),
        ("start",),
        ("stop",),
        ("reset",),
        ("calibrate",),
        ("shutdown",),
    ]


def test_cli_monitor_renders_current_hand_model_fields(monkeypatch, capsys) -> None:
    """CLI 不能再读取已删除的 timestamp、JOINT1_DEBUG 或 jd_* 字段。"""
    monkeypatch.setattr("desktop.app.cli_monitor.os.system", lambda _command: 0)

    _print_console(_RenderBackend(), "COM9", "DISABLED")

    output = capsys.readouterr().out
    assert "Mapped joint angles" in output
    assert "J00" in output
    assert "[26]" in output


class _FakeRosBridge:
    def start(self):
        pass

    def stop(self):
        pass


def test_cli_start_and_ros2_output_authorization_are_separate(monkeypatch) -> None:
    backend = FakeBackend()
    monkeypatch.setattr(
        "desktop.app.cli_monitor._make_ros2_bridge",
        lambda *_args, **_kwargs: _FakeRosBridge(),
    )

    code = run_cli_monitor(
        port="COM9",
        no_plot=True,
        ros2_enabled=True,
        backend=backend,
        key_reader_factory=lambda: FakeKeyReader(["s", "i", "o", "q"]),
        render=False,
        sleep_s=0.0,
    )

    assert code == 0
    assert backend.calls.count(("start",)) == 1
    assert ("ros2_output", True) in backend.calls
    assert ("ros2_output", False) in backend.calls
