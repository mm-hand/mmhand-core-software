from desktop.app.cli_monitor import run_cli_monitor


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
