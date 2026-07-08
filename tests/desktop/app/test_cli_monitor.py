from desktop.app.cli_monitor import run_cli_monitor


class FakeBackend:
    def __init__(self) -> None:
        self.calls = []

    def connect(self, port: str) -> bool:
        self.calls.append(("connect", port))
        return True

    def set_pid_control(self, enabled: bool) -> None:
        self.calls.append(("set_pid_control", enabled))

    def start(self) -> None:
        self.calls.append(("start",))

    def stop(self) -> None:
        self.calls.append(("stop",))

    def reset(self) -> None:
        self.calls.append(("reset",))

    def request_firmware_calibration(self) -> bool:
        self.calls.append(("calibrate",))
        return True

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
        ("set_pid_control", False),
        ("start",),
        ("stop",),
        ("reset",),
        ("calibrate",),
        ("shutdown",),
    ]
