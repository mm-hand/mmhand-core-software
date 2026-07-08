from desktop.app import ui_runner
from desktop.core.hand_backend import HandBackend


def test_teleop_runner_forwards_ros2_configuration(monkeypatch) -> None:
    calls = []

    def fake_run_teleop(backend, port, tactile_port, *, ros2_enabled, ros_topic):
        calls.append((backend, port, tactile_port, ros2_enabled, ros_topic))

    monkeypatch.setattr(ui_runner, "_run_teleop_gui", fake_run_teleop)
    backend = HandBackend()

    ui_runner.run(
        ui_runner.Mode.TELEOP,
        backend,
        port="PORT",
        tactile_port="TACTILE",
        ros2_enabled=True,
        ros_topic="/test/manus_targets",
    )

    assert calls == [(backend, "PORT", "TACTILE", True, "/test/manus_targets")]
