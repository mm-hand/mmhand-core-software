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
