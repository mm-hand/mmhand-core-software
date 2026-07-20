import time
from types import SimpleNamespace

from desktop.algorithm import example_algorithm
from desktop.core.protocol import ENCODER_COUNT, SYSTEM_STATE_READY


def _state(active_mask: int):
    return SimpleNamespace(
        active_joint_mask=active_mask,
        angles=[10.0] * ENCODER_COUNT,
        system_state=SYSTEM_STATE_READY,
    )


def _patch_ready_workspace(monkeypatch, base_target: float = 15.0):
    def fake_workspace(_state, joint, *_args):
        return SimpleNamespace(
            open_pose_target=lambda offset: base_target + float(joint) + float(offset)
        )

    monkeypatch.setattr(example_algorithm, "resolve_joint_workspace_range", fake_workspace)


def test_pose_targets_change_only_active_joints(monkeypatch):
    state = _state((1 << 0) | (1 << 12))
    _patch_ready_workspace(monkeypatch)

    open_targets = example_algorithm.build_open_pose_targets(state)
    fist_targets = example_algorithm.build_fist_pose_targets(state)

    assert open_targets[0] == 20.0
    assert open_targets[12] == 32.0
    assert open_targets[1] == 10.0
    assert fist_targets[0] == 48.0
    assert fist_targets[12] == 58.0
    assert fist_targets[1] == 10.0


def test_pose_cycles_stop_after_rejected_command(monkeypatch):
    state = _state(1 << 0)
    sent = []

    class Backend:
        def get_state_snapshot(self):
            return state

        def send_joint_targets(self, targets, **kwargs):
            sent.append((list(targets), kwargs))
            return SimpleNamespace(sent=False, blocked=[], clamped=[], message="blocked for test")

    monkeypatch.setattr(example_algorithm, "_state_ready_reason", lambda _state: "")
    _patch_ready_workspace(monkeypatch)
    completed = example_algorithm.run_pose_cycles(
        Backend(), cycle_count=2, hold_seconds=0.0, ready_poll_seconds=0.0
    )

    assert completed == 0
    assert len(sent) == 1


def test_pose_cycles_complete_open_and_fist(monkeypatch):
    state = _state(1 << 0)
    sent = []

    class Backend:
        def get_state_snapshot(self):
            return state

        def send_joint_targets(self, targets, **kwargs):
            sent.append((list(targets), kwargs))
            return SimpleNamespace(sent=True, blocked=[], clamped=[], message="")

    monkeypatch.setattr(example_algorithm, "_state_ready_reason", lambda _state: "")
    _patch_ready_workspace(monkeypatch)
    completed = example_algorithm.run_pose_cycles(
        Backend(), cycle_count=2, hold_seconds=0.0, ready_poll_seconds=0.0
    )

    assert completed == 2
    assert len(sent) == 4
    assert [entry[1]["source"].endswith(name) for entry, name in zip(sent, ["Open", "Fist", "Open", "Fist"])] == [True] * 4


def test_pose_cycle_controller_starts_only_after_enable(monkeypatch):
    class Backend:
        pass

    runs = []

    def fake_run(backend, stop_event=None):
        runs.append((backend, stop_event))

    monkeypatch.setattr(example_algorithm, "run_pose_cycles", fake_run)
    controller = example_algorithm.PoseCycleController(Backend())

    assert controller.is_running() is False
    assert runs == []
    assert controller.start() is True
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.shutdown()

    assert len(runs) == 1


def test_pose_cycle_controller_cancel_stops_ready_wait(monkeypatch):
    class Backend:
        def get_state_snapshot(self):
            return _state(1 << 0)

    monkeypatch.setattr(example_algorithm, "_state_ready_reason", lambda _state: "waiting")
    controller = example_algorithm.PoseCycleController(Backend())

    assert controller.start() is True
    assert controller.stop("test") is True
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)
    controller.shutdown()

    assert controller.is_running() is False
