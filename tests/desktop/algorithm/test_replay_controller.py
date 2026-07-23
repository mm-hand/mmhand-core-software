import time
from pathlib import Path
from types import SimpleNamespace

from desktop.algorithm.replay_controller import ReplayController
from desktop.algorithm.replay_models import ReplayFrame, ReplayTrace
from desktop.core.protocol import (
    ENCODER_COUNT,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_FEEDBACK,
)


def _trace():
    return ReplayTrace(
        path=Path("teleop.csv"),
        active_joint_mask=1,
        frames=[
            ReplayFrame(0.0, [10.0] * ENCODER_COUNT),
            ReplayFrame(0.03, [15.0] * ENCODER_COUNT),
            ReplayFrame(0.06, [20.0] * ENCODER_COUNT),
        ],
    )


class _Backend:
    def __init__(self, system_state=SYSTEM_STATE_READY):
        self.state = SimpleNamespace(
            system_state=system_state,
            active_joint_mask=1,
            feedback_collision_hold_bitmap=0,
            angles=[10.0] * ENCODER_COUNT,
        )
        self.sent = []
        self.token = 7
        self.valid = True

    def get_state_snapshot(self):
        return self.state

    def begin_replay_motion(self):
        return True, self.token, "ok"

    def replay_motion_token_valid(self, token):
        return self.valid and token == self.token

    def cancel_replay_motion(self, _reason):
        self.valid = False
        return True

    def end_replay_motion(self, _token):
        self.valid = False

    def send_joint_targets(self, targets, **kwargs):
        self.sent.append((list(targets), kwargs))
        self.state.system_state = SYSTEM_STATE_RUN_FEEDBACK
        return SimpleNamespace(sent=True, message="")


def test_replay_requires_exact_ready(monkeypatch):
    backend = _Backend(system_state=SYSTEM_STATE_RUN_FEEDBACK)
    controller = ReplayController(backend)

    ok, message = controller.start(_trace())

    assert ok is False
    assert "READY" in message
    assert backend.sent == []


def test_replay_sends_complete_trace_and_finishes(monkeypatch):
    backend = _Backend()
    controller = ReplayController(backend)
    monkeypatch.setattr(controller, "_entry_reject_reason", lambda *_: "")

    ok, _ = controller.start(_trace())
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok is True
    sent_values = [row[0][0] for row in backend.sent]
    assert len(sent_values) == 7
    assert sent_values[0] == 10.0
    assert sent_values[-1] == 20.0
    assert sent_values == sorted(sent_values)
    assert backend.sent[-1][1]["source"].startswith("Replay:")
    assert backend.sent[-1][1]["command_joint_mask"] == 1
    assert controller.status().state == "completed"
    assert controller.status().current_targets[0] == 20.0


def test_collision_hold_keeps_replay_running_for_retreat_frames(monkeypatch):
    backend = _Backend()
    controller = ReplayController(backend)
    monkeypatch.setattr(controller, "_entry_reject_reason", lambda *_: "")
    original_send = backend.send_joint_targets

    def send_and_collide(targets, **kwargs):
        result = original_send(targets, **kwargs)
        backend.state.feedback_collision_hold_bitmap = 1
        return result

    backend.send_joint_targets = send_and_collide
    ok, _ = controller.start(_trace())
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok is True
    assert controller.status().state == "completed"
    assert len(backend.sent) == 7


def test_replay_sends_discrete_ui_command_once(monkeypatch):
    backend = _Backend()
    controller = ReplayController(backend)
    monkeypatch.setattr(controller, "_entry_reject_reason", lambda *_: "")
    trace = ReplayTrace(
        path=Path("single-ui-command.csv"),
        active_joint_mask=1,
        frames=[
            ReplayFrame(
                0.0,
                [12.0] * ENCODER_COUNT,
                frame_kind="discrete",
                source_kind="ui_feedback",
                stream_id="event-1",
                command_joint_mask=1,
            )
        ],
    )

    ok, _ = controller.start(trace)
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok is True
    assert len(backend.sent) == 1
    assert backend.sent[0][0][0] == 12.0
