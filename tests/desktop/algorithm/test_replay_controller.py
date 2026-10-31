import time
from pathlib import Path
from types import SimpleNamespace

from desktop.core.angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from desktop.core.data_models import HandModel
from desktop.core.protocol import manual_encoder_reference_entry_crc
from desktop.replay.replay_controller import (
    REPLAY_TIMELINE_REBASE_THRESHOLD_SECONDS,
    ReplayController,
    replay_start_state_allowed,
)
from desktop.replay.replay_models import ReplayFrame, ReplayTrace
from desktop.core.joint_target_adapter import active_mapping_identity
from desktop.core.protocol import (
    ENCODER_COUNT,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_FEEDBACK,
)


def _trace(active_joint_mask=1, frames=None):
    return ReplayTrace(
        path=Path("teleop.csv"),
        active_joint_mask=active_joint_mask,
        frames=frames
        or [
            ReplayFrame(0.0, [10.0] * ENCODER_COUNT),
            ReplayFrame(0.03, [15.0] * ENCODER_COUNT),
            ReplayFrame(0.06, [16.0] * ENCODER_COUNT),
        ],
    )


def _ready_state(active_mask=1, calibrated_mask=None):
    calibrated_mask = active_mask if calibrated_mask is None else calibrated_mask
    mapping_version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    state = HandModel()
    state.system_state = SYSTEM_STATE_READY
    state.active_joint_mask = active_mask
    state.feedback_collision_hold_bitmap = 0
    state.manual_reference_committed = True
    state.manual_reference_valid_bitmap = calibrated_mask
    state.auto_calibration_valid_bitmap = calibrated_mask
    for joint in range(ENCODER_COUNT):
        if not (calibrated_mask & (1 << joint)):
            continue
        spec = get_spec(joint)
        state.manual_reference_min_raw[joint] = 1000 + joint
        state.manual_reference_max_raw[joint] = 5000 + joint
        state.joint_calib_results[joint] = {
            "success": True,
            "mappingVersion": mapping_version,
            "urdfConfigCrc": urdf_crc,
            "manualReferenceCrc": manual_encoder_reference_entry_crc(
                joint,
                state.manual_reference_min_raw[joint],
                state.manual_reference_max_raw[joint],
            ),
            "currentMinDeg": spec.urdf_range_deg[0],
            "currentMaxDeg": spec.urdf_range_deg[1],
            "angleMin": spec.urdf_range_deg[0],
            "angleMax": spec.urdf_range_deg[1],
            "angleK": (spec.urdf_range_deg[1] - spec.urdf_range_deg[0]) / 4000.0,
            "angleB": spec.urdf_range_deg[0],
            "manualSpanCounts": 4000,
            "hasManualScaledV4Fields": True,
        }
    return state


class _Backend:
    def __init__(
        self,
        system_state=SYSTEM_STATE_READY,
        active_mask=1,
        calibrated_mask=None,
    ):
        self.state = _ready_state(active_mask, calibrated_mask)
        self.state.system_state = system_state
        self.sent = []
        self.token = 7
        self.valid = True

    def get_state_snapshot(self):
        return self.state

    def begin_replay_motion(self):
        self.token += 1
        self.valid = True
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


def test_replay_start_state_allows_ready_and_run_feedback():
    assert replay_start_state_allowed(SYSTEM_STATE_READY)
    assert replay_start_state_allowed(SYSTEM_STATE_RUN_FEEDBACK)
    assert not replay_start_state_allowed(-1)
    assert not replay_start_state_allowed("bad")


def test_replay_requires_startable_system_state():
    backend = _Backend(system_state=-1)
    controller = ReplayController(backend)

    ok, message = controller.start(_trace())

    assert ok is False
    assert "READY or RUN_FEEDBACK" in message
    assert backend.sent == []


def test_replay_sends_complete_trace_and_finishes():
    backend = _Backend()
    controller = ReplayController(backend)

    ok, _ = controller.start(_trace())
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok is True
    sent_values = [row[0][0] for row in backend.sent]
    assert len(sent_values) == 7
    assert sent_values[0] == 10.0
    assert sent_values[-1] == 16.0
    assert sent_values == sorted(sent_values)
    assert backend.sent[-1][1]["source"].startswith("Replay:")
    assert backend.sent[-1][1]["command_joint_mask"] == 1
    assert controller.status().state == "completed"
    assert controller.status().current_targets[0] == 16.0


def test_replay_can_start_again_after_completed_run():
    backend = _Backend()
    controller = ReplayController(backend)

    ok, _ = controller.start(_trace())
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)
    first_count = len(backend.sent)

    assert ok is True
    assert controller.status().state == "completed"
    assert backend.state.system_state == SYSTEM_STATE_RUN_FEEDBACK

    ok, _ = controller.start(_trace())
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok is True
    assert controller.status().state == "completed"
    assert len(backend.sent) == first_count * 2


def test_collision_hold_keeps_replay_running_for_retreat_frames(monkeypatch):
    backend = _Backend()
    controller = ReplayController(backend)
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


def test_replay_sends_discrete_ui_command_once():
    backend = _Backend()
    controller = ReplayController(backend)
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


def test_replay_intersects_csv_scope_with_current_scope():
    backend = _Backend(active_mask=1, calibrated_mask=1)
    controller = ReplayController(backend)
    trace = _trace(active_joint_mask=0b11)

    ok, _ = controller.start(trace)
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok
    assert all(kwargs["command_joint_mask"] == 1 for _, kwargs in backend.sent)
    assert controller.status().effective_joint_mask == 1
    assert controller.status().skipped_joint_mask == 0b10


def test_replay_skips_uncalibrated_joint_and_clamps_current_workspace():
    backend = _Backend(active_mask=0b11, calibrated_mask=1)
    controller = ReplayController(backend)
    low = [-100.0] * ENCODER_COUNT
    high = [1000.0] * ENCODER_COUNT
    trace = _trace(
        active_joint_mask=0b11,
        frames=[
            ReplayFrame(0.0, low, command_joint_mask=0b11),
            ReplayFrame(0.01, high, command_joint_mask=0b11),
        ],
    )

    ok, _ = controller.start(trace)
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok
    assert controller.status().effective_joint_mask == 1
    assert controller.status().skipped_joint_mask == 0b10
    assert controller.status().clamped_joint_mask == 1
    assert controller.status().clamped_sample_count == 2
    assert backend.sent[0][0][0] == get_spec(0).urdf_range_deg[0]
    assert backend.sent[-1][0][0] == get_spec(0).urdf_range_deg[1]


def test_replay_rebases_timeline_instead_of_aborting(monkeypatch):
    backend = _Backend()
    controller = ReplayController(backend)
    monkeypatch.setattr(
        "desktop.replay.replay_controller.REPLAY_TIMELINE_REBASE_THRESHOLD_SECONDS",
        -1.0,
    )

    ok, _ = controller.start(_trace())
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok
    assert controller.status().state == "completed"
    assert controller.status().timeline_rebase_count > 0
    assert len(backend.sent) == 7
    assert REPLAY_TIMELINE_REBASE_THRESHOLD_SECONDS == 0.25


def test_matching_joint_order_allows_mapping_and_range_changes():
    backend = _Backend()
    controller = ReplayController(backend)
    mapping = active_mapping_identity()
    trace = _trace()
    trace.joint_order_crc = mapping.joint_order_crc
    trace.mapping_id = "renamed-model"
    trace.mapping_version = mapping.mapping_version + 1
    trace.urdf_crc = mapping.urdf_config_crc ^ 1

    ok, _ = controller.start(trace)
    deadline = time.monotonic() + 1.0
    while controller.is_running() and time.monotonic() < deadline:
        time.sleep(0.01)

    assert ok
    assert controller.status().state == "completed"
    warnings = "; ".join(controller.status().warnings)
    assert "mapping_id differs" in warnings
    assert "mapping_version differs" in warnings
    assert "URDF range CRC differs" in warnings


def test_mismatched_joint_order_is_rejected():
    backend = _Backend()
    controller = ReplayController(backend)
    mapping = active_mapping_identity()
    trace = _trace()
    trace.joint_order_crc = mapping.joint_order_crc ^ 1

    ok, message = controller.start(trace)

    assert not ok
    assert "joint order" in message
    assert backend.sent == []
