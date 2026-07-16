from types import MethodType, SimpleNamespace

from desktop.core.protocol import (
    SYSTEM_STATE_READY,
    SYSTEM_STATE_ROPE_INIT_RUNNING,
)
from desktop.ui.ui_app import HandGUI
from desktop.ui.ui_control_panels import ControlPanel


class _ProgressHarness:
    def __init__(self):
        self._p4_rope_init_phase = "queued"
        self._p4_rope_init_joint_mask = 0x00000005
        self._p4_rope_init_requested_at = 0.0
        self._p4_rope_init_transition_base = 10
        self.status = ""
        self.actions = []
        self.context = SimpleNamespace(emit_action=self.actions.append)
        self.rope_init_btn = SimpleNamespace(setText=lambda _text: None)
        self._reset_p4_rope_init_progress = MethodType(
            ControlPanel._reset_p4_rope_init_progress, self
        )
        self._finish_p4_rope_init_progress = MethodType(
            ControlPanel._finish_p4_rope_init_progress, self
        )

    def _set_status(self, text):
        self.status = str(text)

    def _update_manual_action_buttons(self):
        pass


def test_rope_init_progress_follows_p4_state(monkeypatch):
    harness = _ProgressHarness()
    monkeypatch.setattr("desktop.ui.ui_control_panels.time.monotonic", lambda: 1.0)

    ControlPanel._update_p4_rope_init_progress(
        harness,
        SimpleNamespace(
            system_state=SYSTEM_STATE_ROPE_INIT_RUNNING,
            state_transition_count=11,
        ),
    )

    assert harness._p4_rope_init_phase == "running"
    assert "started on P4" in harness.actions[-1]
    assert "running on P4" in harness.status

    ControlPanel._update_p4_rope_init_progress(
        harness,
        SimpleNamespace(
            system_state=SYSTEM_STATE_READY,
            state_transition_count=12,
        ),
    )

    assert harness._p4_rope_init_phase == "idle"
    assert "complete on P4" in harness.actions[-1]
    assert "READY confirmed" in harness.actions[-1]


def test_manual_events_are_not_misclassified_as_calibration():
    assert HandGUI._event_type_from_message(
        "Manual saved and verified; automatic calibration is required"
    ) == "manual_reference_write"
    assert HandGUI._event_type_from_message(
        "Manual endpoints cleared; Flash=empty"
    ) == "manual_reference_clear"
    assert HandGUI._event_type_from_message(
        "Manual endpoints read back"
    ) == "manual_reference_refresh"
