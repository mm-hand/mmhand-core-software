import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from desktop.replay.replay_controller import ReplayStatus
from desktop.core.protocol import ENCODER_COUNT, SYSTEM_STATE_RUN_FEEDBACK
from desktop.ui.ui_qt_compat import ensure_app
from desktop.ui.ui_replay_view import ReplayView


_APP = ensure_app()


class _ReplayService:
    def __init__(self):
        self._trace = None

    def trace(self):
        return self._trace

    def status(self):
        return ReplayStatus(state="completed", message="Replay complete")

    def is_running(self):
        return False


class _ControlPanel:
    def joint_encoder_status(self, _joint, _state):
        return "ok"

    def joint_angle_feedback_valid(self, _joint, _state):
        return True

    def joint_angle_display_calibrated(self, _joint, _state):
        return True


class _Context:
    def __init__(self):
        self.replay_service = _ReplayService()
        self.control_panel = _ControlPanel()
        self.state = SimpleNamespace(
            system_state=SYSTEM_STATE_RUN_FEEDBACK,
            angles=[0.0] * ENCODER_COUNT,
            encoders=[],
        )

    def get_state(self):
        return self.state


def test_replay_send_button_can_restart_from_run_feedback_state():
    context = _Context()
    view = ReplayView(context)
    try:
        context.replay_service._trace = object()

        view._refresh_status()

        assert view.send_btn.isEnabled()
        assert view.ready_status.text() == "P4: RUN_FEEDBACK"
    finally:
        view.close()
