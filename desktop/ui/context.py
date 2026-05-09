from typing import Optional

from data_models import HandModel


class HandGuiContext:
    def __init__(self, app):
        self._app = app

    def __getattr__(self, name):
        return getattr(self._app, name)

    @property
    def root(self):
        return self._app

    @property
    def controller(self):
        return self._app.controller

    @property
    def current_mode(self) -> str:
        return self._app.current_mode

    def is_comm_connected(self) -> bool:
        return self._app.is_comm_connected()

    def request_redraw(self) -> None:
        self._app.request_redraw()

    def emit_action(self, msg: str) -> None:
        self._app.emit_action(msg)

    def set_status(self, text: str) -> None:
        self._app._set_status(text)

    def connect_selected_port(self) -> bool:
        return self._app.connect_selected_port()

    def get_state(self) -> Optional[HandModel]:
        return self._app.get_state()

    def set_state(self, state: Optional[HandModel]) -> None:
        self._app.set_state(state)

    def send_joint_targets(self, angles, source: str = "", live: bool = False) -> None:
        self._app.send_joint_targets(angles, source=source, live=live)

    def start_recording_for_test(self, reason: str = "test") -> None:
        self._app.start_recording_for_test(reason)

    def stop_recording_for_test(self, reason: str = "test stopped") -> None:
        self._app.stop_recording_for_test(reason)

    def refresh_cameras(self) -> None:
        self._app.refresh_cameras()
