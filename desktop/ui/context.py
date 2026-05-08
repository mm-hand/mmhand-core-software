from typing import Optional

from data_models import HandModel


class HandGuiContext:
    """Narrow component-facing facade around HandGUI."""

    def __init__(self, app):
        self._app = app

    def __getattr__(self, name):
        return getattr(self._app, name)

    @property
    def root(self):
        return self._app.root

    @property
    def controller(self):
        return self._app.controller

    @property
    def current_mode(self) -> str:
        return self._app.current_mode

    def is_comm_connected(self) -> bool:
        return self._app._is_comm_connected()

    def request_redraw(self) -> None:
        self._app._schedule_redraw()

    def emit_action(self, msg: str) -> None:
        self._app._emit_action(msg)

    def get_state(self) -> Optional[HandModel]:
        with self._app._state_lock:
            return self._app._state

    def set_state(self, state: Optional[HandModel]) -> None:
        with self._app._state_lock:
            self._app._state = state

    def list_available_cameras(self):
        return self._app._list_available_cameras()

    def select_camera(self, index, slot: int) -> None:
        self._app._on_camera_selected(index, slot)

    def stop_camera_capture(self) -> None:
        self._app._stop_camera_capture()
