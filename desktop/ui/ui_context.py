from typing import Optional

from desktop.core.data_models import HandModel


class HandGuiContext:
    def __init__(self, app):
        self._app = app

    @property
    def root(self):
        return self._app

    @property
    def backend(self):
        return self._app.backend

    @property
    def current_mode(self) -> str:
        return self._app.current_mode

    @property
    def ros2_enabled(self) -> bool:
        return bool(getattr(self._app, "ros2_enabled", False))

    @property
    def ros_topic(self) -> str:
        return str(getattr(self._app, "ros_topic", "/mmhand/joint_targets"))

    @property
    def control_panel(self):
        return self._app.control_panel

    @property
    def replay_service(self):
        return self._app.replay_service

    def is_comm_connected(self) -> bool:
        return self._app.is_comm_connected()

    def request_redraw(self) -> None:
        self._app.request_redraw()

    def emit_action(self, msg: str) -> None:
        self._app.emit_action(msg)

    def cancel_replay(self, reason: str = "stop requested") -> None:
        self._app.cancel_replay(reason)

    def set_status(self, text: str) -> None:
        self._app._set_status(text)

    def connect_selected_port(self) -> bool:
        return self._app.connect_selected_port()

    def disconnect_current_port(self, status: str = "Disconnected", stop_lower: bool = False) -> None:
        self._app._disconnect_current_port(status, stop_lower=stop_lower)

    def get_state(self) -> Optional[HandModel]:
        return self._app.get_state()

    def set_state(self, state: Optional[HandModel]) -> None:
        self._app.set_state(state)

    def send_joint_targets(
        self,
        angles,
        source: str = "",
        live: bool = False,
        input_space: str = "urdf_deg",
        raw_targets=None,
        command_joint_mask=None,
    ):
        return self._app.send_joint_targets(
            angles,
            source=source,
            live=live,
            input_space=input_space,
            raw_targets=raw_targets,
            command_joint_mask=command_joint_mask,
        )

    def start_recording_for_test(self, reason: str = "test") -> None:
        self._app.start_recording_for_test(reason)

    def stop_recording_for_test(self, reason: str = "test stopped") -> None:
        self._app.stop_recording_for_test(reason)

    def refresh_cameras(self) -> None:
        self._app.refresh_cameras()

    def draw_pose_angles(self, angles) -> None:
        self._app.draw_pose_angles(angles)

    def start_calibration(self) -> None:
        self._app.start_calibration()

    def connect_tactile_serial(self, port: str) -> bool:
        return self._app.connect_tactile_serial(port)

    def disconnect_tactile_serial(self) -> None:
        self._app.disconnect_tactile_serial()

    def is_tactile_serial_connected(self) -> bool:
        return self._app.is_tactile_serial_connected()

    def get_tactile_serial_port(self) -> str:
        return self._app.get_tactile_serial_port()
