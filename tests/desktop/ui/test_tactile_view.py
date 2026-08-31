"""触觉页面的发布版界面契约测试。"""

import os
import re
import time

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from desktop.core.data_models import (
    FingerTactile,
    HandModel,
    TACTILE_DATA_LEVEL_FULL,
    TACTILE_DATA_LEVEL_SUMMARY,
    TACTILE_SOURCE_P4_CAN_RELAY,
    TACTILE_SOURCE_S3_SERIAL,
    TactileSensor,
)
from desktop.ui.ui_qt_compat import QtWidgets, ensure_app
from desktop.ui.ui_tactile_view import TactileView


_APP = ensure_app()


class _Context:
    """提供触觉页面需要的最小串口上下文，不实际连接硬件。"""

    current_port = ""

    def __init__(
        self,
        state: HandModel,
        *,
        tactile_connected: bool = False,
        tactile_port: str = "",
    ):
        self._state = state
        self._tactile_connected = bool(tactile_connected)
        self._tactile_port = str(tactile_port)

    def get_state(self) -> HandModel:
        return self._state

    def is_comm_connected(self) -> bool:
        return False

    def is_tactile_serial_connected(self) -> bool:
        return self._tactile_connected

    def get_tactile_serial_port(self) -> str:
        return self._tactile_port

    def set_status(self, _text: str) -> None:
        return None


def _tactile_state(*, full: bool = False) -> HandModel:
    fingers = []
    for finger_index in range(5):
        sensors = []
        for sensor_index in range(3):
            contact = np.zeros((5, 5, 3), dtype=float)
            sensors.append(TactileSensor(
                contact_forces=contact,
                resultant=np.array([finger_index + 1, sensor_index + 2, 10.0]),
            ))
        fingers.append(FingerTactile(sensors=sensors))
    state = HandModel(tactile_data=fingers)
    state.tactile_online = True
    state.tactile_last_rx_ts = time.time()
    state.tactile_module_valid_mask = 0x7FFF
    state.tactile_module_online = [True] * 15
    state.tactile_source = TACTILE_SOURCE_S3_SERIAL if full else TACTILE_SOURCE_P4_CAN_RELAY
    state.tactile_data_level = TACTILE_DATA_LEVEL_FULL if full else TACTILE_DATA_LEVEL_SUMMARY
    if full:
        state.tactile_full_seq = 7
    return state


def test_tactile_view_removes_module_and_3d_controls_and_shows_summary_vectors():
    state = _tactile_state()
    view = TactileView(_Context(state))
    try:
        view.update_if_due(state, force=True)

        assert not hasattr(view, "view3d_check")
        assert not hasattr(view, "module_status_labels")
        assert "SUMMARY" in view.force_panel.source_label.text()
        assert "15/15 VALID" in view.force_panel.source_label.text()
        assert "Fx    1" in view.force_panel._vector_labels[0][0].text()
        assert "Fz   10" in view.force_panel._vector_labels[0][0].text()
    finally:
        view.close()


def test_tactile_view_marks_full_taxel_data_without_changing_resultant_vectors():
    state = _tactile_state(full=True)
    view = TactileView(_Context(state))
    try:
        view.update_if_due(state, force=True)

        assert "FULL TAXEL" in view.force_panel.source_label.text()
        assert "Fx    5" in view.force_panel._vector_labels[4][2].text()
        assert "Fy    4" in view.force_panel._vector_labels[4][2].text()
    finally:
        view.close()


def test_tactile_view_marks_zero_force_valid_and_read_failure_separately():
    state = _tactile_state()
    state.tactile_module_valid_mask &= ~(1 << 1)
    state.tactile_module_online[1] = False
    for sensor in state.tactile_data[0].sensors:
        sensor.resultant[:] = 0.0

    view = TactileView(_Context(state))
    try:
        view.update_if_due(state, force=True)

        assert "14/15 VALID" in view.force_panel.source_label.text()
        assert "Fx    0" in view.force_panel._vector_labels[0][0].text()
        assert "READ FAIL" in view.force_panel._vector_labels[0][1].text()
    finally:
        view.close()


def test_tactile_view_contains_no_chinese_user_visible_text():
    state = _tactile_state()
    view = TactileView(_Context(state))
    try:
        view.update_if_due(state, force=True)
        visible_text = []
        for widget in view.findChildren(QtWidgets.QWidget):
            getter = getattr(widget, "text", None)
            if callable(getter):
                visible_text.append(str(getter()))

        assert not any(re.search(r"[\u4e00-\u9fff]", text) for text in visible_text)
    finally:
        view.close()


def test_tactile_view_distinguishes_open_serial_port_from_fresh_data():
    state = HandModel()
    context = _Context(
        state,
        tactile_connected=True,
        tactile_port="COM45",
    )
    view = TactileView(context)
    try:
        view._refresh_s3_status()

        assert view.s3_status_label.text() == "S3 serial open: COM45"
        assert "#475569" in view.s3_status_label.styleSheet()
    finally:
        view.close()
