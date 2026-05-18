import json
import math
import re
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

from data_models import HandModel
from protocol import (
    ENCODER_COUNT,
    HAND_PROFILE,
    MOTOR_COUNT,
    VALID_MOTOR_MASK,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_PAUSED,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_READY,
)
from .ui_qt_compat import QtCore, QtGui, QtWidgets
from .ui_tendon_guard import (
    MOTOR_TO_JOINT_INDEX,
    TendonGuard,
    clamp_motor_abs,
    motor_target_from_servo_feedback,
)
from .ui_teleop_view import TeleopView
from .ui_algorithm_view import AlgorithmView


MOTOR_TARGET_SCALE_LO = 0
MOTOR_TARGET_SCALE_HI = 4095
MOTOR_SINGLE_TURN_STEPS = 4096
MOTOR_JUMP_RESYNC_THRESH = 2048
MOTOR_SLIDER_WRAP_MARGIN = 512
MOTOR_SLIDER_MAX_TARGET_DELTA = MOTOR_SINGLE_TURN_STEPS
MOTOR_DIR_HINT_CLEAR_TOL = 3
MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES = 4
MOTOR_ABS_LIMIT_RECOVER_MARGIN = 16
ROPE_INIT_STEP_COUNTS = 50
MOTOR_MID_CALIB_PATH = Path(__file__).resolve().parents[1] / "config" / "motor_mid_calib.json"
TEST_ROW_COUNT = 5
TEST_TICK_MS = 10
TEST_DEFAULT_FREQ_HZ = 0.20
TEST_DEFAULT_PHASE_DEG = 0.0
TEST_DEFAULT_HOLD_S = 0.0
FF_TEST_DEFAULT_TARGETS_DEG = "20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40"
FF_TEST_DEFAULT_STEP_S = 1.0
TRI_TEST_DEFAULT_FREQ_HZ = 0.20
JOINT_TARGET_MIN_DEG_BY_INDEX = [0.0] * ENCODER_COUNT
JOINT_TARGET_MAX_DEG_BY_INDEX = [
    63.98, 73.98, 88.00, 92.99, 64.99, 74.99, 88.00, 92.99,
    83.98, 83.98, 83.98, 83.98, 83.98, 83.98, 83.98, 83.98,
    83.98, 83.98, 83.98, 83.98, 83.98,
]
_OPEN_20_RAD = (
    [0.1, 0.0, 0.0, 0.0], [0.0, 0.2, 0.1, 0.0], [0.0, 0.1, 0.1, 0.0],
    [0.0, 0.1, 0.1, 0.0], [-0.1, 0.1, 0.1, 0.0],
)
_FIST_20_RAD = (
    [0.6, 0.4, 0.7, 0.9], [0.0, 0.8, 0.9, 1.0], [0.0, 0.8, 0.9, 1.0],
    [0.0, 0.8, 0.9, 1.0], [0.0, 0.8, 0.9, 1.0],
)
_ANGLE_MAP = [[3, 0, 1, 2], [4, 5, 6, 7], [8, 9, 10, 11], [12, 13, 14, 15], [16, 17, 18, 19]]


def _load_default_auto_calibration_joint_indices() -> List[int]:
    fallback = [11]
    try:
        repo_root = Path(__file__).resolve().parents[2]
        source_path = repo_root / "ESP32-P4" / "ServoBoardMain" / "src" / "calibration" / "AutoCalibrationTask.cpp"
        text = source_path.read_text(encoding="utf-8", errors="ignore")
        match = re.search(
            r"const\s+uint8_t\s+kDefaultAutoCalibrationJoints\s*\[\]\s*=\s*\{([^}]*)\}",
            text,
            re.MULTILINE,
        )
        if not match:
            return fallback
        joints = [int(token) for token in re.findall(r"\b\d+\b", match.group(1))]
        joints = [idx for idx in joints if 0 <= idx < ENCODER_COUNT]
        return joints or fallback
    except Exception:
        return fallback


AUTO_CALIBRATION_JOINT_INDICES = _load_default_auto_calibration_joint_indices()


def _motor_channel_valid(idx: int) -> bool:
    return 0 <= idx < MOTOR_COUNT and ((VALID_MOTOR_MASK >> idx) & 0x1) != 0


FINGER_JOINT_RANGES = [
    ("Pinky J00-J03", 0, 3),
    ("Ring J04-J07", 4, 7),
    ("Middle J08-J11", 8, 11),
    ("Index J12-J15", 12, 15),
    ("Thumb J16-J20", 16, 20),
]


def _motor_rows_by_finger_group() -> Tuple[List[int], List[Tuple[int, int, str]]]:
    rows: List[int] = []
    group_ranges: List[Tuple[int, int, str]] = []
    used = set()

    for group_label, joint_start, joint_end in FINGER_JOINT_RANGES:
        group_start_row = len(rows)
        for joint_idx in range(joint_start, joint_end + 1):
            for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
                if (
                    _motor_channel_valid(motor_idx) and
                    motor_idx not in used and
                    int(mapped_joint) == joint_idx
                ):
                    rows.append(motor_idx)
                    used.add(motor_idx)
        if len(rows) > group_start_row:
            group_ranges.append((group_start_row, len(rows) - 1, group_label))

    for motor_idx in range(MOTOR_COUNT):
        if _motor_channel_valid(motor_idx) and motor_idx not in used:
            rows.append(motor_idx)
            used.add(motor_idx)
            group_ranges.append((len(rows) - 1, len(rows) - 1, f"Extra M{motor_idx:02d}"))

    for motor_idx in range(MOTOR_COUNT):
        if motor_idx not in used:
            rows.append(motor_idx)
            used.add(motor_idx)

    return rows, group_ranges


class MotorGroupBorderDelegate(QtWidgets.QStyledItemDelegate):
    def __init__(self, group_ranges: Sequence[Tuple[int, int, str]], parent=None):
        super().__init__(parent)
        self._group_ranges = list(group_ranges)

    def _range_for_row(self, row: int) -> Optional[Tuple[int, int, str]]:
        for start, end, label in self._group_ranges:
            if start <= row <= end:
                return start, end, label
        return None

    def paint(self, painter, option, index):  # noqa: N802 - Qt override
        super().paint(painter, option, index)
        group = self._range_for_row(index.row())
        if group is None:
            return
        start, end, _label = group
        model = index.model()
        col = index.column()
        last_col = model.columnCount() - 1 if model is not None else col

        pen = QtGui.QPen(QtGui.QColor("#c62828"))
        pen.setWidth(2)
        painter.save()
        painter.setPen(pen)
        rect = option.rect.adjusted(1, 1, -1, -1)
        if index.row() == start:
            painter.drawLine(rect.topLeft(), rect.topRight())
        if index.row() == end:
            painter.drawLine(rect.bottomLeft(), rect.bottomRight())
        if col == 0:
            painter.drawLine(rect.topLeft(), rect.bottomLeft())
        if col == last_col:
            painter.drawLine(rect.topRight(), rect.bottomRight())
        painter.restore()


class _ModeBody(QtWidgets.QWidget):
    def __init__(self, child: Optional[QtWidgets.QWidget] = None, parent=None):
        super().__init__(parent)
        self.child = child

    def update_state(self, state) -> None:
        updater = getattr(self.child, "update_state", None)
        if callable(updater):
            updater(state)

    def close(self) -> None:
        closer = getattr(self.child, "close", None)
        if callable(closer):
            closer()
        super().close()


def _preset_20rad_to_21deg(preset_20: tuple) -> List[float]:
    out = [0.0] * ENCODER_COUNT
    for finger_idx, indices in enumerate(_ANGLE_MAP):
        for joint_in_finger, motor_idx in enumerate(indices):
            if 0 <= motor_idx < ENCODER_COUNT:
                out[motor_idx] = math.degrees(float(preset_20[finger_idx][joint_in_finger]))
    return out


class ControlPanel(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.controller = context.controller
        self._updating = False
        self._interaction_enabled = False
        self._control_widgets: List[QtWidgets.QWidget] = []
        self._state: Optional[HandModel] = None
        self._estop_latched = False

        self.joint_current_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_encoder_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_status_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_target_spins: List[QtWidgets.QDoubleSpinBox] = []
        self.joint_sliders: List[QtWidgets.QSlider] = []
        self.joint_send_buttons: List[QtWidgets.QPushButton] = []
        self.joint_tie_sign_edits: List[QtWidgets.QLineEdit] = []
        self.joint_tie_x1_edits: List[QtWidgets.QLineEdit] = []
        self._joint_target_user_set: List[bool] = []
        self._joint_target_initialized = False
        self._joint_run_states = [False] * ENCODER_COUNT

        self.motor_turn_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_raw_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_abs_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_limit_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_joint_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_target_spins: List[QtWidgets.QSpinBox] = []
        self.motor_sliders: List[QtWidgets.QSlider] = []
        self.motor_send_buttons: List[QtWidgets.QPushButton] = []
        self.motor_recover_buttons: List[QtWidgets.QPushButton] = []
        self._motor_target_user_set: List[bool] = []
        self._motor_target_initialized = False
        self.abs_limit_unlock_btn: Optional[QtWidgets.QPushButton] = None
        self.abs_limit_lock_btn: Optional[QtWidgets.QPushButton] = None
        # UI relative multi-turn targets. Device command = origin feedback + this value.
        self._motor_abs_pos: List[Optional[int]] = [None] * MOTOR_COUNT
        # Servo absolute feedback captured when direct control is synced or held.
        self._motor_session_origin_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_dir_hint: List[int] = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear: List[bool] = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count: List[int] = [0] * MOTOR_COUNT
        self._motor_last_slider: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_slider_dragging: List[bool] = [False] * MOTOR_COUNT
        self._motor_last_sent_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT
        self._last_abs_limit_warning_ts = 0.0
        self._last_feedback_abs_limit_warning_ts = 0.0

        self._test_running = False
        self._ff_test_running = False
        self._tri_test_running = False
        self._local_test_running = False
        self._test_timer = QtCore.QTimer(self)
        self._test_timer.setInterval(TEST_TICK_MS)
        self._test_timer.timeout.connect(self._test_tick)
        self._ff_test_timer = QtCore.QTimer(self)
        self._ff_test_timer.setInterval(TEST_TICK_MS)
        self._ff_test_timer.timeout.connect(self._ff_test_tick)
        self._tri_test_timer = QtCore.QTimer(self)
        self._tri_test_timer.setInterval(TEST_TICK_MS)
        self._tri_test_timer.timeout.connect(self._tri_test_tick)
        self._test_active_rows: List[Tuple[int, float, float, float, float]] = []
        self._test_t0 = 0.0
        self._test_hold_s = float(TEST_DEFAULT_HOLD_S)
        self._ff_test_joint_idx = 0
        self._ff_test_targets: List[float] = []
        self._ff_test_step_s = float(FF_TEST_DEFAULT_STEP_S)
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = 0.0
        self._tri_test_joint_idx = 0
        self._tri_test_min_deg = 0.0
        self._tri_test_max_deg = 0.0
        self._tri_test_freq_hz = float(TRI_TEST_DEFAULT_FREQ_HZ)
        self._tri_test_t0 = 0.0

        self._body_widget: Optional[QtWidgets.QWidget] = None
        self._algorithm_view: Optional[AlgorithmView] = None
        self._app_mode = "manual"
        self.scope_combo: Optional[QtWidgets.QComboBox] = None
        self._scope_selection_index = 0
        self._last_manual_tab_index = 0

        self._build()
        self._apply_interaction_enabled(False)
        self._apply_control_mode_policy()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        device = QtWidgets.QGroupBox("Device")
        dev_layout = QtWidgets.QGridLayout(device)
        self.enable_btn = QtWidgets.QPushButton("ENABLE")
        self.stop_btn = QtWidgets.QPushButton("E-STOP")
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.home_btn = QtWidgets.QPushButton("Home")
        self.reset_btn = QtWidgets.QPushButton("Reset State")
        self.reset_btn.setToolTip("Clear latched protection flags, clear command state, and return firmware to UNCONTROLLABLE")
        self.calib_btn = QtWidgets.QPushButton("Calibrate")
        self.rope_init_btn = QtWidgets.QPushButton("Rope Init +50")
        self.rope_init_btn.setToolTip("Move calibrated joints that still read below 0 deg by +50 counts")
        self.abs_limit_unlock_btn = QtWidgets.QPushButton("Unlock ABS")
        self.abs_limit_unlock_btn.setToolTip("Temporarily disable calibrated ABS travel limits for the current control set")
        self.abs_limit_lock_btn = QtWidgets.QPushButton("Lock ABS")
        self.abs_limit_lock_btn.setToolTip("Re-enable calibrated ABS travel limits for the current control set")
        self.enable_btn.setMinimumHeight(46)
        self.stop_btn.setMinimumHeight(46)
        self.pause_btn.setMinimumHeight(46)
        self.enable_btn.setStyleSheet("font-weight:700;background:#2e7d32;color:white;")
        self.stop_btn.setStyleSheet("font-weight:700;background:#c62828;color:white;")
        self.pause_btn.setStyleSheet("font-weight:700;background:#f9a825;color:black;")
        dev_layout.addWidget(self.enable_btn, 0, 0)
        dev_layout.addWidget(self.stop_btn, 0, 1)
        dev_layout.addWidget(self.pause_btn, 0, 2)
        dev_layout.addWidget(self.home_btn, 0, 3)
        dev_layout.addWidget(self.calib_btn, 1, 3)
        dev_layout.addWidget(self.reset_btn, 0, 4)
        dev_layout.addWidget(self.rope_init_btn, 1, 4)
        dev_layout.addWidget(self.abs_limit_unlock_btn, 0, 5)
        dev_layout.addWidget(self.abs_limit_lock_btn, 1, 5)
        for col in range(6):
            dev_layout.setColumnStretch(col, 1)
        layout.addWidget(device)
        self._update_connection_lights()

        self.enable_btn.clicked.connect(lambda _checked=False: self.start_controller())
        self.stop_btn.clicked.connect(lambda _checked=False: self.stop_controller())
        self.pause_btn.clicked.connect(lambda _checked=False: self.pause_controller())
        self.home_btn.clicked.connect(lambda _checked=False: self.apply_home())
        self.reset_btn.clicked.connect(lambda _checked=False: self.reset_controller())
        self.calib_btn.clicked.connect(lambda _checked=False: self.calibrate())
        self.rope_init_btn.clicked.connect(lambda _checked=False: self.rope_init_controller())
        self.abs_limit_unlock_btn.clicked.connect(lambda _checked=False: self.set_abs_limit_lock(False))
        self.abs_limit_lock_btn.clicked.connect(lambda _checked=False: self.set_abs_limit_lock(True))
        self.body_container = QtWidgets.QWidget()
        self.body_layout = QtWidgets.QVBoxLayout(self.body_container)
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.body_container, 1)
        self._build_manual_body()

    @staticmethod
    def _scope_range(start: int, end: int, require_can: bool = True) -> Tuple[int, int, bool]:
        motor_mask = 0
        joint_mask = 0
        for joint_idx in range(start, end + 1):
            if 0 <= joint_idx < ENCODER_COUNT:
                joint_mask |= (1 << joint_idx)
        for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
            if _motor_channel_valid(motor_idx) and start <= int(joint_idx) <= end:
                motor_mask |= (1 << motor_idx)
        return motor_mask, joint_mask, require_can

    def _make_scope_combo(self) -> QtWidgets.QComboBox:
        combo = QtWidgets.QComboBox()
        combo.setMinimumWidth(230)
        combo.setToolTip("Select the active motors/joints used for control, calibration, and READY checks")
        dual_suffix = " (dual)" if HAND_PROFILE.lower() == "v2" else ""
        combo.addItem("Control Set: Pinky J00-J03", self._scope_range(0, 3, True))
        combo.addItem(f"Control Set: Joint J00{dual_suffix}", self._scope_range(0, 0, True))
        combo.addItem("Control Set: Joint J01", self._scope_range(1, 1, True))
        combo.addItem("Control Set: Joint J02", self._scope_range(2, 2, True))
        combo.addItem("Control Set: Joint J03", self._scope_range(3, 3, True))
        combo.addItem("Control Set: Ring J04-J07", self._scope_range(4, 7, True))
        combo.addItem("Control Set: Middle J08-J11", self._scope_range(8, 11, True))
        combo.addItem("Control Set: Index J12-J15", self._scope_range(12, 15, True))
        combo.addItem("Control Set: Thumb J16-J20", self._scope_range(16, 20, True))
        combo.addItem("Control Set: Full Hand", (VALID_MOTOR_MASK, (1 << ENCODER_COUNT) - 1, True))
        if _motor_channel_valid(14):
            combo.addItem("Control Set: Single M14", (1 << 14, 1 << 14, True))
            combo.addItem("Control Set: M14 Servo Only", (1 << 14, 1 << 14, False))
        if _motor_channel_valid(11):
            combo.addItem("Control Set: Single M11", (1 << 11, 1 << 11, True))
            combo.addItem("Control Set: M11 Servo Only", (1 << 11, 1 << 11, False))
        selected = max(0, min(int(getattr(self, "_scope_selection_index", 0)), combo.count() - 1))
        combo.setCurrentIndex(selected)
        combo.currentIndexChanged.connect(lambda _idx: self._on_active_scope_changed())
        return combo

    def _make_connection_light(self) -> QtWidgets.QLabel:
        light = QtWidgets.QLabel()
        light.setFixedSize(14, 14)
        light.setToolTip("Lower serial disconnected")
        return light

    def _button_with_connection_light(
        self,
        button: QtWidgets.QPushButton,
        light: QtWidgets.QLabel,
    ) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(panel)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.addWidget(button, 1)
        row.addWidget(light, 0, QtCore.Qt.AlignVCenter)
        return panel

    def _update_connection_lights(self) -> None:
        connected = bool(self.context.is_comm_connected())
        color = "#2e7d32" if connected else "#c62828"
        border = "#1b5e20" if connected else "#8e0000"
        tip = "Lower serial connected" if connected else "Lower serial disconnected"
        for light in (getattr(self, "run_conn_light", None), getattr(self, "stop_conn_light", None)):
            if light is None:
                continue
            light.setToolTip(tip)
            light.setStyleSheet(
                f"background:{color}; border:1px solid {border}; border-radius:7px;"
            )

    def _clear_body(self) -> None:
        self.stop_all_tests("stopped on view switch", update_status=False)
        if getattr(self, "scope_combo", None) is not None:
            try:
                self._scope_selection_index = int(self.scope_combo.currentIndex())
            except RuntimeError:
                pass
        if getattr(self, "tabs", None) is not None:
            try:
                self._last_manual_tab_index = int(self.tabs.currentIndex())
            except RuntimeError:
                pass
        if self._body_widget is not None:
            closer = getattr(self._body_widget, "close", None)
            if callable(closer):
                try:
                    closer()
                except RuntimeError:
                    pass
            self.body_layout.removeWidget(self._body_widget)
            self._body_widget.deleteLater()
            self._body_widget = None
        self.scope_combo = None
        self._algorithm_view = None
        self._reset_manual_widget_refs()

    def _reset_manual_widget_refs(self) -> None:
        self._control_widgets = []
        self.manual_check = None
        self.send_all_btn = None
        self.open_btn = None
        self.fist_btn = None
        self.zero_btn = None
        self.tabs = None
        self.joint_table = None
        self.motor_table = None
        self.test_panel = None
        self.test_start_btn = None
        self.test_stop_btn = None
        self.test_hold_spin = None
        self.ff_start_btn = None
        self.ff_stop_btn = None
        self.ff_joint_combo = None
        self.ff_targets_edit = None
        self.ff_step_spin = None
        self.tri_start_btn = None
        self.tri_stop_btn = None
        self.tri_joint_combo = None
        self.tri_min_spin = None
        self.tri_max_spin = None
        self.tri_freq_spin = None
        self.local_start_btn = None
        self.local_stop_btn = None
        self.local_freq = None
        self.local_amp = None
        self.test_joint_combos = []
        self.test_min_spins = []
        self.test_max_spins = []
        self.test_freq_spins = []
        self.test_phase_spins = []
        self.joint_current_items = []
        self.joint_encoder_items = []
        self.joint_status_items = []
        self.joint_target_spins = []
        self.joint_sliders = []
        self.joint_send_buttons = []
        self.joint_tie_sign_edits = []
        self.joint_tie_x1_edits = []
        self._joint_target_user_set = []
        self.motor_turn_items = []
        self.motor_raw_items = []
        self.motor_abs_items = []
        self.motor_joint_items = []
        self.motor_target_spins = []
        self.motor_sliders = []
        self.motor_send_buttons = []
        self.motor_recover_buttons = []
        self._motor_target_user_set = []
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._joint_run_states = [False] * ENCODER_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT

    def _build_scope_workflow(self, title: str) -> tuple[QtWidgets.QGroupBox, QtWidgets.QHBoxLayout]:
        self.scope_combo = self._make_scope_combo()
        workflow = QtWidgets.QGroupBox(title)
        wf_layout = QtWidgets.QHBoxLayout(workflow)
        wf_layout.addWidget(QtWidgets.QLabel("Control Set:"))
        wf_layout.addWidget(self.scope_combo)
        return workflow, wf_layout

    def _build_manual_body(self) -> None:
        interaction_enabled = bool(getattr(self, "_interaction_enabled", False))
        self._clear_body()
        self._reset_manual_widget_refs()
        self._app_mode = "manual"

        body = _ModeBody()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)

        workflow, wf_layout = self._build_scope_workflow("Manual Workflow")
        self.manual_check = QtWidgets.QCheckBox("Hand Manual")
        self.manual_check.setChecked(False)
        self.send_all_btn = QtWidgets.QPushButton("Send All")
        self.open_btn = QtWidgets.QPushButton("Open Pose")
        self.fist_btn = QtWidgets.QPushButton("Fist Pose")
        self.zero_btn = QtWidgets.QPushButton("Zero Pose")
        wf_layout.insertWidget(0, self.manual_check)
        wf_layout.addStretch(1)
        for btn in [self.open_btn, self.fist_btn, self.zero_btn, self.send_all_btn]:
            wf_layout.addWidget(btn)
        layout.addWidget(workflow)

        self.manual_check.toggled.connect(self._apply_interaction_enabled)
        self.send_all_btn.clicked.connect(lambda _checked=False: self.send_all())
        self.open_btn.clicked.connect(lambda _checked=False: self.apply_preset(_preset_20rad_to_21deg(_OPEN_20_RAD)))
        self.fist_btn.clicked.connect(lambda _checked=False: self.apply_preset(_preset_20rad_to_21deg(_FIST_20_RAD)))
        self.zero_btn.clicked.connect(lambda _checked=False: self.apply_preset([0.0] * ENCODER_COUNT))

        self.tabs = QtWidgets.QTabWidget()
        self.joint_table = self._build_joint_table()
        self.motor_table = self._build_motor_table()
        self.test_panel = self._build_test_panel()
        self.tabs.addTab(self.joint_table, "Joint Channels")
        self.tabs.addTab(self.motor_table, "Motor Channels")
        self.tabs.addTab(self.test_panel, "Test")
        self.tabs.currentChanged.connect(lambda _idx: self._on_manual_tab_changed())
        tab_index = max(0, min(int(getattr(self, "_last_manual_tab_index", 0)), self.tabs.count() - 1))
        self.tabs.setCurrentIndex(tab_index)
        layout.addWidget(self.tabs, 1)
        self.body_layout.addWidget(body)
        self._body_widget = body
        self._apply_interaction_enabled(interaction_enabled)
        self._on_manual_tab_changed()

    def _build_teleop_body(self) -> None:
        self._clear_body()
        self._reset_manual_widget_refs()
        self._app_mode = "teleop"

        body = _ModeBody()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)

        workflow, wf_layout = self._build_scope_workflow("Teleop Workflow")
        wf_layout.addStretch(1)
        layout.addWidget(workflow)

        teleop_view = TeleopView(self.context)
        body.child = teleop_view
        layout.addWidget(teleop_view, 1)

        self.body_layout.addWidget(body)
        self._body_widget = body

    def _make_item(self, text: str = "-") -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        return item

    def _build_joint_table(self) -> QtWidgets.QTableWidget:
        table = QtWidgets.QTableWidget(ENCODER_COUNT, 9)
        table.setHorizontalHeaderLabels([
            "Joint", "Enc", "Current", "Raw", "Min", "Target", "Max", "Action",
            "Target Slider",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        for i in range(ENCODER_COUNT):
            table.setItem(i, 0, self._make_item(f"J{i:02d}"))
            status = self._make_item("*")
            status.setTextAlignment(QtCore.Qt.AlignCenter)
            status.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
            enc = self._make_item("-")
            cur = self._make_item("-")
            self.joint_status_items.append(status)
            self.joint_current_items.append(cur)
            self.joint_encoder_items.append(enc)
            table.setItem(i, 1, status)
            table.setItem(i, 2, cur)
            table.setItem(i, 3, enc)
            table.setItem(i, 4, self._make_item(f"{self._joint_target_min_deg(i):.2f}"))
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(self._joint_target_min_deg(i), self._joint_target_max_deg(i))
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=i: self._joint_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: self._mark_joint_target_user_set(idx))
            self.joint_target_spins.append(spin)
            table.setCellWidget(i, 5, spin)
            table.setItem(i, 6, self._make_item(f"{self._joint_target_max_deg(i):.2f}"))
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_joint_row(idx))
            self.joint_send_buttons.append(send)
            table.setCellWidget(i, 7, send)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(self._joint_target_min_deg(i) * 100), int(self._joint_target_max_deg(i) * 100))
            slider.valueChanged.connect(lambda value, idx=i: self._joint_slider_changed(idx, value))
            self.joint_sliders.append(slider)
            table.setCellWidget(i, 8, slider)
            self._joint_target_user_set.append(False)
            self._control_widgets.extend([spin, send, slider])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(8, QtWidgets.QHeaderView.Stretch)
        return table

    def _build_motor_table(self) -> QtWidgets.QTableWidget:
        motor_rows, group_ranges = _motor_rows_by_finger_group()
        table = QtWidgets.QTableWidget(len(motor_rows), 12)
        table.setHorizontalHeaderLabels([
            "Motor", "Turn", "RawNow", "AbsNow", "Limit", "Target(rel)", "Action", "Raw",
            "-", "Raw Slider", "+", "Joint Angle",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        self.motor_turn_items = [None] * MOTOR_COUNT
        self.motor_raw_items = [None] * MOTOR_COUNT
        self.motor_abs_items = [None] * MOTOR_COUNT
        self.motor_limit_items = [None] * MOTOR_COUNT
        self.motor_joint_items = [None] * MOTOR_COUNT
        self.motor_target_spins = [None] * MOTOR_COUNT
        self.motor_sliders = [None] * MOTOR_COUNT
        self.motor_send_buttons = [None] * MOTOR_COUNT
        self.motor_recover_buttons = [None] * MOTOR_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        row_to_group_label = {}
        for start, end, label in group_ranges:
            for row in range(start, end + 1):
                row_to_group_label[row] = label
        for row, i in enumerate(motor_rows):
            group_label = row_to_group_label.get(row, "")
            is_group_start = row in [start for start, _end, _label in group_ranges]
            motor_text = f"M{i:02d}  {group_label}" if is_group_start and group_label else f"M{i:02d}"
            motor_item = self._make_item(motor_text)
            joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
            if group_label:
                motor_item.setToolTip(f"{group_label}; mapped joint J{int(joint_idx):02d}" if int(joint_idx) >= 0 else group_label)
            if is_group_start:
                font = motor_item.font()
                font.setBold(True)
                motor_item.setFont(font)
            table.setItem(row, 0, motor_item)
            turn = self._make_item("-")
            raw_now = self._make_item("-")
            abs_now = self._make_item("-")
            limit = self._make_item("-")
            joint = self._make_item("-")
            self.motor_turn_items[i] = turn
            self.motor_raw_items[i] = raw_now
            self.motor_abs_items[i] = abs_now
            self.motor_limit_items[i] = limit
            self.motor_joint_items[i] = joint
            table.setItem(row, 1, turn)
            table.setItem(row, 2, raw_now)
            table.setItem(row, 3, abs_now)
            table.setItem(row, 4, limit)
            spin = QtWidgets.QSpinBox()
            spin.setRange(MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX)
            spin.setSingleStep(10)
            spin.valueChanged.connect(lambda value, idx=i: self._motor_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: self._mark_motor_target_user_set(idx))
            self.motor_target_spins[i] = spin
            table.setCellWidget(row, 5, spin)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_motor_row(idx))
            self.motor_send_buttons[i] = send
            table.setCellWidget(row, 6, send)
            recover = QtWidgets.QPushButton("Raw")
            recover.clicked.connect(lambda _=False, idx=i: self.recover_motor_row_to_raw(idx))
            recover.setToolTip("Recover this motor from +/-30719 using its current single-turn raw feedback")
            self.motor_recover_buttons[i] = recover
            table.setCellWidget(row, 7, recover)
            minus = QtWidgets.QPushButton("-")
            minus.setMaximumWidth(34)
            minus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, -1))
            table.setCellWidget(row, 8, minus)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(MOTOR_TARGET_SCALE_LO, MOTOR_TARGET_SCALE_HI)
            slider.sliderPressed.connect(lambda idx=i: self._on_motor_slider_press(idx))
            slider.sliderReleased.connect(lambda idx=i: self._on_motor_slider_release(idx))
            slider.valueChanged.connect(lambda value, idx=i: self._motor_slider_changed(idx, value))
            self.motor_sliders[i] = slider
            table.setCellWidget(row, 9, slider)
            plus = QtWidgets.QPushButton("+")
            plus.setMaximumWidth(34)
            plus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, 1))
            table.setCellWidget(row, 10, plus)
            table.setItem(row, 11, joint)
            self._control_widgets.extend([spin, send, recover, minus, slider, plus])
            if not _motor_channel_valid(i):
                table.setRowHidden(row, True)
        table.setItemDelegate(MotorGroupBorderDelegate(group_ranges, table))
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(9, QtWidgets.QHeaderView.Stretch)
        table.setColumnWidth(9, 220)
        table.horizontalHeader().setSectionResizeMode(11, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(11, 82)
        return table

    def _build_test_panel(self) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(box)
        layout.addWidget(self._build_sine_test_box())
        layout.addWidget(self._build_feedforward_test_box())
        layout.addWidget(self._build_triangle_test_box())
        layout.addWidget(self._build_local_test_box())
        layout.addStretch(1)
        return box

    def _joint_choices(self) -> List[str]:
        return ["None"] + [f"J{i:02d}" for i in range(ENCODER_COUNT)]

    def _build_sine_test_box(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Joint Sine Test")
        layout = QtWidgets.QGridLayout(group)
        for col, text in enumerate(["#", "Joint", "Min(deg)", "Max(deg)", "Freq(Hz)", "Phase(deg)"]):
            layout.addWidget(QtWidgets.QLabel(text), 0, col)
        self.test_joint_combos: List[QtWidgets.QComboBox] = []
        self.test_min_spins: List[QtWidgets.QDoubleSpinBox] = []
        self.test_max_spins: List[QtWidgets.QDoubleSpinBox] = []
        self.test_freq_spins: List[QtWidgets.QDoubleSpinBox] = []
        self.test_phase_spins: List[QtWidgets.QDoubleSpinBox] = []
        for row in range(TEST_ROW_COUNT):
            layout.addWidget(QtWidgets.QLabel(str(row + 1)), row + 1, 0)
            combo = QtWidgets.QComboBox()
            combo.addItems(self._joint_choices())
            combo.currentTextChanged.connect(lambda _text, idx=row: self._on_test_joint_changed(idx))
            lo = self._make_float_spin(0.0, 120.0, 0.0, 2)
            hi = self._make_float_spin(0.0, 120.0, 10.0, 2)
            freq = self._make_float_spin(0.001, 100.0, TEST_DEFAULT_FREQ_HZ, 4)
            phase = self._make_float_spin(0.0, 360.0, TEST_DEFAULT_PHASE_DEG, 2)
            for col, widget in enumerate([combo, lo, hi, freq, phase], start=1):
                layout.addWidget(widget, row + 1, col)
            self.test_joint_combos.append(combo)
            self.test_min_spins.append(lo)
            self.test_max_spins.append(hi)
            self.test_freq_spins.append(freq)
            self.test_phase_spins.append(phase)
            self._control_widgets.extend([combo, lo, hi, freq, phase])
        row = QtWidgets.QHBoxLayout()
        self.test_start_btn = QtWidgets.QPushButton("Start Test")
        self.test_stop_btn = QtWidgets.QPushButton("Stop Test")
        self.test_hold_spin = self._make_float_spin(0.0, 60.0, TEST_DEFAULT_HOLD_S, 3)
        self.test_start_btn.clicked.connect(lambda _checked=False: self._on_start_test())
        self.test_stop_btn.clicked.connect(lambda _checked=False: self._on_stop_test())
        for widget in [
            self.test_start_btn, self.test_stop_btn, QtWidgets.QLabel("Initial Hold(s)"), self.test_hold_spin,
        ]:
            row.addWidget(widget)
        row.addStretch(1)
        layout.addLayout(row, TEST_ROW_COUNT + 1, 0, 1, 6)
        self._control_widgets.extend([self.test_start_btn, self.test_stop_btn, self.test_hold_spin])
        return group

    def _build_feedforward_test_box(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Feedforward Test")
        layout = QtWidgets.QGridLayout(group)
        self.ff_joint_combo = QtWidgets.QComboBox()
        self.ff_joint_combo.addItems(self._joint_choices())
        self.ff_targets_edit = QtWidgets.QLineEdit(FF_TEST_DEFAULT_TARGETS_DEG)
        self.ff_step_spin = self._make_float_spin(0.001, 60.0, FF_TEST_DEFAULT_STEP_S, 3)
        self.ff_start_btn = QtWidgets.QPushButton("Start Feedforward Test")
        self.ff_stop_btn = QtWidgets.QPushButton("Stop Feedforward Test")
        self.ff_start_btn.clicked.connect(lambda _checked=False: self._on_start_ff_test())
        self.ff_stop_btn.clicked.connect(lambda _checked=False: self._on_stop_ff_test())
        for col, text in enumerate(["Joint", "Targets(deg)", "Step Time(s)"]):
            layout.addWidget(QtWidgets.QLabel(text), 0, col)
        layout.addWidget(self.ff_joint_combo, 1, 0)
        layout.addWidget(self.ff_targets_edit, 1, 1)
        layout.addWidget(self.ff_step_spin, 1, 2)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.ff_start_btn)
        row.addWidget(self.ff_stop_btn)
        row.addStretch(1)
        layout.addLayout(row, 2, 0, 1, 3)
        self._control_widgets.extend([
            self.ff_joint_combo, self.ff_targets_edit, self.ff_step_spin, self.ff_start_btn, self.ff_stop_btn,
        ])
        return group

    def _build_triangle_test_box(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Triangle Wave Test")
        layout = QtWidgets.QGridLayout(group)
        self.tri_joint_combo = QtWidgets.QComboBox()
        self.tri_joint_combo.addItems(self._joint_choices())
        self.tri_min_spin = self._make_float_spin(0.0, 120.0, 0.0, 2)
        self.tri_max_spin = self._make_float_spin(0.0, 120.0, 10.0, 2)
        self.tri_freq_spin = self._make_float_spin(0.001, 100.0, TRI_TEST_DEFAULT_FREQ_HZ, 4)
        self.tri_start_btn = QtWidgets.QPushButton("Start Triangle Test")
        self.tri_stop_btn = QtWidgets.QPushButton("Stop Triangle Test")
        self.tri_start_btn.clicked.connect(lambda _checked=False: self._on_start_tri_test())
        self.tri_stop_btn.clicked.connect(lambda _checked=False: self._on_stop_tri_test())
        for col, text in enumerate(["Joint", "Min(deg)", "Max(deg)", "Freq(Hz)"]):
            layout.addWidget(QtWidgets.QLabel(text), 0, col)
        for col, widget in enumerate([self.tri_joint_combo, self.tri_min_spin, self.tri_max_spin, self.tri_freq_spin]):
            layout.addWidget(widget, 1, col)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.tri_start_btn)
        row.addWidget(self.tri_stop_btn)
        row.addStretch(1)
        layout.addLayout(row, 2, 0, 1, 4)
        self._control_widgets.extend([
            self.tri_joint_combo, self.tri_min_spin, self.tri_max_spin, self.tri_freq_spin,
            self.tri_start_btn, self.tri_stop_btn,
        ])
        return group

    def _build_local_test_box(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Local Test Mode")
        layout = QtWidgets.QHBoxLayout(group)
        self.local_freq = self._make_float_spin(0.001, 100.0, 0.5, 4)
        self.local_amp = self._make_float_spin(-180.0, 180.0, 20.0, 3)
        self.local_start_btn = QtWidgets.QPushButton("Start Local Test")
        self.local_stop_btn = QtWidgets.QPushButton("Stop Local Test")
        self.local_start_btn.clicked.connect(lambda _checked=False: self._on_start_local_test())
        self.local_stop_btn.clicked.connect(lambda _checked=False: self._on_stop_local_test())
        for widget in [
            QtWidgets.QLabel("Freq(Hz)"), self.local_freq, QtWidgets.QLabel("Amp(deg)"), self.local_amp,
            self.local_start_btn, self.local_stop_btn,
        ]:
            layout.addWidget(widget)
        layout.addStretch(1)
        self._control_widgets.extend([self.local_freq, self.local_amp, self.local_start_btn, self.local_stop_btn])
        return group

    @staticmethod
    def _make_float_spin(low: float, high: float, value: float, decimals: int) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(float(low), float(high))
        spin.setDecimals(int(decimals))
        spin.setSingleStep(0.1 if decimals <= 2 else 0.01)
        spin.setValue(float(value))
        return spin

    def _set_status(self, text: str) -> None:
        self.context.set_status(text)

    def _warn(self, title: str, text: str) -> None:
        QtWidgets.QMessageBox.warning(self, title, text)

    def _error(self, title: str, text: str) -> None:
        QtWidgets.QMessageBox.critical(self, title, text)

    def _manual_submode(self) -> str:
        if not hasattr(self, "tabs"):
            return "joint"
        if self.tabs is None:
            return "joint"
        widget = self.tabs.currentWidget()
        if widget is getattr(self, "motor_table", None):
            return "motor"
        if widget is getattr(self, "test_panel", None):
            return "test"
        return "joint"

    def _on_manual_tab_changed(self) -> None:
        previous = getattr(self, "_last_control_mode", "joint")
        current = self._manual_submode()
        if previous == "test" and current != "test":
            self.stop_all_tests("stopped on mode switch", update_status=False)
        if previous != "motor" and current == "motor":
            state = self._state
            if state is not None and getattr(state, "system_state", None) == SYSTEM_STATE_READY:
                self._warn(
                    "Leaving READY",
                    "Motor direct control does not use the calibrated joint workspace check. "
                    "The lower controller will switch back to IDLE; return to Joint Channels to enter READY again.",
                )
        if current == "motor":
            self._refresh_motor_feedback_view(sync_target=True)
        elif current == "test":
            pass
        else:
            self._refresh_joint_feedback_view(sync_target=True)
        self._last_control_mode = current
        self._apply_control_mode_policy()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _apply_interaction_enabled(self, enabled: bool) -> None:
        self._interaction_enabled = bool(enabled)
        if getattr(self, "manual_check", None) is not None:
            self.manual_check.blockSignals(True)
            self.manual_check.setChecked(self._interaction_enabled)
            self.manual_check.blockSignals(False)
        for widget in self._control_widgets:
            try:
                widget.setEnabled(self._interaction_enabled)
            except Exception:
                pass
        self._update_manual_action_buttons()
        self._apply_control_mode_policy()
        self._set_status(f"Hand Manual: {'ON' if self._interaction_enabled else 'OFF'}")
        if not self._interaction_enabled:
            self.stop_all_tests("stopped: Hand Manual OFF", update_status=False)

    def _apply_control_mode_policy(self) -> None:
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        if mode == "motor":
            self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.controller.set_pid_control(False)
        else:
            self.controller.set_control_mode(ControlMode.JOINT_ANGLE)
            self.controller.set_pid_control(bool(self._interaction_enabled))

    def _update_manual_action_buttons(self) -> None:
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        interaction = bool(self._interaction_enabled)
        system_state = getattr(self._state, "system_state", None) if self._state is not None else None
        ready_for_feedback = system_state == SYSTEM_STATE_READY
        for btn in [getattr(self, "send_all_btn", None)]:
            if btn is not None:
                btn.setEnabled(interaction and ready_for_feedback)
        for btn in [getattr(self, "open_btn", None), getattr(self, "fist_btn", None), getattr(self, "zero_btn", None), self.home_btn]:
            if btn is not None:
                btn.setEnabled(interaction and ready_for_feedback and mode in ("joint", "test"))
        if getattr(self, "rope_init_btn", None) is not None:
            self.rope_init_btn.setEnabled(interaction and mode == "joint")
        if getattr(self, "calib_btn", None) is not None:
            self.calib_btn.setEnabled(system_state in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY))
        if getattr(self, "mid_calib_btn", None) is not None:
            self.mid_calib_btn.setEnabled(bool(self.context.is_comm_connected()))
        has_abs_limit = self._active_scope_abs_limit_valid_mask() != 0
        if getattr(self, "abs_limit_unlock_btn", None) is not None:
            self.abs_limit_unlock_btn.setEnabled(bool(self.context.is_comm_connected()) and has_abs_limit)
        if getattr(self, "abs_limit_lock_btn", None) is not None:
            self.abs_limit_lock_btn.setEnabled(bool(self.context.is_comm_connected()) and has_abs_limit)
        if getattr(self, "pause_btn", None) is not None:
            self.pause_btn.setEnabled(system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK))
        self._set_test_button_states()

    def sync_app_mode(self, mode_name: str) -> None:
        mode = str(mode_name or "").lower()
        if "teleop" in mode:
            self._build_teleop_body()
            self.controller.set_control_mode(ControlMode.JOINT_ANGLE)
            self.controller.set_pid_control(True)
        elif "algorithm" in mode:
            self._clear_body()
            self._app_mode = "algorithm"
            self._algorithm_view = AlgorithmView(self.context)
            self._body_widget = self._algorithm_view
            self.body_layout.addWidget(self._body_widget)
            self.controller.set_control_mode(ControlMode.JOINT_ANGLE)
            self.controller.set_pid_control(True)
        else:
            self._build_manual_body()
            if self._state is not None:
                self.update_state(self._state)

    def _joint_spin_changed(self, idx: int, value: float) -> None:
        if self._updating:
            return
        clamped = self._clamp_joint_target_deg(idx, float(value))
        self.joint_sliders[idx].blockSignals(True)
        self.joint_sliders[idx].setValue(int(round(clamped * 100.0)))
        self.joint_sliders[idx].blockSignals(False)

    def _joint_slider_changed(self, idx: int, value: int) -> None:
        if self._updating:
            return
        target = self._clamp_joint_target_deg(idx, float(value) / 100.0)
        self.joint_target_spins[idx].blockSignals(True)
        self.joint_target_spins[idx].setValue(target)
        self.joint_target_spins[idx].blockSignals(False)
        if self._interaction_enabled:
            self._on_joint_slider_change(idx, target)

    def _motor_spin_changed(self, idx: int, value: int) -> None:
        if self._updating:
            return
        relative_target = clamp_motor_abs(int(value))
        single = self._slider_from_abs_pos(relative_target)
        self.motor_sliders[idx].blockSignals(True)
        self.motor_sliders[idx].setValue(single)
        self.motor_sliders[idx].blockSignals(False)
        if 0 <= idx < MOTOR_COUNT:
            if self._motor_session_origin_abs[idx] is None:
                self._motor_resync_one_channel(idx)
            self._motor_abs_pos[idx] = relative_target
            self._motor_last_slider[idx] = single

    def _motor_slider_changed(self, idx: int, value: int) -> None:
        if self._updating:
            return
        if self._interaction_enabled:
            self._on_motor_slider_change(idx, int(value))
        else:
            self.motor_target_spins[idx].blockSignals(True)
            self.motor_target_spins[idx].setValue(int(value))
            self.motor_target_spins[idx].blockSignals(False)

    def resolved_joint_targets(self) -> List[float]:
        targets = []
        for i in range(ENCODER_COUNT):
            if i < len(self._joint_target_user_set) and self._joint_target_user_set[i]:
                raw = float(self.joint_target_spins[i].value())
            else:
                raw = self._joint_feedback_target(i)
            value = self._clamp_joint_target_deg(i, raw)
            targets.append(value)
        return targets

    def resolved_motor_targets(self) -> List[int]:
        state = self._state
        return [self._motor_abs_for_command_packet(i, state) for i in range(MOTOR_COUNT)]

    def motor_sweep_targets(self) -> List[int]:
        return [self._read_motor_target_slider_int(i) for i in range(MOTOR_COUNT)]

    def recover_motor_row_to_raw(self, idx: int) -> None:
        if not self._interaction_enabled:
            return
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        try:
            raw_target = self._motor_raw_single_feedback(idx)
            if raw_target is None:
                self._set_status(f"Raw recover blocked: waiting for M{idx:02d} raw feedback")
                return
            if not self.context.is_comm_connected():
                raise RuntimeError("Serial port is not connected")

            self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.controller.set_pid_control(False)
            if self._estop_latched and not self.controller.is_started():
                self._set_status("Raw recover blocked: press ENABLE after E-STOP")
                return
            if not self.controller.is_started():
                self._set_status("Raw recover blocked: waiting for controllable IDLE after ENABLE")
                return
            unknown_mask = self._active_scope_multiturn_unknown_mask()
            if unknown_mask != 0 and (unknown_mask & ~(1 << idx)) != 0:
                self._set_status(
                    f"Raw recover blocked: choose a single-motor scope for M{idx:02d} while Turn is UNKNOWN"
                )
                return

            if not self._send_motor_raw_recovery_target(idx, int(raw_target), f"Raw recover M{idx:02d}"):
                return
            self._motor_session_origin_abs[idx] = int(raw_target)
            self._motor_abs_pos[idx] = 0
            self._motor_last_slider[idx] = 0
            self._motor_target_user_set[idx] = False
            self._set_motor_target_widget(idx, 0)
            self._set_row_running(True, idx, True)
            self.context.emit_action(f"Raw recover M{idx:02d}: targetAbs={int(raw_target)}")
            self._set_status(f"M{idx:02d} raw recover sent: abs={int(raw_target)}")
        except Exception as exc:
            self._error("Raw Recover Failed", f"{type(exc).__name__}: {exc}")

    def send_joint_row(self, idx: int) -> None:
        if not self._interaction_enabled:
            return
        if idx < len(self._joint_run_states) and self._joint_run_states[idx]:
            self._hold_joint_row_at_feedback(idx)
            self._set_row_running(False, idx, False)
            self._set_status(f"Channel held at current feedback (J{idx})")
            return
        self._mark_joint_target_user_set(idx)
        if not self._guard_joint_command_started():
            return
        targets = self.resolved_joint_targets()
        self.context.send_joint_targets(targets, source=f"Run J{idx}", live=False)
        self._set_row_running(False, idx, True)
        self._set_status(f"Joint command sent (J{idx})")

    def send_motor_row(self, idx: int) -> None:
        if not self._interaction_enabled:
            return
        if idx < len(self._motor_run_states) and self._motor_run_states[idx]:
            self._hold_motor_row_at_feedback(idx)
            self._set_row_running(True, idx, False)
            self._set_status(f"Channel held at current feedback (M{idx})")
            return
        self._mark_motor_target_user_set(idx)
        self._sync_motor_abs_from_slider_before_send(idx)
        if self._send_motor_abs_from_ui(f"Run M{idx}"):
            self._set_row_running(True, idx, True)
            if self._active_scope_multiturn_unknown_mask() == 0:
                self._set_status(f"Motor absolute command sent (M{idx})")

    def send_all(self) -> None:
        if not self._interaction_enabled:
            return
        try:
            if self._manual_submode() == "motor":
                for idx in range(MOTOR_COUNT):
                    self._sync_motor_abs_from_slider_before_send(idx)
                self._send_motor_abs_from_ui("Send All")
                self._set_status(f"Sent {MOTOR_COUNT} motor absolute targets")
            else:
                if not self._guard_joint_command_started():
                    return
                self.context.send_joint_targets(self.resolved_joint_targets(), source="Send All", live=False)
                self._set_status(f"Sent {ENCODER_COUNT} joint targets")
        except Exception as exc:
            self._error("Send Failed", f"{type(exc).__name__}: {exc}")

    def apply_preset(self, angles: List[float]) -> None:
        if not self._interaction_enabled:
            return
        self._updating = True
        try:
            for i, value in enumerate(angles[:ENCODER_COUNT]):
                value = self._clamp_joint_target_deg(i, float(value))
                self.joint_target_spins[i].setValue(value)
                self.joint_sliders[i].setValue(int(round(value * 100.0)))
                self._joint_target_user_set[i] = True
        finally:
            self._updating = False
        self.context.draw_pose_angles(angles)

    def apply_home(self) -> None:
        if not self._interaction_enabled:
            self._set_status("Home requires Hand Manual ON")
            return
        self.apply_preset([0.0] * ENCODER_COUNT)
        self.send_all()

    def start_controller(self) -> None:
        if not self.context.is_comm_connected():
            if not self.context.connect_selected_port():
                self._update_connection_lights()
                return
            self._update_connection_lights()
        if not self._apply_active_scope_for_current_mode():
            self._set_status("Enable blocked: active scope was not accepted by lower controller")
            return
        state = self._state
        if state is not None and getattr(state, "system_state", None) == SYSTEM_STATE_PAUSED:
            self._estop_latched = False
            self.controller.start()
            self.context.emit_action("Resume")
            self._set_status("Resume requested: continuing paused target")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.controller.set_pid_control(False)
            self._estop_latched = False
            self.controller.start()
            self.context.emit_action(f"Enable recovery without motor target sync: unknown=0x{unknown_mask:08X}")
            self._set_status(f"Enable recovery: Turn UNKNOWN 0x{unknown_mask:08X}, direct motor commands allowed")
            return
        if not self._sync_motor_targets_to_feedback():
            self._set_status("Enable waits for fresh motor feedback")
            return
        targets = self._build_motor_abs_command_list()
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        self.controller.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        self._estop_latched = False
        self.controller.start()
        self.context.emit_action("Start with current motor feedback synced as target zero")
        self._set_status("Enable requested: waiting for IDLE controllable")

    def stop_controller(self) -> None:
        self._update_connection_lights()
        self._estop_latched = True
        self.stop_all_tests("stopped by E-STOP", update_status=False)
        try:
            self.controller.stop()
        except Exception:
            pass
        self._clear_run_states()
        self.context.emit_action("Stop")

    def pause_controller(self) -> None:
        try:
            self.controller.pause()
        except Exception:
            pass
        self.context.emit_action("Pause")
        self._set_status("Pause requested: target cache kept for resume")

    def safe_close_controller(self) -> None:
        self.stop_all_tests("safe close", update_status=False)
        try:
            self.controller.hold_current_motor_positions(delay_s=0.25)
        except Exception:
            pass
        self.context.disconnect_current_port("Safe closed: holding current motor targets, then STOP", stop_lower=True)

    def reset_controller(self) -> None:
        self._estop_latched = True
        self.controller.reset()
        self._clear_run_states()
        self.context.emit_action("Reset State")

    def _active_scope_joint_indices(self) -> List[int]:
        data = self.scope_combo.currentData() if getattr(self, "scope_combo", None) is not None else None
        if data:
            _motor_mask, joint_mask, _require_can = data
            joints = [
                idx for idx in range(ENCODER_COUNT)
                if (int(joint_mask) & (1 << idx)) != 0
            ]
            if joints:
                return joints
        return list(AUTO_CALIBRATION_JOINT_INDICES)

    def _rope_init_joint_motor_pairs(self) -> Tuple[List[Tuple[int, int, float]], List[str], List[str]]:
        state = self._state
        if state is None or not bool(getattr(state, "has_sensor_data", False)):
            return [], [], ["joint angle feedback"]
        angles = getattr(state, "angles", []) or []
        servo_online = getattr(state, "servo_online", []) or []
        servo_angles = getattr(state, "servo_angles", []) or []

        pairs: List[Tuple[int, int, float]] = []
        skipped: List[str] = []
        missing: List[str] = []
        seen_motors = set()
        for joint_idx in self._active_scope_joint_indices():
            if joint_idx < 0 or joint_idx >= ENCODER_COUNT:
                continue
            if joint_idx >= len(angles):
                missing.append(f"J{joint_idx:02d} angle")
                continue
            joint_deg = float(angles[joint_idx])
            if joint_deg >= 0.0:
                skipped.append(f"J{joint_idx:02d}={joint_deg:.2f}")
                continue
            motor_idx = TendonGuard.primary_motor_for_joint(joint_idx)
            if motor_idx < 0 or motor_idx >= MOTOR_COUNT:
                missing.append(f"J{joint_idx:02d} motor")
                continue
            if motor_idx in seen_motors:
                continue
            if motor_idx >= len(servo_online) or not bool(servo_online[motor_idx]):
                missing.append(f"M{motor_idx:02d} online")
                continue
            if motor_idx >= len(servo_angles):
                missing.append(f"M{motor_idx:02d} abs")
                continue
            seen_motors.add(motor_idx)
            pairs.append((joint_idx, motor_idx, joint_deg))
        return pairs, skipped, missing

    # =========================================================================
    # rope_init_controller - 鑷姩瀵绘壘缁崇涓庡井璋冨姛鑳?(Rope Init)
    # =========================================================================
    # 鏍稿績浣滅敤锛氬湪绯荤粺宸茬粡瀹屾垚杩囦竴娆″叏闈㈣嚜鍔ㄦ爣瀹氾紝浣嗗洜闀挎湡鏈€氱數鎴栫幆澧冨彉鍖栧鑷?    # 閮ㄥ垎宸叉爣瀹氬叧鑺傚湪涓婄數鍚庤鍙栫殑纾佺紪鐮佸櫒瑙掑害浠嶅井灏忎簬 0 搴?(濡?-0.5掳) 鏃朵娇鐢ㄣ€?    #
    # 杩囨护涓庢嫤鎴€昏緫 (_rope_init_joint_motor_pairs)锛?    # 1. 浠呴拡瀵瑰畾涔夊湪 AUTO_CALIBRATION_JOINT_INDICES 涓殑闇€瑕佹媺浼哥殑鍏宠妭銆?    # 2. 濡傛灉瑙掑害宸茬粡 >= 0.0锛屽垽瀹氫负鏃犻渶鎷夌揣锛岀洿鎺ヨ烦杩?(skipped)銆?    # 3. 濡傛灉缂哄皯纾佺紪鐮佸櫒瑙掑害銆佽埖鏈烘帀绾跨瓑锛屽垽瀹氫负缂虹姸鎬?(missing)锛屼笉浜堟墽琛屻€?    #
    # 鎵ц閫昏緫锛?    # 閽堝绛涢€夊嚭鐨勫叧鑺傚搴旂殑鑸垫満锛屽己鍒跺皢鍏跺綋鍓嶇殑缁濆浣嶇疆鍐嶁€滄鍚戞敹绱р€?ROPE_INIT_STEP_COUNTS 涓剦鍐?    # (甯镐负 +50)锛屼互閫艰揩鍏惰秺杩囨満姊版鍖恒€傝繖鏄竴绉嶅皬姝ュ揩璺戠殑杞婚噺绾ч噸鏍囧畾銆?    # =========================================================================
    def rope_init_controller(self) -> None:
        if self._manual_submode() != "joint":
            self._set_status("Rope Init is only available in Joint Channels")
            return
        if not self.context.is_comm_connected():
            if not self.context.connect_selected_port():
                return
        if not self.controller.is_started():
            self._set_status("Rope Init blocked: press ENABLE to enter IDLE first")
            return
        state = self._state
        system_state = getattr(state, "system_state", None)
        if system_state not in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY, SYSTEM_STATE_RUN_DIRECT):
            self._set_status("Rope Init blocked: wait for IDLE/READY")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._set_status(f"Rope Init blocked: Turn UNKNOWN 0x{unknown_mask:08X}; recover multi-turn first")
            return

        pairs, skipped, missing = self._rope_init_joint_motor_pairs()
        if not pairs:
            detail = "; ".join(skipped[:4]) if skipped else ", ".join(missing[:4])
            self._set_status(f"Rope Init no action: {detail or 'no calibrated joints need pull'}")
            return

        targets = self._current_motor_hold_targets_from_feedback()
        feedback_missing = []
        for _joint_idx, idx, _joint_deg in pairs:
            feedback_abs = self._motor_feedback_abs(idx)
            if feedback_abs is None:
                feedback_missing.append(f"M{idx:02d}")
                continue
            targets[idx] = clamp_motor_abs(int(feedback_abs) + ROPE_INIT_STEP_COUNTS)

        if feedback_missing:
            self._set_status(f"Rope Init blocked: waiting for feedback {','.join(feedback_missing)}")
            return

        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        self.controller.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        for _joint_idx, idx, _joint_deg in pairs:
            self._motor_session_origin_abs[idx] = int(targets[idx])
            self._motor_abs_pos[idx] = 0
            self._motor_last_slider[idx] = 0
            self._motor_target_user_set[idx] = False
            self._set_motor_target_widget(idx, 0)
        self.context.emit_action(
            f"Rope Init +{ROPE_INIT_STEP_COUNTS}: " +
            ",".join(f"J{joint_idx:02d}/M{idx:02d} {joint_deg:.2f}->{targets[idx]}" for joint_idx, idx, joint_deg in pairs)
        )
        skipped_text = f"; skipped {', '.join(skipped[:3])}" if skipped else ""
        self._set_status(f"Rope Init sent +{ROPE_INIT_STEP_COUNTS} for {len(pairs)} negative joint(s){skipped_text}")

    def calibrate(self) -> None:
        self.context.start_calibration()

    def _active_scope_abs_limit_valid_mask(self) -> int:
        state = self._state
        if state is None:
            return 0
        return self._active_motor_mask_from_ui() & int(getattr(state, "motor_abs_limit_valid_bitmap", 0))

    def set_abs_limit_lock(self, locked: bool) -> None:
        if not self.context.is_comm_connected():
            self._set_status("ABS limit lock blocked: serial port is not connected")
            return
        mask = self._active_scope_abs_limit_valid_mask()
        if mask == 0:
            self._set_status("ABS limit lock blocked: no calibrated ABS limit in current control set")
            return
        try:
            ok = self.controller.set_motor_abs_limit_lock(mask, bool(locked))
        except Exception as exc:
            self._set_status(f"ABS limit lock failed: {exc}")
            return
        action = "locked" if locked else "unlocked"
        self.context.emit_action(f"ABS limits {action}: mask=0x{mask:08X}")
        self._set_status(f"ABS limits {action}" if ok else f"ABS limits {action} ACK timeout")

    def _on_active_scope_changed(self) -> None:
        if getattr(self, "scope_combo", None) is not None:
            self._scope_selection_index = int(self.scope_combo.currentIndex())
        self._apply_active_scope_for_current_mode()

    def _active_motor_mask_from_ui(self) -> int:
        data = self.scope_combo.currentData() if getattr(self, "scope_combo", None) is not None else None
        if not data:
            return 0
        try:
            return int(data[0])
        except Exception:
            return 0

    def _active_motor_indices_from_ui(self) -> List[int]:
        mask = self._active_motor_mask_from_ui()
        return [
            idx for idx in range(MOTOR_COUNT)
            if _motor_channel_valid(idx) and (mask & (1 << idx)) != 0
        ]

    def _active_scope_label_from_ui(self) -> str:
        combo = getattr(self, "scope_combo", None)
        if combo is None:
            return ""
        try:
            return str(combo.currentText())
        except Exception:
            return ""

    def _active_joint_mask_from_ui(self) -> int:
        data = self.scope_combo.currentData() if getattr(self, "scope_combo", None) is not None else None
        if not data:
            return 0
        try:
            return int(data[1])
        except Exception:
            return 0

    def _active_scope_multiturn_unknown_mask(self) -> int:
        state = self._state
        if state is None:
            return 0
        try:
            unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0))
        except Exception:
            unknown = 0
        return unknown & self._active_motor_mask_from_ui()

    def record_mid_calibration(self) -> None:
        state = self._state
        if state is None or not bool(getattr(state, "has_servo_angle_data", False)):
            self._set_status("Mid Calib blocked: waiting for servo ABS feedback")
            return

        motor_indices = self._active_motor_indices_from_ui()
        if not motor_indices:
            self._set_status("Mid Calib blocked: active scope has no motors")
            return

        servo_abs = getattr(state, "servo_angles", []) or []
        servo_raw = getattr(state, "servo_raw_positions", []) or []
        servo_online = getattr(state, "servo_online", []) or []
        raw_online = getattr(state, "servo_raw_online", []) or []
        unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)

        records = []
        missing = []
        for motor_idx in motor_indices:
            if motor_idx >= len(servo_abs):
                missing.append(f"M{motor_idx:02d} abs")
                continue
            if motor_idx >= len(servo_online) or not bool(servo_online[motor_idx]):
                missing.append(f"M{motor_idx:02d} online")
                continue
            joint_idx = MOTOR_TO_JOINT_INDEX[motor_idx] if motor_idx < len(MOTOR_TO_JOINT_INDEX) else -1
            raw_value = None
            raw_ok = motor_idx < len(servo_raw) and motor_idx < len(raw_online) and bool(raw_online[motor_idx])
            if raw_ok:
                raw_value = int(servo_raw[motor_idx])
            records.append({
                "motor": int(motor_idx),
                "motor_label": f"M{motor_idx:02d}",
                "joint": int(joint_idx) if int(joint_idx) >= 0 else None,
                "joint_label": f"J{int(joint_idx):02d}" if int(joint_idx) >= 0 else None,
                "abs": int(servo_abs[motor_idx]),
                "raw": raw_value,
                "multi_turn_known": ((unknown_mask & (1 << motor_idx)) == 0),
            })

        if not records:
            self._set_status(f"Mid Calib blocked: {', '.join(missing[:4])}")
            return

        payload = {
            "comment": "Motor middle calibration. Record motor ABS when the tendon/rope is unwrapped; intended for later side-sway joint calibration.",
            "updated_at_host_time": time.time(),
            "hand_profile": HAND_PROFILE,
            "active_scope": {
                "label": self._active_scope_label_from_ui(),
                "motor_mask": f"0x{self._active_motor_mask_from_ui():08X}",
                "joint_mask": f"0x{self._active_joint_mask_from_ui():08X}",
            },
            "motors": records,
            "by_motor": {item["motor_label"]: item for item in records},
        }
        try:
            MOTOR_MID_CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
            MOTOR_MID_CALIB_PATH.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
        except OSError as exc:
            self._set_status(f"Mid Calib save failed: {exc}")
            return

        self.context.emit_action(
            "Mid Calib saved: " +
            ",".join(f"{item['motor_label']}={item['abs']}" for item in records)
        )
        suffix = f"; skipped {', '.join(missing[:3])}" if missing else ""
        self._set_status(f"Mid Calib saved {len(records)} motor ABS to {MOTOR_MID_CALIB_PATH.name}{suffix}")

    def _apply_active_scope_for_current_mode(self) -> bool:
        data = self.scope_combo.currentData() if getattr(self, "scope_combo", None) is not None else None
        if not data:
            return False
        motor_mask, joint_mask, require_can = data
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        if mode == "motor":
            joint_mask = 0
            require_can = False
        if not self.context.is_comm_connected():
            self._set_status("Active scope will apply after connect")
            return False
        try:
            ok = self.controller.set_active_scope(int(motor_mask), int(joint_mask), bool(require_can))
        except Exception as exc:
            self._set_status(f"Active scope failed: {exc}")
            return False
        self._set_status("Active scope applied" if ok else "Active scope ACK timeout")
        return bool(ok)

    def append_algorithm_output(self, text: str) -> None:
        if self._algorithm_view is not None:
            self._algorithm_view.append_output(text)

    def update_state(self, state) -> None:
        self._state = state
        self._update_connection_lights()
        self._check_feedback_abs_limit_violation()
        if self._app_mode != "manual":
            updater = getattr(self._body_widget, "update_state", None)
            if callable(updater):
                updater(state)
            return
        self._refresh_joint_feedback_view(sync_target=not self._joint_target_initialized)
        self._refresh_motor_feedback_view(sync_target=not self._motor_target_initialized)
        self._update_manual_action_buttons()

    def clear_live_state(self) -> None:
        self.stop_all_tests("stopped on disconnect", update_status=False)
        self._state = None
        self._update_connection_lights()
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._motor_abs_pos = [None] * MOTOR_COUNT
        self._motor_session_origin_abs = [None] * MOTOR_COUNT
        self._motor_dir_hint = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count = [0] * MOTOR_COUNT
        self._motor_last_slider = [None] * MOTOR_COUNT
        self._motor_slider_dragging = [False] * MOTOR_COUNT
        self._motor_last_sent_abs = [None] * MOTOR_COUNT
        self._joint_target_user_set = [False] * ENCODER_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        self._clear_run_states()
        self._updating = True
        try:
            for item in self.joint_current_items + self.joint_encoder_items:
                item.setText("-")
            for item in self.motor_turn_items + self.motor_raw_items + self.motor_abs_items + self.motor_limit_items + self.motor_joint_items:
                if item is not None:
                    item.setText("-")
            for i in range(ENCODER_COUNT):
                self._set_joint_encoder_dot(i, None)
        finally:
            self._updating = False

    def stop_all_tests(self, reason: str = "stopped", update_status: bool = True, include_local: bool = True) -> None:
        self._on_stop_test(reason=reason, update_status=update_status)
        self._on_stop_ff_test(reason=reason, update_status=update_status)
        self._on_stop_tri_test(reason=reason, update_status=update_status)
        if include_local:
            self._on_stop_local_test(reason=reason, update_status=update_status)

    def _guard_joint_command_started(self) -> bool:
        self.controller.set_control_mode(ControlMode.JOINT_ANGLE)
        self.controller.set_pid_control(True)
        system_state = getattr(self._state, "system_state", None) if self._state is not None else None
        if system_state not in (SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK):
            if self._estop_latched:
                self._set_status("Command blocked: press ENABLE after E-STOP")
            elif system_state == SYSTEM_STATE_IDLE:
                self._set_status("Joint command blocked: wait for READY")
            else:
                self._set_status("Joint command blocked: controller is not READY")
            return False
        if self.controller.is_started():
            return True
        if self._estop_latched:
            self._set_status("Command blocked: press ENABLE after E-STOP")
            return False
        self._set_status("Joint command blocked: press ENABLE first")
        return False

    @staticmethod
    def _slider_from_abs_pos(abs_pos: int) -> int:
        return int(abs_pos) % MOTOR_SINGLE_TURN_STEPS

    @staticmethod
    def _single_turn_diff(a: int, b: int) -> int:
        diff = abs(int(a) - int(b)) % MOTOR_SINGLE_TURN_STEPS
        return min(diff, MOTOR_SINGLE_TURN_STEPS - diff)

    def _read_motor_target_slider_int(self, idx: int) -> int:
        if 0 <= idx < len(self.motor_sliders):
            return max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, int(self.motor_sliders[idx].value())))
        if 0 <= idx < len(self.motor_target_spins):
            return self._slider_from_abs_pos(int(self.motor_target_spins[idx].value()))
        return MOTOR_TARGET_SCALE_LO

    def _session_abs_from_slider_locked_turn(self, idx: int, slider_value: int, drag_sign: int = 0) -> Optional[int]:
        if idx < 0 or idx >= MOTOR_COUNT:
            return None
        slider = max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, int(slider_value)))
        current = self._motor_abs_pos[idx]
        if current is None:
            current = 0
        hint = int(self._motor_dir_hint[idx])
        step = MOTOR_SINGLE_TURN_STEPS
        if hint > 0 or drag_sign > 0:
            k = int((int(current) - slider) // step)
            candidate = slider + k * step
            if candidate < int(current):
                candidate += step
        elif hint < 0 or drag_sign < 0:
            k = int((int(current) - slider) // step)
            candidate = slider + k * step
            if candidate > int(current):
                candidate -= step
        else:
            k = int(round((int(current) - slider) / float(step)))
            candidate = slider + k * step
        return clamp_motor_abs(candidate)

    def _motor_resync_one_channel(self, idx: int) -> None:
        dev_abs = self._motor_feedback_abs(idx)
        raw_single = self._motor_raw_single_feedback(idx)
        if dev_abs is not None and raw_single is not None and self._is_motor_abs_at_limit(dev_abs):
            dev_abs = raw_single
        if dev_abs is None:
            return
        self._motor_session_origin_abs[idx] = dev_abs
        self._motor_abs_pos[idx] = 0
        self._motor_last_slider[idx] = 0

    def _sync_motor_targets_to_feedback(self) -> bool:
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            return False
        self._refresh_motor_feedback_view(sync_target=True)
        return any(origin is not None for origin in self._motor_session_origin_abs)

    def _motor_feedback_abs(self, idx: int) -> Optional[int]:
        state = self._state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return None
        if bool(getattr(state, "has_servo_angle_data", False)):
            if idx < len(getattr(state, "servo_online", [])) and state.servo_online[idx] and idx < len(getattr(state, "servo_angles", [])):
                return clamp_motor_abs(int(state.servo_angles[idx]))
        if bool(getattr(state, "has_servo_raw_data", False)):
            if idx < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[idx] and idx < len(getattr(state, "servo_raw_positions", [])):
                return clamp_motor_abs(motor_target_from_servo_feedback(state.servo_raw_positions[idx]))
        return None

    def _motor_raw_single_feedback(self, idx: int) -> Optional[int]:
        state = self._state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return None
        if not bool(getattr(state, "has_servo_raw_data", False)):
            return None
        if idx >= len(getattr(state, "servo_raw_online", [])) or not state.servo_raw_online[idx]:
            return None
        if idx >= len(getattr(state, "servo_raw_positions", [])):
            return None
        return motor_target_from_servo_feedback(state.servo_raw_positions[idx])

    @staticmethod
    def _is_motor_abs_at_limit(value: int) -> bool:
        return int(value) >= (MOTOR_ABS_CMD_MAX - MOTOR_ABS_LIMIT_RECOVER_MARGIN) or int(value) <= (MOTOR_ABS_CMD_MIN + MOTOR_ABS_LIMIT_RECOVER_MARGIN)

    @staticmethod
    def _motor_turn_from_abs(value: int) -> int:
        raw = int(value) % MOTOR_SINGLE_TURN_STEPS
        return (int(value) - raw) // MOTOR_SINGLE_TURN_STEPS

    def _current_motor_hold_targets_from_feedback(self) -> List[int]:
        state = self._state
        targets: List[int] = []
        for i in range(MOTOR_COUNT):
            target: Optional[int] = None
            if state is not None and bool(getattr(state, "has_servo_angle_data", False)):
                if i < len(getattr(state, "servo_online", [])) and state.servo_online[i] and i < len(getattr(state, "servo_angles", [])):
                    target = clamp_motor_abs(int(state.servo_angles[i]))
            if target is None:
                raw_single = self._motor_raw_single_feedback(i)
                if raw_single is not None:
                    target = int(raw_single)
            if target is None and i < len(self._motor_last_sent_abs) and self._motor_last_sent_abs[i] is not None:
                target = clamp_motor_abs(int(self._motor_last_sent_abs[i]))
            targets.append(0 if target is None else clamp_motor_abs(int(target)))
        return targets

    def _motor_abs_for_command_packet(self, idx: int, state: Optional[HandModel], *, ignore_last_sent: bool = False) -> int:
        relative_target = self._motor_abs_pos[idx]
        origin = self._motor_session_origin_abs[idx]
        if origin is None and state is not None:
            if bool(getattr(state, "has_servo_angle_data", False)):
                if idx < len(getattr(state, "servo_online", [])) and state.servo_online[idx] and idx < len(state.servo_angles):
                    origin = clamp_motor_abs(int(state.servo_angles[idx]))
                    raw_single = self._motor_raw_single_feedback(idx)
                    if raw_single is not None and self._is_motor_abs_at_limit(origin):
                        origin = int(raw_single)
            elif bool(getattr(state, "has_servo_raw_data", False)):
                if idx < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[idx] and idx < len(state.servo_raw_positions):
                    origin = clamp_motor_abs(motor_target_from_servo_feedback(state.servo_raw_positions[idx]))
        if relative_target is None:
            relative_target = 0
            self._motor_abs_pos[idx] = relative_target
        if origin is not None:
            self._motor_session_origin_abs[idx] = origin
            return clamp_motor_abs(int(origin) + int(relative_target))
        if not ignore_last_sent and self._motor_last_sent_abs[idx] is not None:
            return clamp_motor_abs(int(self._motor_last_sent_abs[idx]))
        return 0

    def _build_motor_abs_command_list(self) -> List[int]:
        return [self._motor_abs_for_command_packet(i, self._state) for i in range(MOTOR_COUNT)]

    def _seed_motor_last_sent_after_sync(self, state: HandModel) -> None:
        for i in range(MOTOR_COUNT):
            self._motor_last_sent_abs[i] = self._motor_abs_for_command_packet(i, state, ignore_last_sent=True)

    def _motor_command_debug_summary(self, trigger: str, targets: List[int]) -> str:
        idx = self._motor_index_from_trigger(trigger)
        if idx is None:
            idx = 0
        relative = self._motor_abs_pos[idx] if 0 <= idx < len(self._motor_abs_pos) else None
        origin = self._motor_session_origin_abs[idx] if 0 <= idx < len(self._motor_session_origin_abs) else None
        sent = targets[idx] if 0 <= idx < len(targets) else None
        return f"M{idx:02d} rel={relative if relative is not None else '-'} origin={origin if origin is not None else '-'} sent={sent if sent is not None else '-'}"

    def _motor_index_from_trigger(self, trigger: str) -> Optional[int]:
        marker = "M"
        pos = str(trigger).rfind(marker)
        if pos >= 0:
            digits = []
            for ch in str(trigger)[pos + 1:]:
                if ch.isdigit():
                    digits.append(ch)
                else:
                    break
            if digits:
                return max(0, min(MOTOR_COUNT - 1, int("".join(digits))))
        return None

    def _send_motor_raw_recovery_target(self, idx: int, raw_target: int, trigger: str) -> bool:
        if idx < 0 or idx >= MOTOR_COUNT:
            return False
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self.context.emit_action(f"Raw recovery allowed with Turn UNKNOWN ({trigger}): unknown=0x{unknown_mask:08X}")
        if not self.context.is_comm_connected():
            self._set_status("Raw recovery blocked: serial port is not connected")
            return False
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        if self._estop_latched and not self.controller.is_started():
            self._set_status("Raw recovery blocked: press ENABLE after E-STOP")
            return False
        if not self.controller.is_started():
            self._set_status("Raw recovery blocked: waiting for controllable IDLE after ENABLE")
            return False
        active_mask = self._active_motor_mask_from_ui()
        if ((active_mask >> idx) & 0x1) == 0:
            self._set_status(f"Raw recovery blocked: M{idx:02d} is not in current active scope")
            return False
        targets: List[int] = [0] * MOTOR_COUNT
        missing = []
        for ch in range(MOTOR_COUNT):
            if ((active_mask >> ch) & 0x1) == 0:
                continue
            raw_feedback = self._motor_raw_single_feedback(ch)
            if raw_feedback is None:
                missing.append(f"M{ch:02d}")
                continue
            targets[ch] = int(raw_feedback) & 0x0FFF
        if missing:
            self._set_status(f"Raw recovery blocked: waiting raw feedback {','.join(missing[:4])}")
            return False

        targets[idx] = int(raw_target) & 0x0FFF
        self.controller.set_motor_positions_raw_sweep(targets)
        self._motor_last_slider[idx] = int(targets[idx])
        self._motor_abs_pos[idx] = 0
        self._motor_target_user_set[idx] = False
        self.context.emit_action(f"Motor raw recovery sent ({trigger}): M{idx:02d} raw={targets[idx]}")
        if unknown_mask != 0:
            self._set_status(f"Raw recovery sent while Turn UNKNOWN: M{idx:02d} raw={targets[idx]}")
        else:
            self._set_status(f"Raw recovery sent: M{idx:02d} raw={targets[idx]}")
        return True

    def _send_motor_abs_from_ui(self, trigger: str) -> bool:
        if not self._interaction_enabled:
            return False
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        if self._estop_latched and not self.controller.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return False
        if not self.controller.is_started():
            self._set_status("Motor command blocked: waiting for controllable IDLE after ENABLE")
            return False
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return False
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self.context.emit_action(f"Motor command allowed with Turn UNKNOWN ({trigger}): unknown=0x{unknown_mask:08X}")
        for idx in range(MOTOR_COUNT):
            if self._motor_session_origin_abs[idx] is None:
                self._motor_resync_one_channel(idx)
            if self._motor_abs_pos[idx] is None:
                self._motor_abs_pos[idx] = 0
        targets = self._build_motor_abs_command_list()
        targets, limit_hits = self._clamp_motor_targets_to_calibrated_limits(targets)
        if limit_hits:
            self._show_motor_abs_limit_warning(limit_hits)
        self.controller.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        self.context.emit_action(f"Motor absolute positions sent ({trigger}): {self._motor_command_debug_summary(trigger, targets)}")
        if unknown_mask != 0:
            self._set_status(f"Motor command sent while Turn UNKNOWN 0x{unknown_mask:08X}")
        return True

    def _clamp_motor_targets_to_calibrated_limits(self, targets: List[int]) -> Tuple[List[int], List[Tuple[int, int, int, int]]]:
        state = self._state
        if state is None:
            return targets, []
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0))
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", valid_bitmap))
        active_mask = self._active_motor_mask_from_ui()
        mins = getattr(state, "motor_abs_limit_min", [])
        maxs = getattr(state, "motor_abs_limit_max", [])
        clamped = list(targets)
        hits: List[Tuple[int, int, int, int]] = []
        for idx in range(min(MOTOR_COUNT, len(clamped), len(mins), len(maxs))):
            if ((active_mask >> idx) & 0x01) == 0:
                continue
            if ((valid_bitmap >> idx) & 0x01) == 0:
                continue
            if ((locked_bitmap >> idx) & 0x01) == 0:
                continue
            lo = int(mins[idx])
            hi = int(maxs[idx])
            if lo > hi:
                lo, hi = hi, lo
            requested = int(clamped[idx])
            limited = max(lo, min(hi, requested))
            if limited != requested:
                clamped[idx] = limited
                hits.append((idx, requested, lo, hi))
        return clamped, hits

    def _show_motor_abs_limit_warning(self, hits: List[Tuple[int, int, int, int]]) -> None:
        now = time.time()
        if now - self._last_abs_limit_warning_ts < 1.5:
            return
        self._last_abs_limit_warning_ts = now
        preview = ", ".join(
            f"M{idx:02d}: {requested}->{max(lo, min(hi, requested))} [{lo},{hi}]"
            for idx, requested, lo, hi in hits[:4]
        )
        suffix = "" if len(hits) <= 4 else f" ... +{len(hits) - 4}"
        self._set_status(f"Motor ABS limited: {preview}{suffix}")
        QtWidgets.QMessageBox.warning(
            self,
            "Motor ABS Limit",
            "Direct motor target is outside the calibrated ABS window and was clamped.\n"
            f"{preview}{suffix}",
        )

    def _check_feedback_abs_limit_violation(self) -> None:
        state = self._state
        if state is None or getattr(state, "system_state", None) != SYSTEM_STATE_RUN_FEEDBACK:
            return
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0))
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", valid_bitmap))
        monitor_mask = self._active_motor_mask_from_ui()
        if monitor_mask == 0:
            monitor_mask = valid_bitmap
        monitor_mask &= valid_bitmap & locked_bitmap
        if monitor_mask == 0:
            return
        mins = getattr(state, "motor_abs_limit_min", [])
        maxs = getattr(state, "motor_abs_limit_max", [])
        hits: List[Tuple[int, int, int, int]] = []
        for idx in range(min(MOTOR_COUNT, len(mins), len(maxs))):
            if ((monitor_mask >> idx) & 0x01) == 0:
                continue
            feedback_abs = self._motor_feedback_abs(idx)
            if feedback_abs is None:
                continue
            lo = int(mins[idx])
            hi = int(maxs[idx])
            if lo > hi:
                lo, hi = hi, lo
            if feedback_abs < lo or feedback_abs > hi:
                hits.append((idx, int(feedback_abs), lo, hi))
        if hits:
            self._show_feedback_abs_limit_violation(hits)

    def _show_feedback_abs_limit_violation(self, hits: List[Tuple[int, int, int, int]]) -> None:
        now = time.time()
        if now - self._last_feedback_abs_limit_warning_ts < 5.0:
            return
        self._last_feedback_abs_limit_warning_ts = now
        preview = ", ".join(
            f"M{idx:02d}: abs={feedback_abs} [{lo},{hi}]"
            for idx, feedback_abs, lo, hi in hits[:4]
        )
        suffix = "" if len(hits) <= 4 else f" ... +{len(hits) - 4}"
        self._set_status(f"Feedback ABS out of limit: {preview}{suffix}")
        QtWidgets.QMessageBox.warning(
            self,
            "Feedback ABS Limit",
            "RUN_FEEDBACK servo feedback ABS is outside the calibrated locked window.\n"
            f"{preview}{suffix}",
        )

    def _send_motor_sweep_from_ui(self, trigger: str) -> None:
        if not self._interaction_enabled:
            return
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        if self._estop_latched and not self.controller.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return
        if not self.controller.is_started():
            self._set_status("Motor command blocked: waiting for controllable IDLE after ENABLE")
            return
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return
        targets = self.motor_sweep_targets()
        self.controller.set_motor_positions_raw_sweep(targets)
        self.context.emit_action(f"Motor sweep positions sent ({trigger})")

    def _sync_motor_abs_from_slider_before_send(self, idx: int) -> None:
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        cur = self._read_motor_target_slider_int(idx)
        mapped = self._session_abs_from_slider_locked_turn(idx, cur, 0)
        if mapped is not None:
            self._motor_abs_pos[idx] = mapped
            self._motor_last_slider[idx] = cur

    def _on_motor_slider_press(self, idx: int) -> None:
        if 0 <= idx < MOTOR_COUNT:
            self._motor_slider_dragging[idx] = True

    def _on_motor_slider_release(self, idx: int) -> None:
        if 0 <= idx < MOTOR_COUNT:
            self._motor_slider_dragging[idx] = False

    def _mark_joint_target_user_set(self, idx: int) -> None:
        if 0 <= idx < len(self._joint_target_user_set):
            self._joint_target_user_set[idx] = True

    def _mark_motor_target_user_set(self, idx: int) -> None:
        if 0 <= idx < len(self._motor_target_user_set):
            self._motor_target_user_set[idx] = True

    def _joint_target_max_deg(self, idx: int) -> float:
        return float(JOINT_TARGET_MAX_DEG_BY_INDEX[idx] if 0 <= idx < len(JOINT_TARGET_MAX_DEG_BY_INDEX) else JOINT_TARGET_MAX_DEG_BY_INDEX[-1])

    def _joint_target_min_deg(self, idx: int) -> float:
        return float(JOINT_TARGET_MIN_DEG_BY_INDEX[idx] if 0 <= idx < len(JOINT_TARGET_MIN_DEG_BY_INDEX) else JOINT_TARGET_MIN_DEG_BY_INDEX[-1])

    def _clamp_joint_target_deg(self, idx: int, value: float) -> float:
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = self._joint_target_min_deg(idx)
        if not np.isfinite(value):
            value = self._joint_target_min_deg(idx)
        lo = self._joint_target_min_deg(idx)
        hi = self._joint_target_max_deg(idx)
        return max(lo, min(hi, value))

    def _joint_feedback_target(self, idx: int) -> float:
        state = self._state
        if state is not None and bool(getattr(state, "has_sensor_data", False)) and idx < len(getattr(state, "angles", [])):
            return self._clamp_joint_target_deg(idx, float(state.angles[idx]))
        if idx < len(self.joint_target_spins):
            return self._clamp_joint_target_deg(idx, float(self.joint_target_spins[idx].value()))
        return self._joint_target_min_deg(idx)

    def _motor_feedback_target(self, idx: int) -> int:
        state = self._state
        if state is not None:
            if bool(getattr(state, "has_servo_raw_data", False)) and idx < len(getattr(state, "servo_raw_positions", [])):
                return motor_target_from_servo_feedback(state.servo_raw_positions[idx])
            if bool(getattr(state, "has_servo_angle_data", False)) and idx < len(getattr(state, "servo_angles", [])):
                return self._slider_from_abs_pos(int(state.servo_angles[idx]))
        return self._read_motor_target_slider_int(idx)

    def _on_joint_tie_field_commit(self, joint_idx: int) -> None:
        if joint_idx < 0 or joint_idx >= ENCODER_COUNT:
            return
        sign_text = self.joint_tie_sign_edits[joint_idx].text().strip()
        sign = TendonGuard.parse_sign_text(sign_text)
        if sign > 0:
            self.joint_tie_sign_edits[joint_idx].setText("+")
        elif sign < 0:
            self.joint_tie_sign_edits[joint_idx].setText("-")
        else:
            self.joint_tie_sign_edits[joint_idx].setText("")
        try:
            x1 = int(float(self.joint_tie_x1_edits[joint_idx].text()))
        except (TypeError, ValueError):
            x1 = 0
        x1 = clamp_motor_abs(x1)
        self.joint_tie_x1_edits[joint_idx].setText(str(x1))
        saved = self.context.tendon_guard.set_joint(joint_idx, sign != 0, 1 if sign >= 0 else -1, x1)
        self.context.tendon_guard.sync_to_controller(self.controller, force=True)
        if saved:
            if sign == 0:
                self._set_status(f"tie saved: J{joint_idx} disabled, x1={x1}")
            else:
                self._set_status(f"tie saved: J{joint_idx} sign={'+' if sign > 0 else '-'} x1={x1}")
        else:
            self._set_status(f"tie save failed: J{joint_idx}")

    def _refresh_tendon_guard_from_ui(self) -> None:
        config = []
        for idx in range(min(ENCODER_COUNT, len(self.joint_tie_sign_edits), len(self.joint_tie_x1_edits))):
            sign = TendonGuard.parse_sign_text(self.joint_tie_sign_edits[idx].text())
            try:
                x1 = int(float(self.joint_tie_x1_edits[idx].text()))
            except (TypeError, ValueError):
                x1 = 0
            config.append({"enabled": sign != 0, "pull_sign": 1 if sign >= 0 else -1, "x1_abs": clamp_motor_abs(x1)})
        if len(config) == ENCODER_COUNT:
            self.context.tendon_guard.config = config
        self.context.tendon_guard.sync_to_controller(self.controller, force=False)

    def _refresh_joint_feedback_view(self, sync_target: bool = False) -> None:
        state = self._state
        joint_has_data = bool(state is not None and getattr(state, "has_sensor_data", False))
        self._updating = True
        try:
            for i in range(ENCODER_COUNT):
                if joint_has_data and i < len(getattr(state, "angles", [])):
                    self.joint_current_items[i].setText(f"{float(state.angles[i]):.2f}")
                    if i < len(getattr(state, "encoders", [])):
                        self.joint_encoder_items[i].setText(str(getattr(state.encoders[i], "raw_sensor", getattr(state.encoders[i], "raw", "-"))))
                        self._set_joint_encoder_dot(i, not bool(getattr(state.encoders[i], "error", False)))
                    else:
                        self.joint_encoder_items[i].setText("-")
                        self._set_joint_encoder_dot(i, None)
                else:
                    self.joint_current_items[i].setText("-")
                    self.joint_encoder_items[i].setText("-")
                    self._set_joint_encoder_dot(i, None)
            if joint_has_data and sync_target:
                for i in range(min(ENCODER_COUNT, len(getattr(state, "angles", [])))):
                    target = self._clamp_joint_target_deg(i, float(state.angles[i]))
                    self.joint_target_spins[i].setValue(target)
                    self.joint_sliders[i].setValue(int(round(target * 100.0)))
                    self._joint_target_user_set[i] = False
                self._joint_target_initialized = True
        finally:
            self._updating = False

    def _refresh_motor_feedback_view(self, sync_target: bool = False) -> None:
        state = self._state
        if state is None:
            return
        motor_has_abs = bool(getattr(state, "has_servo_angle_data", False))
        motor_has_raw = bool(getattr(state, "has_servo_raw_data", False))
        self._updating = True
        try:
            for i in range(MOTOR_COUNT):
                dev_abs = self._motor_feedback_abs(i)
                if i < len(self.motor_turn_items):
                    self.motor_turn_items[i].setText(str(self._motor_turn_from_abs(dev_abs)) if dev_abs is not None else "-")
                raw_single = self._motor_raw_single_feedback(i)
                if i < len(self.motor_raw_items):
                    self.motor_raw_items[i].setText(f"{raw_single:4d}" if raw_single is not None else "-")
                if i < len(self.motor_abs_items):
                    self.motor_abs_items[i].setText(str(int(dev_abs)) if dev_abs is not None else "-")
                if i < len(self.motor_limit_items) and self.motor_limit_items[i] is not None:
                    limit_text = "-"
                    valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0))
                    locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", valid_bitmap))
                    if ((valid_bitmap >> i) & 0x01) != 0:
                        mins = getattr(state, "motor_abs_limit_min", [])
                        maxs = getattr(state, "motor_abs_limit_max", [])
                        if i < len(mins) and i < len(maxs):
                            limit_text = f"{int(mins[i])}..{int(maxs[i])}"
                            if ((locked_bitmap >> i) & 0x01) == 0:
                                limit_text = f"UNLOCK {limit_text}"
                    self.motor_limit_items[i].setText(limit_text)
                joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
                if 0 <= joint_idx < len(getattr(state, "angles", [])):
                    self.motor_joint_items[i].setText(f"{float(state.angles[joint_idx]):.2f}")
                elif i < len(self.motor_joint_items):
                    self.motor_joint_items[i].setText("-")
            if (motor_has_raw or motor_has_abs) and sync_target:
                for i in range(MOTOR_COUNT):
                    dev_abs = self._motor_feedback_abs(i)
                    if dev_abs is None:
                        continue
                    raw_single = self._motor_raw_single_feedback(i)
                    if raw_single is not None and self._is_motor_abs_at_limit(dev_abs):
                        dev_abs = int(raw_single)
                    self._motor_session_origin_abs[i] = dev_abs
                    self._motor_abs_pos[i] = 0
                    self.motor_target_spins[i].setValue(0)
                    self.motor_sliders[i].setValue(0)
                    self._motor_last_slider[i] = 0
                    self._motor_target_user_set[i] = False
                self._motor_target_initialized = True
                self._seed_motor_last_sent_after_sync(state)
        finally:
            self._updating = False
        for i in range(MOTOR_COUNT):
            cur_single = self._motor_feedback_target(i)
            tgt_single = self._read_motor_target_slider_int(i)
            if self._single_turn_diff(cur_single, tgt_single) <= MOTOR_DIR_HINT_CLEAR_TOL:
                if self._motor_dir_hint_pending_clear[i]:
                    self._motor_dir_hint_stable_count[i] += 1
                if self._motor_dir_hint_stable_count[i] >= MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES:
                    self._motor_dir_hint[i] = 0
                    self._motor_dir_hint_pending_clear[i] = False
                    self._motor_dir_hint_stable_count[i] = 0

    def _set_joint_encoder_dot(self, joint_index: int, is_connected: Optional[bool]) -> None:
        if joint_index < 0 or joint_index >= len(self.joint_status_items):
            return
        item = self.joint_status_items[joint_index]
        if is_connected is True:
            item.setForeground(QtGui.QBrush(QtGui.QColor("#19a55a")))
        elif is_connected is False:
            item.setForeground(QtGui.QBrush(QtGui.QColor("#d84343")))
        else:
            item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))

    def _clear_run_states(self) -> None:
        self._joint_run_states = [False] * ENCODER_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT
        for btn in self.joint_send_buttons + self.motor_send_buttons:
            btn.setText("Send")

    def _set_row_running(self, is_motor: bool, idx: int, running: bool) -> None:
        states = self._motor_run_states if is_motor else self._joint_run_states
        buttons = self.motor_send_buttons if is_motor else self.joint_send_buttons
        if 0 <= idx < len(states):
            states[idx] = bool(running)
        if 0 <= idx < len(buttons):
            buttons[idx].setText("Hold" if running else "Send")

    def _hold_joint_row_at_feedback(self, idx: int) -> None:
        if not self._guard_joint_command_started():
            return
        self._updating = True
        try:
            target = self._joint_feedback_target(idx)
            self.joint_target_spins[idx].setValue(target)
            self.joint_sliders[idx].setValue(int(round(target * 100.0)))
            self._joint_target_user_set[idx] = False
        finally:
            self._updating = False
        self.context.send_joint_targets(self.resolved_joint_targets(), source=f"Hold J{idx}", live=False)

    def _hold_motor_row_at_feedback(self, idx: int) -> None:
        dev_abs = self._motor_feedback_abs(idx)
        if dev_abs is not None:
            self._motor_session_origin_abs[idx] = dev_abs
            self._motor_abs_pos[idx] = 0
            self._set_motor_target_widget(idx, 0)
            self._motor_last_slider[idx] = 0
        else:
            self._motor_resync_one_channel(idx)
            self._set_motor_target_widget(idx, 0)
        self._motor_target_user_set[idx] = False
        self._send_motor_abs_from_ui(f"Hold M{idx}")

    def _set_motor_target_widget(self, idx: int, value: int) -> None:
        self._updating = True
        try:
            self.motor_target_spins[idx].setValue(int(value))
            self.motor_sliders[idx].setValue(self._slider_from_abs_pos(int(value)))
        finally:
            self._updating = False

    def _set_motor_relative_target_widget(self, idx: int, relative_target: int) -> None:
        self._updating = True
        try:
            target = clamp_motor_abs(int(relative_target))
            self.motor_target_spins[idx].setValue(target)
            self.motor_sliders[idx].setValue(self._slider_from_abs_pos(target))
        finally:
            self._updating = False

    def _restore_motor_slider_to_relative_target(self, idx: int) -> None:
        if idx < 0 or idx >= len(self.motor_sliders):
            return
        current = self._motor_abs_pos[idx]
        if current is None:
            current = 0
        self._motor_last_slider[idx] = self._slider_from_abs_pos(int(current))
        self._set_motor_relative_target_widget(idx, int(current))

    def _on_joint_slider_change(self, idx: int, value: float) -> None:
        self._mark_joint_target_user_set(idx)
        if not self._guard_joint_command_started():
            return
        try:
            targets = self.resolved_joint_targets()
            self.context.send_joint_targets(targets, source=f"Slider J{idx}", live=True)
        except Exception as exc:
            self._set_status(f"Joint slider send failed: {exc}")

    def _on_motor_slider_change(self, idx: int, value: int) -> None:
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Slider ignored: waiting for motor feedback")
            return
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        new_single = int(value)
        last = self._motor_last_slider[idx]
        drag_sign = 0
        if last is not None:
            diff = new_single - int(last)
            last_single = int(last)
            if (
                last_single >= MOTOR_TARGET_SCALE_HI - MOTOR_SLIDER_WRAP_MARGIN
                and new_single <= MOTOR_TARGET_SCALE_LO + MOTOR_SLIDER_WRAP_MARGIN
            ):
                drag_sign = 1
            elif (
                last_single <= MOTOR_TARGET_SCALE_LO + MOTOR_SLIDER_WRAP_MARGIN
                and new_single >= MOTOR_TARGET_SCALE_HI - MOTOR_SLIDER_WRAP_MARGIN
            ):
                drag_sign = -1
            elif abs(diff) > MOTOR_JUMP_RESYNC_THRESH:
                self._restore_motor_slider_to_relative_target(idx)
                self._set_status(
                    f"Motor slider jump blocked: M{idx:02d} slider {last_single} -> {new_single}"
                )
                return
            else:
                drag_sign = 1 if diff > 0 else (-1 if diff < 0 else 0)
        mapped = self._session_abs_from_slider_locked_turn(idx, new_single, drag_sign)
        if mapped is None:
            return
        current_target = int(self._motor_abs_pos[idx] or 0)
        if abs(int(mapped) - current_target) > MOTOR_SLIDER_MAX_TARGET_DELTA:
            self._restore_motor_slider_to_relative_target(idx)
            self._set_status(
                f"Motor slider jump blocked: M{idx:02d} {current_target} -> {int(mapped)}"
            )
            return
        if last is not None and self._single_turn_diff(new_single, int(last)) > MOTOR_JUMP_RESYNC_THRESH:
            if self._motor_slider_dragging[idx]:
                self._restore_motor_slider_to_relative_target(idx)
                return
            self._motor_resync_one_channel(idx)
            return
        self._motor_abs_pos[idx] = mapped
        self._motor_last_slider[idx] = new_single
        self._mark_motor_target_user_set(idx)
        self._set_motor_relative_target_widget(idx, mapped)
        try:
            self._send_motor_abs_from_ui(f"Slider M{idx}")
        except Exception as exc:
            self._set_status(f"Motor slider send failed: {exc}")

    def _step_motor_target(self, idx: int, delta: int) -> None:
        if not self._interaction_enabled:
            return
        step_dir = 1 if int(delta) > 0 else (-1 if int(delta) < 0 else 0)
        if step_dir == 0:
            return
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        cur_sl = self._read_motor_target_slider_int(idx)
        if step_dir > 0:
            new_sl = MOTOR_TARGET_SCALE_LO if cur_sl >= MOTOR_TARGET_SCALE_HI else cur_sl + 1
        elif step_dir < 0:
            new_sl = MOTOR_TARGET_SCALE_HI if cur_sl <= MOTOR_TARGET_SCALE_LO else cur_sl - 1
        self._motor_abs_pos[idx] = clamp_motor_abs(int(self._motor_abs_pos[idx]) + step_dir)
        self._motor_dir_hint[idx] = step_dir
        self._motor_dir_hint_pending_clear[idx] = True
        self._motor_dir_hint_stable_count[idx] = 0
        self._motor_last_slider[idx] = new_sl
        self._mark_motor_target_user_set(idx)
        self._set_motor_relative_target_widget(idx, self._motor_abs_pos[idx])
        self._send_motor_abs_from_ui(f"{'Step+' if delta > 0 else 'Step-'} M{idx}")

    def _normalize_test_joint_value(self, text: str) -> str:
        text = str(text or "").strip()
        if text.lower() == "none" or not text:
            return "None"
        if text.upper().startswith("J"):
            text = text[1:]
        try:
            idx = int(text)
        except ValueError:
            return "None"
        return f"J{idx:02d}" if 0 <= idx < ENCODER_COUNT else "None"

    def _joint_index_from_combo(self, combo: QtWidgets.QComboBox) -> Optional[int]:
        text = self._normalize_test_joint_value(combo.currentText())
        combo.setCurrentText(text)
        if text == "None":
            return None
        return int(text[1:])

    def _on_test_joint_changed(self, row_idx: int) -> None:
        current = self._normalize_test_joint_value(self.test_joint_combos[row_idx].currentText())
        self.test_joint_combos[row_idx].setCurrentText(current)
        if current == "None":
            return
        for i, combo in enumerate(self.test_joint_combos):
            if i != row_idx and self._normalize_test_joint_value(combo.currentText()) == current:
                self.test_joint_combos[row_idx].setCurrentText("None")
                self._set_status(f"Test config duplicate joint: {current} already selected")
                return

    def _validate_test_rows(self, show_message: bool = True) -> Optional[List[Tuple[int, float, float, float, float]]]:
        active = []
        seen = set()
        for row in range(TEST_ROW_COUNT):
            joint_idx = self._joint_index_from_combo(self.test_joint_combos[row])
            if joint_idx is None:
                continue
            if joint_idx in seen:
                if show_message:
                    self._warn("Test Config", f"Test row {row + 1}: duplicated J{joint_idx:02d}")
                return None
            seen.add(joint_idx)
            lo = self._clamp_joint_target_deg(joint_idx, self.test_min_spins[row].value())
            hi = self._clamp_joint_target_deg(joint_idx, self.test_max_spins[row].value())
            if hi < lo:
                lo, hi = hi, lo
            freq = float(self.test_freq_spins[row].value())
            phase = float(self.test_phase_spins[row].value())
            if not (np.isfinite(freq) and freq > 0.0 and np.isfinite(phase) and 0.0 <= phase <= 360.0):
                if show_message:
                    self._warn("Test Config", f"Test row {row + 1}: invalid frequency or phase")
                return None
            active.append((joint_idx, lo, hi, freq, phase))
        if not active and show_message:
            self._warn("Test Config", "Select at least one valid joint (not None).")
        return active or None

    def _set_test_button_states(self) -> None:
        any_running = self._test_running or self._ff_test_running or self._tri_test_running or self._local_test_running
        manual = bool(self._interaction_enabled)
        for btn, running in [
            (getattr(self, "test_stop_btn", None), self._test_running),
            (getattr(self, "ff_stop_btn", None), self._ff_test_running),
            (getattr(self, "tri_stop_btn", None), self._tri_test_running),
            (getattr(self, "local_stop_btn", None), self._local_test_running),
        ]:
            if btn is not None:
                try:
                    btn.setEnabled(running)
                except RuntimeError:
                    pass
        for btn, running in [
            (getattr(self, "test_start_btn", None), self._test_running),
            (getattr(self, "ff_start_btn", None), self._ff_test_running),
            (getattr(self, "tri_start_btn", None), self._tri_test_running),
            (getattr(self, "local_start_btn", None), self._local_test_running),
        ]:
            if btn is not None:
                try:
                    btn.setEnabled(manual and not any_running and not running)
                except RuntimeError:
                    pass

    def _start_test_recording(self, reason: str) -> bool:
        try:
            self.context.start_recording_for_test(reason)
            return True
        except Exception as exc:
            self._error("Start Test Failed", f"Cannot start recording: {exc}")
            return False

    def _on_start_test(self) -> None:
        if not self._interaction_enabled:
            self._set_status("Start Test blocked: Hand Manual must be ON")
            return
        if self._manual_submode() != "test" and hasattr(self, "tabs"):
            self.tabs.setCurrentWidget(self.test_panel)
        active = self._validate_test_rows(True)
        if not active:
            return
        self.stop_all_tests("restarted by sine test", update_status=False)
        if not self._guard_joint_command_started():
            return
        if not self._start_test_recording("test"):
            return
        self._test_active_rows = active
        self._test_hold_s = max(0.0, float(self.test_hold_spin.value()))
        self._test_running = True
        self._test_t0 = time.time()
        self._set_test_button_states()
        self.context.emit_action("Test started")
        self._test_tick()
        self._test_timer.start()

    def _on_stop_test(self, reason: str = "stopped by test", update_status: bool = True) -> None:
        was_running = self._test_running
        self._test_running = False
        self._test_timer.stop()
        self._test_active_rows = []
        self._set_test_button_states()
        if was_running:
            self.context.stop_recording_for_test(reason)
            if update_status:
                self.context.emit_action(f"Test stopped ({reason})")

    def _parse_ff_targets_text(self, raw: str) -> Optional[List[float]]:
        targets = []
        for token in str(raw or "").replace(",", " ").split():
            try:
                targets.append(float(token))
            except ValueError:
                continue
        return targets or None

    def _validate_ff_test_config(self, show_message: bool = True) -> Optional[Tuple[int, List[float], float]]:
        joint_idx = self._joint_index_from_combo(self.ff_joint_combo)
        if joint_idx is None:
            if show_message:
                self._warn("Feedforward Test", "Select one valid joint.")
            return None
        targets = self._parse_ff_targets_text(self.ff_targets_edit.text())
        if not targets:
            if show_message:
                self._warn("Feedforward Test", "Targets(deg) must contain at least one valid number.")
            return None
        clamped = [self._clamp_joint_target_deg(joint_idx, value) for value in targets]
        step_s = float(self.ff_step_spin.value())
        if not np.isfinite(step_s) or step_s <= 0:
            if show_message:
                self._warn("Feedforward Test", "Step Time(s) must be finite and > 0.")
            return None
        self.ff_targets_edit.setText(", ".join(f"{v:.2f}" for v in clamped))
        return joint_idx, clamped, step_s

    def _on_start_ff_test(self) -> None:
        if not self._interaction_enabled:
            self._set_status("Feedforward Test blocked: Hand Manual must be ON")
            return
        if self._manual_submode() != "test" and hasattr(self, "tabs"):
            self.tabs.setCurrentWidget(self.test_panel)
        validated = self._validate_ff_test_config(True)
        if validated is None:
            return
        self.stop_all_tests("restarted by feedforward test", update_status=False)
        if not self._guard_joint_command_started():
            return
        if not self._start_test_recording("feedforward test"):
            return
        self._ff_test_joint_idx, self._ff_test_targets, self._ff_test_step_s = validated
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = time.time()
        self._ff_test_running = True
        self._set_test_button_states()
        self.context.emit_action(f"Feedforward Test started on J{self._ff_test_joint_idx:02d}")
        self._ff_test_tick()
        self._ff_test_timer.start()

    def _on_stop_ff_test(self, reason: str = "stopped by feedforward test", update_status: bool = True) -> None:
        was_running = self._ff_test_running
        self._ff_test_running = False
        self._ff_test_timer.stop()
        self._ff_test_targets = []
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = 0.0
        self._set_test_button_states()
        if was_running:
            self.context.stop_recording_for_test(reason)
            if update_status:
                self.context.emit_action(f"Feedforward Test stopped ({reason})")

    def _validate_tri_test_config(self, show_message: bool = True) -> Optional[Tuple[int, float, float, float]]:
        joint_idx = self._joint_index_from_combo(self.tri_joint_combo)
        if joint_idx is None:
            if show_message:
                self._warn("Triangle Wave Test", "Select one valid joint.")
            return None
        lo = self._clamp_joint_target_deg(joint_idx, self.tri_min_spin.value())
        hi = self._clamp_joint_target_deg(joint_idx, self.tri_max_spin.value())
        if hi < lo:
            lo, hi = hi, lo
        freq = float(self.tri_freq_spin.value())
        if not np.isfinite(freq) or freq <= 0:
            if show_message:
                self._warn("Triangle Wave Test", "Frequency must be finite and > 0.")
            return None
        return joint_idx, lo, hi, freq

    def _on_start_tri_test(self) -> None:
        if not self._interaction_enabled:
            self._set_status("Triangle Wave Test blocked: Hand Manual must be ON")
            return
        if self._manual_submode() != "test" and hasattr(self, "tabs"):
            self.tabs.setCurrentWidget(self.test_panel)
        validated = self._validate_tri_test_config(True)
        if validated is None:
            return
        self.stop_all_tests("restarted by triangle test", update_status=False)
        if not self._guard_joint_command_started():
            return
        if not self._start_test_recording("triangle test"):
            return
        self._tri_test_joint_idx, self._tri_test_min_deg, self._tri_test_max_deg, self._tri_test_freq_hz = validated
        self._tri_test_t0 = time.time()
        self._tri_test_running = True
        self._set_test_button_states()
        self.context.emit_action(f"Triangle Wave Test started on J{self._tri_test_joint_idx:02d}")
        self._tri_test_tick()
        self._tri_test_timer.start()

    def _on_stop_tri_test(self, reason: str = "stopped by triangle test", update_status: bool = True) -> None:
        was_running = self._tri_test_running
        self._tri_test_running = False
        self._tri_test_timer.stop()
        self._tri_test_t0 = 0.0
        self._set_test_button_states()
        if was_running:
            self.context.stop_recording_for_test(reason)
            if update_status:
                self.context.emit_action(f"Triangle Wave Test stopped ({reason})")

    def _on_start_local_test(self) -> None:
        if not self._interaction_enabled:
            self._set_status("Local Test blocked: Hand Manual must be ON")
            return
        self.stop_all_tests("restarted by local test", update_status=False)
        if not self.context.is_comm_connected():
            self._set_status("Local Test blocked: serial port is not connected")
            return
        if not self._start_test_recording("local test"):
            return
        try:
            self.controller.set_local_test_params(float(self.local_freq.value()), float(self.local_amp.value()))
            self.controller.start_local_test()
        except Exception as exc:
            self.context.stop_recording_for_test("local test failed")
            self._error("Start Local Test Failed", f"{type(exc).__name__}: {exc}")
            return
        self._local_test_running = True
        self._set_test_button_states()
        self.context.emit_action(
            f"Local Test started (freq={float(self.local_freq.value()):.4g} Hz, amp={float(self.local_amp.value()):.4g} deg)"
        )

    def _on_stop_local_test(self, reason: str = "stopped by local test", update_status: bool = True) -> None:
        was_running = self._local_test_running
        self._local_test_running = False
        if was_running:
            try:
                self.controller.stop_local_test()
            except Exception:
                pass
            self.context.stop_recording_for_test(reason)
            if update_status:
                self.context.emit_action(f"Local Test stopped ({reason})")
        self._set_test_button_states()

    def _test_tick(self) -> None:
        if not self._test_running:
            return
        if self._manual_submode() != "test":
            self._on_stop_test("stopped on mode switch", update_status=False)
            return
        if not self._interaction_enabled:
            self._on_stop_test("stopped: Hand Manual OFF")
            return
        if not self.controller.is_started():
            self._on_stop_test("stopped: press Enable to start controller")
            return
        elapsed = max(0.0, time.time() - self._test_t0)
        angles = self.resolved_joint_targets()
        for joint_idx, lo, hi, freq, phase_deg in self._test_active_rows:
            center = 0.5 * (lo + hi)
            amp = 0.5 * (hi - lo)
            phase = math.radians(phase_deg)
            if elapsed < self._test_hold_s:
                target = center + amp * math.sin(phase)
            else:
                target = center + amp * math.sin(2.0 * math.pi * freq * (elapsed - self._test_hold_s) + phase)
            angles[joint_idx] = self._clamp_joint_target_deg(joint_idx, target)
            self._set_joint_target_widget(joint_idx, angles[joint_idx])
            self._joint_target_user_set[joint_idx] = True
        try:
            self.context.send_joint_targets(angles, source="Test", live=True)
        except Exception as exc:
            self._on_stop_test(f"stopped by send error: {exc}")

    def _ff_test_tick(self) -> None:
        if not self._ff_test_running:
            return
        if self._manual_submode() != "test":
            self._on_stop_ff_test("stopped on mode switch", update_status=False)
            return
        if not self._interaction_enabled:
            self._on_stop_ff_test("stopped: Hand Manual OFF")
            return
        if not self.controller.is_started():
            self._on_stop_ff_test("stopped: press Enable to start controller")
            return
        if not self._ff_test_targets:
            self._on_stop_ff_test("stopped: no feedforward targets")
            return
        now = time.time()
        dt = 0.0 if self._ff_test_last_ts <= 0.0 else max(0.0, now - self._ff_test_last_ts)
        self._ff_test_last_ts = now
        self._ff_test_t_accum += dt
        while self._ff_test_t_accum >= self._ff_test_step_s and self._ff_test_idx < len(self._ff_test_targets) - 1:
            self._ff_test_t_accum -= self._ff_test_step_s
            self._ff_test_idx += 1
        target = self._clamp_joint_target_deg(self._ff_test_joint_idx, self._ff_test_targets[self._ff_test_idx])
        angles = self.resolved_joint_targets()
        angles[self._ff_test_joint_idx] = target
        self._set_joint_target_widget(self._ff_test_joint_idx, target)
        self._joint_target_user_set[self._ff_test_joint_idx] = True
        try:
            self.context.send_joint_targets(angles, source="Feedforward Test", live=True)
        except Exception as exc:
            self._on_stop_ff_test(f"stopped by send error: {exc}")

    def _tri_test_tick(self) -> None:
        if not self._tri_test_running:
            return
        if self._manual_submode() != "test":
            self._on_stop_tri_test("stopped on mode switch", update_status=False)
            return
        if not self._interaction_enabled:
            self._on_stop_tri_test("stopped: Hand Manual OFF")
            return
        if not self.controller.is_started():
            self._on_stop_tri_test("stopped: press Enable to start controller")
            return
        elapsed = max(0.0, time.time() - self._tri_test_t0)
        phase = (elapsed * self._tri_test_freq_hz) % 1.0
        tri = 4.0 * abs(phase - 0.5) - 1.0
        center = 0.5 * (self._tri_test_min_deg + self._tri_test_max_deg)
        amp = 0.5 * (self._tri_test_max_deg - self._tri_test_min_deg)
        target = self._clamp_joint_target_deg(self._tri_test_joint_idx, center + amp * tri)
        angles = self.resolved_joint_targets()
        angles[self._tri_test_joint_idx] = target
        self._set_joint_target_widget(self._tri_test_joint_idx, target)
        self._joint_target_user_set[self._tri_test_joint_idx] = True
        try:
            self.context.send_joint_targets(angles, source="Triangle Wave Test", live=True)
        except Exception as exc:
            self._on_stop_tri_test(f"stopped by send error: {exc}")

    def _set_joint_target_widget(self, idx: int, value: float) -> None:
        self._updating = True
        try:
            self.joint_target_spins[idx].setValue(float(value))
            self.joint_sliders[idx].setValue(int(round(float(value) * 100.0)))
        finally:
            self._updating = False
