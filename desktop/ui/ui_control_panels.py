import json
import math
import re
import time
from pathlib import Path
from typing import List, Optional, Tuple

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
    motor_abs_after_tendon_action,
    motor_tendon_shorten_abs_sign,
    motor_target_from_servo_feedback,
)
from .ui_teleop_view import TeleopView
from .ui_algorithm_view import AlgorithmView


MOTOR_TARGET_SCALE_LO = 0
MOTOR_TARGET_SCALE_HI = 4095
MOTOR_SINGLE_TURN_STEPS = 4096
MOTOR_RELEASE_HALF_TURN_COUNTS = MOTOR_SINGLE_TURN_STEPS // 2
MOTOR_JUMP_RESYNC_THRESH = 2048
MOTOR_SLIDER_WRAP_MARGIN = 512
MOTOR_SLIDER_MAX_TARGET_DELTA = MOTOR_SINGLE_TURN_STEPS
MOTOR_DIR_HINT_CLEAR_TOL = 3
MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES = 4
MOTOR_ABS_LIMIT_RECOVER_MARGIN = 16
ROPE_INIT_TIGHTEN_STEP_COUNTS = 60
ROPE_INIT_RELEASE_STEP_COUNTS = 30
ROPE_INIT_STEP_COUNTS = ROPE_INIT_TIGHTEN_STEP_COUNTS
ROPE_INIT_TIMER_MS = 50
ROPE_INIT_TIMEOUT_S = 8.0
ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG = 0.0
MOTOR_MID_CALIB_PATH = Path(__file__).resolve().parents[1] / "config" / "motor_mid_calib.json"
CALIB_LOAD_SNAPSHOT_PATH = Path(__file__).resolve().parents[1] / "config" / "calib_load_snapshot.json"
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

FINGER_JOINT_ORDER = {
    "Thumb J16-J20": [19, 18, 17, 16, 20],
}


def _joint_indices_for_group(group_label: str, joint_start: int, joint_end: int) -> List[int]:
    custom = FINGER_JOINT_ORDER.get(group_label)
    if custom:
        return [idx for idx in custom if joint_start <= idx <= joint_end]
    return list(range(joint_start, joint_end + 1))


def _joint_group_mask(group_label: str, joint_start: int, joint_end: int) -> int:
    mask = 0
    for joint_idx in _joint_indices_for_group(group_label, joint_start, joint_end):
        mask |= (1 << joint_idx)
    return mask


def _joint_rows_by_finger_group() -> Tuple[List[int], List[Tuple[int, int, str]]]:
    rows: List[int] = []
    group_ranges: List[Tuple[int, int, str]] = []
    for group_label, joint_start, joint_end in FINGER_JOINT_RANGES:
        group_start_row = len(rows)
        rows.extend(_joint_indices_for_group(group_label, joint_start, joint_end))
        if len(rows) > group_start_row:
            group_ranges.append((group_start_row, len(rows) - 1, group_label))
    return rows, group_ranges


def _motor_rows_by_finger_group() -> Tuple[List[int], List[Tuple[int, int, str]]]:
    rows: List[int] = []
    group_ranges: List[Tuple[int, int, str]] = []
    used = set()

    for group_label, joint_start, joint_end in FINGER_JOINT_RANGES:
        group_start_row = len(rows)
        for joint_idx in _joint_indices_for_group(group_label, joint_start, joint_end):
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


def _motors_by_joint_index() -> List[List[int]]:
    motors_by_joint = [[] for _ in range(ENCODER_COUNT)]
    for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
        joint_idx = int(joint_idx)
        if _motor_channel_valid(motor_idx) and 0 <= joint_idx < ENCODER_COUNT:
            motors_by_joint[joint_idx].append(int(motor_idx))
    return motors_by_joint


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
        self._motors_by_joint = _motors_by_joint_index()
        self._last_calib_status: Optional[str] = None
        self._last_calib_snapshot_signature: Optional[str] = None
        default_motor_mask, default_joint_mask, _default_require_can = self._scope_range(0, 3, True)
        self._selected_joint_mask = int(default_joint_mask)
        self._selected_motor_mask = int(default_motor_mask)
        self._teleop_selected_joint_mask = int(default_joint_mask)
        self._teleop_selected_motor_mask = int(default_motor_mask)
        self._scope_update_guard = False
        self.joint_use_checks: List[Optional[QtWidgets.QCheckBox]] = [None] * ENCODER_COUNT
        self.joint_group_use_checks: List[QtWidgets.QCheckBox] = []
        self.teleop_joint_use_checks: List[Optional[QtWidgets.QCheckBox]] = [None] * ENCODER_COUNT
        self.teleop_group_use_checks: List[QtWidgets.QCheckBox] = []
        self.teleop_hand_use_check: Optional[QtWidgets.QCheckBox] = None

        self.motor_turn_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_raw_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_abs_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_load_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_limit_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_joint_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_target_spins: List[QtWidgets.QSpinBox] = []
        self.motor_sliders: List[QtWidgets.QSlider] = []
        self.motor_send_buttons: List[QtWidgets.QPushButton] = []
        self.motor_recover_buttons: List[QtWidgets.QPushButton] = []
        self._motor_target_user_set: List[bool] = []
        self._motor_target_initialized = False
        self.motor_use_checks: List[Optional[QtWidgets.QCheckBox]] = [None] * MOTOR_COUNT
        self.motor_group_use_checks: List[QtWidgets.QCheckBox] = []
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
        self._last_state_transition_count: Optional[int] = None
        self._last_multi_turn_unknown_bitmap: Optional[int] = None
        self._last_abs_limit_warning_ts = 0.0
        self._last_feedback_abs_limit_warning_ts = 0.0

        self._test_running = False
        self._ff_test_running = False
        self._tri_test_running = False
        self._test_timer = QtCore.QTimer(self)
        self._test_timer.setInterval(TEST_TICK_MS)
        self._test_timer.timeout.connect(self._test_tick)
        self._ff_test_timer = QtCore.QTimer(self)
        self._ff_test_timer.setInterval(TEST_TICK_MS)
        self._ff_test_timer.timeout.connect(self._ff_test_tick)
        self._tri_test_timer = QtCore.QTimer(self)
        self._tri_test_timer.setInterval(TEST_TICK_MS)
        self._tri_test_timer.timeout.connect(self._tri_test_tick)
        self._rope_init_timer = QtCore.QTimer(self)
        self._rope_init_timer.setInterval(ROPE_INIT_TIMER_MS)
        self._rope_init_timer.timeout.connect(self._rope_init_tick)
        self._rope_init_tasks: List[dict] = []
        self._rope_init_started_at = 0.0
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
        dev_layout.setHorizontalSpacing(8)
        dev_layout.setVerticalSpacing(6)
        self.enable_btn = QtWidgets.QPushButton("ENABLE")
        self.stop_btn = QtWidgets.QPushButton("E-STOP")
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.reset_btn = QtWidgets.QPushButton("Clear Fault")
        self.reset_btn.setToolTip("Clear latched protection/fault flags, clear command state, and return firmware to UNCONTROLLABLE")
        self.calib_btn = QtWidgets.QPushButton("Calibrate")
        self.rope_init_btn = QtWidgets.QPushButton("Rope Init")
        self.rope_init_btn.setToolTip(
            "Single motors pull into the workspace; dual motors move antagonistically to 40 deg"
        )
        self.abs_limit_unlock_btn = QtWidgets.QPushButton("Unlock ABS")
        self.abs_limit_unlock_btn.setToolTip("Temporarily disable calibrated ABS travel limits for the current control set")
        self.abs_limit_lock_btn = QtWidgets.QPushButton("Lock ABS")
        self.abs_limit_lock_btn.setToolTip("Re-enable calibrated ABS travel limits for the current control set")
        self.nvs_control_btn = QtWidgets.QPushButton("Flash ON")
        self.nvs_control_btn.setToolTip("Toggle multi-turn persistence. OFF clears lower flash snapshots and desktop restore cache.")
        self.enable_btn.setMinimumHeight(46)
        self.stop_btn.setMinimumHeight(46)
        self.pause_btn.setMinimumHeight(46)
        for btn in (
            self.calib_btn,
            self.reset_btn,
            self.rope_init_btn,
            self.abs_limit_unlock_btn,
            self.abs_limit_lock_btn,
            self.nvs_control_btn,
        ):
            btn.setMinimumHeight(34)
        self.enable_btn.setStyleSheet("font-weight:700;background:#2e7d32;color:white;")
        self.stop_btn.setStyleSheet("font-weight:700;background:#c62828;color:white;")
        self.pause_btn.setStyleSheet("font-weight:700;background:#f9a825;color:black;")
        self.nvs_control_btn.setStyleSheet("font-weight:700;background:#e8f5e9;color:#1b5e20;")

        def centered_action_cell(button: QtWidgets.QPushButton) -> QtWidgets.QWidget:
            cell = QtWidgets.QWidget()
            cell_layout = QtWidgets.QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.addStretch(1)
            cell_layout.addWidget(button)
            cell_layout.addStretch(1)
            return cell

        dev_layout.addWidget(centered_action_cell(self.enable_btn), 0, 0, 2, 1)
        dev_layout.addWidget(centered_action_cell(self.stop_btn), 0, 1, 2, 1)
        dev_layout.addWidget(centered_action_cell(self.pause_btn), 0, 2, 2, 1)
        dev_layout.addWidget(self.calib_btn, 0, 3)
        dev_layout.addWidget(self.reset_btn, 0, 4)
        dev_layout.addWidget(self.abs_limit_unlock_btn, 0, 5)
        dev_layout.addWidget(self.rope_init_btn, 1, 3)
        dev_layout.addWidget(self.nvs_control_btn, 1, 4)
        dev_layout.addWidget(self.abs_limit_lock_btn, 1, 5)
        dev_layout.setRowStretch(0, 1)
        dev_layout.setRowStretch(1, 1)
        for col in range(6):
            dev_layout.setColumnStretch(col, 1)
        layout.addWidget(device)
        self._update_connection_lights()

        self.enable_btn.clicked.connect(lambda _checked=False: self.start_controller())
        self.stop_btn.clicked.connect(lambda _checked=False: self.stop_controller())
        self.pause_btn.clicked.connect(lambda _checked=False: self.pause_controller())
        self.reset_btn.clicked.connect(lambda _checked=False: self.clear_fault_with_confirm())
        self.calib_btn.clicked.connect(lambda _checked=False: self.calibrate())
        self.rope_init_btn.clicked.connect(lambda _checked=False: self.rope_init_controller())
        self.abs_limit_unlock_btn.clicked.connect(lambda _checked=False: self.set_abs_limit_lock(False))
        self.abs_limit_lock_btn.clicked.connect(lambda _checked=False: self.set_abs_limit_lock(True))
        self.nvs_control_btn.clicked.connect(lambda _checked=False: self.toggle_multiturn_nvs())
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

    @staticmethod
    def _motor_mask_for_joint_mask(joint_mask: int) -> int:
        mask = 0
        for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
            joint_idx = int(joint_idx)
            if _motor_channel_valid(motor_idx) and 0 <= joint_idx < ENCODER_COUNT and (int(joint_mask) & (1 << joint_idx)) != 0:
                mask |= (1 << motor_idx)
        return mask

    @staticmethod
    def _centered_cell_widget(widget: QtWidgets.QWidget) -> QtWidgets.QWidget:
        cell = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(cell)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addStretch(1)
        layout.addWidget(widget)
        layout.addStretch(1)
        return cell

    def _make_use_checkbox_cell(
        self,
        checked: bool,
        tooltip: str,
    ) -> Tuple[QtWidgets.QWidget, QtWidgets.QCheckBox]:
        check = QtWidgets.QCheckBox()
        check.setChecked(bool(checked))
        check.setToolTip(tooltip)
        return self._centered_cell_widget(check), check

    def _make_group_use_widget(
        self,
        text: str,
        checked: bool,
        tooltip: str,
    ) -> Tuple[QtWidgets.QWidget, QtWidgets.QCheckBox]:
        check = QtWidgets.QCheckBox(text)
        check.setChecked(bool(checked))
        check.setToolTip(tooltip)
        font = check.font()
        font.setBold(True)
        check.setFont(font)
        return self._centered_cell_widget(check), check

    def _paint_group_cell_widget(self, widget: QtWidgets.QWidget, color: QtGui.QColor) -> None:
        name = color.name()
        widget.setAutoFillBackground(True)
        palette = widget.palette()
        palette.setColor(QtGui.QPalette.Window, color)
        palette.setColor(QtGui.QPalette.Base, color)
        widget.setPalette(palette)
        widget.setStyleSheet(f"background-color: {name};")

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
        self.calib_load_btn = None
        self.open_btn = None
        self.release_half_turn_btn = None
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
        self.test_joint_combos = []
        self.test_min_spins = []
        self.test_max_spins = []
        self.test_freq_spins = []
        self.test_phase_spins = []
        self.finger_test_group_combo = None
        self.finger_test_same_check = None
        self.finger_test_same_spin = None
        self.finger_test_spins = []
        self.finger_test_current_items = []
        self.finger_test_error_items = []
        self.finger_test_apply_btn = None
        self.finger_test_sync_btn = None
        self.joint_current_items = []
        self.joint_encoder_items = []
        self.joint_status_items = []
        self.joint_target_spins = []
        self.joint_sliders = []
        self.joint_send_buttons = []
        self.joint_tie_sign_edits = []
        self.joint_tie_x1_edits = []
        self.joint_use_checks = [None] * ENCODER_COUNT
        self.joint_group_use_checks = []
        self.teleop_joint_use_checks = [None] * ENCODER_COUNT
        self.teleop_group_use_checks = []
        self.teleop_hand_use_check = None
        self._joint_target_user_set = []
        self.motor_turn_items = []
        self.motor_raw_items = []
        self.motor_abs_items = []
        self.motor_load_items = []
        self.motor_limit_items = []
        self.motor_joint_items = []
        self.motor_target_spins = []
        self.motor_sliders = []
        self.motor_send_buttons = []
        self.motor_recover_buttons = []
        self.motor_use_checks = [None] * MOTOR_COUNT
        self.motor_group_use_checks = []
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

        workflow = QtWidgets.QGroupBox("Manual Workflow")
        wf_layout = QtWidgets.QHBoxLayout(workflow)
        self.manual_check = QtWidgets.QCheckBox("Hand Manual")
        self.manual_check.setChecked(False)
        self.send_all_btn = QtWidgets.QPushButton("Send All")
        self.calib_load_btn = QtWidgets.QPushButton("Calib Load")
        self.calib_load_btn.setToolTip("Load the last saved calibration snapshot when mechanics have not changed")
        self.open_btn = QtWidgets.QPushButton("Open Pose")
        self.release_half_turn_btn = QtWidgets.QPushButton("Release Half Turn")
        self.fist_btn = QtWidgets.QPushButton("Fist Pose")
        self.zero_btn = QtWidgets.QPushButton("Zero Pose")
        wf_layout.addWidget(self.manual_check)
        wf_layout.addStretch(1)
        for btn in [self.calib_load_btn, self.open_btn, self.release_half_turn_btn, self.fist_btn, self.zero_btn, self.send_all_btn]:
            wf_layout.addWidget(btn)
        layout.addWidget(workflow)

        self.manual_check.toggled.connect(self._apply_interaction_enabled)
        self.send_all_btn.clicked.connect(lambda _checked=False: self.send_all())
        self.calib_load_btn.clicked.connect(lambda _checked=False: self.load_calibration_snapshot())
        self.open_btn.clicked.connect(lambda _checked=False: self.apply_preset(_preset_20rad_to_21deg(_OPEN_20_RAD)))
        self.release_half_turn_btn.clicked.connect(lambda _checked=False: self.release_all_motors_half_turn())
        self.fist_btn.clicked.connect(lambda _checked=False: self.apply_preset(_preset_20rad_to_21deg(_FIST_20_RAD)))
        self.zero_btn.clicked.connect(lambda _checked=False: self.apply_preset([0.0] * ENCODER_COUNT))

        self.tabs = QtWidgets.QTabWidget()
        self.joint_table = self._build_joint_table()
        self.motor_table = self._build_motor_table()
        self._sync_scope_group_checks()
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

        workflow = self._build_teleop_scope_workflow()
        layout.addWidget(workflow)

        teleop_view = TeleopView(
            self.context,
            selected_joint_mask_getter=self._teleop_selected_joint_mask_value,
        )
        body.child = teleop_view
        layout.addWidget(teleop_view, 1)

        self.body_layout.addWidget(body)
        self._body_widget = body

    def _build_teleop_scope_workflow(self) -> QtWidgets.QGroupBox:
        workflow = QtWidgets.QGroupBox("Teleop Workflow")
        row_layout = QtWidgets.QHBoxLayout(workflow)
        row_layout.setContentsMargins(8, 4, 8, 4)
        row_layout.setSpacing(10)
        self._teleop_selected_motor_mask = self._motor_mask_for_joint_mask(self._teleop_selected_joint_mask)

        all_joint_mask = (1 << ENCODER_COUNT) - 1
        hand_check = QtWidgets.QCheckBox("Hand")
        hand_check.setToolTip("Select all teleop joints")
        hand_check.setChecked((self._teleop_selected_joint_mask & all_joint_mask) == all_joint_mask)
        hand_check.toggled.connect(lambda checked: self._on_teleop_hand_use_changed(checked))
        hand_check.setMinimumWidth(64)
        self.teleop_hand_use_check = hand_check
        row_layout.addWidget(hand_check, 0)

        separator = QtWidgets.QFrame()
        separator.setFrameShape(QtWidgets.QFrame.VLine)
        separator.setFrameShadow(QtWidgets.QFrame.Sunken)
        row_layout.addWidget(separator)

        for label, start, end in FINGER_JOINT_RANGES:
            group_mask = _joint_group_mask(label, start, end)
            column = QtWidgets.QWidget()
            col_layout = QtWidgets.QVBoxLayout(column)
            col_layout.setContentsMargins(0, 0, 0, 0)
            col_layout.setSpacing(2)
            group_check = QtWidgets.QCheckBox(label)
            group_check.setToolTip("Select this finger group for Teleop active scope")
            group_check.setChecked((self._teleop_selected_joint_mask & group_mask) == group_mask)
            group_check.toggled.connect(
                lambda checked, s=start, e=end: self._on_teleop_joint_group_use_changed(s, e, checked)
            )
            self.teleop_group_use_checks.append(group_check)
            col_layout.addWidget(group_check)

            joint_row = QtWidgets.QHBoxLayout()
            joint_row.setContentsMargins(14, 0, 0, 0)
            joint_row.setSpacing(4)
            for joint_idx in _joint_indices_for_group(label, start, end):
                use_check = QtWidgets.QCheckBox(f"J{joint_idx:02d}")
                use_check.setToolTip(f"Use J{joint_idx:02d} in Teleop active scope")
                use_check.setChecked((self._teleop_selected_joint_mask & (1 << joint_idx)) != 0)
                use_check.toggled.connect(
                    lambda checked, idx=joint_idx: self._on_teleop_joint_use_changed(idx, checked)
                )
                self.teleop_joint_use_checks[joint_idx] = use_check
                joint_row.addWidget(use_check)
            col_layout.addLayout(joint_row)
            row_layout.addWidget(column, 1)

        row_layout.addStretch(1)
        self._sync_teleop_group_checks()
        return workflow

    def _make_item(self, text: str = "-") -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        return item

    def _build_joint_table(self) -> QtWidgets.QTableWidget:
        joint_rows, group_ranges = _joint_rows_by_finger_group()
        table = QtWidgets.QTableWidget(len(joint_rows), 11)
        table.setHorizontalHeaderLabels([
            "Group", "Use", "Joint", "Enc", "Current", "Raw", "Min", "Target", "Max", "Action",
            "Target Slider",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(False)
        self.joint_current_items = [None] * ENCODER_COUNT
        self.joint_encoder_items = [None] * ENCODER_COUNT
        self.joint_status_items = [None] * ENCODER_COUNT
        self.joint_target_spins = [None] * ENCODER_COUNT
        self.joint_sliders = [None] * ENCODER_COUNT
        self.joint_send_buttons = [None] * ENCODER_COUNT
        self._joint_target_user_set = [False] * ENCODER_COUNT
        group_colors = [QtGui.QColor("#ffffff"), QtGui.QColor("#eeeeee")]
        row_to_group_color = {}
        group_start_rows = set()
        for group_index, (row_start, row_end, label) in enumerate(group_ranges):
            target = next((item for item in FINGER_JOINT_RANGES if item[0] == label), None)
            if target is None:
                continue
            _label, start, end = target
            color = group_colors[group_index % len(group_colors)]
            group_start_rows.add(row_start)
            group_mask = _joint_group_mask(label, start, end)
            group_widget, group_check = self._make_group_use_widget(
                label,
                (self._selected_joint_mask & group_mask) == group_mask,
                "Select this finger group for Manual Joint Channels active scope",
            )
            self._paint_group_cell_widget(group_widget, color)
            group_check.toggled.connect(
                lambda checked, s=start, e=end: self._on_joint_group_use_changed(s, e, checked)
            )
            self.joint_group_use_checks.append(group_check)
            table.setCellWidget(row_start, 0, group_widget)
            if row_end > row_start:
                table.setSpan(row_start, 0, row_end - row_start + 1, 1)
            for row in range(row_start, row_end + 1):
                row_to_group_color[row] = color

        def apply_group_background(item: QtWidgets.QTableWidgetItem, row: int) -> None:
            color = row_to_group_color.get(row)
            if color is not None:
                item.setBackground(QtGui.QBrush(color))

        def apply_group_widget_background(widget: QtWidgets.QWidget, row: int) -> None:
            color = row_to_group_color.get(row)
            if color is None:
                return
            name = color.name()
            widget.setAutoFillBackground(True)
            palette = widget.palette()
            palette.setColor(QtGui.QPalette.Window, color)
            palette.setColor(QtGui.QPalette.Base, color)
            widget.setPalette(palette)
            widget.setStyleSheet(f"background-color: {name};")

        for row, i in enumerate(joint_rows):
            if row not in group_start_rows:
                filler = self._make_item("")
                apply_group_background(filler, row)
                table.setItem(row, 0, filler)
            joint_item = self._make_item(f"J{i:02d}")
            apply_group_background(joint_item, row)
            table.setItem(row, 2, joint_item)
            use_cell, use_check = self._make_use_checkbox_cell(
                (self._selected_joint_mask & (1 << i)) != 0,
                f"Use J{i:02d} in Manual Joint Channels active scope",
            )
            apply_group_widget_background(use_cell, row)
            use_check.toggled.connect(lambda checked, idx=i: self._on_joint_use_changed(idx, checked))
            self.joint_use_checks[i] = use_check
            table.setCellWidget(row, 1, use_cell)
            status = self._make_item("*")
            status.setTextAlignment(QtCore.Qt.AlignCenter)
            status.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
            enc = self._make_item("-")
            cur = self._make_item("-")
            min_item = self._make_item(f"{self._joint_target_min_deg(i):.2f}")
            max_item = self._make_item(f"{self._joint_target_max_deg(i):.2f}")
            for item in (status, enc, cur, min_item, max_item):
                apply_group_background(item, row)
            self.joint_status_items[i] = status
            self.joint_current_items[i] = cur
            self.joint_encoder_items[i] = enc
            table.setItem(row, 3, status)
            table.setItem(row, 4, cur)
            table.setItem(row, 5, enc)
            table.setItem(row, 6, min_item)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(self._joint_target_min_deg(i), self._joint_target_max_deg(i))
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=i: self._joint_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: self._mark_joint_target_user_set(idx))
            apply_group_widget_background(spin, row)
            self.joint_target_spins[i] = spin
            table.setCellWidget(row, 7, spin)
            table.setItem(row, 8, max_item)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_joint_row(idx))
            apply_group_widget_background(send, row)
            self.joint_send_buttons[i] = send
            table.setCellWidget(row, 9, send)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(self._joint_target_min_deg(i) * 100), int(self._joint_target_max_deg(i) * 100))
            slider.valueChanged.connect(lambda value, idx=i: self._joint_slider_changed(idx, value))
            apply_group_widget_background(slider, row)
            self.joint_sliders[i] = slider
            table.setCellWidget(row, 10, slider)
            self._control_widgets.extend([spin, send, slider])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(0, 152)
        table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(1, 44)
        table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(2, 58)
        table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(3, 32)
        table.horizontalHeader().setSectionResizeMode(4, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(4, 70)
        table.horizontalHeader().setSectionResizeMode(5, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(5, 68)
        table.horizontalHeader().setSectionResizeMode(6, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(6, 64)
        table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(7, 78)
        table.horizontalHeader().setSectionResizeMode(8, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(8, 60)
        table.horizontalHeader().setSectionResizeMode(9, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(9, 82)
        table.horizontalHeader().setSectionResizeMode(10, QtWidgets.QHeaderView.Stretch)
        table.setColumnWidth(10, 420)
        return table

    def _build_motor_table(self) -> QtWidgets.QTableWidget:
        motor_rows, group_ranges = _motor_rows_by_finger_group()
        table = QtWidgets.QTableWidget(len(motor_rows), 15)
        table.setHorizontalHeaderLabels([
            "Group", "Use", "Motor", "Turn", "RawNow", "AbsNow", "Load", "Limit", "Target(rel)", "Action", "Raw",
            "-", "Raw Slider", "+", "Joint Angle",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(False)
        self.motor_turn_items = [None] * MOTOR_COUNT
        self.motor_raw_items = [None] * MOTOR_COUNT
        self.motor_abs_items = [None] * MOTOR_COUNT
        self.motor_load_items = [None] * MOTOR_COUNT
        self.motor_limit_items = [None] * MOTOR_COUNT
        self.motor_joint_items = [None] * MOTOR_COUNT
        self.motor_target_spins = [None] * MOTOR_COUNT
        self.motor_sliders = [None] * MOTOR_COUNT
        self.motor_send_buttons = [None] * MOTOR_COUNT
        self.motor_recover_buttons = [None] * MOTOR_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        group_colors = [QtGui.QColor("#ffffff"), QtGui.QColor("#eeeeee")]
        row_to_group_label = {}
        row_to_group_color = {}
        group_start_rows = set()
        for group_index, (start, end, label) in enumerate(group_ranges):
            group_start_rows.add(start)
            color = group_colors[group_index % len(group_colors)]
            group_mask = 0
            for row in range(start, end + 1):
                if 0 <= row < len(motor_rows):
                    motor_idx = int(motor_rows[row])
                    if _motor_channel_valid(motor_idx):
                        group_mask |= (1 << motor_idx)
            group_widget, group_check = self._make_group_use_widget(
                label,
                group_mask != 0 and (self._selected_motor_mask & group_mask) == group_mask,
                "Select this finger group for Manual Motor Channels active scope",
            )
            self._paint_group_cell_widget(group_widget, color)
            group_check.toggled.connect(
                lambda checked, s=start, e=end, rows=motor_rows: self._on_motor_group_use_changed(rows[s:e + 1], checked)
            )
            self.motor_group_use_checks.append(group_check)
            table.setCellWidget(start, 0, group_widget)
            if end > start:
                table.setSpan(start, 0, end - start + 1, 1)
            for row in range(start, end + 1):
                row_to_group_label[row] = label
                row_to_group_color[row] = color

        def apply_group_background(item: QtWidgets.QTableWidgetItem, row: int) -> None:
            color = row_to_group_color.get(row)
            if color is not None:
                item.setBackground(QtGui.QBrush(color))

        def apply_group_widget_background(widget: QtWidgets.QWidget, row: int) -> None:
            color = row_to_group_color.get(row)
            if color is None:
                return
            name = color.name()
            widget.setAutoFillBackground(True)
            palette = widget.palette()
            palette.setColor(QtGui.QPalette.Window, color)
            palette.setColor(QtGui.QPalette.Base, color)
            widget.setPalette(palette)
            widget.setStyleSheet(f"background-color: {name};")

        for row, i in enumerate(motor_rows):
            group_label = row_to_group_label.get(row, "")
            if row not in group_start_rows and group_label:
                filler = self._make_item("")
                apply_group_background(filler, row)
                table.setItem(row, 0, filler)
            motor_item = self._make_item(f"M{i:02d}")
            apply_group_background(motor_item, row)
            joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
            if group_label:
                motor_item.setToolTip(f"{group_label}; mapped joint J{int(joint_idx):02d}" if int(joint_idx) >= 0 else group_label)
            table.setItem(row, 2, motor_item)
            use_cell, use_check = self._make_use_checkbox_cell(
                (self._selected_motor_mask & (1 << i)) != 0,
                f"Use M{i:02d} in Manual Motor Channels active scope",
            )
            apply_group_widget_background(use_cell, row)
            use_check.toggled.connect(lambda checked, idx=i: self._on_motor_use_changed(idx, checked))
            self.motor_use_checks[i] = use_check
            table.setCellWidget(row, 1, use_cell)
            turn = self._make_item("-")
            raw_now = self._make_item("-")
            abs_now = self._make_item("-")
            load = self._make_item("-")
            limit = self._make_item("-")
            joint = self._make_item("-")
            for item in (turn, raw_now, abs_now, load, limit, joint):
                apply_group_background(item, row)
            self.motor_turn_items[i] = turn
            self.motor_raw_items[i] = raw_now
            self.motor_abs_items[i] = abs_now
            self.motor_load_items[i] = load
            self.motor_limit_items[i] = limit
            self.motor_joint_items[i] = joint
            table.setItem(row, 3, turn)
            table.setItem(row, 4, raw_now)
            table.setItem(row, 5, abs_now)
            table.setItem(row, 6, load)
            table.setItem(row, 7, limit)
            spin = QtWidgets.QSpinBox()
            spin.setRange(MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX)
            spin.setSingleStep(10)
            spin.valueChanged.connect(lambda value, idx=i: self._motor_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: self._mark_motor_target_user_set(idx))
            apply_group_widget_background(spin, row)
            self.motor_target_spins[i] = spin
            table.setCellWidget(row, 8, spin)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_motor_row(idx))
            apply_group_widget_background(send, row)
            self.motor_send_buttons[i] = send
            table.setCellWidget(row, 9, send)
            recover = QtWidgets.QPushButton("Raw")
            recover.clicked.connect(lambda _=False, idx=i: self.recover_motor_row_to_raw(idx))
            recover.setToolTip("Recover this motor from +/-30719 using its current single-turn raw feedback")
            apply_group_widget_background(recover, row)
            self.motor_recover_buttons[i] = recover
            table.setCellWidget(row, 10, recover)
            minus = QtWidgets.QPushButton("-")
            minus.setMaximumWidth(34)
            minus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, -1))
            apply_group_widget_background(minus, row)
            table.setCellWidget(row, 11, minus)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimumWidth(72)
            slider.setRange(MOTOR_TARGET_SCALE_LO, MOTOR_TARGET_SCALE_HI)
            slider.sliderPressed.connect(lambda idx=i: self._on_motor_slider_press(idx))
            slider.sliderReleased.connect(lambda idx=i: self._on_motor_slider_release(idx))
            slider.valueChanged.connect(lambda value, idx=i: self._motor_slider_changed(idx, value))
            apply_group_widget_background(slider, row)
            self.motor_sliders[i] = slider
            table.setCellWidget(row, 12, slider)
            plus = QtWidgets.QPushButton("+")
            plus.setMaximumWidth(34)
            plus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, 1))
            apply_group_widget_background(plus, row)
            table.setCellWidget(row, 13, plus)
            table.setItem(row, 14, joint)
            self._control_widgets.extend([spin, send, recover, minus, slider, plus])
            if not _motor_channel_valid(i):
                table.setRowHidden(row, True)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(0, 152)
        table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(1, 44)
        table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(2, 58)
        table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(7, 126)
        table.horizontalHeader().setSectionResizeMode(12, QtWidgets.QHeaderView.Stretch)
        table.setColumnWidth(12, 150)
        table.horizontalHeader().setSectionResizeMode(14, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(14, 82)
        return table

    def _build_test_panel(self) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(box)
        layout.addWidget(self._build_finger_static_test_box())
        layout.addWidget(self._build_sine_test_box())
        layout.addWidget(self._build_feedforward_test_box())
        layout.addWidget(self._build_triangle_test_box())
        layout.addStretch(1)
        return box

    def _joint_choices(self) -> List[str]:
        return ["None"] + [f"J{i:02d}" for i in range(ENCODER_COUNT)]

    def _build_finger_static_test_box(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Finger Static Q Test")
        layout = QtWidgets.QGridLayout(group)
        self.finger_test_group_combo = QtWidgets.QComboBox()
        for label, joint_start, joint_end in FINGER_JOINT_RANGES:
            self.finger_test_group_combo.addItem(label, (joint_start, joint_end))
        self.finger_test_group_combo.currentIndexChanged.connect(
            lambda _idx=0: self._refresh_finger_static_test_controls()
        )
        self.finger_test_same_check = QtWidgets.QCheckBox("Same angle")
        self.finger_test_same_check.setChecked(True)
        self.finger_test_same_check.stateChanged.connect(
            lambda _state=0: self._refresh_finger_static_test_controls()
        )
        self.finger_test_same_spin = self._make_float_spin(0.0, 120.0, 30.0, 2)
        self.finger_test_same_spin.valueChanged.connect(
            lambda value: self._on_finger_static_same_angle_changed(float(value))
        )
        self.finger_test_apply_btn = QtWidgets.QPushButton("Apply Finger Q")
        self.finger_test_sync_btn = QtWidgets.QPushButton("Use Current")
        self.finger_test_apply_btn.clicked.connect(lambda _checked=False: self._on_apply_finger_static_test())
        self.finger_test_sync_btn.clicked.connect(lambda _checked=False: self._sync_finger_static_test_from_current())

        layout.addWidget(QtWidgets.QLabel("Finger"), 0, 0)
        layout.addWidget(self.finger_test_group_combo, 0, 1)
        layout.addWidget(self.finger_test_same_check, 0, 2)
        layout.addWidget(QtWidgets.QLabel("Angle(deg)"), 0, 3)
        layout.addWidget(self.finger_test_same_spin, 0, 4)
        layout.addWidget(self.finger_test_apply_btn, 0, 5)
        layout.addWidget(self.finger_test_sync_btn, 0, 6)
        for col, text in enumerate(["Joint", "Target(deg)", "Current(deg)", "Err(deg)"]):
            layout.addWidget(QtWidgets.QLabel(text), 1, col)

        self.finger_test_spins = []
        self.finger_test_current_items = []
        self.finger_test_error_items = []
        for row in range(5):
            joint_label = QtWidgets.QLabel("-")
            spin = self._make_float_spin(0.0, 120.0, 30.0, 2)
            current = QtWidgets.QLabel("-")
            error = QtWidgets.QLabel("-")
            layout.addWidget(joint_label, row + 2, 0)
            layout.addWidget(spin, row + 2, 1)
            layout.addWidget(current, row + 2, 2)
            layout.addWidget(error, row + 2, 3)
            spin.valueChanged.connect(lambda _value=0.0: self._refresh_finger_static_test_feedback())
            self.finger_test_spins.append(spin)
            self.finger_test_current_items.append(current)
            self.finger_test_error_items.append(error)
            setattr(self, f"_finger_test_joint_label_{row}", joint_label)
            self._control_widgets.append(spin)

        self._control_widgets.extend([
            self.finger_test_group_combo,
            self.finger_test_same_check,
            self.finger_test_same_spin,
            self.finger_test_apply_btn,
            self.finger_test_sync_btn,
        ])
        self._refresh_finger_static_test_controls()
        return group

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

    @staticmethod
    def _make_float_spin(low: float, high: float, value: float, decimals: int) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(float(low), float(high))
        spin.setDecimals(int(decimals))
        spin.setSingleStep(0.1 if decimals <= 2 else 0.01)
        spin.setValue(float(value))
        return spin

    def _selected_finger_static_joints(self) -> List[int]:
        combo = getattr(self, "finger_test_group_combo", None)
        if combo is None:
            return []
        data = combo.currentData()
        if not isinstance(data, tuple) or len(data) != 2:
            return []
        start, end = int(data[0]), int(data[1])
        return [idx for idx in range(start, end + 1) if 0 <= idx < ENCODER_COUNT]

    def _refresh_finger_static_test_controls(self) -> None:
        joints = self._selected_finger_static_joints()
        same = bool(
            getattr(self, "finger_test_same_check", None) is not None and
            self.finger_test_same_check.isChecked()
        )
        if getattr(self, "finger_test_same_spin", None) is not None:
            self.finger_test_same_spin.setEnabled(same)
        for row, spin in enumerate(getattr(self, "finger_test_spins", [])):
            joint_idx = joints[row] if row < len(joints) else None
            label = getattr(self, f"_finger_test_joint_label_{row}", None)
            if label is not None:
                label.setText(f"J{joint_idx:02d}" if joint_idx is not None else "-")
                label.setVisible(joint_idx is not None)
            spin.setVisible(joint_idx is not None)
            spin.setEnabled(not same and joint_idx is not None)
            if joint_idx is not None:
                spin.setRange(self._joint_target_min_deg(joint_idx), self._joint_target_max_deg(joint_idx))
                if same and getattr(self, "finger_test_same_spin", None) is not None:
                    spin.setValue(self._clamp_joint_target_deg(joint_idx, self.finger_test_same_spin.value()))
            if row < len(getattr(self, "finger_test_current_items", [])):
                self.finger_test_current_items[row].setVisible(joint_idx is not None)
            if row < len(getattr(self, "finger_test_error_items", [])):
                self.finger_test_error_items[row].setVisible(joint_idx is not None)
        self._refresh_finger_static_test_feedback()

    def _on_finger_static_same_angle_changed(self, value: float) -> None:
        if not (
            getattr(self, "finger_test_same_check", None) is not None and
            self.finger_test_same_check.isChecked()
        ):
            return
        joints = self._selected_finger_static_joints()
        for row, joint_idx in enumerate(joints[:len(getattr(self, "finger_test_spins", []))]):
            self.finger_test_spins[row].setValue(self._clamp_joint_target_deg(joint_idx, float(value)))
        self._refresh_finger_static_test_feedback()

    def _sync_finger_static_test_from_current(self) -> None:
        state = self._state
        angles = getattr(state, "angles", []) if state is not None else []
        if not angles:
            self._set_status("Finger Q Test sync blocked: no joint feedback")
            return
        joints = self._selected_finger_static_joints()
        if not joints:
            return
        same_values: List[float] = []
        for row, joint_idx in enumerate(joints[:len(getattr(self, "finger_test_spins", []))]):
            if joint_idx >= len(angles):
                continue
            value = self._clamp_joint_target_deg(joint_idx, float(angles[joint_idx]))
            self.finger_test_spins[row].setValue(value)
            same_values.append(value)
        if same_values and getattr(self, "finger_test_same_spin", None) is not None:
            self.finger_test_same_spin.setValue(sum(same_values) / float(len(same_values)))
        self._refresh_finger_static_test_feedback()
        self._set_status("Finger Q Test synced from current angles")

    def _on_apply_finger_static_test(self) -> None:
        if self._manual_submode() != "test" and hasattr(self, "tabs"):
            self.tabs.setCurrentWidget(self.test_panel)
        if not self._guard_joint_command_started():
            return
        joints = self._selected_finger_static_joints()
        if not joints:
            self._set_status("Finger Q Test blocked: no finger selected")
            return
        angles = self.resolved_joint_targets()
        same = bool(
            getattr(self, "finger_test_same_check", None) is not None and
            self.finger_test_same_check.isChecked()
        )
        same_value = float(self.finger_test_same_spin.value()) if getattr(self, "finger_test_same_spin", None) else 0.0
        for row, joint_idx in enumerate(joints[:len(getattr(self, "finger_test_spins", []))]):
            raw = same_value if same else float(self.finger_test_spins[row].value())
            target = self._clamp_joint_target_deg(joint_idx, raw)
            angles[joint_idx] = target
            self._set_joint_target_widget(joint_idx, target)
            self._joint_target_user_set[joint_idx] = True
            self.finger_test_spins[row].setValue(target)
        try:
            self.context.send_joint_targets(angles, source="Finger Q Test", live=True)
        except Exception as exc:
            self._set_status(f"Finger Q Test send failed: {exc}")
            return
        self._refresh_finger_static_test_feedback()
        joint_text = ",".join(f"J{idx:02d}" for idx in joints)
        self._set_status(f"Finger Q Test sent: {joint_text}")

    def _refresh_finger_static_test_feedback(self) -> None:
        state = self._state
        angles = getattr(state, "angles", []) if state is not None else []
        joints = self._selected_finger_static_joints()
        for row, joint_idx in enumerate(joints[:len(getattr(self, "finger_test_current_items", []))]):
            current_label = self.finger_test_current_items[row]
            error_label = self.finger_test_error_items[row]
            if joint_idx < len(angles):
                current = float(angles[joint_idx])
                target = float(self.finger_test_spins[row].value())
                current_label.setText(f"{current:.2f}")
                error_label.setText(f"{(target - current):+.2f}")
            else:
                current_label.setText("-")
                error_label.setText("-")

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
        ready_for_feedback = system_state == SYSTEM_STATE_READY and self._active_scope_feedback_calibrated()
        for btn in [getattr(self, "send_all_btn", None)]:
            if btn is not None:
                btn.setEnabled(interaction and ready_for_feedback)
        if getattr(self, "open_btn", None) is not None:
            self.open_btn.setVisible(mode in ("joint", "test"))
            self.open_btn.setEnabled(interaction and ready_for_feedback and mode in ("joint", "test"))
        if getattr(self, "release_half_turn_btn", None) is not None:
            self.release_half_turn_btn.setVisible(mode == "motor")
            self.release_half_turn_btn.setEnabled(interaction and mode == "motor")
        if getattr(self, "calib_load_btn", None) is not None:
            can_load = (
                bool(self.context.is_comm_connected()) and
                CALIB_LOAD_SNAPSHOT_PATH.exists() and
                system_state not in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK)
            )
            self.calib_load_btn.setEnabled(can_load)
        for btn in [getattr(self, "fist_btn", None), getattr(self, "zero_btn", None)]:
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
        self._mark_motor_target_user_set(idx)
        self._sync_motor_abs_from_slider_before_send(idx)
        sent_abs = self._send_single_motor_abs_from_ui(idx, f"Run M{idx}")
        if sent_abs is not None:
            self._set_row_running(True, idx, True)
            self._clear_motor_target_after_single_send(idx, sent_abs)
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

    def release_all_motors_half_turn(self) -> None:
        if not self._interaction_enabled:
            return
        if self._manual_submode() != "motor":
            self._set_status("Release Half Turn is only available in Motor Channels")
            return
        try:
            if not self.context.is_comm_connected():
                raise RuntimeError("Serial port is not connected")
            self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.controller.set_pid_control(False)
            if self._estop_latched and not self.controller.is_started():
                self._set_status("Release blocked: press ENABLE after E-STOP")
                return
            if not self.controller.is_started():
                self._set_status("Release blocked: waiting for controllable IDLE after ENABLE")
                return
            unknown_mask = self._active_scope_multiturn_unknown_mask()
            if unknown_mask != 0:
                self._set_status(f"Release blocked: Turn UNKNOWN 0x{unknown_mask:08X}; recover multi-turn first")
                return
            state = self._state
            if state is None or not bool(getattr(state, "has_servo_angle_data", False)):
                self._set_status("Release blocked: waiting for motor ABS feedback")
                return

            active_mask = self._active_motor_mask_from_ui() & int(VALID_MOTOR_MASK) & ((1 << MOTOR_COUNT) - 1)
            if active_mask == 0:
                self._set_status("Release blocked: no active motor selected")
                return
            targets = self._current_motor_hold_targets_from_feedback()
            current_abs: List[Optional[int]] = [None] * MOTOR_COUNT
            missing: List[str] = []
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                if (
                    idx >= len(getattr(state, "servo_online", [])) or
                    not state.servo_online[idx] or
                    idx >= len(getattr(state, "servo_angles", []))
                ):
                    missing.append(f"M{idx:02d}")
                    continue
                current = clamp_motor_abs(int(state.servo_angles[idx]))
                current_abs[idx] = current
                targets[idx] = motor_abs_after_tendon_action(current, idx, -MOTOR_RELEASE_HALF_TURN_COUNTS)

            if missing:
                self._set_status(f"Release blocked: waiting ABS feedback {','.join(missing[:4])}")
                return

            targets, limit_hits = self._clamp_motor_targets_to_calibrated_limits(targets, active_mask=active_mask)
            if limit_hits:
                self._show_motor_abs_limit_warning(limit_hits)
            self.controller.set_motor_positions_absolute(targets)
            self._motor_last_sent_abs = list(targets)
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                self._motor_session_origin_abs[idx] = int(targets[idx])
                self._motor_abs_pos[idx] = 0
                self._motor_last_slider[idx] = 0
                self._motor_target_user_set[idx] = False
                if idx < len(self.motor_target_spins) and idx < len(self.motor_sliders):
                    self._set_motor_target_widget(idx, 0)

            preview_items = []
            for idx in range(MOTOR_COUNT):
                if ((active_mask >> idx) & 0x01) == 0:
                    continue
                if current_abs[idx] is not None:
                    preview_items.append(f"M{idx:02d}:{current_abs[idx]}->{targets[idx]}")
                if len(preview_items) >= 4:
                    break
            active_count = bin(active_mask).count("1")
            suffix = "" if active_count <= 4 else f" ... +{active_count - 4}"
            preview = ", ".join(preview_items)
            self.context.emit_action(f"Release Half Turn sent: {preview}{suffix}")
            self._set_status(f"Released {active_count} active motor(s) by 1/2 turn ({MOTOR_RELEASE_HALF_TURN_COUNTS} counts)")
        except Exception as exc:
            self._error("Release Failed", f"{type(exc).__name__}: {exc}")

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

    def clear_fault_with_confirm(self) -> None:
        state = self._state
        if state is None:
            try:
                state = self.context.get_state()
            except Exception:
                state = None

        summary = None
        root = getattr(self.context, "root", None)
        builder = getattr(root, "_build_fault_summary", None)
        if callable(builder) and state is not None:
            try:
                summary = builder(state)
            except Exception:
                summary = None

        if summary and summary.get("active"):
            body_builder = getattr(root, "_fault_body_text", None)
            if callable(body_builder):
                detail = body_builder(summary)
            else:
                detail = "\n".join(summary.get("lines") or [])
            text = (
                "当前检测到以下故障：\n\n"
                f"{detail}\n\n"
                "请确认已经排除机械卡滞、绳索异常、接线/供电问题后，再清除故障。\n"
                "清除后不会自动恢复运行，需要重新 ENABLE 或重新发送目标。"
            )
            title = "清除故障确认"
            accept_text = "我已检查，清除故障"
        else:
            text = (
                "当前未检测到故障，是否仍要清除状态？\n\n"
                "这会向下位机发送 reset，清除本地运行状态，并回到需要重新 ENABLE 的状态。"
            )
            title = "Clear Fault"
            accept_text = "仍然清除"

        box = QtWidgets.QMessageBox(self)
        box.setIcon(QtWidgets.QMessageBox.Warning if summary and summary.get("active") else QtWidgets.QMessageBox.Question)
        box.setWindowTitle(title)
        box.setText(text)
        clear_button = box.addButton(accept_text, QtWidgets.QMessageBox.AcceptRole)
        box.addButton("取消", QtWidgets.QMessageBox.RejectRole)
        box.setDefaultButton(clear_button)
        box.exec()
        if box.clickedButton() is not clear_button:
            self._set_status("Clear Fault cancelled")
            return

        self.reset_controller()
        notifier = getattr(root, "_on_clear_fault_confirmed", None)
        if callable(notifier):
            notifier()

    def reset_controller(self) -> None:
        self._estop_latched = True
        self.controller.reset()
        self._clear_run_states()
        self.context.emit_action("Clear Fault")

    def _active_scope_joint_indices(self) -> List[int]:
        data = self._current_active_scope_tuple()
        if data is not None:
            _motor_mask, joint_mask, _require_can = data
            return [
                idx for idx in range(ENCODER_COUNT)
                if (int(joint_mask) & (1 << idx)) != 0
            ]
        return list(AUTO_CALIBRATION_JOINT_INDICES)

    def _dual_rope_init_motors_for_joint(self, joint_idx: int) -> Optional[Tuple[int, int]]:
        if joint_idx < 0 or joint_idx >= len(self._motors_by_joint):
            return None
        motors = [
            int(motor_idx)
            for motor_idx in self._motors_by_joint[joint_idx]
            if _motor_channel_valid(int(motor_idx))
        ]
        if len(motors) < 2:
            return None
        return motors[0], motors[1]

    def _build_rope_init_tasks(self) -> Tuple[List[dict], List[str], List[str]]:
        state = self._state
        if state is None or not bool(getattr(state, "has_sensor_data", False)):
            return [], [], ["joint angle feedback"]
        angles = getattr(state, "angles", []) or []
        servo_online = getattr(state, "servo_online", []) or []
        servo_angles = getattr(state, "servo_angles", []) or []

        tasks: List[dict] = []
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

            dual_motors = self._dual_rope_init_motors_for_joint(joint_idx)
            if dual_motors is not None:
                primary_motor, secondary_motor = dual_motors
                if joint_deg >= ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG:
                    skipped.append(f"J{joint_idx:02d}={joint_deg:.2f}/workspace")
                    continue
                if primary_motor in seen_motors or secondary_motor in seen_motors:
                    missing.append(f"J{joint_idx:02d} motor conflict")
                    continue
                for motor_idx in (primary_motor, secondary_motor):
                    if motor_idx >= len(servo_online) or not bool(servo_online[motor_idx]):
                        missing.append(f"M{motor_idx:02d} online")
                        break
                    if motor_idx >= len(servo_angles):
                        missing.append(f"M{motor_idx:02d} abs")
                        break
                else:
                    seen_motors.add(primary_motor)
                    seen_motors.add(secondary_motor)
                    tasks.append({
                        "type": "dual",
                        "joint": joint_idx,
                        "primary": primary_motor,
                        "secondary": secondary_motor,
                        "target_deg": ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG,
                    })
                continue

            if joint_deg >= ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG:
                skipped.append(f"J{joint_idx:02d}={joint_deg:.2f}/workspace")
                continue
            motor_idx = TendonGuard.primary_motor_for_joint(joint_idx)
            if motor_idx < 0 or motor_idx >= MOTOR_COUNT:
                missing.append(f"J{joint_idx:02d} motor")
                continue
            if motor_idx in seen_motors:
                missing.append(f"M{motor_idx:02d} conflict")
                continue
            if motor_idx >= len(servo_online) or not bool(servo_online[motor_idx]):
                missing.append(f"M{motor_idx:02d} online")
                continue
            if motor_idx >= len(servo_angles):
                missing.append(f"M{motor_idx:02d} abs")
                continue
            seen_motors.add(motor_idx)
            tasks.append({
                "type": "single",
                "joint": joint_idx,
                "motor": motor_idx,
                "target_deg": ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG,
            })
        return tasks, skipped, missing

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
        if self._rope_init_timer.isActive():
            self._stop_rope_init("Rope Init restarted", update_status=False)
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

        tasks, skipped, missing = self._build_rope_init_tasks()
        if not tasks:
            detail = "; ".join(skipped[:4]) if skipped else ", ".join(missing[:4])
            self._set_status(f"Rope Init no action: {detail or 'all selected joints are inside workspace'}")
            return

        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        self._rope_init_tasks = list(tasks)
        self._rope_init_started_at = time.monotonic()
        self.context.emit_action(
            "Rope Init start: " +
            ",".join(
                (
                    f"J{task['joint']:02d}/M{task['motor']:02d}->workspace"
                    if task.get("type") == "single"
                    else f"J{task['joint']:02d}/M{task['primary']:02d},M{task['secondary']:02d}->workspace"
                )
                for task in tasks
            )
        )
        skipped_text = f"; skipped {', '.join(skipped[:3])}" if skipped else ""
        self._set_status(f"Rope Init running for {len(tasks)} joint(s){skipped_text}")
        self._rope_init_timer.start()
        self._rope_init_tick()

    def _stop_rope_init(self, reason: str = "stopped", update_status: bool = True) -> None:
        was_running = self._rope_init_timer.isActive() or bool(self._rope_init_tasks)
        self._rope_init_timer.stop()
        self._rope_init_tasks = []
        if update_status and was_running:
            self._set_status(reason)

    def _apply_rope_init_motor_target_to_ui(self, motor_idx: int, target_abs: int) -> None:
        if motor_idx < 0 or motor_idx >= MOTOR_COUNT:
            return
        target_abs = clamp_motor_abs(int(target_abs))
        self._motor_session_origin_abs[motor_idx] = target_abs
        self._motor_abs_pos[motor_idx] = 0
        self._motor_last_slider[motor_idx] = 0
        self._motor_target_user_set[motor_idx] = False
        self._set_motor_target_widget(motor_idx, 0)

    def _rope_init_dual_motor_delta(self, motor_idx: int, raw_direction: int) -> int:
        direction = 1 if int(raw_direction) >= 0 else -1
        is_tightening = direction == motor_tendon_shorten_abs_sign(motor_idx)
        counts = ROPE_INIT_TIGHTEN_STEP_COUNTS if is_tightening else ROPE_INIT_RELEASE_STEP_COUNTS
        return direction * counts

    def _rope_init_tick(self) -> None:
        if not self._rope_init_tasks:
            self._stop_rope_init("Rope Init complete")
            return
        if (time.monotonic() - self._rope_init_started_at) > ROPE_INIT_TIMEOUT_S:
            self._stop_rope_init("Rope Init timeout: check joint feedback")
            return

        state = self._state
        if state is None or not bool(getattr(state, "has_sensor_data", False)):
            self._stop_rope_init("Rope Init stopped: missing joint angle feedback")
            return
        system_state = getattr(state, "system_state", None)
        if system_state not in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY, SYSTEM_STATE_RUN_DIRECT):
            self._stop_rope_init("Rope Init stopped: wait for IDLE/READY")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._stop_rope_init(f"Rope Init stopped: Turn UNKNOWN 0x{unknown_mask:08X}")
            return

        angles = getattr(state, "angles", []) or []
        targets = self._current_motor_hold_targets_from_feedback()
        remaining: List[dict] = []
        moved_motors = set()
        feedback_missing: List[str] = []
        completed: List[str] = []

        for task in self._rope_init_tasks:
            joint_idx = int(task.get("joint", -1))
            if joint_idx < 0 or joint_idx >= ENCODER_COUNT or joint_idx >= len(angles):
                feedback_missing.append(f"J{joint_idx:02d} angle")
                continue
            joint_deg = float(angles[joint_idx])

            if task.get("type") == "dual":
                primary_motor = int(task.get("primary", -1))
                secondary_motor = int(task.get("secondary", -1))
                target_deg = float(task.get("target_deg", ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG))
                if joint_deg >= target_deg:
                    completed.append(f"J{joint_idx:02d}={joint_deg:.2f}")
                    continue
                error_deg = target_deg - joint_deg
                primary_abs = self._motor_feedback_abs(primary_motor)
                secondary_abs = self._motor_feedback_abs(secondary_motor)
                if primary_abs is None or secondary_abs is None:
                    feedback_missing.append(f"J{joint_idx:02d} motor feedback")
                    continue
                step_sign = 1 if error_deg > 0.0 else -1
                primary_delta = self._rope_init_dual_motor_delta(primary_motor, step_sign)
                secondary_delta = self._rope_init_dual_motor_delta(secondary_motor, -step_sign)
                targets[primary_motor] = clamp_motor_abs(int(primary_abs) + primary_delta)
                targets[secondary_motor] = clamp_motor_abs(int(secondary_abs) + secondary_delta)
                moved_motors.update((primary_motor, secondary_motor))
                remaining.append(task)
            else:
                motor_idx = int(task.get("motor", -1))
                if joint_deg >= ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG:
                    completed.append(f"J{joint_idx:02d}={joint_deg:.2f}")
                    continue
                feedback_abs = self._motor_feedback_abs(motor_idx)
                if feedback_abs is None:
                    feedback_missing.append(f"M{motor_idx:02d}")
                    continue
                targets[motor_idx] = motor_abs_after_tendon_action(
                    int(feedback_abs),
                    motor_idx,
                    ROPE_INIT_STEP_COUNTS,
                )
                moved_motors.add(motor_idx)
                remaining.append(task)

        if feedback_missing:
            self._stop_rope_init(f"Rope Init stopped: waiting for {','.join(feedback_missing[:4])}")
            return

        if not moved_motors:
            done_text = f" ({', '.join(completed[:4])})" if completed else ""
            self._stop_rope_init(f"Rope Init complete{done_text}")
            return

        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        self.controller.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        for motor_idx in sorted(moved_motors):
            self._apply_rope_init_motor_target_to_ui(motor_idx, int(targets[motor_idx]))
        self._rope_init_tasks = remaining

    def calibrate(self) -> None:
        for joint_idx in self._active_scope_joint_indices():
            if 0 <= joint_idx < len(self._joint_target_user_set):
                self._joint_target_user_set[joint_idx] = False
        self._joint_target_initialized = False
        self.context.start_calibration()

    def _active_scope_abs_limit_valid_mask(self) -> int:
        state = self._state
        if state is None:
            return 0
        return self._active_motor_mask_from_ui() & int(getattr(state, "motor_abs_limit_valid_bitmap", 0))

    def _active_scope_abs_limit_missing_mask(self) -> int:
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        if active_mask == 0:
            return 0
        state = self._state
        if state is None:
            return active_mask
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
        return active_mask & ~valid_bitmap

    def _active_scope_feedback_calibrated(self) -> bool:
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        return active_mask != 0 and self._active_scope_abs_limit_missing_mask() == 0

    def _format_motor_mask(self, mask: int, limit: int = 6) -> str:
        indices = [
            idx for idx in range(MOTOR_COUNT)
            if _motor_channel_valid(idx) and (int(mask) & (1 << idx)) != 0
        ]
        if not indices:
            return "none"
        text = ",".join(f"M{idx:02d}" for idx in indices[:limit])
        if len(indices) > limit:
            text += f"...(+{len(indices) - limit})"
        return text

    def feedback_command_allowed(self, source: str = "", show_status: bool = True) -> bool:
        if self._app_mode not in ("manual", "teleop"):
            return True
        active_mask = self._active_motor_mask_from_ui() & VALID_MOTOR_MASK
        if active_mask == 0:
            if show_status:
                self._set_status("Feedback blocked: no active motor selected")
            return False
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            if show_status:
                self._set_status(f"Feedback blocked: Turn UNKNOWN {self._format_motor_mask(unknown_mask)}")
            return False
        missing_mask = self._active_scope_abs_limit_missing_mask()
        if missing_mask != 0:
            if show_status:
                label = self._active_scope_label_from_ui() or str(source or "Feedback")
                self._set_status(
                    f"{label} blocked: calibrate active motors first ({self._format_motor_mask(missing_mask)})"
                )
            return False
        return True

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

    def toggle_multiturn_nvs(self) -> None:
        if not self.context.is_comm_connected():
            self._set_status("Flash control blocked: serial port is not connected")
            return
        state = self._state or self.controller.current_state
        current = bool(getattr(state, "multiturn_nvs_enabled", True))
        target = not current
        try:
            ok = self.controller.set_multiturn_nvs_enabled(target)
        except Exception as exc:
            self._set_status(f"Flash control failed: {exc}")
            return
        if ok:
            if target:
                self._set_status("Flash multi-turn save enabled")
            else:
                self._set_status("Flash multi-turn save disabled; lower snapshots and desktop cache cleared")
        else:
            self._set_status("Flash control ACK timeout")

    def _refresh_nvs_control_button(self) -> None:
        btn = getattr(self, "nvs_control_btn", None)
        if btn is None:
            return
        state = self._state
        enabled = bool(getattr(state, "multiturn_nvs_enabled", True)) if state is not None else True
        snapshot = bool(getattr(state, "multiturn_nvs_snapshot_valid", False)) if state is not None else False
        btn.setText("Flash ON" if enabled else "Flash OFF")
        if enabled:
            btn.setStyleSheet("font-weight:700;background:#e8f5e9;color:#1b5e20;")
            btn.setToolTip(
                "Flash multi-turn save/boot restore enabled; "
                + ("snapshot valid" if snapshot else "no snapshot yet")
            )
        else:
            btn.setStyleSheet("font-weight:700;background:#ffebee;color:#c62828;")
            btn.setToolTip("Flash save disabled; lower snapshots and desktop restore cache cleared; click to enable future saves")

    def _on_active_scope_changed(self) -> None:
        if getattr(self, "scope_combo", None) is not None:
            self._scope_selection_index = int(self.scope_combo.currentIndex())
        self._apply_active_scope_for_current_mode()

    def _sync_motor_selection_from_joint_selection(self) -> None:
        motor_mask = self._motor_mask_for_joint_mask(self._selected_joint_mask)
        self._selected_motor_mask = int(motor_mask) & VALID_MOTOR_MASK
        self._scope_update_guard = True
        try:
            for motor_idx, check in enumerate(getattr(self, "motor_use_checks", [])):
                if check is None:
                    continue
                check.blockSignals(True)
                check.setChecked((self._selected_motor_mask & (1 << motor_idx)) != 0)
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False

    def _sync_scope_group_checks(self) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for check in getattr(self, "joint_group_use_checks", []):
                text = str(check.text())
                target = next((item for item in FINGER_JOINT_RANGES if item[0] == text), None)
                if target is None:
                    continue
                label, start, end = target
                group_mask = _joint_group_mask(label, start, end)
                check.blockSignals(True)
                check.setChecked((self._selected_joint_mask & group_mask) == group_mask)
                check.blockSignals(False)

            motor_rows, group_ranges = _motor_rows_by_finger_group()
            for check, (start, end, _label) in zip(getattr(self, "motor_group_use_checks", []), group_ranges):
                group_mask = 0
                for row in range(start, end + 1):
                    if 0 <= row < len(motor_rows):
                        motor_idx = int(motor_rows[row])
                        if _motor_channel_valid(motor_idx):
                            group_mask |= (1 << motor_idx)
                check.blockSignals(True)
                check.setChecked(group_mask != 0 and (self._selected_motor_mask & group_mask) == group_mask)
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False

    def _on_joint_use_changed(self, joint_idx: int, checked: bool) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        if 0 <= int(joint_idx) < ENCODER_COUNT:
            bit = 1 << int(joint_idx)
            if checked:
                self._selected_joint_mask |= bit
            else:
                self._selected_joint_mask &= ~bit
        self._sync_motor_selection_from_joint_selection()
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_joint_group_use_changed(self, start: int, end: int, checked: bool) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for joint_idx in range(int(start), int(end) + 1):
                if 0 <= joint_idx < ENCODER_COUNT:
                    bit = 1 << joint_idx
                    if checked:
                        self._selected_joint_mask |= bit
                    else:
                        self._selected_joint_mask &= ~bit
                    check = self.joint_use_checks[joint_idx] if joint_idx < len(self.joint_use_checks) else None
                    if check is not None:
                        check.blockSignals(True)
                        check.setChecked(bool(checked))
                        check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._sync_motor_selection_from_joint_selection()
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_motor_use_changed(self, motor_idx: int, checked: bool) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        if _motor_channel_valid(int(motor_idx)):
            bit = 1 << int(motor_idx)
            if checked:
                self._selected_motor_mask |= bit
            else:
                self._selected_motor_mask &= ~bit
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_motor_group_use_changed(self, motor_indices: List[int], checked: bool) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for motor_idx in motor_indices:
                motor_idx = int(motor_idx)
                if not _motor_channel_valid(motor_idx):
                    continue
                bit = 1 << motor_idx
                if checked:
                    self._selected_motor_mask |= bit
                else:
                    self._selected_motor_mask &= ~bit
                check = self.motor_use_checks[motor_idx] if motor_idx < len(self.motor_use_checks) else None
                if check is not None:
                    check.blockSignals(True)
                    check.setChecked(bool(checked))
                    check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._sync_scope_group_checks()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _teleop_selected_joint_mask_value(self) -> int:
        return int(getattr(self, "_teleop_selected_joint_mask", 0)) & ((1 << ENCODER_COUNT) - 1)

    def _sync_teleop_group_checks(self) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            hand_check = getattr(self, "teleop_hand_use_check", None)
            if hand_check is not None:
                all_mask = (1 << ENCODER_COUNT) - 1
                hand_check.blockSignals(True)
                hand_check.setChecked((self._teleop_selected_joint_mask & all_mask) == all_mask)
                hand_check.blockSignals(False)
            for check in getattr(self, "teleop_group_use_checks", []):
                text = str(check.text())
                target = next((item for item in FINGER_JOINT_RANGES if item[0] == text), None)
                if target is None:
                    continue
                label, start, end = target
                group_mask = _joint_group_mask(label, start, end)
                check.blockSignals(True)
                check.setChecked((self._teleop_selected_joint_mask & group_mask) == group_mask)
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False

    def _on_teleop_hand_use_changed(self, checked: bool) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            all_mask = (1 << ENCODER_COUNT) - 1
            self._teleop_selected_joint_mask = all_mask if checked else 0
            for joint_idx, check in enumerate(getattr(self, "teleop_joint_use_checks", [])):
                if check is not None:
                    check.blockSignals(True)
                    check.setChecked(bool(checked))
                    check.blockSignals(False)
            for check in getattr(self, "teleop_group_use_checks", []):
                check.blockSignals(True)
                check.setChecked(bool(checked))
                check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._teleop_selected_motor_mask = self._motor_mask_for_joint_mask(self._teleop_selected_joint_mask)
        self._apply_active_scope_for_current_mode()

    def _on_teleop_joint_use_changed(self, joint_idx: int, checked: bool) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        if 0 <= int(joint_idx) < ENCODER_COUNT:
            bit = 1 << int(joint_idx)
            if checked:
                self._teleop_selected_joint_mask |= bit
            else:
                self._teleop_selected_joint_mask &= ~bit
        self._teleop_selected_motor_mask = self._motor_mask_for_joint_mask(self._teleop_selected_joint_mask)
        self._sync_teleop_group_checks()
        self._apply_active_scope_for_current_mode()

    def _on_teleop_joint_group_use_changed(self, start: int, end: int, checked: bool) -> None:
        if bool(getattr(self, "_scope_update_guard", False)):
            return
        self._scope_update_guard = True
        try:
            for joint_idx in range(int(start), int(end) + 1):
                if 0 <= joint_idx < ENCODER_COUNT:
                    bit = 1 << joint_idx
                    if checked:
                        self._teleop_selected_joint_mask |= bit
                    else:
                        self._teleop_selected_joint_mask &= ~bit
                    check = (
                        self.teleop_joint_use_checks[joint_idx]
                        if joint_idx < len(self.teleop_joint_use_checks)
                        else None
                    )
                    if check is not None:
                        check.blockSignals(True)
                        check.setChecked(bool(checked))
                        check.blockSignals(False)
        finally:
            self._scope_update_guard = False
        self._teleop_selected_motor_mask = self._motor_mask_for_joint_mask(self._teleop_selected_joint_mask)
        self._sync_teleop_group_checks()
        self._apply_active_scope_for_current_mode()

    def _current_active_scope_tuple(self) -> Optional[Tuple[int, int, bool]]:
        if self._app_mode == "manual":
            mode = self._manual_submode()
            if mode == "motor":
                return int(self._selected_motor_mask) & VALID_MOTOR_MASK, 0, False
            joint_mask = int(self._selected_joint_mask) & ((1 << ENCODER_COUNT) - 1)
            motor_mask = self._motor_mask_for_joint_mask(joint_mask)
            return motor_mask, joint_mask, True

        if self._app_mode == "teleop":
            joint_mask = self._teleop_selected_joint_mask_value()
            motor_mask = self._motor_mask_for_joint_mask(joint_mask)
            self._teleop_selected_motor_mask = int(motor_mask) & VALID_MOTOR_MASK
            return int(motor_mask), int(joint_mask), True

        data = self.scope_combo.currentData() if getattr(self, "scope_combo", None) is not None else None
        if not data:
            return None
        try:
            return int(data[0]), int(data[1]), bool(data[2])
        except Exception:
            return None

    def _active_motor_mask_from_ui(self) -> int:
        data = self._current_active_scope_tuple()
        if data is None:
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
        if self._app_mode == "manual":
            mode = self._manual_submode()
            if mode == "motor":
                indices = self._active_motor_indices_from_ui()
                if len(indices) == 1:
                    return f"Manual Motor M{indices[0]:02d}"
                if indices:
                    return f"Manual Motors {len(indices)} selected"
                return "Manual Motors none"
            joints = self._active_scope_joint_indices()
            if len(joints) == 1:
                return f"Manual Joint J{joints[0]:02d}"
            if joints:
                return f"Manual Joints {len(joints)} selected"
            return "Manual Joints none"
        if self._app_mode == "teleop":
            joints = self._active_scope_joint_indices()
            if len(joints) == 1:
                return f"Teleop Joint J{joints[0]:02d}"
            if joints:
                return f"Teleop Joints {len(joints)} selected"
            return "Teleop Joints none"
        combo = getattr(self, "scope_combo", None)
        if combo is not None:
            try:
                return str(combo.currentText())
            except Exception:
                return ""
        return ""

    def _active_joint_mask_from_ui(self) -> int:
        data = self._current_active_scope_tuple()
        if data is None:
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

    def _valid_joint_calib_results_for_mask(self, state, joint_mask: int) -> List[Optional[dict]]:
        source = getattr(state, "joint_calib_results", []) or []
        results: List[Optional[dict]] = [None] * ENCODER_COUNT
        for joint_idx in range(ENCODER_COUNT):
            if joint_idx >= len(source):
                continue
            item = source[joint_idx]
            if not isinstance(item, dict) or not bool(item.get("success", False)):
                continue
            if (int(joint_mask) & (1 << joint_idx)) == 0:
                results[joint_idx] = dict(item)
                continue
            results[joint_idx] = dict(item)
        return results

    def _maybe_save_calibration_snapshot(self, state) -> None:
        if state is None or str(getattr(state, "calib_status", "IDLE")) != "SUCCESS":
            return
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
        if valid_bitmap == 0:
            return
        active_joint_mask = int(self._active_joint_mask_from_ui())
        active_motor_mask = int(self._active_motor_mask_from_ui())
        if active_joint_mask == 0:
            return
        required_motor_mask = active_motor_mask & VALID_MOTOR_MASK
        if required_motor_mask != 0 and (valid_bitmap & required_motor_mask) != required_motor_mask:
            return

        joint_results = self._valid_joint_calib_results_for_mask(state, active_joint_mask)
        if not any(item is not None for item in joint_results):
            return
        required_joints = [
            idx for idx in range(ENCODER_COUNT)
            if (active_joint_mask & (1 << idx)) != 0
        ]
        if required_joints and not all(joint_results[idx] for idx in required_joints):
            return

        mins = [int(v) for v in list(getattr(state, "motor_abs_limit_min", []) or [])[:MOTOR_COUNT]]
        maxs = [int(v) for v in list(getattr(state, "motor_abs_limit_max", []) or [])[:MOTOR_COUNT]]
        if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
            return

        payload = {
            "schema_version": 1,
            "saved_at_host_time": time.time(),
            "hand_profile": HAND_PROFILE,
            "active_joint_mask": f"0x{active_joint_mask:08X}",
            "active_motor_mask": f"0x{active_motor_mask:08X}",
            "motor_abs_limit_valid_bitmap": f"0x{valid_bitmap:08X}",
            "motor_abs_limit_locked_bitmap": f"0x{int(getattr(state, 'motor_abs_limit_locked_bitmap', valid_bitmap) or 0):08X}",
            "motor_abs_limit_min": mins,
            "motor_abs_limit_max": maxs,
            "joint_calib_results": joint_results,
        }
        signature = json.dumps(
            {
                "active_joint_mask": payload["active_joint_mask"],
                "active_motor_mask": payload["active_motor_mask"],
                "valid": payload["motor_abs_limit_valid_bitmap"],
                "locked": payload["motor_abs_limit_locked_bitmap"],
                "mins": mins,
                "maxs": maxs,
                "joints": joint_results,
            },
            sort_keys=True,
            ensure_ascii=False,
        )
        if signature == self._last_calib_snapshot_signature:
            return
        try:
            CALIB_LOAD_SNAPSHOT_PATH.parent.mkdir(parents=True, exist_ok=True)
            CALIB_LOAD_SNAPSHOT_PATH.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
        except OSError as exc:
            self._set_status(f"Calib snapshot save failed: {exc}")
            return
        self._last_calib_snapshot_signature = signature
        self.context.emit_action(f"Calib snapshot saved: {CALIB_LOAD_SNAPSHOT_PATH.name}")
        self._set_status(f"Calib snapshot saved to {CALIB_LOAD_SNAPSHOT_PATH.name}")
        self._update_manual_action_buttons()

    def load_calibration_snapshot(self) -> None:
        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            self._warn("Calib Load", "没有找到上一次标定快照文件。请先完成一次 Calibrate。")
            return
        try:
            snapshot = json.loads(CALIB_LOAD_SNAPSHOT_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self._error("Calib Load", f"读取标定快照失败：{exc}")
            return
        if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
            self._error(
                "Calib Load",
                f"标定快照 hand_profile 不一致：{snapshot.get('hand_profile')} != {HAND_PROFILE}",
            )
            return
        state = self._state
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK):
            self._warn("Calib Load", "当前正在运行控制，请先停止后再加载标定。")
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Calib Load",
            "确认机械结构、绳长、舵机安装、磁编码器安装都没有改变？\n\n"
            "点击确认后会把上一次标定结果恢复到下位机。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._set_status("Calib Load sending snapshot...")
        ok, message = self.controller.load_calibration_snapshot(snapshot)
        if ok:
            self._joint_target_initialized = False
            self._motor_target_initialized = False
            self._set_status(f"Calib Load success: {message}")
            self.context.emit_action(f"Calib Load success: {message}")
        else:
            self._error("Calib Load", f"加载失败：{message}")
            self._set_status(f"Calib Load failed: {message}")

    def _apply_active_scope_for_current_mode(self) -> bool:
        data = self._current_active_scope_tuple()
        if data is None:
            return False
        motor_mask, joint_mask, require_can = data
        if int(motor_mask) == 0 and int(joint_mask) == 0:
            self._set_status("Active scope blocked: no channel selected")
            return False
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
        self._handle_firmware_epoch_change(state)
        calib_status = str(getattr(state, "calib_status", "IDLE"))
        active_calib_statuses = {"PENDING", "RUNNING"}
        calib_finished_success = (
            calib_status == "SUCCESS" and
            self._last_calib_status in active_calib_statuses
        )
        if calib_finished_success:
            self._joint_target_initialized = False
        self._last_calib_status = calib_status
        self._update_connection_lights()
        self._refresh_nvs_control_button()
        self._check_feedback_abs_limit_violation()
        self._maybe_save_calibration_snapshot(state)
        if self._app_mode != "manual":
            updater = getattr(self._body_widget, "update_state", None)
            if callable(updater):
                updater(state)
            return
        self._refresh_joint_feedback_view(
            sync_target=(not self._joint_target_initialized) or calib_finished_success
        )
        self._refresh_motor_feedback_view(sync_target=not self._motor_target_initialized)
        self._update_manual_action_buttons()

    def _reset_motor_target_cache_for_device_epoch(self, reason: str) -> None:
        self._motor_target_initialized = False
        self._motor_abs_pos = [None] * MOTOR_COUNT
        self._motor_session_origin_abs = [None] * MOTOR_COUNT
        self._motor_last_slider = [None] * MOTOR_COUNT
        self._motor_last_sent_abs = [None] * MOTOR_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        self._clear_run_states()
        self.context.emit_action(reason)

    def _handle_firmware_epoch_change(self, state) -> None:
        if state is None or not bool(getattr(state, "has_system_status_data", False)):
            return
        try:
            transition_count = int(getattr(state, "state_transition_count", 0))
        except (TypeError, ValueError):
            transition_count = 0
        try:
            unknown_bitmap = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
        except (TypeError, ValueError):
            unknown_bitmap = 0

        if self._last_state_transition_count is not None and transition_count < self._last_state_transition_count:
            self._reset_motor_target_cache_for_device_epoch(
                "Firmware reboot detected: motor targets re-synced to feedback"
            )
        elif self._last_multi_turn_unknown_bitmap is not None and unknown_bitmap != self._last_multi_turn_unknown_bitmap:
            self._reset_motor_target_cache_for_device_epoch(
                "Turn status changed: motor targets re-synced to feedback"
            )

        self._last_state_transition_count = transition_count
        self._last_multi_turn_unknown_bitmap = unknown_bitmap

    def clear_live_state(self) -> None:
        self.stop_all_tests("stopped on disconnect", update_status=False)
        self._state = None
        self._update_connection_lights()
        self._refresh_nvs_control_button()
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._last_calib_status = None
        self._motor_abs_pos = [None] * MOTOR_COUNT
        self._motor_session_origin_abs = [None] * MOTOR_COUNT
        self._motor_dir_hint = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count = [0] * MOTOR_COUNT
        self._motor_last_slider = [None] * MOTOR_COUNT
        self._motor_slider_dragging = [False] * MOTOR_COUNT
        self._motor_last_sent_abs = [None] * MOTOR_COUNT
        self._last_state_transition_count = None
        self._last_multi_turn_unknown_bitmap = None
        self._joint_target_user_set = [False] * ENCODER_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        self._clear_run_states()
        self._updating = True
        try:
            for item in self.joint_current_items + self.joint_encoder_items:
                item.setText("-")
            for item in self.motor_turn_items + self.motor_raw_items + self.motor_abs_items + self.motor_load_items + self.motor_limit_items + self.motor_joint_items:
                if item is not None:
                    item.setText("-")
                    item.setToolTip("")
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
            for i in range(ENCODER_COUNT):
                self._set_joint_encoder_dot(i, None)
        finally:
            self._updating = False

    def stop_all_tests(self, reason: str = "stopped", update_status: bool = True, include_local: bool = True) -> None:
        self._stop_rope_init(reason=reason, update_status=update_status)
        self._on_stop_test(reason=reason, update_status=update_status)
        self._on_stop_ff_test(reason=reason, update_status=update_status)
        self._on_stop_tri_test(reason=reason, update_status=update_status)

    def _guard_joint_command_started(self) -> bool:
        self.controller.set_control_mode(ControlMode.JOINT_ANGLE)
        system_state = getattr(self._state, "system_state", None) if self._state is not None else None
        if system_state not in (SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK):
            if self._estop_latched:
                self._set_status("Command blocked: press ENABLE after E-STOP")
            elif system_state == SYSTEM_STATE_IDLE:
                self._set_status("Joint command blocked: wait for READY")
            else:
                self._set_status("Joint command blocked: controller is not READY")
            return False
        if system_state == SYSTEM_STATE_READY and not self._apply_active_scope_for_current_mode():
            return False
        self.controller.set_pid_control(True)
        if not self.feedback_command_allowed(show_status=True):
            self.controller.set_pid_control(False)
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

    def _send_single_motor_abs_from_ui(self, idx: int, trigger: str) -> Optional[int]:
        if idx < 0 or idx >= MOTOR_COUNT:
            return None
        if not self._interaction_enabled:
            return None
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        if self._estop_latched and not self.controller.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return None
        if not self.controller.is_started():
            self._set_status("Motor command blocked: waiting for controllable IDLE after ENABLE")
            return None
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return None
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self.context.emit_action(f"Motor command allowed with Turn UNKNOWN ({trigger}): unknown=0x{unknown_mask:08X}")
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        targets = self._build_motor_abs_command_list()
        targets, limit_hits = self._clamp_motor_targets_to_calibrated_limits(targets)
        if limit_hits:
            self._show_motor_abs_limit_warning(limit_hits)
        target_abs = int(targets[idx])
        target_abs, limit_hit = self._clamp_single_motor_target_to_calibrated_limit(idx, target_abs)
        if limit_hit is not None:
            self._show_motor_abs_limit_warning([limit_hit])
        self.controller.set_single_motor_position_absolute(idx, target_abs)
        if 0 <= idx < len(self._motor_last_sent_abs):
            self._motor_last_sent_abs[idx] = target_abs
        self.context.emit_action(
            f"Single motor absolute position sent ({trigger}): M{idx:02d}={target_abs}"
        )
        if unknown_mask != 0:
            self._set_status(f"Motor command sent while Turn UNKNOWN 0x{unknown_mask:08X}")
        return target_abs

    def _clear_motor_target_after_single_send(self, idx: int, sent_abs: int) -> None:
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        self._motor_session_origin_abs[idx] = int(sent_abs)
        self._motor_abs_pos[idx] = 0
        self._motor_last_slider[idx] = 0
        self._motor_target_user_set[idx] = False
        self._set_motor_target_widget(idx, 0)

    def _clamp_single_motor_target_to_calibrated_limit(
        self, idx: int, target: int
    ) -> Tuple[int, Optional[Tuple[int, int, int, int]]]:
        state = self._state
        if state is None:
            return int(target), None
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0))
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", valid_bitmap))
        if ((valid_bitmap >> idx) & 0x01) == 0 or ((locked_bitmap >> idx) & 0x01) == 0:
            return int(target), None
        mins = getattr(state, "motor_abs_limit_min", [])
        maxs = getattr(state, "motor_abs_limit_max", [])
        if idx >= len(mins) or idx >= len(maxs):
            return int(target), None
        lo = int(mins[idx])
        hi = int(maxs[idx])
        if lo > hi:
            lo, hi = hi, lo
        requested = int(target)
        limited = max(lo, min(hi, requested))
        if limited == requested:
            return limited, None
        return limited, (idx, requested, lo, hi)

    def _clamp_motor_targets_to_calibrated_limits(
        self,
        targets: List[int],
        active_mask: Optional[int] = None,
    ) -> Tuple[List[int], List[Tuple[int, int, int, int]]]:
        state = self._state
        if state is None:
            return targets, []
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0))
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", valid_bitmap))
        active_mask = self._active_motor_mask_from_ui() if active_mask is None else int(active_mask)
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
        self._set_status(f"Feedback ABS clamped by firmware: {preview}{suffix}")

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
            if joint_has_data:
                for i in range(min(ENCODER_COUNT, len(getattr(state, "angles", [])))):
                    user_set = i < len(self._joint_target_user_set) and self._joint_target_user_set[i]
                    if not sync_target and user_set:
                        continue
                    target = self._clamp_joint_target_deg(i, float(state.angles[i]))
                    self.joint_target_spins[i].setValue(target)
                    self.joint_sliders[i].setValue(int(round(target * 100.0)))
                    if sync_target and i < len(self._joint_target_user_set):
                        self._joint_target_user_set[i] = False
                self._joint_target_initialized = True
            self._refresh_finger_static_test_feedback()
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
            unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
            overload_flags = getattr(state, "servo_overload_fault", []) or []
            overload_loads = getattr(state, "servo_overload_trigger_load", []) or []
            for i in range(MOTOR_COUNT):
                dev_abs = self._motor_feedback_abs(i)
                if i < len(self.motor_turn_items):
                    turn_known = ((unknown_mask & (1 << i)) == 0)
                    self.motor_turn_items[i].setText(str(self._motor_turn_from_abs(dev_abs)) if dev_abs is not None and turn_known else "-")
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
            buttons[idx].setText("Send")

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
            self._send_single_motor_abs_from_ui(idx, f"Slider M{idx}")
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
        self._send_single_motor_abs_from_ui(idx, f"{'Step+' if delta > 0 else 'Step-'} M{idx}")

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
        any_running = self._test_running or self._ff_test_running or self._tri_test_running
        manual = bool(self._interaction_enabled)
        for btn, running in [
            (getattr(self, "test_stop_btn", None), self._test_running),
            (getattr(self, "ff_stop_btn", None), self._ff_test_running),
            (getattr(self, "tri_stop_btn", None), self._tri_test_running),
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
        try:
            self.controller.set_sine_test_params(active, self._test_hold_s)
            self.controller.start_sine_test()
        except Exception as exc:
            self.context.stop_recording_for_test("sine test failed")
            self._error("Start Test Failed", f"{type(exc).__name__}: {exc}")
            return
        self._test_running = True
        self._test_t0 = time.time()
        self._set_test_button_states()
        self.context.emit_action("Sine Test started on firmware")

    def _on_stop_test(self, reason: str = "stopped by test", update_status: bool = True) -> None:
        was_running = self._test_running
        self._test_running = False
        self._test_timer.stop()
        self._test_active_rows = []
        self._set_test_button_states()
        if was_running:
            try:
                self.controller.stop_sine_test()
            except Exception:
                pass
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
