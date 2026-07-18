import json
import math
import re
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from desktop.paths import CALIB_LOAD_SNAPSHOT_PATH, MOTOR_MID_CALIB_PATH, PROJECT_ROOT
from desktop.core.angle_mapping import (
    get_spec,
    urdf_manual_scaled_mapping_metadata,
)
from desktop.core.data_models import (
    HandModel,
    JointWorkspaceRange,
    resolve_joint_workspace_range,
)
from desktop.core.protocol import (
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
    ENCODER_COUNT,
    HAND_PROFILE,
    MOTOR_COUNT,
    DUAL_TENDON_JOINT_INDICES,
    DUAL_TENDON_LUT_MAX_SAMPLES,
    VALID_MOTOR_MASK,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
    SYSTEM_STATE_UNCONTROLLABLE,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_PAUSED,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_ROPE_INIT_RUNNING,
    SYSTEM_STATE_READY,
    SYSTEM_FAULT_MULTITURN_UNKNOWN,
    ROPE_INIT_STATUS_RUNNING,
    ROPE_INIT_STATUS_SUCCESS,
    ROPE_INIT_STATUS_FAILED,
    ROPE_INIT_REASON_NONE,
    ROPE_INIT_REASON_TIMEOUT,
    ROPE_INIT_REASON_FEEDBACK_STALE,
    ROPE_INIT_REASON_SERVO_OFFLINE,
    ROPE_INIT_REASON_MULTITURN_UNKNOWN,
    ROPE_INIT_REASON_MANUAL_REQUIRED,
    ROPE_INIT_REASON_AUTO_REQUIRED,
    ROPE_INIT_REASON_TARGET_PUBLISH_FAILED,
    ROPE_INIT_REASON_SYSTEM_FAULT,
    ROPE_INIT_REASON_CANCELLED,
    ROPE_INIT_REASON_TASK_START_FAILED,
    manual_encoder_reference_entry_crc,
)
from .ui_qt_compat import QtCore, QtGui, QtWidgets, Signal
from .ui_device_check_view import DeviceCheckView
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
MOTOR_RAW_HW_MISMATCH_TOL = 64
MOTOR_RELEASE_HALF_TURN_COUNTS = MOTOR_SINGLE_TURN_STEPS // 2
MOTOR_JUMP_RESYNC_THRESH = 2048
MOTOR_SLIDER_WRAP_MARGIN = 512
MOTOR_SLIDER_MAX_TARGET_DELTA = MOTOR_SINGLE_TURN_STEPS
MOTOR_DIR_HINT_CLEAR_TOL = 3
MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES = 4
MOTOR_ABS_LIMIT_RECOVER_MARGIN = 16
ROPE_INIT_START_TIMEOUT_S = 3.0
ROPE_INIT_REASON_LABELS = {
    ROPE_INIT_REASON_NONE: "no error",
    ROPE_INIT_REASON_TIMEOUT: "timeout",
    ROPE_INIT_REASON_FEEDBACK_STALE: "feedback stale",
    ROPE_INIT_REASON_SERVO_OFFLINE: "servo offline",
    ROPE_INIT_REASON_MULTITURN_UNKNOWN: "multi-turn unknown",
    ROPE_INIT_REASON_MANUAL_REQUIRED: "Manual endpoints required",
    ROPE_INIT_REASON_AUTO_REQUIRED: "Auto v3 calibration required",
    ROPE_INIT_REASON_TARGET_PUBLISH_FAILED: "target publish failed",
    ROPE_INIT_REASON_SYSTEM_FAULT: "system fault",
    ROPE_INIT_REASON_CANCELLED: "cancelled",
    ROPE_INIT_REASON_TASK_START_FAILED: "task start failed",
}
TEST_ROW_COUNT = 5
TEST_TICK_MS = 10
TEST_DEFAULT_FREQ_HZ = 0.20
TEST_DEFAULT_PHASE_DEG = 0.0
TEST_DEFAULT_HOLD_S = 0.0
FF_TEST_DEFAULT_TARGETS_DEG = "20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40"
FF_TEST_DEFAULT_STEP_S = 1.0
TRI_TEST_DEFAULT_FREQ_HZ = 0.20
JOINT_TARGET_MIN_DEG_BY_INDEX = [get_spec(i).urdf_range_deg[0] for i in range(ENCODER_COUNT)]
JOINT_TARGET_MAX_DEG_BY_INDEX = [get_spec(i).urdf_range_deg[1] for i in range(ENCODER_COUNT)]
try:
    WORKSPACE_MAPPING_VERSION, WORKSPACE_URDF_CRC = urdf_manual_scaled_mapping_metadata()
except (OSError, TypeError, ValueError):
    WORKSPACE_MAPPING_VERSION, WORKSPACE_URDF_CRC = 0, 0

OPEN_POSE_CURRENT_MIN_OFFSET_DEG = 5.0


def _load_default_auto_calibration_joint_indices() -> List[int]:
    fallback = [
        0, 1, 2, 3,
        4, 5, 6, 7,
        8, 9, 10, 11,
        12, 13, 14, 15,
        20, 16, 17, 18, 19,
    ]
    try:
        repo_root = PROJECT_ROOT
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

FINGER_WORKFLOW_JOINT_ORDER = {
    # Thumb workflow order follows the calibration sequence requested for the
    # thumb mechanics. UI tables still display joints by numeric Jxx order.
    "Thumb J16-J20": [20, 16, 17, 18, 19],
}


def _joint_indices_for_group(group_label: str, joint_start: int, joint_end: int) -> List[int]:
    return list(range(joint_start, joint_end + 1))


def _workflow_joint_indices_for_group(group_label: str, joint_start: int, joint_end: int) -> List[int]:
    custom = FINGER_WORKFLOW_JOINT_ORDER.get(group_label)
    if custom:
        return [idx for idx in custom if joint_start <= idx <= joint_end]
    return _joint_indices_for_group(group_label, joint_start, joint_end)


def _joint_group_mask(group_label: str, joint_start: int, joint_end: int) -> int:
    mask = 0
    for joint_idx in _joint_indices_for_group(group_label, joint_start, joint_end):
        mask |= (1 << joint_idx)
    return mask


def _ordered_joint_indices_from_mask(joint_mask: int) -> List[int]:
    ordered: List[int] = []
    seen = set()
    for group_label, joint_start, joint_end in FINGER_JOINT_RANGES:
        for joint_idx in _workflow_joint_indices_for_group(group_label, joint_start, joint_end):
            if (int(joint_mask) & (1 << joint_idx)) != 0:
                ordered.append(joint_idx)
                seen.add(joint_idx)
    for joint_idx in range(ENCODER_COUNT):
        if joint_idx not in seen and (int(joint_mask) & (1 << joint_idx)) != 0:
            ordered.append(joint_idx)
    return ordered


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


def _antagonist_motor_row_groups(motor_rows: List[int]) -> List[Tuple[int, int, List[int], int]]:
    groups: List[Tuple[int, int, List[int], int]] = []
    row = 0
    while row < len(motor_rows):
        motor_idx = int(motor_rows[row])
        joint_idx = (
            int(MOTOR_TO_JOINT_INDEX[motor_idx])
            if 0 <= motor_idx < len(MOTOR_TO_JOINT_INDEX)
            else -1
        )
        end = row + 1
        while end < len(motor_rows):
            next_motor = int(motor_rows[end])
            next_joint = (
                int(MOTOR_TO_JOINT_INDEX[next_motor])
                if 0 <= next_motor < len(MOTOR_TO_JOINT_INDEX)
                else -1
            )
            if next_joint != joint_idx:
                break
            end += 1
        motors = [int(motor_rows[idx]) for idx in range(row, end)]
        if joint_idx >= 0 and len(motors) == 2 and all(_motor_channel_valid(idx) for idx in motors):
            groups.append((row, end - 1, motors, joint_idx))
        row = end
    return groups


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

class ControlPanel(QtWidgets.QWidget):
    startup_device_check_finished = Signal(int, str, bool, str)

    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.backend = context.backend
        self._updating = False
        self._interaction_enabled = False
        self._control_widgets: List[QtWidgets.QWidget] = []
        self._state: Optional[HandModel] = None
        self._estop_latched = False

        self.joint_current_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_encoder_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_abs_limit_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_status_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_min_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_max_items: List[QtWidgets.QTableWidgetItem] = []
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
        self._calibration_warning_popup_mask = 0
        self._last_calib_snapshot_signature: Optional[str] = None
        self._last_joint_calib_valid_mask: Optional[int] = None
        self._startup_device_check_epoch = 0
        self._startup_device_check_running = False
        self._startup_missing_warning_shown = False
        self._pending_calib_joint_mask = 0
        self._pending_calib_motor_mask = 0
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
        self.motor_hw_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_abs_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_load_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_joint_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_target_spins: List[QtWidgets.QSpinBox] = []
        self.motor_sliders: List[QtWidgets.QSlider] = []
        self.motor_send_buttons: List[QtWidgets.QPushButton] = []
        self._motor_target_user_set: List[bool] = []
        self._motor_target_initialized = False
        self.motor_use_checks: List[Optional[QtWidgets.QCheckBox]] = [None] * MOTOR_COUNT
        self.motor_group_use_checks: List[QtWidgets.QCheckBox] = []
        self.abs_limit_lock_toggle_btn: Optional[QtWidgets.QPushButton] = None
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
        self._turn_recovery_popup_episode_active = False
        self._turn_recovery_popup_episode_mask = 0
        self._last_servo_online_bitmap: Optional[int] = None
        self._last_encoder_online_bitmap: Optional[int] = None
        self._last_encoder_warn_bitmap: Optional[int] = None
        self._last_encoder_fault_bitmap: Optional[int] = None
        self._feedback_abs_limit_popup_episode_active = False
        self._abs_limit_warning_unlocked_mask = 0

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
        self._p4_rope_init_phase = "idle"
        self._p4_rope_init_joint_mask = 0
        self._p4_rope_init_requested_at = 0.0
        self._p4_rope_init_transition_base = 0
        self._p4_rope_init_sequence_base = 0
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
        self.startup_device_check_finished.connect(
            self._on_startup_device_check_finished
        )
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
            "Ask the P4 Rope Init task to move selected IDLE joints into their calibrated workspace"
        )
        self.nvs_control_btn = QtWidgets.QPushButton("Flash ON")
        self.nvs_control_btn.setToolTip("Toggle P4 Flash/NVS multi-turn persistence. P4 NVS is the only restore source.")
        self.enable_btn.setMinimumHeight(46)
        self.stop_btn.setMinimumHeight(46)
        self.pause_btn.setMinimumHeight(46)
        for btn in (
            self.calib_btn,
            self.reset_btn,
            self.rope_init_btn,
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
        dev_layout.addWidget(self.rope_init_btn, 1, 3)
        dev_layout.addWidget(self.nvs_control_btn, 1, 4)
        dev_layout.setRowStretch(0, 1)
        dev_layout.setRowStretch(1, 1)
        for col in range(5):
            dev_layout.setColumnStretch(col, 1)
        layout.addWidget(device)
        self._update_connection_lights()

        self.enable_btn.clicked.connect(lambda _checked=False: self.start_controller())
        self.stop_btn.clicked.connect(lambda _checked=False: self.stop_controller())
        self.pause_btn.clicked.connect(lambda _checked=False: self.pause_controller())
        self.reset_btn.clicked.connect(lambda _checked=False: self.clear_fault_with_confirm())
        self.calib_btn.clicked.connect(lambda _checked=False: self.calibrate())
        self.rope_init_btn.clicked.connect(lambda _checked=False: self.rope_init_controller())
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
        self.new_device_reset_btn = None
        self.calib_load_btn = None
        self.open_btn = None
        self.release_half_turn_btn = None
        self.tighten_half_turn_btn = None
        self.fist_btn = None
        self.zero_btn = None
        self.tabs = None
        self.device_check_view = None
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
        self.joint_abs_limit_items = []
        self.joint_status_items = []
        self.joint_min_items = []
        self.joint_max_items = []
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
        self.motor_hw_items = []
        self.motor_abs_items = []
        self.motor_load_items = []
        self.motor_joint_items = []
        self.motor_target_spins = []
        self.motor_sliders = []
        self.motor_send_buttons = []
        self.abs_limit_lock_toggle_btn = None
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
        self.manual_check = QtWidgets.QCheckBox("Control Enable")
        self.manual_check.setChecked(False)
        self.send_all_btn = QtWidgets.QPushButton("Send All")
        self.new_device_reset_btn = QtWidgets.QPushButton("New Device Reset")
        self.new_device_reset_btn.setToolTip(
            "Clear saved multi-turn data and set each online motor's current raw position as turn0"
        )
        self.calib_load_btn = QtWidgets.QPushButton("Calib Load")
        self.calib_load_btn.setToolTip("Load the last saved calibration snapshot when mechanics have not changed")
        self.open_btn = QtWidgets.QPushButton("Open Pose")
        self.release_half_turn_btn = QtWidgets.QPushButton("Release Half Turn")
        self.tighten_half_turn_btn = QtWidgets.QPushButton("Tighten Half Turn")
        self.fist_btn = QtWidgets.QPushButton("Fist Pose")
        self.zero_btn = QtWidgets.QPushButton("Zero Pose")
        wf_layout.addWidget(self.manual_check)
        wf_layout.addStretch(1)
        for btn in [
            self.new_device_reset_btn,
            self.calib_load_btn,
            self.open_btn,
            self.release_half_turn_btn,
            self.tighten_half_turn_btn,
            self.fist_btn,
            self.zero_btn,
            self.send_all_btn,
        ]:
            wf_layout.addWidget(btn)
        layout.addWidget(workflow)

        self.manual_check.toggled.connect(self._apply_interaction_enabled)
        self.send_all_btn.clicked.connect(lambda _checked=False: self.send_all())
        self.new_device_reset_btn.clicked.connect(lambda _checked=False: self.new_device_reset())
        self.calib_load_btn.clicked.connect(lambda _checked=False: self.load_calibration_snapshot())
        self.open_btn.clicked.connect(lambda _checked=False: self.apply_open_pose())
        self.release_half_turn_btn.clicked.connect(lambda _checked=False: self.release_all_motors_half_turn())
        self.tighten_half_turn_btn.clicked.connect(lambda _checked=False: self.tighten_all_motors_half_turn())
        self.fist_btn.clicked.connect(lambda _checked=False: self.apply_fist_pose())
        self.zero_btn.clicked.connect(lambda _checked=False: self.apply_zero_pose())

        self.tabs = QtWidgets.QTabWidget()
        self.device_check_view = DeviceCheckView(
            self.context,
            mapping_version=WORKSPACE_MAPPING_VERSION,
            urdf_crc=WORKSPACE_URDF_CRC,
        )
        self.device_check_view.operation_finished.connect(
            self._on_device_check_operation_finished
        )
        self.joint_table = self._build_joint_table()
        self.motor_table = self._build_motor_table()
        joint_page = QtWidgets.QWidget()
        joint_page_layout = QtWidgets.QVBoxLayout(joint_page)
        joint_page_layout.setContentsMargins(0, 0, 0, 0)
        joint_page_layout.setSpacing(4)
        abs_limit_bar = QtWidgets.QWidget()
        abs_limit_layout = QtWidgets.QHBoxLayout(abs_limit_bar)
        abs_limit_layout.setContentsMargins(6, 2, 6, 2)
        abs_limit_layout.addWidget(QtWidgets.QLabel("Feedback ABS Limit (default unlocked)"))
        abs_limit_layout.addStretch(1)
        self.abs_limit_lock_toggle_btn = QtWidgets.QPushButton("LOCK")
        self.abs_limit_lock_toggle_btn.setMinimumWidth(86)
        self.abs_limit_lock_toggle_btn.clicked.connect(
            lambda _checked=False: self.toggle_abs_limit_lock()
        )
        abs_limit_layout.addWidget(self.abs_limit_lock_toggle_btn)
        joint_page_layout.addWidget(abs_limit_bar)
        joint_page_layout.addWidget(self.joint_table, 1)
        self._sync_scope_group_checks()
        self.test_panel = self._build_test_panel()
        self.tabs.addTab(self.device_check_view, "Device Check")
        self.tabs.addTab(joint_page, "Joint Channels")
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

    @staticmethod
    def _set_item_text_if_changed(item: Optional[QtWidgets.QTableWidgetItem], text: str) -> None:
        if item is not None and item.text() != str(text):
            item.setText(str(text))

    @staticmethod
    def _set_item_tooltip_if_changed(item: Optional[QtWidgets.QTableWidgetItem], text: str) -> None:
        if item is not None and item.toolTip() != str(text):
            item.setToolTip(str(text))

    @staticmethod
    def _set_spin_value_if_changed(spin, value, tol: float = 0.0) -> None:
        if spin is None:
            return
        current = float(spin.value())
        target = float(value)
        if abs(current - target) <= float(tol):
            return
        was_blocked = spin.blockSignals(True)
        try:
            spin.setValue(value)
        finally:
            spin.blockSignals(was_blocked)

    @staticmethod
    def _set_slider_value_if_changed(slider: Optional[QtWidgets.QSlider], value: int) -> None:
        if slider is None:
            return
        target = int(value)
        if int(slider.value()) == target:
            return
        was_blocked = slider.blockSignals(True)
        try:
            slider.setValue(target)
        finally:
            slider.blockSignals(was_blocked)

    @staticmethod
    def _set_spin_range_if_changed(spin, lo, hi) -> None:
        if spin is None:
            return
        if abs(float(spin.minimum()) - float(lo)) <= 0.0005 and abs(float(spin.maximum()) - float(hi)) <= 0.0005:
            return
        was_blocked = spin.blockSignals(True)
        try:
            spin.setRange(lo, hi)
        finally:
            spin.blockSignals(was_blocked)

    @staticmethod
    def _set_slider_range_if_changed(slider: Optional[QtWidgets.QSlider], lo: int, hi: int) -> None:
        if slider is None:
            return
        lo = int(lo)
        hi = int(hi)
        if int(slider.minimum()) == lo and int(slider.maximum()) == hi:
            return
        was_blocked = slider.blockSignals(True)
        try:
            slider.setRange(lo, hi)
        finally:
            slider.blockSignals(was_blocked)

    def _build_joint_table(self) -> QtWidgets.QTableWidget:
        joint_rows, group_ranges = _joint_rows_by_finger_group()
        table = QtWidgets.QTableWidget(len(joint_rows), 12)
        table.setWordWrap(False)
        table.setHorizontalHeaderLabels([
            "Group", "Use", "Joint", "Enc", "Current", "Raw", "ABS Limit", "Min", "Target", "Max",
            "Action", "Target Slider",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(False)
        self.joint_current_items = [None] * ENCODER_COUNT
        self.joint_encoder_items = [None] * ENCODER_COUNT
        self.joint_abs_limit_items = [None] * ENCODER_COUNT
        self.joint_status_items = [None] * ENCODER_COUNT
        self.joint_min_items = [None] * ENCODER_COUNT
        self.joint_max_items = [None] * ENCODER_COUNT
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
            abs_limit = self._make_item("-")
            min_item = self._make_item(f"{self._joint_target_min_deg(i):.2f}")
            max_item = self._make_item(f"{self._joint_target_max_deg(i):.2f}")
            for item in (status, enc, cur, abs_limit, min_item, max_item):
                apply_group_background(item, row)
            self.joint_status_items[i] = status
            self.joint_current_items[i] = cur
            self.joint_encoder_items[i] = enc
            self.joint_abs_limit_items[i] = abs_limit
            self.joint_min_items[i] = min_item
            self.joint_max_items[i] = max_item
            table.setItem(row, 3, status)
            table.setItem(row, 4, cur)
            table.setItem(row, 5, enc)
            table.setItem(row, 6, abs_limit)
            table.setItem(row, 7, min_item)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(self._joint_target_min_deg(i), self._joint_target_max_deg(i))
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=i: self._joint_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: self._mark_joint_target_user_set(idx))
            apply_group_widget_background(spin, row)
            self.joint_target_spins[i] = spin
            table.setCellWidget(row, 8, spin)
            table.setItem(row, 9, max_item)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_joint_row(idx))
            apply_group_widget_background(send, row)
            self.joint_send_buttons[i] = send
            table.setCellWidget(row, 10, send)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(self._joint_target_min_deg(i) * 100), int(self._joint_target_max_deg(i) * 100))
            slider.valueChanged.connect(lambda value, idx=i: self._joint_slider_changed(idx, value))
            apply_group_widget_background(slider, row)
            self.joint_sliders[i] = slider
            table.setCellWidget(row, 11, slider)
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
        table.setColumnWidth(6, 190)
        table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(7, 64)
        table.horizontalHeader().setSectionResizeMode(8, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(8, 78)
        table.horizontalHeader().setSectionResizeMode(9, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(9, 60)
        table.horizontalHeader().setSectionResizeMode(10, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(10, 82)
        table.horizontalHeader().setSectionResizeMode(11, QtWidgets.QHeaderView.Stretch)
        table.setColumnWidth(11, 420)
        return table

    def _build_motor_table(self) -> QtWidgets.QTableWidget:
        motor_rows, group_ranges = _motor_rows_by_finger_group()
        antagonist_groups = _antagonist_motor_row_groups(motor_rows)
        antagonist_by_row = {}
        antagonist_start_rows = set()
        for start, end, motors, joint_idx in antagonist_groups:
            antagonist_start_rows.add(start)
            group = (start, end, motors, joint_idx)
            for row in range(start, end + 1):
                antagonist_by_row[row] = group
        table = QtWidgets.QTableWidget(len(motor_rows), 14)
        table.setHorizontalHeaderLabels([
            "Group", "Use", "Motor", "Turn", "RawNow", "HWNow", "AbsNow", "Load", "Target(rel)", "Action",
            "-", "Raw Slider", "+", "Joint Angle",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(False)
        self.motor_turn_items = [None] * MOTOR_COUNT
        self.motor_raw_items = [None] * MOTOR_COUNT
        self.motor_hw_items = [None] * MOTOR_COUNT
        self.motor_abs_items = [None] * MOTOR_COUNT
        self.motor_load_items = [None] * MOTOR_COUNT
        self.motor_joint_items = [None] * MOTOR_COUNT
        self.motor_target_spins = [None] * MOTOR_COUNT
        self.motor_sliders = [None] * MOTOR_COUNT
        self.motor_send_buttons = [None] * MOTOR_COUNT
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
            joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
            antagonist = antagonist_by_row.get(row)
            if antagonist is None:
                motor_item = self._make_item(f"M{i:02d}")
                apply_group_background(motor_item, row)
                if group_label:
                    motor_item.setToolTip(
                        f"{group_label}; mapped joint J{int(joint_idx):02d}"
                        if int(joint_idx) >= 0
                        else group_label
                    )
                table.setItem(row, 2, motor_item)
                use_cell, use_check = self._make_use_checkbox_cell(
                    (self._selected_motor_mask & (1 << i)) != 0,
                    f"Use M{i:02d} in Manual Motor Channels active scope",
                )
                apply_group_widget_background(use_cell, row)
                use_check.toggled.connect(lambda checked, idx=i: self._on_motor_use_changed(idx, checked))
                self.motor_use_checks[i] = use_check
                table.setCellWidget(row, 1, use_cell)
            elif row in antagonist_start_rows:
                pair_start, pair_end, pair_motors, pair_joint = antagonist

                pair_cell = QtWidgets.QWidget()
                pair_layout = QtWidgets.QGridLayout(pair_cell)
                pair_layout.setContentsMargins(0, 0, 0, 0)
                pair_layout.setHorizontalSpacing(0)
                pair_layout.setVerticalSpacing(0)
                pair_layout.setColumnMinimumWidth(0, 44)
                pair_layout.setColumnMinimumWidth(1, 58)
                pair_layout.setColumnStretch(0, 0)
                pair_layout.setColumnStretch(1, 1)
                for pair_row, motor_idx in enumerate(pair_motors):
                    check_row, use_check = self._make_use_checkbox_cell(
                        (self._selected_motor_mask & (1 << motor_idx)) != 0,
                        f"Use M{motor_idx:02d} in Manual Motor Channels active scope",
                    )
                    use_check.toggled.connect(
                        lambda checked, idx=motor_idx: self._on_motor_use_changed(idx, checked)
                    )
                    self.motor_use_checks[motor_idx] = use_check
                    pair_layout.addWidget(check_row, pair_row, 0)

                    label = QtWidgets.QLabel(f"M{motor_idx:02d}")
                    label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
                    label.setContentsMargins(4, 0, 0, 0)
                    label.setToolTip(
                        f"J{pair_joint:02d} antagonist motor; feedback and commands remain independent"
                    )
                    font = label.font()
                    font.setBold(True)
                    label.setFont(font)
                    pair_layout.addWidget(label, pair_row, 1)
                    pair_layout.setRowStretch(pair_row, 1)
                apply_group_widget_background(pair_cell, row)
                table.setCellWidget(row, 1, pair_cell)
                table.setSpan(pair_start, 1, pair_end - pair_start + 1, 2)
            turn = self._make_item("-")
            raw_now = self._make_item("-")
            hw_now = self._make_item("-")
            abs_now = self._make_item("-")
            load = self._make_item("-")
            joint = self._make_item("-")
            for item in (turn, raw_now, hw_now, abs_now, load, joint):
                apply_group_background(item, row)
            self.motor_turn_items[i] = turn
            self.motor_raw_items[i] = raw_now
            self.motor_hw_items[i] = hw_now
            self.motor_abs_items[i] = abs_now
            self.motor_load_items[i] = load
            self.motor_joint_items[i] = joint
            table.setItem(row, 3, turn)
            table.setItem(row, 4, raw_now)
            table.setItem(row, 5, hw_now)
            table.setItem(row, 6, abs_now)
            table.setItem(row, 7, load)
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
            minus = QtWidgets.QPushButton("-")
            minus.setMaximumWidth(34)
            minus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, -1))
            apply_group_widget_background(minus, row)
            table.setCellWidget(row, 10, minus)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimumWidth(72)
            slider.setRange(MOTOR_TARGET_SCALE_LO, MOTOR_TARGET_SCALE_HI)
            slider.sliderPressed.connect(lambda idx=i: self._on_motor_slider_press(idx))
            slider.sliderReleased.connect(lambda idx=i: self._on_motor_slider_release(idx))
            slider.valueChanged.connect(lambda value, idx=i: self._motor_slider_changed(idx, value))
            apply_group_widget_background(slider, row)
            self.motor_sliders[i] = slider
            table.setCellWidget(row, 11, slider)
            plus = QtWidgets.QPushButton("+")
            plus.setMaximumWidth(34)
            plus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, 1))
            apply_group_widget_background(plus, row)
            table.setCellWidget(row, 12, plus)
            table.setItem(row, 13, joint)
            self._control_widgets.extend([spin, send, minus, slider, plus])
            if not _motor_channel_valid(i):
                table.setRowHidden(row, True)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(0, 152)
        table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(1, 44)
        table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(2, 58)
        for col in (4, 5, 6):
            table.horizontalHeader().setSectionResizeMode(col, QtWidgets.QHeaderView.Fixed)
            table.setColumnWidth(col, 76)
        table.horizontalHeader().setSectionResizeMode(8, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(8, 126)
        table.horizontalHeader().setSectionResizeMode(11, QtWidgets.QHeaderView.Stretch)
        table.setColumnWidth(11, 150)
        table.horizontalHeader().setSectionResizeMode(13, QtWidgets.QHeaderView.Fixed)
        table.setColumnWidth(13, 82)
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
        self.ff_joint_combo.currentTextChanged.connect(
            lambda _text: self._refresh_single_test_workspace("feedforward")
        )
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
        self.tri_joint_combo.currentTextChanged.connect(
            lambda _text: self._refresh_single_test_workspace("triangle")
        )
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
        if widget is getattr(self, "device_check_view", None):
            return "device_check"
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
        elif current == "device_check":
            if self.device_check_view is not None:
                self.device_check_view.update_state(self._state)
        else:
            self._refresh_joint_feedback_view(sync_target=True)
        self._last_control_mode = current
        self._apply_control_mode_policy()
        self._apply_active_scope_for_current_mode()
        self._update_manual_action_buttons()

    def _on_device_check_operation_finished(
        self, _name: str, _ok: bool, _message: str
    ) -> None:
        """Publish Device Check readback into the parent panel immediately."""
        try:
            state = self.backend.get_state_snapshot()
        except Exception:
            return
        if state is None:
            return
        self._state = state
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
        if getattr(self, "joint_target_spins", None):
            for idx in range(min(ENCODER_COUNT, len(self.joint_target_spins))):
                self._refresh_joint_target_range(idx, self._state)
        self._update_manual_action_buttons()
        self._apply_control_mode_policy()
        self._set_status(f"Control Enable: {'ON' if self._interaction_enabled else 'OFF'}")
        if not self._interaction_enabled:
            self.stop_all_tests("stopped: Control Enable OFF", update_status=False)

    def _apply_control_mode_policy(self) -> None:
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        if mode == "device_check":
            self.backend.set_pid_control(False)
        elif mode == "motor":
            self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.backend.set_pid_control(False)
        else:
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.backend.set_pid_control(bool(self._interaction_enabled))

    def _update_manual_action_buttons(self) -> None:
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        interaction = bool(self._interaction_enabled)
        system_state = getattr(self._state, "system_state", None) if self._state is not None else None
        active_joints = self._active_scope_joint_indices()
        joint_workspace_ready = bool(active_joints) and not self._active_scope_joint_calib_missing_indices()
        if getattr(self, "manual_check", None) is not None:
            self.manual_check.setVisible(mode != "device_check")
            self.manual_check.setEnabled(mode != "device_check")
        for btn in [getattr(self, "send_all_btn", None)]:
            if btn is not None:
                btn.setVisible(mode != "motor")
                # Keep the action reachable so its command guard can report why
                # feedback is not ready instead of leaving a silent grey button.
                btn.setEnabled(interaction and mode != "motor" and joint_workspace_ready)
        if getattr(self, "open_btn", None) is not None:
            self.open_btn.setVisible(mode in ("joint", "test"))
            self.open_btn.setEnabled(mode in ("joint", "test") and joint_workspace_ready)
        if getattr(self, "release_half_turn_btn", None) is not None:
            self.release_half_turn_btn.setVisible(mode == "motor")
            self.release_half_turn_btn.setEnabled(interaction and mode == "motor")
        if getattr(self, "tighten_half_turn_btn", None) is not None:
            self.tighten_half_turn_btn.setVisible(mode == "motor")
            self.tighten_half_turn_btn.setEnabled(interaction and mode == "motor")
        if getattr(self, "calib_load_btn", None) is not None:
            can_load = (
                bool(self.context.is_comm_connected()) and
                CALIB_LOAD_SNAPSHOT_PATH.exists() and
                system_state not in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK)
            )
            self.calib_load_btn.setEnabled(can_load)
        if getattr(self, "new_device_reset_btn", None) is not None:
            multi_turn_unknown_mask = (
                int(getattr(self._state, "multi_turn_unknown_bitmap", 0) or 0)
                & VALID_MOTOR_MASK
            )
            can_reset_device = (
                bool(self.context.is_comm_connected()) and
                multi_turn_unknown_mask != 0 and
                system_state not in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK)
            )
            self.new_device_reset_btn.setEnabled(can_reset_device)
        for btn in [getattr(self, "fist_btn", None), getattr(self, "zero_btn", None)]:
            if btn is not None:
                btn.setVisible(mode in ("joint", "test"))
                btn.setEnabled(mode in ("joint", "test") and joint_workspace_ready)
        if getattr(self, "rope_init_btn", None) is not None:
            self.rope_init_btn.setEnabled(
                interaction and
                mode == "joint" and
                joint_workspace_ready and
                self._p4_rope_init_phase == "idle"
            )
        if getattr(self, "calib_btn", None) is not None:
            connected = bool(self.context.is_comm_connected())
            task_busy = system_state in (
                SYSTEM_STATE_CALIBRATION_RUNNING,
                SYSTEM_STATE_ROPE_INIT_RUNNING,
            )
            # Do not permanently grey this action from a cached Manual packet.
            # calibrate() performs a fresh P4 query and reports the exact blocker.
            self.calib_btn.setEnabled(connected and mode == "joint" and not task_busy)
            missing_manual = self._active_scope_manual_missing_indices()
            if missing_manual:
                self.calib_btn.setToolTip(
                    "Click to refresh Manual endpoints from P4 and validate: "
                    + self._format_joint_indices(missing_manual)
                )
            elif system_state not in (
                SYSTEM_STATE_UNCONTROLLABLE,
                SYSTEM_STATE_IDLE,
                SYSTEM_STATE_READY,
            ):
                self.calib_btn.setToolTip(
                    "Calibration requires UNCONTROLLABLE, IDLE, or READY; "
                    "click to see the current-state blocker"
                )
            else:
                self.calib_btn.setToolTip(
                    "Start automatic calibration for the selected joints; "
                    "Control Enable is not a calibration prerequisite"
                )
        if getattr(self, "enable_btn", None) is not None:
            self.enable_btn.setEnabled(
                mode != "device_check" and
                (mode == "motor" or joint_workspace_ready)
            )
        if getattr(self, "mid_calib_btn", None) is not None:
            self.mid_calib_btn.setEnabled(bool(self.context.is_comm_connected()))
        has_active_motor = (self._active_motor_mask_from_ui() & VALID_MOTOR_MASK) != 0
        abs_limit_mode = mode in ("joint", "test")
        if getattr(self, "abs_limit_lock_toggle_btn", None) is not None:
            self._refresh_abs_limit_lock_toggle_button()
            self.abs_limit_lock_toggle_btn.setEnabled(
                abs_limit_mode and bool(self.context.is_comm_connected()) and has_active_motor
            )
        if getattr(self, "pause_btn", None) is not None:
            self.pause_btn.setEnabled(system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK))
        self._set_test_button_states()

    def sync_app_mode(self, mode_name: str) -> None:
        mode = str(mode_name or "").lower()
        if "teleop" in mode:
            self._build_teleop_body()
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.backend.set_pid_control(True)
        elif "algorithm" in mode:
            self._clear_body()
            self._app_mode = "algorithm"
            self._algorithm_view = AlgorithmView(self.context)
            self._body_widget = self._algorithm_view
            self.body_layout.addWidget(self._body_widget)
            self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.backend.set_pid_control(True)
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
                if self._send_motor_abs_from_ui("Send All"):
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
            self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.backend.set_pid_control(False)
            if self._estop_latched and not self.backend.is_started():
                self._set_status("Release blocked: press ENABLE after E-STOP")
                return
            if not self.backend.is_started():
                self._set_status("Release blocked: waiting for controllable IDLE after ENABLE")
                return
            unknown_mask = self._active_scope_multiturn_unknown_mask()
            if unknown_mask != 0:
                self._show_turn_recovery_guide(unknown_mask, "Release Half Turn")
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

            self.backend.set_motor_positions_absolute(targets)
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

    def tighten_all_motors_half_turn(self) -> None:
        if not self._interaction_enabled:
            return
        if self._manual_submode() != "motor":
            self._set_status("Tighten Half Turn is only available in Motor Channels")
            return
        try:
            if not self.context.is_comm_connected():
                raise RuntimeError("Serial port is not connected")
            self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.backend.set_pid_control(False)
            if self._estop_latched and not self.backend.is_started():
                self._set_status("Tighten blocked: press ENABLE after E-STOP")
                return
            if not self.backend.is_started():
                self._set_status("Tighten blocked: waiting for controllable IDLE after ENABLE")
                return
            unknown_mask = self._active_scope_multiturn_unknown_mask()
            if unknown_mask != 0:
                self._show_turn_recovery_guide(unknown_mask, "Tighten Half Turn")
                return
            state = self._state
            if state is None or not bool(getattr(state, "has_servo_angle_data", False)):
                self._set_status("Tighten blocked: waiting for motor ABS feedback")
                return

            active_mask = self._active_motor_mask_from_ui() & int(VALID_MOTOR_MASK) & ((1 << MOTOR_COUNT) - 1)
            if active_mask == 0:
                self._set_status("Tighten blocked: no active motor selected")
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
                targets[idx] = motor_abs_after_tendon_action(current, idx, MOTOR_RELEASE_HALF_TURN_COUNTS)

            if missing:
                self._set_status(f"Tighten blocked: waiting ABS feedback {','.join(missing[:4])}")
                return

            self.backend.set_motor_positions_absolute(targets)
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
            self.context.emit_action(f"Tighten Half Turn sent: {preview}{suffix}")
            self._set_status(f"Tightened {active_count} active motor(s) by 1/2 turn ({MOTOR_RELEASE_HALF_TURN_COUNTS} counts)")
        except Exception as exc:
            self._error("Tighten Failed", f"{type(exc).__name__}: {exc}")

    def apply_preset(self, angles: List[float], joint_indices: Optional[List[int]] = None) -> None:
        clamped_angles = list(angles[:ENCODER_COUNT])
        if len(clamped_angles) < ENCODER_COUNT:
            clamped_angles.extend([0.0] * (ENCODER_COUNT - len(clamped_angles)))
        indices = range(ENCODER_COUNT) if joint_indices is None else joint_indices
        self._updating = True
        try:
            for i in indices:
                if i < 0 or i >= ENCODER_COUNT:
                    continue
                value = clamped_angles[i]
                value = self._clamp_joint_target_deg(i, float(value))
                clamped_angles[i] = value
                self.joint_target_spins[i].setValue(value)
                self.joint_sliders[i].setValue(int(round(value * 100.0)))
                self._joint_target_user_set[i] = True
        finally:
            self._updating = False
        self.context.draw_pose_angles(clamped_angles)

    def _build_current_min_open_pose_targets(self) -> Optional[Tuple[List[float], List[int]]]:
        joints = self._active_scope_joint_indices()
        if not joints:
            self._set_status("Open/Zero Pose blocked: no active joints selected")
            return None
        targets = self.resolved_joint_targets()
        missing: List[str] = []
        for joint in joints:
            workspace = self._joint_workspace_range(joint, self._state)
            if not workspace.ready:
                missing.append(f"J{joint:02d} {workspace.reason}")
                continue
            targets[joint] = workspace.open_pose_target(OPEN_POSE_CURRENT_MIN_OFFSET_DEG)
        if missing:
            detail = "; ".join(missing[:4])
            if len(missing) > 4:
                detail += f"; +{len(missing) - 4} more"
            self._set_status(f"Open/Zero Pose blocked: {detail}")
            return None
        return targets, joints

    def apply_open_pose(self) -> None:
        pose = self._build_current_min_open_pose_targets()
        if pose is None:
            return
        targets, joints = pose
        self.apply_preset(targets, joint_indices=joints)
        self._set_status(
            "Open Pose set: active Current Min + "
            f"{OPEN_POSE_CURRENT_MIN_OFFSET_DEG:.0f} deg"
        )

    def apply_zero_pose(self) -> None:
        pose = self._build_current_min_open_pose_targets()
        if pose is None:
            return
        targets, joints = pose
        self.apply_preset(targets, joint_indices=joints)
        self._set_status(
            "Zero Pose set: active Current Min + "
            f"{OPEN_POSE_CURRENT_MIN_OFFSET_DEG:.0f} deg"
        )

    def apply_fist_pose(self) -> None:
        angles: List[float] = []
        for i in range(ENCODER_COUNT):
            max_deg = self._joint_target_max_deg(i)
            target = max(self._joint_target_min_deg(i), max_deg - 2.0)
            angles.append(self._clamp_joint_target_deg(i, target))
        self.apply_preset(angles)
        self._set_status("Fist Pose set: URDF Max-2 deg")

    def start_controller(self) -> None:
        if self._app_mode == "manual" and self._manual_submode() == "device_check":
            self._set_status("ENABLE blocked: switch to Joint or Motor Channels after Device Check")
            return
        if not self.context.is_comm_connected():
            if not self.context.connect_selected_port():
                self._update_connection_lights()
                return
            self._update_connection_lights()
        if self._manual_submode() != "motor":
            missing_workspace = self._active_scope_joint_calib_missing_indices()
            if missing_workspace:
                self._set_status(
                    "ENABLE blocked: active workspace is not ready ("
                    + self._format_joint_indices(missing_workspace)
                    + ")"
                )
                return
        if not self._apply_active_scope_for_current_mode():
            self._set_status("Enable blocked: active scope was not accepted by lower controller")
            return
        state = self._state
        if state is not None and getattr(state, "system_state", None) == SYSTEM_STATE_PAUSED:
            self._estop_latched = False
            self.backend.start()
            self.context.emit_action("Resume")
            self._set_status("Resume requested: continuing paused target")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, "ENABLE")
            return
        if not self._sync_motor_targets_to_feedback():
            self._set_status("Enable waits for fresh motor feedback")
            return
        targets = self._build_motor_abs_command_list()
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        self.backend.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        self._estop_latched = False
        self.backend.start()
        self.context.emit_action("Start with current motor feedback synced as target zero")
        self._set_status("Enable requested: waiting for IDLE controllable")

    def stop_controller(self) -> None:
        self._update_connection_lights()
        self._estop_latched = True
        self.context.cancel_algorithm("E-STOP")
        self.stop_all_tests("stopped by E-STOP", update_status=False)
        try:
            self.backend.stop()
        except Exception:
            pass
        self._clear_run_states()
        self.context.emit_action("Stop")

    def pause_controller(self) -> None:
        try:
            self.backend.pause()
        except Exception:
            pass
        self.context.emit_action("Pause")
        self._set_status("Pause requested: target cache kept for resume")

    def safe_close_controller(self) -> None:
        self.stop_all_tests("safe close", update_status=False)
        try:
            self.backend.hold_current_motor_positions(delay_s=0.25)
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
            try:
                unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0) & VALID_MOTOR_MASK
            except Exception:
                unknown_mask = 0
            try:
                fault_bitmap = int(summary.get("bitmap", 0) or 0)
            except Exception:
                fault_bitmap = 0
            if unknown_mask != 0 or (fault_bitmap & SYSTEM_FAULT_MULTITURN_UNKNOWN) != 0:
                guide_mask = unknown_mask or self._active_scope_multiturn_unknown_mask() or VALID_MOTOR_MASK
                detail = (
                    f"{detail}\n\n"
                    "Turn UNKNOWN recovery guide:\n"
                    f"{self._turn_recovery_guide_text(guide_mask, 'Clear Fault')}"
                )
            text = (
                "当前仍检测到故障：\n\n"
                f"{detail}\n\n"
                "请先确认故障原因已经排除。\n"
                "清除故障会让下位机回到不可控状态，之后需要重新按 ENABLE。"
            )
            title = "Clear Fault"
            accept_text = "清除故障"
        else:
            text = (
                "确认清除当前故障/保护状态？\n\n"
                "这会发送 reset 指令，让下位机回到不可控状态，之后需要重新按 ENABLE。"
            )
            title = "Clear Fault"
            accept_text = "清除"
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
        self._reset_p4_rope_init_progress()
        self.backend.reset()
        self._clear_run_states()
        self.context.emit_action("Clear Fault")

    def _active_scope_joint_indices(self) -> List[int]:
        data = self._current_active_scope_tuple()
        if data is not None:
            _motor_mask, joint_mask, _require_can = data
            return _ordered_joint_indices_from_mask(int(joint_mask))
        return list(AUTO_CALIBRATION_JOINT_INDICES)

    # P4 owns Rope Init motion; the desktop only submits and observes it.
    def rope_init_controller(self) -> None:
        if self._p4_rope_init_phase != "idle":
            self._set_status("Rope Init is already queued or running on P4")
            return
        if self._manual_submode() != "joint":
            self._set_status("Rope Init is only available in Joint Channels")
            return
        if not self.context.is_comm_connected():
            if not self.context.connect_selected_port():
                return
        if not self.backend.is_started():
            self._set_status("Rope Init blocked: press ENABLE to enter IDLE first")
            return
        state = self._state
        system_state = getattr(state, "system_state", None)
        if system_state not in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY):
            self._set_status("Rope Init blocked: wait for IDLE/READY")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, "Rope Init")
            return

        if not self._apply_active_scope_for_current_mode():
            self._set_status("Rope Init blocked: active scope was not accepted")
            return
        active_joint_mask = int(self._active_joint_mask_from_ui()) & (
            (1 << ENCODER_COUNT) - 1
        )
        if active_joint_mask == 0:
            self._set_status("Rope Init blocked: no active joint selected")
            return
        transition_base = int(
            getattr(state, "state_transition_count", 0) or 0
        )
        sequence_base = int(getattr(state, "rope_init_sequence", 0) or 0)
        if not self.backend.rope_init(active_joint_mask):
            self._set_status(
                "Rope Init rejected: check Manual/v3 calibration, state, and Turn"
            )
            return
        self._p4_rope_init_phase = "queued"
        self._p4_rope_init_joint_mask = active_joint_mask
        self._p4_rope_init_requested_at = time.monotonic()
        self._p4_rope_init_transition_base = transition_base
        self._p4_rope_init_sequence_base = sequence_base
        self.rope_init_btn.setText("Rope Init...")
        self.context.emit_action(
            f"Rope Init queued on P4: joint_mask=0x{active_joint_mask:08X}"
        )
        self._set_status(
            f"Rope Init queued on P4: joint_mask=0x{active_joint_mask:08X}"
        )
        self._update_manual_action_buttons()

    def _reset_p4_rope_init_progress(self) -> None:
        self._p4_rope_init_phase = "idle"
        self._p4_rope_init_joint_mask = 0
        self._p4_rope_init_requested_at = 0.0
        self._p4_rope_init_transition_base = 0
        self._p4_rope_init_sequence_base = 0
        button = getattr(self, "rope_init_btn", None)
        if button is not None:
            button.setText("Rope Init")

    def _finish_p4_rope_init_progress(self, success: bool, detail: str) -> None:
        joint_mask = self._p4_rope_init_joint_mask
        result = "complete" if success else "failed"
        message = (
            f"Rope Init {result} on P4: joint_mask=0x{joint_mask:08X}; {detail}"
        )
        self._reset_p4_rope_init_progress()
        self.context.emit_action(message)
        self._update_manual_action_buttons()

    def _update_p4_rope_init_progress(self, state) -> None:
        if self._p4_rope_init_phase == "idle" or state is None:
            return
        system_state = int(getattr(state, "system_state", -1) or 0)
        transition_count = int(getattr(state, "state_transition_count", 0) or 0)
        transition_delta = max(
            0, transition_count - self._p4_rope_init_transition_base
        )
        elapsed = max(0.0, time.monotonic() - self._p4_rope_init_requested_at)

        if bool(getattr(state, "has_motion_result_status", False)):
            rope_sequence = int(getattr(state, "rope_init_sequence", 0) or 0)
            rope_status = int(getattr(state, "rope_init_status", 0) or 0)
            rope_reason = int(getattr(state, "rope_init_reason", 0) or 0)
            if rope_sequence > self._p4_rope_init_sequence_base:
                if rope_status == ROPE_INIT_STATUS_RUNNING:
                    if self._p4_rope_init_phase != "running":
                        self._p4_rope_init_phase = "running"
                        self.context.emit_action(
                            "Rope Init started on P4: "
                            f"joint_mask=0x{self._p4_rope_init_joint_mask:08X}"
                        )
                        self._update_manual_action_buttons()
                    self._set_status(f"Rope Init running on P4: {elapsed:.1f} s")
                    return
                if rope_status == ROPE_INIT_STATUS_SUCCESS:
                    self._finish_p4_rope_init_progress(
                        True, "P4 reported SUCCESS"
                    )
                    return
                if rope_status == ROPE_INIT_STATUS_FAILED:
                    reason_label = ROPE_INIT_REASON_LABELS.get(
                        rope_reason, f"unknown reason {rope_reason}"
                    )
                    self._finish_p4_rope_init_progress(
                        False, f"P4 reported FAILED: {reason_label}"
                    )
                    return

        if system_state == SYSTEM_STATE_ROPE_INIT_RUNNING:
            if self._p4_rope_init_phase != "running":
                self._p4_rope_init_phase = "running"
                self.context.emit_action(
                    "Rope Init started on P4: "
                    f"joint_mask=0x{self._p4_rope_init_joint_mask:08X}"
                )
                self._update_manual_action_buttons()
            self._set_status(f"Rope Init running on P4: {elapsed:.1f} s")
            return

        task_has_finished = (
            self._p4_rope_init_phase == "running" or transition_delta >= 2
        )
        if task_has_finished:
            if system_state == SYSTEM_STATE_READY:
                self._finish_p4_rope_init_progress(True, "READY confirmed")
            elif system_state in (SYSTEM_STATE_IDLE, SYSTEM_STATE_UNCONTROLLABLE):
                self._finish_p4_rope_init_progress(
                    False,
                    f"final_state={system_state}; active joints did not reach READY",
                )
            return

        if elapsed >= ROPE_INIT_START_TIMEOUT_S:
            self._finish_p4_rope_init_progress(
                False,
                f"P4 did not enter ROPE_INIT within {ROPE_INIT_START_TIMEOUT_S:.1f} s",
            )
            return
        self._set_status(f"Rope Init queued on P4: waiting to start ({elapsed:.1f} s)")

    def calibrate(self) -> None:
        if self._manual_submode() != "joint":
            self._set_status("Calibrate is only available in Joint Channels")
            return
        if not bool(self.context.is_comm_connected()):
            self._warn("Calibration unavailable", "P4 is not connected.")
            self._set_status("Calibrate blocked: P4 is not connected")
            return

        try:
            latest_state = self.backend.get_state_snapshot()
        except Exception:
            latest_state = None
        if latest_state is not None:
            self._state = latest_state

        missing_manual = self._active_scope_manual_missing_indices()
        if missing_manual:
            self._set_status(
                "Calibrate: refreshing active-scope Manual endpoints from P4..."
            )
            try:
                refreshed = bool(self.backend.query_manual_encoder_references())
            except Exception:
                refreshed = False
            if refreshed:
                try:
                    latest_state = self.backend.get_state_snapshot()
                except Exception:
                    latest_state = None
                if latest_state is not None:
                    self._state = latest_state
                    if self.device_check_view is not None:
                        self.device_check_view.update_state(latest_state)
                missing_manual = self._active_scope_manual_missing_indices()
        if missing_manual:
            self._warn(
                "Manual Required",
                "Active joints are missing persisted Manual endpoints: "
                + self._format_joint_indices(missing_manual),
            )
            self._set_status("Calibrate blocked: active scope Manual is incomplete")
            self._update_manual_action_buttons()
            return

        system_state = getattr(self._state, "system_state", None)
        if system_state not in (
            SYSTEM_STATE_UNCONTROLLABLE,
            SYSTEM_STATE_IDLE,
            SYSTEM_STATE_READY,
        ):
            self._warn(
                "Calibration unavailable",
                "Stop the active motion first. Calibration requires P4 state "
                f"UNCONTROLLABLE, IDLE, or READY; current state={system_state}.",
            )
            self._set_status(
                f"Calibrate blocked: P4 system_state={system_state}"
            )
            return
        if not self._apply_active_scope_for_current_mode():
            self._set_status("Calibrate blocked: active scope was not accepted by lower controller")
            return
        self._start_calibration_now()

    def _start_calibration_now(self) -> None:
        self._abs_limit_warning_unlocked_mask = 0
        self._feedback_abs_limit_popup_episode_active = False
        active_joint_mask = int(self._active_joint_mask_from_ui()) & ((1 << ENCODER_COUNT) - 1)
        active_motor_mask = int(self._active_motor_mask_from_ui()) & VALID_MOTOR_MASK
        self._pending_calib_joint_mask = active_joint_mask
        self._pending_calib_motor_mask = active_motor_mask
        for joint_idx in _ordered_joint_indices_from_mask(active_joint_mask):
            if 0 <= joint_idx < len(self._joint_target_user_set):
                self._joint_target_user_set[joint_idx] = False
        self._joint_target_initialized = False
        self.context.start_calibration()

    def _active_scope_abs_limit_valid_mask(self) -> int:
        state = self._state
        if state is None:
            return 0
        return self._active_motor_mask_from_ui() & int(getattr(state, "motor_abs_limit_valid_bitmap", 0))

    def _abs_limit_toggle_target_locked(self) -> bool:
        mask = self._active_scope_abs_limit_valid_mask()
        if mask == 0 or self._state is None:
            return True
        locked_bitmap = int(getattr(self._state, "motor_abs_limit_locked_bitmap", 0) or 0)
        locked_bitmap &= ~int(getattr(self, "_abs_limit_warning_unlocked_mask", 0) or 0)
        all_locked = (locked_bitmap & mask) == mask
        return not all_locked

    def _refresh_abs_limit_lock_toggle_button(self) -> None:
        button = getattr(self, "abs_limit_lock_toggle_btn", None)
        if button is None:
            return
        target_locked = self._abs_limit_toggle_target_locked()
        button.setText("LOCK" if target_locked else "UNLOCK")
        if target_locked:
            button.setToolTip(
                "Lock the calibrated 5-turn ABS travel limits for the selected feedback joints"
            )
        else:
            button.setToolTip(
                "Unlock the calibrated 5-turn ABS travel limits for the selected feedback joints"
            )

    def toggle_abs_limit_lock(self) -> None:
        self.set_abs_limit_lock(self._abs_limit_toggle_target_locked())

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
        if active_mask == 0 or self._active_scope_abs_limit_missing_mask() != 0:
            return False
        mode = self._manual_submode() if self._app_mode == "manual" else "joint"
        if mode != "motor" and self._active_scope_joint_calib_missing_indices():
            return False
        return True

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

    def _turn_recovery_guide_text(self, unknown_mask: int, source: str = "") -> str:
        motors = self._format_motor_mask(unknown_mask)
        source_text = f"{source}\n\n" if source else ""
        return (
            f"{source_text}"
            "当前存在 Turn UNKNOWN，多圈位置不可信，系统已经阻止继续控制。\n\n"
            f"受影响电机：{motors}\n"
            f"unknown mask：0x{int(unknown_mask):08X}\n\n"
            "为什么会这样：\n"
            "- 舵机单独断电后，P4 保存的多圈状态可能和真实单圈 raw 不对应。\n"
            "- 机构被手动转动、换舵机、换机械或改映射后，旧多圈/旧标定不能继续使用。\n"
            "- P4 NVS 或旧快照恢复失败时，也会保持 Turn UNKNOWN。\n\n"
            "恢复步骤：\n"
            "1. 停止发送运动命令，必要时点击 E-STOP。\n"
            "2. 确认舵机电源、总线、ID 映射正常，机构没有继续被手动转动。\n"
            "3. 点击 New Device Reset，将当前在线舵机 raw 作为新的 turn0。\n"
            "4. New Device Reset 会清旧 ABS limit、旧关节标定，并归档旧 Calib Load。\n"
            "5. 若仍有故障锁存，再点击 Clear Fault。\n"
            "6. 重新 ENABLE，然后执行 Rope Init / Calibrate。\n"
            "7. 确认 TURN KNOWN、CALIB SUCCESS、OL OK 后再控制。"
        )

    def turn_unknown_popup_seen(self) -> bool:
        return bool(self._turn_recovery_popup_episode_active)

    def mark_turn_unknown_popup_seen(self, unknown_mask: int) -> None:
        unknown_mask = int(unknown_mask) & VALID_MOTOR_MASK
        if unknown_mask == 0:
            return
        self._turn_recovery_popup_episode_active = True
        self._turn_recovery_popup_episode_mask |= unknown_mask

    def reset_turn_unknown_popup_episode(self) -> None:
        self._turn_recovery_popup_episode_active = False
        self._turn_recovery_popup_episode_mask = 0

    def _show_turn_recovery_guide(self, unknown_mask: int, source: str = "") -> None:
        unknown_mask = int(unknown_mask) & VALID_MOTOR_MASK
        if unknown_mask == 0:
            return
        self.context.emit_action(
            f"{source or 'Command'} blocked: Turn UNKNOWN unknown=0x{unknown_mask:08X}"
        )
        self._set_status(
            f"{source or 'Command'} blocked: Turn UNKNOWN {self._format_motor_mask(unknown_mask)}; run New Device Reset first"
        )
        if self.turn_unknown_popup_seen():
            return
        self.mark_turn_unknown_popup_seen(unknown_mask)
        QtWidgets.QMessageBox.warning(
            self,
            "Turn UNKNOWN",
            self._turn_recovery_guide_text(unknown_mask, source),
        )

    def _format_joint_indices(self, indices: List[int], limit: int = 6) -> str:
        if not indices:
            return "none"
        text = ",".join(f"J{idx:02d}" for idx in indices[:limit])
        if len(indices) > limit:
            text += f"...(+{len(indices) - limit})"
        return text

    def _active_scope_joint_calib_missing_indices(self) -> List[int]:
        state = self._state
        active_joints = self._active_scope_joint_indices()
        if state is None:
            return active_joints
        missing: List[int] = []
        for joint_idx in active_joints:
            if not self._joint_workspace_range(joint_idx, state).ready:
                missing.append(joint_idx)
        return missing

    def _active_scope_manual_missing_indices(self) -> List[int]:
        active_joints = self._active_scope_joint_indices()
        state = self._state
        if state is None or not bool(getattr(state, "manual_reference_nvs_valid", False)):
            return active_joints
        valid_mask = int(getattr(state, "manual_reference_valid_bitmap", 0) or 0)
        return [joint for joint in active_joints if (valid_mask & (1 << joint)) == 0]

    def _active_scope_status_mismatch(self) -> str:
        state = self._state
        if state is None or not bool(getattr(state, "has_active_scope_status", False)):
            return ""
        data = self._current_active_scope_tuple()
        if data is None:
            return ""
        expected_motor, expected_joint, expected_can = data
        actual_motor = int(getattr(state, "active_motor_mask", 0) or 0) & VALID_MOTOR_MASK
        actual_joint = int(getattr(state, "active_joint_mask", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
        actual_can = bool(getattr(state, "require_can_feedback", False))
        expected_motor = int(expected_motor) & VALID_MOTOR_MASK
        expected_joint = int(expected_joint) & ((1 << ENCODER_COUNT) - 1)
        expected_can = bool(expected_can)
        parts: List[str] = []
        if actual_motor != expected_motor:
            parts.append(f"motor P4=0x{actual_motor:08X} UI=0x{expected_motor:08X}")
        if actual_joint != expected_joint:
            parts.append(f"joint P4=0x{actual_joint:08X} UI=0x{expected_joint:08X}")
        if actual_can != expected_can:
            parts.append(f"can P4={int(actual_can)} UI={int(expected_can)}")
        return "; ".join(parts)

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
                self._show_turn_recovery_guide(unknown_mask, str(source or "Feedback"))
            return False
        missing_mask = self._active_scope_abs_limit_missing_mask()
        if missing_mask != 0:
            if show_status:
                label = self._active_scope_label_from_ui() or str(source or "Feedback")
                self._set_status(
                    f"{label} blocked: calibrate active motors first ({self._format_motor_mask(missing_mask)})"
                )
            return False
        missing_joints = self._active_scope_joint_calib_missing_indices()
        if missing_joints:
            if show_status:
                self._set_status(
                    "Feedback blocked: joint calibration not loaded "
                    f"({self._format_joint_indices(missing_joints)}); press Calib Load or recalibrate"
                )
            return False
        mismatch = self._active_scope_status_mismatch()
        if mismatch:
            if show_status:
                self._set_status(f"Feedback blocked: active scope mismatch ({mismatch})")
            return False
        return True

    def set_abs_limit_lock(self, locked: bool) -> None:
        if self._app_mode == "manual" and self._manual_submode() not in ("joint", "test"):
            self._set_status("ABS limit lock is only available in Joint/Test Channels")
            return
        if not self.context.is_comm_connected():
            self._set_status("ABS limit lock blocked: serial port is not connected")
            return
        mask = self._active_scope_abs_limit_valid_mask()
        if mask == 0:
            self._set_status("ABS limit lock blocked: no calibrated ABS limit in current control set")
            return
        try:
            ok = self.backend.set_motor_abs_limit_lock(mask, bool(locked))
        except Exception as exc:
            self._set_status(f"ABS limit lock failed: {exc}")
            return
        action = "locked" if locked else "unlocked"
        self.context.emit_action(f"ABS limits {action}: mask=0x{mask:08X}")
        self._set_status(f"ABS limits {action}" if ok else f"ABS limits {action} ACK timeout")
        if ok and self._state is not None:
            if locked:
                self._abs_limit_warning_unlocked_mask &= ~mask
            else:
                self._abs_limit_warning_unlocked_mask |= mask
            self._feedback_abs_limit_popup_episode_active = False
            current = int(getattr(self._state, "motor_abs_limit_locked_bitmap", 0) or 0)
            self._state.motor_abs_limit_locked_bitmap = (current | mask) if locked else (current & ~mask)
            self._refresh_abs_limit_lock_toggle_button()
            for idx in range(ENCODER_COUNT):
                self._refresh_joint_abs_limit_item(idx, self._state)

    def toggle_multiturn_nvs(self) -> None:
        if not self.context.is_comm_connected():
            self._set_status("Flash control blocked: serial port is not connected")
            return
        state = self._state or self.backend.get_current_state()
        current = bool(getattr(state, "multiturn_nvs_enabled", True))
        target = not current
        try:
            ok = self.backend.set_multiturn_nvs_enabled(target)
        except Exception as exc:
            self._set_status(f"Flash control failed: {exc}")
            return
        if ok:
            if target:
                self._set_status("Flash multi-turn save enabled")
            else:
                self._set_status("Flash multi-turn save disabled; lower P4 snapshots cleared")
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
            btn.setToolTip("P4 Flash save disabled; lower snapshots cleared; click to enable future saves")

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

    @staticmethod
    def _format_joint_list(indices: List[int]) -> str:
        return ",".join(f"J{idx:02d}" for idx in indices)

    @staticmethod
    def _format_motor_list(indices: List[int]) -> str:
        return ",".join(f"M{idx:02d}" for idx in indices)

    @staticmethod
    def _snapshot_mask_value(value) -> int:
        try:
            if isinstance(value, str):
                return int(value, 0)
            return int(value)
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _joint_calib_result_has_valid_dual_lut(item: Optional[dict]) -> bool:
        if not isinstance(item, dict) or not bool(item.get("success", False)):
            return False
        if not bool(item.get("dualTendonLutValid", False)):
            return False
        try:
            count = int(item.get("dualTendonLutCount", 0))
        except (TypeError, ValueError):
            return False
        if count < 3 or count > DUAL_TENDON_LUT_MAX_SAMPLES:
            return False
        motors = item.get("dualTendonLutMotor", [])
        if not isinstance(motors, list) or len(motors) < 2:
            return False
        try:
            if any(int(motors[i]) < 0 or int(motors[i]) >= MOTOR_COUNT for i in range(2)):
                return False
        except (TypeError, ValueError):
            return False
        motor_abs = item.get("dualTendonLutMotorAbs", [])
        if not isinstance(motor_abs, list) or len(motor_abs) < 2:
            return False
        for row in motor_abs[:2]:
            if not isinstance(row, list) or len(row) < count:
                return False
        return True

    def _valid_joint_calib_results_for_mask(self, state, joint_mask: int) -> List[Optional[dict]]:
        source = getattr(state, "joint_calib_results", []) or []
        results: List[Optional[dict]] = [None] * ENCODER_COUNT
        mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
        for joint_idx in range(ENCODER_COUNT):
            if (mask & (1 << joint_idx)) == 0:
                continue
            if joint_idx >= len(source):
                continue
            item = source[joint_idx]
            if not isinstance(item, dict) or not bool(item.get("success", False)):
                continue
            results[joint_idx] = dict(item)
        return results

    def _maybe_save_calibration_snapshot(self, state) -> None:
        if state is None or str(getattr(state, "calib_status", "IDLE")) not in {"SUCCESS", "WARN"}:
            return
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
        if valid_bitmap == 0:
            return
        active_joint_mask = int(self._pending_calib_joint_mask)
        active_motor_mask = int(self._pending_calib_motor_mask)
        active_joint_mask &= (1 << ENCODER_COUNT) - 1
        active_motor_mask &= VALID_MOTOR_MASK
        if active_joint_mask == 0:
            return
        required_motor_mask = active_motor_mask & VALID_MOTOR_MASK
        if required_motor_mask != 0 and (valid_bitmap & required_motor_mask) != required_motor_mask:
            missing_motors = [
                idx for idx in range(MOTOR_COUNT)
                if (required_motor_mask & (1 << idx)) and not (valid_bitmap & (1 << idx))
            ]
            if missing_motors:
                self._set_status(
                    "Calib snapshot waiting motor limits: " +
                    self._format_motor_list(missing_motors[:8])
                )
            return

        joint_results = self._valid_joint_calib_results_for_mask(state, active_joint_mask)
        if not any(item is not None for item in joint_results):
            return
        required_joints = [
            idx for idx in range(ENCODER_COUNT)
            if (active_joint_mask & (1 << idx)) != 0
        ]
        missing_joints = [idx for idx in required_joints if not joint_results[idx]]
        if missing_joints:
            self._set_status(
                "Calib snapshot waiting joint results: " +
                self._format_joint_list(missing_joints[:8])
            )
            return
        missing_dual_lut = [
            idx for idx in required_joints
            if idx in DUAL_TENDON_JOINT_INDICES and
            not self._joint_calib_result_has_valid_dual_lut(joint_results[idx])
        ]
        if missing_dual_lut:
            self._set_status(
                "Calib snapshot waiting dual tendon LUT: " +
                self._format_joint_list(missing_dual_lut[:8])
            )
            return

        manual_valid_mask = int(
            getattr(state, "manual_reference_valid_bitmap", 0) or 0
        )
        manual_min_values = list(
            getattr(state, "manual_reference_min_raw", []) or []
        )
        manual_max_values = list(
            getattr(state, "manual_reference_max_raw", []) or []
        )
        if (manual_valid_mask & active_joint_mask) != active_joint_mask:
            missing_mask = active_joint_mask & ~manual_valid_mask
            self._set_status(
                "Calib snapshot save blocked: Manual missing " +
                self._format_joint_list(_ordered_joint_indices_from_mask(missing_mask))
            )
            return
        if (
            len(manual_min_values) < ENCODER_COUNT
            or len(manual_max_values) < ENCODER_COUNT
        ):
            self._set_status(
                "Calib snapshot save blocked: Manual endpoint readback incomplete"
            )
            return
        incompatible_v3 = [
            idx for idx in required_joints
            if int(joint_results[idx].get("mappingVersion", 0) or 0) != 3 or
            int(joint_results[idx].get("manualReferenceCrc", 0) or 0)
            != manual_encoder_reference_entry_crc(
                idx,
                manual_min_values[idx],
                manual_max_values[idx],
            )
        ]
        if incompatible_v3:
            self._set_status(
                "Calib snapshot save blocked: v3/Manual CRC mismatch " +
                self._format_joint_list(incompatible_v3)
            )
            return

        mins = [int(v) for v in list(getattr(state, "motor_abs_limit_min", []) or [])[:MOTOR_COUNT]]
        maxs = [int(v) for v in list(getattr(state, "motor_abs_limit_max", []) or [])[:MOTOR_COUNT]]
        if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
            return

        try:
            mapping_version, urdf_config_crc = (
                urdf_manual_scaled_mapping_metadata()
            )
        except (OSError, TypeError, ValueError) as exc:
            self._set_status(f"Calib snapshot save blocked: invalid URDF mapping: {exc}")
            return

        payload = {
            "schema_version": 3,
            "saved_at_host_time": time.time(),
            "hand_profile": HAND_PROFILE,
            "feedback_mapping_version": mapping_version,
            "manual_scaled_mapping_version": mapping_version,
            "urdf_config_crc": f"0x{urdf_config_crc:08X}",
            "manual_reference_crc": (
                f"0x{int(getattr(state, 'manual_reference_crc', 0) or 0):08X}"
            ),
            "active_joint_mask": f"0x{active_joint_mask:08X}",
            "active_motor_mask": f"0x{active_motor_mask:08X}",
            "motor_abs_limit_valid_bitmap": f"0x{valid_bitmap:08X}",
            "motor_abs_limit_locked_bitmap": "0x00000000",
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
        self._pending_calib_joint_mask = 0
        self._pending_calib_motor_mask = 0
        self.context.emit_action(f"Calib snapshot saved: {CALIB_LOAD_SNAPSHOT_PATH.name}")
        self._set_status(f"Calib snapshot saved to {CALIB_LOAD_SNAPSHOT_PATH.name}")
        self._update_manual_action_buttons()

    def _archive_calib_load_snapshot_for_new_device(self) -> str:
        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            return "no Calib Load snapshot"
        stamp = time.strftime("%Y%m%d_%H%M%S")
        archive_path = CALIB_LOAD_SNAPSHOT_PATH.with_name(
            f"{CALIB_LOAD_SNAPSHOT_PATH.stem}.before_new_device_{stamp}{CALIB_LOAD_SNAPSHOT_PATH.suffix}"
        )
        try:
            CALIB_LOAD_SNAPSHOT_PATH.replace(archive_path)
        except OSError as exc:
            self.context.emit_action(f"Calib snapshot archive failed: {exc}")
            return f"Calib Load snapshot archive failed: {exc}"
        self._last_calib_snapshot_signature = None
        self.context.emit_action(f"Calib snapshot archived: {archive_path.name}")
        return f"Calib Load snapshot archived: {archive_path.name}"

    def _clear_local_calibration_state_after_new_device(self) -> None:
        state = self._state
        if state is not None:
            try:
                state.motor_abs_limit_valid_bitmap = 0
                state.motor_abs_limit_locked_bitmap = 0
                state.motor_abs_limit_min = [0] * MOTOR_COUNT
                state.motor_abs_limit_max = [0] * MOTOR_COUNT
                state.joint_calib_results = [None] * ENCODER_COUNT
                state.calib_status = "IDLE"
            except Exception:
                pass
        self._last_calib_snapshot_signature = None
        self._abs_limit_warning_unlocked_mask = 0
        for idx in range(ENCODER_COUNT):
            self._refresh_joint_abs_limit_item(idx, state)
            if idx < len(self.joint_max_items) and self.joint_max_items[idx] is not None:
                self._refresh_joint_target_range(idx, state)

    def _validate_calibration_snapshot_for_load(self, snapshot: dict) -> Optional[str]:
        snapshot_joint_mask = self._snapshot_mask_value(snapshot.get("active_joint_mask"))
        snapshot_motor_mask = self._snapshot_mask_value(snapshot.get("active_motor_mask")) & VALID_MOTOR_MASK
        valid_motor_mask = self._snapshot_mask_value(
            snapshot.get("motor_abs_limit_valid_bitmap")
        ) & VALID_MOTOR_MASK
        current_joint_mask = int(self._active_joint_mask_from_ui()) & ((1 << ENCODER_COUNT) - 1)
        current_motor_mask = int(self._active_motor_mask_from_ui()) & VALID_MOTOR_MASK

        if current_joint_mask and snapshot_joint_mask and current_joint_mask != snapshot_joint_mask:
            current_joints = _ordered_joint_indices_from_mask(current_joint_mask)
            snapshot_joints = _ordered_joint_indices_from_mask(snapshot_joint_mask)
            return (
                "snapshot active joints do not match current scope: "
                f"snapshot={self._format_joint_list(snapshot_joints)}, "
                f"current={self._format_joint_list(current_joints)}"
            )
        if current_motor_mask and snapshot_motor_mask and current_motor_mask != snapshot_motor_mask:
            current_motors = [idx for idx in range(MOTOR_COUNT) if current_motor_mask & (1 << idx)]
            snapshot_motors = [idx for idx in range(MOTOR_COUNT) if snapshot_motor_mask & (1 << idx)]
            return (
                "snapshot active motors do not match current scope: "
                f"snapshot={self._format_motor_list(snapshot_motors)}, "
                f"current={self._format_motor_list(current_motors)}"
            )

        joint_results = snapshot.get("joint_calib_results", [])
        if len(joint_results) != ENCODER_COUNT:
            return f"invalid joint calibration count: {len(joint_results)}"
        if snapshot_joint_mask == 0:
            for joint_idx, item in enumerate(joint_results):
                if isinstance(item, dict) and bool(item.get("success", False)):
                    snapshot_joint_mask |= (1 << joint_idx)
        missing_joints = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (snapshot_joint_mask & (1 << joint_idx)) and (
                not isinstance(joint_results[joint_idx], dict) or
                not bool(joint_results[joint_idx].get("success", False))
            )
        ]
        if missing_joints:
            return (
                "snapshot missing active joint calibration: " +
                self._format_joint_list(missing_joints)
            )
        try:
            expected_version, expected_crc = (
                urdf_manual_scaled_mapping_metadata()
            )
        except (OSError, TypeError, ValueError) as exc:
            return f"current manual-scaled URDF mapping is invalid: {exc}"
        incompatible_mapping = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (snapshot_joint_mask & (1 << joint_idx)) and (
                int(joint_results[joint_idx].get("mappingVersion", 0) or 0)
                != expected_version or
                int(joint_results[joint_idx].get("urdfConfigCrc", 0) or 0)
                != expected_crc
            )
        ]
        if incompatible_mapping:
            return (
                "legacy/v1-v2 snapshot is view-only; v3 recalibration required: " +
                self._format_joint_list(incompatible_mapping)
            )
        missing_dual_lut = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (snapshot_joint_mask & (1 << joint_idx)) and
            joint_idx in DUAL_TENDON_JOINT_INDICES and
            not self._joint_calib_result_has_valid_dual_lut(joint_results[joint_idx])
        ]
        if missing_dual_lut:
            return (
                "snapshot missing dual tendon LUT; recalibrate or load a schema_version 2 snapshot: " +
                self._format_joint_list(missing_dual_lut)
            )
        missing_motors = [
            motor_idx for motor_idx in range(MOTOR_COUNT)
            if (snapshot_motor_mask & (1 << motor_idx)) and
            not (valid_motor_mask & (1 << motor_idx))
        ]
        if missing_motors:
            return (
                "snapshot missing active motor ABS limits: " +
                self._format_motor_list(missing_motors)
            )
        return None

    def new_device_reset(self) -> None:
        if not self.context.is_comm_connected():
            self._warn("New Device Reset", "串口未连接，无法重置多圈基准。")
            return
        state = self._state
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK):
            self._warn("New Device Reset", "控制正在运行，请先 E-STOP 或 Pause 后再重置。")
            return
        multi_turn_unknown_mask = (
            int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
            & VALID_MOTOR_MASK
        )
        if multi_turn_unknown_mask == 0:
            self._warn(
                "New Device Reset",
                "当前设备多圈圈数可信，禁止执行 New Device Reset。",
            )
            return
        if state is None or not bool(getattr(state, "has_servo_raw_data", False)):
            self._warn("New Device Reset", "尚未收到舵机 raw 反馈，无法建立 turn0 基准。")
            return

        reply = QtWidgets.QMessageBox.question(
            self,
            "New Device Reset",
            (
                "将为新机械/新舵机/新映射重置多圈基准。\n\n"
                "此操作会：\n"
                "- 停止当前控制输出\n"
                "- 清除 P4 Flash/NVS 旧多圈快照\n"
                "- 将每个在线电机的当前 raw 作为 turn0\n"
                "- 清除 UI 电机目标缓存\n\n"
                "完成后请重新 Rope Init / 标定，并确认 ABS 与 raw 合理。"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            self._set_status("New Device Reset cancelled")
            return

        self._set_status("New Device Reset running...")
        ok, message = self.backend.reset_multiturn_baseline_for_new_device()
        if ok:
            self._joint_target_initialized = False
            self._reset_motor_target_cache_for_device_epoch("New Device Reset: motor targets cleared")
            self._clear_local_calibration_state_after_new_device()
            archive_message = self._archive_calib_load_snapshot_for_new_device()
            self._last_multi_turn_unknown_bitmap = None
            self.reset_turn_unknown_popup_episode()
            final_message = f"{message}; {archive_message}; recalibrate required"
            self._set_status(f"New Device Reset success: {final_message}")
            self.context.emit_action(f"New Device Reset success: {final_message}")
        else:
            self._error("New Device Reset", f"重置失败：{message}")
            self._set_status(f"New Device Reset failed: {message}")
        self._update_manual_action_buttons()

    def load_calibration_snapshot(self) -> None:
        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            self._warn("Calib Load", "没有找到可加载的标定快照，请先完成 Calibrate。")
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
        snapshot_error = self._validate_calibration_snapshot_for_load(snapshot)
        if snapshot_error:
            self._error("Calib Load", snapshot_error)
            self._set_status(f"Calib Load blocked: {snapshot_error}")
            return
        state = self._state
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK):
            self._warn("Calib Load", "控制正在运行，请先 E-STOP 或 Pause 后再加载。")
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Calib Load",
            (
                "确认加载最近保存的标定快照？\n\n"
                "仅当机械、舵机 ID、映射和标定姿态没有变化时才应使用。"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._set_status("Calib Load sending snapshot...")
        ok, message = self.backend.load_calibration_snapshot(snapshot)
        if ok:
            self._joint_target_initialized = False
            self._motor_target_initialized = False
            mode = self._manual_submode() if self._app_mode == "manual" else "joint"
            scope_ok = False
            if mode != "device_check":
                scope_ok = self._apply_active_scope_for_current_mode()
            ready_recheck = False
            current_state = getattr(self._state, "system_state", None) if self._state is not None else None
            if (
                scope_ok and
                mode in ("joint", "test") and
                current_state in (SYSTEM_STATE_UNCONTROLLABLE, SYSTEM_STATE_IDLE, SYSTEM_STATE_READY)
            ):
                self.backend.start()
                ready_recheck = True
            if mode == "device_check":
                suffix = "; snapshot restored; switch to Joint Channels to apply active scope"
            elif scope_ok:
                suffix = "; active scope applied"
                if ready_recheck:
                    suffix += "; READY recheck requested"
            else:
                suffix = "; active scope not applied"
            self._set_status(f"Calib Load success: {message}{suffix}")
            self.context.emit_action(f"Calib Load success: {message}{suffix}")
        else:
            self._error("Calib Load", f"加载失败：{message}")
            self._set_status(f"Calib Load failed: {message}")

    def _apply_active_scope_for_current_mode(self) -> bool:
        if self._app_mode == "manual" and self._manual_submode() == "device_check":
            self.backend.set_pid_control(False)
            return True
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
            mode = self._manual_submode() if self._app_mode == "manual" else "joint"
            if self._app_mode in ("manual", "teleop"):
                if mode == "motor":
                    self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
                    self.backend.set_pid_control(False)
                    require_can = False
                else:
                    self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
                    self.backend.set_pid_control(bool(self._interaction_enabled or self._app_mode == "teleop"))
                    require_can = True
            ok = self.backend.set_active_scope(int(motor_mask), int(joint_mask), bool(require_can))
            if ok and self._state is not None and bool(getattr(self._state, "has_active_scope_status", False)):
                self._state.active_motor_mask = int(motor_mask) & VALID_MOTOR_MASK
                self._state.active_joint_mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
                self._state.require_can_feedback = bool(require_can)
        except Exception as exc:
            self._set_status(f"Active scope failed: {exc}")
            return False
        self._set_status("Active scope applied" if ok else "Active scope ACK timeout")
        return bool(ok)

    def begin_startup_device_check(self) -> None:
        if self._startup_device_check_running or not self.context.is_comm_connected():
            return
        self._startup_device_check_running = True
        self._startup_device_check_epoch += 1
        epoch = self._startup_device_check_epoch

        def query_worker() -> None:
            try:
                ok = bool(self.backend.query_manual_encoder_references())
                message = "Manual endpoints received" if ok else "Manual query ACK/readback timeout"
            except Exception as exc:
                ok = False
                message = f"{type(exc).__name__}: {exc}"
            self.startup_device_check_finished.emit(epoch, "query", ok, message)

        threading.Thread(
            target=query_worker,
            name="mmhand-startup-device-check",
            daemon=True,
        ).start()

    def _on_startup_device_check_finished(
        self,
        epoch: int,
        phase: str,
        ok: bool,
        message: str,
    ) -> None:
        if int(epoch) != self._startup_device_check_epoch or not self.context.is_comm_connected():
            return
        if phase == "load":
            self._startup_device_check_running = False
            if ok:
                self._joint_target_initialized = False
                self._motor_target_initialized = False
                self._set_status(f"Startup Calib Load success: {message}; switch to Joint Channels to apply scope")
                self.context.emit_action(f"Startup Calib Load success: {message}")
            else:
                self._set_status(f"Startup Calib Load skipped: {message}; automatic calibration required")
            return

        if not ok:
            self._startup_device_check_running = False
            self._set_status(f"Device Check failed: {message}")
            return

        try:
            state = self.backend.get_state_snapshot()
        except Exception:
            state = self._state
        if state is not None:
            self._state = state
            if self.device_check_view is not None:
                self.device_check_view.update_state(state)

        active_joint_mask = int(self._active_joint_mask_from_ui()) & ((1 << ENCODER_COUNT) - 1)
        manual_mask = int(getattr(state, "manual_reference_valid_bitmap", 0) or 0) if state is not None else 0
        nvs_valid = bool(getattr(state, "manual_reference_nvs_valid", False)) if state is not None else False
        missing_mask = active_joint_mask if not nvs_valid else active_joint_mask & ~manual_mask
        if missing_mask:
            self._startup_device_check_running = False
            missing = _ordered_joint_indices_from_mask(missing_mask)
            text = self._format_joint_indices(missing)
            self._set_status(f"Device Check: active scope Manual missing ({text})")
            if not self._startup_missing_warning_shown:
                self._startup_missing_warning_shown = True
                self._warn(
                    "Manual Required",
                    "Current control set is missing persisted Manual Min/Max: "
                    + text
                    + ". Enter them in Device Check before calibration or control.",
                )
            return

        if not CALIB_LOAD_SNAPSHOT_PATH.exists():
            self._startup_device_check_running = False
            self._set_status("Device Check passed: no calibration snapshot; run automatic calibration")
            return
        try:
            snapshot = json.loads(CALIB_LOAD_SNAPSHOT_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self._startup_device_check_running = False
            self._set_status(f"Startup Calib Load skipped: invalid snapshot ({exc})")
            return
        if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
            self._startup_device_check_running = False
            self._set_status("Startup Calib Load skipped: hand profile mismatch")
            return
        snapshot_error = self._validate_calibration_snapshot_for_load(snapshot)
        if snapshot_error:
            self._startup_device_check_running = False
            self._set_status(f"Startup Calib Load skipped: {snapshot_error}")
            return
        system_state = getattr(state, "system_state", None) if state is not None else None
        if system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_CALIBRATION_RUNNING):
            self._startup_device_check_running = False
            self._set_status("Startup Calib Load skipped: lower controller is moving")
            return

        def load_worker() -> None:
            try:
                loaded, load_message = self.backend.load_calibration_snapshot(snapshot)
            except Exception as exc:
                loaded = False
                load_message = f"{type(exc).__name__}: {exc}"
            self.startup_device_check_finished.emit(
                int(epoch),
                "load",
                bool(loaded),
                str(load_message),
            )

        threading.Thread(
            target=load_worker,
            name="mmhand-startup-calib-load",
            daemon=True,
        ).start()

    def append_algorithm_output(self, text: str) -> None:
        if self._algorithm_view is not None:
            self._algorithm_view.append_output(text)

    def _maybe_show_calibration_assembly_warning(self, state) -> None:
        if not bool(getattr(state, "has_calibration_warning_status", False)):
            return
        mask = int(getattr(state, "calibration_warning_bitmap", 0) or 0)
        mask &= (1 << ENCODER_COUNT) - 1
        if mask == 0:
            self._calibration_warning_popup_mask = 0
            return
        if str(getattr(state, "calib_status", "IDLE") or "IDLE") != "WARN":
            return
        new_mask = mask & ~self._calibration_warning_popup_mask
        if new_mask == 0:
            return
        min_offsets = list(
            getattr(state, "calibration_auto_min_offset_counts", []) or []
        )
        results = list(getattr(state, "joint_calib_results", []) or [])
        joint_sections = []
        event_items = []
        for joint in range(ENCODER_COUNT):
            if (new_mask & (1 << joint)) == 0:
                continue
            result = results[joint] if joint < len(results) else None
            if not isinstance(result, dict):
                # SYSTEM_STATUS may arrive before this joint's snapshot.
                return

            reason = int(result.get("assemblyWarningReason", 0) or 0)
            min_offset = (
                int(min_offsets[joint]) if joint < len(min_offsets) else 0
            )
            max_shortfall = 0
            auto_span = 0
            manual_span = 0
            urdf_span = 0
            span_difference = 0
            details = []
            if bool(result.get("hasAssemblyWarningFields", False)):
                min_offset = int(result.get("autoMinOffsetCounts", 0) or 0)
                max_shortfall = int(
                    result.get("autoMaxShortfallCounts", 0) or 0
                )
                auto_span = int(result.get("assemblyAutoSpanCounts", 0) or 0)
                has_span_comparison = bool(
                    result.get("hasAssemblySpanComparisonFields", False)
                )
                manual_span = int(
                    result.get("assemblyManualSpanCounts", 0) or 0
                )
                urdf_span = int(
                    result.get("assemblyUrdfSpanCounts", 0) or 0
                )
                span_difference = int(
                    result.get("assemblySpanDifferenceCounts", 0) or 0
                )
                if reason & CALIB_WARN_AUTO_MIN_OFFSET:
                    details.append(
                        f"Auto Min 比 Manual Min 正向内缩 {min_offset} counts，"
                        "达到或超过 350 counts"
                    )
                if reason & CALIB_WARN_AUTO_MAX_SHORTFALL:
                    details.append(
                        f"Auto Max 距 Manual Max 仍差 {max_shortfall} counts，"
                        "达到或超过 300 counts"
                    )
                if reason & CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH:
                    if has_span_comparison:
                        details.append(
                            f"Manual Span 为 {manual_span} counts，URDF 理论磁编 "
                            f"Span 为 {urdf_span} counts，差值 {span_difference} "
                            "counts，超过 360 counts"
                        )
                    else:
                        details.append(
                            f"旧固件报告 Auto Span 范围警告：{auto_span} counts"
                        )
            else:
                # Compatibility with the first WARN-capable firmware, which
                # only reported the Auto Min offset in SYSTEM_STATUS.
                reason = CALIB_WARN_AUTO_MIN_OFFSET
                details.append(
                    f"Auto Min 比 Manual Min 正向内缩 {min_offset} counts，"
                    "达到或超过 350 counts"
                )

            if not details:
                details.append("P4 报告装配警告，但未提供可识别的原因位")
            joint_sections.append(
                f"J{joint:02d}：\n" + "\n".join(f"- {item}" for item in details)
            )
            event_items.append(
                f"J{joint:02d}:reason=0x{reason:02X},"
                f"minOffset={min_offset},maxShortfall={max_shortfall},"
                f"autoSpan={auto_span},manualSpan={manual_span},"
                f"urdfSpan={urdf_span},spanDifference={span_difference}"
            )

        self._calibration_warning_popup_mask |= new_mask
        detail = "\n\n".join(joint_sections)
        message = (
            "标定已成功保存，但检测到可能的手指装配偏差：\n\n"
            f"{detail}\n\n"
            "该提示不属于故障，不会阻止 READY 或负反馈控制。\n"
            "建议检查对应手指的绳路、装配位置和机械端点。"
        )
        self.context.emit_action(
            f"Calibration assembly warning mask=0x{new_mask:08X}: " +
            ", ".join(event_items)
        )
        self._warn("标定装配警告", message)

    def update_state(self, state) -> None:
        self._state = state
        unknown_bitmap = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0) & VALID_MOTOR_MASK
        if unknown_bitmap == 0:
            self.reset_turn_unknown_popup_episode()
        self._handle_firmware_epoch_change(state)
        self._handle_connectivity_transitions(state)
        self._update_p4_rope_init_progress(state)
        calib_status = str(getattr(state, "calib_status", "IDLE"))
        active_calib_statuses = {"PENDING", "RUNNING"}
        calib_finished_success = (
            calib_status in {"SUCCESS", "WARN"} and
            self._last_calib_status in active_calib_statuses
        )
        if calib_finished_success:
            self._joint_target_initialized = False
        joint_calib_valid_mask = self._joint_calib_valid_mask(state)
        if self._last_joint_calib_valid_mask is not None and joint_calib_valid_mask != self._last_joint_calib_valid_mask:
            self._joint_target_initialized = False
            if joint_calib_valid_mask == 0:
                self._joint_target_user_set = [False] * ENCODER_COUNT
        self._last_joint_calib_valid_mask = joint_calib_valid_mask
        if calib_status in {"FAILED", "IDLE"} and self._last_calib_status in active_calib_statuses:
            self._pending_calib_joint_mask = 0
            self._pending_calib_motor_mask = 0
        self._last_calib_status = calib_status
        self._update_connection_lights()
        self._refresh_nvs_control_button()
        self._check_feedback_abs_limit_violation()
        self._maybe_show_calibration_assembly_warning(state)
        self._maybe_save_calibration_snapshot(state)
        if self._app_mode != "manual":
            updater = getattr(self._body_widget, "update_state", None)
            if callable(updater):
                updater(state)
            return
        if self.device_check_view is not None:
            self.device_check_view.update_state(state)
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

    def _handle_connectivity_transitions(self, state) -> None:
        if state is None:
            return
        if bool(getattr(state, "has_connectivity_status", False)):
            servo_mask = int(getattr(state, "servo_online_bitmap", 0) or 0) & VALID_MOTOR_MASK
            encoder_mask = int(getattr(state, "encoder_online_bitmap", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
            encoder_warn_mask = int(getattr(state, "encoder_warn_bitmap", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
            encoder_fault_mask = int(getattr(state, "encoder_fault_bitmap", 0) or 0) & ((1 << ENCODER_COUNT) - 1)
        else:
            servo_mask = 0
            for idx, online in enumerate(getattr(state, "servo_online", [])[:MOTOR_COUNT]):
                if online:
                    servo_mask |= (1 << idx)
            encoder_mask = 0
            encoder_warn_mask = 0
            encoder_fault_mask = 0
            if bool(getattr(state, "has_sensor_data", False)):
                for idx, encoder in enumerate(getattr(state, "encoders", [])[:ENCODER_COUNT]):
                    if not bool(getattr(encoder, "error", False)):
                        encoder_mask |= (1 << idx)

        if self._last_servo_online_bitmap is not None and servo_mask != self._last_servo_online_bitmap:
            newly_offline = self._last_servo_online_bitmap & ~servo_mask
            newly_online = servo_mask & ~self._last_servo_online_bitmap
            if newly_offline:
                message = f"Servo offline: {self._format_motor_mask(newly_offline)}, stale values hidden and targets cleared"
                self.context.emit_action(message)
                self._set_status(message)
            if newly_online:
                unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
                trusted = newly_online & ~unknown
                untrusted = newly_online & unknown
                parts: List[str] = []
                if trusted:
                    parts.append(f"trusted {self._format_motor_mask(trusted)}")
                if untrusted:
                    parts.append(f"TURN UNKNOWN {self._format_motor_mask(untrusted)}")
                message = "Servo online: " + "; ".join(parts)
                self.context.emit_action(message)
                self._set_status(message)
            self._reset_motor_target_cache_for_device_epoch("Servo connectivity changed: motor targets cleared")

        if self._last_encoder_online_bitmap is not None and encoder_mask != self._last_encoder_online_bitmap:
            newly_offline = self._last_encoder_online_bitmap & ~encoder_mask
            newly_online = encoder_mask & ~self._last_encoder_online_bitmap
            parts: List[str] = []
            if newly_offline:
                parts.append(f"offline {self._format_joint_indices(_ordered_joint_indices_from_mask(newly_offline))}, stale values hidden")
            if newly_online:
                parts.append(f"online {self._format_joint_indices(_ordered_joint_indices_from_mask(newly_online))}")
            if parts:
                message = "Encoder status: " + "; ".join(parts)
                self.context.emit_action(message)
                self._set_status(message)

        if bool(getattr(state, "has_encoder_health_status", False)):
            if self._last_encoder_warn_bitmap is not None and encoder_warn_mask != self._last_encoder_warn_bitmap:
                new_warn = encoder_warn_mask & ~self._last_encoder_warn_bitmap
                cleared_warn = self._last_encoder_warn_bitmap & ~encoder_warn_mask
                parts: List[str] = []
                if new_warn:
                    parts.append(f"warn {self._format_joint_indices(_ordered_joint_indices_from_mask(new_warn))}")
                if cleared_warn:
                    parts.append(f"warn cleared {self._format_joint_indices(_ordered_joint_indices_from_mask(cleared_warn))}")
                if parts:
                    message = "Encoder health: " + "; ".join(parts)
                    self.context.emit_action(message)
                    self._set_status(message)
            if self._last_encoder_fault_bitmap is not None and encoder_fault_mask != self._last_encoder_fault_bitmap:
                new_fault = encoder_fault_mask & ~self._last_encoder_fault_bitmap
                cleared_fault = self._last_encoder_fault_bitmap & ~encoder_fault_mask
                parts = []
                if new_fault:
                    parts.append(f"fault {self._format_joint_indices(_ordered_joint_indices_from_mask(new_fault))}")
                if cleared_fault:
                    parts.append(f"fault cleared {self._format_joint_indices(_ordered_joint_indices_from_mask(cleared_fault))}")
                if parts:
                    message = "Encoder health: " + "; ".join(parts)
                    self.context.emit_action(message)
                    self._set_status(message)

        self._last_servo_online_bitmap = servo_mask
        self._last_encoder_online_bitmap = encoder_mask
        self._last_encoder_warn_bitmap = encoder_warn_mask
        self._last_encoder_fault_bitmap = encoder_fault_mask

    def clear_live_state(self) -> None:
        self.stop_all_tests("stopped on disconnect", update_status=False)
        self._reset_p4_rope_init_progress()
        self._state = None
        self._startup_device_check_epoch += 1
        self._startup_device_check_running = False
        self._startup_missing_warning_shown = False
        if self.device_check_view is not None:
            self.device_check_view.clear_state()
        self._update_connection_lights()
        self._refresh_nvs_control_button()
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._last_calib_status = None
        self._calibration_warning_popup_mask = 0
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
        self.reset_turn_unknown_popup_episode()
        self._last_servo_online_bitmap = None
        self._last_encoder_online_bitmap = None
        self._last_encoder_warn_bitmap = None
        self._last_encoder_fault_bitmap = None
        self._joint_target_user_set = [False] * ENCODER_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        self._clear_run_states()
        self._updating = True
        try:
            for item in self.joint_current_items + self.joint_encoder_items + self.joint_abs_limit_items:
                item.setText("-")
                item.setToolTip("")
            for item in (
                self.motor_turn_items
                + self.motor_raw_items
                + self.motor_hw_items
                + self.motor_abs_items
                + self.motor_load_items
                + self.motor_joint_items
            ):
                if item is not None:
                    item.setText("-")
                    item.setToolTip("")
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
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
        self.backend.set_control_mode(ControlMode.JOINT_ANGLE)
        system_state = getattr(self._state, "system_state", None) if self._state is not None else None
        if system_state in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY):
            if not self._apply_active_scope_for_current_mode():
                return False
        if not self.feedback_command_allowed(show_status=True):
            return False
        # ACTIVE_SCOPE may have just moved P4 from IDLE to READY, but this UI
        # callback still holds the preceding status packet. Let P4 validate
        # RUN_FEEDBACK instead of falsely requiring a later UI action to
        # refresh the desktop-side state cache.
        if system_state not in (
            SYSTEM_STATE_IDLE,
            SYSTEM_STATE_READY,
            SYSTEM_STATE_RUN_FEEDBACK,
        ):
            if self._estop_latched:
                self._set_status("Command blocked: press ENABLE after E-STOP")
            else:
                self._set_status("Joint command blocked: controller is not READY")
            return False
        self.backend.set_pid_control(True)
        if self.backend.is_started():
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

    def _motor_display_valid(self, idx: int, state: Optional[HandModel] = None) -> bool:
        state = self._state if state is None else state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return False
        if bool(getattr(state, "has_connectivity_status", False)):
            return (int(getattr(state, "servo_online_bitmap", 0) or 0) & (1 << idx)) != 0
        if idx < len(getattr(state, "servo_online", []) or []) and getattr(state, "servo_online", [])[idx]:
            return True
        if idx < len(getattr(state, "servo_raw_online", []) or []) and getattr(state, "servo_raw_online", [])[idx]:
            return True
        if idx < len(getattr(state, "servo_telem_online", []) or []) and getattr(state, "servo_telem_online", [])[idx]:
            return True
        return False

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
        if not self._motor_display_valid(idx, state):
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
        if not self._motor_display_valid(idx, state):
            return None
        if not bool(getattr(state, "has_servo_raw_data", False)):
            return None
        if idx >= len(getattr(state, "servo_raw_online", [])) or not state.servo_raw_online[idx]:
            return None
        if idx >= len(getattr(state, "servo_raw_positions", [])):
            return None
        return motor_target_from_servo_feedback(state.servo_raw_positions[idx])

    def _motor_hw_feedback(self, idx: int) -> Optional[int]:
        state = self._state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return None
        if not self._motor_display_valid(idx, state):
            return None
        if not bool(getattr(state, "has_servo_hardware_data", False)):
            return None
        if idx >= len(getattr(state, "servo_raw_online", [])) or not state.servo_raw_online[idx]:
            return None
        hw_values = getattr(state, "servo_hardware_positions", []) or []
        if idx < len(hw_values):
            return clamp_motor_abs(int(hw_values[idx]))
        return None

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
            if state is not None and self._motor_display_valid(i, state) and bool(getattr(state, "has_servo_angle_data", False)):
                if i < len(getattr(state, "servo_online", [])) and state.servo_online[i] and i < len(getattr(state, "servo_angles", [])):
                    target = clamp_motor_abs(int(state.servo_angles[i]))
            if target is None:
                raw_single = self._motor_raw_single_feedback(i)
                if raw_single is not None:
                    target = int(raw_single)
            if (
                target is None and
                state is not None and
                self._motor_display_valid(i, state) and
                i < len(self._motor_last_sent_abs) and
                self._motor_last_sent_abs[i] is not None
            ):
                target = clamp_motor_abs(int(self._motor_last_sent_abs[i]))
            targets.append(0 if target is None else clamp_motor_abs(int(target)))
        return targets

    def _motor_abs_for_command_packet(self, idx: int, state: Optional[HandModel], *, ignore_last_sent: bool = False) -> int:
        relative_target = self._motor_abs_pos[idx]
        origin = self._motor_session_origin_abs[idx]
        if origin is None and state is not None and self._motor_display_valid(idx, state):
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
            sign = motor_tendon_shorten_abs_sign(idx)
            return clamp_motor_abs(int(origin) + sign * int(relative_target))
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
        sign = motor_tendon_shorten_abs_sign(idx)
        return (
            f"M{idx:02d} rel={relative if relative is not None else '-'} "
            f"sign={sign:+d} origin={origin if origin is not None else '-'} "
            f"sent={sent if sent is not None else '-'}"
        )

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

    def _send_motor_abs_from_ui(self, trigger: str) -> bool:
        if not self._interaction_enabled:
            return False
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        if self._estop_latched and not self.backend.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return False
        if not self.backend.is_started():
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
            self._show_turn_recovery_guide(unknown_mask, f"Motor command ({trigger})")
            return False
        for idx in range(MOTOR_COUNT):
            if self._motor_session_origin_abs[idx] is None:
                self._motor_resync_one_channel(idx)
            if self._motor_abs_pos[idx] is None:
                self._motor_abs_pos[idx] = 0
        targets = self._build_motor_abs_command_list()
        self.backend.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        self.context.emit_action(f"Motor absolute positions sent ({trigger}): {self._motor_command_debug_summary(trigger, targets)}")
        return True

    def _send_single_motor_abs_from_ui(self, idx: int, trigger: str) -> Optional[int]:
        if idx < 0 or idx >= MOTOR_COUNT:
            return None
        if not self._interaction_enabled:
            return None
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        if self._estop_latched and not self.backend.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return None
        if not self.backend.is_started():
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
            self._show_turn_recovery_guide(unknown_mask, f"Motor command ({trigger})")
            return None
        if self._motor_session_origin_abs[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            self._motor_abs_pos[idx] = 0
        targets = self._build_motor_abs_command_list()
        target_abs = int(targets[idx])
        self.backend.set_single_motor_position_absolute(idx, target_abs)
        if 0 <= idx < len(self._motor_last_sent_abs):
            self._motor_last_sent_abs[idx] = target_abs
        self.context.emit_action(
            f"Single motor absolute position sent ({trigger}): M{idx:02d}={target_abs}"
        )
        return target_abs

    def _clear_motor_target_after_single_send(self, idx: int, sent_abs: int) -> None:
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        self._motor_session_origin_abs[idx] = int(sent_abs)
        self._motor_abs_pos[idx] = 0
        self._motor_last_slider[idx] = 0
        self._motor_target_user_set[idx] = False
        self._set_motor_target_widget(idx, 0)

    def _check_feedback_abs_limit_violation(self) -> None:
        state = self._state
        if state is None or getattr(state, "system_state", None) != SYSTEM_STATE_RUN_FEEDBACK:
            self._feedback_abs_limit_popup_episode_active = False
            return
        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0))
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", 0))
        self._abs_limit_warning_unlocked_mask &= valid_bitmap
        monitor_mask = self._active_motor_mask_from_ui()
        if monitor_mask == 0:
            monitor_mask = valid_bitmap
        monitor_mask &= valid_bitmap & locked_bitmap
        monitor_mask &= ~int(getattr(self, "_abs_limit_warning_unlocked_mask", 0) or 0)
        if monitor_mask == 0:
            self._feedback_abs_limit_popup_episode_active = False
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
        if hits and not self._feedback_abs_limit_popup_episode_active:
            self._feedback_abs_limit_popup_episode_active = True
            self._show_feedback_abs_limit_violation(hits)
        elif not hits:
            self._feedback_abs_limit_popup_episode_active = False

    def _show_feedback_abs_limit_violation(self, hits: List[Tuple[int, int, int, int]]) -> None:
        preview = ", ".join(
            f"M{idx:02d}: abs={feedback_abs} [{lo},{hi}]"
            for idx, feedback_abs, lo, hi in hits[:4]
        )
        suffix = "" if len(hits) <= 4 else f" ... +{len(hits) - 4}"
        self._set_status(f"Feedback ABS out of limit: {preview}{suffix}")
        QtWidgets.QMessageBox.warning(
            self,
            "Feedback ABS Limit (default unlocked)",
            "RUN_FEEDBACK servo feedback ABS is outside the calibrated 5-turn locked window.\n"
            "Feedback limits are unlocked by default; use LOCK only when you intentionally need this guard.\n"
            f"{preview}{suffix}",
        )

    def _send_motor_sweep_from_ui(self, trigger: str) -> None:
        if not self._interaction_enabled:
            return
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.backend.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.backend.set_pid_control(False)
        if self._estop_latched and not self.backend.is_started():
            self._set_status("Motor command blocked: press ENABLE after E-STOP")
            return
        if not self.backend.is_started():
            self._set_status("Motor command blocked: waiting for controllable IDLE after ENABLE")
            return
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return
        unknown_mask = self._active_scope_multiturn_unknown_mask()
        if unknown_mask != 0:
            self._show_turn_recovery_guide(unknown_mask, f"Motor sweep ({trigger})")
            return
        targets = self.motor_sweep_targets()
        self.backend.set_motor_positions_raw_sweep(targets)
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

    def _joint_default_target_max_deg(self, idx: int) -> float:
        return float(JOINT_TARGET_MAX_DEG_BY_INDEX[idx] if 0 <= idx < len(JOINT_TARGET_MAX_DEG_BY_INDEX) else JOINT_TARGET_MAX_DEG_BY_INDEX[-1])

    def _joint_default_target_min_deg(self, idx: int) -> float:
        return float(JOINT_TARGET_MIN_DEG_BY_INDEX[idx] if 0 <= idx < len(JOINT_TARGET_MIN_DEG_BY_INDEX) else JOINT_TARGET_MIN_DEG_BY_INDEX[-1])

    def _joint_workspace_range(
        self,
        idx: int,
        state: Optional[HandModel] = None,
    ) -> JointWorkspaceRange:
        state = self._state if state is None else state
        return resolve_joint_workspace_range(
            state,
            idx,
            self._joint_default_target_min_deg(idx),
            self._joint_default_target_max_deg(idx),
            WORKSPACE_MAPPING_VERSION,
            WORKSPACE_URDF_CRC,
        )

    def _joint_calibrated_target_min_deg(self, idx: int, state: Optional[HandModel] = None) -> Optional[float]:
        workspace = self._joint_workspace_range(idx, state)
        return workspace.current_min_deg if workspace.ready else None

    def _joint_target_max_deg(self, idx: int) -> float:
        # The upper target boundary is always the static URDF Max.
        return self._joint_default_target_max_deg(idx)

    def _joint_target_min_deg(self, idx: int, state: Optional[HandModel] = None) -> float:
        calibrated = self._joint_calibrated_target_min_deg(idx, state)
        if calibrated is not None:
            return calibrated
        return self._joint_default_target_min_deg(idx)

    def _clamp_joint_target_deg(self, idx: int, value: float) -> float:
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = self._joint_target_min_deg(idx)
        if not np.isfinite(value):
            value = self._joint_target_min_deg(idx)
        lo = self._joint_target_min_deg(idx, self._state)
        hi = self._joint_target_max_deg(idx)
        return max(lo, min(hi, value))

    def _refresh_joint_target_range(self, idx: int, state: Optional[HandModel]) -> None:
        if idx < 0 or idx >= ENCODER_COUNT:
            return
        workspace = self._joint_workspace_range(idx, state)
        lo = workspace.current_min_deg if workspace.ready else self._joint_default_target_min_deg(idx)
        hi = self._joint_target_max_deg(idx)
        if idx < len(self.joint_min_items):
            item = self.joint_min_items[idx]
            if item is not None:
                self._set_item_text_if_changed(item, f"{lo:.2f}" if workspace.ready else "-")
                self._set_item_tooltip_if_changed(
                    item,
                    "Dynamic Current Min from P4 v3 calibration" if workspace.ready else workspace.reason,
                )
        if idx < len(self.joint_max_items):
            item = self.joint_max_items[idx]
            if item is not None:
                self._set_item_text_if_changed(item, f"{hi:.2f}")
                self._set_item_tooltip_if_changed(item, "Static URDF Max")
        if idx < len(self.joint_target_spins):
            spin = self.joint_target_spins[idx]
            if spin is not None:
                self._set_spin_range_if_changed(spin, lo, hi)
        if idx < len(self.joint_sliders):
            slider = self.joint_sliders[idx]
            if slider is not None:
                self._set_slider_range_if_changed(
                    slider,
                    int(round(lo * 100.0)),
                    int(round(hi * 100.0)),
                )
        controls_enabled = bool(self._interaction_enabled and workspace.ready)
        for widgets in (self.joint_target_spins, self.joint_sliders, self.joint_send_buttons):
            if idx < len(widgets) and widgets[idx] is not None:
                widgets[idx].setEnabled(controls_enabled)

    def _joint_angle_feedback_valid(self, idx: int, state=None) -> bool:
        state = self._state if state is None else state
        if state is None or not bool(getattr(state, "has_sensor_data", False)):
            return False
        angles = getattr(state, "angles", []) or []
        if idx < 0 or idx >= len(angles):
            return False
        encoders = getattr(state, "encoders", []) or []
        if idx < len(encoders):
            return not bool(getattr(encoders[idx], "angle_error", getattr(encoders[idx], "error", False)))
        return True

    def _joint_raw_feedback_valid(self, idx: int, state=None) -> bool:
        state = self._state if state is None else state
        if state is None or not bool(getattr(state, "has_encoder_raw_data", False)):
            return False
        encoders = getattr(state, "encoders", []) or []
        if idx < 0 or idx >= len(encoders):
            return False
        return not bool(getattr(encoders[idx], "raw_error", getattr(encoders[idx], "error", False)))

    def _joint_encoder_status(self, idx: int, state: Optional[HandModel] = None) -> Optional[str]:
        state = self._state if state is None else state
        if idx < 0 or idx >= ENCODER_COUNT or state is None:
            return None
        if bool(getattr(state, "has_encoder_health_status", False)):
            bit = 1 << idx
            online = (int(getattr(state, "encoder_online_bitmap", 0) or 0) & bit) != 0
            warn = (int(getattr(state, "encoder_warn_bitmap", 0) or 0) & bit) != 0
            fault = (int(getattr(state, "encoder_fault_bitmap", 0) or 0) & bit) != 0
            if fault or not online:
                return "fault"
            if warn:
                return "warn"
            return "ok"
        if not bool(getattr(state, "has_sensor_data", False)):
            return None
        encoders = getattr(state, "encoders", []) or []
        if idx >= len(encoders):
            return None
        if bool(getattr(encoders[idx], "angle_error", getattr(encoders[idx], "error", False))):
            return "fault"
        return "ok"

    def _joint_encoder_display_valid(self, idx: int, state: Optional[HandModel] = None) -> bool:
        return self._joint_encoder_status(idx, state) in ("ok", "warn")

    def _joint_angle_display_calibrated(self, idx: int, state: Optional[HandModel] = None) -> bool:
        return self._joint_workspace_range(idx, state).ready

    def _joint_calib_valid_mask(self, state: Optional[HandModel]) -> int:
        if state is None:
            return 0
        mask = 0
        for idx in range(ENCODER_COUNT):
            if self._joint_workspace_range(idx, state).ready:
                mask |= (1 << idx)
        return mask

    def _joint_feedback_target(self, idx: int) -> float:
        state = self._state
        if (
            self._joint_angle_display_calibrated(idx, state) and
            self._joint_encoder_display_valid(idx, state) and
            self._joint_angle_feedback_valid(idx, state)
        ):
            return self._clamp_joint_target_deg(idx, float(state.angles[idx]))
        if idx < len(self.joint_target_spins):
            return self._clamp_joint_target_deg(idx, float(self.joint_target_spins[idx].value()))
        return self._joint_target_min_deg(idx)

    def _joint_angle_batch_suspicious_zero(self, state: Optional[HandModel]) -> bool:
        if state is None or not bool(getattr(state, "has_sensor_data", False)):
            return False
        angles = list(getattr(state, "angles", []) or [])
        if len(angles) < ENCODER_COUNT:
            return False
        if any(abs(float(value)) > 0.001 for value in angles[:ENCODER_COUNT]):
            return False
        encoders = list(getattr(state, "encoders", []) or [])
        raw_online = any(
            int(getattr(encoders[i], "raw_sensor", 0) or 0) not in (0, 0xFFFF)
            for i in range(min(ENCODER_COUNT, len(encoders)))
        )
        if not raw_online:
            return False
        calib_active = str(getattr(state, "calib_status", "") or "") in ("PENDING", "RUNNING")
        system_calib = int(getattr(state, "system_state", -1) or -1) == SYSTEM_STATE_CALIBRATION_RUNNING
        return bool(self._joint_target_initialized or calib_active or system_calib)

    def _motor_feedback_target(self, idx: int) -> int:
        state = self._state
        if state is not None and self._motor_display_valid(idx, state):
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

    def _refresh_joint_feedback_view(self, sync_target: bool = False) -> None:
        state = self._state
        joint_has_data = bool(state is not None and getattr(state, "has_sensor_data", False))
        suspicious_zero_angles = self._joint_angle_batch_suspicious_zero(state)
        table = getattr(self, "joint_table", None)
        if table is not None:
            table.setUpdatesEnabled(False)
        self._updating = True
        try:
            for i in range(ENCODER_COUNT):
                self._refresh_joint_target_range(i, state)
                self._refresh_joint_abs_limit_item(i, state)
                encoder_status = self._joint_encoder_status(i, state)
                display_valid = encoder_status in ("ok", "warn")
                angle_feedback_valid = self._joint_angle_feedback_valid(i, state)
                raw_feedback_valid = self._joint_raw_feedback_valid(i, state)
                angle_calibrated = self._joint_angle_display_calibrated(i, state)
                self._set_joint_encoder_dot(i, encoder_status)
                if (
                    display_valid and
                    angle_feedback_valid and
                    joint_has_data and
                    i < len(getattr(state, "angles", []))
                ):
                    if angle_calibrated and not suspicious_zero_angles:
                        current_item = self.joint_current_items[i]
                        current_deg = float(state.angles[i])
                        self._set_item_text_if_changed(current_item, f"{current_deg:.2f}")
                        out_of_workspace = bool(
                            i < len(getattr(state, "encoders", [])) and
                            getattr(state.encoders[i], "angle_out_of_workspace", False)
                        )
                        if out_of_workspace:
                            current_item.setForeground(QtGui.QBrush(QtGui.QColor("#d89b00")))
                            self._set_item_tooltip_if_changed(
                                current_item,
                                "Live joint angle is outside the calibrated URDF workspace.",
                            )
                        else:
                            current_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                            self._set_item_tooltip_if_changed(current_item, "Live calibrated joint angle")
                    elif not angle_calibrated:
                        current_item = self.joint_current_items[i]
                        self._set_item_text_if_changed(current_item, "0.00")
                        current_item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                        self._set_item_tooltip_if_changed(current_item, "Joint is not calibrated")
                else:
                    current_item = self.joint_current_items[i]
                    self._set_item_text_if_changed(current_item, "-")
                    current_item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                    self._set_item_tooltip_if_changed(current_item, "No valid mapped-angle feedback")
                if (
                    display_valid and
                    raw_feedback_valid and
                    i < len(getattr(state, "encoders", []))
                ):
                    self._set_item_text_if_changed(
                        self.joint_encoder_items[i],
                        str(getattr(state.encoders[i], "raw_sensor", getattr(state.encoders[i], "raw", "-"))),
                    )
                else:
                    self._set_item_text_if_changed(self.joint_encoder_items[i], "-")
            if joint_has_data and sync_target and not suspicious_zero_angles:
                for i in range(min(ENCODER_COUNT, len(getattr(state, "angles", [])))):
                    if not self._joint_encoder_display_valid(i, state):
                        continue
                    if not self._joint_angle_feedback_valid(i, state):
                        continue
                    user_set = i < len(self._joint_target_user_set) and self._joint_target_user_set[i]
                    if user_set:
                        continue
                    if self._joint_angle_display_calibrated(i, state):
                        target = self._clamp_joint_target_deg(i, float(state.angles[i]))
                    else:
                        target = 0.0
                    self._set_spin_value_if_changed(self.joint_target_spins[i], target, tol=0.0005)
                    self._set_slider_value_if_changed(self.joint_sliders[i], int(round(target * 100.0)))
                    if i < len(self._joint_target_user_set):
                        self._joint_target_user_set[i] = False
                self._joint_target_initialized = True
            self._refresh_test_workspace_ranges()
        finally:
            self._updating = False
            if table is not None:
                table.setUpdatesEnabled(True)

    def _refresh_joint_abs_limit_item(self, joint_idx: int, state: Optional[HandModel]) -> None:
        if joint_idx < 0 or joint_idx >= len(self.joint_abs_limit_items):
            return
        item = self.joint_abs_limit_items[joint_idx]
        if item is None or state is None:
            if item is not None:
                item.setText("-")
                item.setToolTip("")
            return

        valid_bitmap = int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0)
        locked_bitmap = int(getattr(state, "motor_abs_limit_locked_bitmap", 0) or 0)
        mins = getattr(state, "motor_abs_limit_min", []) or []
        maxs = getattr(state, "motor_abs_limit_max", []) or []
        summaries: List[str] = []
        details: List[str] = []
        motors = self._motors_by_joint[joint_idx] if joint_idx < len(self._motors_by_joint) else []
        for motor_idx in motors:
            if motor_idx >= len(mins) or motor_idx >= len(maxs):
                continue
            if ((valid_bitmap >> motor_idx) & 0x01) == 0:
                continue
            lo = int(mins[motor_idx])
            hi = int(maxs[motor_idx])
            if lo > hi:
                lo, hi = hi, lo
            state_text = "LOCK" if ((locked_bitmap >> motor_idx) & 0x01) != 0 else "UNLOCK"
            summaries.append(f"M{motor_idx:02d} {state_text[0]} [{lo},{hi}]")
            details.append(f"M{motor_idx:02d}: {state_text}, 5-turn ABS range [{lo}, {hi}]")

        item.setText(" / ".join(summaries) if summaries else "-")
        item.setToolTip("\n".join(details))

    def _refresh_motor_joint_feedback_item(self, motor_idx: int, state: Optional[HandModel]) -> None:
        if motor_idx < 0 or motor_idx >= len(self.motor_joint_items):
            return
        item = self.motor_joint_items[motor_idx]
        if item is None or state is None:
            return
        joint_idx = MOTOR_TO_JOINT_INDEX[motor_idx] if motor_idx < len(MOTOR_TO_JOINT_INDEX) else -1
        if (
            0 <= joint_idx < len(getattr(state, "angles", []) or []) and
            self._joint_encoder_display_valid(joint_idx, state)
        ):
            item.setText(f"{float(state.angles[joint_idx]):.2f}")
            item.setToolTip("")
        else:
            item.setText("-")
            item.setToolTip("Joint encoder feedback unavailable")

    def _hide_offline_motor_feedback_items(self, motor_idx: int) -> None:
        tip = f"M{motor_idx:02d} offline: stale feedback hidden"
        for items in (
            self.motor_turn_items,
            self.motor_raw_items,
            self.motor_hw_items,
            self.motor_abs_items,
            self.motor_load_items,
        ):
            if motor_idx < len(items):
                item = items[motor_idx]
                if item is not None:
                    item.setText("-")
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                    item.setToolTip(tip)

    def _refresh_motor_feedback_view(self, sync_target: bool = False) -> None:
        state = self._state
        if state is None:
            return
        motor_has_abs = bool(getattr(state, "has_servo_angle_data", False))
        motor_has_raw = bool(getattr(state, "has_servo_raw_data", False))
        table = getattr(self, "motor_table", None)
        if table is not None:
            table.setUpdatesEnabled(False)
        self._updating = True
        try:
            unknown_mask = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
            overload_flags = getattr(state, "servo_overload_fault", []) or []
            overload_loads = getattr(state, "servo_overload_trigger_load", []) or []
            for i in range(MOTOR_COUNT):
                motor_online = self._motor_display_valid(i, state)
                if not motor_online:
                    self._hide_offline_motor_feedback_items(i)
                    self._refresh_motor_joint_feedback_item(i, state)
                    self._motor_session_origin_abs[i] = None
                    self._motor_abs_pos[i] = None
                    self._motor_last_slider[i] = None
                    self._motor_last_sent_abs[i] = None
                    self._motor_target_user_set[i] = False
                    continue
                dev_abs = self._motor_feedback_abs(i)
                turn_unknown = ((unknown_mask & (1 << i)) != 0)
                if i < len(self.motor_turn_items):
                    turn_item = self.motor_turn_items[i]
                    turn_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                    turn_item.setToolTip("P4 logical turn count")
                    turn_item.setText(str(self._motor_turn_from_abs(dev_abs)) if dev_abs is not None and not turn_unknown else "-")
                    if turn_unknown:
                        turn_item.setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                        turn_item.setToolTip(
                            "Turn UNKNOWN: P4 cannot trust the multi-turn position. "
                            "Use New Device Reset or restore multi-turn before control."
                        )
                raw_single = self._motor_raw_single_feedback(i)
                if i < len(self.motor_raw_items):
                    raw_item = self.motor_raw_items[i]
                    raw_item.setText(f"{raw_single:4d}" if raw_single is not None else "-")
                    raw_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                    raw_item.setToolTip("Single-turn raw position, 0..4095")
                hw_abs = self._motor_hw_feedback(i)
                if i < len(self.motor_hw_items) and self.motor_hw_items[i] is not None:
                    self.motor_hw_items[i].setText(str(int(hw_abs)) if hw_abs is not None else "-")
                    self.motor_hw_items[i].setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                    self.motor_hw_items[i].setToolTip("Feetech Present Position before P4 logical offset")
                if i < len(self.motor_abs_items):
                    abs_item = self.motor_abs_items[i]
                    abs_item.setText(str(int(dev_abs)) if dev_abs is not None else "-")
                    abs_item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                    abs_item.setToolTip("P4 logical ABS used by control")
                    if turn_unknown:
                        abs_item.setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                        abs_item.setToolTip(
                            "Turn UNKNOWN: P4 cannot trust this logical ABS. "
                            "Do not control until multi-turn is recovered."
                        )
                    elif dev_abs is not None and hw_abs is not None:
                        offset = int(dev_abs) - int(hw_abs)
                        abs_item.setToolTip(
                            "P4 logical ABS used by control. "
                            f"Logical offset vs Feetech HW: {offset} counts ({offset / MOTOR_SINGLE_TURN_STEPS:.2f} turns)."
                        )
                if raw_single is not None and hw_abs is not None:
                    hw_single = int(hw_abs) % MOTOR_SINGLE_TURN_STEPS
                    raw_hw_diff = self._single_turn_diff(int(raw_single), hw_single)
                    if raw_hw_diff > MOTOR_RAW_HW_MISMATCH_TOL:
                        tip = (
                            f"Raw/HW single-turn mismatch: RawNow={int(raw_single)}, "
                            f"HWNow mod 4096={hw_single}, diff={raw_hw_diff} counts."
                        )
                        if i < len(self.motor_raw_items):
                            self.motor_raw_items[i].setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                            self.motor_raw_items[i].setToolTip(tip)
                        if i < len(self.motor_hw_items) and self.motor_hw_items[i] is not None:
                            self.motor_hw_items[i].setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                            self.motor_hw_items[i].setToolTip(tip)
                if i < len(self.motor_load_items) and self.motor_load_items[i] is not None:
                    item = self.motor_load_items[i]
                    load_text = "-"
                    loads = getattr(state, "servo_load", [])
                    online = getattr(state, "servo_telem_online", [])
                    overload_active = i < len(overload_flags) and bool(overload_flags[i])
                    if overload_active:
                        trigger_load = "?"
                        if i < len(overload_loads) and overload_loads[i] is not None:
                            try:
                                trigger_load = str(int(overload_loads[i]))
                            except Exception:
                                trigger_load = str(overload_loads[i])
                        load_text = f"OL {trigger_load}"
                        item.setForeground(QtGui.QBrush(QtGui.QColor("#c62828")))
                        item.setToolTip(f"M{i:02d} overload trigger load={trigger_load}")
                    else:
                        item.setForeground(QtGui.QBrush(QtGui.QColor("#000000")))
                        item.setToolTip("")
                    if i < len(loads) and i < len(online) and online[i]:
                        if not overload_active:
                            load_text = str(int(loads[i]))
                    item.setText(load_text)
                self._refresh_motor_joint_feedback_item(i, state)
            if (motor_has_raw or motor_has_abs) and sync_target:
                for i in range(MOTOR_COUNT):
                    if not self._motor_display_valid(i, state):
                        continue
                    dev_abs = self._motor_feedback_abs(i)
                    if dev_abs is None:
                        continue
                    raw_single = self._motor_raw_single_feedback(i)
                    if raw_single is not None and self._is_motor_abs_at_limit(dev_abs):
                        dev_abs = int(raw_single)
                    self._motor_session_origin_abs[i] = dev_abs
                    self._motor_abs_pos[i] = 0
                    self._set_spin_value_if_changed(self.motor_target_spins[i], 0)
                    self._set_slider_value_if_changed(self.motor_sliders[i], 0)
                    self._motor_last_slider[i] = 0
                    self._motor_target_user_set[i] = False
                self._motor_target_initialized = True
                self._seed_motor_last_sent_after_sync(state)
        finally:
            self._updating = False
            if table is not None:
                table.setUpdatesEnabled(True)
        for i in range(MOTOR_COUNT):
            if not self._motor_display_valid(i, state):
                continue
            cur_single = self._motor_feedback_target(i)
            tgt_single = self._read_motor_target_slider_int(i)
            if self._single_turn_diff(cur_single, tgt_single) <= MOTOR_DIR_HINT_CLEAR_TOL:
                if self._motor_dir_hint_pending_clear[i]:
                    self._motor_dir_hint_stable_count[i] += 1
                if self._motor_dir_hint_stable_count[i] >= MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES:
                    self._motor_dir_hint[i] = 0
                    self._motor_dir_hint_pending_clear[i] = False
                    self._motor_dir_hint_stable_count[i] = 0

    def _set_joint_encoder_dot(self, joint_index: int, status: Optional[object]) -> None:
        if joint_index < 0 or joint_index >= len(self.joint_status_items):
            return
        item = self.joint_status_items[joint_index]
        if status is True or status == "ok":
            item.setForeground(QtGui.QBrush(QtGui.QColor("#19a55a")))
            item.setToolTip("Encoder OK")
        elif status == "warn":
            item.setForeground(QtGui.QBrush(QtGui.QColor("#d89b00")))
            item.setToolTip("Encoder WARN: transient CAN/encoder bad frame, using last trusted value")
        elif status is False or status == "fault":
            item.setForeground(QtGui.QBrush(QtGui.QColor("#d84343")))
            item.setToolTip("Encoder FAULT/OFFLINE: stable feedback is unavailable")
        else:
            item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
            item.setToolTip("Encoder state unknown")

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
            self._set_spin_value_if_changed(self.motor_target_spins[idx], int(value))
            self._set_slider_value_if_changed(self.motor_sliders[idx], self._slider_from_abs_pos(int(value)))
        finally:
            self._updating = False

    def _set_motor_relative_target_widget(self, idx: int, relative_target: int) -> None:
        self._updating = True
        try:
            target = clamp_motor_abs(int(relative_target))
            self._set_spin_value_if_changed(self.motor_target_spins[idx], target)
            self._set_slider_value_if_changed(self.motor_sliders[idx], self._slider_from_abs_pos(target))
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
        joint_idx = int(current[1:])
        workspace = self._joint_workspace_range(joint_idx, self._state)
        lo = workspace.current_min_deg if workspace.ready else self._joint_default_target_min_deg(joint_idx)
        hi = self._joint_target_max_deg(joint_idx)
        self._set_spin_range_if_changed(self.test_min_spins[row_idx], lo, hi)
        self._set_spin_range_if_changed(self.test_max_spins[row_idx], lo, hi)

    def _refresh_single_test_workspace(self, test_kind: str) -> None:
        if test_kind == "triangle":
            combo = getattr(self, "tri_joint_combo", None)
            min_spin = getattr(self, "tri_min_spin", None)
            max_spin = getattr(self, "tri_max_spin", None)
        else:
            combo = getattr(self, "ff_joint_combo", None)
            min_spin = None
            max_spin = None
        if combo is None:
            return
        joint_idx = self._joint_index_from_combo(combo)
        if joint_idx is None:
            return
        workspace = self._joint_workspace_range(joint_idx, self._state)
        if min_spin is not None and max_spin is not None:
            lo = workspace.current_min_deg if workspace.ready else self._joint_default_target_min_deg(joint_idx)
            hi = self._joint_target_max_deg(joint_idx)
            self._set_spin_range_if_changed(min_spin, lo, hi)
            self._set_spin_range_if_changed(max_spin, lo, hi)

    def _refresh_test_workspace_ranges(self) -> None:
        for row_idx in range(len(getattr(self, "test_joint_combos", []))):
            self._on_test_joint_changed(row_idx)
        self._refresh_single_test_workspace("feedforward")
        self._refresh_single_test_workspace("triangle")

    def _test_workspace_ready(self, joint_idx: int, title: str, show_message: bool) -> bool:
        workspace = self._joint_workspace_range(joint_idx, self._state)
        if workspace.ready:
            return True
        if show_message:
            self._warn(title, f"J{joint_idx:02d} workspace unavailable: {workspace.reason}")
        return False

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
            if not self._test_workspace_ready(joint_idx, "Test Config", show_message):
                return None
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
            self._set_status("Start Test blocked: Control Enable must be ON")
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
        if not self._test_workspace_ready(joint_idx, "Feedforward Test", show_message):
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
            self._set_status("Feedforward Test blocked: Control Enable must be ON")
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
        if not self._test_workspace_ready(joint_idx, "Triangle Wave Test", show_message):
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
            self._set_status("Triangle Wave Test blocked: Control Enable must be ON")
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
            self._set_status("Local Test blocked: Control Enable must be ON")
            return
        self.stop_all_tests("restarted by local test", update_status=False)
        if not self.context.is_comm_connected():
            self._set_status("Local Test blocked: serial port is not connected")
            return
        if not self.feedback_command_allowed(source="Local Test", show_status=True):
            return
        if not self._start_test_recording("local test"):
            return
        try:
            self.backend.set_local_test_params(float(self.local_freq.value()), float(self.local_amp.value()))
            self.backend.start_local_test()
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
                self.backend.stop_local_test()
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
            self._on_stop_test("stopped: Control Enable OFF")
            return
        if not self.backend.is_started():
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
            self._on_stop_ff_test("stopped: Control Enable OFF")
            return
        if not self.backend.is_started():
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
            self._on_stop_tri_test("stopped: Control Enable OFF")
            return
        if not self.backend.is_started():
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
