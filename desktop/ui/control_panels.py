import math
import time
from typing import List, Optional, Tuple

import numpy as np

from data_models import HandModel
from protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
)
from .qt_compat import QtCore, QtGui, QtWidgets
from .tendon_guard import (
    MOTOR_TO_JOINT_INDEX,
    TendonGuard,
    clamp_motor_abs,
    motor_target_from_servo_feedback,
)
from .teleop_view import TeleopView
from .algorithm_view import AlgorithmView


MOTOR_TARGET_SCALE_LO = 0
MOTOR_TARGET_SCALE_HI = 4095
MOTOR_SINGLE_TURN_STEPS = 4096
MOTOR_JUMP_RESYNC_THRESH = 2048
MOTOR_DIR_HINT_CLEAR_TOL = 3
MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES = 4
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

        self.motor_current_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_joint_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_target_spins: List[QtWidgets.QSpinBox] = []
        self.motor_sliders: List[QtWidgets.QSlider] = []
        self.motor_send_buttons: List[QtWidgets.QPushButton] = []
        self._motor_target_user_set: List[bool] = []
        self._motor_target_initialized = False
        self._motor_abs_pos: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_session_origin_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_dir_hint: List[int] = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear: List[bool] = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count: List[int] = [0] * MOTOR_COUNT
        self._motor_last_slider: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_slider_dragging: List[bool] = [False] * MOTOR_COUNT
        self._motor_first_drag_limit_active: List[bool] = [True] * MOTOR_COUNT
        self._motor_last_sent_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT

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
        self.home_btn = QtWidgets.QPushButton("Home")
        self.reset_btn = QtWidgets.QPushButton("Clear Fault")
        self.calib_btn = QtWidgets.QPushButton("Calibrate")
        self.enable_btn.setMinimumHeight(46)
        self.stop_btn.setMinimumHeight(46)
        self.enable_btn.setStyleSheet("font-weight:700;background:#2e7d32;color:white;")
        self.stop_btn.setStyleSheet("font-weight:700;background:#c62828;color:white;")
        dev_layout.addWidget(self.enable_btn, 0, 0, 2, 1)
        dev_layout.addWidget(self.stop_btn, 0, 1, 2, 1)
        dev_layout.addWidget(self.home_btn, 0, 2)
        dev_layout.addWidget(self.calib_btn, 1, 2)
        dev_layout.addWidget(self.reset_btn, 0, 3, 2, 1)
        layout.addWidget(device)

        self.enable_btn.clicked.connect(lambda _checked=False: self.start_controller())
        self.stop_btn.clicked.connect(lambda _checked=False: self.stop_controller())
        self.home_btn.clicked.connect(lambda _checked=False: self.apply_home())
        self.reset_btn.clicked.connect(lambda _checked=False: self.reset_controller())
        self.calib_btn.clicked.connect(lambda _checked=False: self.calibrate())

        self.body_container = QtWidgets.QWidget()
        self.body_layout = QtWidgets.QVBoxLayout(self.body_container)
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.body_container, 1)
        self._build_manual_body()

    def _clear_body(self) -> None:
        self.stop_all_tests("stopped on view switch", update_status=False)
        if self._body_widget is not None:
            self.body_layout.removeWidget(self._body_widget)
            self._body_widget.deleteLater()
            self._body_widget = None
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
        self.motor_current_items = []
        self.motor_joint_items = []
        self.motor_target_spins = []
        self.motor_sliders = []
        self.motor_send_buttons = []
        self._motor_target_user_set = []
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._joint_run_states = [False] * ENCODER_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT

    def _build_manual_body(self) -> None:
        self._clear_body()
        self._reset_manual_widget_refs()
        self._app_mode = "manual"

        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(0, 0, 0, 0)

        workflow = QtWidgets.QGroupBox("Manual Workflow")
        wf_layout = QtWidgets.QHBoxLayout(workflow)
        self.manual_check = QtWidgets.QCheckBox("Hand Manual")
        self.manual_check.setChecked(False)
        self.send_all_btn = QtWidgets.QPushButton("Send All")
        self.open_btn = QtWidgets.QPushButton("Open Pose")
        self.fist_btn = QtWidgets.QPushButton("Fist Pose")
        self.zero_btn = QtWidgets.QPushButton("Zero Pose")
        wf_layout.addWidget(self.manual_check)
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
        layout.addWidget(self.tabs, 1)
        self.body_layout.addWidget(body)
        self._body_widget = body
        self._apply_interaction_enabled(False)
        self._on_manual_tab_changed()

    def _make_item(self, text: str = "-") -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        return item

    def _build_joint_table(self) -> QtWidgets.QTableWidget:
        table = QtWidgets.QTableWidget(ENCODER_COUNT, 11)
        table.setHorizontalHeaderLabels([
            "Joint", "Enc", "Current", "Min", "Target", "Max", "Action",
            "Target Slider", "tie", "+/-", "x1(abs)",
        ])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        for i in range(ENCODER_COUNT):
            table.setItem(i, 0, self._make_item(f"J{i:02d}"))
            status = self._make_item("●")
            status.setTextAlignment(QtCore.Qt.AlignCenter)
            status.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
            enc = self._make_item("-")
            cur = self._make_item("-")
            self.joint_status_items.append(status)
            self.joint_encoder_items.append(enc)
            self.joint_current_items.append(cur)
            table.setItem(i, 1, status)
            table.setItem(i, 2, cur)
            table.setItem(i, 3, self._make_item(f"{self._joint_target_min_deg(i):.2f}"))
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(self._joint_target_min_deg(i), self._joint_target_max_deg(i))
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=i: self._joint_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: self._mark_joint_target_user_set(idx))
            self.joint_target_spins.append(spin)
            table.setCellWidget(i, 4, spin)
            table.setItem(i, 5, self._make_item(f"{self._joint_target_max_deg(i):.2f}"))
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_joint_row(idx))
            self.joint_send_buttons.append(send)
            table.setCellWidget(i, 6, send)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(self._joint_target_min_deg(i) * 100), int(self._joint_target_max_deg(i) * 100))
            slider.valueChanged.connect(lambda value, idx=i: self._joint_slider_changed(idx, value))
            self.joint_sliders.append(slider)
            table.setCellWidget(i, 7, slider)
            table.setItem(i, 8, self._make_item("tie"))
            cfg = self.context.tendon_guard.config[i] if i < len(self.context.tendon_guard.config) else {}
            sign_text = TendonGuard.sign_text(cfg)
            sign_edit = QtWidgets.QLineEdit(sign_text)
            sign_edit.setMaximumWidth(42)
            x1_edit = QtWidgets.QLineEdit(str(int(cfg.get("x1_abs", 0) or 0)))
            x1_edit.setMaximumWidth(84)
            sign_edit.editingFinished.connect(lambda idx=i: self._on_joint_tie_field_commit(idx))
            x1_edit.editingFinished.connect(lambda idx=i: self._on_joint_tie_field_commit(idx))
            self.joint_tie_sign_edits.append(sign_edit)
            self.joint_tie_x1_edits.append(x1_edit)
            table.setCellWidget(i, 9, sign_edit)
            table.setCellWidget(i, 10, x1_edit)
            self._joint_target_user_set.append(False)
            self._control_widgets.extend([spin, send, slider, sign_edit, x1_edit])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Stretch)
        return table

    def _build_motor_table(self) -> QtWidgets.QTableWidget:
        table = QtWidgets.QTableWidget(MOTOR_COUNT, 8)
        table.setHorizontalHeaderLabels(["Motor", "Current", "Target", "Action", "-", "Raw Slider", "+", "Joint Angle"])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        for i in range(MOTOR_COUNT):
            table.setItem(i, 0, self._make_item(f"M{i:02d}"))
            cur = self._make_item("-")
            joint = self._make_item("-")
            self.motor_current_items.append(cur)
            self.motor_joint_items.append(joint)
            table.setItem(i, 1, cur)
            spin = QtWidgets.QSpinBox()
            spin.setRange(MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX)
            spin.setSingleStep(10)
            spin.valueChanged.connect(lambda value, idx=i: self._motor_spin_changed(idx, value))
            spin.editingFinished.connect(lambda idx=i: self._mark_motor_target_user_set(idx))
            self.motor_target_spins.append(spin)
            table.setCellWidget(i, 2, spin)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_motor_row(idx))
            self.motor_send_buttons.append(send)
            table.setCellWidget(i, 3, send)
            minus = QtWidgets.QPushButton("-")
            minus.setMaximumWidth(34)
            minus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, -1))
            table.setCellWidget(i, 4, minus)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(MOTOR_TARGET_SCALE_LO, MOTOR_TARGET_SCALE_HI)
            slider.sliderPressed.connect(lambda idx=i: self._on_motor_slider_press(idx))
            slider.sliderReleased.connect(lambda idx=i: self._on_motor_slider_release(idx))
            slider.valueChanged.connect(lambda value, idx=i: self._motor_slider_changed(idx, value))
            self.motor_sliders.append(slider)
            table.setCellWidget(i, 5, slider)
            plus = QtWidgets.QPushButton("+")
            plus.setMaximumWidth(34)
            plus.clicked.connect(lambda _=False, idx=i: self._step_motor_target(idx, 1))
            table.setCellWidget(i, 6, plus)
            table.setItem(i, 7, joint)
            self._motor_target_user_set.append(False)
            self._control_widgets.extend([spin, send, minus, slider, plus])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)
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
        if current == "motor":
            self._refresh_motor_feedback_view(sync_target=True)
        elif current == "test":
            pass
        else:
            self._refresh_joint_feedback_view(sync_target=True)
        self._last_control_mode = current
        self._apply_control_mode_policy()
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
        for btn in [getattr(self, "send_all_btn", None)]:
            if btn is not None:
                btn.setEnabled(interaction)
        for btn in [getattr(self, "open_btn", None), getattr(self, "fist_btn", None), getattr(self, "zero_btn", None), self.home_btn]:
            if btn is not None:
                btn.setEnabled(interaction and mode in ("joint", "test"))
        self._set_test_button_states()

    def sync_app_mode(self, mode_name: str) -> None:
        mode = str(mode_name or "").lower()
        if "teleop" in mode:
            self._clear_body()
            self._app_mode = "teleop"
            self._body_widget = TeleopView(self.context)
            self.body_layout.addWidget(self._body_widget)
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
        single = self._slider_from_abs_pos(int(value))
        self.motor_sliders[idx].blockSignals(True)
        self.motor_sliders[idx].setValue(single)
        self.motor_sliders[idx].blockSignals(False)

    def _motor_slider_changed(self, idx: int, value: int) -> None:
        if self._updating:
            return
        self.motor_target_spins[idx].blockSignals(True)
        self.motor_target_spins[idx].setValue(int(value))
        self.motor_target_spins[idx].blockSignals(False)
        if self._interaction_enabled:
            self._on_motor_slider_change(idx, int(value))

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
        self._send_motor_abs_from_ui(f"Run M{idx}")
        self._set_row_running(True, idx, True)
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
                return
        self.controller.start()
        self.context.emit_action("Start")

    def stop_controller(self) -> None:
        self.stop_all_tests("stopped by E-STOP", update_status=False)
        try:
            self.controller.stop()
        except Exception:
            pass
        self._clear_run_states()
        self.context.emit_action("Stop")

    def reset_controller(self) -> None:
        self.controller.reset()
        self._clear_run_states()
        self.context.emit_action("Reset")

    def calibrate(self) -> None:
        self.context.start_calibration()

    def append_algorithm_output(self, text: str) -> None:
        if self._algorithm_view is not None:
            self._algorithm_view.append_output(text)

    def update_state(self, state) -> None:
        self._state = state
        if self._app_mode != "manual":
            return
        self._refresh_joint_feedback_view(sync_target=not self._joint_target_initialized)
        self._refresh_motor_feedback_view(sync_target=not self._motor_target_initialized)

    def clear_live_state(self) -> None:
        self.stop_all_tests("stopped on disconnect", update_status=False)
        self._state = None
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        self._motor_abs_pos = [None] * MOTOR_COUNT
        self._motor_session_origin_abs = [None] * MOTOR_COUNT
        self._motor_dir_hint = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count = [0] * MOTOR_COUNT
        self._motor_last_slider = [None] * MOTOR_COUNT
        self._motor_slider_dragging = [False] * MOTOR_COUNT
        self._motor_first_drag_limit_active = [True] * MOTOR_COUNT
        self._motor_last_sent_abs = [None] * MOTOR_COUNT
        self._joint_target_user_set = [False] * ENCODER_COUNT
        self._motor_target_user_set = [False] * MOTOR_COUNT
        self._clear_run_states()
        self._updating = True
        try:
            for item in self.joint_current_items + self.joint_encoder_items:
                item.setText("-")
            for item in self.motor_current_items + self.motor_joint_items:
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
        if self.controller.is_started():
            return True
        self._set_status("Joint command blocked: press Enable first")
        return False

    @staticmethod
    def _slider_from_abs_pos(abs_pos: int) -> int:
        return int(abs_pos) % MOTOR_SINGLE_TURN_STEPS

    @staticmethod
    def _single_turn_diff(a: int, b: int) -> int:
        diff = abs(int(a) - int(b)) % MOTOR_SINGLE_TURN_STEPS
        return min(diff, MOTOR_SINGLE_TURN_STEPS - diff)

    def _read_motor_target_slider_int(self, idx: int) -> int:
        if 0 <= idx < len(self.motor_target_spins):
            return max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, int(self.motor_target_spins[idx].value())))
        return MOTOR_TARGET_SCALE_LO

    def _session_abs_from_slider_locked_turn(self, idx: int, slider_value: int, drag_sign: int = 0) -> Optional[int]:
        if idx < 0 or idx >= MOTOR_COUNT:
            return None
        origin = self._motor_session_origin_abs[idx]
        if origin is None:
            return None
        origin_single = self._slider_from_abs_pos(origin)
        slider = max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, int(slider_value)))
        base = int(slider - origin_single)
        current = self._motor_abs_pos[idx]
        if current is None:
            return clamp_motor_abs(base)
        hint = int(self._motor_dir_hint[idx])
        step = MOTOR_SINGLE_TURN_STEPS
        if hint > 0 or drag_sign > 0:
            k = int((int(current) - base) // step)
            candidate = base + k * step
            if candidate < int(current):
                candidate += step
        elif hint < 0 or drag_sign < 0:
            k = int((int(current) - base) // step)
            candidate = base + k * step
            if candidate > int(current):
                candidate -= step
        else:
            k = int(round((int(current) - base) / float(step)))
            candidate = base + k * step
        return clamp_motor_abs(candidate)

    def _motor_resync_one_channel(self, idx: int) -> None:
        state = self._state
        if idx < 0 or idx >= MOTOR_COUNT or state is None:
            return
        user_sl = self._read_motor_target_slider_int(idx)
        if bool(getattr(state, "has_servo_angle_data", False)) and idx < len(getattr(state, "servo_angles", [])):
            if idx < len(getattr(state, "servo_online", [])) and state.servo_online[idx]:
                dev_abs = clamp_motor_abs(int(state.servo_angles[idx]))
                if self._motor_session_origin_abs[idx] is None:
                    self._motor_session_origin_abs[idx] = dev_abs
                self._motor_abs_pos[idx] = clamp_motor_abs(dev_abs - int(self._motor_session_origin_abs[idx]))
                self._motor_last_slider[idx] = user_sl
                return
        if bool(getattr(state, "has_servo_raw_data", False)) and idx < len(getattr(state, "servo_raw_positions", [])):
            if idx < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[idx]:
                dev_abs = clamp_motor_abs(motor_target_from_servo_feedback(state.servo_raw_positions[idx]))
                if self._motor_session_origin_abs[idx] is None:
                    self._motor_session_origin_abs[idx] = dev_abs
                self._motor_abs_pos[idx] = clamp_motor_abs(dev_abs - int(self._motor_session_origin_abs[idx]))
                self._motor_last_slider[idx] = user_sl

    def _motor_abs_for_command_packet(self, idx: int, state: Optional[HandModel], *, ignore_last_sent: bool = False) -> int:
        session_abs = self._motor_abs_pos[idx]
        origin = self._motor_session_origin_abs[idx]
        if (session_abs is None or origin is None) and state is not None:
            if bool(getattr(state, "has_servo_angle_data", False)):
                if idx < len(getattr(state, "servo_online", [])) and state.servo_online[idx] and idx < len(state.servo_angles):
                    dev_abs = clamp_motor_abs(int(state.servo_angles[idx]))
                    if origin is None:
                        origin = dev_abs
                    if session_abs is None:
                        session_abs = clamp_motor_abs(dev_abs - int(origin))
            elif bool(getattr(state, "has_servo_raw_data", False)):
                if idx < len(getattr(state, "servo_raw_online", [])) and state.servo_raw_online[idx] and idx < len(state.servo_raw_positions):
                    dev_abs = clamp_motor_abs(motor_target_from_servo_feedback(state.servo_raw_positions[idx]))
                    if origin is None:
                        origin = dev_abs
                    if session_abs is None:
                        session_abs = clamp_motor_abs(dev_abs - int(origin))
        if session_abs is not None:
            self._motor_abs_pos[idx] = session_abs
        if origin is not None:
            self._motor_session_origin_abs[idx] = origin
        if session_abs is not None and origin is not None:
            return clamp_motor_abs(int(origin) + int(session_abs))
        if not ignore_last_sent and self._motor_last_sent_abs[idx] is not None:
            return clamp_motor_abs(int(self._motor_last_sent_abs[idx]))
        return 0

    def _build_motor_abs_command_list(self) -> List[int]:
        return [self._motor_abs_for_command_packet(i, self._state) for i in range(MOTOR_COUNT)]

    def _seed_motor_last_sent_after_sync(self, state: HandModel) -> None:
        for i in range(MOTOR_COUNT):
            self._motor_last_sent_abs[i] = self._motor_abs_for_command_packet(i, state, ignore_last_sent=True)

    def _send_motor_abs_from_ui(self, trigger: str) -> None:
        if not self._interaction_enabled:
            return
        if not self.context.is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        if not self.controller.is_started():
            self.controller.start()
        state = self._state
        if state is None or not (
            bool(getattr(state, "has_servo_angle_data", False)) or bool(getattr(state, "has_servo_raw_data", False))
        ):
            self._set_status("Motor command blocked: waiting for motor feedback")
            return
        if all(v is None for v in self._motor_last_sent_abs):
            self._seed_motor_last_sent_after_sync(state)
        for idx in range(MOTOR_COUNT):
            if self._motor_abs_pos[idx] is None:
                self._motor_resync_one_channel(idx)
        targets = self._build_motor_abs_command_list()
        self.controller.set_motor_positions_absolute(targets)
        self._motor_last_sent_abs = list(targets)
        self.context.emit_action(f"Motor absolute positions sent ({trigger})")

    def _sync_motor_abs_from_slider_before_send(self, idx: int) -> None:
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        if self._motor_abs_pos[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            return
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
                        self.joint_encoder_items[i].setText(str(getattr(state.encoders[i], "raw", "-")))
                        self._set_joint_encoder_dot(i, not bool(getattr(state.encoders[i], "error", False)))
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
                if motor_has_raw and i < len(getattr(state, "servo_raw_positions", [])):
                    cur_single = motor_target_from_servo_feedback(state.servo_raw_positions[i])
                    self.motor_current_items[i].setText(f"{cur_single:4d}")
                elif motor_has_abs and i < len(getattr(state, "servo_angles", [])):
                    cur_single = self._slider_from_abs_pos(int(state.servo_angles[i]))
                    self.motor_current_items[i].setText(f"{cur_single:4d}")
                if i < len(getattr(state, "angles", [])):
                    joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else min(i, ENCODER_COUNT - 1)
                    if 0 <= joint_idx < len(state.angles):
                        self.motor_joint_items[i].setText(f"{float(state.angles[joint_idx]):.2f}")
            if (motor_has_raw or motor_has_abs) and sync_target:
                for i in range(MOTOR_COUNT):
                    if motor_has_abs and i < len(getattr(state, "servo_angles", [])):
                        dev_abs = clamp_motor_abs(int(state.servo_angles[i]))
                        self._motor_session_origin_abs[i] = dev_abs
                        self._motor_abs_pos[i] = 0
                        single = self._slider_from_abs_pos(dev_abs)
                    elif motor_has_raw and i < len(getattr(state, "servo_raw_positions", [])):
                        single = motor_target_from_servo_feedback(state.servo_raw_positions[i])
                        self._motor_session_origin_abs[i] = clamp_motor_abs(single)
                        self._motor_abs_pos[i] = 0
                    else:
                        continue
                    self.motor_target_spins[i].setValue(single)
                    self.motor_sliders[i].setValue(single)
                    self._motor_last_slider[i] = single
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
        state = self._state
        if state is not None and bool(getattr(state, "has_servo_angle_data", False)):
            if idx < len(getattr(state, "servo_online", [])) and state.servo_online[idx] and idx < len(state.servo_angles):
                dev_abs = clamp_motor_abs(int(state.servo_angles[idx]))
                if self._motor_session_origin_abs[idx] is None:
                    self._motor_session_origin_abs[idx] = dev_abs
                self._motor_abs_pos[idx] = clamp_motor_abs(dev_abs - int(self._motor_session_origin_abs[idx]))
                single = self._slider_from_abs_pos(dev_abs)
                self._set_motor_target_widget(idx, single)
                self._motor_last_slider[idx] = single
        else:
            self._set_motor_target_widget(idx, self._motor_feedback_target(idx))
            self._motor_resync_one_channel(idx)
        self._motor_target_user_set[idx] = False
        self._send_motor_abs_from_ui(f"Hold M{idx}")

    def _set_motor_target_widget(self, idx: int, value: int) -> None:
        self._updating = True
        try:
            self.motor_target_spins[idx].setValue(int(value))
            self.motor_sliders[idx].setValue(int(value))
        finally:
            self._updating = False

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
        if self._motor_abs_pos[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            return
        new_single = int(value)
        last = self._motor_last_slider[idx]
        drag_sign = 0
        if last is not None:
            diff = new_single - int(last)
            drag_sign = 1 if diff > 0 else (-1 if diff < 0 else 0)
        mapped = self._session_abs_from_slider_locked_turn(idx, new_single, drag_sign)
        if mapped is None:
            return
        if self._motor_first_drag_limit_active[idx]:
            mapped = max(-MOTOR_TARGET_SCALE_HI, min(MOTOR_TARGET_SCALE_HI, int(mapped)))
        if last is not None and abs(new_single - int(last)) > MOTOR_JUMP_RESYNC_THRESH:
            if self._motor_slider_dragging[idx]:
                return
            self._motor_resync_one_channel(idx)
            return
        self._motor_abs_pos[idx] = mapped
        self._motor_last_slider[idx] = new_single
        self._mark_motor_target_user_set(idx)
        self._motor_first_drag_limit_active[idx] = False
        try:
            self._send_motor_abs_from_ui(f"Slider M{idx}")
        except Exception as exc:
            self._set_status(f"Motor slider send failed: {exc}")

    def _step_motor_target(self, idx: int, delta: int) -> None:
        if not self._interaction_enabled:
            return
        if self._motor_abs_pos[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            return
        cur_sl = self._read_motor_target_slider_int(idx)
        if int(delta) > 0:
            if cur_sl < MOTOR_TARGET_SCALE_HI:
                return
            new_sl = MOTOR_TARGET_SCALE_LO
            d_abs = 1
        elif int(delta) < 0:
            if cur_sl > MOTOR_TARGET_SCALE_LO:
                return
            new_sl = MOTOR_TARGET_SCALE_HI
            d_abs = -1
        else:
            return
        self._motor_abs_pos[idx] = clamp_motor_abs(int(self._motor_abs_pos[idx]) + d_abs)
        self._motor_dir_hint[idx] = 1 if d_abs > 0 else -1
        self._motor_dir_hint_pending_clear[idx] = True
        self._motor_dir_hint_stable_count[idx] = 0
        self._set_motor_target_widget(idx, new_sl)
        self._motor_last_slider[idx] = new_sl
        self._mark_motor_target_user_set(idx)
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
