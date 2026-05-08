import math
from typing import List, Optional

import numpy as np

from protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
)
from .qt_compat import QtCore, QtWidgets


JOINT_TARGET_MIN_DEG_BY_INDEX = [0.0] * ENCODER_COUNT
JOINT_TARGET_MAX_DEG_BY_INDEX = [
    63.98, 73.98, 88.00, 92.99, 64.99, 74.99, 88.00, 92.99,
    83.98, 83.98, 83.98, 83.98, 83.98, 83.98, 83.98, 83.98,
    83.98, 83.98, 83.98, 83.98, 83.98,
]
MOTOR_TARGET_MIN = 0
MOTOR_TARGET_MAX = 4095

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
    for i, indices in enumerate(_ANGLE_MAP):
        for j, midx in enumerate(indices):
            if 0 <= midx < ENCODER_COUNT:
                out[midx] = math.degrees(preset_20[i][j])
    return out


class ControlPanel(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.controller = context.controller
        self._updating = False
        self.joint_target_spins: List[QtWidgets.QDoubleSpinBox] = []
        self.joint_sliders: List[QtWidgets.QSlider] = []
        self.joint_current_items: List[QtWidgets.QTableWidgetItem] = []
        self.joint_encoder_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_target_spins: List[QtWidgets.QSpinBox] = []
        self.motor_sliders: List[QtWidgets.QSlider] = []
        self.motor_current_items: List[QtWidgets.QTableWidgetItem] = []
        self.motor_joint_items: List[QtWidgets.QTableWidgetItem] = []
        self._build()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        device = QtWidgets.QGroupBox("Device")
        dev_layout = QtWidgets.QGridLayout(device)
        self.enable_btn = QtWidgets.QPushButton("Enable")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.home_btn = QtWidgets.QPushButton("Home")
        self.reset_btn = QtWidgets.QPushButton("Clear Fault")
        self.calib_btn = QtWidgets.QPushButton("Calibrate")
        for idx, btn in enumerate([self.enable_btn, self.stop_btn, self.home_btn, self.reset_btn, self.calib_btn]):
            dev_layout.addWidget(btn, idx // 3, idx % 3)
        layout.addWidget(device)

        self.enable_btn.clicked.connect(self.start_controller)
        self.stop_btn.clicked.connect(self.stop_controller)
        self.home_btn.clicked.connect(self.apply_home)
        self.reset_btn.clicked.connect(self.reset_controller)
        self.calib_btn.clicked.connect(self.calibrate)

        workflow = QtWidgets.QGroupBox("Workflow")
        wf_layout = QtWidgets.QHBoxLayout(workflow)
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Joint", "Motor", "Test"])
        self.manual_check = QtWidgets.QCheckBox("Hand Manual")
        self.manual_check.setChecked(True)
        self.send_all_btn = QtWidgets.QPushButton("Send All")
        self.open_btn = QtWidgets.QPushButton("Open")
        self.fist_btn = QtWidgets.QPushButton("Fist")
        self.zero_btn = QtWidgets.QPushButton("Zero")
        wf_layout.addWidget(QtWidgets.QLabel("Mode:"))
        wf_layout.addWidget(self.mode_combo)
        wf_layout.addWidget(self.manual_check)
        wf_layout.addStretch(1)
        for btn in [self.open_btn, self.fist_btn, self.zero_btn, self.send_all_btn]:
            wf_layout.addWidget(btn)
        layout.addWidget(workflow)

        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self.send_all_btn.clicked.connect(self.send_all)
        self.open_btn.clicked.connect(lambda: self.apply_preset(_preset_20rad_to_21deg(_OPEN_20_RAD)))
        self.fist_btn.clicked.connect(lambda: self.apply_preset(_preset_20rad_to_21deg(_FIST_20_RAD)))
        self.zero_btn.clicked.connect(lambda: self.apply_preset([0.0] * ENCODER_COUNT))

        self.tabs = QtWidgets.QTabWidget()
        self.joint_table = self._build_joint_table()
        self.motor_table = self._build_motor_table()
        self.test_panel = self._build_test_panel()
        self.teleop_panel = self._build_teleop_panel()
        self.algorithm_panel = self._build_algorithm_panel()
        self.tabs.addTab(self.joint_table, "Joint Channels")
        self.tabs.addTab(self.motor_table, "Motor Channels")
        self.tabs.addTab(self.test_panel, "Test")
        self.tabs.addTab(self.teleop_panel, "Teleop")
        self.tabs.addTab(self.algorithm_panel, "Algorithm")
        layout.addWidget(self.tabs, 1)
        self._on_mode_changed(self.mode_combo.currentText())

    def _make_item(self, text: str = "-") -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        return item

    def _build_joint_table(self) -> QtWidgets.QTableWidget:
        table = QtWidgets.QTableWidget(ENCODER_COUNT, 8)
        table.setHorizontalHeaderLabels(["Joint", "Encoder", "Current", "Min", "Target", "Max", "Send", "Slider"])
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)
        for i in range(ENCODER_COUNT):
            table.setItem(i, 0, self._make_item(f"J{i:02d}"))
            enc = self._make_item("-")
            cur = self._make_item("-")
            self.joint_encoder_items.append(enc)
            self.joint_current_items.append(cur)
            table.setItem(i, 1, enc)
            table.setItem(i, 2, cur)
            table.setItem(i, 3, self._make_item(f"{JOINT_TARGET_MIN_DEG_BY_INDEX[i]:.2f}"))
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(JOINT_TARGET_MIN_DEG_BY_INDEX[i], JOINT_TARGET_MAX_DEG_BY_INDEX[i])
            spin.setDecimals(2)
            spin.setSingleStep(1.0)
            spin.valueChanged.connect(lambda value, idx=i: self._joint_spin_changed(idx, value))
            self.joint_target_spins.append(spin)
            table.setCellWidget(i, 4, spin)
            table.setItem(i, 5, self._make_item(f"{JOINT_TARGET_MAX_DEG_BY_INDEX[i]:.2f}"))
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_joint_row(idx))
            table.setCellWidget(i, 6, send)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(JOINT_TARGET_MIN_DEG_BY_INDEX[i] * 100), int(JOINT_TARGET_MAX_DEG_BY_INDEX[i] * 100))
            slider.valueChanged.connect(lambda value, idx=i: self._joint_slider_changed(idx, value))
            self.joint_sliders.append(slider)
            table.setCellWidget(i, 7, slider)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Stretch)
        return table

    def _build_motor_table(self) -> QtWidgets.QTableWidget:
        table = QtWidgets.QTableWidget(MOTOR_COUNT, 6)
        table.setHorizontalHeaderLabels(["Motor", "Current", "Target", "Send", "Slider", "Joint Angle"])
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
            self.motor_target_spins.append(spin)
            table.setCellWidget(i, 2, spin)
            send = QtWidgets.QPushButton("Send")
            send.clicked.connect(lambda _=False, idx=i: self.send_motor_row(idx))
            table.setCellWidget(i, 3, send)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(MOTOR_TARGET_MIN, MOTOR_TARGET_MAX)
            slider.valueChanged.connect(lambda value, idx=i: self._motor_slider_changed(idx, value))
            self.motor_sliders.append(slider)
            table.setCellWidget(i, 4, slider)
            table.setItem(i, 5, joint)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        return table

    def _build_test_panel(self) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(box)
        layout.addWidget(QtWidgets.QLabel("Qt migration keeps hardware test command entry points. Use Local Test for firmware-side sine testing."))
        row = QtWidgets.QHBoxLayout()
        self.local_freq = QtWidgets.QDoubleSpinBox()
        self.local_freq.setRange(0.01, 20.0)
        self.local_freq.setValue(0.5)
        self.local_amp = QtWidgets.QDoubleSpinBox()
        self.local_amp.setRange(0.0, 120.0)
        self.local_amp.setValue(20.0)
        start = QtWidgets.QPushButton("Start Local Test")
        stop = QtWidgets.QPushButton("Stop Local Test")
        start.clicked.connect(self.start_local_test)
        stop.clicked.connect(self.stop_local_test)
        for w in [QtWidgets.QLabel("Freq Hz"), self.local_freq, QtWidgets.QLabel("Amp deg"), self.local_amp, start, stop]:
            row.addWidget(w)
        row.addStretch(1)
        layout.addLayout(row)
        layout.addStretch(1)
        return box

    def _build_teleop_panel(self) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(box)
        self.teleop_enabled = QtWidgets.QCheckBox("Teleop Enabled")
        layout.addWidget(self.teleop_enabled)
        layout.addWidget(QtWidgets.QLabel("Teleoperation backend remains in example_teleop.py."))
        layout.addStretch(1)
        return box

    def _build_algorithm_panel(self) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(box)
        self.algorithm_enabled = QtWidgets.QCheckBox("Algorithm Enabled")
        self.algorithm_output = QtWidgets.QPlainTextEdit()
        self.algorithm_output.setReadOnly(True)
        layout.addWidget(self.algorithm_enabled)
        layout.addWidget(self.algorithm_output, 1)
        return box

    def _joint_spin_changed(self, idx: int, value: float) -> None:
        if self._updating:
            return
        self.joint_sliders[idx].blockSignals(True)
        self.joint_sliders[idx].setValue(int(round(float(value) * 100)))
        self.joint_sliders[idx].blockSignals(False)

    def _joint_slider_changed(self, idx: int, value: int) -> None:
        if self._updating:
            return
        self.joint_target_spins[idx].blockSignals(True)
        self.joint_target_spins[idx].setValue(float(value) / 100.0)
        self.joint_target_spins[idx].blockSignals(False)

    def _motor_spin_changed(self, idx: int, value: int) -> None:
        if self._updating:
            return
        self.motor_sliders[idx].blockSignals(True)
        self.motor_sliders[idx].setValue(int(value) % 4096)
        self.motor_sliders[idx].blockSignals(False)

    def _motor_slider_changed(self, idx: int, value: int) -> None:
        if self._updating:
            return
        self.motor_target_spins[idx].blockSignals(True)
        self.motor_target_spins[idx].setValue(int(value))
        self.motor_target_spins[idx].blockSignals(False)

    def _on_mode_changed(self, mode: str) -> None:
        mode = str(mode).lower()
        if mode == "motor":
            self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.tabs.setCurrentWidget(self.motor_table)
        else:
            self.controller.set_control_mode(ControlMode.JOINT_ANGLE)
            if mode == "test":
                self.tabs.setCurrentWidget(self.test_panel)
            else:
                self.tabs.setCurrentWidget(self.joint_table)

    def resolved_joint_targets(self) -> List[float]:
        return [float(spin.value()) for spin in self.joint_target_spins]

    def resolved_motor_targets(self) -> List[int]:
        return [int(spin.value()) for spin in self.motor_target_spins]

    def send_joint_row(self, idx: int) -> None:
        targets = self.resolved_joint_targets()
        self.controller.set_target_angles(targets)
        self.context.emit_action(f"Send J{idx:02d}")

    def send_motor_row(self, idx: int) -> None:
        self.controller.set_motor_positions_absolute(self.resolved_motor_targets())
        self.context.emit_action(f"Send M{idx:02d}")

    def send_all(self) -> None:
        if self.mode_combo.currentText().lower() == "motor":
            self.controller.set_motor_positions_absolute(self.resolved_motor_targets())
            self.context.emit_action("Send all motor targets")
        else:
            self.controller.set_target_angles(self.resolved_joint_targets())
            self.context.emit_action("Send all joint targets")

    def apply_preset(self, angles: List[float]) -> None:
        self._updating = True
        for i, value in enumerate(angles[:ENCODER_COUNT]):
            value = max(JOINT_TARGET_MIN_DEG_BY_INDEX[i], min(JOINT_TARGET_MAX_DEG_BY_INDEX[i], float(value)))
            self.joint_target_spins[i].setValue(value)
            self.joint_sliders[i].setValue(int(round(value * 100)))
        self._updating = False
        self.context.draw_pose_angles(angles)

    def apply_home(self) -> None:
        self.apply_preset([0.0] * ENCODER_COUNT)
        self.send_all()

    def start_controller(self) -> None:
        self.controller.start()
        self.context.emit_action("Start")

    def stop_controller(self) -> None:
        self.controller.stop()
        self.context.emit_action("Stop")

    def reset_controller(self) -> None:
        self.controller.reset()
        self.context.emit_action("Reset")

    def calibrate(self) -> None:
        self.context.start_calibration()

    def start_local_test(self) -> None:
        self.controller.set_local_test_params(float(self.local_freq.value()), float(self.local_amp.value()))
        self.controller.start_local_test()
        self.context.emit_action("Start local test")

    def stop_local_test(self) -> None:
        self.controller.stop_local_test()
        self.context.emit_action("Stop local test")

    def append_algorithm_output(self, text: str) -> None:
        self.algorithm_output.appendPlainText(text)
        if self.algorithm_output.blockCount() > 300:
            cursor = self.algorithm_output.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def update_state(self, state) -> None:
        self._updating = True
        for i in range(ENCODER_COUNT):
            if i < len(getattr(state, "angles", [])):
                self.joint_current_items[i].setText(f"{float(state.angles[i]):.2f}")
            if i < len(getattr(state, "encoders", [])):
                enc = state.encoders[i]
                self.joint_encoder_items[i].setText(str(getattr(enc, "raw", "-")))
        servo_raw = (
            getattr(state, "servo_raw_positions", [])
            or getattr(state, "servo_raw", [])
            or getattr(state, "motor_raw", [])
        )
        for i in range(MOTOR_COUNT):
            if i < len(servo_raw):
                self.motor_current_items[i].setText(str(servo_raw[i]))
            if i < len(getattr(state, "angles", [])):
                self.motor_joint_items[i].setText(f"{float(state.angles[min(i, ENCODER_COUNT - 1)]):.2f}")
        self._updating = False
