"""ui_device_check_view.py - MMHand Desktop 模块契约。

职责：展示 21 关节 Manual/Auto 数据并完成写入核验。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
from __future__ import annotations

import math
import threading
from types import SimpleNamespace
from typing import Callable, Optional

from desktop.core.angle_mapping import get_spec
from desktop.core.data_models import HandModel, resolve_joint_workspace_range
from desktop.core.protocol import (
    ENCODER_COUNT,
    ENCODER_RAW_MODULO,
    MOTOR_COUNT,
    directed_encoder_delta,
)

from .ui_qt_compat import QtCore, QtGui, QtWidgets, Signal


class DeviceCheckView(QtWidgets.QWidget):
    operation_finished = Signal(str, bool, str)
    manual_prepare_finished = Signal(object)
    manual_capture_finished = Signal(int, bool, object)

    COL_STATUS = 0
    COL_JOINT = 1
    COL_ENCODER_DIRECTION = 2
    COL_CURRENT_RAW = 3
    COL_MANUAL_MIN = 4
    COL_CAPTURE_MIN = 5
    COL_MANUAL_MAX = 6
    COL_CAPTURE_MAX = 7
    COL_MANUAL_SPAN = 8
    COL_AUTO_MIN = 9
    COL_AUTO_MAX = 10
    COL_URDF_SPAN = 11
    COL_URDF_MIN = 12
    COL_CURRENT_MIN = 13
    COL_URDF_MAX = 14
    COL_FLASH = 15

    def __init__(
        self,
        context,
        *,
        mapping_version: int,
        urdf_crc: int,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.context = context
        self.backend = context.backend
        self.mapping_version = int(mapping_version)
        self.urdf_crc = int(urdf_crc) & 0xFFFFFFFF
        self._state: Optional[HandModel] = None
        self._busy = False
        self._dirty_rows: set[int] = set()
        self._current_raw: list[Optional[int]] = [None] * ENCODER_COUNT
        self._status_items: list[QtWidgets.QTableWidgetItem] = []
        self._direction_items: list[QtWidgets.QTableWidgetItem] = []
        self._raw_items: list[QtWidgets.QTableWidgetItem] = []
        self._span_items: list[QtWidgets.QTableWidgetItem] = []
        self._auto_min_items: list[QtWidgets.QTableWidgetItem] = []
        self._auto_max_items: list[QtWidgets.QTableWidgetItem] = []
        self._current_min_items: list[QtWidgets.QTableWidgetItem] = []
        self._flash_items: list[QtWidgets.QTableWidgetItem] = []
        self._manual_min_spins: list[QtWidgets.QSpinBox] = []
        self._manual_max_spins: list[QtWidgets.QSpinBox] = []
        self._capture_min_buttons: list[QtWidgets.QPushButton] = []
        self._capture_max_buttons: list[QtWidgets.QPushButton] = []
        self.operation_finished.connect(self._on_operation_finished)
        self.manual_prepare_finished.connect(self._on_manual_prepare_finished)
        self.manual_capture_finished.connect(self._on_manual_capture_finished)
        self._build()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        toolbar = QtWidgets.QWidget()
        toolbar_layout = QtWidgets.QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(4, 2, 4, 2)
        toolbar_layout.setSpacing(6)
        self.summary_label = QtWidgets.QLabel("Manual 0/21 | Auto 0/21 | P4 RAM unknown")
        self.summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        toolbar_layout.addWidget(self.summary_label)
        toolbar_layout.addStretch(1)

        self.load_calibration_btn = QtWidgets.QPushButton("Load Calibration…")
        self.load_calibration_btn.setToolTip("Validate and import a saved calibration JSON")
        toolbar_layout.addWidget(self.load_calibration_btn)
        self.manual_prepare_btn = QtWidgets.QPushButton("Release Rope")
        self.manual_prepare_btn.setToolTip(
            "Reset online, uniquely routed servo turn counters, clear desktop multi-turn anchors, "
            "invalidate their Auto/LUT, then release these motors by 1500 counts"
        )
        self.clear_btn = QtWidgets.QPushButton("Clear Manual")
        self.clear_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogDiscardButton))
        self.clear_btn.setToolTip("Clear all Manual endpoints from P4 RAM and desktop JSON")
        toolbar_layout.addWidget(self.manual_prepare_btn)
        toolbar_layout.addWidget(self.clear_btn)
        layout.addWidget(toolbar)

        headers = [
            "Status",
            "Joint",
            "Raw Dir",
            "Current Raw",
            "Manual Min",
            "Set",
            "Manual Max",
            "Set",
            "Manual Span",
            "Auto Min",
            "Auto Max",
            "URDF Span (Enc)",
            "URDF Min",
            "Current Min",
            "URDF Max",
            "RAM/JSON",
        ]
        self.table = QtWidgets.QTableWidget(ENCODER_COUNT, len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        direction_header = self.table.horizontalHeaderItem(self.COL_ENCODER_DIRECTION)
        if direction_header is not None:
            direction_header.setToolTip(
                "Magnetic encoder raw direction from Manual Min toward Manual Max: "
                "up means increasing, down means decreasing"
            )
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.verticalHeader().setVisible(False)
        self.table.setSortingEnabled(False)
        self.table.setWordWrap(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

        for joint in range(ENCODER_COUNT):
            self.table.setRowHeight(joint, 30)
            status = self._make_item("Waiting")
            joint_item = self._make_item(f"J{joint:02d}")
            joint_item.setToolTip(get_spec(joint).joint_name)
            direction_item = self._make_item("?")
            direction_item.setToolTip("Waiting for P4 encoderDirection")
            raw = self._make_item("-")
            span = self._make_item("-")
            auto_min = self._make_item("-")
            auto_max = self._make_item("-")
            spec = get_spec(joint)
            urdf_min_deg, urdf_max_deg = spec.urdf_range_deg
            urdf_span_deg = urdf_max_deg - urdf_min_deg
            urdf_span_counts = int(math.floor(
                urdf_span_deg * ENCODER_RAW_MODULO / 360.0 + 0.5
            ))
            urdf_span = self._make_item(str(urdf_span_counts))
            urdf_span.setToolTip(
                f"Theoretical magnetic encoder span: {urdf_span_deg:.2f} deg "
                f"x {ENCODER_RAW_MODULO}/360 = {urdf_span_counts} counts"
            )
            urdf_min = self._make_item(f"{urdf_min_deg:.2f}")
            current_min = self._make_item("-")
            urdf_max = self._make_item(f"{urdf_max_deg:.2f}")
            flash = self._make_item("-")

            min_spin = self._make_raw_spin(joint)
            max_spin = self._make_raw_spin(joint)
            capture_min = self._make_capture_button(joint, is_min=True)
            capture_max = self._make_capture_button(joint, is_min=False)

            self.table.setItem(joint, self.COL_STATUS, status)
            self.table.setItem(joint, self.COL_JOINT, joint_item)
            self.table.setItem(joint, self.COL_ENCODER_DIRECTION, direction_item)
            self.table.setItem(joint, self.COL_CURRENT_RAW, raw)
            self.table.setCellWidget(joint, self.COL_MANUAL_MIN, min_spin)
            self.table.setCellWidget(joint, self.COL_CAPTURE_MIN, capture_min)
            self.table.setCellWidget(joint, self.COL_MANUAL_MAX, max_spin)
            self.table.setCellWidget(joint, self.COL_CAPTURE_MAX, capture_max)
            self.table.setItem(joint, self.COL_MANUAL_SPAN, span)
            self.table.setItem(joint, self.COL_AUTO_MIN, auto_min)
            self.table.setItem(joint, self.COL_AUTO_MAX, auto_max)
            self.table.setItem(joint, self.COL_URDF_SPAN, urdf_span)
            self.table.setItem(joint, self.COL_URDF_MIN, urdf_min)
            self.table.setItem(joint, self.COL_CURRENT_MIN, current_min)
            self.table.setItem(joint, self.COL_URDF_MAX, urdf_max)
            self.table.setItem(joint, self.COL_FLASH, flash)

            self._status_items.append(status)
            self._direction_items.append(direction_item)
            self._raw_items.append(raw)
            self._span_items.append(span)
            self._auto_min_items.append(auto_min)
            self._auto_max_items.append(auto_max)
            self._current_min_items.append(current_min)
            self._flash_items.append(flash)
            min_spin.setReadOnly(True)
            max_spin.setReadOnly(True)
            self._manual_min_spins.append(min_spin)
            self._manual_max_spins.append(max_spin)
            self._capture_min_buttons.append(capture_min)
            self._capture_max_buttons.append(capture_max)

        layout.addWidget(self.table, 1)
        self.manual_prepare_btn.clicked.connect(
            lambda _checked=False: self.manual_prepare()
        )
        self.clear_btn.clicked.connect(lambda _checked=False: self.clear_manual())
        self._refresh_buttons()

    @staticmethod
    def _make_item(text: str) -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        return item

    @staticmethod
    def _encoder_direction_display(direction: int) -> tuple[str, str]:
        """把 P4 encoderDirection 转成用户可直接识别的 raw 增减箭头。"""
        if int(direction) > 0:
            return "↑", "Raw increases from Manual Min toward Manual Max"
        if int(direction) < 0:
            return "↓", "Raw decreases from Manual Min toward Manual Max"
        return "?", "Waiting for P4 encoderDirection"

    def _make_raw_spin(self, joint: int) -> QtWidgets.QSpinBox:
        spin = QtWidgets.QSpinBox()
        spin.setRange(0, ENCODER_RAW_MODULO - 1)
        spin.setAlignment(QtCore.Qt.AlignCenter)
        spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        spin.setToolTip("Raw 14-bit endpoint; Min may be numerically greater than Max across wrap")
        spin.valueChanged.connect(lambda _value, idx=joint: self._mark_dirty(idx))
        return spin

    def _make_capture_button(self, joint: int, *, is_min: bool) -> QtWidgets.QPushButton:
        button = QtWidgets.QPushButton("Set")
        button.setFixedWidth(42)
        endpoint = "Manual Min" if is_min else "Manual Max"
        button.setToolTip(f"Capture latest valid backend raw into {endpoint}; both endpoints are required")
        button.clicked.connect(
            lambda _checked=False, idx=joint, target_min=is_min: self._capture_raw(idx, target_min)
        )
        return button

    def _mark_dirty(self, joint: int) -> None:
        self._dirty_rows.add(int(joint))
        self._refresh_buttons()

    def _capture_raw(self, joint: int, is_min: bool) -> None:
        if self._busy or not self.context.is_comm_connected():
            return
        self._busy = True
        self._refresh_buttons()
        def capture():
            try:
                result = self.backend.capture_manual_endpoint(joint, is_min)
            except Exception as exc:
                result = SimpleNamespace(accepted=False, saved=False, raw=None, message=str(exc))
            self.manual_capture_finished.emit(joint, is_min, result)
        threading.Thread(target=capture, name="mmhand-manual-capture", daemon=True).start()

    def _on_manual_capture_finished(self, joint: int, is_min: bool, result) -> None:
        self._busy = False
        if result.accepted and result.raw is not None:
            target = self._manual_min_spins[joint] if is_min else self._manual_max_spins[joint]
            target.blockSignals(True)
            target.setValue(result.raw)
            target.blockSignals(False)
            self._dirty_rows.add(joint)
        if result.saved:
            self._dirty_rows.discard(joint)
        self._flash_items[joint].setText("Saved" if result.saved else "Not saved")
        self.context.set_status(result.message)
        self.context.emit_action(result.message)
        self._refresh_buttons()
        if not result.accepted:
            QtWidgets.QMessageBox.warning(self, "Manual capture", result.message)

    def _run_operation(self, name: str, callback: Callable[[], tuple[bool, str]]) -> None:
        if self._busy:
            return
        if not self.context.is_comm_connected():
            self.context.set_status(f"{name} blocked: serial port is not connected")
            return
        self._busy = True
        self._refresh_buttons()
        self.context.set_status(f"{name}...")

        def worker() -> None:
            try:
                ok, message = callback()
            except Exception as exc:
                ok, message = False, f"{type(exc).__name__}: {exc}"
            self.operation_finished.emit(name, bool(ok), str(message))

        threading.Thread(target=worker, name=f"mmhand-{name.lower().replace(' ', '-')}", daemon=True).start()

    def manual_prepare(self) -> None:
        """确认后在后台执行全手 turn0 与统一预松，避免阻塞 Qt 主线程。"""
        if self._busy:
            return
        if not self.context.is_comm_connected():
            self.context.set_status("Release Rope blocked: serial port is not connected")
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Release Rope",
            (
                "This operation affects only online servos with unique routes and will:\n\n"
                "1. Reset their hardware turn counts;\n"
                "2. Remove their multi-turn JSON anchors;\n"
                "3. Invalidate Auto/LUT/ABS Limit for their joints while preserving Manual Min/Max;\n"
                "4. Move each servo 1500 counts in the tendon-release direction.\n\n"
                "Automatic calibration is required afterward. Continue?"
            ),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        self._busy = True
        self._refresh_buttons()
        self.context.set_status("Release Rope: checking connected motors...")

        def worker() -> None:
            try:
                result = self.backend.manual_prepare_all_motors()
            except Exception as exc:
                result = SimpleNamespace(
                    ok=False,
                    message=f"{type(exc).__name__}: {exc}",
                    reset_motor_mask=0,
                    failed_motor_mask=(1 << MOTOR_COUNT) - 1,
                    invalidated_joint_mask=0,
                    reached_motor_mask=0,
                )
            self.manual_prepare_finished.emit(result)

        threading.Thread(
            target=worker,
            name="mmhand-manual-prepare",
            daemon=True,
        ).start()

    def clear_manual(self) -> None:
        reply = QtWidgets.QMessageBox.question(
            self,
            "Clear Manual",
            "Clear all 21 Manual endpoint pairs from P4 RAM and desktop JSON? Calibration and feedback will be blocked until Manual is entered again.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        def clear() -> tuple[bool, str]:
            ok = bool(self.backend.clear_manual_encoder_references())
            return (
                ok,
                "Manual endpoints cleared; ACK=ok, readback=ok, desktop JSON=empty"
                if ok else
                "Manual clear failed; ACK/readback/desktop JSON verification failed",
            )

        self._run_operation("Manual clear", clear)

    def _on_operation_finished(self, name: str, ok: bool, message: str) -> None:
        self._busy = False
        if ok and name in ("Manual clear",):
            self._dirty_rows.clear()
        self._refresh_buttons()
        self.context.set_status(message)
        self.context.emit_action(message)
        if not ok:
            QtWidgets.QMessageBox.warning(self, name, message)
        try:
            self.update_state(self.backend.get_state_snapshot())
        except Exception:
            pass

    def _on_manual_prepare_finished(self, result) -> None:
        """收敛维护事务的 UI 状态；详细缓存失效由父 ControlPanel 处理。"""
        self._busy = False
        self._refresh_buttons()
        ok = bool(getattr(result, "ok", False))
        message = str(getattr(result, "message", "Release Rope failed"))
        self.context.set_status(message)
        self.context.emit_action(message)
        if not ok:
            QtWidgets.QMessageBox.warning(self, "Release Rope", message)
        try:
            self.update_state(self.backend.get_state_snapshot())
        except Exception:
            pass

    def _refresh_buttons(self) -> None:
        connected = bool(self.context.is_comm_connected())
        self.manual_prepare_btn.setEnabled(connected and not self._busy)
        self.clear_btn.setEnabled(connected and not self._busy)
        for joint in range(ENCODER_COUNT):
            enabled = connected and not self._busy and self._current_raw[joint] is not None
            self._capture_min_buttons[joint].setEnabled(enabled)
            self._capture_max_buttons[joint].setEnabled(enabled)

    def clear_state(self) -> None:
        self._state = None
        self._current_raw = [None] * ENCODER_COUNT
        self.summary_label.setText("Manual 0/21 | Auto 0/21 | P4 RAM disconnected")
        for joint in range(ENCODER_COUNT):
            self._status_items[joint].setText("Disconnected")
            self._direction_items[joint].setText("?")
            self._direction_items[joint].setToolTip("Waiting for P4 encoderDirection")
            self._raw_items[joint].setText("-")
            self._span_items[joint].setText("-")
            self._auto_min_items[joint].setText("-")
            self._auto_max_items[joint].setText("-")
            self._current_min_items[joint].setText("-")
            self._flash_items[joint].setText("-")
        self._refresh_buttons()

    def update_state(self, state: Optional[HandModel]) -> None:
        self._state = state
        if state is None:
            self.clear_state()
            return

        valid_mask = int(getattr(state, "manual_reference_valid_bitmap", 0) or 0)
        auto_mask = int(getattr(state, "auto_calibration_valid_bitmap", 0) or 0)
        committed = bool(getattr(state, "manual_reference_committed", False))
        commit_status = int(getattr(state, "manual_reference_commit_status", 0) or 0)
        revision = int(getattr(state, "manual_reference_revision", 0) or 0)
        ram_text = "committed" if committed and commit_status == 1 else ("failed" if commit_status == 2 else "not ready")
        self.summary_label.setText(
            f"Manual {valid_mask.bit_count()}/{ENCODER_COUNT} | "
            f"Auto {auto_mask.bit_count()}/{ENCODER_COUNT} | "
            f"P4 RAM {ram_text}, revision={revision}"
        )

        min_raw_values = list(getattr(state, "manual_reference_min_raw", []) or [])
        max_raw_values = list(getattr(state, "manual_reference_max_raw", []) or [])
        directions = list(getattr(state, "manual_reference_encoder_directions", []) or [])
        has_directions = bool(getattr(state, "has_manual_reference_encoder_directions", False))
        results = list(getattr(state, "joint_calib_results", []) or [])
        encoders = list(getattr(state, "encoders", []) or [])

        for joint in range(ENCODER_COUNT):
            bit = 1 << joint
            direction = (
                int(directions[joint])
                if has_directions and joint < len(directions)
                else 0
            )
            direction_text, direction_tip = self._encoder_direction_display(direction)
            self._direction_items[joint].setText(direction_text)
            self._direction_items[joint].setToolTip(direction_tip)
            raw: Optional[int] = None
            if bool(getattr(state, "has_encoder_raw_data", False)) and joint < len(encoders):
                encoder = encoders[joint]
                raw_error = bool(getattr(encoder, "raw_error", getattr(encoder, "error", False)))
                health_ok = True
                if bool(getattr(state, "has_encoder_health_status", False)):
                    health_ok = bool(int(getattr(state, "encoder_online_bitmap", 0) or 0) & bit)
                if not raw_error and health_ok:
                    raw = int(getattr(encoder, "raw_sensor", getattr(encoder, "raw", 0))) & 0x3FFF
            self._current_raw[joint] = raw
            self._raw_items[joint].setText("-" if raw is None else str(raw))

            manual_valid = committed and bool(valid_mask & bit)
            manual_span_ready = False
            span = None
            if manual_valid and joint < len(min_raw_values) and joint < len(max_raw_values):
                min_raw = int(min_raw_values[joint]) & 0x3FFF
                max_raw = int(max_raw_values[joint]) & 0x3FFF
                if joint not in self._dirty_rows:
                    for spin, value in (
                        (self._manual_min_spins[joint], min_raw),
                        (self._manual_max_spins[joint], max_raw),
                    ):
                        spin.blockSignals(True)
                        spin.setValue(value)
                        spin.blockSignals(False)
                span = directed_encoder_delta(min_raw, max_raw, direction)
                manual_span_ready = span is not None and span > 0
                self._span_items[joint].setText("-" if span is None else str(span))
                self._span_items[joint].setToolTip(
                    "Waiting for P4 direction packet"
                    if span is None else
                    ("Saved, but Min and Max are equal; capture the other endpoint before calibration"
                     if span == 0 else
                     f"P4 directed span, direction={direction:+d}")
                )
            else:
                self._span_items[joint].setText("-")

            result = results[joint] if joint < len(results) and isinstance(results[joint], dict) else None
            has_auto_fields = bool(
                auto_mask & bit
                and result
                and result.get("success")
                and result.get("hasManualScaledV4Fields")
            )
            self._auto_min_items[joint].setText(str(int(result.get("autoMinRaw"))) if has_auto_fields else "-")
            self._auto_max_items[joint].setText(str(int(result.get("autoMaxRaw"))) if has_auto_fields else "-")
            self._current_min_items[joint].setText(f"{float(result.get('currentMinDeg')):.2f}" if has_auto_fields else "-")

            spec = get_spec(joint)
            workspace = resolve_joint_workspace_range(
                state,
                joint,
                spec.urdf_range_deg[0],
                spec.urdf_range_deg[1],
                self.mapping_version,
                self.urdf_crc,
            )
            if workspace.ready:
                status_text, color = "READY", "#2e7d32"
            elif not manual_valid:
                status_text, color = "Manual missing", "#c62828"
            elif not manual_span_ready:
                status_text, color = "Manual span zero", "#d17a00"
            elif not bool(auto_mask & bit):
                status_text, color = "Needs auto calib", "#d17a00"
            else:
                status_text, color = "Blocked", "#c62828"
            self._status_items[joint].setText(status_text)
            self._status_items[joint].setToolTip(workspace.reason or "Current workspace is valid")
            self._status_items[joint].setForeground(QtGui.QBrush(QtGui.QColor(color)))

            if not (valid_mask & bit):
                row_flash = "Empty"
            elif committed and commit_status == 1:
                row_flash = "Saved"
            elif commit_status == 2:
                row_flash = "Failed"
            else:
                row_flash = "Pending"
            self._flash_items[joint].setText(row_flash)

        self._refresh_buttons()


__all__ = ["DeviceCheckView"]
