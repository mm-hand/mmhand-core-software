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
from typing import Callable, Optional

from desktop.core.angle_mapping import get_spec
from desktop.core.data_models import HandModel, resolve_joint_workspace_range
from desktop.core.protocol import ENCODER_COUNT, ENCODER_RAW_MODULO, directed_encoder_delta

from .ui_qt_compat import QtCore, QtGui, QtWidgets, Signal


class DeviceCheckView(QtWidgets.QWidget):
    operation_finished = Signal(str, bool, str)

    COL_STATUS = 0
    COL_JOINT = 1
    COL_CURRENT_RAW = 2
    COL_MANUAL_MIN = 3
    COL_CAPTURE_MIN = 4
    COL_MANUAL_MAX = 5
    COL_CAPTURE_MAX = 6
    COL_MANUAL_SPAN = 7
    COL_AUTO_MIN = 8
    COL_AUTO_MAX = 9
    COL_URDF_SPAN = 10
    COL_URDF_MIN = 11
    COL_CURRENT_MIN = 12
    COL_URDF_MAX = 13
    COL_FLASH = 14

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
        self._build()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        toolbar = QtWidgets.QWidget()
        toolbar_layout = QtWidgets.QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(4, 2, 4, 2)
        toolbar_layout.setSpacing(6)
        self.summary_label = QtWidgets.QLabel("Manual 0/21 | Auto 0/21 | Flash unknown")
        self.summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        toolbar_layout.addWidget(self.summary_label)
        toolbar_layout.addStretch(1)

        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.refresh_btn.setToolTip("Query Manual endpoints from P4 and verify the returned packet")
        self.write_btn = QtWidgets.QPushButton("Write Modified")
        self.write_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogSaveButton))
        self.write_btn.setToolTip("Persist only rows edited in this table, then read them back")
        self.clear_btn = QtWidgets.QPushButton("Clear Manual")
        self.clear_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogDiscardButton))
        self.clear_btn.setToolTip("Clear all Manual endpoints from P4 Flash")
        toolbar_layout.addWidget(self.refresh_btn)
        toolbar_layout.addWidget(self.write_btn)
        toolbar_layout.addWidget(self.clear_btn)
        layout.addWidget(toolbar)

        headers = [
            "Status",
            "Joint",
            "Current Raw",
            "Manual Min",
            "Use Raw",
            "Manual Max",
            "Use Raw",
            "Manual Span",
            "Auto Min",
            "Auto Max",
            "URDF Span (Enc)",
            "URDF Min",
            "Current Min",
            "URDF Max",
            "Flash",
        ]
        self.table = QtWidgets.QTableWidget(ENCODER_COUNT, len(headers))
        self.table.setHorizontalHeaderLabels(headers)
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
            self._raw_items.append(raw)
            self._span_items.append(span)
            self._auto_min_items.append(auto_min)
            self._auto_max_items.append(auto_max)
            self._current_min_items.append(current_min)
            self._flash_items.append(flash)
            self._manual_min_spins.append(min_spin)
            self._manual_max_spins.append(max_spin)
            self._capture_min_buttons.append(capture_min)
            self._capture_max_buttons.append(capture_max)

        layout.addWidget(self.table, 1)
        self.refresh_btn.clicked.connect(lambda _checked=False: self.refresh_from_p4())
        self.write_btn.clicked.connect(lambda _checked=False: self.write_modified())
        self.clear_btn.clicked.connect(lambda _checked=False: self.clear_manual())
        self._refresh_buttons()

    @staticmethod
    def _make_item(text: str) -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        return item

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
        button.setToolTip(f"Copy this joint's Current Raw into {endpoint}")
        button.clicked.connect(
            lambda _checked=False, idx=joint, target_min=is_min: self._capture_raw(idx, target_min)
        )
        return button

    def _mark_dirty(self, joint: int) -> None:
        self._dirty_rows.add(int(joint))
        self._refresh_buttons()

    def _capture_raw(self, joint: int, is_min: bool) -> None:
        raw = self._current_raw[joint] if 0 <= joint < len(self._current_raw) else None
        if raw is None:
            self.context.set_status(f"J{joint:02d} capture blocked: Current Raw is unavailable")
            return
        target = self._manual_min_spins[joint] if is_min else self._manual_max_spins[joint]
        target.setValue(int(raw))
        self._mark_dirty(joint)

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

    def refresh_from_p4(self) -> None:
        self._run_operation(
            "Manual refresh",
            lambda: (
                bool(self.backend.query_manual_encoder_references()),
                "Manual endpoints read back" if self.backend.get_current_state() is not None else "Manual query completed",
            ),
        )

    def write_modified(self) -> None:
        rows = sorted(self._dirty_rows)
        if not rows:
            self.context.set_status("Manual write skipped: no modified row")
            return
        values = [
            (
                joint,
                int(self._manual_min_spins[joint].value()),
                int(self._manual_max_spins[joint].value()),
            )
            for joint in rows
        ]

        def write_rows() -> tuple[bool, str]:
            completed: list[int] = []
            for joint, min_raw, max_raw in values:
                if not self.backend.set_manual_encoder_reference(joint, min_raw, max_raw):
                    done = ",".join(f"J{idx:02d}" for idx in completed) or "none"
                    return False, (
                        f"Manual write failed: J{joint:02d} min={min_raw} max={max_raw}; "
                        f"completed={done}; ACK/readback/CRC/Flash verification failed"
                    )
                completed.append(joint)
            self.backend.query_manual_encoder_references()
            details = ",".join(
                f"J{joint:02d}[min={min_raw},max={max_raw}]"
                for joint, min_raw, max_raw in values
            )
            return True, (
                f"Manual saved and verified: {details}; "
                "ACK=ok, readback=ok, CRC=ok, Flash=saved; automatic calibration is required"
            )

        self._run_operation("Manual write", write_rows)

    def clear_manual(self) -> None:
        reply = QtWidgets.QMessageBox.question(
            self,
            "Clear Manual",
            "Clear all 21 Manual endpoint pairs from P4 Flash? Calibration and feedback will be blocked until Manual is entered again.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        def clear() -> tuple[bool, str]:
            ok = bool(self.backend.clear_manual_encoder_references())
            return (
                ok,
                "Manual endpoints cleared; ACK=ok, readback=ok, Flash=empty"
                if ok else
                "Manual clear failed; ACK/readback/Flash verification failed",
            )

        self._run_operation("Manual clear", clear)

    def _on_operation_finished(self, name: str, ok: bool, message: str) -> None:
        self._busy = False
        if ok and name in ("Manual write", "Manual clear"):
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

    def _refresh_buttons(self) -> None:
        connected = bool(self.context.is_comm_connected())
        self.refresh_btn.setEnabled(connected and not self._busy)
        self.write_btn.setEnabled(connected and not self._busy and bool(self._dirty_rows))
        self.clear_btn.setEnabled(connected and not self._busy)
        for joint in range(ENCODER_COUNT):
            enabled = connected and not self._busy and self._current_raw[joint] is not None
            self._capture_min_buttons[joint].setEnabled(enabled)
            self._capture_max_buttons[joint].setEnabled(enabled)

    def clear_state(self) -> None:
        self._state = None
        self._current_raw = [None] * ENCODER_COUNT
        self.summary_label.setText("Manual 0/21 | Auto 0/21 | Flash disconnected")
        for joint in range(ENCODER_COUNT):
            self._status_items[joint].setText("Disconnected")
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
        nvs_valid = bool(getattr(state, "manual_reference_nvs_valid", False))
        save_status = int(getattr(state, "manual_reference_save_status", 0) or 0)
        sequence = int(getattr(state, "manual_reference_nvs_sequence", 0) or 0)
        flash_text = "saved" if nvs_valid and save_status == 1 else ("failed" if save_status == 2 else "not ready")
        self.summary_label.setText(
            f"Manual {valid_mask.bit_count()}/{ENCODER_COUNT} | "
            f"Auto {auto_mask.bit_count()}/{ENCODER_COUNT} | "
            f"Flash {flash_text}, sequence={sequence}"
        )

        min_raw_values = list(getattr(state, "manual_reference_min_raw", []) or [])
        max_raw_values = list(getattr(state, "manual_reference_max_raw", []) or [])
        directions = list(getattr(state, "manual_reference_encoder_directions", []) or [])
        has_directions = bool(getattr(state, "has_manual_reference_encoder_directions", False))
        results = list(getattr(state, "joint_calib_results", []) or [])
        encoders = list(getattr(state, "encoders", []) or [])

        for joint in range(ENCODER_COUNT):
            bit = 1 << joint
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

            manual_valid = nvs_valid and bool(valid_mask & bit)
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
                direction = int(directions[joint]) if has_directions and joint < len(directions) else 0
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
            elif nvs_valid and save_status == 1:
                row_flash = "Saved"
            elif save_status == 2:
                row_flash = "Failed"
            else:
                row_flash = "Pending"
            self._flash_items[joint].setText(row_flash)

        self._refresh_buttons()


__all__ = ["DeviceCheckView"]
