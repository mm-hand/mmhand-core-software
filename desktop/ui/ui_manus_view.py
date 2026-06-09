import time
from datetime import datetime
from typing import Callable, Optional

from teleop.manus_calibration import (
    ACTUAL_ROBOT_COMMAND_ENABLED,
    DEFAULT_MANUS_SIDE,
    ManusCalibrationManager,
)
from teleop.manus_receiver import MANUS_CONTROL_ENABLED, MANUS_TIMEOUT_S, ManusFrame, ManusReceiver
try:
    from teleop.manus_sdk_receiver import ManusSdkReceiver
except Exception:
    ManusSdkReceiver = None
from protocol import ControlMode, ENCODER_COUNT

from .ui_qt_compat import QtCore, QtGui, QtWidgets


class ManusUdpView(QtWidgets.QWidget):
    def __init__(self, context, selected_joint_mask_getter: Optional[Callable[[], int]] = None, parent=None):
        super().__init__(parent)
        self.context = context
        self.selected_joint_mask_getter = selected_joint_mask_getter
        self._source_mode = "tcp"
        self.receiver = self._make_receiver(self._source_mode)
        self.calibration = ManusCalibrationManager()
        self._last_status = ""
        self._last_displayed_seq: Optional[int] = None
        self._last_sent_seq: Optional[int] = None
        self._sent_count = 0
        self._recording_pose: Optional[str] = None
        self._feedback_angles: list[Optional[float]] = [None] * ENCODER_COUNT
        self._last_sent_targets: list[Optional[float]] = [None] * ENCODER_COUNT
        self._build()
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._refresh)
        self._start_receiver()

    def _build(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        panel = QtWidgets.QGroupBox("MANUS Input / Preview")
        layout = QtWidgets.QVBoxLayout(panel)

        source_row = QtWidgets.QHBoxLayout()
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.addItem("TCP Bridge 127.0.0.1:9001", "tcp")
        self.source_combo.addItem("Direct SDK", "sdk")
        self.source_combo.currentIndexChanged.connect(lambda _idx: self._on_source_changed())
        source_row.addWidget(QtWidgets.QLabel("Source:"))
        source_row.addWidget(self.source_combo, 1)
        layout.addLayout(source_row)

        status_row = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("Stopped")
        self.live_label = QtWidgets.QLabel("STALE")
        self.seq_label = QtWidgets.QLabel("seq=-")
        self.rate_label = QtWidgets.QLabel("recv_hz=-")
        self.age_label = QtWidgets.QLabel("age=-")
        status_row.addWidget(self.status_label, 1)
        status_row.addWidget(self.live_label)
        status_row.addWidget(self.seq_label)
        status_row.addWidget(self.rate_label)
        status_row.addWidget(self.age_label)
        layout.addLayout(status_row)

        info = QtWidgets.QFormLayout()
        self.endpoint_label = QtWidgets.QLabel("")
        self.thread_label = QtWidgets.QLabel("-")
        self.source_label = QtWidgets.QLabel("-")
        self.connection_label = QtWidgets.QLabel("0")
        self.client_label = QtWidgets.QLabel("no")
        self.eof_label = QtWidgets.QLabel("0")
        self.event_label = QtWidgets.QLabel("-")
        self.glove_label = QtWidgets.QLabel("-")
        self.side_label = QtWidgets.QLabel("-")
        self.count_label = QtWidgets.QLabel("0")
        self.update_label = QtWidgets.QLabel("-")
        self.drop_label = QtWidgets.QLabel("0")
        self.packet_label = QtWidgets.QLabel("0")
        self.raw_packet_label = QtWidgets.QLabel("0")
        self.bridge_dt_label = QtWidgets.QLabel("-")
        self.safety_label = QtWidgets.QLabel("MANUS direct motor disabled; joint send gated")
        info.addRow("Listen:", self.endpoint_label)
        info.addRow("Thread:", self.thread_label)
        info.addRow("Client Connected:", self.client_label)
        info.addRow("TCP Connections:", self.connection_label)
        info.addRow("TCP EOF:", self.eof_label)
        info.addRow("Last TCP Event:", self.event_label)
        info.addRow("Last Source:", self.source_label)
        info.addRow("Glove ID:", self.glove_label)
        info.addRow("Side:", self.side_label)
        info.addRow("Ergonomics:", self.count_label)
        info.addRow("Last Update:", self.update_label)
        info.addRow("Bridge dt:", self.bridge_dt_label)
        info.addRow("Raw Frames:", self.raw_packet_label)
        info.addRow("Frames:", self.packet_label)
        info.addRow("Seq Drops:", self.drop_label)
        info.addRow("Safety:", self.safety_label)
        layout.addLayout(info)

        calib = QtWidgets.QGroupBox("MANUS Calibration / Joint Send")
        calib_layout = QtWidgets.QVBoxLayout(calib)
        button_row = QtWidgets.QHBoxLayout()
        self.record_open_btn = QtWidgets.QPushButton("Record Open")
        self.record_close_btn = QtWidgets.QPushButton("Record Close")
        self.record_pinch_btn = QtWidgets.QPushButton("Record Pinch")
        self.record_opposition_btn = QtWidgets.QPushButton("Record Opposition")
        self.load_calib_btn = QtWidgets.QPushButton("Load Calibration")
        for btn in (
            self.record_open_btn,
            self.record_close_btn,
            self.record_pinch_btn,
            self.record_opposition_btn,
            self.load_calib_btn,
        ):
            button_row.addWidget(btn)
        calib_layout.addLayout(button_row)
        send_row = QtWidgets.QHBoxLayout()
        self.send_enable_check = QtWidgets.QCheckBox("Enable Joint Send")
        self.send_enable_check.setChecked(False)
        self.send_enable_check.toggled.connect(lambda checked: self._on_send_enabled(bool(checked)))
        self.side_cfg_label = QtWidgets.QLabel(f"Side: MANUS {DEFAULT_MANUS_SIDE} -> robot left")
        self.send_count_label = QtWidgets.QLabel("sent=0")
        self.last_sent_label = QtWidgets.QLabel("last_sent_seq=-")
        send_row.addWidget(self.send_enable_check)
        send_row.addWidget(self.side_cfg_label)
        send_row.addWidget(self.send_count_label)
        send_row.addWidget(self.last_sent_label)
        send_row.addStretch(1)
        calib_layout.addLayout(send_row)
        self.calib_status_label = QtWidgets.QLabel("Preview only until Enable Joint Send is checked")
        calib_layout.addWidget(self.calib_status_label)
        layout.addWidget(calib)
        self._apply_source_safety()
        self._update_endpoint_label()

        self.record_open_btn.clicked.connect(lambda _checked=False: self._record_calibration("open"))
        self.record_close_btn.clicked.connect(lambda _checked=False: self._record_calibration("close"))
        self.record_pinch_btn.clicked.connect(lambda _checked=False: self._record_calibration("pinch"))
        self.record_opposition_btn.clicked.connect(lambda _checked=False: self._record_calibration("opposition"))
        self.load_calib_btn.clicked.connect(lambda _checked=False: self._load_calibration())

        preview_tabs = QtWidgets.QTabWidget()
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Index", "Name", "Value"])
        self.table.verticalHeader().setVisible(False)
        self._fit_table_header(self.table)
        preview_tabs.addTab(self.table, "Typed Ergonomics")

        self.feature_table = QtWidgets.QTableWidget(0, 6)
        self.feature_table.setHorizontalHeaderLabels([
            "Type", "Human Angle", "Open", "Close", "Normalized", "Valid",
        ])
        self.feature_table.verticalHeader().setVisible(False)
        self._fit_table_header(self.feature_table)
        preview_tabs.addTab(self.feature_table, "Normalized Features")

        self.preview_table = QtWidgets.QTableWidget(0, 8)
        self.preview_table.setHorizontalHeaderLabels([
            "Robot Joint", "Human -> Robot Mapping", "Human Angle", "Normalized", "Joint Limit", "Limit Source", "robot_q_target_preview", "Over Limit",
        ])
        self.preview_table.verticalHeader().setVisible(False)
        self._fit_table_header(self.preview_table)
        preview_tabs.addTab(self.preview_table, "Robot Preview")

        layout.addWidget(preview_tabs, 1)

        root.addWidget(panel, 1)

    @staticmethod
    def _fit_table_header(table: QtWidgets.QTableWidget) -> None:
        header = table.horizontalHeader()
        header.setStretchLastSection(False)
        for col in range(table.columnCount()):
            if col == 1:
                header.setSectionResizeMode(col, QtWidgets.QHeaderView.Stretch)
            else:
                header.setSectionResizeMode(col, QtWidgets.QHeaderView.ResizeToContents)

    def _start_receiver(self) -> None:
        if MANUS_CONTROL_ENABLED or ACTUAL_ROBOT_COMMAND_ENABLED:
            self._set_status("MANUS blocked: direct motor control flag must stay disabled")
            return
        try:
            self.receiver.start()
            self._timer.start()
            self._set_status(self._waiting_status_text())
        except Exception as exc:
            self._set_status(f"MANUS receiver failed: {exc}")

    def _make_receiver(self, mode: str):
        if mode == "sdk":
            if ManusSdkReceiver is None:
                raise RuntimeError("MANUS Direct SDK receiver is not available")
            return ManusSdkReceiver(timeout_s=MANUS_TIMEOUT_S)
        return ManusReceiver(ip="127.0.0.1", port=9001, timeout_s=MANUS_TIMEOUT_S)

    def _on_source_changed(self) -> None:
        mode = str(self.source_combo.currentData() or "tcp")
        if mode == self._source_mode:
            return
        self._timer.stop()
        try:
            self.receiver.stop()
        except Exception:
            pass
        self._source_mode = mode
        self.receiver = self._make_receiver(mode)
        self._last_displayed_seq = None
        self._last_sent_seq = None
        self._apply_source_safety()
        self._update_endpoint_label()
        self._start_receiver()

    def _apply_source_safety(self) -> None:
        if not hasattr(self, "send_enable_check"):
            return
        self.send_enable_check.setEnabled(True)
        self.safety_label.setText("MANUS direct motor disabled; joint send gated")
        self.calib_status_label.setText("Preview only until Enable Joint Send is checked")

    def _update_endpoint_label(self) -> None:
        if not hasattr(self, "endpoint_label"):
            return
        if self._source_mode == "sdk":
            dll_path = getattr(self.receiver, "dll_path", "")
            self.endpoint_label.setText(f"Direct SDK dll={dll_path or '-'} timeout={MANUS_TIMEOUT_S:.1f}s")
        else:
            self.endpoint_label.setText(f"TCP 127.0.0.1:9001 timeout={MANUS_TIMEOUT_S:.1f}s")

    def _waiting_status_text(self) -> str:
        if self._source_mode == "sdk":
            return "MANUS Direct SDK waiting for frames"
        return "MANUS waiting for TCP connection"

    def _frame_waiting_text(self) -> str:
        if self._source_mode == "sdk":
            return "MANUS Direct SDK waiting for frames"
        return "MANUS waiting for TCP frames"

    def _refresh(self) -> None:
        error = self.receiver.last_error()
        frame = self.receiver.get_latest_frame()
        alive = self.receiver.is_alive()
        self._update_live_badge(alive)
        self._update_table_live_style(alive)

        if error:
            self._set_status(error)
        elif frame is None:
            self._set_status(self._frame_waiting_text())
        elif alive:
            self._set_status("MANUS connected")
        else:
            self._set_status("MANUS disconnected / timeout")

        if frame is None:
            return

        self._update_realtime_stats(frame)
        if frame.seq != self._last_displayed_seq:
            self._update_frame(frame)
            preview = self._update_preview(frame)
            self._maybe_send_joint_targets(frame, preview)
            self._last_displayed_seq = frame.seq

    def _update_live_badge(self, alive: bool) -> None:
        if alive:
            self.live_label.setText("LIVE")
            self.live_label.setStyleSheet("font-weight:700;color:white;background:#2e7d32;padding:2px 8px;")
        else:
            self.live_label.setText("STALE")
            self.live_label.setStyleSheet("font-weight:700;color:white;background:#c62828;padding:2px 8px;")

    def _update_realtime_stats(self, frame: ManusFrame) -> None:
        recv_dt = self.receiver.recv_dt()
        age = self.receiver.last_recv_age()
        recv_hz = self.receiver.recv_hz()
        alive = self.receiver.is_alive()

        self.seq_label.setText(f"seq={frame.seq}")
        self.rate_label.setText(f"recv_hz={recv_hz:.1f}" if alive and recv_hz > 0.0 else "recv_hz=-")
        self.age_label.setText(f"age={age * 1000.0:.0f}ms" if age >= 0.0 else "age=-")
        self.packet_label.setText(str(self.receiver.packet_count()))
        self.raw_packet_label.setText(str(self.receiver.raw_packet_count()))
        self.drop_label.setText(str(self.receiver.dropped_frames()))
        self.bridge_dt_label.setText(f"{frame.dt:.4f}s" if frame.dt > 0.0 else "-")
        self.thread_label.setText("running" if self.receiver.thread_alive() else "stopped")
        self.connection_label.setText(str(self.receiver.connection_count()))
        self.client_label.setText("yes" if self.receiver.client_connected() else "no")
        self.eof_label.setText(str(self.receiver.eof_count()))
        self.event_label.setText(self.receiver.last_event() or "-")

    def _update_frame(self, frame: ManusFrame) -> None:
        self.glove_label.setText(str(frame.glove_id))
        self.source_label.setText(frame.source_addr or "-")
        self.side_label.setText(frame.side)
        self.count_label.setText(str(len(frame.ergonomics_values)))
        self.update_label.setText(datetime.fromtimestamp(frame.recv_time).strftime("%H:%M:%S.%f")[:-3])

        row_count = len(frame.ergonomics_values)
        if self.table.rowCount() != row_count:
            self.table.setRowCount(row_count)

        live_brush = QtGui.QBrush(QtGui.QColor("#111111"))
        stale_brush = QtGui.QBrush(QtGui.QColor("#777777"))
        value_brush = live_brush if self.receiver.is_alive() else stale_brush

        for row, value in enumerate(frame.ergonomics_values):
            name = frame.ergonomics_names[row] if row < len(frame.ergonomics_names) else f"ergonomics_{row}"
            for col, text in (
                (0, str(row)),
                (1, name),
                (2, f"{float(value):.6f}"),
            ):
                item = self.table.item(row, col)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                    self.table.setItem(row, col, item)
                item.setText(text)
                item.setForeground(value_brush)

    def _update_preview(self, frame: ManusFrame):
        features = self.calibration.normalized_features(frame)
        self._set_table_rows(self.feature_table, len(features))
        for row, feature in enumerate(features):
            self._set_item(self.feature_table, row, 0, feature.type_name)
            self._set_item(self.feature_table, row, 1, f"{feature.human_joint_angle:.3f}")
            self._set_item(self.feature_table, row, 2, f"{feature.open_value:.3f}")
            self._set_item(self.feature_table, row, 3, f"{feature.close_value:.3f}")
            self._set_item(self.feature_table, row, 4, f"{feature.normalized_feature:.3f}")
            self._set_item(self.feature_table, row, 5, "yes" if feature.valid else "no")

        preview = self.calibration.robot_q_target_preview(frame, side=DEFAULT_MANUS_SIDE)
        self._set_table_rows(self.preview_table, len(preview))
        for row, item in enumerate(preview):
            self._set_item(self.preview_table, row, 0, item.joint_name)
            self._set_item(self.preview_table, row, 1, item.human_joint_mapping)
            self._set_item(self.preview_table, row, 2, f"{item.human_joint_angle:.3f}")
            self._set_item(self.preview_table, row, 3, f"{item.normalized_feature:.3f}")
            self._set_item(self.preview_table, row, 4, item.joint_limit_deg)
            self._set_item(self.preview_table, row, 5, item.joint_limit_source)
            self._set_item(self.preview_table, row, 6, f"{item.robot_q_target_preview:.3f}")
            self._set_item(self.preview_table, row, 7, "yes" if item.over_limit else "no")
        return preview

    def _on_send_enabled(self, checked: bool) -> None:
        if checked:
            if not self.calibration.has_open_close():
                self.send_enable_check.setChecked(False)
                self._set_status("MANUS joint send blocked: record/load Open and Close first")
                return
            try:
                self.context.controller.set_control_mode(ControlMode.JOINT_ANGLE)
                self.context.controller.set_pid_control(True)
            except Exception as exc:
                self.send_enable_check.setChecked(False)
                self._set_status(f"MANUS joint send failed to arm: {exc}")
                return
            self.calib_status_label.setText("MANUS joint send armed: left glove -> left robot joints")
            self._set_status("MANUS joint send armed")
        else:
            try:
                self.context.controller.invalidate_joint_commands()
            except Exception:
                pass
            self.calib_status_label.setText("Preview only until Enable Joint Send is checked")
            self._set_status("MANUS joint send disabled")

    def _maybe_send_joint_targets(self, frame: ManusFrame, preview) -> None:
        if not self.send_enable_check.isChecked():
            return
        if not self.receiver.is_alive():
            self._set_status("MANUS joint send paused: source stale")
            return
        if not self.calibration.has_open_close():
            self.send_enable_check.setChecked(False)
            self._set_status("MANUS joint send disabled: missing Open/Close calibration")
            return
        if frame.seq == self._last_sent_seq:
            return
        angles = [0.0] * ENCODER_COUNT
        for item in preview:
            if 0 <= item.joint_index < len(angles):
                angles[item.joint_index] = float(item.robot_q_target_preview)
        selected_targets, missing_feedback = self._targets_for_selected_joints(angles)
        if selected_targets is None:
            if missing_feedback:
                missing = ",".join(f"J{idx:02d}" for idx in missing_feedback[:4])
                self._set_status(f"MANUS joint send blocked: waiting feedback {missing}")
            else:
                self._set_status("MANUS joint send blocked: no teleop joint selected")
            return
        try:
            self.context.controller.set_control_mode(ControlMode.JOINT_ANGLE)
            self.context.controller.set_pid_control(True)
            self.context.send_joint_targets(selected_targets, source="MANUS left glove joint send", live=True)
            self._last_sent_targets = list(selected_targets)
            self._last_sent_seq = frame.seq
            self._sent_count += 1
            self.send_count_label.setText(f"sent={self._sent_count}")
            self.last_sent_label.setText(f"last_sent_seq={frame.seq}")
        except Exception as exc:
            self.send_enable_check.setChecked(False)
            self._set_status(f"MANUS joint send failed: {exc}")

    def _record_calibration(self, pose: str) -> None:
        if self._recording_pose is not None:
            self._set_status(f"MANUS calibration busy: {self._recording_pose}")
            return
        if self.receiver.get_latest_frame() is None:
            self._set_status("MANUS calibration blocked: no frame")
            return
        self._recording_pose = pose
        self._set_calib_buttons_enabled(False)
        self.calib_status_label.setText(f"Recording {pose} for 1.0s...")
        QtWidgets.QApplication.processEvents()
        QtCore.QTimer.singleShot(0, lambda p=pose: self._finish_record_calibration(p))

    def _finish_record_calibration(self, pose: str) -> None:
        try:
            count = self.calibration.record_pose(pose, self.receiver, duration_s=1.0)
            self.calib_status_label.setText(f"Recorded {pose}: {count} typed ergonomics")
            self._set_status(f"MANUS calibration saved {pose}: {count} values")
        except Exception as exc:
            self.calib_status_label.setText(f"Record {pose} failed: {exc}")
            self._set_status(f"MANUS calibration failed: {exc}")
        finally:
            self._recording_pose = None
            self._set_calib_buttons_enabled(True)

    def _load_calibration(self) -> None:
        try:
            loaded = self.calibration.load()
            text = "Loaded calibration" if loaded else "No calibration file yet"
            self.calib_status_label.setText(text)
            self._set_status(text)
        except Exception as exc:
            self.calib_status_label.setText(f"Load failed: {exc}")
            self._set_status(f"MANUS calibration load failed: {exc}")

    def _set_calib_buttons_enabled(self, enabled: bool) -> None:
        for btn in (
            self.record_open_btn,
            self.record_close_btn,
            self.record_pinch_btn,
            self.record_opposition_btn,
            self.load_calib_btn,
        ):
            btn.setEnabled(bool(enabled))

    def _set_table_rows(self, table: QtWidgets.QTableWidget, rows: int) -> None:
        if table.rowCount() != rows:
            table.setRowCount(rows)

    def _set_item(self, table: QtWidgets.QTableWidget, row: int, col: int, text: str) -> None:
        item = table.item(row, col)
        if item is None:
            item = QtWidgets.QTableWidgetItem()
            item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
            table.setItem(row, col, item)
        item.setText(str(text))

    def _update_table_live_style(self, alive: bool) -> None:
        brush = QtGui.QBrush(QtGui.QColor("#111111" if alive else "#777777"))
        for row in range(self.table.rowCount()):
            for col in range(self.table.columnCount()):
                item = self.table.item(row, col)
                if item is not None:
                    item.setForeground(brush)

    def _selected_joint_mask(self) -> int:
        if callable(self.selected_joint_mask_getter):
            try:
                return int(self.selected_joint_mask_getter()) & ((1 << ENCODER_COUNT) - 1)
            except Exception:
                return 0
        return (1 << ENCODER_COUNT) - 1

    def _targets_for_selected_joints(self, manus_targets: list[float]) -> tuple[Optional[list[float]], list[int]]:
        mask = self._selected_joint_mask()
        if mask == 0:
            return None, []

        targets: list[float] = []
        missing_feedback: list[int] = []
        for idx in range(ENCODER_COUNT):
            if (mask & (1 << idx)) != 0:
                targets.append(float(manus_targets[idx]))
                continue

            hold = self._feedback_angles[idx]
            if hold is None:
                hold = self._last_sent_targets[idx]
            if hold is None:
                missing_feedback.append(idx)
                hold = 0.0
            targets.append(float(hold))

        if missing_feedback:
            return None, missing_feedback
        return targets, []

    def _set_status(self, text: str) -> None:
        text = str(text)
        self.status_label.setText(text)
        if text != self._last_status:
            self.context.set_status(text)
            self._last_status = text

    def update_state(self, state) -> None:
        angles = list(getattr(state, "angles", []) or []) if state is not None else []
        for idx in range(ENCODER_COUNT):
            if idx < len(angles):
                try:
                    self._feedback_angles[idx] = float(angles[idx])
                except (TypeError, ValueError):
                    self._feedback_angles[idx] = None
            else:
                self._feedback_angles[idx] = None

    def closeEvent(self, event):  # noqa: N802 - Qt override
        self._timer.stop()
        self.receiver.stop()
        super().closeEvent(event)


__all__ = ["ManusUdpView"]
