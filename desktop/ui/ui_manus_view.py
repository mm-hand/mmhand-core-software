import json
import os
import subprocess
import sys
import time
from datetime import datetime
import math
from typing import Callable, Optional

from desktop.teleop.manus_calibration import (
    ACTUAL_ROBOT_COMMAND_ENABLED,
    DEFAULT_MANUS_SIDE,
    ManusCalibrationManager,
)
from desktop.teleop.manus_raw_skeleton_retarget import FullRawSkeletonRetargeter
from desktop.teleop.manus_receiver import MANUS_CONTROL_ENABLED, MANUS_TIMEOUT_S, ManusFrame, ManusReceiver
try:
    from desktop.teleop.manus_sdk_receiver import ManusSdkReceiver
except Exception:
    ManusSdkReceiver = None
from desktop.core.protocol import ControlMode, ENCODER_COUNT
from desktop.teleop.joint_target_adapter import (
    LEGACY_CALIBRATED_DEG,
    URDF_DEG,
    active_mapping_identity,
    adapt_joint_targets,
)

from .ui_qt_compat import QtCore, QtGui, QtWidgets


class ManusUdpView(QtWidgets.QWidget):
    def __init__(self, context, selected_joint_mask_getter: Optional[Callable[[], int]] = None, parent=None):
        super().__init__(parent)
        self.context = context
        self.selected_joint_mask_getter = selected_joint_mask_getter
        self._source_mode = "sdk"
        self._send_enabled = False
        self.receiver = self._make_receiver(self._source_mode)
        self.calibration = ManusCalibrationManager()
        self.raw_skeleton_retargeter = FullRawSkeletonRetargeter(calibration_manager=self.calibration)
        self._last_status = ""
        self._last_displayed_seq: Optional[int] = None
        self._last_sent_seq: Optional[int] = None
        self._sent_count = 0
        self._recording_pose: Optional[str] = None
        self._feedback_angles: list[Optional[float]] = [None] * ENCODER_COUNT
        self._last_sent_targets: list[Optional[float]] = [None] * ENCODER_COUNT
        self._last_raw_skeleton_preview = []
        self._last_urdf_preview_t = 0.0
        self._urdf_viewer_process: Optional[subprocess.Popen] = None
        self.urdf_viewer_status_label: Optional[QtWidgets.QLabel] = None
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
        self.source_combo.addItem("Direct SDK", "sdk")
        self.source_combo.addItem("TCP Bridge :9001", "tcp")
        self.source_combo.currentIndexChanged.connect(lambda _idx: self._on_source_changed())
        source_row.addWidget(QtWidgets.QLabel("Source:"))
        source_row.addWidget(self.source_combo, 1)
        layout.addLayout(source_row)

        retarget_row = QtWidgets.QHBoxLayout()
        self.retarget_combo = QtWidgets.QComboBox()
        self.retarget_combo.addItem("Full raw skeleton IK", "raw_pinky")
        self.retarget_combo.addItem("Ergonomics angle mapping", "ergonomics")
        self.retarget_combo.currentIndexChanged.connect(lambda _idx: self._on_retarget_changed())
        retarget_row.addWidget(QtWidgets.QLabel("Retarget:"))
        retarget_row.addWidget(self.retarget_combo, 1)
        layout.addLayout(retarget_row)

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
        calib = QtWidgets.QGroupBox("MANUS Calibration / Joint Send")
        calib_layout = QtWidgets.QVBoxLayout(calib)
        button_row = QtWidgets.QHBoxLayout()
        self.record_align_btn = QtWidgets.QPushButton("Record Align")
        self.record_open_aa_btn = QtWidgets.QPushButton("Record Open AA")
        self.record_fist_flex_btn = QtWidgets.QPushButton("Record Fist Flex")
        self.record_thumb_pinky_root_btn = QtWidgets.QPushButton("Record Thumb->PinkyRoot")
        self.load_calib_btn = QtWidgets.QPushButton("Load Calibration")
        for btn in (
            self.record_align_btn,
            self.record_open_aa_btn,
            self.record_fist_flex_btn,
            self.record_thumb_pinky_root_btn,
            self.load_calib_btn,
        ):
            button_row.addWidget(btn)
        calib_layout.addLayout(button_row)
        send_row = QtWidgets.QHBoxLayout()
        self.send_btn = QtWidgets.QPushButton("Send")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.send_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.stop_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop))
        self.send_btn.clicked.connect(lambda _checked=False: self._start_joint_send())
        self.stop_btn.clicked.connect(lambda _checked=False: self._stop_joint_send())
        self.side_cfg_label = QtWidgets.QLabel(f"Side: MANUS {DEFAULT_MANUS_SIDE} -> robot left")
        self.send_count_label = QtWidgets.QLabel("sent=0")
        self.last_sent_label = QtWidgets.QLabel("last_sent_seq=-")
        send_row.addWidget(self.send_btn)
        send_row.addWidget(self.stop_btn)
        send_row.addWidget(self.side_cfg_label)
        send_row.addWidget(self.send_count_label)
        send_row.addWidget(self.last_sent_label)
        send_row.addStretch(1)
        calib_layout.addLayout(send_row)
        self.calib_status_label = QtWidgets.QLabel("Preview only until Send is pressed")
        calib_layout.addWidget(self.calib_status_label)
        layout.addWidget(calib)
        self._apply_source_safety()
        self._update_endpoint_label()

        self.record_align_btn.clicked.connect(lambda _checked=False: self._record_calibration("align"))
        self.record_open_aa_btn.clicked.connect(lambda _checked=False: self._record_calibration("open_aa"))
        self.record_fist_flex_btn.clicked.connect(lambda _checked=False: self._record_calibration("fist_flex"))
        self.record_thumb_pinky_root_btn.clicked.connect(lambda _checked=False: self._record_calibration("thumb_pinky_root"))
        self.load_calib_btn.clicked.connect(lambda _checked=False: self._load_calibration())

        preview_tabs = QtWidgets.QTabWidget()
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Index", "Name", "Value"])
        self.table.verticalHeader().setVisible(False)
        self._fit_table_header(self.table)
        preview_tabs.addTab(self.table, "Typed Ergonomics")

        self.preview_table = QtWidgets.QTableWidget(0, 9)
        self.preview_table.setHorizontalHeaderLabels([
            "Robot Joint", "Human -> Robot Mapping", "Human Delta", "Gain", "Legacy Limit", "Limit Source", "Legacy q_target", "URDF Target", "Over Limit",
        ])
        self.preview_table.verticalHeader().setVisible(False)
        self._fit_table_header(self.preview_table)
        preview_tabs.addTab(self.preview_table, "Ergonomics Angle Mapping")

        self.raw_skeleton_table = QtWidgets.QTableWidget(0, 13)
        self.raw_skeleton_table.setHorizontalHeaderLabels([
            "Source", "Node", "ID", "Parent", "X", "Y", "Z", "Norm", "Qw", "Qx", "Qy", "Qz", "Rot",
        ])
        self.raw_skeleton_table.verticalHeader().setVisible(False)
        self._fit_table_header(self.raw_skeleton_table)
        preview_tabs.addTab(self.raw_skeleton_table, "Raw Skeleton")

        self.raw_skeleton_preview_table = QtWidgets.QTableWidget(0, 9)
        self.raw_skeleton_preview_table.setHorizontalHeaderLabels([
            "Robot Joint", "Target Debug", "Robot Target XYZ", "IK Error mm", "Joint Limit", "q_urdf", "q_target", "Valid", "Send",
        ])
        self.raw_skeleton_preview_table.verticalHeader().setVisible(False)
        self._fit_table_header(self.raw_skeleton_preview_table)
        preview_tabs.addTab(self.raw_skeleton_preview_table, "Raw IK")

        urdf_viewer_panel = QtWidgets.QWidget()
        urdf_viewer_layout = QtWidgets.QVBoxLayout(urdf_viewer_panel)
        urdf_viewer_row = QtWidgets.QHBoxLayout()
        launch_viewer_btn = QtWidgets.QPushButton("Launch URDF Viewer")
        launch_viewer_btn.clicked.connect(lambda _checked=False: self._launch_urdf_viewer())
        urdf_viewer_row.addWidget(launch_viewer_btn)
        urdf_viewer_row.addStretch(1)
        urdf_viewer_layout.addLayout(urdf_viewer_row)
        self.urdf_viewer_status_label = QtWidgets.QLabel(
            f"active-URDF target stream: {self._urdf_preview_stream_path()}"
        )
        self.urdf_viewer_status_label.setWordWrap(True)
        urdf_viewer_layout.addWidget(self.urdf_viewer_status_label)
        urdf_viewer_layout.addStretch(1)
        preview_tabs.addTab(urdf_viewer_panel, "URDF Viewer")

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
        return ManusReceiver(ip="0.0.0.0", port=9001, timeout_s=MANUS_TIMEOUT_S)

    def _on_source_changed(self) -> None:
        mode = str(self.source_combo.currentData() or "sdk")
        if mode == self._source_mode:
            return
        self._stop_joint_send("MANUS joint send stopped: source changed")
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
        if not hasattr(self, "send_btn") or not hasattr(self, "stop_btn"):
            return
        self.send_btn.setEnabled(not self._send_enabled)
        self.stop_btn.setEnabled(self._send_enabled)
        self.safety_label.setText("MANUS direct motor disabled; joint send gated")

    def _update_endpoint_label(self) -> None:
        if not hasattr(self, "endpoint_label"):
            return
        if self._source_mode == "sdk":
            dll_path = getattr(self.receiver, "dll_path", "")
            self.endpoint_label.setText(f"Direct SDK dll={dll_path or '-'} timeout={MANUS_TIMEOUT_S:.1f}s")
        else:
            self.endpoint_label.setText(f"TCP 0.0.0.0:9001 timeout={MANUS_TIMEOUT_S:.1f}s")

    def _waiting_status_text(self) -> str:
        if self._source_mode == "sdk":
            return "MANUS Direct SDK waiting for frames"
        return "MANUS waiting for TCP connection"

    def _frame_waiting_text(self) -> str:
        if self._source_mode == "sdk":
            return "MANUS Direct SDK waiting for frames"
        return "MANUS waiting for TCP frames"

    def _on_retarget_changed(self) -> None:
        self._stop_joint_send("MANUS joint send stopped: retarget changed")
        mode = self._retarget_mode()
        if mode == "raw_pinky":
            self.calib_status_label.setText("Full raw skeleton IK selected; Send drives selected J00-J20")
        else:
            self.calib_status_label.setText("Ergonomics angle mapping selected; Send uses Open/Close calibration")
        self._apply_source_safety()
        self._set_status(f"MANUS retarget selected: {self.retarget_combo.currentText()}")

    def _retarget_mode(self) -> str:
        if hasattr(self, "retarget_combo"):
            return str(self.retarget_combo.currentData() or "raw_pinky")
        return "raw_pinky"

    def _refresh(self) -> None:
        error = self.receiver.last_error()
        recv_call_start = time.perf_counter()
        frame = self.receiver.get_latest_frame()
        recv_call_end = time.perf_counter()
        if frame is not None:
            frame.timing_recv_call_ms = (recv_call_end - recv_call_start) * 1000.0
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
            self._update_raw_skeleton(frame)
            raw_skeleton_preview = self._update_raw_skeleton_preview(frame)
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
        preview = self.calibration.robot_q_target_preview(frame, side=DEFAULT_MANUS_SIDE)
        self._set_table_rows(self.preview_table, len(preview))
        for row, item in enumerate(preview):
            self._set_item(self.preview_table, row, 0, item.joint_name)
            self._set_item(self.preview_table, row, 1, item.human_joint_mapping)
            self._set_item(self.preview_table, row, 2, f"{item.human_delta_deg:.3f}")
            self._set_item(self.preview_table, row, 3, f"{item.gain_deg_per_human_deg:.3f}")
            self._set_item(self.preview_table, row, 4, item.joint_limit_deg)
            self._set_item(self.preview_table, row, 5, item.joint_limit_source)
            self._set_item(self.preview_table, row, 6, f"{item.robot_q_target_preview:.3f}")
            self._set_item(self.preview_table, row, 7, f"{item.robot_urdf_target_preview:.3f}")
            self._set_item(self.preview_table, row, 8, "yes" if item.over_limit else "no")
        return preview

    def _update_raw_skeleton_preview(self, frame: ManusFrame):
        preview = self.raw_skeleton_retargeter.preview(frame)
        self._last_raw_skeleton_preview = preview
        self._set_table_rows(self.raw_skeleton_preview_table, len(preview))
        send_mode = self._retarget_mode() == "raw_pinky"
        for row, item in enumerate(preview):
            self._set_item(self.raw_skeleton_preview_table, row, 0, item.joint_name)
            self._set_item(self.raw_skeleton_preview_table, row, 1, item.target_debug)
            self._set_item(self.raw_skeleton_preview_table, row, 2, item.target_xyz)
            self._set_item(self.raw_skeleton_preview_table, row, 3, f"{item.ik_error_mm:.2f}")
            self._set_item(self.raw_skeleton_preview_table, row, 4, item.joint_limit_deg)
            self._set_item(self.raw_skeleton_preview_table, row, 5, f"{item.robot_q_urdf_preview:.3f}")
            self._set_item(self.raw_skeleton_preview_table, row, 6, f"{item.robot_q_target_preview:.3f}")
            self._set_item(self.raw_skeleton_preview_table, row, 7, "yes" if item.valid else "no")
            self._set_item(self.raw_skeleton_preview_table, row, 8, "armed" if send_mode else "preview")
        raw_angles = [0.0] * ENCODER_COUNT
        for item in preview:
            if item.valid and 0 <= item.joint_index < ENCODER_COUNT:
                raw_angles[item.joint_index] = float(item.robot_q_urdf_preview)
        if self._retarget_mode() == "raw_pinky":
            self._update_urdf_preview_angles(raw_angles)
        return preview

    def _update_urdf_preview_angles(self, angles: list[float]) -> None:
        # Stream canonical active-URDF degrees; the viewer performs no legacy
        # sign or range conversion.
        now = time.monotonic()
        if now - self._last_urdf_preview_t < 0.030:
            return
        self._last_urdf_preview_t = now
        try:
            self._write_urdf_preview_stream(list(angles))
        except Exception as exc:
            self._set_status(f"URDF viewer stream write failed: {exc}")

    @staticmethod
    def _urdf_preview_stream_path() -> str:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(root, "config", "raw_ik_q_target.json")

    def _write_urdf_preview_stream(self, angles: list[float]) -> None:
        path = self._urdf_preview_stream_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        values = [0.0] * ENCODER_COUNT
        for idx, value in enumerate(list(angles)[:ENCODER_COUNT]):
            values[idx] = float(value)
        payload = {
            "timestamp": time.time(),
            "angles_deg": values,
            "angle_space": URDF_DEG,
            "source": "raw_ik_urdf_target",
        }
        mapping = active_mapping_identity()
        payload.update(
            {
                "mapping_id": mapping.mapping_id,
                "mapping_version": mapping.mapping_version,
                "urdf_config_crc": mapping.urdf_config_crc,
            }
        )
        tmp_path = f"{path}.tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, separators=(",", ":"))
        os.replace(tmp_path, path)

    def _launch_urdf_viewer(self) -> None:
        if self._urdf_viewer_process is not None and self._urdf_viewer_process.poll() is None:
            self._set_status("URDF viewer already running")
            return
        desktop_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        viewer_script = os.path.join(desktop_root, "tools", "raw_ik_urdf_viewer.py")
        try:
            self._urdf_viewer_process = subprocess.Popen(
                [sys.executable, viewer_script, "--stream", self._urdf_preview_stream_path()],
                cwd=desktop_root,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            self._set_status("URDF viewer launched")
            if self.urdf_viewer_status_label is not None:
                self.urdf_viewer_status_label.setText(
                    f"URDF viewer running. active-URDF target stream: {self._urdf_preview_stream_path()}"
                )
        except Exception as exc:
            self._set_status(f"URDF viewer launch failed: {exc}")

    @staticmethod
    def _node_pos(node) -> Optional[list[float]]:
        if not isinstance(node, dict):
            return None
        pos = node.get("pos")
        if not isinstance(pos, list) or len(pos) < 3:
            return None
        try:
            return [float(pos[0]), float(pos[1]), float(pos[2])]
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _sub_pos(a: list[float], b: list[float]) -> list[float]:
        return [float(a[i]) - float(b[i]) for i in range(3)]

    @staticmethod
    def _norm_pos(pos: list[float]) -> float:
        return math.sqrt(sum(float(v) * float(v) for v in pos[:3]))

    @staticmethod
    def _node_rot(node) -> Optional[list[float]]:
        if not isinstance(node, dict):
            return None
        rot = node.get("rot")
        if not isinstance(rot, list) or len(rot) < 4:
            return None
        try:
            return [float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])]
        except (TypeError, ValueError):
            return None

    def _set_skeleton_row(
        self,
        row: int,
        source: str,
        node_name: str,
        node_id: str,
        parent_id: str,
        pos: Optional[list[float]],
        rot: Optional[list[float]] = None,
    ) -> None:
        if pos is None:
            values = [source, node_name, node_id, parent_id, "-", "-", "-", "-"]
        else:
            values = [
                source,
                node_name,
                node_id,
                parent_id,
                f"{pos[0]:.4f}",
                f"{pos[1]:.4f}",
                f"{pos[2]:.4f}",
                f"{self._norm_pos(pos):.4f}",
            ]
        if rot is None:
            values.extend(["-", "-", "-", "-", "-"])
        else:
            values.extend([
                f"{rot[0]:.4f}",
                f"{rot[1]:.4f}",
                f"{rot[2]:.4f}",
                f"{rot[3]:.4f}",
                f"{math.sqrt(sum(float(v) * float(v) for v in rot[:4])):.4f}",
            ])
        for col, text in enumerate(values):
            self._set_item(self.raw_skeleton_table, row, col, text)

    def _update_raw_skeleton(self, frame: ManusFrame) -> None:
        skeleton = getattr(frame, "skeleton", {}) or {}
        if not isinstance(skeleton, dict):
            skeleton = {}
        rows: list[tuple[str, str, str, str, Optional[list[float]], Optional[list[float]]]] = []
        hand = skeleton.get("hand") if isinstance(skeleton.get("hand"), dict) else None
        node_order = [("hand", hand)]
        for finger_name in ("thumb", "index", "middle", "ring", "pinky"):
            finger = skeleton.get(finger_name) if isinstance(skeleton.get(finger_name), dict) else {}
            if not isinstance(finger, dict):
                finger = {}
            for joint_name in ("metacarpal", "proximal", "intermediate", "distal", "tip"):
                node_order.append((f"{finger_name}.{joint_name}", finger.get(joint_name)))

        for name, node in node_order:
            node_id = "-"
            parent_id = "-"
            if isinstance(node, dict):
                node_id = str(node.get("node_id", "-"))
                parent_id = str(node.get("parent_id", "-"))
            rows.append(("raw skeleton", name, node_id, parent_id, self._node_pos(node), self._node_rot(node)))

        hand_pos = self._node_pos(hand)
        for finger_name in ("thumb", "index", "middle", "ring", "pinky"):
            finger = skeleton.get(finger_name) if isinstance(skeleton.get(finger_name), dict) else {}
            if not isinstance(finger, dict):
                finger = {}
            meta_pos = self._node_pos(finger.get("metacarpal"))
            tip_pos = self._node_pos(finger.get("tip"))
            if hand_pos is not None and tip_pos is not None:
                rows.append(("derived", f"hand -> {finger_name}.tip", "-", "-", self._sub_pos(tip_pos, hand_pos), None))
            else:
                rows.append(("derived", f"hand -> {finger_name}.tip", "-", "-", None, None))
            if meta_pos is not None and tip_pos is not None:
                rows.append(("derived", f"{finger_name}.metacarpal -> tip", "-", "-", self._sub_pos(tip_pos, meta_pos), None))
            else:
                rows.append(("derived", f"{finger_name}.metacarpal -> tip", "-", "-", None, None))

        self._set_table_rows(self.raw_skeleton_table, len(rows))
        for row, (source, name, node_id, parent_id, pos, rot) in enumerate(rows):
            self._set_skeleton_row(row, source, name, node_id, parent_id, pos, rot)

    def _start_joint_send(self) -> None:
        if self._send_enabled:
            return
        if self._retarget_mode() == "ergonomics" and not self.calibration.has_open_close():
            self._set_status("MANUS joint send blocked: record/load Open and Close first")
            return
        try:
            self.context.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.context.backend.set_pid_control(True)
        except Exception as exc:
            self._set_status(f"MANUS joint send failed to arm: {exc}")
            return

        self._send_enabled = True
        self._apply_source_safety()
        if self._retarget_mode() == "raw_pinky":
            self.calib_status_label.setText("MANUS joint send armed: full raw skeleton IK -> selected J00-J20")
        else:
            self.calib_status_label.setText("MANUS joint send armed: left glove -> left robot joints")
        self._set_status("MANUS joint send armed")

    def _stop_joint_send(self, status: str = "MANUS joint send stopped") -> None:
        self._send_enabled = False
        try:
            self.context.backend.invalidate_joint_commands()
        except Exception:
            pass
        self._apply_source_safety()
        self.calib_status_label.setText("Preview only until Send is pressed")
        self._set_status(status)

    def _maybe_send_joint_targets(self, frame: ManusFrame, preview) -> None:
        if not self._send_enabled:
            return
        if not self.receiver.is_alive():
            self._set_status("MANUS joint send paused: source stale")
            return
        if self._retarget_mode() == "ergonomics" and not self.calibration.has_open_close():
            self._stop_joint_send("MANUS joint send stopped: missing Open/Close calibration")
            return
        if frame.seq == self._last_sent_seq:
            return
        if self._retarget_mode() == "raw_pinky":
            raw_preview = self._last_raw_skeleton_preview or self.raw_skeleton_retargeter.preview(frame)
            mask = self._selected_joint_mask()
            invalid_selected = [
                item.joint_index
                for item in raw_preview
                if 0 <= item.joint_index < ENCODER_COUNT
                and (mask & (1 << item.joint_index)) != 0
                and not item.valid
            ]
            if invalid_selected:
                missing = ",".join(f"J{idx:02d}" for idx in invalid_selected[:6])
                self._set_status(f"Full raw skeleton send blocked: invalid {missing}")
                return
            angles = self.raw_skeleton_retargeter.target_angles(raw_preview)
            source_name = "MANUS full raw skeleton IK"
            source_space = URDF_DEG
            raw_source_targets = list(angles)
        else:
            legacy_angles = [0.0] * ENCODER_COUNT
            for item in preview:
                if 0 <= item.joint_index < len(legacy_angles):
                    legacy_angles[item.joint_index] = float(item.robot_q_target_preview)
            adaptation = adapt_joint_targets(
                legacy_angles,
                source_space=LEGACY_CALIBRATED_DEG,
            )
            angles = list(adaptation.urdf_targets_deg)
            source_name = "MANUS ergonomics joint send"
            source_space = LEGACY_CALIBRATED_DEG
            raw_source_targets = legacy_angles
        selected_targets, missing_feedback = self._targets_for_selected_joints(angles)
        if selected_targets is None:
            if missing_feedback:
                missing = ",".join(f"J{idx:02d}" for idx in missing_feedback[:4])
                self._set_status(f"MANUS joint send blocked: waiting feedback {missing}")
            else:
                self._set_status("MANUS joint send blocked: no teleop joint selected")
            return
        try:
            self.context.backend.set_control_mode(ControlMode.JOINT_ANGLE)
            self.context.backend.set_pid_control(True)
            result = self.context.send_joint_targets(
                selected_targets,
                source=source_name,
                live=True,
                input_space=source_space,
                raw_targets=raw_source_targets,
            )
            if result is False or not bool(getattr(result, "sent", True)):
                message = str(getattr(result, "message", "") or "")
                self._set_status(message or "MANUS joint send blocked by workspace guard")
                return
            self._last_sent_targets = list(selected_targets)
            self._last_sent_seq = frame.seq
            self._sent_count += 1
            self.send_count_label.setText(f"sent={self._sent_count}")
            self.last_sent_label.setText(f"last_sent_seq={frame.seq}")
        except Exception as exc:
            self._stop_joint_send(f"MANUS joint send failed: {exc}")

    def _record_calibration(self, pose: str) -> None:
        if self._recording_pose is not None:
            self._set_status(f"MANUS calibration busy: {self._recording_pose}")
            return
        if self.receiver.get_latest_frame() is None:
            self._set_status("MANUS calibration blocked: no frame")
            return
        self._recording_pose = pose
        self._set_calib_buttons_enabled(False)
        display_pose = self._calibration_pose_label(pose)
        self.calib_status_label.setText(f"Recording {display_pose} for 1.0s...")
        QtWidgets.QApplication.processEvents()
        QtCore.QTimer.singleShot(0, lambda p=pose: self._finish_record_calibration(p))

    def _finish_record_calibration(self, pose: str) -> None:
        display_pose = self._calibration_pose_label(pose)
        try:
            count = self.calibration.record_pose(pose, self.receiver, duration_s=1.0)
            self.calib_status_label.setText(f"Recorded {display_pose}: {count} calibration values")
            self._set_status(f"MANUS calibration saved {display_pose}: {count} values")
        except Exception as exc:
            self.calib_status_label.setText(f"Record {display_pose} failed: {exc}")
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
            self.record_align_btn,
            self.record_open_aa_btn,
            self.record_fist_flex_btn,
            self.record_thumb_pinky_root_btn,
            self.load_calib_btn,
        ):
            btn.setEnabled(bool(enabled))

    @staticmethod
    def _calibration_pose_label(pose: str) -> str:
        labels = {
            "align": "four-finger align",
            "open_aa": "open AA",
            "fist_flex": "fist flex",
            "thumb_pinky_root": "thumb to pinky root",
            "open": "legacy open",
        }
        return labels.get(str(pose), str(pose))

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
        self._stop_joint_send("MANUS joint send stopped: window closed")
        self.receiver.stop()
        super().closeEvent(event)


__all__ = ["ManusUdpView"]
