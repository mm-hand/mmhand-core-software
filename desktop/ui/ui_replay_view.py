"""精简的遥操作CSV整段回放页面。"""

from __future__ import annotations

from pathlib import Path

from desktop.algorithm.replay_controller import replay_start_state_allowed
from desktop.core.protocol import (
    ENCODER_COUNT,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_FEEDBACK,
)

from .ui_qt_compat import QtCore, QtGui, QtWidgets


class ReplayView(QtWidgets.QWidget):
    """页面只保留“导入CSV”和“Send/Stop”两个操作按钮。"""

    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.service = context.replay_service
        self._build()
        self._refresh_trace()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        file_group = QtWidgets.QGroupBox("遥操作轨迹 CSV")
        file_layout = QtWidgets.QHBoxLayout(file_group)
        self.path_edit = QtWidgets.QLineEdit()
        self.path_edit.setReadOnly(True)
        self.path_edit.setPlaceholderText(
            "请选择 Save Data 生成的 07_遥操作总表.csv"
        )
        self.import_btn = QtWidgets.QPushButton("导入 CSV")
        self.import_btn.clicked.connect(self._import_csv)
        file_layout.addWidget(self.path_edit, 1)
        file_layout.addWidget(self.import_btn)
        layout.addWidget(file_group)

        summary_group = QtWidgets.QGroupBox("轨迹信息")
        summary_layout = QtWidgets.QVBoxLayout(summary_group)
        self.summary_label = QtWidgets.QLabel("尚未导入CSV")
        self.summary_label.setWordWrap(True)
        self.warning_label = QtWidgets.QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet("color:#9a5b00;")
        summary_layout.addWidget(self.summary_label)
        summary_layout.addWidget(self.warning_label)
        layout.addWidget(summary_group)

        preview_group = QtWidgets.QGroupBox("21关节实时角度")
        preview_layout = QtWidgets.QVBoxLayout(preview_group)
        self.preview_table = QtWidgets.QTableWidget(ENCODER_COUNT, 5)
        self.preview_table.setHorizontalHeaderLabels(
            ["Joint", "Current deg", "Valid", "Replay Target", "Error"]
        )
        self.preview_table.verticalHeader().setVisible(False)
        self.preview_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.preview_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        header = self.preview_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        for column in (1, 3, 4):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Stretch)
        for joint in range(ENCODER_COUNT):
            self.preview_table.setItem(
                joint, 0, QtWidgets.QTableWidgetItem(f"J{joint:02d}")
            )
            for column in range(1, 5):
                self.preview_table.setItem(
                    joint, column, QtWidgets.QTableWidgetItem("-")
                )
        preview_layout.addWidget(self.preview_table)
        layout.addWidget(preview_group, 1)

        status_layout = QtWidgets.QHBoxLayout()
        self.ready_status = QtWidgets.QLabel("P4: -")
        self.replay_status = QtWidgets.QLabel("Replay空闲")
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        self.progress.setFormat("0.0%")
        status_layout.addWidget(self.ready_status)
        status_layout.addWidget(self.replay_status, 1)
        status_layout.addWidget(self.progress)
        layout.addLayout(status_layout)

        self.send_btn = QtWidgets.QPushButton("▶ Send")
        self.send_btn.setMinimumHeight(42)
        self.send_btn.clicked.connect(self._send_or_stop)
        layout.addWidget(self.send_btn)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start(100)

    def _import_csv(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "导入动作轨迹",
            "",
            "动作或关节状态 CSV (*.csv);;所有文件 (*)",
        )
        if not path:
            return
        try:
            self.service.load_csv(Path(path))
            self._refresh_trace()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "导入CSV失败", str(exc))

    def _send_or_stop(self) -> None:
        if self.service.is_running():
            self.service.stop("Replay UI Stop")
            return
        ok, message = self.service.send()
        if not ok:
            QtWidgets.QMessageBox.warning(self, "Replay无法启动", message)

    def _refresh_trace(self) -> None:
        trace = self.service.trace()
        if trace is None:
            self.path_edit.clear()
            self.summary_label.setText("尚未导入CSV")
            self.warning_label.clear()
            return
        self.path_edit.setText(str(trace.path))
        self.path_edit.setToolTip(str(trace.path))
        scope = (
            f"0x{trace.active_joint_mask:08X}"
            if trace.active_joint_mask
            else "使用P4当前active scope"
        )
        self.summary_label.setText(
            f"数据：{'实际关节角' if trace.data_kind == 'joint_actual' else '动作指令'}    "
            f"来源：{trace.source or '-'}    记录帧数：{trace.frame_count}    "
            f"时长：{trace.duration_s:.3f} s    平均频率：{trace.average_hz:.2f} Hz\n"
            f"开始时间：{trace.start_host_time or '-'}    最大帧间隔："
            f"{trace.max_gap_s:.3f} s    Active：{scope}\n"
            f"Mapping：{trace.mapping_id or '-'} v{trace.mapping_version}    "
            f"URDF CRC：0x{trace.urdf_crc:08X}    "
            f"Order CRC：0x{trace.joint_order_crc:08X}    "
            f"过滤行数：{trace.dropped_row_count}/{trace.raw_row_count}"
        )
        self.warning_label.setText(
            "" if not trace.warnings else "注意：" + "；".join(trace.warnings)
        )

    def _refresh_status(self) -> None:
        state = self.context.get_state()
        system_state = (
            int(getattr(state, "system_state", -1)) if state is not None else -1
        )
        start_allowed = replay_start_state_allowed(system_state)
        if system_state == SYSTEM_STATE_READY:
            state_text = "READY"
        elif system_state == SYSTEM_STATE_RUN_FEEDBACK:
            state_text = "RUN_FEEDBACK"
        else:
            state_text = "NOT READY"
        self.ready_status.setText(f"P4: {state_text}")
        self.ready_status.setStyleSheet(
            "font-weight:700;color:#1b7f3a;"
            if start_allowed
            else "font-weight:700;color:#b71c1c;"
        )

        status = self.service.status()
        running = self.service.is_running()
        runtime_detail = ""
        if status.effective_joint_mask:
            runtime_detail = (
                f" | 源{self._mask_count(status.source_joint_mask)}关节"
                f" → 实际{self._mask_count(status.effective_joint_mask)}关节"
            )
            if status.skipped_joint_mask:
                runtime_detail += (
                    f" | 跳过 {self._format_joint_mask(status.skipped_joint_mask)}"
                )
            if status.clamped_sample_count:
                runtime_detail += (
                    f" | 钳位{status.clamped_sample_count}个关键帧"
                )
            if status.timeline_rebase_count:
                runtime_detail += (
                    f" | 时间轴重定位{status.timeline_rebase_count}次/"
                    f"{status.timeline_shift_s * 1000.0:.0f}ms"
                )
        self.replay_status.setText(status.message + runtime_detail)
        trace = self.service.trace()
        warnings = list(getattr(trace, "warnings", []) or []) if trace is not None else []
        warnings.extend(list(status.warnings or ()))
        self.warning_label.setText(
            "" if not warnings else "注意：" + "；".join(dict.fromkeys(warnings))
        )
        if status.duration_s > 0.0:
            ratio = max(0.0, min(1.0, status.elapsed_s / status.duration_s))
        elif status.total_frames > 0:
            ratio = max(
                0.0,
                min(1.0, float(status.frame_index) / float(status.total_frames)),
            )
        else:
            ratio = 0.0
        self.progress.setValue(int(round(ratio * 1000.0)))
        self.progress.setFormat(f"{ratio * 100.0:.1f}%")

        self.import_btn.setEnabled(not running)
        self.send_btn.setText("■ Stop" if running else "▶ Send")
        self.send_btn.setStyleSheet(
            "font-weight:700;color:#b71c1c;" if running else "font-weight:700;"
        )
        self.send_btn.setEnabled(
            running or (self.service.trace() is not None and start_allowed)
        )
        self._refresh_joint_feedback(state, status)

    def _refresh_joint_feedback(self, state, replay_status) -> None:
        """复用 Joint Channels 的有效性判断，刷新真实反馈和当前回放目标。"""
        panel = self.context.control_panel
        angles = list(getattr(state, "angles", []) or []) if state is not None else []
        targets = tuple(getattr(replay_status, "current_targets", ()) or ())
        target_mask = int(getattr(replay_status, "current_joint_mask", 0) or 0)

        self.preview_table.setUpdatesEnabled(False)
        try:
            for joint in range(ENCODER_COUNT):
                encoder_status = panel.joint_encoder_status(joint, state)
                angle_valid = (
                    encoder_status in ("ok", "warn")
                    and panel.joint_angle_feedback_valid(joint, state)
                    and panel.joint_angle_display_calibrated(joint, state)
                    and joint < len(angles)
                )
                current = float(angles[joint]) if angle_valid else None
                current_item = self.preview_table.item(joint, 1)
                valid_item = self.preview_table.item(joint, 2)
                target_item = self.preview_table.item(joint, 3)
                error_item = self.preview_table.item(joint, 4)

                if current is None:
                    current_item.setText("-")
                    current_item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                    valid_item.setText("-")
                    valid_item.setForeground(QtGui.QBrush(QtGui.QColor("#9aa0a6")))
                else:
                    current_item.setText(f"{current:.3f}")
                    out_of_workspace = bool(
                        joint < len(getattr(state, "encoders", []) or [])
                        and getattr(
                            state.encoders[joint],
                            "angle_out_of_workspace",
                            False,
                        )
                    )
                    current_item.setForeground(
                        QtGui.QBrush(
                            QtGui.QColor("#d89b00" if out_of_workspace else "#000000")
                        )
                    )
                    valid_item.setText("WARN" if encoder_status == "warn" else "OK")
                    valid_item.setForeground(
                        QtGui.QBrush(
                            QtGui.QColor(
                                "#d89b00" if encoder_status == "warn" else "#1b7f3a"
                            )
                        )
                    )

                target_valid = bool(
                    target_mask & (1 << joint) and joint < len(targets)
                )
                if target_valid:
                    target = float(targets[joint])
                    target_item.setText(f"{target:.3f}")
                    target_item.setForeground(
                        QtGui.QBrush(
                            QtGui.QColor(
                                "#d89b00"
                                if replay_status.clamped_joint_mask & (1 << joint)
                                else "#000000"
                            )
                        )
                    )
                    error_item.setText(
                        f"{target - current:.3f}" if current is not None else "-"
                    )
                elif replay_status.skipped_joint_mask & (1 << joint):
                    target_item.setText("SKIP")
                    target_item.setForeground(
                        QtGui.QBrush(QtGui.QColor("#d89b00"))
                    )
                    error_item.setText("-")
                else:
                    target_item.setText("-")
                    target_item.setForeground(
                        QtGui.QBrush(QtGui.QColor("#9aa0a6"))
                    )
                    error_item.setText("-")
        finally:
            self.preview_table.setUpdatesEnabled(True)

    @staticmethod
    def _mask_count(mask: int) -> int:
        return int(mask).bit_count()

    @staticmethod
    def _format_joint_mask(mask: int) -> str:
        labels = [
            f"J{joint:02d}"
            for joint in range(ENCODER_COUNT)
            if int(mask) & (1 << joint)
        ]
        return ",".join(labels) if labels else "-"
