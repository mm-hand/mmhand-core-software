"""精简的遥操作CSV整段回放页面。"""

from __future__ import annotations

from pathlib import Path

from desktop.core.protocol import ENCODER_COUNT, SYSTEM_STATE_READY

from .ui_qt_compat import QtCore, QtWidgets


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

        preview_group = QtWidgets.QGroupBox("21关节轨迹范围预览")
        preview_layout = QtWidgets.QVBoxLayout(preview_group)
        self.preview_table = QtWidgets.QTableWidget(ENCODER_COUNT, 5)
        self.preview_table.setHorizontalHeaderLabels(
            ["Joint", "Start deg", "End deg", "Min deg", "Max deg"]
        )
        self.preview_table.verticalHeader().setVisible(False)
        self.preview_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.preview_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        header = self.preview_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        for column in range(1, 5):
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
            "导入遥操作轨迹",
            "",
            "遥操作CSV (*.csv);;所有文件 (*)",
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
            f"来源：{trace.source or '-'}    帧数：{trace.frame_count}    "
            f"时长：{trace.duration_s:.3f} s    平均频率：{trace.average_hz:.2f} Hz\n"
            f"开始时间：{trace.start_host_time or '-'}    最大帧间隔："
            f"{trace.max_gap_s:.3f} s    Active：{scope}\n"
            f"Mapping：{trace.mapping_id or '-'} v{trace.mapping_version}    "
            f"URDF CRC：0x{trace.urdf_crc:08X}    "
            f"过滤行数：{trace.dropped_row_count}/{trace.raw_row_count}"
        )
        self.warning_label.setText(
            "" if not trace.warnings else "注意：" + "；".join(trace.warnings)
        )
        for joint in range(ENCODER_COUNT):
            start, end, low, high = trace.joint_statistics(joint)
            for column, value in enumerate((start, end, low, high), start=1):
                self.preview_table.setItem(
                    joint, column, QtWidgets.QTableWidgetItem(f"{value:.3f}")
                )

    def _refresh_status(self) -> None:
        state = self.context.get_state()
        ready = (
            state is not None
            and int(getattr(state, "system_state", -1)) == SYSTEM_STATE_READY
        )
        self.ready_status.setText("P4: READY" if ready else "P4: NOT READY")
        self.ready_status.setStyleSheet(
            "font-weight:700;color:#1b7f3a;" if ready else "font-weight:700;color:#b71c1c;"
        )

        status = self.service.status()
        running = self.service.is_running()
        self.replay_status.setText(status.message)
        if status.duration_s > 0.0:
            ratio = max(0.0, min(1.0, status.elapsed_s / status.duration_s))
        else:
            ratio = 0.0
        self.progress.setValue(int(round(ratio * 1000.0)))
        self.progress.setFormat(f"{ratio * 100.0:.1f}%")

        self.import_btn.setEnabled(not running)
        self.send_btn.setText("■ Stop" if running else "▶ Send")
        self.send_btn.setStyleSheet(
            "font-weight:700;color:#b71c1c;" if running else "font-weight:700;"
        )
        self.send_btn.setEnabled(running or (self.service.trace() is not None and ready))
