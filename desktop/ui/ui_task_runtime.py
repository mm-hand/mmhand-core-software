"""ui_task_runtime.py - MMHand Desktop 模块契约。

职责：展示 P4 任务周期、执行时间、栈和 CPU 诊断。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
import csv
import os
import time
from datetime import datetime
from typing import Dict, Optional, Tuple

from desktop.core.protocol import TASK_RUNTIME_NAMES, TASK_RUNTIME_STATE_MACHINE, TASK_RUNTIME_CONTROL
from desktop.paths import LOG_DIR
from .ui_qt_compat import QtCore, QtGui, QtWidgets


TASK_RUNTIME_LOG_DIR = os.path.join(str(LOG_DIR), "task_runtime")
TASK_RUNTIME_LOG_KEEP = 20
UNKNOWN_PERMILLE = 0xFFFF


class TaskRuntimeWindow(QtWidgets.QDialog):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("Task Runtime")
        self.resize(1180, 430)
        self._last_logged_keys: Dict[int, Tuple[int, int]] = {}
        self._csv_file = None
        self._csv_writer = None
        self._open_csv_log()
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        summary = QtWidgets.QHBoxLayout()
        summary.setSpacing(8)
        self.core0_label = self._make_summary_label("Core0 idle -")
        self.core1_label = self._make_summary_label("Core1 idle -")
        self.sys_gap_label = self._make_summary_label("SYS gap -")
        self.stop_label = self._make_summary_label("STOP -")
        self.rx_label = self._make_summary_label("RX -")
        for label in [
            self.core0_label,
            self.core1_label,
            self.sys_gap_label,
            self.stop_label,
            self.rx_label,
        ]:
            summary.addWidget(label)
        summary.addStretch(1)
        layout.addLayout(summary)

        self.table = QtWidgets.QTableWidget(0, 13, self)
        self.table.setHorizontalHeaderLabels([
            "Task",
            "Active",
            "Prio",
            "Core",
            "Max Gap ms",
            "Max Exec ms",
            "Overruns",
            "Consec",
            "Max Consec",
            "Stack Current",
            "Stack Min",
            "Min %",
            "Loops",
        ])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.table, 1)

        hint = QtWidgets.QLabel("Thresholds: StateMachine max gap <20 ms, SYSTEM_STATUS max gap <300 ms, STOP unload <30 ms, core idle >=15%, historical min stack free >=25%.")
        hint.setStyleSheet("color:#57606a;")
        layout.addWidget(hint)

    @staticmethod
    def _make_summary_label(text: str) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setMinimumHeight(26)
        label.setMinimumWidth(130)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet(
            "QLabel {"
            "color:#24292f;"
            "background:#f6f8fa;"
            "border:1px solid #d0d7de;"
            "border-radius:3px;"
            "padding:3px 8px;"
            "font-weight:600;"
            "}"
        )
        return label

    @staticmethod
    def _style_label(label: QtWidgets.QLabel, state: str) -> None:
        palette = {
            "ok": ("#1b5e20", "#e8f5e9", "#81c784"),
            "warn": ("#8a5a00", "#fff8e1", "#ffcc80"),
            "bad": ("#c62828", "#ffebee", "#ef9a9a"),
            "neutral": ("#24292f", "#f6f8fa", "#d0d7de"),
        }
        fg, bg, border = palette.get(state, palette["neutral"])
        label.setStyleSheet(
            f"color:{fg};"
            f"background:{bg};"
            f"border:1px solid {border};"
            "border-radius:3px;"
            "padding:3px 8px;"
            "font-weight:700;"
        )

    def update_state(self, state) -> None:
        task_runtime = dict(getattr(state, "task_runtime", {}) or {})
        if not task_runtime:
            self.rx_label.setText("RX no task diag")
            self._style_label(self.rx_label, "neutral")
            return

        latest = max(task_runtime.values(), key=lambda item: float(item.get("host_rx_ts", 0.0) or 0.0))
        self._update_summary(latest, getattr(state, "task_runtime_last_rx_ts", 0.0))
        rows = sorted(task_runtime.items(), key=lambda item: int(item[0]))
        self.table.setRowCount(len(rows))
        for row, (task_id, diag) in enumerate(rows):
            self._populate_row(row, int(task_id), diag)
            self._log_diag(int(task_id), diag)

    def _update_summary(self, latest: dict, last_rx_ts: float) -> None:
        core0 = int(latest.get("core0_idle_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE)
        core1 = int(latest.get("core1_idle_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE)
        sys_gap = int(latest.get("system_status_max_gap_ms", 0) or 0)
        stop_last = int(latest.get("stop_last_latency_us", 0) or 0)
        stop_max = int(latest.get("stop_max_latency_us", 0) or 0)

        self.core0_label.setText(f"Core0 idle {self._format_permille(core0)}")
        self.core1_label.setText(f"Core1 idle {self._format_permille(core1)}")
        self.sys_gap_label.setText(f"SYS gap {sys_gap} ms")
        self.stop_label.setText(f"STOP {stop_last / 1000.0:.1f}/{stop_max / 1000.0:.1f} ms")

        now = time.time()
        age_ms = max(0.0, (now - float(last_rx_ts or 0.0)) * 1000.0) if last_rx_ts else 0.0
        self.rx_label.setText(f"RX {age_ms:.0f} ms ago")

        self._style_label(self.core0_label, self._idle_state(core0))
        self._style_label(self.core1_label, self._idle_state(core1))
        self._style_label(self.sys_gap_label, "bad" if sys_gap > 300 else "ok")
        self._style_label(self.stop_label, "bad" if stop_max > 30000 else "ok")
        self._style_label(self.rx_label, "bad" if age_ms > 1000.0 else "ok")

    @staticmethod
    def _format_permille(value: int) -> str:
        if value == UNKNOWN_PERMILLE:
            return "-"
        return f"{value / 10.0:.1f}%"

    @staticmethod
    def _idle_state(value: int) -> str:
        if value == UNKNOWN_PERMILLE:
            return "neutral"
        if value < 150:
            return "bad"
        if value < 250:
            return "warn"
        return "ok"

    def _populate_row(self, row: int, task_id: int, diag: dict) -> None:
        alloc = int(diag.get("stack_allocated_bytes", 0) or 0)
        current_free = int(diag.get("stack_current_free_bytes", diag.get("stack_min_free_bytes", 0)) or 0)
        min_free = int(diag.get("stack_min_free_bytes", 0) or 0)
        stack_pct = (min_free * 100.0 / alloc) if alloc > 0 else 0.0
        max_gap_ms = int(diag.get("max_gap_us", 0) or 0) / 1000.0
        max_exec_ms = int(diag.get("max_exec_us", 0) or 0) / 1000.0
        values = [
            str(diag.get("task_name") or TASK_RUNTIME_NAMES.get(task_id, f"Task{task_id}")),
            "Y" if bool(diag.get("active", False)) else "N",
            str(int(diag.get("priority", 0) or 0)),
            str(int(diag.get("core_id", -1) or -1)),
            f"{max_gap_ms:.2f}",
            f"{max_exec_ms:.2f}",
            str(int(diag.get("overrun_count", 0) or 0)),
            str(int(diag.get("consecutive_overruns", 0) or 0)),
            str(int(diag.get("max_consecutive_overruns", 0) or 0)),
            f"{current_free}/{alloc}" if current_free > 0 else "-",
            f"{min_free}/{alloc}",
            f"{stack_pct:.1f}%",
            str(int(diag.get("loop_count", 0) or 0)),
        ]
        row_state = self._row_state(task_id, diag, stack_pct, max_gap_ms)
        for col, text in enumerate(values):
            item = QtWidgets.QTableWidgetItem(text)
            item.setTextAlignment(QtCore.Qt.AlignCenter if col > 0 else QtCore.Qt.AlignVCenter)
            if row_state == "bad":
                item.setBackground(QtGui.QColor("#ffebee"))
                item.setForeground(QtGui.QColor("#c62828"))
            elif row_state == "warn":
                item.setBackground(QtGui.QColor("#fff8e1"))
                item.setForeground(QtGui.QColor("#8a5a00"))
            self.table.setItem(row, col, item)

    @staticmethod
    def _row_state(task_id: int, diag: dict, stack_pct: float, max_gap_ms: float) -> str:
        if stack_pct > 0.0 and stack_pct < 25.0:
            return "bad"
        if task_id == TASK_RUNTIME_STATE_MACHINE and max_gap_ms > 20.0:
            return "bad"
        if task_id == TASK_RUNTIME_CONTROL and int(diag.get("consecutive_overruns", 0) or 0) > 1:
            return "bad"
        if int(diag.get("system_status_max_gap_ms", 0) or 0) > 300:
            return "bad"
        if int(diag.get("stop_max_latency_us", 0) or 0) > 30000:
            return "bad"
        if stack_pct > 0.0 and stack_pct < 40.0:
            return "warn"
        return "ok"

    def _open_csv_log(self) -> None:
        os.makedirs(TASK_RUNTIME_LOG_DIR, exist_ok=True)
        self._prune_old_logs()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(TASK_RUNTIME_LOG_DIR, f"task_runtime_{stamp}.csv")
        self._csv_file = open(path, "w", newline="", encoding="utf-8")
        fields = [
            "host_ts",
            "task_id",
            "task_name",
            "active",
            "priority",
            "core_id",
            "stack_allocated_bytes",
            "stack_current_free_bytes",
            "stack_min_free_bytes",
            "stack_min_free_pct",
            "loop_count",
            "max_gap_us",
            "max_exec_us",
            "overrun_count",
            "consecutive_overruns",
            "max_consecutive_overruns",
            "core0_idle_permille",
            "core1_idle_permille",
            "system_status_max_gap_ms",
            "stop_last_latency_us",
            "stop_max_latency_us",
            "device_timestamp_ms",
        ]
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=fields)
        self._csv_writer.writeheader()

    @staticmethod
    def _prune_old_logs() -> None:
        if not os.path.isdir(TASK_RUNTIME_LOG_DIR):
            return
        paths = [
            os.path.join(TASK_RUNTIME_LOG_DIR, name)
            for name in os.listdir(TASK_RUNTIME_LOG_DIR)
            if name.lower().endswith(".csv")
        ]
        paths.sort(key=lambda path: os.path.getmtime(path), reverse=True)
        for path in paths[TASK_RUNTIME_LOG_KEEP - 1:]:
            try:
                os.remove(path)
            except OSError:
                pass

    def _log_diag(self, task_id: int, diag: dict) -> None:
        if self._csv_writer is None:
            return
        key = (
            int(diag.get("device_timestamp_ms", 0) or 0),
            int(diag.get("loop_count", 0) or 0),
        )
        if self._last_logged_keys.get(task_id) == key:
            return
        self._last_logged_keys[task_id] = key
        alloc = int(diag.get("stack_allocated_bytes", 0) or 0)
        current_free = int(diag.get("stack_current_free_bytes", diag.get("stack_min_free_bytes", 0)) or 0)
        min_free = int(diag.get("stack_min_free_bytes", 0) or 0)
        stack_pct = (min_free * 100.0 / alloc) if alloc > 0 else 0.0
        row = {
            "host_ts": f"{time.time():.3f}",
            "task_id": task_id,
            "task_name": diag.get("task_name") or TASK_RUNTIME_NAMES.get(task_id, f"Task{task_id}"),
            "active": int(bool(diag.get("active", False))),
            "priority": int(diag.get("priority", 0) or 0),
            "core_id": int(diag.get("core_id", -1) or -1),
            "stack_allocated_bytes": alloc,
            "stack_current_free_bytes": current_free,
            "stack_min_free_bytes": min_free,
            "stack_min_free_pct": f"{stack_pct:.1f}",
            "loop_count": int(diag.get("loop_count", 0) or 0),
            "max_gap_us": int(diag.get("max_gap_us", 0) or 0),
            "max_exec_us": int(diag.get("max_exec_us", 0) or 0),
            "overrun_count": int(diag.get("overrun_count", 0) or 0),
            "consecutive_overruns": int(diag.get("consecutive_overruns", 0) or 0),
            "max_consecutive_overruns": int(diag.get("max_consecutive_overruns", 0) or 0),
            "core0_idle_permille": int(diag.get("core0_idle_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE),
            "core1_idle_permille": int(diag.get("core1_idle_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE),
            "system_status_max_gap_ms": int(diag.get("system_status_max_gap_ms", 0) or 0),
            "stop_last_latency_us": int(diag.get("stop_last_latency_us", 0) or 0),
            "stop_max_latency_us": int(diag.get("stop_max_latency_us", 0) or 0),
            "device_timestamp_ms": int(diag.get("device_timestamp_ms", 0) or 0),
        }
        self._csv_writer.writerow(row)
        self._csv_file.flush()

    def closeEvent(self, event) -> None:
        try:
            if self._csv_file is not None:
                self._csv_file.close()
                self._csv_file = None
                self._csv_writer = None
                self._prune_old_logs()
        finally:
            super().closeEvent(event)
