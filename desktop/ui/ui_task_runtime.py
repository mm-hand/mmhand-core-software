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

from desktop.core.protocol import (
    TASK_RUNTIME_NAMES,
    TASK_RUNTIME_STATE_MACHINE,
    TASK_RUNTIME_CONTROL,
    TASK_RUNTIME_SERVO_COMM,
)
from desktop.paths import LOG_DIR
from .ui_qt_compat import QtCore, QtGui, QtWidgets


TASK_RUNTIME_LOG_DIR = os.path.join(str(LOG_DIR), "task_runtime")
TASK_RUNTIME_LOG_KEEP = 20
UNKNOWN_PERMILLE = 0xFFFF
TASK_RUNTIME_RX_WARN_MS = 2500.0
TASK_RUNTIME_RX_TIMEOUT_MS = 5000.0
TASK_RUNTIME_ASSEMBLY_GRACE_MS = 250.0


class TaskRuntimeWindow(QtWidgets.QDialog):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("Task Runtime")
        # 关闭即销毁，确保 UI 能在 destroyed 回调中及时退订 P4 诊断流。
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.resize(1500, 460)
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
        self.sys_gap_label = self._make_summary_label("System status gap -")
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

        self.table = QtWidgets.QTableWidget(0, 19, self)
        self.table.setHorizontalHeaderLabels([
            "Task",
            "Active",
            "Prio",
            "Core",
            "CPU %",
            "Avg ms",
            "P95 ms",
            "P99 ms",
            "1s Max ms",
            "Hist Max ms",
            "Max Gap ms",
            "Overruns",
            "Consec",
            "Stack Current",
            "Stack Min",
            "Min %",
            "Loops/s",
            "Periodic/s",
            "Events/s",
        ])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.table, 1)

        hint = QtWidgets.QLabel("Window: 1 s; CPU is the task share of total runtime; P95/P99 are execution-time percentiles.")
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
            self._update_rx_health(state, 0.0)
            return

        latest = max(task_runtime.values(), key=lambda item: float(item.get("host_rx_ts", 0.0) or 0.0))
        self._update_summary(latest, state)
        rows = sorted(task_runtime.items(), key=lambda item: int(item[0]))
        self.table.setRowCount(len(rows))
        for row, (task_id, diag) in enumerate(rows):
            self._populate_row(row, int(task_id), diag)
            self._log_diag(int(task_id), diag)

    def _update_summary(self, latest: dict, state) -> None:
        core0 = int(latest.get("core0_idle_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE)
        core1 = int(latest.get("core1_idle_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE)
        sys_gap = int(latest.get("system_status_max_gap_ms", 0) or 0)
        stop_last = int(latest.get("stop_last_latency_us", 0) or 0)
        stop_max = int(latest.get("stop_max_latency_us", 0) or 0)

        self.core0_label.setText(f"Core0 idle {self._format_permille(core0)}")
        self.core1_label.setText(f"Core1 idle {self._format_permille(core1)}")
        self.sys_gap_label.setText(f"System status gap {sys_gap} ms")
        self.stop_label.setText(f"STOP {stop_last / 1000.0:.1f}/{stop_max / 1000.0:.1f} ms")

        self._style_label(self.core0_label, self._idle_state(core0))
        self._style_label(self.core1_label, self._idle_state(core1))
        self._style_label(self.sys_gap_label, "bad" if sys_gap > 300 else "ok")
        self._style_label(self.stop_label, "bad" if stop_max > 30000 else "ok")
        self._update_rx_health(state, getattr(state, "task_runtime_last_rx_ts", 0.0))

    def _update_rx_health(self, state, last_complete_rx_ts: float) -> None:
        """区分完整窗口超时和七包接收不完整，避免把一秒正常周期误报为掉线。"""
        expected_mask = int(getattr(state, "task_runtime_expected_mask", 0) or 0)
        received_mask = int(getattr(state, "task_runtime_received_mask", 0) or 0)
        dropped = int(getattr(state, "task_runtime_dropped_windows", 0) or 0)
        if expected_mask and received_mask != expected_mask:
            last_packet_rx_ts = float(
                getattr(state, "task_runtime_last_packet_rx_ts", 0.0) or 0.0
            )
            partial_age_ms = (
                max(0.0, (time.time() - last_packet_rx_ts) * 1000.0)
                if last_packet_rx_ts
                else 0.0
            )
            missing = [
                TASK_RUNTIME_NAMES.get(task_id, str(task_id))
                for task_id in TASK_RUNTIME_NAMES
                if (received_mask & (1 << task_id)) == 0
            ]
            assembling = partial_age_ms <= TASK_RUNTIME_ASSEMBLY_GRACE_MS
            prefix = "RX assembling" if assembling else "RX partial"
            self.rx_label.setText(
                f"{prefix} {received_mask.bit_count()}/{expected_mask.bit_count()}"
            )
            self.rx_label.setToolTip(f"Missing: {', '.join(missing)}; dropped windows: {dropped}")
            self._style_label(self.rx_label, "neutral" if assembling else "warn")
            return

        now = time.time()
        age_ms = (
            max(0.0, (now - float(last_complete_rx_ts)) * 1000.0)
            if last_complete_rx_ts
            else 0.0
        )
        if not last_complete_rx_ts:
            self.rx_label.setText("RX no task diag")
            self.rx_label.setToolTip(f"Dropped windows: {dropped}")
            self._style_label(self.rx_label, "neutral")
            return
        self.rx_label.setText(f"RX {age_ms:.0f} ms ago")
        self.rx_label.setToolTip(f"Dropped windows: {dropped}")
        state_name = "bad" if age_ms > TASK_RUNTIME_RX_TIMEOUT_MS else (
            "warn" if age_ms > TASK_RUNTIME_RX_WARN_MS else "ok"
        )
        self._style_label(self.rx_label, state_name)

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
        avg_exec_ms = int(diag.get("avg_exec_us", 0) or 0) / 1000.0
        p95_exec_ms = int(diag.get("p95_exec_us", 0) or 0) / 1000.0
        p99_exec_ms = int(diag.get("p99_exec_us", 0) or 0) / 1000.0
        window_max_exec_ms = int(diag.get("window_max_exec_us", 0) or 0) / 1000.0
        max_exec_ms = int(diag.get("max_exec_us", 0) or 0) / 1000.0
        window_duration_us = int(diag.get("window_duration_us", 0) or 0)
        window_loop_count = int(diag.get("window_loop_count", 0) or 0)
        window_periodic_count = int(diag.get("window_periodic_count", 0) or 0)
        window_event_count = int(diag.get("window_event_count", 0) or 0)
        loops_per_s = window_loop_count * 1000000.0 / window_duration_us if window_duration_us > 0 else 0.0
        periodic_per_s = window_periodic_count * 1000000.0 / window_duration_us if window_duration_us > 0 else 0.0
        events_per_s = window_event_count * 1000000.0 / window_duration_us if window_duration_us > 0 else 0.0
        values = [
            str(diag.get("task_name") or TASK_RUNTIME_NAMES.get(task_id, f"Task{task_id}")),
            "Y" if bool(diag.get("active", False)) else "N",
            str(int(diag.get("priority", 0) or 0)),
            str(int(diag.get("core_id", -1) or -1)),
            self._format_permille(int(diag.get("cpu_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE)),
            f"{avg_exec_ms:.2f}",
            f"{p95_exec_ms:.2f}",
            f"{p99_exec_ms:.2f}",
            f"{window_max_exec_ms:.2f}",
            f"{max_exec_ms:.2f}",
            f"{max_gap_ms:.2f}",
            str(int(diag.get("overrun_count", 0) or 0)),
            str(int(diag.get("consecutive_overruns", 0) or 0)),
            f"{current_free}/{alloc}" if current_free > 0 else "-",
            f"{min_free}/{alloc}",
            f"{stack_pct:.1f}%",
            f"{loops_per_s:.1f}",
            f"{periodic_per_s:.1f}",
            f"{events_per_s:.1f}",
        ]
        row_state = self._row_state(task_id, diag, stack_pct, max_gap_ms)
        bus_tooltip = self._servo_bus_timing_tooltip(diag) if task_id == TASK_RUNTIME_SERVO_COMM else ""
        for col, text in enumerate(values):
            item = QtWidgets.QTableWidgetItem(text)
            if bus_tooltip:
                item.setToolTip(bus_tooltip)
            item.setTextAlignment(QtCore.Qt.AlignCenter if col > 0 else QtCore.Qt.AlignVCenter)
            if row_state == "bad":
                item.setBackground(QtGui.QColor("#ffebee"))
                item.setForeground(QtGui.QColor("#c62828"))
            elif row_state == "warn":
                item.setBackground(QtGui.QColor("#fff8e1"))
                item.setForeground(QtGui.QColor("#8a5a00"))
            self.table.setItem(row, col, item)

    @staticmethod
    def _servo_bus_timing_tooltip(diag: dict) -> str:
        """展示四条 BUS 的一秒同步读统计，不再为表格增加二十四个常驻列。"""
        samples = list(diag.get("io_sample_count", []) or [])
        requested = list(diag.get("io_requested_count", []) or [])
        succeeded = list(diag.get("io_succeeded_count", []) or [])
        avg_us = list(diag.get("io_avg_us", []) or [])
        p95_us = list(diag.get("io_p95_us", []) or [])
        p99_us = list(diag.get("io_p99_us", []) or [])
        lines = []
        for bus in range(4):
            def value(items):
                return int(items[bus]) if bus < len(items) else 0
            lines.append(
                f"BUS{bus}: calls={value(samples)}, feedback={value(succeeded)}/{value(requested)}, "
                f"avg={value(avg_us) / 1000.0:.2f} ms, "
                f"P95={value(p95_us) / 1000.0:.2f} ms, "
                f"P99={value(p99_us) / 1000.0:.2f} ms"
            )
        return "\n".join(lines)

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
            "window_sequence",
            "window_duration_us",
            "window_loop_count",
            "loops_per_s",
            "window_periodic_count",
            "periodic_per_s",
            "window_event_count",
            "events_per_s",
            "cpu_permille",
            "avg_exec_us",
            "p95_exec_us",
            "p99_exec_us",
            "window_max_exec_us",
            "historical_loop_count",
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
        for bus in range(4):
            fields.extend([
                f"bus{bus}_read_count",
                f"bus{bus}_requested",
                f"bus{bus}_succeeded",
                f"bus{bus}_avg_us",
                f"bus{bus}_p95_us",
                f"bus{bus}_p99_us",
            ])
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
            int(diag.get("window_sequence", 0) or 0),
            int(diag.get("device_timestamp_ms", 0) or 0),
        )
        if self._last_logged_keys.get(task_id) == key:
            return
        self._last_logged_keys[task_id] = key
        alloc = int(diag.get("stack_allocated_bytes", 0) or 0)
        current_free = int(diag.get("stack_current_free_bytes", diag.get("stack_min_free_bytes", 0)) or 0)
        min_free = int(diag.get("stack_min_free_bytes", 0) or 0)
        stack_pct = (min_free * 100.0 / alloc) if alloc > 0 else 0.0
        window_duration_us = int(diag.get("window_duration_us", 0) or 0)
        window_loop_count = int(diag.get("window_loop_count", 0) or 0)
        window_periodic_count = int(diag.get("window_periodic_count", 0) or 0)
        window_event_count = int(diag.get("window_event_count", 0) or 0)
        loops_per_s = window_loop_count * 1000000.0 / window_duration_us if window_duration_us > 0 else 0.0
        periodic_per_s = window_periodic_count * 1000000.0 / window_duration_us if window_duration_us > 0 else 0.0
        events_per_s = window_event_count * 1000000.0 / window_duration_us if window_duration_us > 0 else 0.0
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
            "window_sequence": int(diag.get("window_sequence", 0) or 0),
            "window_duration_us": window_duration_us,
            "window_loop_count": window_loop_count,
            "loops_per_s": f"{loops_per_s:.3f}",
            "window_periodic_count": window_periodic_count,
            "periodic_per_s": f"{periodic_per_s:.3f}",
            "window_event_count": window_event_count,
            "events_per_s": f"{events_per_s:.3f}",
            "cpu_permille": int(diag.get("cpu_permille", UNKNOWN_PERMILLE) or UNKNOWN_PERMILLE),
            "avg_exec_us": int(diag.get("avg_exec_us", 0) or 0),
            "p95_exec_us": int(diag.get("p95_exec_us", 0) or 0),
            "p99_exec_us": int(diag.get("p99_exec_us", 0) or 0),
            "window_max_exec_us": int(diag.get("window_max_exec_us", 0) or 0),
            "historical_loop_count": int(diag.get("loop_count", 0) or 0),
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
        io_fields = (
            ("read_count", "io_sample_count"),
            ("requested", "io_requested_count"),
            ("succeeded", "io_succeeded_count"),
            ("avg_us", "io_avg_us"),
            ("p95_us", "io_p95_us"),
            ("p99_us", "io_p99_us"),
        )
        for bus in range(4):
            for csv_name, diag_name in io_fields:
                values = list(diag.get(diag_name, []) or [])
                row[f"bus{bus}_{csv_name}"] = int(values[bus]) if bus < len(values) else 0
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
