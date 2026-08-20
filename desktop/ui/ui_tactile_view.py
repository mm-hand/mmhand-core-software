"""ui_tactile_view.py - MMHand Desktop 模块契约。

职责：展示五指触觉汇总和可选全量触觉阵列。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math
import threading
import time

import numpy as np

from desktop.core.hand_backend import list_ports
from desktop.core.data_models import (
    HandModel,
    FingerTactile,
    TACTILE_DATA_LEVEL_FULL,
    TACTILE_DATA_LEVEL_SUMMARY,
    TACTILE_SOURCE_P4_CAN_RELAY,
    TACTILE_SOURCE_S3_SERIAL,
    TactileSensor,
    tactile_band_shape,
    tactile_segment_shape,
    tactile_upsample_tip_grid_to_band0,
    tactile_upsample_pad_grid_to_band,
    TACTILE_BAND_WIDTH,
    TACTILE_FINGER_STRIP_ROWS,
    TACTILE_NBANDS,
    TACTILE_STITCH_SPEC,
    TACTILE_BAND_SENSOR_INDEX,
    TACTILE_FOUR_FINGER_GAP,
    TACTILE_JOINT_BAND_INDICES,
    tactile_apply_fingertip_round_cap,
)
from .official_tactile_layout import get_official_tactile_layout
from .ui_qt_compat import QtCore, QtWidgets
import pyqtgraph as pg


TACTILE_REAL_INTERVAL_S = 0.033
TACTILE_FAKE_INTERVAL_S = 0.25
TACTILE_NOISE_FLOOR = 1.0
TACTILE_RISE_ALPHA = 0.85
TACTILE_FALL_ALPHA = 0.45
TACTILE_DISPLAY_FLOOR = 6.0
TACTILE_AXIS_LEVELS = (70.0, 70.0, 180.0)
TACTILE_2D_ARRAY_SCALE = 8
TACTILE_2D_MIN_DOT_RADIUS = 2.4
TACTILE_2D_MAX_DOT_RADIUS = 13.0
TACTILE_2D_DENSE_POINT_LIMIT = 1500
TACTILE_2D_DENSE_MIN_DOT_RADIUS = 1.0
TACTILE_2D_DENSE_MAX_DOT_RADIUS = 4.2
TACTILE_2D_OFFICIAL_INTERP_STRIDE = 12
TACTILE_2D_Z_VISUAL_LEVEL = 35.0
TACTILE_2D_ARROW_FLOOR = 5.0
TACTILE_2D_ARROW_PAD_FLOOR = 3.0
TACTILE_2D_ARROW_CONTACT_FLOOR = TACTILE_DISPLAY_FLOOR
TACTILE_2D_ARROW_CONTACT_RATIO = 0.16
TACTILE_2D_ARROW_MAX_LEVEL = 12.0
TACTILE_2D_ARROW_GRID_STEP = 18
TACTILE_2D_ARROW_MAX_COUNT = 12
TACTILE_2D_ARROW_RESULTANT_FLOOR = 3.0
TACTILE_2D_ARROW_RESULTANT_Z_RATIO = 0.35
TACTILE_2D_SHOW_ARROWS_IN_Z_VIEW = True
TACTILE_2D_SHOW_HAND_GUIDE = True
TACTILE_2D_GUIDE_FILL = (248, 250, 252, 255)
TACTILE_2D_GUIDE_PALM_FILL = (241, 245, 249, 255)
TACTILE_2D_GUIDE_EDGE = (148, 163, 184, 255)
TACTILE_2D_GUIDE_SOFT_EDGE = (203, 213, 225, 255)
TACTILE_2D_IDLE_DOT = (71, 85, 105, 255)
TACTILE_2D_MANUAL_ORIGIN_AT_ROOT = True
TACTILE_2D_MANUAL_MIRROR_COLS = False
TACTILE_2D_REQUIRE_RESULTANT_GATE = False
TACTILE_FINGER_LABELS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
_OFFICIAL_TACTILE_PROJECTION_CACHE = {}
S3_TACTILE_PORT_SETTINGS_KEY = "s3_tactile/last_port"


@dataclass
class _TactileRenderJob:
    token: int
    state: Optional[HandModel]
    axis: int


@dataclass
class _TactileRenderResult:
    token: int
    axis: int
    matrix: np.ndarray
    has_data: bool


@dataclass(frozen=True)
class _TaxelPeak:
    score: float
    raw_index: int
    raw_row: int
    raw_col: int
    ui_row: int
    ui_col: int
    raw_xyz: Tuple[int, int, int]
    processed_xyz: Tuple[int, int, int]
    byte_z: Optional[int]


@dataclass(frozen=True)
class _TaxelCenter:
    total: float
    raw_row: float
    raw_col: float
    ui_row: float
    ui_col: float


class _LatestTactileMatrixWorker:
    def __init__(self, build_matrix):
        self._build_matrix = build_matrix
        self._pending: Optional[_TactileRenderJob] = None
        self._latest: Optional[_TactileRenderResult] = None
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, job: _TactileRenderJob) -> None:
        with self._lock:
            self._pending = job
        self._event.set()

    def clear(self) -> None:
        with self._lock:
            self._pending = None
            self._latest = None

    def consume_latest(self) -> Optional[_TactileRenderResult]:
        with self._lock:
            result = self._latest
            self._latest = None
            return result

    def close(self) -> None:
        self._stop.set()
        self._event.set()
        self._thread.join(timeout=0.5)

    def _run(self) -> None:
        while not self._stop.is_set():
            self._event.wait(timeout=0.25)
            self._event.clear()
            if self._stop.is_set():
                return
            with self._lock:
                job = self._pending
                self._pending = None
            if job is None:
                continue
            try:
                mat = self._build_matrix(job.state, job.axis)
            except Exception:
                continue
            has_data = bool(job.state and getattr(job.state, "tactile_data", None))
            with self._lock:
                self._latest = _TactileRenderResult(job.token, job.axis, mat, has_data)


class _TactileForcePanel(QtWidgets.QScrollArea):
    """展示触觉摘要包中的真实三轴合力，不把它伪装成 taxel 分布。"""

    _SENSOR_LABELS = ("Tip", "Pad-1", "Pad-2")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setMinimumWidth(286)
        self.setMaximumWidth(348)
        self.setStyleSheet(
            "QScrollArea { border: 1px solid #dbe4ee; border-radius: 10px; background: #f8fafc; }"
        )
        self._vector_labels: List[List[QtWidgets.QLabel]] = []
        self._build_ui()

    def _build_ui(self) -> None:
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        title = QtWidgets.QLabel("Real-Time 3-Axis Force")
        title.setStyleSheet("font-size:16px; font-weight:700; color:#0f172a;")
        layout.addWidget(title)
        self.source_label = QtWidgets.QLabel("NO DATA")
        self.source_label.setStyleSheet(self._badge_style("#64748b", "#f1f5f9"))
        layout.addWidget(self.source_label)
        self.total_label = QtWidgets.QLabel("Total Hand Force\nFx    0  Fy    0  Fz    0")
        self.total_label.setStyleSheet("color:#334155; font-family:Consolas, 'Courier New';")
        layout.addWidget(self.total_label)

        for finger_name in TACTILE_FINGER_LABELS:
            card = QtWidgets.QFrame()
            card.setStyleSheet(
                "QFrame { background:#ffffff; border:1px solid #e2e8f0; border-radius:8px; }"
            )
            card_layout = QtWidgets.QVBoxLayout(card)
            card_layout.setContentsMargins(9, 7, 9, 7)
            card_layout.setSpacing(3)
            header = QtWidgets.QLabel(finger_name)
            header.setStyleSheet("font-weight:700; color:#1e293b;")
            card_layout.addWidget(header)
            labels: List[QtWidgets.QLabel] = []
            for sensor_name in self._SENSOR_LABELS:
                label = QtWidgets.QLabel(f"{sensor_name:<5} Fx    0  Fy    0\n      Fz    0  |F|    0")
                label.setWordWrap(True)
                label.setStyleSheet("color:#475569; font-family:Consolas, 'Courier New'; font-size:11px;")
                labels.append(label)
                card_layout.addWidget(label)
            self._vector_labels.append(labels)
            layout.addWidget(card)
        layout.addStretch(1)
        self.setWidget(body)

    @staticmethod
    def _badge_style(foreground: str, background: str) -> str:
        return (
            f"color:{foreground}; background:{background}; border-radius:6px; padding:5px 7px; "
            "font-size:11px; font-weight:700;"
        )

    @staticmethod
    def _resultant(sensor: Optional[TactileSensor]) -> Tuple[int, int, int, int]:
        values = getattr(sensor, "resultant", None) if sensor is not None else None
        if values is None or len(values) < 3:
            return (0, 0, 0, 0)
        vector = np.asarray(values[:3], dtype=float)
        vector = np.where(np.isfinite(vector), vector, 0.0)
        fx, fy, fz = (int(round(float(value))) for value in vector)
        return fx, fy, fz, int(round(float(np.linalg.norm(vector))))

    def update_state(self, state: Optional[HandModel], *, demo: bool = False) -> None:
        tactile = list(getattr(state, "tactile_data", []) or []) if state is not None else []
        has_data = bool(tactile)
        source = str(getattr(state, "tactile_source", "NONE")) if state is not None else "NONE"
        data_level = str(getattr(state, "tactile_data_level", "NONE")) if state is not None else "NONE"
        valid_mask = int(getattr(state, "tactile_module_valid_mask", 0) or 0) if state is not None else 0
        online = bool(getattr(state, "tactile_online", False)) if state is not None else False
        last_rx = float(getattr(state, "tactile_last_rx_ts", 0.0) or 0.0) if state is not None else 0.0
        stale = not online or (last_rx > 0.0 and (time.time() - last_rx) > 0.6)
        if demo:
            source_text, foreground, background = "DEMO · synthetic data", "#0369a1", "#e0f2fe"
        elif not has_data:
            source_text, foreground, background = "NO DATA · waiting for tactile stream", "#64748b", "#f1f5f9"
        elif stale:
            source_text, foreground, background = "STALE · last tactile frame expired", "#b91c1c", "#fee2e2"
        else:
            source_name = {
                TACTILE_SOURCE_P4_CAN_RELAY: "P4 CAN RELAY",
                TACTILE_SOURCE_S3_SERIAL: "S3 SERIAL",
            }.get(source, source or "TACTILE")
            level_name = "FULL TAXEL" if data_level == TACTILE_DATA_LEVEL_FULL else "SUMMARY"
            valid_count = (valid_mask & 0x7FFF).bit_count()
            source_text = f"{source_name} · {level_name} · {valid_count}/15 VALID"
            if data_level == TACTILE_DATA_LEVEL_FULL:
                foreground, background = "#166534", "#dcfce7"
            else:
                foreground, background = "#9a3412", "#ffedd5"
        self.source_label.setText(source_text)
        self.source_label.setStyleSheet(self._badge_style(foreground, background))

        total = np.zeros(3, dtype=float)
        for finger_idx, labels in enumerate(self._vector_labels):
            finger = tactile[finger_idx] if finger_idx < len(tactile) else FingerTactile(sensors=[])
            for sensor_idx, label in enumerate(labels):
                sensor = finger.sensors[sensor_idx] if sensor_idx < len(finger.sensors) else None
                fx, fy, fz, magnitude = self._resultant(sensor)
                module_index = finger_idx * 3 + sensor_idx
                module_valid = bool(demo or (online and (valid_mask & (1 << module_index))))
                if module_valid:
                    total += (fx, fy, fz)
                    label.setText(
                        f"{self._SENSOR_LABELS[sensor_idx]:<5} Fx {fx:4d}  Fy {fy:4d}\n"
                        f"      Fz {fz:4d}  |F| {magnitude:4d}"
                    )
                    label.setStyleSheet(
                        "color:#166534; background:#f0fdf4; border-left:3px solid #22c55e; "
                        "padding:3px; font-family:Consolas, 'Courier New'; font-size:11px;"
                    )
                elif online:
                    label.setText(
                        f"{self._SENSOR_LABELS[sensor_idx]:<5} READ FAIL\n"
                        "      SPI/CRC invalid"
                    )
                    label.setStyleSheet(
                        "color:#b91c1c; background:#fef2f2; border-left:3px solid #ef4444; "
                        "padding:3px; font-family:Consolas, 'Courier New'; font-size:11px;"
                    )
                else:
                    label.setText(f"{self._SENSOR_LABELS[sensor_idx]:<5} OFFLINE\n      no fresh frame")
                    label.setStyleSheet(
                        "color:#64748b; background:#f1f5f9; border-left:3px solid #94a3b8; "
                        "padding:3px; font-family:Consolas, 'Courier New'; font-size:11px;"
                    )
        self.total_label.setText(
            f"Total Hand Force\nFx {int(total[0]):4d}  Fy {int(total[1]):4d}  Fz {int(total[2]):4d}"
        )


class TactileView(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.axis = 2
        self.fake_enabled = False
        self._last_update_ts = 0.0
        self._token = 0
        self._last_applied_token = 0
        self._fake_state: Optional[HandModel] = None
        self._display_matrix: Optional[np.ndarray] = None
        self._last_matrix_shape = None
        self._worker = _LatestTactileMatrixWorker(self._build_matrix)
        self.raw_visible = False
        self._raw_text_cache = ""
        self._settings = QtCore.QSettings("mmhand", "desktop")
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(9)
        self.setStyleSheet("TactileView { background:#f8fafc; }")

        top = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Tactile Monitor")
        title.setStyleSheet("font-size:18px; font-weight:700; color:#0f172a;")
        top.addWidget(title)
        top.addSpacing(14)
        axis_label = QtWidgets.QLabel("Force Axis")
        axis_label.setStyleSheet("font-weight:600; color:#475569;")
        top.addWidget(axis_label)
        self.axis_group = QtWidgets.QButtonGroup(self)
        for idx, label in enumerate(["X", "Y", "Z"]):
            btn = QtWidgets.QRadioButton(label)
            btn.setChecked(idx == self.axis)
            btn.setStyleSheet("color:#334155; font-weight:600;")
            self.axis_group.addButton(btn, idx)
            top.addWidget(btn)
        top.addStretch(1)
        self.frame_status_label = QtWidgets.QLabel("Waiting for S3 tactile data")
        self.frame_status_label.setStyleSheet(
            "color:#64748b; background:#f1f5f9; border-radius:7px; padding:6px 9px; font-weight:600;"
        )
        top.addWidget(self.frame_status_label)
        layout.addLayout(top)
        self.axis_group.idClicked.connect(self._set_axis)

        s3_row = QtWidgets.QHBoxLayout()
        s3_row.setSpacing(6)
        port_label = QtWidgets.QLabel("S3 Tactile Port")
        port_label.setStyleSheet("font-weight:600; color:#334155;")
        s3_row.addWidget(port_label)
        self.s3_port_combo = QtWidgets.QComboBox()
        self.s3_port_combo.setMinimumWidth(230)
        self.s3_port_combo.setToolTip("Only for S3 tactile serial. Do not select the P4 main control port.")
        s3_row.addWidget(self.s3_port_combo)
        self.s3_refresh_btn = QtWidgets.QPushButton("Refresh")
        self.s3_refresh_btn.setToolTip("Refresh available serial ports for the S3 tactile board")
        s3_row.addWidget(self.s3_refresh_btn)
        self.s3_connect_btn = QtWidgets.QPushButton("Connect S3")
        self.s3_connect_btn.setToolTip("Connect the selected port as the S3 tactile serial stream")
        s3_row.addWidget(self.s3_connect_btn)
        self.s3_disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.s3_disconnect_btn.setToolTip("Disconnect only the S3 tactile serial stream")
        s3_row.addWidget(self.s3_disconnect_btn)
        self.s3_status_label = QtWidgets.QLabel("S3 tactile: disconnected")
        self.s3_status_label.setMinimumWidth(220)
        self.s3_status_label.setStyleSheet("color:#475569; font-weight:600;")
        s3_row.addWidget(self.s3_status_label, 1)
        s3_hint = QtWidgets.QLabel("S3 tactile only, not P4 main port")
        s3_hint.setStyleSheet("color:#64748b;")
        s3_row.addWidget(s3_hint)
        layout.addLayout(s3_row)
        self.s3_refresh_btn.clicked.connect(lambda _checked=False: self._refresh_s3_ports())
        self.s3_connect_btn.clicked.connect(lambda _checked=False: self._connect_s3_tactile_port())
        self.s3_disconnect_btn.clicked.connect(lambda _checked=False: self._disconnect_s3_tactile_port())
        self._refresh_s3_ports()
        self._refresh_s3_status()

        self.plot = pg.GraphicsLayoutWidget()
        self.plot.setBackground("w")
        self.view = self.plot.addViewBox(lockAspect=True)
        self.view.invertY(True)
        self.image = pg.ImageItem()
        try:
            self.image.setAutoDownsample(False)
        except Exception:
            pass
        self.view.addItem(self.image)
        self.no_data = QtWidgets.QLabel("No tactile data")
        self.no_data.setAlignment(QtCore.Qt.AlignCenter)
        self.no_data.setStyleSheet("color: white; background: rgba(0, 0, 0, 120); padding: 4px;")
        stack = QtWidgets.QStackedLayout()
        holder = QtWidgets.QWidget()
        holder_layout = QtWidgets.QVBoxLayout(holder)
        holder_layout.setContentsMargins(0, 0, 0, 0)
        holder_layout.addWidget(self.plot)
        overlay = QtWidgets.QWidget()
        overlay_layout = QtWidgets.QVBoxLayout(overlay)
        overlay_layout.addStretch(1)
        overlay_layout.addWidget(self.no_data, alignment=QtCore.Qt.AlignCenter)
        overlay_layout.addStretch(1)
        stack.addWidget(holder)
        stack.addWidget(overlay)
        stack.setStackingMode(QtWidgets.QStackedLayout.StackAll)
        container = QtWidgets.QWidget()
        container.setLayout(stack)
        content = QtWidgets.QHBoxLayout()
        content.setSpacing(10)
        content.addWidget(container, 1)
        self.force_panel = _TactileForcePanel()
        content.addWidget(self.force_panel)
        layout.addLayout(content, 1)
        lut = np.zeros((256, 4), dtype=np.ubyte)
        lut[0] = (255, 255, 255, 255)  # outside the hand/sensor area
        lut[1] = (0, 0, 0, 255)        # valid sensor area with no pressure
        lut[2:, 0] = np.linspace(55, 255, 254).astype(np.ubyte)
        lut[2:, 1] = 0
        lut[2:, 2] = 0
        lut[2:, 3] = 255
        self.image.setLookupTable(lut)
        diagnostics_toggle = QtWidgets.QToolButton()
        diagnostics_toggle.setText("Diagnostics")
        diagnostics_toggle.setCheckable(True)
        diagnostics_toggle.setStyleSheet("color:#475569; font-weight:600; border:0; padding:4px 0;")
        layout.addWidget(diagnostics_toggle)
        diagnostics = QtWidgets.QWidget()
        diagnostics_layout = QtWidgets.QVBoxLayout(diagnostics)
        diagnostics_layout.setContentsMargins(0, 0, 0, 0)
        diagnostics_tools = QtWidgets.QHBoxLayout()
        self.fake_check = QtWidgets.QCheckBox("Demo Data")
        self.fake_check.setChecked(False)
        diagnostics_tools.addWidget(self.fake_check)
        self.raw_check = QtWidgets.QCheckBox("Raw Data")
        self.raw_check.setChecked(False)
        diagnostics_tools.addWidget(self.raw_check)
        diagnostics_tools.addStretch(1)
        diagnostics_layout.addLayout(diagnostics_tools)
        self.raw_text = QtWidgets.QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        self.raw_text.setMaximumHeight(260)
        self.raw_text.setVisible(False)
        try:
            self.raw_text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        except AttributeError:
            self.raw_text.setLineWrapMode(QtWidgets.QPlainTextEdit.LineWrapMode.NoWrap)
        self.raw_text.setStyleSheet(
            "color: #0f172a; background: #f8fafc; "
            "font-family: Consolas, 'Courier New', monospace; font-size: 11px;"
        )
        diagnostics_layout.addWidget(self.raw_text)
        diagnostics.setVisible(False)
        diagnostics_toggle.toggled.connect(diagnostics.setVisible)
        layout.addWidget(diagnostics)
        self.fake_check.toggled.connect(self._set_fake)
        self.raw_check.toggled.connect(self._set_raw_visible)
        self._apply_matrix(self._build_matrix(None, self.axis), self.axis, False)
        self.force_panel.update_state(None)

    def build(self, _parent) -> None:
        pass

    def update(self, state: Optional[HandModel]) -> None:
        self.update_if_due(state, force=True)

    def update_if_due(self, state: Optional[HandModel], *, force: bool = False) -> bool:
        use_fake = self._should_use_fake(state)
        interval = TACTILE_FAKE_INTERVAL_S if use_fake else TACTILE_REAL_INTERVAL_S
        now = time.time()
        if not force and now - self._last_update_ts < interval:
            return False
        self._last_update_ts = now
        self._refresh_s3_status()
        display_state = self._fake_display_state(state) if use_fake else state
        self._refresh_frame_status(display_state, demo=use_fake)
        self.force_panel.update_state(display_state, demo=use_fake)
        if self.raw_visible:
            self._update_raw_text(self._format_raw_values(display_state, self.axis))
        has_data = bool(display_state and getattr(display_state, "tactile_data", None))

        self._apply_ready_frame()
        if force:
            mat = self._build_matrix(display_state, self.axis)
            self._apply_matrix(mat, self.axis, has_data)
            return True
        self._token += 1
        self._worker.submit(_TactileRenderJob(self._token, display_state, self.axis))
        return True

    def close(self) -> None:
        self._worker.close()

    def clear(self) -> None:
        self._display_matrix = None
        self.raw_text.clear()
        self._raw_text_cache = ""
        self._refresh_frame_status(None)
        self.force_panel.update_state(None)
        self._apply_matrix(self._build_matrix(None, self.axis), self.axis, False)

    def _set_axis(self, axis: int) -> None:
        self.axis = int(axis)
        self._display_matrix = None
        self.update_if_due(self.context.get_state(), force=True)

    def _set_fake(self, checked: bool) -> None:
        self.fake_enabled = bool(checked)
        self._fake_state = None
        self._display_matrix = None
        self.update_if_due(self.context.get_state(), force=True)

    def _set_raw_visible(self, checked: bool) -> None:
        self.raw_visible = bool(checked)
        self.raw_text.setVisible(self.raw_visible)
        if not self.raw_visible:
            self.raw_text.clear()
            self._raw_text_cache = ""
            return
        self.update_if_due(self.context.get_state(), force=True)

    def _update_raw_text(self, text: str) -> None:
        if text == self._raw_text_cache:
            return
        self._raw_text_cache = text
        vbar = self.raw_text.verticalScrollBar()
        hbar = self.raw_text.horizontalScrollBar()
        old_v = vbar.value()
        old_h = hbar.value()
        self.raw_text.setUpdatesEnabled(False)
        try:
            self.raw_text.setPlainText(text)
            vbar.setValue(min(old_v, vbar.maximum()))
            hbar.setValue(min(old_h, hbar.maximum()))
        finally:
            self.raw_text.setUpdatesEnabled(True)

        def restore_scroll() -> None:
            vbar.setValue(min(old_v, vbar.maximum()))
            hbar.setValue(min(old_h, hbar.maximum()))

        QtCore.QTimer.singleShot(0, restore_scroll)

    def _selected_s3_port(self) -> str:
        data = self.s3_port_combo.currentData() if hasattr(self, "s3_port_combo") else None
        return str(data or "").strip()

    def _preferred_s3_port(self) -> str:
        try:
            current = str(self.context.get_tactile_serial_port() or "").strip()
        except Exception:
            current = ""
        if current:
            return current
        try:
            saved = self._settings.value(S3_TACTILE_PORT_SETTINGS_KEY, "", type=str)
        except TypeError:
            saved = self._settings.value(S3_TACTILE_PORT_SETTINGS_KEY, "")
        return str(saved or "").strip()

    def _refresh_s3_ports(self) -> None:
        preferred = self._preferred_s3_port() or self._selected_s3_port()
        self.s3_port_combo.blockSignals(True)
        self.s3_port_combo.clear()
        self.s3_port_combo.addItem("Select S3 tactile port", "")
        for device, desc in list_ports():
            self.s3_port_combo.addItem(f"{device} ({desc})", device)
        if preferred:
            idx = self.s3_port_combo.findData(preferred)
            if idx < 0:
                self.s3_port_combo.addItem(f"{preferred} (saved, not currently listed)", preferred)
                idx = self.s3_port_combo.findData(preferred)
            if idx >= 0:
                self.s3_port_combo.setCurrentIndex(idx)
        self.s3_port_combo.blockSignals(False)
        self._refresh_s3_status()

    def _p4_main_port(self) -> str:
        return str(getattr(self.context, "current_port", "") or "").strip()

    def _connect_s3_tactile_port(self) -> None:
        port = self._selected_s3_port()
        if not port:
            self.s3_status_label.setText("S3 tactile: select a port first")
            return
        p4_port = self._p4_main_port()
        if p4_port and port.upper() == p4_port.upper():
            QtWidgets.QMessageBox.warning(
                self,
                "S3 Tactile Port Conflict",
                (
                    f"{port} appears to be the P4 main control port.\n\n"
                    "Select a different serial port for the S3 tactile stream. "
                    "The S3 tactile stream and P4 control link cannot share one port."
                ),
            )
            self.s3_status_label.setText("S3 tactile: blocked, selected P4 port")
            return
        try:
            ok = bool(self.context.connect_tactile_serial(port))
        except Exception as exc:
            self.s3_status_label.setText(f"S3 tactile: failed ({exc})")
            return
        if ok:
            self._settings.setValue(S3_TACTILE_PORT_SETTINGS_KEY, port)
            self.s3_status_label.setText(f"S3 tactile connected: {port}")
            self.context.set_status(f"S3 tactile serial connected: {port}")
        else:
            self.s3_status_label.setText(f"S3 tactile connect failed: {port}")
        self._refresh_s3_status()

    def _disconnect_s3_tactile_port(self) -> None:
        try:
            self.context.disconnect_tactile_serial()
        except Exception as exc:
            self.s3_status_label.setText(f"S3 tactile disconnect failed: {exc}")
            return
        self.s3_status_label.setText("S3 tactile: disconnected")
        self.clear()
        self._refresh_s3_status()

    def _refresh_s3_status(self) -> None:
        if not hasattr(self, "s3_status_label"):
            return
        try:
            connected = bool(self.context.is_tactile_serial_connected())
        except Exception:
            connected = False
        try:
            port = str(self.context.get_tactile_serial_port() or "").strip()
        except Exception:
            port = ""
        if connected:
            text = f"S3 tactile connected: {port or self._selected_s3_port()}"
            color = "#166534"
        else:
            text = "S3 tactile: disconnected"
            color = "#475569"
        self.s3_status_label.setText(text)
        self.s3_status_label.setStyleSheet(f"color:{color};")
        self.s3_connect_btn.setEnabled(not connected)
        self.s3_disconnect_btn.setEnabled(connected)

    def _refresh_frame_status(self, state: Optional[HandModel], *, demo: bool = False) -> None:
        """显示当前热图是否来自真实全量 taxel，而非把摘要投影混淆为实测阵列。"""
        tactile = list(getattr(state, "tactile_data", []) or []) if state is not None else []
        source = str(getattr(state, "tactile_source", "NONE")) if state is not None else "NONE"
        data_level = str(getattr(state, "tactile_data_level", "NONE")) if state is not None else "NONE"
        online = bool(getattr(state, "tactile_online", False)) if state is not None else False
        valid_mask = int(getattr(state, "tactile_module_valid_mask", 0) or 0) if state is not None else 0
        last_rx = float(getattr(state, "tactile_last_rx_ts", 0.0) or 0.0) if state is not None else 0.0
        stale = not online or (last_rx > 0.0 and (time.time() - last_rx) > 0.6)
        if demo:
            text, foreground, background = "Demo Data", "#0369a1", "#e0f2fe"
        elif not tactile:
            text, foreground, background = "Waiting for tactile data", "#64748b", "#f1f5f9"
        elif stale:
            text, foreground, background = "Tactile Data Stale", "#b91c1c", "#fee2e2"
        else:
            source_name = "P4 Relay" if source == TACTILE_SOURCE_P4_CAN_RELAY else "S3 Serial"
            valid_count = (valid_mask & 0x7FFF).bit_count()
            if data_level == TACTILE_DATA_LEVEL_FULL:
                text = f"{source_name} · Full Taxel Array · {valid_count}/15 Valid"
                foreground, background = "#166534", "#dcfce7"
            else:
                text = f"{source_name} · Force Summary · {valid_count}/15 Valid"
                foreground, background = "#9a3412", "#ffedd5"
        self.frame_status_label.setText(text)
        self.frame_status_label.setStyleSheet(
            f"color:{foreground}; background:{background}; border-radius:7px; padding:6px 9px; font-weight:600;"
        )

    @staticmethod
    def _axis_name(axis: int) -> str:
        names = ("X", "Y", "Z")
        return names[min(max(int(axis), 0), len(names) - 1)]

    @staticmethod
    def _channel_matrix(values, axis: int) -> Optional[np.ndarray]:
        if values is None:
            return None
        arr = np.asarray(values, dtype=float)
        if arr.ndim == 2:
            return arr
        if arr.ndim < 3 or arr.shape[2] <= 0:
            return None
        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        return np.asarray(arr[:, :, axis_index], dtype=float)

    @staticmethod
    def _matrix_stats(matrix: Optional[np.ndarray]) -> str:
        if matrix is None:
            return "none"
        arr = np.asarray(matrix, dtype=float)
        finite = arr[np.isfinite(arr)]
        if finite.size == 0:
            return "none"
        return (
            f"min={int(round(float(np.min(finite))))} "
            f"max={int(round(float(np.max(finite))))} "
            f"abs_sum={int(round(float(np.sum(np.abs(finite)))))}"
        )

    @staticmethod
    def _vector_sum(values) -> Tuple[int, int, int]:
        if values is None:
            return (0, 0, 0)
        arr = np.asarray(values, dtype=float)
        if arr.ndim < 3 or arr.shape[2] <= 0:
            return (0, 0, 0)
        out = []
        for channel in range(3):
            if channel >= arr.shape[2]:
                out.append(0)
                continue
            ch = np.asarray(arr[:, :, channel], dtype=float)
            finite = np.where(np.isfinite(ch), ch, 0.0)
            out.append(int(round(float(np.sum(finite)))))
        return (out[0], out[1], out[2])

    @staticmethod
    def _resultant_tuple(sensor: TactileSensor) -> Tuple[int, int, int]:
        resultant = getattr(sensor, "resultant", None)
        if resultant is None or len(resultant) < 3:
            return (0, 0, 0)
        out = []
        for channel in range(3):
            value = float(resultant[channel])
            out.append(int(round(value)) if np.isfinite(value) else 0)
        return (out[0], out[1], out[2])

    @staticmethod
    def _vector_at(values, row: int, col: int) -> Tuple[int, int, int]:
        if values is None:
            return (0, 0, 0)
        arr = np.asarray(values, dtype=float)
        if arr.ndim < 3 or arr.shape[2] <= 0 or row >= arr.shape[0] or col >= arr.shape[1]:
            return (0, 0, 0)
        out = []
        for channel in range(3):
            if channel >= arr.shape[2]:
                out.append(0)
                continue
            value = arr[row, col, channel]
            out.append(int(round(float(value))) if np.isfinite(value) else 0)
        return (out[0], out[1], out[2])

    @staticmethod
    def _ui_rc_from_raw_rc(rows: int, cols: int, row: int, col: int) -> Tuple[int, int]:
        ui_row = rows - 1 - row if TACTILE_2D_MANUAL_ORIGIN_AT_ROOT else row
        ui_col = cols - 1 - col if TACTILE_2D_MANUAL_MIRROR_COLS else col
        return ui_row, ui_col

    @staticmethod
    def _ui_rc_from_raw_xy(rows: int, cols: int, row: float, col: float) -> Tuple[float, float]:
        ui_row = float(rows - 1) - row if TACTILE_2D_MANUAL_ORIGIN_AT_ROOT else row
        ui_col = float(cols - 1) - col if TACTILE_2D_MANUAL_MIRROR_COLS else col
        return ui_row, ui_col

    @staticmethod
    def _is_dp52_shape(rows: int, cols: int) -> bool:
        return int(rows) == 4 and int(cols) == 13

    @classmethod
    def _ui_rc_from_raw_rc_for_sensor(
        cls,
        sensor_index: int,
        rows: int,
        cols: int,
        row: int,
        col: int,
    ) -> Tuple[int, int]:
        if int(sensor_index) == 0 and cls._is_dp52_shape(rows, cols):
            return int(cols) - 1 - int(col), int(row)
        return cls._ui_rc_from_raw_rc(rows, cols, row, col)

    @classmethod
    def _ui_rc_from_raw_xy_for_sensor(
        cls,
        sensor_index: int,
        rows: int,
        cols: int,
        row: float,
        col: float,
    ) -> Tuple[float, float]:
        if int(sensor_index) == 0 and cls._is_dp52_shape(rows, cols):
            return float(cols - 1) - float(col), float(row)
        return cls._ui_rc_from_raw_xy(rows, cols, row, col)

    @classmethod
    def _find_sensor_peak(
        cls,
        sensor: TactileSensor,
        axis: int,
        sensor_index: int = -1,
    ) -> Optional[_TaxelPeak]:
        raw_values = getattr(sensor, "raw_contact_forces", None)
        processed_values = getattr(sensor, "contact_forces", None)
        source = raw_values if raw_values is not None else processed_values
        if source is None:
            return None
        arr = np.asarray(source, dtype=float)
        if arr.ndim < 3 or arr.shape[0] <= 0 or arr.shape[1] <= 0 or arr.shape[2] <= 0:
            return None

        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        channel = np.asarray(arr[:, :, axis_index], dtype=float)
        if axis_index == 2:
            scores = np.where(np.isfinite(channel), np.clip(channel, 0.0, None), -np.inf)
        else:
            scores = np.where(np.isfinite(channel), np.abs(channel), -np.inf)
        if not np.any(np.isfinite(scores)):
            return None
        score = float(np.nanmax(scores))
        if score <= 0.0:
            return None

        raw_row, raw_col = np.unravel_index(int(np.nanargmax(scores)), scores.shape)
        raw_row = int(raw_row)
        raw_col = int(raw_col)
        raw_index = raw_row * int(scores.shape[1]) + raw_col
        ui_row, ui_col = cls._ui_rc_from_raw_rc_for_sensor(
            sensor_index,
            int(scores.shape[0]),
            int(scores.shape[1]),
            raw_row,
            raw_col,
        )
        raw_unsigned = getattr(sensor, "raw_unsigned_contact_forces", None)
        byte_z = None
        if raw_unsigned is not None:
            unsigned_arr = np.asarray(raw_unsigned, dtype=float)
            if (
                unsigned_arr.ndim >= 3
                and raw_row < unsigned_arr.shape[0]
                and raw_col < unsigned_arr.shape[1]
                and unsigned_arr.shape[2] >= 3
            ):
                z_value = unsigned_arr[raw_row, raw_col, 2]
                byte_z = int(round(float(z_value))) if np.isfinite(z_value) else None
        return _TaxelPeak(
            score=score,
            raw_index=raw_index,
            raw_row=raw_row,
            raw_col=raw_col,
            ui_row=int(ui_row),
            ui_col=int(ui_col),
            raw_xyz=cls._vector_at(raw_values, raw_row, raw_col),
            processed_xyz=cls._vector_at(processed_values, raw_row, raw_col),
            byte_z=byte_z,
        )

    @classmethod
    def _find_sensor_center(
        cls,
        sensor: TactileSensor,
        axis: int,
        sensor_index: int = -1,
    ) -> Optional[_TaxelCenter]:
        raw_values = getattr(sensor, "raw_contact_forces", None)
        processed_values = getattr(sensor, "contact_forces", None)
        source = raw_values if raw_values is not None else processed_values
        if source is None:
            return None
        arr = np.asarray(source, dtype=float)
        if arr.ndim < 3 or arr.shape[0] <= 0 or arr.shape[1] <= 0 or arr.shape[2] <= 0:
            return None

        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        channel = np.asarray(arr[:, :, axis_index], dtype=float)
        if axis_index == 2:
            weights = np.where(np.isfinite(channel), np.clip(channel, 0.0, None), 0.0)
        else:
            weights = np.where(np.isfinite(channel), np.abs(channel), 0.0)
        total = float(np.sum(weights))
        if total <= 0.0:
            return None

        row_grid, col_grid = np.indices(weights.shape, dtype=float)
        raw_row = float(np.sum(row_grid * weights) / total)
        raw_col = float(np.sum(col_grid * weights) / total)
        ui_row, ui_col = cls._ui_rc_from_raw_xy_for_sensor(
            sensor_index,
            int(weights.shape[0]),
            int(weights.shape[1]),
            raw_row,
            raw_col,
        )
        return _TaxelCenter(
            total=total,
            raw_row=raw_row,
            raw_col=raw_col,
            ui_row=float(ui_row),
            ui_col=float(ui_col),
        )

    @staticmethod
    def _format_peak_detail(peak: Optional[_TaxelPeak]) -> str:
        if peak is None:
            return "none"
        address = 1038 + peak.raw_index * 3
        byte_z = "-" if peak.byte_z is None else str(peak.byte_z)
        return (
            f"i0={peak.raw_index:02d} i1={peak.raw_index + 1:02d} addr={address} "
            f"raw_rc=({peak.raw_row},{peak.raw_col}) ui_rc=({peak.ui_row},{peak.ui_col}) "
            f"raw=({peak.raw_xyz[0]},{peak.raw_xyz[1]},{peak.raw_xyz[2]}) "
            f"byteZ={byte_z} "
            f"proc=({peak.processed_xyz[0]},{peak.processed_xyz[1]},{peak.processed_xyz[2]}) "
            f"score={int(round(peak.score))}"
        )

    @staticmethod
    def _format_center_detail(center: Optional[_TaxelCenter]) -> str:
        if center is None:
            return "none"
        return (
            f"raw_rc=({center.raw_row:.2f},{center.raw_col:.2f}) "
            f"ui_rc=({center.ui_row:.2f},{center.ui_col:.2f}) "
            f"total={int(round(center.total))}"
        )

    @classmethod
    def _format_sum_detail(cls, sensor: TactileSensor) -> str:
        raw_sum = cls._vector_sum(getattr(sensor, "raw_contact_forces", None))
        proc_sum = cls._vector_sum(getattr(sensor, "contact_forces", None))
        summary = cls._resultant_tuple(sensor)
        raw_delta = (
            raw_sum[0] - summary[0],
            raw_sum[1] - summary[1],
            raw_sum[2] - summary[2],
        )
        proc_delta = (
            proc_sum[0] - summary[0],
            proc_sum[1] - summary[1],
            proc_sum[2] - summary[2],
        )
        return (
            f"raw_sum=({raw_sum[0]},{raw_sum[1]},{raw_sum[2]}) "
            f"proc_sum=({proc_sum[0]},{proc_sum[1]},{proc_sum[2]}) "
            f"summary=({summary[0]},{summary[1]},{summary[2]}) "
            f"raw-summary=({raw_delta[0]},{raw_delta[1]},{raw_delta[2]}) "
            f"proc-summary=({proc_delta[0]},{proc_delta[1]},{proc_delta[2]})"
        )

    @staticmethod
    def _format_matrix(matrix: Optional[np.ndarray]) -> str:
        if matrix is None:
            return "  <none>"
        arr = np.asarray(matrix, dtype=float)
        if arr.ndim != 2:
            return "  <bad-shape>"
        lines = []
        for row in arr:
            cells = []
            for value in row:
                if not np.isfinite(value):
                    cells.append("   .")
                else:
                    cells.append(f"{int(round(float(value))):4d}")
            lines.append(" ".join(cells))
        return "\n".join(lines)

    @classmethod
    def _format_raw_values(cls, state: Optional[HandModel], axis: int) -> str:
        tactile = state.tactile_data if state and state.tactile_data else []
        axis_index = min(max(int(axis), 0), 2)
        axis_name = cls._axis_name(axis_index)
        if not tactile:
            return f"Axis {axis_name}\nNo tactile data."

        lines = [f"Axis {axis_name} raw debug"]
        if state is not None:
            summary_seq = getattr(state, "tactile_summary_seq", None)
            full_seq = getattr(state, "tactile_full_seq", None)
            full_summary_seq = getattr(state, "tactile_full_summary_seq", None)
            full_crc = getattr(state, "tactile_full_crc", None)
            crc_text = f"0x{int(full_crc) & 0xFFFF:04X}" if full_crc is not None else "-"
            last_rx = float(getattr(state, "tactile_last_rx_ts", 0.0) or 0.0)
            age_text = f"{time.time() - last_rx:.2f}s" if last_rx > 0.0 else "-"
            source = str(getattr(state, "tactile_source", "NONE"))
            data_level = str(getattr(state, "tactile_data_level", "NONE"))
            module_valid_mask = int(getattr(state, "tactile_module_valid_mask", 0) or 0)
            lines.append(
                "link "
                f"online={1 if bool(getattr(state, 'tactile_online', False)) else 0} "
                f"age={age_text} "
                f"source={source} level={data_level} "
                f"module_valid=0x{module_valid_mask & 0x7FFF:04X} "
                f"summary_seq={summary_seq if summary_seq is not None else '-'} "
                f"full_seq={full_seq if full_seq is not None else '-'} "
                f"full_summary_seq={full_summary_seq if full_summary_seq is not None else '-'} "
                f"full_crc={crc_text} "
                f"crc_err={int(getattr(state, 'tactile_full_crc_errors', 0))} "
                f"seq_mismatch={int(getattr(state, 'tactile_full_seq_mismatches', 0))} "
                f"dropped={int(getattr(state, 'tactile_full_dropped', 0))}"
            )
        for fi in range(5):
            finger = tactile[fi] if fi < len(tactile) else FingerTactile(sensors=[])
            for si in range(3):
                if si >= len(finger.sensors):
                    continue
                sensor = finger.sensors[si]
                resultant = getattr(sensor, "resultant", None)
                if resultant is not None and len(resultant) >= 3:
                    result_text = (
                        f"({int(round(float(resultant[0]))):4d},"
                        f"{int(round(float(resultant[1]))):4d},"
                        f"{int(round(float(resultant[2]))):4d})"
                    )
                else:
                    result_text = "(   0,   0,   0)"

                raw_signed = cls._channel_matrix(getattr(sensor, "raw_contact_forces", None), axis_index)
                raw_unsigned_z = cls._channel_matrix(
                    getattr(sensor, "raw_unsigned_contact_forces", None),
                    2,
                )
                processed = cls._channel_matrix(getattr(sensor, "contact_forces", None), axis_index)

                lines.append(
                    f"G{fi} S{si} resultant={result_text} "
                    f"raw[{cls._matrix_stats(raw_signed)}] "
                    f"proc[{cls._matrix_stats(processed)}]"
                )
                lines.append(f"PEAK {axis_name}: {cls._format_peak_detail(cls._find_sensor_peak(sensor, axis_index, si))}")
                lines.append(f"CENTER {axis_name}: {cls._format_center_detail(cls._find_sensor_center(sensor, axis_index, si))}")
                lines.append(f"SUM: {cls._format_sum_detail(sensor)}")
                lines.append(f"RAW signed {axis_name}")
                lines.append(cls._format_matrix(raw_signed))
                if axis_index == 2:
                    lines.append("RAW byte Z")
                    lines.append(cls._format_matrix(raw_unsigned_z))
                lines.append(f"PROC {axis_name}")
                lines.append(cls._format_matrix(processed))
                lines.append("")
        return "\n".join(lines).rstrip()

    def _should_use_fake(self, state: Optional[HandModel]) -> bool:
        if not self.fake_enabled:
            return False
        if self.context.is_comm_connected() and state is not None:
            return False
        return True

    @staticmethod
    def _state_has_active_tactile_summary(state: Optional[HandModel]) -> bool:
        tactile = state.tactile_data if state and state.tactile_data else []
        for finger in tactile:
            for sensor in getattr(finger, "sensors", []) or []:
                resultant = getattr(sensor, "resultant", None)
                if resultant is None or len(resultant) < 3:
                    continue
                try:
                    if max(0.0, float(resultant[2])) >= TACTILE_DISPLAY_FLOOR:
                        return True
                except (TypeError, ValueError):
                    continue
        return False

    def _apply_ready_frame(self) -> None:
        result = self._worker.consume_latest()
        if result is None or result.axis != self.axis or result.token <= self._last_applied_token:
            return
        self._last_applied_token = result.token
        self._apply_matrix(result.matrix, result.axis, result.has_data)

    @staticmethod
    def _zero_tactile_data() -> List[FingerTactile]:
        fingers: List[FingerTactile] = []
        for _finger_idx in range(5):
            sensors: List[TactileSensor] = []
            for sensor_idx in range(3):
                rows, cols = tactile_segment_shape(0 if sensor_idx == 0 else 2)
                sensors.append(
                    TactileSensor(
                        contact_forces=np.zeros((rows, cols, 3), dtype=float),
                        resultant=np.zeros(3, dtype=float),
                    )
                )
            fingers.append(FingerTactile(sensors=sensors))
        return fingers

    def _build_zero_matrix(self, axis: int) -> np.ndarray:
        state = HandModel(tactile_data=self._zero_tactile_data())
        return self._build_matrix(state, axis)

    def _apply_matrix(
        self,
        mat: np.ndarray,
        axis: int,
        has_data: bool,
        *,
        reset_smoothing: bool = False,
    ) -> None:
        data = np.asarray(mat, dtype=float)
        if data.ndim == 2:
            vector_data = np.full(data.shape + (3,), np.nan, dtype=float)
            vector_data[:, :, 2] = data
        else:
            vector_data = data.copy()

        finite_mask = np.isfinite(vector_data[:, :, 2])
        current = vector_data.copy()
        z_low_mask = finite_mask & (np.abs(current[:, :, 2]) < TACTILE_NOISE_FLOOR)
        current[:, :, 2][z_low_mask] = 0.0
        xy_mag = np.sqrt(current[:, :, 0] * current[:, :, 0] + current[:, :, 1] * current[:, :, 1])
        xy_low_mask = finite_mask & (xy_mag < TACTILE_NOISE_FLOOR)
        current[:, :, 0][xy_low_mask] = 0.0
        current[:, :, 1][xy_low_mask] = 0.0

        smoothed = current.copy()
        previous = self._display_matrix
        if (
            not reset_smoothing
            and previous is not None
            and previous.shape == current.shape
        ):
            prev_finite = np.isfinite(previous[:, :, 2])
            blend_mask = finite_mask & prev_finite
            if np.any(blend_mask):
                current_abs = np.abs(current)
                previous_abs = np.abs(previous)
                alpha = np.where(
                    current_abs >= previous_abs,
                    TACTILE_RISE_ALPHA,
                    TACTILE_FALL_ALPHA,
                )
                smoothed[blend_mask] = (
                    previous[blend_mask] * (1.0 - alpha[blend_mask])
                    + current[blend_mask] * alpha[blend_mask]
                )

        display_floor_mask = finite_mask & (smoothed[:, :, 2] >= 0.0) & (smoothed[:, :, 2] < TACTILE_DISPLAY_FLOOR)
        smoothed[:, :, 2][display_floor_mask] = 0.0
        self._display_matrix = smoothed

        rendered = self._render_vector_matrix(smoothed, axis)
        self.image.setLookupTable(None)
        self.image.setImage(rendered.transpose(1, 0, 2), autoLevels=False)
        if self._last_matrix_shape != data.shape[:2]:
            self._last_matrix_shape = data.shape[:2]
            self.view.setRange(xRange=(0, data.shape[0]), yRange=(0, data.shape[1]), padding=0.02)
        self.no_data.setVisible(not has_data)

    @staticmethod
    def _tactile_band_for_matrix_point(row: int, col: int) -> Optional[int]:
        scale = max(1, int(TACTILE_2D_ARRAY_SCALE))
        base_row = int(row) // scale
        base_col = int(col) // scale
        spec = TACTILE_STITCH_SPEC

        def find_in_strip(top: int, left: int) -> Optional[int]:
            if not (
                top <= base_row < top + TACTILE_FINGER_STRIP_ROWS
                and left <= base_col < left + TACTILE_BAND_WIDTH
            ):
                return None
            local_row = base_row - top
            band_top = 0
            for band in range(TACTILE_NBANDS):
                band_h, _band_w = tactile_band_shape(band)
                if band_top <= local_row < band_top + band_h:
                    return band
                band_top += band_h
            return None

        thumb_band = find_in_strip(spec.thumb_r, spec.thumb_c)
        if thumb_band is not None:
            return thumb_band

        step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
        for fi in range(4):
            band = find_in_strip(spec.four_r, spec.four_c + fi * step)
            if band is not None:
                return band
        return None

    @staticmethod
    def _fill_rect(
        image: np.ndarray,
        row: int,
        col: int,
        height: int,
        width: int,
        color: Tuple[int, int, int, int],
    ) -> None:
        r0 = max(0, int(row))
        c0 = max(0, int(col))
        r1 = min(image.shape[0], int(row + height))
        c1 = min(image.shape[1], int(col + width))
        if r1 <= r0 or c1 <= c0:
            return
        image[r0:r1, c0:c1] = color

    @classmethod
    def _draw_rect_outline(
        cls,
        image: np.ndarray,
        row: int,
        col: int,
        height: int,
        width: int,
        color: Tuple[int, int, int, int],
        *,
        line_width: int = 1,
    ) -> None:
        r0 = int(row)
        c0 = int(col)
        r1 = int(row + height - 1)
        c1 = int(col + width - 1)
        cls._draw_line(image, r0, c0, r0, c1, color, width=line_width)
        cls._draw_line(image, r1, c0, r1, c1, color, width=line_width)
        cls._draw_line(image, r0, c0, r1, c0, color, width=line_width)
        cls._draw_line(image, r0, c1, r1, c1, color, width=line_width)

    @classmethod
    def _draw_tactile_hand_guide(cls, image: np.ndarray) -> None:
        if not TACTILE_2D_SHOW_HAND_GUIDE:
            return
        spec = TACTILE_STITCH_SPEC
        scale = max(1, int(TACTILE_2D_ARRAY_SCALE))

        def rect(units_row: int, units_col: int, units_h: int, units_w: int):
            return (
                int(units_row * scale),
                int(units_col * scale),
                int(units_h * scale),
                int(units_w * scale),
            )

        palm = rect(spec.palm_r, spec.palm_c, spec.palm_h, spec.palm_w)
        cls._fill_rect(image, *palm, TACTILE_2D_GUIDE_PALM_FILL)
        cls._draw_rect_outline(image, *palm, TACTILE_2D_GUIDE_EDGE, line_width=1)

        finger_rects: List[Tuple[int, int, int, int]] = [
            rect(spec.thumb_r, spec.thumb_c, spec.thumb_h, spec.thumb_w)
        ]
        step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
        for fi in range(4):
            finger_rects.append(
                rect(
                    spec.four_r,
                    spec.four_c + fi * step,
                    TACTILE_FINGER_STRIP_ROWS,
                    TACTILE_BAND_WIDTH,
                )
            )

        for r, c, h, w in finger_rects:
            cls._fill_rect(image, r, c, h, w, TACTILE_2D_GUIDE_FILL)
            cls._draw_rect_outline(image, r, c, h, w, TACTILE_2D_GUIDE_EDGE, line_width=1)

        # Soft connection lines make the separate sensor islands read as one hand.
        palm_top = spec.palm_r * scale
        palm_center_col = (spec.palm_c + spec.palm_w * 0.5) * scale
        for r, c, h, w in finger_rects[1:]:
            finger_root_row = r + h
            finger_center_col = c + w * 0.5
            cls._draw_line(
                image,
                palm_top,
                palm_center_col,
                finger_root_row,
                finger_center_col,
                TACTILE_2D_GUIDE_SOFT_EDGE,
                width=1,
            )

        thumb_r, thumb_c, thumb_h, thumb_w = finger_rects[0]
        cls._draw_line(
            image,
            spec.palm_r * scale + spec.palm_h * scale * 0.50,
            spec.palm_c * scale,
            thumb_r + thumb_h,
            thumb_c + thumb_w,
            TACTILE_2D_GUIDE_SOFT_EDGE,
            width=1,
        )

    def _render_vector_matrix(self, field: np.ndarray, axis: int) -> np.ndarray:
        rows, cols = field.shape[:2]
        image = np.full((rows, cols, 4), 255, dtype=np.uint8)
        finite = np.isfinite(field[:, :, 2])
        if not np.any(finite):
            return image

        self._draw_tactile_hand_guide(image)

        vmax_z = TACTILE_2D_Z_VISUAL_LEVEL
        positions = np.argwhere(finite)
        dense_mode = positions.shape[0] >= TACTILE_2D_DENSE_POINT_LIMIT
        min_radius = TACTILE_2D_DENSE_MIN_DOT_RADIUS if dense_mode else TACTILE_2D_MIN_DOT_RADIUS
        max_radius = TACTILE_2D_DENSE_MAX_DOT_RADIUS if dense_mode else TACTILE_2D_MAX_DOT_RADIUS
        base_rows = positions[:, 0]
        base_cols = positions[:, 1]
        idle_radius = 1 if dense_mode else 2
        self._draw_many_circles(
            image,
            base_rows,
            base_cols,
            idle_radius,
            TACTILE_2D_IDLE_DOT,
        )

        z_values = np.where(finite, np.clip(field[:, :, 2], 0.0, None), 0.0)
        active = finite & (z_values > 0.0)
        if np.any(active):
            active_positions = np.argwhere(active)
            active_rows = active_positions[:, 0]
            active_cols = active_positions[:, 1]
            z_norm = np.clip(z_values[active] / max(vmax_z, 1.0), 0.0, 1.0)
            radii = np.rint(
                min_radius
                + np.sqrt(z_norm) * (max_radius - min_radius)
            ).astype(np.int16)
            for radius in np.unique(radii):
                group = radii == radius
                ratio = (float(radius) - min_radius) / max(
                    1.0,
                    max_radius - min_radius,
                )
                red = int(round(180 + 75 * np.clip(ratio, 0.0, 1.0)))
                self._draw_many_circles(
                    image,
                    active_rows[group],
                    active_cols[group],
                    int(radius),
                    (red, 0, 0, 255),
                )

        if int(axis) == 2 and not TACTILE_2D_SHOW_ARROWS_IN_Z_VIEW:
            return image

        arrow_color = (0, 92, 255, 255)
        arrow_samples: Dict[Tuple[int, int], Tuple[float, int, int, float, float]] = {}
        fx_values = np.asarray(field[base_rows, base_cols, 0], dtype=float)
        fy_values = np.asarray(field[base_rows, base_cols, 1], dtype=float)
        fx_values = np.where(np.isfinite(fx_values), fx_values, 0.0)
        fy_values = np.where(np.isfinite(fy_values), fy_values, 0.0)
        mag_values = np.hypot(fx_values, fy_values)
        z_at_points = z_values[base_rows, base_cols]
        arrow_floor = np.full(mag_values.shape, float(TACTILE_2D_ARROW_PAD_FLOOR), dtype=float)
        contact = z_at_points >= TACTILE_2D_ARROW_CONTACT_FLOOR
        arrow_floor[contact] = np.maximum(
            arrow_floor[contact],
            z_at_points[contact] * TACTILE_2D_ARROW_CONTACT_RATIO,
        )
        arrow_indices = np.nonzero(mag_values >= arrow_floor)[0]
        for point_index in arrow_indices:
            row = int(base_rows[point_index])
            col = int(base_cols[point_index])
            fx = float(fx_values[point_index])
            fy = float(fy_values[point_index])
            mag = float(mag_values[point_index])
            z = max(0.0, float(z_at_points[point_index]))
            score = mag + z * 0.25
            key = (int(row) // TACTILE_2D_ARROW_GRID_STEP, int(col) // TACTILE_2D_ARROW_GRID_STEP)
            if key not in arrow_samples or score > arrow_samples[key][0]:
                arrow_samples[key] = (score, int(row), int(col), fx, fy)

        arrow_values = sorted(arrow_samples.values(), key=lambda item: item[0], reverse=True)
        for _score, row, col, fx, fy in arrow_values[:TACTILE_2D_ARROW_MAX_COUNT]:
            mag = math.hypot(fx, fy)
            unit_x = fx / max(mag, 1e-9)
            unit_y = -fy / max(mag, 1e-9)
            length = 12.0 + min(1.0, mag / max(TACTILE_2D_ARROW_MAX_LEVEL, 1.0)) * 34.0
            end_row = row + unit_y * length
            end_col = col + unit_x * length
            self._draw_line(image, row, col, end_row, end_col, (255, 255, 255, 255), width=4)
            self._draw_line(image, row, col, end_row, end_col, arrow_color, width=2)
            self._draw_arrow_head(image, row, col, end_row, end_col, arrow_color, width=3)
        return image

    @staticmethod
    def _draw_many_circles(
        image: np.ndarray,
        rows: np.ndarray,
        cols: np.ndarray,
        radius: int,
        color: Tuple[int, int, int, int],
    ) -> None:
        if radius <= 0 or rows.size == 0:
            return
        rows = np.asarray(rows, dtype=np.int32)
        cols = np.asarray(cols, dtype=np.int32)
        r2 = radius * radius
        for dy in range(-radius, radius + 1):
            span = int(math.floor(math.sqrt(max(0, r2 - dy * dy))))
            rr = rows + dy
            row_mask = (rr >= 0) & (rr < image.shape[0])
            if not np.any(row_mask):
                continue
            valid_rr = rr[row_mask]
            valid_cols = cols[row_mask]
            for dx in range(-span, span + 1):
                cc = valid_cols + dx
                mask = (cc >= 0) & (cc < image.shape[1])
                if np.any(mask):
                    image[valid_rr[mask], cc[mask]] = color

    @staticmethod
    def _draw_circle(
        image: np.ndarray,
        row: float,
        col: float,
        radius: int,
        color: Tuple[int, int, int, int],
    ) -> None:
        r0 = max(0, int(math.floor(row - radius)))
        r1 = min(image.shape[0], int(math.ceil(row + radius + 1)))
        c0 = max(0, int(math.floor(col - radius)))
        c1 = min(image.shape[1], int(math.ceil(col + radius + 1)))
        if r1 <= r0 or c1 <= c0:
            return
        yy, xx = np.ogrid[r0:r1, c0:c1]
        mask = (yy - row) * (yy - row) + (xx - col) * (xx - col) <= radius * radius
        image[r0:r1, c0:c1][mask] = color

    @classmethod
    def _draw_line(
        cls,
        image: np.ndarray,
        row0: float,
        col0: float,
        row1: float,
        col1: float,
        color: Tuple[int, int, int, int],
        *,
        width: int = 1,
    ) -> None:
        steps = max(1, int(math.ceil(max(abs(row1 - row0), abs(col1 - col0)) * 1.6)))
        t = np.linspace(0.0, 1.0, steps + 1, dtype=float)
        rows = np.rint(row0 + (row1 - row0) * t).astype(np.int32)
        cols = np.rint(col0 + (col1 - col0) * t).astype(np.int32)
        radius = max(0, int(width))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dy * dy + dx * dx > radius * radius:
                    continue
                rr = rows + dy
                cc = cols + dx
                mask = (rr >= 0) & (rr < image.shape[0]) & (cc >= 0) & (cc < image.shape[1])
                if np.any(mask):
                    image[rr[mask], cc[mask]] = color

    @classmethod
    def _draw_arrow_head(
        cls,
        image: np.ndarray,
        row0: float,
        col0: float,
        row1: float,
        col1: float,
        color: Tuple[int, int, int, int],
        *,
        width: int = 2,
    ) -> None:
        dx = col1 - col0
        dy = row1 - row0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return
        ux = dx / length
        uy = dy / length
        head = min(11.0, max(6.0, length * 0.42))
        spread = 0.62
        left_col = col1 - ux * head - uy * head * spread
        left_row = row1 - uy * head + ux * head * spread
        right_col = col1 - ux * head + uy * head * spread
        right_row = row1 - uy * head - ux * head * spread
        cls._draw_line(image, row1, col1, left_row, left_col, color, width=width)
        cls._draw_line(image, row1, col1, right_row, right_col, color, width=width)

    def _fake_display_state(self, state: Optional[HandModel]) -> HandModel:
        now = time.time()
        if self._fake_state is None or now - getattr(self, "_last_fake_ts", 0.0) >= TACTILE_FAKE_INTERVAL_S:
            out = HandModel()
            out.tactile_data = self._fake_tactile()
            self._fake_state = out
            self._last_fake_ts = now
        return self._fake_state

    def _fake_tactile(self) -> List[FingerTactile]:
        t = time.time()
        out: List[FingerTactile] = []
        for finger_idx in range(5):
            sensors: List[TactileSensor] = []
            for seg in (0, 2, 2):
                r, c = tactile_segment_shape(seg)
                cf = np.zeros((r, c, 3), dtype=float)
                phase = t * 0.8 + finger_idx + len(sensors) * 1.3
                cf[:, :, 2] = 18.0 + 10.0 * np.sin(phase) + np.random.rand(r, c) * 24.0
                cf[:, :, 0] = np.random.rand(r, c) * 6.0
                cf[:, :, 1] = np.random.rand(r, c) * 6.0
                sensors.append(TactileSensor(contact_forces=cf, resultant=np.mean(cf, axis=(0, 1))))
            out.append(FingerTactile(sensors=sensors))
        return out

    @staticmethod
    def _extract_finger_strip(fd: FingerTactile, axis: int) -> np.ndarray:
        strip = np.full((TACTILE_FINGER_STRIP_ROWS, TACTILE_BAND_WIDTH), np.nan, dtype=float)
        row = 0
        for band in range(TACTILE_NBANDS):
            h, w = tactile_band_shape(band)
            if band in TACTILE_JOINT_BAND_INDICES:
                row += h
                continue
            si = TACTILE_BAND_SENSOR_INDEX.get(band)
            if si is not None and si < len(fd.sensors):
                cf = getattr(fd.sensors[si], "contact_forces", None)
                if cf is not None and hasattr(cf, "shape") and len(cf.shape) >= 3:
                    raw_ch = np.asarray(cf[:, :, min(axis, cf.shape[2] - 1)], dtype=float)
                    ch = np.clip(raw_ch, 0, None) if axis == 2 else np.abs(raw_ch)
                    if ch.shape == (h, w):
                        strip[row:row + h, :w] = ch
                    elif band == 0:
                        strip[row:row + h, :] = tactile_upsample_tip_grid_to_band0(ch)
                    else:
                        strip[row:row + h, :] = tactile_upsample_pad_grid_to_band(ch, h, w)
            row += h
        return strip

    @staticmethod
    def _orient_manual_sensor_array(values: np.ndarray) -> np.ndarray:
        out = np.asarray(values, dtype=float)
        if TACTILE_2D_MANUAL_ORIGIN_AT_ROOT:
            out = np.flip(out, axis=0)
        if TACTILE_2D_MANUAL_MIRROR_COLS:
            out = np.flip(out, axis=1)
        return np.ascontiguousarray(out)

    @classmethod
    def _orient_sensor_array(cls, values: np.ndarray, sensor_index: int) -> np.ndarray:
        out = np.asarray(values, dtype=float)
        if int(sensor_index) == 0 and out.ndim >= 2 and cls._is_dp52_shape(out.shape[0], out.shape[1]):
            out = np.swapaxes(out, 0, 1)
            out = np.flip(out, axis=0)
            return np.ascontiguousarray(out)
        return cls._orient_manual_sensor_array(out)

    @staticmethod
    def _sensor_resultant_is_active(sensor: TactileSensor) -> bool:
        if not TACTILE_2D_REQUIRE_RESULTANT_GATE:
            return True
        cf = getattr(sensor, "contact_forces", None)
        if cf is not None and hasattr(cf, "shape") and len(cf.shape) >= 3:
            arr = np.asarray(cf, dtype=float)
            if arr.size and arr.shape[2] >= 3:
                z = np.clip(arr[:, :, 2], 0.0, None)
                if np.any(np.isfinite(z)) and float(np.nanmax(z)) >= TACTILE_DISPLAY_FLOOR:
                    return True
        resultant = getattr(sensor, "resultant", None)
        if resultant is None or len(resultant) < 3:
            return True
        force = np.asarray(resultant[:3], dtype=float)
        if not np.all(np.isfinite(force)):
            return True
        return max(0.0, float(force[2])) >= TACTILE_DISPLAY_FLOOR

    @staticmethod
    def _sensor_resultant_allows_arrows(sensor: TactileSensor) -> bool:
        resultant = getattr(sensor, "resultant", None)
        if resultant is None or len(resultant) < 3:
            return True
        force = np.asarray(resultant[:3], dtype=float)
        if not np.all(np.isfinite(force)):
            return True
        xy_mag = math.hypot(float(force[0]), float(force[1]))
        z_mag = max(0.0, float(force[2]))
        if xy_mag < TACTILE_2D_ARROW_RESULTANT_FLOOR:
            return False
        if z_mag >= TACTILE_DISPLAY_FLOOR and xy_mag < z_mag * TACTILE_2D_ARROW_RESULTANT_Z_RATIO:
            return False
        return True

    @classmethod
    def _extract_sensor_vector_array_raw(cls, fd: FingerTactile, sensor_index: int) -> np.ndarray:
        expected = tactile_segment_shape(0 if sensor_index == 0 else 2)
        out = np.zeros(expected + (3,), dtype=float)
        if sensor_index >= len(fd.sensors):
            return out
        sensor = fd.sensors[sensor_index]
        if not cls._sensor_resultant_is_active(sensor):
            return out
        cf = getattr(sensor, "contact_forces", None)
        if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
            return out

        arr = np.asarray(cf, dtype=float)
        rows = min(expected[0], arr.shape[0])
        cols = min(expected[1], arr.shape[1])
        chans = min(3, arr.shape[2])
        out[:rows, :cols, :chans] = arr[:rows, :cols, :chans]
        out[:, :, 2] = np.clip(out[:, :, 2], 0.0, None)
        return out

    @classmethod
    def _extract_sensor_vector_array(cls, fd: FingerTactile, sensor_index: int) -> np.ndarray:
        expected = tactile_segment_shape(0 if sensor_index == 0 else 2)
        out = np.zeros(expected + (3,), dtype=float)
        if sensor_index >= len(fd.sensors):
            return cls._orient_sensor_array(out, sensor_index)
        sensor = fd.sensors[sensor_index]
        if not cls._sensor_resultant_is_active(sensor):
            return cls._orient_sensor_array(out, sensor_index)
        cf = getattr(sensor, "contact_forces", None)
        if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
            return cls._orient_sensor_array(out, sensor_index)

        arr = np.asarray(cf, dtype=float)
        rows = min(expected[0], arr.shape[0])
        cols = min(expected[1], arr.shape[1])
        chans = min(3, arr.shape[2])
        out[:rows, :cols, :chans] = arr[:rows, :cols, :chans]
        out[:, :, 2] = np.clip(out[:, :, 2], 0.0, None)
        return cls._orient_sensor_array(out, sensor_index)

    @classmethod
    def _place_official_sensor_array(
        cls,
        mat: np.ndarray,
        top: int,
        left: int,
        height: int,
        width: int,
        values: np.ndarray,
    ) -> bool:
        arr = np.asarray(values, dtype=float)
        if arr.ndim < 3 or arr.shape[2] < 3:
            return False

        flat = arr.reshape((-1, arr.shape[2]))
        scale = max(1, int(TACTILE_2D_ARRAY_SCALE))
        region_top = int(top * scale)
        region_left = int(left * scale)
        region_h = max(1, int(height * scale))
        region_w = max(1, int(width * scale))
        projection = cls._official_sensor_projection(flat.shape[0], region_h, region_w)
        if projection is None:
            return False

        local_rows, local_cols, raw_indices, neighbors, weights, interp_mask = projection
        vectors = np.zeros((local_rows.shape[0], 3), dtype=float)
        raw_mask = raw_indices >= 0
        if np.any(raw_mask):
            vectors[raw_mask] = flat[raw_indices[raw_mask], :3]
            vectors[raw_mask, 2] = np.clip(vectors[raw_mask, 2], 0.0, None)

        if np.any(interp_mask):
            interp_neighbors = neighbors[interp_mask]
            interp_weights = weights[interp_mask]
            safe_neighbors = np.clip(interp_neighbors, 0, flat.shape[0] - 1)
            gathered = flat[safe_neighbors, :3] * interp_weights[:, :, None]
            gathered[interp_neighbors < 0] = 0.0
            vectors[interp_mask] = np.sum(gathered, axis=1)
            vectors[interp_mask, 2] = np.abs(vectors[interp_mask, 2])

        vectors = np.where(np.isfinite(vectors), vectors, 0.0)
        rows = region_top + local_rows
        cols = region_left + local_cols
        in_bounds = (
            (rows >= 0)
            & (rows < mat.shape[0])
            & (cols >= 0)
            & (cols < mat.shape[1])
        )
        if not np.any(in_bounds):
            return True

        scores = np.abs(vectors[:, 2]) + 0.2 * np.hypot(vectors[:, 0], vectors[:, 1])
        order = np.argsort(scores[in_bounds])
        use_rows = rows[in_bounds][order]
        use_cols = cols[in_bounds][order]
        use_vectors = vectors[in_bounds][order]
        mat[use_rows, use_cols, :] = use_vectors
        return True

    @staticmethod
    def _official_sensor_projection(taxel_count: int, region_h: int, region_w: int):
        key = (
            int(taxel_count),
            int(region_h),
            int(region_w),
            int(TACTILE_2D_OFFICIAL_INTERP_STRIDE),
        )
        cached = _OFFICIAL_TACTILE_PROJECTION_CACHE.get(key)
        if cached is not None:
            return cached

        layout = get_official_tactile_layout(taxel_count)
        if layout is None:
            return None

        span_x = max(1e-9, float(layout.max_x - layout.min_x))
        span_y = max(1e-9, float(layout.max_y - layout.min_y))
        pad = max(2.0, min(region_h, region_w) * 0.035)
        usable_h = max(1.0, region_h - 1 - 2.0 * pad)
        usable_w = max(1.0, region_w - 1 - 2.0 * pad)

        selected = []
        interp_stride = max(1, int(TACTILE_2D_OFFICIAL_INTERP_STRIDE))
        for point_index, point in enumerate(layout.points):
            if point.idx is None and interp_stride > 1 and point_index % interp_stride != 0:
                continue
            local_col = pad + ((float(point.x) - float(layout.min_x)) / span_x) * usable_w
            local_row = pad + ((float(layout.max_y) - float(point.y)) / span_y) * usable_h
            selected.append((int(round(local_row)), int(round(local_col)), point))

        if not selected:
            return None

        max_neighbors = max((len(point.neighbor) for _row, _col, point in selected), default=0)
        max_neighbors = max(1, max_neighbors)
        local_rows = np.asarray([row for row, _col, _point in selected], dtype=np.int32)
        local_cols = np.asarray([col for _row, col, _point in selected], dtype=np.int32)
        raw_indices = np.full((len(selected),), -1, dtype=np.int32)
        neighbors = np.full((len(selected), max_neighbors), -1, dtype=np.int32)
        weights = np.zeros((len(selected), max_neighbors), dtype=float)
        interp_mask = np.zeros((len(selected),), dtype=bool)

        for index, (_row, _col, point) in enumerate(selected):
            if point.idx is not None:
                raw_indices[index] = int(point.idx)
                continue
            interp_mask[index] = True
            count = min(max_neighbors, len(point.neighbor), len(point.shape))
            if count:
                neighbors[index, :count] = np.asarray(point.neighbor[:count], dtype=np.int32)
                weights[index, :count] = np.asarray(point.shape[:count], dtype=float)

        projection = (local_rows, local_cols, raw_indices, neighbors, weights, interp_mask)
        _OFFICIAL_TACTILE_PROJECTION_CACHE[key] = projection
        return projection

    @classmethod
    def _extract_sensor_array(cls, fd: FingerTactile, sensor_index: int, axis: int) -> np.ndarray:
        expected = tactile_segment_shape(0 if sensor_index == 0 else 2)
        out = np.zeros(expected, dtype=float)
        if sensor_index >= len(fd.sensors):
            return cls._orient_sensor_array(out, sensor_index)
        sensor = fd.sensors[sensor_index]
        if not cls._sensor_resultant_is_active(sensor):
            return cls._orient_sensor_array(out, sensor_index)
        cf = getattr(sensor, "contact_forces", None)
        if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
            return cls._orient_sensor_array(out, sensor_index)

        arr = np.asarray(cf, dtype=float)
        axis_index = min(max(int(axis), 0), arr.shape[2] - 1)
        raw = arr[:, :, axis_index]
        values = np.clip(raw, 0.0, None) if axis_index == 2 else np.abs(raw)
        if values.shape == expected:
            return cls._orient_sensor_array(values, sensor_index)
        if values.size == expected[0] * expected[1]:
            return cls._orient_sensor_array(values.reshape(expected), sensor_index)
        rows = min(expected[0], values.shape[0])
        cols = min(expected[1], values.shape[1])
        out[:rows, :cols] = values[:rows, :cols]
        return cls._orient_sensor_array(out, sensor_index)

    @staticmethod
    def _place_sensor_array(
        mat: np.ndarray,
        top: int,
        left: int,
        height: int,
        width: int,
        values: np.ndarray,
    ) -> None:
        scale = int(TACTILE_2D_ARRAY_SCALE)
        region_top = int(top * scale)
        region_left = int(left * scale)
        region_h = max(1, int(height * scale))
        region_w = max(1, int(width * scale))
        rows, cols = values.shape[:2]
        if rows <= 0 or cols <= 0:
            return

        row_pitch = region_h / rows
        col_pitch = region_w / cols
        for r in range(rows):
            cy = region_top + (r + 0.5) * row_pitch
            rr = int(round(cy))
            if rr < 0 or rr >= mat.shape[0]:
                continue
            for c in range(cols):
                cx = region_left + (c + 0.5) * col_pitch
                cc = int(round(cx))
                if cc < 0 or cc >= mat.shape[1]:
                    continue
                mat[rr, cc, ...] = values[r, c, ...] if values.ndim == 3 else values[r, c]

    def _place_finger_arrays(
        self,
        mat: np.ndarray,
        finger: FingerTactile,
        top: int,
        left: int,
        axis: int,
    ) -> None:
        _ = axis
        row = 0
        for band in range(TACTILE_NBANDS):
            h, w = tactile_band_shape(band)
            if band not in TACTILE_JOINT_BAND_INDICES:
                si = TACTILE_BAND_SENSOR_INDEX.get(band)
                if si is not None:
                    raw_values = self._extract_sensor_vector_array_raw(finger, si)
                    if not self._place_official_sensor_array(
                        mat,
                        top + row,
                        left,
                        h,
                        w,
                        raw_values,
                    ):
                        values = self._extract_sensor_vector_array(finger, si)
                        self._place_sensor_array(mat, top + row, left, h, w, values)
            row += h

    def _build_matrix(self, state: Optional[HandModel], axis: int) -> np.ndarray:
        spec = TACTILE_STITCH_SPEC
        scale = int(TACTILE_2D_ARRAY_SCALE)
        mat = np.full((spec.height * scale, spec.width * scale, 3), np.nan, dtype=float)
        tactile_list = state.tactile_data if state and state.tactile_data else []

        def fd(idx: int) -> FingerTactile:
            return tactile_list[idx] if idx < len(tactile_list) else FingerTactile(sensors=[])

        self._place_finger_arrays(mat, fd(0), spec.thumb_r, spec.thumb_c, axis)
        for fi in range(4):
            step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
            r0, c0 = spec.four_r, spec.four_c + fi * step
            self._place_finger_arrays(mat, fd(fi + 1), r0, c0, axis)
        return mat
