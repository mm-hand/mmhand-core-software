import time
from collections import deque
from typing import Dict, List, Optional

import numpy as np

from protocol import ENCODER_COUNT
from .ui_qt_compat import QtCore, QtWidgets
import pyqtgraph as pg


JOINT_RT_MAX_PLOT_JOINTS = 5


class JointRealtimePlotView(QtWidgets.QDialog):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.setWindowTitle("Joint Real-Time Curve")
        self.resize(820, 420)
        self.window_sec = 12.0
        self.plot_interval = 0.05
        self.last_plot_ts = 0.0
        self.t0_map: Dict[int, float] = {}
        self.series_t: Dict[int, deque] = {}
        self.series_actual: Dict[int, deque] = {}
        self.series_target: Dict[int, deque] = {}
        self.curves = {}
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Joints:"))
        self.combos: List[QtWidgets.QComboBox] = []
        for i in range(JOINT_RT_MAX_PLOT_JOINTS):
            combo = QtWidgets.QComboBox()
            combo.addItem("None", None)
            for j in range(ENCODER_COUNT):
                combo.addItem(f"J{j:02d}", j)
            combo.currentIndexChanged.connect(lambda _i, slot=i: self._selection_changed(slot))
            top.addWidget(combo)
            self.combos.append(combo)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self.clear)
        top.addWidget(clear)
        top.addStretch(1)
        layout.addLayout(top)
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Angle", units="deg")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        layout.addWidget(self.plot, 1)
        self.combos[0].setCurrentIndex(1)

    def build(self, _parent) -> None:
        pass

    def selected_indices(self) -> List[int]:
        out = []
        for combo in self.combos:
            idx = combo.currentData()
            if idx is not None and idx not in out:
                out.append(int(idx))
        return out[:JOINT_RT_MAX_PLOT_JOINTS]

    def _selection_changed(self, slot: int) -> None:
        idx = self.combos[slot].currentData()
        if idx is not None:
            for i, combo in enumerate(self.combos):
                if i != slot and combo.currentData() == idx:
                    combo.setCurrentIndex(0)
        self.clear()

    def clear(self) -> None:
        self.series_t.clear()
        self.series_actual.clear()
        self.series_target.clear()
        self.t0_map.clear()
        self.curves.clear()
        self.plot.clear()

    def close(self) -> None:
        self.hide()

    def append_state(self, state) -> None:
        selected = self.selected_indices()
        if not selected:
            return
        targets = None
        try:
            targets = self.context.control_panel.resolved_joint_targets()
        except Exception:
            pass
        now = time.time()
        for idx in selected:
            self._ensure_series(idx)
            debug_ts = 0
            if idx < len(getattr(state, "joint_debug_timestamp_ms", [])):
                try:
                    debug_ts = int(state.joint_debug_timestamp_ms[idx])
                except Exception:
                    debug_ts = 0
            if debug_ts > 0:
                if idx not in self.t0_map:
                    self.t0_map[idx] = float(debug_ts)
                t_rel = (float(debug_ts) - self.t0_map[idx]) / 1000.0
                actual = self._safe_get(getattr(state, "joint_debug_actual_deg", []), idx)
                target = self._safe_get(getattr(state, "joint_debug_target_deg", []), idx)
                if self.series_t[idx] and t_rel <= float(self.series_t[idx][-1]):
                    continue
            else:
                if idx not in self.t0_map:
                    self.t0_map[idx] = now
                t_rel = now - self.t0_map[idx]
                actual = self._safe_get(getattr(state, "angles", []), idx)
                target = self._safe_get(targets or [], idx)
            self.series_t[idx].append(float(t_rel))
            self.series_actual[idx].append(float(actual))
            self.series_target[idx].append(float(target))

    def update_plot(self, force: bool = False) -> None:
        if not self.isVisible():
            return
        now = time.time()
        if not force and now - self.last_plot_ts < self.plot_interval:
            return
        self.last_plot_ts = now
        selected = self.selected_indices()
        colors = [pg.intColor(i, hues=max(10, ENCODER_COUNT)) for i in range(ENCODER_COUNT)]
        x_max = 0.0
        vals = []
        for idx in selected:
            t = np.asarray(self.series_t.get(idx, []), dtype=float)
            actual = np.asarray(self.series_actual.get(idx, []), dtype=float)
            target = np.asarray(self.series_target.get(idx, []), dtype=float)
            if t.size == 0:
                continue
            x_max = max(x_max, float(t[-1]))
            vals.extend([float(v) for v in actual if np.isfinite(v)])
            vals.extend([float(v) for v in target if np.isfinite(v)])
            for kind, data in [("actual", actual), ("target", target)]:
                key = (idx, kind)
                curve = self.curves.get(key)
                if curve is None:
                    pen = pg.mkPen(colors[idx], width=2 if kind == "actual" else 1.5)
                    if kind == "target":
                        pen.setStyle(QtCore.Qt.DashLine)
                    curve = self.plot.plot([], [], pen=pen, name=f"J{idx:02d} {kind}")
                    self.curves[key] = curve
                curve.setData(t, data)
        self.plot.setXRange(max(0.0, x_max - self.window_sec), max(self.window_sec, x_max + 0.05), padding=0)
        if vals:
            lo, hi = min(vals), max(vals)
            if lo == hi:
                hi = lo + 1.0
            margin = (hi - lo) * 0.12
            self.plot.setYRange(lo - margin, hi + margin, padding=0)

    def _ensure_series(self, idx: int) -> None:
        if idx not in self.series_t:
            self.series_t[idx] = deque(maxlen=1800)
            self.series_actual[idx] = deque(maxlen=1800)
            self.series_target[idx] = deque(maxlen=1800)

    @staticmethod
    def _safe_get(values, idx: int) -> float:
        try:
            return float(values[idx])
        except Exception:
            return float("nan")
