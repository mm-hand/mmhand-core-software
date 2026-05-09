from dataclasses import dataclass
from typing import List, Optional
import threading
import time

import numpy as np

from data_models import (
    HandModel,
    FingerTactile,
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
from .ui_qt_compat import QtCore, QtWidgets
import pyqtgraph as pg


TACTILE_REAL_INTERVAL_S = 0.10
TACTILE_FAKE_INTERVAL_S = 0.25


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
            pos = mat[np.isfinite(mat) & (mat >= 0.0)]
            has_data = bool(job.state and getattr(job.state, "tactile_data", None) and pos.size and np.max(pos) > 1e-9)
            with self._lock:
                self._latest = _TactileRenderResult(job.token, job.axis, mat, has_data)


class TactileView(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self.axis = 2
        self.fake_enabled = True
        self._last_update_ts = 0.0
        self._token = 0
        self._last_applied_token = 0
        self._fake_state: Optional[HandModel] = None
        self._worker = _LatestTactileMatrixWorker(self._build_matrix)
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Axis:"))
        self.axis_group = QtWidgets.QButtonGroup(self)
        for idx, label in enumerate(["X", "Y", "Z"]):
            btn = QtWidgets.QRadioButton(label)
            btn.setChecked(idx == self.axis)
            self.axis_group.addButton(btn, idx)
            top.addWidget(btn)
        self.fake_check = QtWidgets.QCheckBox("Fake")
        self.fake_check.setChecked(True)
        top.addWidget(self.fake_check)
        top.addStretch(1)
        layout.addLayout(top)
        self.axis_group.idClicked.connect(self._set_axis)
        self.fake_check.toggled.connect(self._set_fake)

        self.plot = pg.GraphicsLayoutWidget()
        self.view = self.plot.addViewBox(lockAspect=True)
        self.view.invertY(True)
        self.image = pg.ImageItem()
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
        layout.addWidget(container, 1)
        cmap = pg.colormap.get("inferno")
        self.image.setLookupTable(cmap.getLookupTable(0.0, 1.0, 256))
        self._apply_matrix(self._build_matrix(self._fake_display_state(None), self.axis), self.axis, True)

    def build(self, _parent) -> None:
        pass

    def update(self, state: Optional[HandModel]) -> None:
        self.update_if_due(state, force=True)

    def update_if_due(self, state: Optional[HandModel], *, force: bool = False) -> bool:
        self._apply_ready_frame()
        use_fake = self._should_use_fake(state)
        interval = TACTILE_FAKE_INTERVAL_S if use_fake else TACTILE_REAL_INTERVAL_S
        now = time.time()
        if not force and now - self._last_update_ts < interval:
            return False
        self._last_update_ts = now
        display_state = self._fake_display_state(state) if use_fake else state
        if force:
            mat = self._build_matrix(display_state, self.axis)
            pos = mat[np.isfinite(mat) & (mat >= 0.0)]
            self._apply_matrix(mat, self.axis, bool(display_state and getattr(display_state, "tactile_data", None) and pos.size and np.max(pos) > 1e-9))
            return True
        self._token += 1
        self._worker.submit(_TactileRenderJob(self._token, display_state, self.axis))
        return True

    def close(self) -> None:
        self._worker.close()

    def clear(self) -> None:
        self._apply_matrix(self._build_matrix(None, self.axis), self.axis, False)

    def _set_axis(self, axis: int) -> None:
        self.axis = int(axis)
        self.update_if_due(self.context.get_state(), force=True)

    def _set_fake(self, checked: bool) -> None:
        self.fake_enabled = bool(checked)
        self._fake_state = None
        self.update_if_due(self.context.get_state(), force=True)

    def _should_use_fake(self, state: Optional[HandModel]) -> bool:
        if not self.fake_enabled:
            return False
        if self.context.is_comm_connected() and state is not None:
            return False
        return True

    def _apply_ready_frame(self) -> None:
        result = self._worker.consume_latest()
        if result is None or result.axis != self.axis or result.token <= self._last_applied_token:
            return
        self._last_applied_token = result.token
        self._apply_matrix(result.matrix, result.axis, result.has_data)

    def _apply_matrix(self, mat: np.ndarray, axis: int, has_data: bool) -> None:
        _ = axis
        data = np.asarray(mat, dtype=float)
        finite = data[np.isfinite(data) & (data >= 0.0)]
        vmax = max(1.0, float(np.max(finite)) if finite.size else 1.0)
        display = np.nan_to_num(data, nan=-0.02)
        self.image.setImage(display.T, levels=(-0.02, vmax), autoLevels=False)
        self.no_data.setVisible(not has_data)

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
            for seg in (2, 0, 0):
                r, c = tactile_segment_shape(seg)
                cf = np.zeros((r, c, 3), dtype=float)
                phase = t * 0.8 + finger_idx + len(sensors) * 1.3
                cf[:, :, 2] = 0.16 + 0.12 * np.sin(phase) + np.random.rand(r, c) * 0.22
                cf[:, :, 0] = np.random.rand(r, c) * 0.06
                cf[:, :, 1] = np.random.rand(r, c) * 0.06
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
                    ch = np.clip(np.asarray(cf[:, :, min(axis, cf.shape[2] - 1)], dtype=float), 0, None)
                    if ch.shape == (h, w):
                        strip[row:row + h, :w] = ch
                    elif band == 0:
                        strip[row:row + h, :] = tactile_upsample_tip_grid_to_band0(ch)
                    else:
                        strip[row:row + h, :] = tactile_upsample_pad_grid_to_band(ch, h, w)
            row += h
        return strip

    def _build_matrix(self, state: Optional[HandModel], axis: int) -> np.ndarray:
        spec = TACTILE_STITCH_SPEC
        mat = np.full((spec.height, spec.width), np.nan, dtype=float)
        tactile_list = state.tactile_data if state and state.tactile_data else []

        def fd(idx: int) -> FingerTactile:
            return tactile_list[idx] if idx < len(tactile_list) else FingerTactile(sensors=[])

        strip0 = self._extract_finger_strip(fd(0), axis)
        mat[spec.thumb_r:spec.thumb_r + spec.thumb_h, spec.thumb_c:spec.thumb_c + spec.thumb_w] = strip0
        tactile_apply_fingertip_round_cap(mat, spec.thumb_r, spec.thumb_c, TACTILE_BAND_WIDTH)
        for fi in range(4):
            strip = self._extract_finger_strip(fd(fi + 1), axis)
            step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
            r0, c0 = spec.four_r, spec.four_c + fi * step
            mat[r0:r0 + TACTILE_FINGER_STRIP_ROWS, c0:c0 + TACTILE_BAND_WIDTH] = strip
            tactile_apply_fingertip_round_cap(mat, r0, c0, TACTILE_BAND_WIDTH)
        return mat
