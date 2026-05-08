from dataclasses import dataclass
from typing import List, Optional
import threading
import time
import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import BoundaryNorm, ListedColormap, Normalize
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
    TACTILE_HEATMAP_STRUCTURE_VALUE,
    TACTILE_HEATMAP_GRAY_RGBA,
    TACTILE_HEATMAP_PALM_RGBA,
    tactile_apply_fingertip_round_cap,
    tactile_sensors_only_anatomy_points,
    tactile_sensors_only_finger_chain_polylines,
)

TACTILE_REAL_INTERVAL_S = 0.10
TACTILE_FAKE_INTERVAL_S = 0.25

def _make_tactile_heatmap_cmap_norm(vmax: float):
    """
    色标：PALM_VALUE → 深灰掌心；(p_hi, 0) 含 STRUCTURE -1 与数值缓冲段 → 浅灰（避免误用 inferno 纯黑）；
    [0, vmax] → inferno；NaN → 白。
    """
    vmax = max(float(vmax), 1e-9)
    p_lo, p_hi = TACTILE_HEATMAP_PALM_BOUNDS
    rest = np.linspace(0.0, vmax, 256)
    # [p_hi, 0.0) 整段浅灰：关节/间隙为 -1，且消除 (-0.99,0) 落进 inferno 底的问题
    bounds = [p_lo, p_hi, 0.0] + list(rest[1:])
    palm = np.array(TACTILE_HEATMAP_PALM_RGBA, dtype=float).reshape(1, 4)
    gray = np.array(TACTILE_HEATMAP_GRAY_RGBA, dtype=float).reshape(1, 4)
    # 258 个边界 → 257 区间：掌心 + 浅灰 + 255 段 inferno
    inferno_colors = plt.cm.inferno(np.linspace(0, 1, 255))
    colors = np.vstack([palm, gray, inferno_colors])
    cmap = ListedColormap(colors)
    cmap.set_bad("white")
    norm = BoundaryNorm(bounds, cmap.N, clip=True)
    return cmap, norm


def _make_tactile_sensors_only_cmap_norm(vmax: float):
    """仅显示传感数值：0~vmax→inferno，NaN 白；无掌心/结构浅灰段。"""
    vmax = max(float(vmax), 1e-9)
    cmap = plt.cm.inferno.copy()
    cmap.set_bad("white")
    norm = Normalize(vmin=0.0, vmax=vmax, clip=True)
    return cmap, norm


def _tactile_cell_is_sensor(mat: np.ndarray, col: float, row: float) -> bool:
    """当前热力图单元是否为有显示的触觉传感（finite 且 ≥0）。坐标：(x=列, y=行)。"""
    H, W = mat.shape
    c = int(round(col))
    r = int(round(row))
    if r < 0 or r >= H or c < 0 or c >= W:
        return False
    v = mat[r, c]
    return bool(np.isfinite(v) and v >= 0.0)


def _tactile_plot_dashed_skip_sensors(
    ax,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    mat: np.ndarray,
    artists: List,
    *,
    linewidth: float = 1.1,
    zorder: int = 12,
) -> None:
    """红色虚线仅从 p0→p1，落在非传感格上的子段才绘制（经过传感格的部分不显示）。"""
    dist = float(np.hypot(x1 - x0, y1 - y0))
    n = max(32, int(np.ceil(dist * 3.0)))
    if n < 2:
        return
    t = np.linspace(0.0, 1.0, n)
    xs = x0 + t * (x1 - x0)
    ys = y0 + t * (y1 - y0)
    on_sensor = np.array(
        [_tactile_cell_is_sensor(mat, float(xs[i]), float(ys[i])) for i in range(n)],
        dtype=bool,
    )
    i = 0
    while i < n:
        while i < n and bool(on_sensor[i]):
            i += 1
        if i >= n:
            break
        j = i + 1
        while j < n and not bool(on_sensor[j]):
            j += 1
        if j - i >= 2:
            (ln,) = ax.plot(
                xs[i:j],
                ys[i:j],
                color="red",
                linestyle="--",
                linewidth=linewidth,
                zorder=zorder,
                clip_on=True,
            )
            artists.append(ln)
        i = j

class TactileViewMixin:
    def _toggle_tactile_axis(self, axis: int):
        labels = ["X", "Y", "Z"]
        if axis == getattr(self, "_tactile_axis_selected", 2):
            return
        self._tactile_axis_selected = axis
        self._emit_action(f"Tactile axis {labels[axis]} selected")
        self._tactile_fig_sel_sig = None  # 强制重建子图布局
        self._refresh_tactile_axis_button_states()
        with self._state_lock:
            s = self._state
        if hasattr(self, "_display_windows"):
            self._display_windows.update_tactile_if_due(s or HandModel(), force=True)
        else:
            tactile_state = self._get_tactile_display_state(s, fake=self._should_use_fake_tactile(s))
            self._update_tactile_matrix_widget(tactile_state)

    def _on_tactile_fake_toggle(self):
        self._tactile_fake_mode = bool(self._tactile_fake_var.get())
        with self._state_lock:
            s = self._state
        if not s:
            s = HandModel()
        if hasattr(self, "_display_windows"):
            self._display_windows.update_tactile_if_due(s, force=True)
        else:
            tactile_state = self._get_tactile_display_state(s, fake=self._should_use_fake_tactile(s))
            self._update_tactile_matrix_widget(tactile_state)

    def _should_use_fake_tactile(self, s: Optional[HandModel]) -> bool:
        """Fake 触觉仅用于未连接/未收到设备状态时的离线预览；一旦成功读到设备数据，就按真实情况显示。"""
        if not bool(getattr(self, "_tactile_fake_mode", False)):
            return False
        if self._is_comm_connected() and s is not None:
            return False
        return True

    def _refresh_tactile_axis_button_states(self):
        if not hasattr(self, "_tactile_axis_buttons"):
            return
        for i, btn in enumerate(self._tactile_axis_buttons):
            selected = i == getattr(self, "_tactile_axis_selected", 2)
            if selected:
                btn.configure(bg="#e0e0e0", fg="blue")
            else:
                btn.configure(bg="#e0e0e0", fg="black")

    def _get_tactile_display_state(self, s: Optional[HandModel], fake: bool = False) -> Optional[HandModel]:
        """
        fake=False（默认）：无设备连接或无触觉数据时显示 "No tactile data"，有设备输入时显示真实数据
        fake=True：始终显示生成器数据
        """
        if fake:
            out = HandModel()
            out.tactile_data = self._tactile_generator()
            return out
        return s

    def _tactile_generator(self) -> List[FingerTactile]:
        """每指 3 组数据：尖 5×5(25)、两腹各 4×13(52)；热力图条带几何仍由上采样嵌入"""
        t = time.time()
        out: List[FingerTactile] = []
        for finger_idx in range(5):
            sensors: List[TactileSensor] = []
            # 指尖 25
            rt, ct = tactile_segment_shape(2)
            cf_tip = np.zeros((rt, ct, 3), dtype=float)
            base_tip = 0.28 * (1.4 if finger_idx in (0, 1) else 1.0)
            phase = (t * 0.5 + finger_idx) % (2 * np.pi)
            cf_tip[:, :, 0] = np.sin(phase) * 0.1 + np.random.rand(rt, ct) * 0.1
            cf_tip[:, :, 1] = np.cos(phase * 0.7) * 0.1 + np.random.rand(rt, ct) * 0.1
            cf_tip[:, :, 2] = base_tip + np.sin(phase * 1.2) * 0.14 + np.random.rand(rt, ct) * 0.28
            res_tip = np.array(
                [np.mean(cf_tip[:, :, 0]), np.mean(cf_tip[:, :, 1]), np.mean(cf_tip[:, :, 2])]
            )
            sensors.append(TactileSensor(contact_forces=cf_tip, resultant=res_tip))
            # 两指腹各 52
            for pk in range(2):
                rp, cp = tactile_segment_shape(0)
                cf_p = np.zeros((rp, cp, 3), dtype=float)
                base_p = 0.12 * (1.4 if finger_idx in (0, 1) else 1.0)
                ph = (t * 0.5 + finger_idx + (pk + 1) * 1.3) % (2 * np.pi)
                cf_p[:, :, 0] = np.sin(ph) * 0.1 + np.random.rand(rp, cp) * 0.1
                cf_p[:, :, 1] = np.cos(ph * 0.7) * 0.1 + np.random.rand(rp, cp) * 0.1
                cf_p[:, :, 2] = base_p + np.sin(ph * 1.2) * 0.12 + np.random.rand(rp, cp) * 0.25
                res_p = np.array(
                    [np.mean(cf_p[:, :, 0]), np.mean(cf_p[:, :, 1]), np.mean(cf_p[:, :, 2])]
                )
                sensors.append(TactileSensor(contact_forces=cf_p, resultant=res_p))
            out.append(FingerTactile(sensors=sensors))
        return out

    def _rebuild_tactile_figure(self) -> None:
        """触觉区固定为单子图，仅显示当前选中的一个通道。"""
        if not hasattr(self, "_tactile_fig"):
            return
        self._clear_tactile_anatomy_overlay()
        self._tactile_fig.clear()
        self._tactile_subplot_imgs = []
        self._tactile_no_data_texts = []
        self._tactile_overlay_sig = None
        self._tactile_norm_vmax = None
        spec = TACTILE_STITCH_SPEC
        mat0 = np.full((spec.height, spec.width), np.nan, dtype=float)
        ax = self._tactile_fig.subplots(1, 1)
        cmap0, norm0 = _make_tactile_sensors_only_cmap_norm(1.0)
        im = ax.imshow(
            np.ma.masked_invalid(mat0),
            cmap=cmap0,
            norm=norm0,
            aspect="equal",
            interpolation="nearest",
        )
        nd = ax.text(
            0.5,
            0.5,
            "No tactile data",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=9,
            color="white",
            alpha=0.85,
        )
        ax.axis("off")
        self._tactile_subplot_imgs.append(im)
        self._tactile_no_data_texts.append(nd)
        self._tactile_fig.subplots_adjust(left=0.01, right=0.99, bottom=0.02, top=0.98)

    def _setup_tactile_matrix_widget(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        self._tactile_row = ttk.Frame(frame)
        self._tactile_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(self._tactile_row, text="Axis:").pack(side=tk.LEFT, padx=2)
        self._tactile_fake_var = tk.BooleanVar(value=bool(getattr(self, "_tactile_fake_mode", True)))
        self._tactile_fake_check = ttk.Checkbutton(
            self._tactile_row,
            text="Fake",
            variable=self._tactile_fake_var,
            command=self._on_tactile_fake_toggle,
        )
        self._tactile_fake_check.pack(side=tk.LEFT, padx=4)
        self._tactile_axis_buttons = []
        for axis, label in enumerate(["X", "Y", "Z"]):
            btn = tk.Button(
                self._tactile_row,
                text=label,
                width=2,
                command=lambda a=axis: self._toggle_tactile_axis(a),
            )
            btn.pack(side=tk.LEFT, padx=1)
            self._tactile_axis_buttons.append(btn)
        self._refresh_tactile_axis_button_states()

        self._tactile_fig = plt.Figure(figsize=(7.8, 4.0))
        self._tactile_canvas = FigureCanvasTkAgg(self._tactile_fig, frame)
        self._tactile_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._tactile_fig_sel_sig = None
        self._tactile_subplot_imgs: List = []
        self._tactile_no_data_texts: List = []
        self._rebuild_tactile_figure()
        self._tactile_fig_sel_sig = int(getattr(self, "_tactile_axis_selected", 2))
        self._tactile_anatomy_artists: List = []
        self._tactile_overlay_sig = None
        self._tactile_norm_vmax = None
        self._tactile_canvas.draw_idle()

    def _safe_remove_artist(self, artist) -> None:
        if artist is None:
            return
        try:
            artist.remove()
        except (AttributeError, ValueError, NotImplementedError):
            pass

    def _clear_tactile_anatomy_overlay(self) -> None:
        for art in getattr(self, "_tactile_anatomy_artists", []) or []:
            self._safe_remove_artist(art)
        self._tactile_anatomy_artists = []

    def _draw_tactile_anatomy_overlay_sensors_only(self, ax, mat: np.ndarray) -> None:
        """仅传感：红点不变；红色虚线不画过当前通道热力图中「传感」格（finite 且 ≥0）。"""
        spec = TACTILE_STITCH_SPEC
        palm_xy, finger_pts = tactile_sensors_only_anatomy_points(spec)
        chains = tactile_sensors_only_finger_chain_polylines(spec)
        px, py = palm_xy
        art = self._tactile_anatomy_artists
        for fx, fy in finger_pts:
            _tactile_plot_dashed_skip_sensors(ax, px, py, fx, fy, mat, art, linewidth=1.15)
        for chain in chains:
            for k in range(len(chain) - 1):
                x0, y0 = chain[k]
                x1, y1 = chain[k + 1]
                _tactile_plot_dashed_skip_sensors(ax, x0, y0, x1, y1, mat, art, linewidth=1.05)
        xs_all = [px]
        ys_all = [py]
        for ch in chains:
            for x, y in ch:
                xs_all.append(x)
                ys_all.append(y)
        sc = ax.scatter(
            xs_all,
            ys_all,
            c="red",
            s=32,
            zorder=13,
            edgecolors="darkred",
            linewidths=0.6,
            clip_on=True,
        )
        self._tactile_anatomy_artists.append(sc)

    @staticmethod
    def _extract_finger_strip(
        fd: FingerTactile, axis: int, structure_as_nan: bool = False
    ) -> np.ndarray:
        """竖条几何不变；尖 5×5、腹 4×13 上采样进条带，或与条带同尺寸则直贴。
        structure_as_nan=True 时不填关节浅灰，保留 NaN（仅传感视图）。"""
        strip = np.full((TACTILE_FINGER_STRIP_ROWS, TACTILE_BAND_WIDTH), np.nan, dtype=float)
        row = 0
        for band in range(TACTILE_NBANDS):
            h, w = tactile_band_shape(band)
            if band in TACTILE_JOINT_BAND_INDICES:
                if not structure_as_nan:
                    strip[row : row + h, :] = TACTILE_HEATMAP_STRUCTURE_VALUE
            elif band in TACTILE_BAND_SENSOR_INDEX:
                si = TACTILE_BAND_SENSOR_INDEX[band]
                if si < len(fd.sensors):
                    s = fd.sensors[si]
                    cf = getattr(s, "contact_forces", None)
                    if cf is not None and hasattr(cf, "shape") and len(cf.shape) >= 3:
                        ax_i = min(int(axis), int(cf.shape[2]) - 1)
                        ch2 = np.asarray(cf[:, :, ax_i], dtype=float)
                        ch2 = np.clip(ch2, 0, None)
                        if ch2.shape == (h, w):
                            strip[row : row + h, :w] = ch2
                        elif band == 0:
                            strip[row : row + h, :] = tactile_upsample_tip_grid_to_band0(ch2)
                        else:
                            strip[row : row + h, :] = tactile_upsample_pad_grid_to_band(ch2, h, w)
            elif len(fd.sensors) > band:
                s = fd.sensors[band]
                cf = getattr(s, "contact_forces", None)
                if cf is not None and hasattr(cf, "shape") and len(cf.shape) >= 3:
                    ax_i = min(int(axis), int(cf.shape[2]) - 1)
                    grid = np.clip(np.asarray(cf[:, :, ax_i], dtype=float), 0, None)
                    if grid.shape == (h, w):
                        strip[row : row + h, :w] = grid
            row += h
        return strip

    def _build_tactile_spatial_matrix(self, state: Optional[HandModel], axis: int) -> np.ndarray:
        """
        整手画布（axis：0=X,1=Y,2=Z）：仅传感布局——不填掌心与浅灰结构区，关节条带为透明，五指条带位置不变。
        """
        spec = TACTILE_STITCH_SPEC
        mat = np.full((spec.height, spec.width), np.nan, dtype=float)
        tactile_list = state.tactile_data if state and state.tactile_data else []

        def _fd(finger_idx: int) -> FingerTactile:
            if finger_idx < len(tactile_list):
                return tactile_list[finger_idx]
            return FingerTactile(sensors=[])

        strip0 = self._extract_finger_strip(_fd(0), axis, structure_as_nan=True)
        mat[
            spec.thumb_r : spec.thumb_r + spec.thumb_h,
            spec.thumb_c : spec.thumb_c + spec.thumb_w,
        ] = strip0
        tactile_apply_fingertip_round_cap(
            mat, spec.thumb_r, spec.thumb_c, TACTILE_BAND_WIDTH
        )

        for fi in range(4):
            strip = self._extract_finger_strip(
                _fd(fi + 1), axis, structure_as_nan=True
            )
            step = TACTILE_BAND_WIDTH + TACTILE_FOUR_FINGER_GAP
            r0, c0 = spec.four_r, spec.four_c + fi * step
            mat[r0 : r0 + TACTILE_FINGER_STRIP_ROWS, c0 : c0 + TACTILE_BAND_WIDTH] = strip
            tactile_apply_fingertip_round_cap(mat, r0, c0, TACTILE_BAND_WIDTH)

        return mat

    def _update_tactile_matrix_widget(self, state: Optional[HandModel]):
        if not hasattr(self, "_tactile_subplot_imgs"):
            return
        axis = int(getattr(self, "_tactile_axis_selected", 2))
        mat = self._build_tactile_spatial_matrix(state, axis)
        tactile_pos = mat[np.isfinite(mat) & (mat >= 0.0)]
        has_data = bool(
            state
            and state.tactile_data
            and tactile_pos.size > 0
            and (np.max(tactile_pos) > 1e-9)
        )
        self._apply_tactile_matrix_widget(mat, axis, has_data)

    def _apply_tactile_matrix_widget(self, mat: np.ndarray, axis: int, has_data: bool):
        if not hasattr(self, "_tactile_subplot_imgs"):
            return
        sig = int(axis)
        if sig != getattr(self, "_tactile_fig_sel_sig", object()):
            self._rebuild_tactile_figure()
            self._tactile_fig_sel_sig = sig

        im = self._tactile_subplot_imgs[0]
        nd_txt = self._tactile_no_data_texts[0]
        tactile_pos = mat[np.isfinite(mat) & (mat >= 0.0)]
        vmax = float(np.max(tactile_pos)) if tactile_pos.size > 0 else 0.0
        vmax = max(vmax, 1.0)
        prev_vmax = getattr(self, "_tactile_norm_vmax", None)
        if prev_vmax is None or abs(float(prev_vmax) - vmax) > max(0.05 * vmax, 0.05):
            cmap_u, norm_u = _make_tactile_sensors_only_cmap_norm(vmax)
            im.set_cmap(cmap_u)
            im.set_norm(norm_u)
            self._tactile_norm_vmax = vmax
        im.set_data(np.ma.masked_invalid(mat))
        nd_txt.set_visible(not has_data)
        sensor_mask = np.isfinite(mat) & (mat >= 0.0)
        overlay_sig = (sig, sensor_mask.shape, sensor_mask.tobytes())
        if overlay_sig != getattr(self, "_tactile_overlay_sig", None):
            self._clear_tactile_anatomy_overlay()
            self._draw_tactile_anatomy_overlay_sensors_only(im.axes, mat)
            self._tactile_overlay_sig = overlay_sig
        self._tactile_canvas.draw_idle()


@dataclass
class _TactileRenderJob:
    token: int
    state: Optional[HandModel]
    axis: int


@dataclass
class _TactileRenderResult:
    token: int
    axis: int
    mat: np.ndarray
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
            tactile_pos = mat[np.isfinite(mat) & (mat >= 0.0)]
            has_data = bool(
                job.state
                and getattr(job.state, "tactile_data", None)
                and tactile_pos.size > 0
                and (np.max(tactile_pos) > 1e-9)
            )
            with self._lock:
                self._latest = _TactileRenderResult(job.token, job.axis, mat, has_data)


class _ComponentProxy:
    def __init__(self, owner):
        self.owner = owner
        self.root = owner.root

    def __getattr__(self, name):
        return getattr(self.owner, name)


class TactileView(_ComponentProxy, TactileViewMixin):
    def __init__(self, owner):
        super().__init__(owner)
        self._last_update_ts = 0.0
        self._last_applied_token = 0
        self._token = 0
        self._fake_state: Optional[HandModel] = None
        self._worker = _LatestTactileMatrixWorker(self._build_tactile_spatial_matrix)

    def build(self, parent) -> None:
        self._setup_tactile_matrix_widget(parent)
        self.update_if_due(self._current_state(), force=True)

    def update(self, state: Optional[HandModel]) -> None:
        self.update_if_due(state, force=True)

    def update_if_due(self, state: Optional[HandModel], *, force: bool = False) -> bool:
        self._apply_ready_frame()
        use_fake = self._should_use_fake_tactile(state)
        interval = TACTILE_FAKE_INTERVAL_S if use_fake else TACTILE_REAL_INTERVAL_S
        now = time.time()
        if not force and (now - self._last_update_ts) < interval:
            return False
        self._last_update_ts = now
        tactile_state = self._display_state(state, use_fake, now, force=force)
        if force:
            self._render_sync(tactile_state)
            return True
        axis = int(getattr(self, "_tactile_axis_selected", 2))
        self._token += 1
        self._worker.submit(_TactileRenderJob(self._token, tactile_state, axis))
        return True

    def force_refresh(self) -> None:
        self.update_if_due(self._current_state(), force=True)

    def clear(self) -> None:
        self._update_tactile_matrix_widget(None)

    def close(self) -> None:
        self._worker.close()

    def _current_state(self) -> Optional[HandModel]:
        return self.owner.get_state() if hasattr(self.owner, "get_state") else getattr(self.owner, "_state", None)

    def _display_state(self, state: Optional[HandModel], use_fake: bool, now: float, *, force: bool) -> Optional[HandModel]:
        if use_fake:
            if self._fake_state is None or force or (now - getattr(self, "_last_fake_ts", 0.0)) >= TACTILE_FAKE_INTERVAL_S:
                self._fake_state = self._get_tactile_display_state(state, fake=True)
                self._last_fake_ts = now
            return self._fake_state
        self._fake_state = None
        return self._get_tactile_display_state(state, fake=False)

    def _render_sync(self, state: Optional[HandModel]) -> None:
        axis = int(getattr(self, "_tactile_axis_selected", 2))
        mat = self._build_tactile_spatial_matrix(state, axis)
        tactile_pos = mat[np.isfinite(mat) & (mat >= 0.0)]
        has_data = bool(state and getattr(state, "tactile_data", None) and tactile_pos.size > 0 and (np.max(tactile_pos) > 1e-9))
        self._token += 1
        self._last_applied_token = self._token
        self._apply_tactile_matrix_widget(mat, axis, has_data)

    def _apply_ready_frame(self) -> None:
        result = self._worker.consume_latest()
        if result is None:
            return
        axis = int(getattr(self, "_tactile_axis_selected", 2))
        if result.axis != axis or result.token <= self._last_applied_token:
            return
        self._last_applied_token = result.token
        self._apply_tactile_matrix_widget(result.mat, result.axis, result.has_data)
