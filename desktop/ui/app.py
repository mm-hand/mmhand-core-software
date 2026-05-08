# ui_main.py - 灵巧手图形界面（含启动时的端口/模式选择）

import csv
import os
import json
from datetime import datetime
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap, Normalize
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from typing import Callable, Dict, List, Optional, Tuple
import threading
import time

from protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
)

# Motor Control 使用单圈原始位置计数。
MOTOR_TARGET_SCALE_LO = 0
MOTOR_TARGET_SCALE_HI = 4095
MOTOR_SINGLE_TURN_STEPS = 4096
MOTOR_JUMP_RESYNC_THRESH = 2048
MOTOR_DIR_HINT_CLEAR_TOL = 3
MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES = 4
START_INDICATOR_RUN_BG = "#1e8e3e"
START_INDICATOR_STOP_BG = "#c62828"
START_INDICATOR_FG = "#ffffff"
TEST_ROW_COUNT = 5
TEST_TICK_MS = 10
TEST_DEFAULT_FREQ_HZ = 0.20
TEST_DEFAULT_PHASE_DEG = 0.0
TEST_DEFAULT_HOLD_S = 0.0
FF_TEST_DEFAULT_TARGETS_DEG = "20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40"
FF_TEST_DEFAULT_STEP_S = 1.0
TRI_TEST_DEFAULT_FREQ_HZ = 0.20
CAMERA_CAPTURE_SLEEP_S = 0.05
CAMERA_UI_POLL_MS = 50
DISPLAY_POLL_MS = 50
JOINT_RT_MAX_PLOT_JOINTS = 5
# Joint control 目标角范围（度），需与下位机 AngleSolver + JointCalibrationProfile 当前生效限幅保持一致。
# 注意：最小值也按关节独立维护，当前默认全部 0.0，后续可按关节单独调整。
JOINT_TARGET_MIN_DEG_BY_INDEX = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0
]
JOINT_TARGET_MAX_DEG_BY_INDEX = [
    63.98, 73.98, 88.00, 92.99, 64.99, 74.99, 88.00, 92.99,
    83.98, 83.98, 83.98, 83.98, 83.98, 83.98, 83.98, 83.98,
    83.98, 83.98, 83.98, 83.98, 83.98
]
# Motor channel (M0..M21) -> Joint channel (J0..J20).
# Thumb base lateral motion is dual-driven by M16/M17, both mapped to J16.
MOTOR_TO_JOINT_INDEX = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20
]
JOINT_TO_PRIMARY_MOTOR_INDEX = [-1] * ENCODER_COUNT
for _mi, _ji in enumerate(MOTOR_TO_JOINT_INDEX):
    if 0 <= _ji < ENCODER_COUNT and JOINT_TO_PRIMARY_MOTOR_INDEX[_ji] < 0:
        JOINT_TO_PRIMARY_MOTOR_INDEX[_ji] = _mi
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
    TACTILE_BAND_HEIGHTS,
    TACTILE_BAND_SENSOR_INDEX,
    TACTILE_FOUR_FINGER_GAP,
    TACTILE_FOUR_FINGER_PALM_GAP,
    TACTILE_JOINT_BAND_INDICES,
    TACTILE_HEATMAP_STRUCTURE_VALUE,
    TACTILE_HEATMAP_GRAY_RGBA,
    TACTILE_HEATMAP_PALM_BOUNDS,
    TACTILE_HEATMAP_PALM_RGBA,
    tactile_apply_fingertip_round_cap,
    tactile_sensors_only_anatomy_points,
    tactile_sensors_only_finger_chain_polylines,
)
from core_logic import HandController
from comm_layer import list_ports
from modes import Mode, run
from .services import AsyncRunDataLogger as RunDataLogger, LatestStateBuffer, clone_hand_model
from .style import (
    configure_windows_process_dpi,
    configure_windows_root,
    configure_windows_ttk_style,
)
from .windows import DisplayWindowManager
from .context import HandGuiContext
from .pose_view import PoseViewMixin
from .tactile_view import TactileViewMixin
from .camera_view import CameraViewMixin
from .joint_plot_view import JointRealtimePlotMixin
from .control_panels import ControlPanelsMixin
from plot_hand_from_urdf import (
    DISPLAY_TRANSFORM,
    POSE_VIEW_BG,
    UrdfKinematics,
    apply_display_transform,
    apply_mplot3d_box_aspect_fill_widget,
    apply_mplot3d_camera_zoom,
    apply_pose_view_background,
    compute_pose_view_camera_dist,
    draw_skeleton,
    fill_3d_axes_to_figure,
    load_motor_degrees_from_text,
    motor_deg_to_urdf_joint_dict,
)

DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT_DIR = os.path.dirname(DESKTOP_DIR)

# 预设姿态：20 关节角(弧度) -> 21 电机(度)，按 DEFAULT_ANGLE_MAPPING 映射
_OPEN_20_RAD = (
    [0.1, 0.0, 0.0, 0.0], [0.0, 0.2, 0.1, 0.0], [0.0, 0.1, 0.1, 0.0],
    [0.0, 0.1, 0.1, 0.0], [-0.1, 0.1, 0.1, 0.0],
)
_FIST_20_RAD = (
    [0.2, 1.2, 1.0, 0.8], [0.1, 1.5, 1.2, 1.0], [0.0, 1.5, 1.2, 1.0],
    [-0.1, 1.5, 1.2, 1.0], [-0.2, 1.4, 1.1, 0.9],
)
_ANGLE_MAP = [[3, 0, 1, 2], [4, 5, 6, 7], [8, 9, 10, 11], [12, 13, 14, 15], [16, 17, 18, 19]]
NA_MODE_MARKER = "NA_MODE"




configure_windows_process_dpi()






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


def _preset_20rad_to_21deg(preset_20: tuple) -> List[float]:
    """将 20 关节角(弧度)预设转为 21 电机角度(度)"""
    import math
    out = [0.0] * ENCODER_COUNT
    flat = [x for row in preset_20 for x in row]
    for i, indices in enumerate(_ANGLE_MAP):
        for j, midx in enumerate(indices):
            if i * 4 + j < len(flat) and midx < ENCODER_COUNT:
                out[midx] = math.degrees(flat[i * 4 + j])
    return out




class HandGUI(PoseViewMixin, TactileViewMixin, CameraViewMixin, JointRealtimePlotMixin, ControlPanelsMixin):
    """灵巧手图形界面：左侧上为 URDF 手部 3D 姿态，下为触觉矩阵与相机；触觉区固定为仅传感布局。"""

    def __init__(
        self,
        controller: HandController,
        title_suffix: str = "",
        current_port: Optional[str] = None,
        current_mode: Optional[str] = None,
        on_close_extra: Optional[Callable[[], None]] = None,
        action_callback: Optional[Callable[[str], None]] = None,
        pose_generate: bool = False,
        pose_generate_path: Optional[str] = None,
    ):
        self.controller = controller
        self.current_port = current_port or ""
        _mode_map = {"监控": "Monitor", "遥操作": "Teleoperation", "算法": "Algorithm"}
        self.current_mode = _mode_map.get(current_mode or "", current_mode or "")
        self._on_close_extra = on_close_extra
        self._action_callback = action_callback
        self._pose_generate = bool(pose_generate)
        self._pose_generate_path = pose_generate_path or os.path.join(
            DESKTOP_DIR, "pose_fist_21deg.txt"
        )
        self._generated_pose_mtime: Optional[float] = None
        self._generated_pose_angles: Optional[List[float]] = None
        self._pose_static_scene_ready = False
        self._pose_view_dirty = True
        self._pose_link_artists: List[Tuple[str, str, object]] = []
        self._pose_nodes_artist = None

        self.root = tk.Tk()
        configure_windows_root(self.root)
        self.style = configure_windows_ttk_style(self.root)
        title = f"{ENCODER_COUNT}-DOF Dexterous Hand Control System"
        if title_suffix:
            title = f"{title} - {title_suffix}"
        self.root.title(title)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.context = HandGuiContext(self)
        self._display_windows = DisplayWindowManager(self)
        self._display_windows.apply_main_geometry()
        self._state: Optional[HandModel] = None
        self._state_lock = threading.Lock()
        self._state_buffer = LatestStateBuffer()
        self._state_buffer_version = 0
        controller.register_update_callback(self._on_state_update)
        self._last_draw = 0.0
        self._draw_interval = 0.1  # 100ms
        self._display_state_dirty = False
        self._redraw_scheduled = False
        self._table_scroll_active_until = 0.0
        # 0=X, 1=Y, 2=Z；触觉区仅显示一个通道，默认显示 Z
        self._tactile_axis_selected = 2
        self._tactile_fake_mode = True  # False=真实数据/no data，True=显示生成数据
        # 主触觉区为仅传感 inferno 色标（见 _make_tactile_sensors_only_cmap_norm）；子图占位仍用全图热力图色标
        self._camera1_cap = None
        self._camera2_cap = None
        self._camera_frame_left_buf = None
        self._camera_frame_right_buf = None
        self._camera_buf_lock = threading.Lock()
        self._camera_thread = None
        self._camera_stop = threading.Event()
        self._cam_left_image_item = None
        self._cam_left_text_item = None
        self._cam_right_image_item = None
        self._cam_right_text_item = None
        # 串口连接成功后即可实时显示数据；Start/Stop 仅控制电机运行。
        self._device_running = self._is_comm_connected()
        self._joint_run_states: List[bool] = [False] * ENCODER_COUNT
        self._motor_run_states: List[bool] = [False] * MOTOR_COUNT
        self._joint_run_buttons: List[ttk.Button] = []
        self._motor_run_buttons: List[ttk.Button] = []
        self._save_data_var = tk.BooleanVar(value=True)
        self._data_logger = RunDataLogger(flush_every=10)
        self._recording_status_note = ""
        self._tendon_cfg_path = os.path.join(DESKTOP_DIR, "config", "tendon_manual_calib.json")
        self._tendon_cfg = self._load_tendon_manual_cfg()
        self._tendon_release_margin_counts = 0
        self._tendon_release_angle_eps_deg = 0.0
        self._tendon_guard_last_warn_ts = 0.0
        self._tendon_guard_warn_interval_s = 0.5
        self._joint_rt_joint_var = tk.StringVar(value="J00")
        self._joint_rt_joint_index = 0
        self._joint_rt_joint_vars: List[tk.StringVar] = [
            tk.StringVar(value="None") for _ in range(JOINT_RT_MAX_PLOT_JOINTS)
        ]
        self._joint_rt_window_sec = 12.0
        self._joint_rt_plot_interval = 0.05
        self._joint_rt_last_plot_ts = 0.0
        self._joint_rt_t0 = 0.0
        self._joint_rt_t0_map: Dict[int, float] = {}
        self._joint_rt_series_t: Dict[int, deque] = {}
        self._joint_rt_series_actual: Dict[int, deque] = {}
        self._joint_rt_series_target: Dict[int, deque] = {}
        self._joint_rt_fig = None
        self._joint_rt_ax = None
        self._joint_rt_canvas = None
        self._joint_rt_line_target = None
        self._joint_rt_line_actual = None
        self._joint_rt_window = None
        self._test_running = False
        self._test_after_job = None
        self._test_t0 = 0.0
        self._test_tick_ms = TEST_TICK_MS
        self._test_hold_s = float(TEST_DEFAULT_HOLD_S)
        self._test_hold_s_var = tk.StringVar(value=f"{TEST_DEFAULT_HOLD_S:.2f}")
        self._test_joint_vars: List[tk.StringVar] = []
        self._test_min_vars: List[tk.StringVar] = []
        self._test_max_vars: List[tk.StringVar] = []
        self._test_freq_vars: List[tk.StringVar] = []
        self._test_phase_vars: List[tk.StringVar] = []
        self._test_active_rows: List[Tuple[int, float, float, float, float]] = []
        self._ff_test_running = False
        self._ff_test_after_job = None
        self._ff_test_joint_var = tk.StringVar(value="None")
        self._ff_test_targets_var = tk.StringVar(value=FF_TEST_DEFAULT_TARGETS_DEG)
        self._ff_test_step_s_var = tk.StringVar(value=f"{FF_TEST_DEFAULT_STEP_S:.2f}")
        self._ff_test_joint_idx = 0
        self._ff_test_targets: List[float] = []
        self._ff_test_step_s = float(FF_TEST_DEFAULT_STEP_S)
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = 0.0
        self._tri_test_running = False
        self._tri_test_after_job = None
        self._tri_test_joint_var = tk.StringVar(value="None")
        self._tri_test_min_var = tk.StringVar(value="0.0")
        self._tri_test_max_var = tk.StringVar(value="10.0")
        self._tri_test_freq_var = tk.StringVar(value=f"{TRI_TEST_DEFAULT_FREQ_HZ:.2f}")
        self._tri_test_joint_idx = 0
        self._tri_test_min_deg = 0.0
        self._tri_test_max_deg = 0.0
        self._tri_test_freq_hz = float(TRI_TEST_DEFAULT_FREQ_HZ)
        self._tri_test_t0 = 0.0
        self._local_test_running = False

        self._setup_ui()
        self._setup_device_menu()
        self._bind_keyboard_shortcuts()
        self._start_camera_poll()
        self._start_display_poll()
        self.root.after(200, self._display_windows.open_pose)
        if self._is_comm_connected():
            self._on_comm_ready()
            self.root.after(150, self._maybe_redraw)
        else:
            self.root.after(150, self._clear_live_displays)

    def _setup_device_menu(self) -> None:
        """Top menu bar: Device (Port/Camera/Teleop Device) and Mode (Monitor/Teleoperation/Algorithm)"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        device_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Device", menu=device_menu)

        port_sub = tk.Menu(device_menu, tearoff=0)
        device_menu.add_cascade(label="Port", menu=port_sub)
        self._port_menu = port_sub
        self._refresh_port_menu()

        self._camera1_var = tk.StringVar(value="None")
        self._camera2_var = tk.StringVar(value="None")
        self._camera1_menu = tk.Menu(device_menu, tearoff=0)
        self._camera2_menu = tk.Menu(device_menu, tearoff=0)
        device_menu.add_command(label="Teleop Device", command=lambda: self._emit_action("Menu [Teleop Device]"))
        device_menu.add_separator()
        device_menu.add_command(label="Refresh", command=self._on_device_refresh)
        self._refresh_camera_menus()

        mode_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Mode", menu=mode_menu)
        for mode_name in ["Monitor", "Teleoperation", "Algorithm"]:
            mode_menu.add_command(
                label=mode_name,
                command=lambda m=mode_name: self._on_mode_menu_selected(m),
            )
        if hasattr(self, "_display_windows"):
            self._display_windows.attach_menu(menubar)

    def _refresh_port_menu(self) -> None:
        """Refresh Port submenu with current available ports (uses existing _ports_cache)"""
        if not hasattr(self, "_port_menu"):
            return
        self._port_menu.delete(0, tk.END)
        for d, desc in self._ports_cache:
            label = f"{d} ({desc})"
            self._port_menu.add_command(
                label=label,
                command=lambda dev=d, lbl=label: self._on_port_menu_selected(dev, lbl),
            )
        if not self._ports_cache:
            self._port_menu.add_command(label="No Connection", state=tk.DISABLED)

    def _on_port_menu_selected(self, device: str, label: str) -> None:
        self.port_select_var.set(label)
        self._emit_action(f"Port selected: {device}")
        self._apply_port_mode_in_main()

    def _on_device_refresh(self) -> None:
        """Refresh all connected devices (ports and cameras)"""
        self._refresh_port_options_main()
        self._refresh_camera_menus()
        self._emit_action("Device [Refresh]")

    def _on_mode_menu_selected(self, mode_name: str) -> None:
        if messagebox.askyesno("Confirm Mode Change", f"Switch to {mode_name} mode?"):
            self._switch_mode_in_place(mode_name)







    def _start_display_poll(self) -> None:
        """串口已连接时定期刷新；断开时清屏。与电机 Start/Stop 解耦。"""
        def poll() -> None:
            try:
                if not self._is_comm_connected():
                    if getattr(self, "_device_running", False):
                        self._device_running = False
                        self._clear_live_displays()
                    if hasattr(self, "_display_windows"):
                        with self._state_lock:
                            preview_state = self._state
                        if preview_state is None:
                            preview_state = HandModel()
                        self._display_windows.update_views(preview_state)
                    self.root.after(DISPLAY_POLL_MS, poll)
                    return
                self._device_running = True
                latest, version = self._state_buffer.consume_latest(self._state_buffer_version)
                has_new_state = latest is not None
                if latest is not None:
                    self._state_buffer_version = version
                    with self._state_lock:
                        self._state = latest
                    self._display_state_dirty = True
                with self._state_lock:
                    has_state = self._state is not None
                    state_for_log = self._state
                need_redraw = (
                    bool(getattr(self, "_display_state_dirty", False))
                    or not has_state
                    or getattr(self, "_tactile_fake_mode", False)
                )
                if need_redraw and self._maybe_redraw():
                    self._display_state_dirty = False
                if state_for_log is not None and has_new_state:
                    self._write_run_data_row(state_for_log)
                self.root.after(DISPLAY_POLL_MS, poll)
            except tk.TclError:
                pass
        self.root.after(DISPLAY_POLL_MS, poll)

    def _bind_keyboard_shortcuts(self) -> None:
        self.root.bind_all("<KeyPress-space>", self._on_keyboard_stop)

    def _on_keyboard_stop(self, _event=None) -> None:
        self._on_btn_stop()

    def _is_comm_connected(self) -> bool:
        comm = getattr(self.controller, "comm", None)
        return bool(
            comm
            and getattr(comm, "serial", None)
            and getattr(comm.serial, "is_open", False)
        )

    def _refresh_start_indicator(self) -> None:
        indicator = getattr(self, "_start_indicator_lbl", None)
        if indicator is None:
            return
        started = bool(self._is_comm_connected() and self.controller.is_started())
        text = "RUN" if started else "STOP"
        bg = START_INDICATOR_RUN_BG if started else START_INDICATOR_STOP_BG
        try:
            indicator.configure(text=text, bg=bg, fg=START_INDICATOR_FG)
        except Exception:
            pass


    def _on_comm_ready(self) -> None:
        """串口打开后重置一次性同步标志，待下位机下一帧编码器/舵机数据再填充 Target。"""
        if hasattr(self, "_motor_target_initialized"):
            self._motor_target_initialized = False
        if hasattr(self, "_motor_abs_pos"):
            self._motor_abs_pos = [None] * MOTOR_COUNT
            self._motor_session_origin_abs = [None] * MOTOR_COUNT
            self._motor_dir_hint = [0] * MOTOR_COUNT
            self._motor_dir_hint_pending_clear = [False] * MOTOR_COUNT
            self._motor_dir_hint_stable_count = [0] * MOTOR_COUNT
            self._motor_last_slider = [None] * MOTOR_COUNT
            self._motor_slider_dragging = [False] * MOTOR_COUNT
            self._motor_first_drag_limit_active = [True] * MOTOR_COUNT
        if hasattr(self, "_motor_last_sent_abs"):
            self._motor_last_sent_abs = [None] * MOTOR_COUNT
        if hasattr(self, "_joint_target_initialized"):
            self._joint_target_initialized = False
        self._refresh_start_indicator()
        if hasattr(self, "_save_data_var") and bool(self._save_data_var.get()) and not self._data_logger.is_running():
            try:
                self._start_data_recording()
            except Exception:
                self._save_data_var.set(False)
                self._recording_status_note = ""
        try:
            self._schedule_redraw()
        except tk.TclError:
            pass

















    def _emit_action(self, msg: str) -> None:
        """在按钮响应中上报操作（如监控模式下直接打印）"""
        if self._action_callback:
            try:
                self._action_callback(msg)
            except Exception:
                pass

    def _run_data_root_dir(self) -> str:
        return os.path.join(PROJECT_ROOT_DIR, "run_data")

    def _build_run_data_csv_path(self) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._run_data_root_dir(), f"hand_run_{ts}.csv")












    def _build_run_data_fieldnames(self) -> List[str]:
        fields: List[str] = ["ts_wall", "ts_model", "control_mode"]
        fields.extend([f"joint_curr_deg_j{i:02d}" for i in range(ENCODER_COUNT)])
        fields.extend([f"joint_target_deg_j{i:02d}" for i in range(ENCODER_COUNT)])
        fields.extend([f"joint_debug_ts_ms_j{i:02d}" for i in range(ENCODER_COUNT)])
        fields.extend([f"joint_debug_target_deg_j{i:02d}" for i in range(ENCODER_COUNT)])
        fields.extend([f"joint_debug_actual_deg_j{i:02d}" for i in range(ENCODER_COUNT)])
        fields.extend([f"servo_curr_single_m{i:02d}" for i in range(MOTOR_COUNT)])
        fields.extend([f"servo_target_single_m{i:02d}" for i in range(MOTOR_COUNT)])
        fields.extend([f"servo_curr_multi_m{i:02d}" for i in range(MOTOR_COUNT)])
        fields.extend(["has_sensor_data", "has_servo_raw_data", "has_servo_angle_data"])
        return fields

    def _start_data_recording(self) -> None:
        if self._data_logger.is_running():
            return
        path = self._build_run_data_csv_path()
        self._data_logger.start(path, fieldnames=self._build_run_data_fieldnames())
        comm = getattr(self.controller, "comm", None)
        if comm:
            comm.start_joint_debug_packet_log(self._build_joint_debug_packet_csv_path(path))
        self._recording_status_note = f"Recording: {os.path.basename(path)}"
        self._emit_action(f"Data recording started: {path}")
        if hasattr(self, "status_var"):
            self.status_var.set(self._recording_status_note)

    def _stop_data_recording(self, reason: str = "stopped", keep_switch: bool = False) -> None:
        comm = getattr(self.controller, "comm", None)
        if comm:
            comm.stop_joint_debug_packet_log()
        if self._data_logger.is_running():
            current_path = self._data_logger.path
            packet_csv_path = self._build_joint_debug_packet_csv_path(current_path)
            self._data_logger.stop()
            self._save_joint_rt_plot_snapshot(current_path, reason=reason)
            self._save_joint_debug_packet_plot_snapshot(packet_csv_path, reason=reason)
            self._emit_action(f"Data recording {reason}: {current_path}")
        self._recording_status_note = ""
        if hasattr(self, "_save_data_var") and not keep_switch:
            self._save_data_var.set(False)
        if hasattr(self, "status_var"):
            self.status_var.set("Recording stopped")

    def _on_toggle_save_data(self) -> None:
        if self._test_running:
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(True)
            if hasattr(self, "status_var"):
                self.status_var.set("Save Data is controlled by Test mode while Test is running")
            return
        enabled = bool(self._save_data_var.get())
        if enabled:
            try:
                self._start_data_recording()
            except Exception as exc:
                self._save_data_var.set(False)
                self._recording_status_note = ""
                messagebox.showerror("Save Data", f"Failed to start recording: {exc}")
        else:
            self._stop_data_recording()

    def _write_run_data_row(self, s: HandModel) -> None:
        if not self._data_logger.is_running():
            return

        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        if mode == "motor":
            control_mode = "motor"
        elif mode == "test":
            control_mode = "test"
        else:
            control_mode = "joint"
        if control_mode in ("joint", "test"):
            joint_targets: List[object] = list(self._resolved_joint_targets())
            servo_targets_single: List[object] = [NA_MODE_MARKER] * MOTOR_COUNT
        else:
            joint_targets = [NA_MODE_MARKER] * ENCODER_COUNT
            servo_targets_single = [self._read_motor_target_slider_int(i) for i in range(MOTOR_COUNT)]

        row = {
            "ts_wall": f"{time.time():.6f}",
            "ts_model": f"{float(getattr(s, 'timestamp', 0.0)):.6f}",
            "control_mode": control_mode,
        }
        row.update({f"joint_curr_deg_j{i:02d}": float(s.angles[i]) if i < len(s.angles) else 0.0 for i in range(ENCODER_COUNT)})
        row.update({f"joint_target_deg_j{i:02d}": joint_targets[i] if i < len(joint_targets) else NA_MODE_MARKER for i in range(ENCODER_COUNT)})
        row.update({
            f"joint_debug_ts_ms_j{i:02d}": (
                int(s.joint_debug_timestamp_ms[i])
                if i < len(s.joint_debug_timestamp_ms) and int(s.joint_debug_timestamp_ms[i]) > 0
                else NA_MODE_MARKER
            )
            for i in range(ENCODER_COUNT)
        })
        row.update({
            f"joint_debug_target_deg_j{i:02d}": (
                float(s.joint_debug_target_deg[i])
                if i < len(s.joint_debug_target_deg)
                and i < len(s.joint_debug_timestamp_ms)
                and int(s.joint_debug_timestamp_ms[i]) > 0
                else NA_MODE_MARKER
            )
            for i in range(ENCODER_COUNT)
        })
        row.update({
            f"joint_debug_actual_deg_j{i:02d}": (
                float(s.joint_debug_actual_deg[i])
                if i < len(s.joint_debug_actual_deg)
                and i < len(s.joint_debug_timestamp_ms)
                and int(s.joint_debug_timestamp_ms[i]) > 0
                else NA_MODE_MARKER
            )
            for i in range(ENCODER_COUNT)
        })
        row.update({f"servo_curr_single_m{i:02d}": int(s.servo_raw_positions[i]) if i < len(s.servo_raw_positions) else 0 for i in range(MOTOR_COUNT)})
        row.update({f"servo_target_single_m{i:02d}": servo_targets_single[i] if i < len(servo_targets_single) else NA_MODE_MARKER for i in range(MOTOR_COUNT)})
        row.update({f"servo_curr_multi_m{i:02d}": int(s.servo_angles[i]) if i < len(s.servo_angles) else 0 for i in range(MOTOR_COUNT)})
        row["has_sensor_data"] = int(bool(getattr(s, "has_sensor_data", False)))
        row["has_servo_raw_data"] = int(bool(getattr(s, "has_servo_raw_data", False)))
        row["has_servo_angle_data"] = int(bool(getattr(s, "has_servo_angle_data", False)))
        self._data_logger.write_row(row)

    def _on_close(self):
        self._on_stop_test(reason="stopped on close", update_status=False)
        self._on_stop_ff_test(reason="stopped on close", update_status=False)
        self._on_stop_tri_test(reason="stopped on close", update_status=False)
        self._on_stop_local_test(reason="stopped on close", update_status=False)
        self._stop_data_recording(reason="stopped on close", keep_switch=True)
        self._destroy_joint_rt_plot()
        if hasattr(self, "_display_windows"):
            self._display_windows.close_all()
        self._cancel_pose_3d_rezoom_timer()
        self._stop_camera_capture()
        try:
            if self._on_state_update in self.controller.update_callbacks:
                self.controller.update_callbacks.remove(self._on_state_update)
        except Exception:
            pass
        if self._on_close_extra:
            try:
                self._on_close_extra()
            except Exception:
                pass
        self.controller.shutdown()
        self.root.destroy()

    def _refresh_port_options_main(self):
        current_choice = ""
        if hasattr(self, "port_select_var"):
            current_choice = self.port_select_var.get() or ""
        self._ports_cache = list_ports()
        choices = ["No Connection"] + [f"{d} ({desc})" for d, desc in self._ports_cache]
        if hasattr(self, "port_combo_main"):
            self.port_combo_main["values"] = choices
        if hasattr(self, "port_select_var") and choices:
            if current_choice in choices:
                self.port_select_var.set(current_choice)
            else:
                self.port_select_var.set("No Connection")
        if hasattr(self, "_port_menu"):
            self._refresh_port_menu()

    def _on_port_combo_preopen(self, _event=None):
        self._refresh_port_options_main()

    def _parse_mode_str(self, mode_str: str) -> Mode:
        if mode_str in ("遥操作", "Teleoperation"):
            return Mode.TELEOP
        if mode_str in ("算法", "Algorithm"):
            return Mode.ALGORITHM
        return Mode.MONITOR

    def _normalize_mode_name(self, mode_name: str) -> str:
        m = (mode_name or "").strip()
        if m in ("Control", "遥操作", "Teleoperation"):
            return "Teleoperation"
        if m in ("Algorithm", "算法"):
            return "Algorithm"
        return "Monitor"

    def _refresh_mode_button_states(self) -> None:
        btns = getattr(self, "_mode_buttons", {})
        current = self._normalize_mode_name(self.mode_select_var.get())
        for mode_name, btn in btns.items():
            try:
                if mode_name == current:
                    btn.configure(
                        relief=tk.SUNKEN,
                        state="normal",
                        bg="#e0e0e0",
                        fg="blue",
                        activebackground="#e0e0e0",
                        activeforeground="blue",
                    )
                else:
                    btn.configure(
                        relief=tk.RAISED,
                        state="normal",
                        bg="#e0e0e0",
                        fg="black",
                        activebackground="#e0e0e0",
                        activeforeground="black",
                    )
            except Exception:
                pass

    def _switch_mode_in_place(self, mode_name: str) -> None:
        mode_norm = self._normalize_mode_name(mode_name)
        self.mode_select_var.set(mode_norm)
        if self._normalize_mode_name(self.current_mode) == mode_norm:
            self._refresh_mode_button_states()
            self._refresh_start_indicator()
            return
        if mode_norm != "Monitor":
            self._on_stop_test(reason="stopped on global mode switch", update_status=False)
            self._on_stop_ff_test(reason="stopped on global mode switch", update_status=False)
            self._on_stop_tri_test(reason="stopped on global mode switch", update_status=False)
            self._on_stop_local_test(reason="stopped on global mode switch", update_status=False)
        self.current_mode = mode_norm
        self._rebuild_right_panel()
        self._refresh_mode_button_states()
        self._refresh_start_indicator()
        self._emit_action(f"Mode selected: {mode_norm}")
        if hasattr(self, "status_var"):
            self.status_var.set(f"Mode switched to {mode_norm}")

    def _rebuild_right_panel(self) -> None:
        holder = getattr(self, "_right_panel_holder", None)
        if holder is None:
            return
        for child in holder.winfo_children():
            try:
                child.destroy()
            except Exception:
                pass
        if self.current_mode in ("遥操作", "Teleoperation"):
            self._setup_teleop_control_panel(holder)
        elif self.current_mode in ("算法", "Algorithm"):
            self._setup_algorithm_control_panel(holder)
        else:
            self._setup_control_panel(holder)

    def _ensure_state_callback_registered(self) -> None:
        try:
            if self._on_state_update not in self.controller.update_callbacks:
                self.controller.register_update_callback(self._on_state_update)
        except Exception:
            pass

    def _apply_port_mode_in_main(self):
        """在主窗口直接应用端口（不重绘整窗）。模式切换仅替换右侧面板。"""
        mode_str = self.mode_select_var.get() or "Monitor"
        mode_norm = self._normalize_mode_name(mode_str)

        port = None
        choice = self.port_select_var.get()
        if choice and choice != "No Connection":
            for d, desc in self._ports_cache:
                if choice.startswith(d):
                    port = d
                    break
            if not port and self._ports_cache:
                port = self._ports_cache[0][0]

        old_port = self.current_port if self.current_port else None
        old_mode_norm = self._normalize_mode_name(self.current_mode or "Monitor")
        if old_port == port and old_mode_norm == mode_norm:
            return

        if old_port != port:
            self.controller.shutdown()
            self._ensure_state_callback_registered()
            if port:
                if not self.controller.initialize(port):
                    messagebox.showerror("Connection Failed", f"Cannot open port: {port}")
                    self.current_port = ""
                    self.port_select_var.set("No Connection")
                    return
                self.current_port = port
                self._on_comm_ready()
                if hasattr(self, "status_var"):
                    self.status_var.set(f"Port connected: {port}")
            else:
                self.current_port = ""
                self._clear_live_displays()
                if hasattr(self, "status_var"):
                    self.status_var.set("Disconnected")

        if old_mode_norm != mode_norm:
            self._switch_mode_in_place(mode_norm)

    def _on_state_update(self, state: HandModel):
        if not self._is_comm_connected():
            return
        self._state_buffer.set(clone_hand_model(state))
        self._display_state_dirty = True

    def _schedule_redraw(self) -> None:
        if getattr(self, "_redraw_scheduled", False):
            return
        self._redraw_scheduled = True

        def do() -> None:
            self._redraw_scheduled = False
            self._maybe_redraw()

        try:
            self.root.after_idle(do)
        except tk.TclError:
            self._redraw_scheduled = False

    def _maybe_redraw(self):
        if not self._is_comm_connected():
            return False
        now = time.time()
        if now < float(getattr(self, "_table_scroll_active_until", 0.0)):
            return False
        if now - self._last_draw < self._draw_interval:
            return False
        self._last_draw = now
        self._update_display()
        return True

    def _should_use_realtime_pose(self) -> bool:
        return (not bool(getattr(self, "_pose_generate", False))) and self._is_comm_connected()

    def _draw_hand_model(
        self,
        angles: List[float],
        state: Optional[HandModel],
        *,
        generate: Optional[bool] = None,
    ) -> None:
        comp = getattr(self, "_display_windows", None)
        if comp is None or not comp.is_open("pose"):
            return
        pose_view = comp.components.get("pose")
        if pose_view is None:
            return
        try:
            pose_view.draw_angles(angles, state, generate=generate)
        except tk.TclError:
            comp.close("pose")
        except Exception:
            pass

    def _setup_ui(self):
        # Top bar: port selection + mode switching
        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_select_var = tk.StringVar(value="")
        self.port_combo_main = ttk.Combobox(top, textvariable=self.port_select_var, width=26, state="readonly")
        self.port_combo_main.pack(side=tk.LEFT, padx=4)
        self.port_combo_main.bind("<Button-1>", self._on_port_combo_preopen, add="+")
        self.port_combo_main.bind("<FocusIn>", self._on_port_combo_preopen, add="+")
        self.port_combo_main.bind("<KeyPress-Down>", self._on_port_combo_preopen, add="+")
        self.port_combo_main.bind("<<ComboboxSelected>>", lambda e: self._apply_port_mode_in_main())
        self._camera1_var = tk.StringVar(value="None")
        self._camera2_var = tk.StringVar(value="None")
        self.mode_select_var = tk.StringVar(value=self.current_mode or "Monitor")
        self._ports_cache = []
        self._refresh_port_options_main()
        if self.current_port:
            matched = None
            for d, desc in self._ports_cache:
                if d == self.current_port:
                    matched = f"{d} ({desc})"
                    break
            self.port_select_var.set(matched if matched else self.current_port)
        else:
            self.port_select_var.set("No Connection")
        mode_btn_row = ttk.Frame(top)
        mode_btn_row.pack(side=tk.LEFT, padx=(8, 2))
        self._mode_buttons = {}
        for label, mode_name in [("Manual", "Monitor"), ("Teleop", "Teleoperation"), ("Algorithm", "Algorithm")]:
            btn = tk.Button(
                mode_btn_row,
                text=label,
                width=9,
                cursor="hand2",
                bd=1,
                command=lambda m=mode_name: self._switch_mode_in_place(m),
            )
            btn.pack(side=tk.LEFT, padx=1)
            self._mode_buttons[mode_name] = btn
        self._refresh_mode_button_states()
        self._start_indicator_lbl = tk.Label(
            top,
            text="STOP",
            bg=START_INDICATOR_STOP_BG,
            fg=START_INDICATOR_FG,
            relief=tk.RIDGE,
            bd=1,
            font=("", 9, "bold"),
            width=6,
            padx=6,
            pady=1,
        )
        self._start_indicator_lbl.pack(side=tk.RIGHT, padx=(8, 0))
        self._save_data_switch = ttk.Checkbutton(
            top,
            text="Save Data",
            variable=self._save_data_var,
            command=self._on_toggle_save_data,
        )
        self._save_data_switch.pack(side=tk.RIGHT, padx=(8, 0))
        self._joint_rt_popup_btn = ttk.Button(
            top,
            text="Plot",
            width=10,
            command=self._open_joint_rt_window,
        )
        self._joint_rt_popup_btn.pack(side=tk.RIGHT, padx=(8, 0))
        self._refresh_start_indicator()

        # weight：窗口被拉大时“多出来的宽度”按权重分给各格；不保证首屏分割线就是 2:1
        # 若要与 weight 一致，需在布局完成后设 sash（见 _set_initial_main_sash）
        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)
        self._main_paned = main
        self._main_left_weight = 2
        self._main_right_weight = 3

        # 左侧垂直分割：上 URDF 3D 姿态，下触觉矩阵 + 相机
        left_col = ttk.PanedWindow(main, orient=tk.VERTICAL)
        main.add(left_col, weight=self._main_left_weight)
        self._left_col_paned = left_col

        pose_holder = ttk.Frame(left_col)
        left_col.add(pose_holder, weight=3)
        tactile_cam_holder = ttk.Frame(left_col)
        left_col.add(tactile_cam_holder, weight=2)
        try:
            left_col.pane(pose_holder, minsize=160)
            left_col.pane(tactile_cam_holder, minsize=140)
        except tk.TclError:
            pass

        # Heavy visualizations live in View menu pop-out windows.

        right = ttk.Frame(main)
        main.add(right, weight=self._main_right_weight)
        self._right_panel_holder = right
        try:
            main.forget(left_col)
        except tk.TclError:
            pass
        self._rebuild_right_panel()

        self._setup_status_bar()
        if bool(self._save_data_var.get()) and self._is_comm_connected():
            self._on_toggle_save_data()
        self.root.after(100, self._set_initial_main_sash)
        self.root.after(120, self._set_initial_tactile_camera_sash)

    def _set_initial_main_sash(self) -> None:
        """首次显示后把主区左右分割线设成与 weight 比例一致（ttk PanedWindow 默认常接近 1:1）。"""
        try:
            paned = getattr(self, "_main_paned", None)
            if paned is None:
                return
            self.root.update_idletasks()
            w = int(paned.winfo_width())
            if w <= 10:
                self.root.after(100, self._set_initial_main_sash)
                return
            wl = int(getattr(self, "_main_left_weight", 2))
            wr = int(getattr(self, "_main_right_weight", 1))
            if wl + wr <= 0:
                return
            # 左窗格右边缘像素位置 = 总宽 * wl/(wl+wr)
            paned.sashpos(0, int(w * wl / (wl + wr)))
        except (tk.TclError, AttributeError, ZeroDivisionError):
            pass

    def _set_initial_tactile_camera_sash(self) -> None:
        """首次显示后把左下触觉/摄像头分割线设到中间，避免初始时右侧相机栏被挤没。"""
        try:
            paned = getattr(self, "_tactile_cam_paned", None)
            if paned is None:
                return
            self.root.update_idletasks()
            w = int(paned.winfo_width())
            if w <= 10:
                self.root.after(120, self._set_initial_tactile_camera_sash)
                return
            paned.sashpos(0, int(w / 2))
        except (tk.TclError, AttributeError, ZeroDivisionError):
            pass






































































































    def _get_port_from_main_combo(self) -> Optional[str]:
        """主界面 Port 下拉当前选中的设备名（不含描述），未选或未连接则 None。"""
        if not hasattr(self, "port_select_var"):
            return None
        choice = self.port_select_var.get() or ""
        if not choice or choice == "No Connection":
            return None
        for d, _desc in getattr(self, "_ports_cache", []):
            if choice.startswith(d):
                return d
        return None

    def _clear_live_displays(self) -> None:
        """断开连接或未连接时：清空关节读数、3D 手与触觉；相机单独由 Device 菜单选择，不受此影响。"""
        self._on_stop_test(reason="stopped on disconnect", update_status=False)
        self._on_stop_ff_test(reason="stopped on disconnect", update_status=False)
        self._on_stop_tri_test(reason="stopped on disconnect", update_status=False)
        self._on_stop_local_test(reason="stopped on disconnect", update_status=False)
        self._stop_data_recording(reason="stopped on disconnect", keep_switch=True)
        self._refresh_start_indicator()
        with self._state_lock:
            self._state = None
        self._joint_target_initialized = False
        self._motor_target_initialized = False
        if hasattr(self, "_motor_abs_pos"):
            self._motor_abs_pos = [None] * MOTOR_COUNT
            self._motor_session_origin_abs = [None] * MOTOR_COUNT
            self._motor_dir_hint = [0] * MOTOR_COUNT
            self._motor_dir_hint_pending_clear = [False] * MOTOR_COUNT
            self._motor_dir_hint_stable_count = [0] * MOTOR_COUNT
            self._motor_last_slider = [None] * MOTOR_COUNT
            self._motor_slider_dragging = [False] * MOTOR_COUNT
            self._motor_first_drag_limit_active = [True] * MOTOR_COUNT
        if hasattr(self, "_motor_last_sent_abs"):
            self._motor_last_sent_abs = [None] * MOTOR_COUNT
        self._joint_target_user_set = [False] * len(getattr(self, "_joint_target_vars", []))
        self._motor_target_user_set = [False] * len(getattr(self, "_motor_target_vars", []))
        self._clear_run_states()
        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)
        neutral = [0.0] * ENCODER_COUNT
        if hasattr(self, "_joint_curr_vars"):
            for v in self._joint_curr_vars:
                v.set("-")
        if hasattr(self, "_joint_status_dots"):
            for i in range(min(ENCODER_COUNT, len(self._joint_status_dots))):
                self._set_joint_encoder_dot(i, None)
        if hasattr(self, "_motor_curr_vars"):
            for v in self._motor_curr_vars:
                v.set("-")
        if hasattr(self, "_motor_joint_angle_vars"):
            for v in self._motor_joint_angle_vars:
                v.set("-")
        if (
            hasattr(self, "ax")
            and hasattr(self, "_display_windows")
            and self._display_windows.is_open("pose")
        ):
            self._draw_hand_model(neutral, None)
        if (
            hasattr(self, "_tactile_subplot_imgs")
            and hasattr(self, "_display_windows")
            and self._display_windows.is_open("tactile")
        ):
            empty = HandModel()
            tactile_state = self._get_tactile_display_state(
                empty, fake=self._should_use_fake_tactile(None)
            )
            self._update_tactile_matrix_widget(tactile_state)
        if hasattr(self, "status_var"):
            self.status_var.set("已停止：按 Start 连接设备并显示手部/触觉（相机在 Device 中单独选）")
        if hasattr(self, "_delay_var"):
            self._delay_var.set("")
        if hasattr(self, "_overload_var"):
            self._overload_var.set("伺服过载: —")
        if hasattr(self, "_overload_lbl"):
            self._overload_lbl.config(fg="gray35")
        if hasattr(self, "_release_fault_var"):
            self._release_fault_var.set("反绕保护: 无")
        if hasattr(self, "_release_fault_lbl"):
            self._release_fault_lbl.config(fg="gray35")












    def _setup_status_bar(self):
        bar = ttk.Frame(self.root)
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        comm = getattr(self.controller, "comm", None)
        connected = bool(
            comm
            and getattr(comm, "serial", None)
            and getattr(comm.serial, "is_open", False)
        )
        self.status_var = tk.StringVar(value="Disconnected" if not connected else "Ready")
        ttk.Label(bar, textvariable=self.status_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._delay_var = tk.StringVar()
        self._overload_var = tk.StringVar(value="伺服过载: —")
        self._overload_lbl = tk.Label(bar, textvariable=self._overload_var, fg="gray35", font=("", 9))
        self._overload_lbl.pack(side=tk.RIGHT, padx=(0, 10))
        self._release_fault_var = tk.StringVar(value="反绕保护: 无")
        self._release_fault_lbl = tk.Label(bar, textvariable=self._release_fault_var, fg="gray35", font=("", 9))
        self._release_fault_lbl.pack(side=tk.RIGHT, padx=(0, 10))
        ttk.Label(bar, textvariable=self._delay_var).pack(side=tk.RIGHT)

    def _update_display(self):
        if not self._is_comm_connected():
            return
        with self._state_lock:
            s = self._state
        # 无连接时 _state 为 None，用默认状态；触觉由 _get_tactile_display_state(fake) 决定
        if not s:
            s = HandModel()

        if hasattr(self, "_joint_curr_vars"):
            self._refresh_joint_feedback_view(sync_target=not self._joint_target_initialized)

        if hasattr(self, "_motor_curr_vars"):
            self._refresh_motor_feedback_view(sync_target=not self._motor_target_initialized)

        # 电机模式不以 0x03 角度重算 abs（避免拖一路时邻路 abs 被陈旧角度改掉、整包误触发）。

        if hasattr(self, "_display_windows"):
            self._display_windows.update_views(s)
        else:
            tactile_state = self._get_tactile_display_state(s, fake=self._should_use_fake_tactile(s))
            self._draw_hand_model(s.angles, tactile_state)
            self._update_tactile_matrix_widget(tactile_state)
        self._append_joint_rt_sample(s)
        self._update_joint_rt_plot()
        if self.current_mode in ("算法", "Algorithm") and hasattr(self, "_alg_output"):
            now = time.time()
            if now - getattr(self, "_alg_last_print", 0.0) >= 0.2:
                self._alg_last_print = now
                line = " ".join([f"J{i}:{s.angles[i]:.1f}" for i in range(min(ENCODER_COUNT, len(s.angles)))])
                self._append_algorithm_output(line)

        # 触觉轴按钮始终显示
        if hasattr(self, "_tactile_row") and not self._tactile_row.winfo_ismapped():
            self._tactile_row.pack(fill=tk.X, padx=2, pady=2)

        self._update_status_text(s, self.controller.is_paused())

    def _update_status_text(self, s: HandModel, paused: bool):
        err_count = sum(1 for m in s.motors if m.error)
        # 简要统计在线伺服数量
        n_online = 0
        if hasattr(s, "servo_online") and s.servo_online:
            n_online = sum(1 for f in s.servo_online if f)
        n_telem_online = 0
        if hasattr(s, "servo_telem_online") and s.servo_telem_online:
            n_telem_online = sum(1 for f in s.servo_telem_online if f)
        n_debug_valid = 0
        if hasattr(s, "joint_debug_valid") and s.joint_debug_valid:
            n_debug_valid = sum(1 for f in s.joint_debug_valid if f)
        base = (
            f"Encoders: {ENCODER_COUNT} | Errors: {err_count} | Calib: {s.calib_status}"
            f" | Servo Online: {n_online}/{MOTOR_COUNT}"
            f" | Telem Online: {n_telem_online}/{MOTOR_COUNT}"
            f" | Debug Valid: {n_debug_valid}/{ENCODER_COUNT}"
        )
        if paused:
            base = "[Paused] " + base
        if self._data_logger.is_running():
            note = self._recording_status_note or "Recording..."
            base = f"{base} | {note}"
        self.status_var.set(base)
        self._delay_var.set(f"Latency: {(time.time() - s.timestamp)*1000:.0f} ms")
        if hasattr(self, "_release_fault_var") and hasattr(self, "_release_fault_lbl"):
            rfl = [f"J{i}" for i, f in enumerate(getattr(s, "joint_reverse_release_fault", []) or []) if f]
            if rfl:
                self._release_fault_var.set(f"反绕保护: {','.join(rfl)}")
                self._release_fault_lbl.config(fg="red")
            else:
                self._release_fault_var.set("反绕保护: 无")
                self._release_fault_lbl.config(fg="gray35")
        if hasattr(self, "_overload_var") and hasattr(self, "_overload_lbl"):
            fl = [str(i) for i, f in enumerate(getattr(s, "servo_overload_fault", []) or []) if f]
            if fl:
                self._overload_var.set(f"伺服过载: {','.join(fl)}")
                self._overload_lbl.config(fg="red")
            else:
                self._overload_var.set("伺服过载: 无")
                self._overload_lbl.config(fg="gray35")

    def run(self):
        self.root.mainloop()
