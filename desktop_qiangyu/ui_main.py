# ui_main.py - 灵巧手图形界面（含启动时的端口/模式选择）

import os
import json
from datetime import datetime
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox
import tkinter.font as tkfont
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
TEST_TICK_MS = 40
TEST_DEFAULT_FREQ_HZ = 0.20
TEST_DEFAULT_PHASE_DEG = 0.0
TEST_DEFAULT_HOLD_S = 0.0
JOINT_RT_MAX_PLOT_JOINTS = 5
CAMERA_INDEX_SCAN_MAX = 5
CAMERA_POLL_ACTIVE_MS = 50
CAMERA_POLL_IDLE_MS = 220
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
from run_data_logger import RunDataLogger
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


def _normalized_mousewheel_steps(event) -> int:
    """Normalize wheel events across Windows/macOS/Linux."""
    delta = int(getattr(event, "delta", 0) or 0)
    if delta != 0:
        if abs(delta) < 120:
            return -1 if delta > 0 else 1
        return int(-delta / 120)
    num = int(getattr(event, "num", 0) or 0)
    if num == 4:
        return -1
    if num == 5:
        return 1
    return 0


def _apply_platform_ui_defaults(root: tk.Tk, *, compact: bool = False) -> None:
    """Apply lightweight platform-specific UI tuning (DPI, fonts, theme)."""
    if os.name == "nt":
        try:
            import ctypes
            try:
                ctypes.windll.shcore.SetProcessDpiAwareness(1)
            except Exception:
                ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass
        try:
            scale = float(root.winfo_fpixels("1i")) / 72.0
            if 1.0 <= scale <= 2.5:
                root.tk.call("tk", "scaling", scale)
        except Exception:
            pass
    style = ttk.Style(root)
    try:
        if os.name == "nt" and "vista" in style.theme_names():
            style.theme_use("vista")
        elif "clam" in style.theme_names():
            style.theme_use("clam")
    except Exception:
        pass
    family = "Segoe UI" if os.name == "nt" else "TkDefaultFont"
    size = 9 if compact else 10
    for fname in ("TkDefaultFont", "TkTextFont", "TkMenuFont"):
        try:
            f = tkfont.nametofont(fname)
            f.configure(family=family, size=size)
        except Exception:
            pass
    try:
        f = tkfont.nametofont("TkHeadingFont")
        f.configure(family=family, size=size + 1, weight="bold")
    except Exception:
        pass
    root.option_add("*tearOff", False)
    try:
        root.configure(bg="#f2f4f8")
    except Exception:
        pass
    style.configure("TFrame", background="#f2f4f8")
    style.configure("TLabel", background="#f2f4f8", foreground="#111827")
    style.configure("TLabelframe", background="#f2f4f8")
    style.configure("TLabelframe.Label", background="#f2f4f8", foreground="#1f2937")
    style.configure("TCheckbutton", background="#f2f4f8")
    style.configure("TButton", padding=(8, 3))
    style.configure("TCombobox", padding=(3, 2))


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


class LauncherWindow:
    """启动窗口：端口选择 + 模式选择，确认后创建控制器并进入主界面"""

    def __init__(self):
        self.root = tk.Tk()
        _apply_platform_ui_defaults(self.root, compact=True)
        self.root.title("Dexterous Hand - Connection & Mode")
        self.root.geometry("460x250")
        self.root.minsize(420, 230)
        self.root.resizable(True, False)

        self.port_var = tk.StringVar(value="")
        self.mode_var = tk.StringVar(value="Monitor")
        self.ports: List[tuple] = []

        self._build()

    def _build(self):
        f = ttk.Frame(self.root, padding=12)
        f.pack(fill=tk.BOTH, expand=True)

        # 端口
        row = ttk.Frame(f)
        row.pack(fill=tk.X, pady=4)
        ttk.Label(row, text="Port:", width=8).pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, width=36, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="Refresh", width=8, command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Label(f, text='(Select "No Connection" to open UI only)', font=("", 9)).pack(anchor=tk.W)

        # 模式（下拉选择）
        row2 = ttk.Frame(f)
        row2.pack(fill=tk.X, pady=8)
        ttk.Label(row2, text="Mode:", width=8).pack(side=tk.LEFT)
        self.mode_combo = ttk.Combobox(
            row2,
            textvariable=self.mode_var,
            width=36,
            state="readonly",
            values=["Monitor", "Teleoperation", "Algorithm"],
        )
        self.mode_combo.pack(side=tk.LEFT, padx=4)
        self.mode_var.set("Monitor")
        ttk.Label(f, text="Algorithm mode: implement logic in example_algorithm.py run(controller)", font=("", 9)).pack(anchor=tk.W)

        # 按钮
        btn_row = ttk.Frame(f)
        btn_row.pack(fill=tk.X, pady=16)
        ttk.Button(btn_row, text="Connect & Start", command=self._on_launch).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Exit", command=self._on_quit).pack(side=tk.LEFT)

        self._refresh_ports()

    def _on_quit(self):
        """退出按钮：销毁窗口并结束 mainloop，使整个程序正常退出"""
        self.root.destroy()
        self.root.quit()

    def _refresh_ports(self):
        self.ports = list_ports()
        choices = ["No Connection"] + [f"{d} ({desc})" for d, desc in self.ports]
        self.port_combo["values"] = choices
        if choices:
            self.port_var.set(choices[0])

    def _on_launch(self):
        port = None
        choice = self.port_var.get()
        if choice and choice != "No Connection":
            for d, desc in self.ports:
                if choice.startswith(d):
                    port = d
                    break
            if not port and self.ports:
                port = self.ports[0][0]

        mode_str = self.mode_var.get()
        mode = Mode.MONITOR
        if mode_str in ("遥操作", "Teleoperation"):
            mode = Mode.TELEOP
        elif mode_str in ("算法", "Algorithm"):
            mode = Mode.ALGORITHM

        controller = HandController()
        if port:
            if not controller.initialize(port):
                messagebox.showerror("Connection Failed", f"Cannot open port: {port}")
                return
        self.root.destroy()
        run(mode, controller, port=port, gui=True)

    def run(self):
        self.root.mainloop()


class HandGUI:
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
            os.path.dirname(__file__), "pose_fist_21deg.txt"
        )
        self._generated_pose_mtime: Optional[float] = None
        self._generated_pose_angles: Optional[List[float]] = None
        self._pose_static_scene_ready = False
        self._pose_view_dirty = True
        self._pose_link_artists: List[Tuple[str, str, object]] = []
        self._pose_nodes_artist = None

        self.root = tk.Tk()
        _apply_platform_ui_defaults(self.root, compact=False)
        title = f"{ENCODER_COUNT}-DOF Dexterous Hand Control System"
        if title_suffix:
            title = f"{title} - {title_suffix}"
        self.root.title(title)
        self.root.geometry("1260x720")
        self.root.minsize(1100, 640)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._ui_base_width = 1260
        self._ui_base_height = 720
        self._ui_font_base_size = 10
        self._ui_scale_bucket = -1
        self._root_resize_job = None
        self._root_resize_debounce_ms = 260
        self._pose_resize_job = None
        self._pose_resize_debounce_ms = 80
        self._pose_resize_pending_size: Optional[Tuple[int, int]] = None

        controller.register_update_callback(self._on_state_update)
        self._state: Optional[HandModel] = None
        self._state_lock = threading.Lock()
        self._last_draw = 0.0
        self._draw_interval = 0.04  # 40ms
        self._pose_draw_interval = 0.06  # 60ms
        self._tactile_draw_interval = 0.08  # 80ms
        self._feedback_draw_interval = 0.06  # 60ms
        self._last_feedback_draw = 0.0
        self._last_pose_draw = 0.0
        self._last_tactile_draw = 0.0
        self._redraw_pending = False
        self._redraw_job = None
        self._pose_3d_enabled_var = tk.BooleanVar(value=False)
        self._camera_chain_enabled_var = tk.BooleanVar(value=False)
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
        self._camera_frame_seq = 0
        self._camera_last_painted_seq = -1
        self._camera_last_blank_drawn = False
        self._camera_scan_lock = threading.Lock()
        self._camera_scan_in_progress = False
        self._camera_scan_pending = False
        self._camera_indices_cache: List[int] = []
        self._camera_scan_last_ts = 0.0
        self._camera_scan_ttl_s = 4.0
        # 串口连接成功后即可实时显示数据；Start/Stop 仅控制电机运行。
        self._device_running = self._is_comm_connected()
        self._joint_run_states: List[bool] = [False] * ENCODER_COUNT
        self._motor_run_states: List[bool] = [False] * MOTOR_COUNT
        self._joint_run_buttons: List[ttk.Button] = []
        self._motor_run_buttons: List[ttk.Button] = []
        self._save_data_var = tk.BooleanVar(value=False)
        self._csv_recording_enabled = False
        self._data_logger = RunDataLogger(flush_every=10)
        self._recording_status_note = ""
        self._tendon_cfg_path = os.path.join(os.path.dirname(__file__), "config", "tendon_manual_calib.json")
        self._tendon_cfg = self._load_tendon_manual_cfg()
        self._tendon_release_margin_counts = 0
        self._tendon_release_angle_eps_deg = 0.0
        self._tendon_guard_last_warn_ts = 0.0
        self._tendon_guard_warn_interval_s = 0.5
        self._last_tendon_guard_sent: Optional[Tuple[Tuple[int, int, int], ...]] = None
        self._joint_rt_joint_var = tk.StringVar(value="J00")
        self._joint_rt_joint_index = 0
        self._joint_rt_joint_vars: List[tk.StringVar] = [
            tk.StringVar(value="None") for _ in range(JOINT_RT_MAX_PLOT_JOINTS)
        ]
        self._joint_rt_window_sec = 12.0
        self._joint_rt_plot_interval = 0.2
        self._joint_rt_last_plot_ts = 0.0
        self._joint_rt_t0 = 0.0
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

        self._setup_ui()
        self._setup_device_menu()
        self._bind_keyboard_shortcuts()
        if os.name != "nt":
            self.root.bind("<Configure>", self._on_root_resized, add="+")
        self.root.after(100, self._apply_responsive_ui_scale)
        self._start_camera_poll()
        self._start_display_poll()
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

        if not hasattr(self, "_camera1_var"):
            self._camera1_var = tk.StringVar(value="None")
        if not hasattr(self, "_camera2_var"):
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

    def _open_cv_capture(self, cv2_mod, index: int):
        """OpenCV camera open helper with a Windows-friendly backend."""
        if os.name == "nt":
            cap_dshow = getattr(cv2_mod, "CAP_DSHOW", None)
            if cap_dshow is not None:
                try:
                    return cv2_mod.VideoCapture(index, cap_dshow)
                except Exception:
                    pass
        return cv2_mod.VideoCapture(index)

    def _list_available_cameras(self) -> List[int]:
        """Probe indices 0-4 for available cameras (avoids OpenCV out-of-bound on some systems)."""
        try:
            import cv2
        except ImportError:
            return []
        available = []
        for idx in range(CAMERA_INDEX_SCAN_MAX):
            cap = None
            try:
                cap = self._open_cv_capture(cv2, idx)
                if cap.isOpened():
                    ret = False
                    for _ in range(2):
                        ret, _ = cap.read()
                        if ret:
                            break
                    if ret:
                        available.append(idx)
            except Exception:
                pass
            finally:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
        return available

    def _request_camera_scan(self, force: bool = False) -> None:
        if not self._is_camera_chain_enabled():
            self._camera_indices_cache = []
            self._update_camera_combo_values()
            return
        if not force and self._camera_indices_cache and (time.time() - self._camera_scan_last_ts) < self._camera_scan_ttl_s:
            self._update_camera_combo_values()
            return
        with self._camera_scan_lock:
            if self._camera_scan_in_progress:
                self._camera_scan_pending = True
                return
            self._camera_scan_in_progress = True
            self._camera_scan_pending = False

        def worker() -> None:
            indices = self._list_available_cameras()
            try:
                self.root.after(0, lambda vals=indices: self._on_camera_scan_done(vals))
            except tk.TclError:
                pass

        threading.Thread(target=worker, daemon=True).start()

    def _on_camera_scan_done(self, indices: List[int]) -> None:
        self._camera_indices_cache = sorted(set(int(i) for i in indices if i is not None))
        self._camera_scan_last_ts = time.time()
        self._update_camera_combo_values()
        rerun = False
        with self._camera_scan_lock:
            rerun = self._camera_scan_pending
            self._camera_scan_pending = False
            self._camera_scan_in_progress = False
        if rerun:
            self._request_camera_scan(force=True)

    def _refresh_camera_menus(self) -> None:
        """Refresh camera combo dropdown values (Device > Refresh)."""
        self._update_camera_combo_values()
        if not self._is_camera_chain_enabled():
            return
        self._request_camera_scan(force=True)

    def _on_camera_selected(self, index: Optional[int], slot: int) -> None:
        """Slot 1=Left widget, 2=Right widget."""
        label = "None" if index is None else f"Camera {index}"
        if slot == 1:
            self._camera1_var.set(label)
        else:
            self._camera2_var.set(label)
        self._emit_action(f"Camera {slot} selected: {index if index is not None else 'None'}")
        if not self._is_camera_chain_enabled():
            return
        threading.Thread(target=lambda: self._set_camera_capture(slot, index), daemon=True).start()

    def _set_camera_capture(self, slot: int, index: Optional[int]) -> None:
        """Open or release camera for slot (1=Left, 2=Right)."""
        try:
            import cv2
        except ImportError:
            return
        if not self._is_camera_chain_enabled():
            index = None
        self._camera_stop.set()
        if self._camera_thread is not None:
            self._camera_thread.join(timeout=1.0)
            self._camera_thread = None
        self._camera_stop.clear()

        cap_attr = "_camera1_cap" if slot == 1 else "_camera2_cap"
        old_cap = getattr(self, cap_attr, None)
        if old_cap is not None:
            try:
                old_cap.release()
            except Exception:
                pass
            setattr(self, cap_attr, None)
        if slot == 1:
            self._camera_frame_left = None
            with self._camera_buf_lock:
                self._camera_frame_left_buf = None
        else:
            self._camera_frame_right = None
            with self._camera_buf_lock:
                self._camera_frame_right_buf = None
        self._camera_last_blank_drawn = False

        if index is not None:
            try:
                cap = self._open_cv_capture(cv2, index)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    setattr(self, cap_attr, cap)
            except Exception:
                pass

        if self._camera1_cap is not None or self._camera2_cap is not None:
            self._camera_thread = threading.Thread(target=self._camera_capture_loop, daemon=True)
            self._camera_thread.start()

    def _camera_capture_loop(self) -> None:
        """Background thread: read frames from cameras, store in buffers."""
        try:
            import cv2
        except ImportError:
            return
        while True:
            if self._camera_stop.is_set():
                break
            left_frame = None
            right_frame = None
            if self._camera1_cap is not None and self._camera1_cap.isOpened():
                try:
                    ret, frame = self._camera1_cap.read()
                    if ret and frame is not None:
                        left_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    pass
            if self._camera2_cap is not None and self._camera2_cap.isOpened():
                try:
                    ret, frame = self._camera2_cap.read()
                    if ret and frame is not None:
                        right_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    pass
            with self._camera_buf_lock:
                self._camera_frame_left_buf = left_frame
                self._camera_frame_right_buf = right_frame
                self._camera_frame_seq += 1
            time.sleep(0.033)

    def _start_display_poll(self) -> None:
        """串口已连接时定期刷新；断开时清屏。与电机 Start/Stop 解耦。"""
        def poll() -> None:
            try:
                if not self._is_comm_connected():
                    if getattr(self, "_device_running", False):
                        self._device_running = False
                        self._clear_live_displays()
                    self.root.after(200, poll)
                    return
                self._device_running = True
                with self._state_lock:
                    has_state = self._state is not None
                need_redraw = not has_state or getattr(self, "_tactile_fake_mode", False)
                if need_redraw:
                    self._maybe_redraw()
                self.root.after(200, poll)
            except tk.TclError:
                pass
        self.root.after(200, poll)

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

    def _guard_joint_command_started(self) -> bool:
        if self.controller.is_started():
            return True
        if hasattr(self, "status_var"):
            self.status_var.set("Joint command blocked: press Enable first")
        self._refresh_start_indicator()
        return False

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
        self._last_tendon_guard_sent = None
        self._sync_tendon_guard_to_lower(force=True)
        self._refresh_start_indicator()
        try:
            self.root.after(0, self._maybe_redraw)
        except tk.TclError:
            pass

    @staticmethod
    def _motor_target_from_servo_feedback(servo_raw_single: int) -> int:
        """单圈反馈 raw → 滑条 0..4095。"""
        v = int(servo_raw_single) % MOTOR_SINGLE_TURN_STEPS
        return max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, v))

    @staticmethod
    def _clamp_motor_abs_cmd(v: int) -> int:
        return max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(v)))

    @staticmethod
    def _slider_from_abs_pos(abs_pos: int) -> int:
        return int(abs_pos) % MOTOR_SINGLE_TURN_STEPS

    def _read_motor_target_slider_int(self, idx: int) -> int:
        """滑条表示用户目标位，只读不改。"""
        if 0 <= idx < len(self._motor_target_vars):
            try:
                v = int(self._motor_target_vars[idx].get())
                return max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, v))
            except (tk.TclError, TypeError, ValueError):
                pass
        return MOTOR_TARGET_SCALE_LO

    def _session_abs_from_slider_locked_turn(
        self, idx: int, slider_value: int, drag_sign: int = 0
    ) -> Optional[int]:
        """
        将单圈目标(0..4095)映射为会话坐标：
        - 锁定“进入会话时所在的设备物理圈”
        - 优先使用方向提示(由 +/- 设定)
        - 无方向提示时按本次拖动方向
        - 无拖动方向信息时再退化为最近等效圈
        """
        if idx < 0 or idx >= MOTOR_COUNT:
            return None
        if idx >= len(self._motor_session_origin_abs):
            return None
        origin = self._motor_session_origin_abs[idx]
        if origin is None:
            return None
        origin_single = self._slider_from_abs_pos(origin)
        sl = max(MOTOR_TARGET_SCALE_LO, min(MOTOR_TARGET_SCALE_HI, int(slider_value)))
        base = int(sl - origin_single)

        current = None
        if idx < len(self._motor_abs_pos):
            current = self._motor_abs_pos[idx]

        if current is None:
            return self._clamp_motor_abs_cmd(base)

        step = MOTOR_SINGLE_TURN_STEPS
        hint = 0
        if idx < len(self._motor_dir_hint):
            hint = int(self._motor_dir_hint[idx])

        if hint > 0:
            # 正向：选择不小于当前会话位置的最近等效圈。
            k = int((int(current) - base) // step)
            candidate = base + k * step
            if candidate < int(current):
                candidate += step
        elif hint < 0:
            # 反向：选择不大于当前会话位置的最近等效圈。
            k = int((int(current) - base) // step)
            candidate = base + k * step
            if candidate > int(current):
                candidate -= step
        elif drag_sign > 0:
            k = int((int(current) - base) // step)
            candidate = base + k * step
            if candidate < int(current):
                candidate += step
        elif drag_sign < 0:
            k = int((int(current) - base) // step)
            candidate = base + k * step
            if candidate > int(current):
                candidate -= step
        else:
            # 无方向提示：使用最近等效圈。
            k = int(round((int(current) - base) / float(step)))
            candidate = base + k * step

        return self._clamp_motor_abs_cmd(candidate)

    @staticmethod
    def _single_turn_diff(a: int, b: int) -> int:
        diff = abs(int(a) - int(b)) % MOTOR_SINGLE_TURN_STEPS
        return min(diff, MOTOR_SINGLE_TURN_STEPS - diff)

    def _motor_resync_one_channel(self, idx: int) -> None:
        """用下位机反馈对齐本通道会话坐标；不移动滑条（滑条仅表示用户目标）。"""
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        with self._state_lock:
            s = self._state
        if not s:
            return
        user_sl = self._read_motor_target_slider_int(idx)
        if bool(getattr(s, "has_servo_angle_data", False)) and idx < len(s.servo_angles):
            if idx < len(s.servo_online) and s.servo_online[idx]:
                dev_abs = self._clamp_motor_abs_cmd(int(s.servo_angles[idx]))
                if self._motor_session_origin_abs[idx] is None:
                    self._motor_session_origin_abs[idx] = dev_abs
                origin = self._motor_session_origin_abs[idx]
                self._motor_abs_pos[idx] = self._clamp_motor_abs_cmd(dev_abs - origin)
                self._motor_last_slider[idx] = user_sl
                return
        if bool(getattr(s, "has_servo_raw_data", False)) and idx < len(s.servo_raw_positions):
            if idx < len(s.servo_raw_online) and s.servo_raw_online[idx]:
                r = self._motor_target_from_servo_feedback(s.servo_raw_positions[idx])
                dev_abs = self._clamp_motor_abs_cmd(r)
                if self._motor_session_origin_abs[idx] is None:
                    self._motor_session_origin_abs[idx] = dev_abs
                origin = self._motor_session_origin_abs[idx]
                self._motor_abs_pos[idx] = self._clamp_motor_abs_cmd(dev_abs - origin)
                self._motor_last_slider[idx] = user_sl

    def _motor_abs_for_command_packet(
        self, idx: int, s: Optional[HandModel], *, ignore_last_sent: bool = False
    ) -> int:
        """会话坐标下发：device_abs = session_origin_abs + session_abs。"""
        session_abs = self._motor_abs_pos[idx] if idx < len(self._motor_abs_pos) else None
        origin = self._motor_session_origin_abs[idx] if idx < len(self._motor_session_origin_abs) else None

        if (session_abs is None or origin is None) and s is not None:
            if bool(getattr(s, "has_servo_angle_data", False)):
                if idx < len(s.servo_online) and s.servo_online[idx] and idx < len(s.servo_angles):
                    dev_abs = self._clamp_motor_abs_cmd(int(s.servo_angles[idx]))
                    if origin is None:
                        origin = dev_abs
                        self._motor_session_origin_abs[idx] = origin
                    if session_abs is None:
                        session_abs = self._clamp_motor_abs_cmd(dev_abs - origin)
            elif bool(getattr(s, "has_servo_raw_data", False)):
                if idx < len(s.servo_raw_online) and s.servo_raw_online[idx] and idx < len(s.servo_raw_positions):
                    dev_abs = self._clamp_motor_abs_cmd(int(self._motor_target_from_servo_feedback(s.servo_raw_positions[idx])))
                    if origin is None:
                        origin = dev_abs
                        self._motor_session_origin_abs[idx] = origin
                    if session_abs is None:
                        session_abs = self._clamp_motor_abs_cmd(dev_abs - origin)

        if session_abs is not None:
            self._motor_abs_pos[idx] = session_abs
        if origin is not None:
            self._motor_session_origin_abs[idx] = origin
        if session_abs is not None and origin is not None:
            return self._clamp_motor_abs_cmd(origin + session_abs)

        if not ignore_last_sent and idx < len(getattr(self, "_motor_last_sent_abs", [])):
            ls = self._motor_last_sent_abs[idx]
            if ls is not None:
                return self._clamp_motor_abs_cmd(int(ls))
        return 0

    def _build_motor_abs_command_list(self) -> List[int]:
        with self._state_lock:
            s = self._state
        return [self._motor_abs_for_command_packet(i, s) for i in range(MOTOR_COUNT)]

    def _seed_motor_last_sent_after_sync(self, s: HandModel) -> None:
        """首帧对齐后缓存「等价于当前状态」的各路上次命令，避免第一次拖条时邻路 abs 空而误发 0。"""
        if not hasattr(self, "_motor_last_sent_abs"):
            return
        for i in range(MOTOR_COUNT):
            self._motor_last_sent_abs[i] = self._motor_abs_for_command_packet(i, s, ignore_last_sent=True)

    def _send_motor_abs_from_ui(self, trigger: str) -> None:
        if not self._interaction_enabled:
            return
        if not self._is_comm_connected():
            raise RuntimeError("Serial port is not connected")
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        self.controller.set_pid_control(False)
        if not self.controller.is_started():
            self.controller.start()
            self._refresh_start_indicator()
        with self._state_lock:
            s = self._state
        if s is None or not (
            bool(getattr(s, "has_servo_angle_data", False))
            or bool(getattr(s, "has_servo_raw_data", False))
        ):
            if hasattr(self, "status_var"):
                self.status_var.set("Motor command blocked: waiting for motor feedback")
            return
        if all(v is None for v in self._motor_last_sent_abs):
            self._seed_motor_last_sent_after_sync(s)
        if s is not None:
            for i in range(MOTOR_COUNT):
                if self._motor_abs_pos[i] is None:
                    self._motor_resync_one_channel(i)
        targets = self._build_motor_abs_command_list()
        self.controller.set_motor_positions_absolute(targets)
        for i in range(min(MOTOR_COUNT, len(self._motor_last_sent_abs))):
            self._motor_last_sent_abs[i] = targets[i]
        if hasattr(self, "status_var"):
            self.status_var.set(f"Motor absolute positions sent ({trigger})")

    def _sync_motor_abs_from_slider_before_send(self, idx: int) -> None:
        """Send / Send All 前：锁定当前会话圈，abs = base_turn*4096 + slider。"""
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        if self._motor_abs_pos[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            return
        cur = int(self._motor_target_vars[idx].get())
        mapped = self._session_abs_from_slider_locked_turn(idx, cur, 0)
        if mapped is None:
            return
        self._motor_abs_pos[idx] = mapped
        self._motor_last_slider[idx] = cur

    def _on_motor_slider_press(self, idx: int, _event=None) -> None:
        if 0 <= idx < len(self._motor_slider_dragging):
            self._motor_slider_dragging[idx] = True

    def _on_motor_slider_release(self, idx: int, _event=None) -> None:
        if 0 <= idx < len(self._motor_slider_dragging):
            self._motor_slider_dragging[idx] = False

    def _should_use_realtime_pose(self) -> bool:
        # Runtime policy:
        # - No port/connection: use txt pose.
        # - Connected: use realtime angles.
        return self._is_comm_connected()

    def _is_pose_3d_enabled(self) -> bool:
        var = getattr(self, "_pose_3d_enabled_var", None)
        return True if var is None else bool(var.get())

    def _is_camera_chain_enabled(self) -> bool:
        var = getattr(self, "_camera_chain_enabled_var", None)
        return True if var is None else bool(var.get())

    @staticmethod
    def _camera_index_from_label(label: str) -> Optional[int]:
        v = (label or "").strip()
        if v.startswith("Camera "):
            try:
                return int(v.replace("Camera ", "", 1))
            except ValueError:
                return None
        return None

    def _selected_camera_index(self, slot: int) -> Optional[int]:
        var = self._camera1_var if slot == 1 else self._camera2_var
        return self._camera_index_from_label(var.get())

    def _set_camera_widget_interaction(self, enabled: bool) -> None:
        combo_state = "readonly" if enabled else "disabled"
        for name in ("_cam_left_combo", "_cam_right_combo"):
            combo = getattr(self, name, None)
            if combo is None:
                continue
            try:
                combo.configure(state=combo_state)
            except Exception:
                pass

    def _apply_camera_chain_state(self, enabled: bool) -> None:
        self._set_camera_widget_interaction(enabled)
        if not enabled:
            self._camera_indices_cache = []
            self._update_camera_combo_values()

        def worker() -> None:
            if not enabled:
                self._set_camera_capture(1, None)
                self._set_camera_capture(2, None)
                return
            self._set_camera_capture(1, self._selected_camera_index(1))
            self._set_camera_capture(2, self._selected_camera_index(2))

        threading.Thread(target=worker, daemon=True).start()
        if not enabled:
            self._camera_last_painted_seq = -1
            self._camera_last_blank_drawn = False
        else:
            self._camera_last_blank_drawn = False

    def _on_toggle_pose_3d(self) -> None:
        enabled = self._is_pose_3d_enabled()
        frame = getattr(self, "_pose_frame", None)
        if frame is not None:
            try:
                visible = bool(frame.winfo_ismapped())
            except Exception:
                visible = False
            if enabled and not visible:
                frame.pack(fill=tk.BOTH, expand=True, pady=(0, 2))
            elif (not enabled) and visible:
                frame.pack_forget()
        if not enabled:
            self._cancel_pose_3d_rezoom_timer()
        if enabled:
            self._request_redraw()

    def _on_toggle_camera_chain(self) -> None:
        enabled = self._is_camera_chain_enabled()
        self._apply_camera_chain_state(enabled)
        if enabled:
            self._request_camera_scan(force=True)

    def _start_camera_poll(self) -> None:
        """Main-thread poll: copy buffers to display (non-blocking)."""

        def poll() -> None:
            try:
                if not self._is_camera_chain_enabled():
                    next_ms = CAMERA_POLL_IDLE_MS
                    if hasattr(self, "_cam_left_canvas") and not self._camera_last_blank_drawn:
                        self._camera_frame_left = None
                        self._camera_frame_right = None
                        self._camera_last_blank_drawn = True
                        self._update_camera_widget()
                    self.root.after(next_ms, poll)
                    return
                with self._camera_buf_lock:
                    left = self._camera_frame_left_buf
                    right = self._camera_frame_right_buf
                    frame_seq = int(self._camera_frame_seq)
                self._camera_frame_left = left
                self._camera_frame_right = right
                has_live_camera = (self._camera1_cap is not None) or (self._camera2_cap is not None)
                has_frame = (left is not None) or (right is not None)
                if hasattr(self, "_cam_left_canvas"):
                    should_redraw = frame_seq != self._camera_last_painted_seq
                    if not should_redraw and (not has_live_camera) and (not has_frame) and (not self._camera_last_blank_drawn):
                        should_redraw = True
                    if should_redraw:
                        self._camera_last_painted_seq = frame_seq
                        self._camera_last_blank_drawn = (not has_live_camera) and (not has_frame)
                        self._update_camera_widget()
                next_ms = CAMERA_POLL_ACTIVE_MS if (has_live_camera or has_frame) else CAMERA_POLL_IDLE_MS
            except Exception:
                next_ms = CAMERA_POLL_IDLE_MS
            try:
                self.root.after(next_ms, poll)
            except tk.TclError:
                pass

        self.root.after(CAMERA_POLL_IDLE_MS, poll)

    def _request_redraw(self) -> None:
        try:
            self.root.after(0, self._maybe_redraw)
        except tk.TclError:
            pass

    def _process_pending_redraw(self) -> None:
        self._request_redraw()

    def _bind_canvas_mousewheel(self, canvas: tk.Canvas) -> None:
        def _on_mousewheel(event):
            steps = _normalized_mousewheel_steps(event)
            if steps:
                canvas.yview_scroll(steps, "units")
                return "break"
            return None

        canvas.bind("<Enter>", lambda _e: canvas.focus_set(), add="+")
        canvas.bind("<MouseWheel>", _on_mousewheel, add="+")
        canvas.bind("<Button-4>", _on_mousewheel, add="+")
        canvas.bind("<Button-5>", _on_mousewheel, add="+")

    def _on_root_resized(self, event=None) -> None:
        if event is not None and event.widget is not self.root:
            return
        job = getattr(self, "_root_resize_job", None)
        if job is not None:
            try:
                self.root.after_cancel(job)
            except tk.TclError:
                pass
        delay_ms = int(getattr(self, "_root_resize_debounce_ms", 260))
        try:
            self._root_resize_job = self.root.after(delay_ms, self._flush_root_resize)
        except tk.TclError:
            self._root_resize_job = None

    def _flush_root_resize(self) -> None:
        self._root_resize_job = None
        self._apply_responsive_ui_scale()
        self._request_redraw()

    def _apply_responsive_ui_scale(self) -> None:
        try:
            w = max(1, int(self.root.winfo_width()))
            h = max(1, int(self.root.winfo_height()))
        except tk.TclError:
            return
        base_w = max(1, int(getattr(self, "_ui_base_width", 1260)))
        base_h = max(1, int(getattr(self, "_ui_base_height", 720)))
        scale = min(w / float(base_w), h / float(base_h))
        bucket = 0
        if scale >= 1.28:
            bucket = 2
        elif scale >= 1.08:
            bucket = 1
        if bucket == int(getattr(self, "_ui_scale_bucket", -1)):
            return
        self._ui_scale_bucket = bucket
        family = "Segoe UI" if os.name == "nt" else "TkDefaultFont"
        font_size = int(getattr(self, "_ui_font_base_size", 10)) + bucket
        for fname in ("TkDefaultFont", "TkTextFont", "TkMenuFont"):
            try:
                tkfont.nametofont(fname).configure(family=family, size=font_size)
            except Exception:
                pass
        try:
            tkfont.nametofont("TkHeadingFont").configure(family=family, size=font_size + 1, weight="bold")
        except Exception:
            pass
        style = ttk.Style(self.root)
        style.configure("TButton", padding=(8 + bucket * 2, 3 + bucket))
        style.configure("TCombobox", padding=(3 + bucket, 2 + bucket // 2))
        if hasattr(self, "_enable_btn"):
            try:
                self._enable_btn.configure(font=(family, 12 + bucket, "bold"))
            except Exception:
                pass
        if hasattr(self, "_estop_btn"):
            try:
                self._estop_btn.configure(font=(family, 12 + bucket, "bold"))
            except Exception:
                pass

    def _emit_action(self, msg: str) -> None:
        """在按钮响应中上报操作（如监控模式下直接打印）"""
        if self._action_callback:
            try:
                self._action_callback(msg)
            except Exception:
                pass

    def _run_data_root_dir(self) -> str:
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(project_root, "run_data")

    def _build_run_data_csv_path(self) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._run_data_root_dir(), f"hand_run_{ts}.csv")

    def _build_run_data_fieldnames(self) -> List[str]:
        fields: List[str] = ["ts_wall", "ts_model", "control_mode"]
        fields.extend([f"joint_curr_deg_j{i:02d}" for i in range(ENCODER_COUNT)])
        fields.extend([f"joint_target_deg_j{i:02d}" for i in range(ENCODER_COUNT)])
        fields.extend([f"servo_curr_single_m{i:02d}" for i in range(MOTOR_COUNT)])
        fields.extend([f"servo_target_single_m{i:02d}" for i in range(MOTOR_COUNT)])
        fields.extend([f"servo_curr_multi_m{i:02d}" for i in range(MOTOR_COUNT)])
        fields.extend(["has_sensor_data", "has_servo_raw_data", "has_servo_angle_data"])
        return fields

    def _start_data_recording(self) -> None:
        if not bool(getattr(self, "_csv_recording_enabled", False)):
            return
        if self._data_logger.is_running():
            return
        path = self._build_run_data_csv_path()
        self._data_logger.start(path, fieldnames=self._build_run_data_fieldnames())
        self._recording_status_note = f"Recording: {os.path.basename(path)}"
        self._emit_action(f"Data recording started: {path}")
        if hasattr(self, "status_var"):
            self.status_var.set(self._recording_status_note)

    def _stop_data_recording(self, reason: str = "stopped", keep_switch: bool = False) -> None:
        if self._data_logger.is_running():
            current_path = self._data_logger.path
            self._data_logger.stop()
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
        row.update({f"servo_curr_single_m{i:02d}": int(s.servo_raw_positions[i]) if i < len(s.servo_raw_positions) else 0 for i in range(MOTOR_COUNT)})
        row.update({f"servo_target_single_m{i:02d}": servo_targets_single[i] if i < len(servo_targets_single) else NA_MODE_MARKER for i in range(MOTOR_COUNT)})
        row.update({f"servo_curr_multi_m{i:02d}": int(s.servo_angles[i]) if i < len(s.servo_angles) else 0 for i in range(MOTOR_COUNT)})
        row["has_sensor_data"] = int(bool(getattr(s, "has_sensor_data", False)))
        row["has_servo_raw_data"] = int(bool(getattr(s, "has_servo_raw_data", False)))
        row["has_servo_angle_data"] = int(bool(getattr(s, "has_servo_angle_data", False)))
        self._data_logger.write_row(row)

    def _on_close(self):
        self._on_stop_test(reason="stopped on close", update_status=False)
        self._stop_data_recording(reason="stopped on close", keep_switch=True)
        self._destroy_joint_rt_plot()
        self._cancel_pose_3d_rezoom_timer()
        stop = getattr(self, "_camera_stop", None)
        if stop is not None:
            stop.set()
        thr = getattr(self, "_camera_thread", None)
        if thr is not None:
            thr.join(timeout=1.0)
            self._camera_thread = None
        redraw_job = getattr(self, "_redraw_job", None)
        if redraw_job is not None:
            try:
                self.root.after_cancel(redraw_job)
            except tk.TclError:
                pass
            self._redraw_job = None
        for job_name in ("_root_resize_job", "_pose_resize_job"):
            job = getattr(self, job_name, None)
            if job is None:
                continue
            try:
                self.root.after_cancel(job)
            except tk.TclError:
                pass
            setattr(self, job_name, None)
        for cap_attr in ("_camera1_cap", "_camera2_cap"):
            cap = getattr(self, cap_attr, None)
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
                setattr(self, cap_attr, None)
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
                        bg="#2563eb",
                        fg="#ffffff",
                        activebackground="#1d4ed8",
                        activeforeground="#ffffff",
                    )
                else:
                    btn.configure(
                        relief=tk.RAISED,
                        state="normal",
                        bg="#e5e7eb",
                        fg="#111827",
                        activebackground="#d1d5db",
                        activeforeground="#111827",
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
        with self._state_lock:
            self._state = state
        if self._redraw_job is None:
            self._redraw_job = self.root.after(0, self._maybe_redraw)

    def _maybe_redraw(self):
        self._redraw_job = None
        if not self._is_comm_connected():
            return
        now = time.time()
        if now - self._last_draw < self._draw_interval:
            delay_s = self._draw_interval - (now - self._last_draw)
            delay_ms = max(1, int(delay_s * 1000))
            if self._redraw_job is None:
                self._redraw_job = self.root.after(delay_ms, self._maybe_redraw)
            return
        self._last_draw = now
        self._update_display()

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
        self._camera_chain_switch = ttk.Checkbutton(
            top,
            text="Camera Link",
            variable=self._camera_chain_enabled_var,
            command=self._on_toggle_camera_chain,
        )
        self._camera_chain_switch.pack(side=tk.RIGHT, padx=(8, 0))
        self._pose_3d_switch = ttk.Checkbutton(
            top,
            text="3D Pose",
            variable=self._pose_3d_enabled_var,
            command=self._on_toggle_pose_3d,
        )
        self._pose_3d_switch.pack(side=tk.RIGHT, padx=(8, 0))
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

        self._setup_3d_display(pose_holder)
        self._setup_tactile_and_camera_widgets(tactile_cam_holder)

        right = ttk.Frame(main)
        main.add(right, weight=self._main_right_weight)
        self._right_panel_holder = right
        self._rebuild_right_panel()

        self._setup_status_bar()
        self._on_toggle_pose_3d()
        self._on_toggle_camera_chain()
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

    def _setup_3d_display(self, parent):
        pose_frame = ttk.Frame(parent)
        pose_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 2))
        self._pose_frame = pose_frame
        self.fig = plt.Figure(figsize=(6.5, 5.0), dpi=110, facecolor=POSE_VIEW_BG)
        self.ax = self.fig.add_axes((0.05, 0.10, 0.945, 0.82), projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, pose_frame)
        cw = self.canvas.get_tk_widget()
        self._pose_canvas_widget = cw
        cw.pack(fill=tk.BOTH, expand=True)
        try:
            cw.configure(bg=POSE_VIEW_BG, highlightthickness=0)
        except (tk.TclError, AttributeError):
            pass
        cw.bind("<Configure>", self._on_pose_canvas_configure)
        self._pose_3d_target_dist = compute_pose_view_camera_dist()
        self._pose_3d_drag_dist_scale = 0.90
        self._pose_3d_rezoom_delay_ms = 500
        self._pose_3d_rezoom_after_drag_job = None
        self.canvas.mpl_connect("scroll_event", self._on_pose_3d_mpl_scroll)
        self.canvas.mpl_connect("motion_notify_event", self._on_pose_3d_mpl_motion)
        self.canvas.mpl_connect("button_release_event", self._on_pose_3d_mpl_release)

        urdf_path = os.path.join(os.path.dirname(__file__), "hku_hand_v2_urdf", "urdf", "hand.urdf")
        try:
            self._urdf_kin = UrdfKinematics(urdf_path)
        except Exception as e:
            print(f"URDF load failed ({urdf_path}): {e}")
            self._urdf_kin = None
        self._draw_hand_model([0.0] * ENCODER_COUNT, None)

    def _on_pose_canvas_configure(self, event) -> None:
        """姿态 3D 区随窗口拉伸，Figure 尺寸与 Tk 画布一致。"""
        if not self._is_pose_3d_enabled():
            return
        if event.widget is not getattr(self, "_pose_canvas_widget", None):
            return
        w, h = int(event.width), int(event.height)
        if w < 24 or h < 24:
            return
        key = (w, h)
        if key == getattr(self, "_pose_canvas_size", None):
            return
        self._pose_canvas_size = key
        self._pose_resize_pending_size = key
        job = getattr(self, "_pose_resize_job", None)
        if job is not None:
            try:
                self.root.after_cancel(job)
            except tk.TclError:
                pass
        delay_ms = int(getattr(self, "_pose_resize_debounce_ms", 80))
        try:
            self._pose_resize_job = self.root.after(delay_ms, self._flush_pose_canvas_resize)
        except tk.TclError:
            self._pose_resize_job = None
        return

    def _flush_pose_canvas_resize(self) -> None:
        self._pose_resize_job = None
        key = getattr(self, "_pose_resize_pending_size", None)
        if key is None:
            return
        if not self._is_pose_3d_enabled():
            return
        if not hasattr(self, "fig") or not hasattr(self, "ax") or not hasattr(self, "canvas"):
            return
        w, h = key
        if w < 24 or h < 24:
            return
        dpi = float(self.fig.dpi)
        self.fig.set_size_inches(w / dpi, h / dpi)
        try:
            rw = int(round(float(self.fig.get_figwidth()) * dpi))
            rh = int(round(float(self.fig.get_figheight()) * dpi))
            if rw < w or rh < h:
                self.fig.set_size_inches((w + 1) / dpi, (h + 1) / dpi)
        except (TypeError, ValueError, AttributeError):
            pass
        fill_3d_axes_to_figure(self.ax, has_title=True)
        apply_pose_view_background(self.ax)
        apply_mplot3d_box_aspect_fill_widget(self.ax)
        apply_mplot3d_camera_zoom(self.ax, dist=self._pose_3d_target_dist)
        self._pose_view_dirty = True
        self.canvas.draw_idle()

    def _sync_pose_3d_target_dist_from_axes(self) -> None:
        ax = getattr(self, "ax", None)
        if ax is None or not hasattr(ax, "_dist"):
            return
        try:
            self._pose_3d_target_dist = float(ax._dist)
        except (TypeError, ValueError):
            pass

    def _cancel_pose_3d_rezoom_timer(self) -> None:
        job = getattr(self, "_pose_3d_rezoom_after_drag_job", None)
        if job is None:
            return
        try:
            self.root.after_cancel(job)
        except tk.TclError:
            pass
        self._pose_3d_rezoom_after_drag_job = None

    def _on_pose_3d_mpl_scroll(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None):
            return
        self._cancel_pose_3d_rezoom_timer()
        self.root.after_idle(self._sync_pose_3d_target_dist_from_axes)

    @staticmethod
    def _pose_3d_mpl_left_drag(event) -> bool:
        if event.button == 1:
            return True
        btns = getattr(event, "buttons", None)
        if not btns:
            return False
        return 1 in btns

    def _on_pose_3d_mpl_motion(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None):
            return
        if not self._pose_3d_mpl_left_drag(event):
            return
        self._cancel_pose_3d_rezoom_timer()
        drag_scale = float(getattr(self, "_pose_3d_drag_dist_scale", 1.0))
        drag_dist = float(self._pose_3d_target_dist) * max(0.5, min(1.0, drag_scale))
        apply_mplot3d_camera_zoom(self.ax, dist=drag_dist)

    def _on_pose_3d_mpl_release(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None) or event.button != 1:
            return
        self._cancel_pose_3d_rezoom_timer()

        def rezoom_after_release() -> None:
            self._pose_3d_rezoom_after_drag_job = None
            apply_mplot3d_camera_zoom(self.ax, dist=self._pose_3d_target_dist)
            self.canvas.draw_idle()

        delay_ms = int(getattr(self, "_pose_3d_rezoom_delay_ms", 500))
        self._pose_3d_rezoom_after_drag_job = self.root.after(delay_ms, rezoom_after_release)

    def _toggle_tactile_axis(self, axis: int):
        _ = axis
        self._emit_action("Tactile axis switch is disabled in dot-matrix mode")

    def _on_tactile_fake_toggle(self):
        self._tactile_fake_mode = bool(self._tactile_fake_var.get())
        with self._state_lock:
            s = self._state
        if not s:
            s = HandModel()
        tactile_state = self._get_tactile_display_state(s, fake=self._should_use_fake_tactile(s))
        self._draw_hand_model(s.angles, tactile_state)
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
                btn.configure(bg="#2563eb", fg="#ffffff", activebackground="#1d4ed8", activeforeground="#ffffff")
            else:
                btn.configure(bg="#e5e7eb", fg="#111827", activebackground="#d1d5db", activeforeground="#111827")

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
        """触觉区参考 simulate_tactile_dot_matrix：3x5 点阵子图，统一显示 Fz。"""
        if not hasattr(self, "_tactile_fig"):
            return

        self._tactile_fig.clear()
        self._tactile_dot_scatters = []
        self._tactile_status_text = None
        self._tactile_colorbar = None

        # Follow paxini_all_read_2.12 semantics:
        # columns = MUX groups D3..D7, rows = DP/IP/CP sensors
        group_names = ["D3", "D4", "D5", "D6", "D7"]
        sensor_names = ["DP(A+B)", "IP(A)", "CP(B)"]
        axes = self._tactile_fig.subplots(3, 5)

        for sensor_idx in range(3):
            scatter_row = []
            rows, cols = self._sensor_shape_by_index(sensor_idx)
            yy, xx = np.mgrid[0:rows, 0:cols]
            base_vals = np.zeros(rows * cols, dtype=float)

            for finger_idx in range(5):
                ax = axes[sensor_idx, finger_idx]
                sc = ax.scatter(
                    xx.flatten(),
                    yy.flatten(),
                    c=base_vals.copy(),
                    cmap="inferno",
                    vmin=0.0,
                    vmax=255.0,
                    s=150,
                    marker="s",
                    edgecolors="#111111",
                    linewidths=0.25,
                )

                ax.set_xlim(-0.75, cols - 0.25)
                ax.set_ylim(rows - 0.25, -0.75)
                ax.set_aspect("equal")
                ax.set_xticks([])
                ax.set_yticks([])
                ax.grid(alpha=0.25, linewidth=0.35)

                if sensor_idx == 0:
                    ax.set_title(group_names[finger_idx], fontsize=9)
                if finger_idx == 0:
                    ax.set_ylabel(sensor_names[sensor_idx], fontsize=8)

                scatter_row.append(sc)

            self._tactile_dot_scatters.append(scatter_row)

        self._tactile_status_text = self._tactile_fig.suptitle("No tactile data", fontsize=10)
        cax = self._tactile_fig.add_axes([0.93, 0.12, 0.015, 0.76])
        self._tactile_colorbar = self._tactile_fig.colorbar(
            self._tactile_dot_scatters[0][0], cax=cax
        )
        self._tactile_colorbar.set_label("Fz")
        self._tactile_fig.subplots_adjust(left=0.06, right=0.90, bottom=0.05, top=0.92)

    def _setup_tactile_matrix_widget(self, parent):
        """触觉空间矩阵：15 个传感器按 5 指 x 3 骨段拼成一个热力图"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # 标题与触觉轴按钮同一行
        self._tactile_row = ttk.Frame(frame)
        self._tactile_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(self._tactile_row, text="Tactile Dot Matrix (2.12, Fz)").pack(side=tk.LEFT, padx=2)
        self._tactile_fake_var = tk.BooleanVar(value=not bool(getattr(self, "_ports_cache", [])))
        self._tactile_fake_check = ttk.Checkbutton(
            self._tactile_row,
            text="Fake",
            variable=self._tactile_fake_var,
            command=self._on_tactile_fake_toggle,
        )
        self._tactile_fake_check.pack(side=tk.LEFT, padx=4)
        self._tactile_axis_buttons = []

        self._tactile_fig = plt.Figure(figsize=(7.8, 4.0))
        self._tactile_canvas = FigureCanvasTkAgg(self._tactile_fig, frame)
        self._tactile_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._tactile_dot_scatters: List = []
        self._tactile_status_text = None
        self._tactile_colorbar = None
        self._rebuild_tactile_figure()
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

    def _setup_tactile_and_camera_widgets(self, parent):
        """将下方区域拆分为左右两栏：触觉矩阵与相机各占左侧主区一半宽度。"""
        split = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        split.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)
        self._tactile_cam_paned = split

        left_frame = ttk.Frame(split)
        split.add(left_frame, weight=1)
        self._setup_tactile_matrix_widget(left_frame)

        cam_frame = ttk.LabelFrame(split, text="")
        split.add(cam_frame, weight=1)
        try:
            split.pane(left_frame, minsize=100)
            split.pane(cam_frame, minsize=120)
        except tk.TclError:
            pass
        self._setup_camera_widget(cam_frame)

    def _setup_camera_widget(self, parent):
        """相机图像窗格：上下两个，每个含摄像头下拉选择 + 画面全填充"""
        self._camera_frame_left = None
        self._camera_frame_right = None

        split = ttk.PanedWindow(parent, orient=tk.VERTICAL)
        split.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        left_box = ttk.Frame(split)
        right_box = ttk.Frame(split)
        split.add(left_box, weight=1)
        split.add(right_box, weight=1)
        try:
            split.pane(left_box, minsize=72)
            split.pane(right_box, minsize=72)
        except tk.TclError:
            pass

        def _make_camera_slot(box, var, slot):
            row = ttk.Frame(box)
            row.pack(fill=tk.X, pady=(0, 2))
            ttk.Label(row, text="Camera:").pack(side=tk.LEFT, padx=(0, 4))
            combo = ttk.Combobox(row, textvariable=var, width=12, state="readonly")
            combo.pack(side=tk.LEFT)
            combo.bind("<<ComboboxSelected>>", lambda e, c=combo, s=slot: self._on_camera_combo_selected(s, c))
            canvas_frame = ttk.Frame(box)
            canvas_frame.pack(fill=tk.BOTH, expand=True)
            canvas = tk.Canvas(canvas_frame, bg="black", highlightthickness=0)
            canvas.pack(fill=tk.BOTH, expand=True)
            return combo, canvas

        self._cam_left_combo, self._cam_left_canvas = _make_camera_slot(left_box, self._camera1_var, 1)
        self._cam_right_combo, self._cam_right_canvas = _make_camera_slot(right_box, self._camera2_var, 2)

        self._refresh_camera_menus()

    def set_camera_frame_left(self, frame: np.ndarray) -> None:
        """外部可调用：设置左相机帧（RGB/BGR/灰度均可）"""
        self._camera_frame_left = frame

    def set_camera_frame_right(self, frame: np.ndarray) -> None:
        """外部可调用：设置右相机帧（RGB/BGR/灰度均可）"""
        self._camera_frame_right = frame

    def set_camera_frame(self, frame: np.ndarray) -> None:
        """兼容旧接口：默认写入左相机帧"""
        self._camera_frame_left = frame

    def _update_camera_combo_values(self) -> None:
        """Set combo values: None + available cameras."""
        choices = ["None"] + [f"Camera {i}" for i in self._camera_indices_cache]
        if hasattr(self, "_cam_left_combo"):
            self._cam_left_combo["values"] = choices
        if hasattr(self, "_cam_right_combo"):
            self._cam_right_combo["values"] = choices

    def _on_camera_combo_selected(self, slot: int, combo: ttk.Combobox) -> None:
        """Handle camera selection from combo (slot 1 or 2). Defer to ensure value is updated."""
        def do():
            val = combo.get()
            if val == "None" or not val:
                self._on_camera_selected(None, slot)
            else:
                try:
                    idx = int(val.replace("Camera ", ""))
                    self._on_camera_selected(idx, slot)
                except ValueError:
                    pass
        self.root.after(0, do)

    def _update_single_camera_widget(self, frame, canvas, photo_attr: str):
        """Update canvas with frame using PIL/PhotoImage for real-time video."""
        try:
            from PIL import Image
            from PIL import ImageTk
        except ImportError:
            return
        canvas.delete("all")
        cw = max(canvas.winfo_width() or 320, 320)
        ch = max(canvas.winfo_height() or 240, 240)
        if frame is None:
            canvas.create_text(cw // 2, ch // 2, text="None", fill="white", font=("", 14))
            return
        img = np.asarray(frame, dtype=np.uint8)
        if img.ndim == 2:
            pil_img = Image.fromarray(img, mode="L")
        elif img.ndim == 3 and img.shape[2] >= 3:
            pil_img = Image.fromarray(img[:, :, :3], mode="RGB")
        else:
            return
        resample = getattr(Image, "Resampling", Image).LANCZOS
        pil_img.thumbnail((max(cw, 1), max(ch, 1)), resample)
        photo = ImageTk.PhotoImage(pil_img)
        setattr(self, photo_attr, photo)
        canvas.create_image(cw // 2, ch // 2, image=photo)

    def _update_camera_widget(self):
        """相机与串口 Start/Stop 无关：在 Device 中选摄像头后即采集显示，由 _start_camera_poll 刷新。"""
        if not hasattr(self, "_cam_left_canvas"):
            return
        self._update_single_camera_widget(
            self._camera_frame_left, self._cam_left_canvas, "_cam_left_photo"
        )
        self._update_single_camera_widget(
            self._camera_frame_right, self._cam_right_canvas, "_cam_right_photo"
        )

    @staticmethod
    def _sensor_shape_by_index(sensor_idx: int) -> Tuple[int, int]:
        if sensor_idx == 0:
            return tactile_segment_shape(2)  # tip: 5x5
        return tactile_segment_shape(0)  # pad: 4x13

    @classmethod
    def _extract_sensor_dot_values(cls, fd: FingerTactile, sensor_idx: int) -> np.ndarray:
        rows, cols = cls._sensor_shape_by_index(sensor_idx)

        if sensor_idx >= len(fd.sensors):
            return np.zeros(rows * cols, dtype=float)

        sensor = fd.sensors[sensor_idx]
        cf = getattr(sensor, "contact_forces", None)
        if cf is not None and hasattr(cf, "shape") and len(cf.shape) >= 3:
            z_idx = min(2, int(cf.shape[2]) - 1)
            grid = np.asarray(cf[:, :, z_idx], dtype=float)
            out = np.zeros((rows, cols), dtype=float)
            r0 = min(rows, grid.shape[0])
            c0 = min(cols, grid.shape[1])
            out[:r0, :c0] = grid[:r0, :c0]
            return np.clip(out, 0.0, None).flatten()

        resultant = getattr(sensor, "resultant", None)
        fz = float(resultant[2]) if resultant is not None and len(resultant) >= 3 else 0.0
        return np.full(rows * cols, max(0.0, fz), dtype=float)

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
        if not hasattr(self, "_tactile_dot_scatters"):
            return

        tactile_list = state.tactile_data if (state and state.tactile_data) else []
        any_data = False
        vmax = 1.0

        for sensor_idx in range(3):
            for finger_idx in range(5):
                scatter = self._tactile_dot_scatters[sensor_idx][finger_idx]
                if finger_idx < len(tactile_list):
                    fd = tactile_list[finger_idx]
                else:
                    fd = FingerTactile(sensors=[])

                vals = self._extract_sensor_dot_values(fd, sensor_idx)
                scatter.set_array(vals)
                if vals.size > 0:
                    local_max = float(np.max(vals))
                    vmax = max(vmax, local_max)
                    if local_max > 1e-9:
                        any_data = True

        for row in self._tactile_dot_scatters:
            for scatter in row:
                scatter.set_clim(0.0, vmax)

        if getattr(self, "_tactile_status_text", None) is not None:
            self._tactile_status_text.set_text(
                f"Tactile Dot Matrix (2.12: D3~D7 x DP/IP/CP, Fz) | "
                f"{'LIVE' if any_data else 'No tactile data'} | vmax={vmax:.1f}"
            )
        if getattr(self, "_tactile_colorbar", None) is not None:
            self._tactile_colorbar.set_label(f"Fz (0..{vmax:.0f})")

        self._tactile_canvas.draw_idle()

    def _get_generated_pose_angles(self) -> List[float]:
        """从 pose 文本缓存或重新加载；文件缺失或解析失败时退回全 0。"""
        path = self._pose_generate_path
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            if self._generated_pose_angles is not None:
                return self._generated_pose_angles
            print(f"pose_generate: 未找到文件 {path!r}，姿态用全 0", flush=True)
            return [0.0] * ENCODER_COUNT
        if self._generated_pose_mtime == mtime and self._generated_pose_angles is not None:
            return self._generated_pose_angles
        try:
            self._generated_pose_angles = load_motor_degrees_from_text(path)
        except Exception as e:
            print(f"pose_generate: 读取失败 {path!r}: {e}", flush=True)
            if self._generated_pose_angles is not None:
                return self._generated_pose_angles
            return [0.0] * ENCODER_COUNT
        self._generated_pose_mtime = mtime
        return self._generated_pose_angles

    def _draw_hand_model(
        self,
        angles: List[float],
        state: Optional[HandModel],
        *,
        generate: Optional[bool] = None,
    ):
        """根据 URDF 正向运动学绘制手部骨架。

        generate：None 时用实例属性 _pose_generate；True 强制从文本读角；False 强制用传入 angles。
        state 保留参数以便与旧调用兼容；触觉叠加已在独立子图中绘制。
        """
        if not self._is_pose_3d_enabled():
            return
        _ = state
        if not hasattr(self, "ax") or not hasattr(self, "fig"):
            return
        if generate is None:
            use_gen = not self._should_use_realtime_pose()
        else:
            use_gen = bool(generate)
        if use_gen:
            angles = self._get_generated_pose_angles()
        if self._urdf_kin is None:
            self.ax.clear()
            apply_pose_view_background(self.ax)
            self.ax.set_title("URDF load failed")
            self.fig.canvas.draw_idle()
            return
        joint_pos = motor_deg_to_urdf_joint_dict(angles)
        # 大部分帧仅更新连杆/节点；只有视图变化（如拖拽、缩放、窗口尺寸变化）时才重建静态背景。
        self._ensure_pose_static_scene(joint_pos)
        self._update_pose_dynamic_artists(joint_pos)
        self.fig.canvas.draw_idle()

    def _clear_pose_dynamic_artists(self) -> None:
        for _pa, _ch, line in getattr(self, "_pose_link_artists", []):
            self._safe_remove_artist(line)
        self._pose_link_artists = []
        n = getattr(self, "_pose_nodes_artist", None)
        if n is not None:
            self._safe_remove_artist(n)
        self._pose_nodes_artist = None

    def _ensure_pose_static_scene(self, joint_pos: dict) -> None:
        if self._pose_static_scene_ready and not self._pose_view_dirty:
            return

        preserve_user_view = bool(self._pose_static_scene_ready)
        prev_elev = None
        prev_azim = None
        prev_roll = None
        prev_dist = None
        if preserve_user_view:
            try:
                prev_elev = float(self.ax.elev)
                prev_azim = float(self.ax.azim)
                prev_dist = float(self._pose_3d_target_dist)
                if hasattr(self.ax, "roll"):
                    prev_roll = float(self.ax.roll)
            except (TypeError, ValueError, AttributeError):
                prev_elev = None
                prev_azim = None
                prev_roll = None
                prev_dist = None

        # draw_skeleton 内部会清空 axes；先清理动态层避免对已脱管 artist 再次 remove。
        self._clear_pose_dynamic_artists()
        draw_skeleton(
            self.ax,
            self._urdf_kin,
            joint_pos,
            display_transform=DISPLAY_TRANSFORM,
            plot_title="Pose (URDF skeleton)",
            draw_links_nodes=False,
        )

        if preserve_user_view and prev_elev is not None and prev_azim is not None:
            try:
                if prev_roll is not None:
                    self.ax.view_init(elev=prev_elev, azim=prev_azim, roll=prev_roll)
                else:
                    self.ax.view_init(elev=prev_elev, azim=prev_azim)
            except TypeError:
                self.ax.view_init(elev=prev_elev, azim=prev_azim)
            if prev_dist is not None:
                apply_mplot3d_camera_zoom(self.ax, dist=prev_dist)
                self._pose_3d_target_dist = float(prev_dist)
            else:
                self._sync_pose_3d_target_dist_from_axes()
        else:
            cd = getattr(self.ax, "_pose_view_cam_dist", None)
            if cd is not None:
                try:
                    self._pose_3d_target_dist = float(cd)
                except (TypeError, ValueError):
                    self._sync_pose_3d_target_dist_from_axes()
            else:
                self._sync_pose_3d_target_dist_from_axes()

        self._pose_static_scene_ready = True
        self._pose_view_dirty = False

    def _update_pose_dynamic_artists(self, joint_pos: dict) -> None:
        poses = apply_display_transform(self._urdf_kin.compute_link_poses(joint_pos), DISPLAY_TRANSFORM)
        if not self._pose_link_artists:
            for j in self._urdf_kin.joints:
                pa, ch = j["parent"], j["child"]
                if pa not in poses or ch not in poses:
                    continue
                p0 = poses[pa][:3, 3]
                p1 = poses[ch][:3, 3]
                (line,) = self.ax.plot(
                    [p0[0], p1[0]],
                    [p0[1], p1[1]],
                    [p0[2], p1[2]],
                    color="black",
                    linewidth=2.0,
                    zorder=20,
                )
                self._pose_link_artists.append((pa, ch, line))
        else:
            for pa, ch, line in self._pose_link_artists:
                if pa not in poses or ch not in poses:
                    continue
                p0 = poses[pa][:3, 3]
                p1 = poses[ch][:3, 3]
                line.set_data_3d([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]])

        xs: List[float] = []
        ys: List[float] = []
        zs: List[float] = []
        for T in poses.values():
            p = T[:3, 3]
            xs.append(float(p[0]))
            ys.append(float(p[1]))
            zs.append(float(p[2]))
        if self._pose_nodes_artist is None:
            self._pose_nodes_artist = self.ax.scatter(xs, ys, zs, s=12, c="red", depthshade=True, zorder=21)
        else:
            self._pose_nodes_artist._offsets3d = (xs, ys, zs)

    def _setup_control_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Manual Control")
        panel.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        device_box = ttk.LabelFrame(panel, text="Device")
        device_box.pack(fill=tk.X, padx=6, pady=(6, 4))
        device_grid = ttk.Frame(device_box)
        device_grid.pack(anchor="w", padx=6, pady=6)
        device_grid.columnconfigure(0, weight=0)
        device_grid.columnconfigure(1, weight=0)
        device_grid.columnconfigure(2, weight=0)
        device_grid.columnconfigure(3, weight=0)

        ttk.Button(device_grid, text="Home", width=11, command=self._on_btn_home).grid(
            row=0, column=0, padx=(0, 6), pady=(0, 4), sticky="w"
        )
        ttk.Button(device_grid, text="Calibrate", width=11, command=self._on_calibrate).grid(
            row=1, column=0, padx=(0, 6), pady=(0, 0), sticky="w"
        )
        ttk.Button(device_grid, text="Clear Fault", width=11, command=self._on_btn_reset).grid(
            row=1, column=1, padx=(0, 6), pady=(0, 0), sticky="w"
        )
        self._enable_btn = tk.Label(
            device_grid,
            text="ENABLE",
            bg="#2e7d32",
            fg="#f6fff6",
            relief=tk.RAISED,
            bd=2,
            font=("", 12, "bold"),
            width=12,
            height=3,
            cursor="hand2",
            padx=10,
            pady=2,
        )
        self._enable_btn.grid(
            row=0, column=2, rowspan=2, padx=(8, 6), pady=0, sticky="nsw"
        )
        self._enable_btn.bind("<Button-1>", lambda _event: self._on_btn_start())
        self._enable_btn.bind("<ButtonRelease-1>", lambda _event: self._enable_btn.configure(bg="#2e7d32"))
        self._enable_btn.bind("<Enter>", lambda _event: self._enable_btn.configure(bg="#1b5e20"))
        self._enable_btn.bind("<Leave>", lambda _event: self._enable_btn.configure(bg="#2e7d32"))
        self._estop_btn = tk.Label(
            device_grid,
            text="E-STOP",
            bg="#c62828",
            fg="#fff7f7",
            relief=tk.RAISED,
            bd=2,
            font=("", 12, "bold"),
            width=12,
            height=3,
            cursor="hand2",
            padx=10,
            pady=2,
        )
        self._estop_btn.grid(
            row=0, column=3, rowspan=2, padx=(8, 0), pady=0, sticky="nsw"
        )
        self._estop_btn.bind("<Button-1>", lambda _event: self._on_btn_stop())
        self._estop_btn.bind("<ButtonRelease-1>", lambda _event: self._estop_btn.configure(bg="#c62828"))
        self._estop_btn.bind("<Enter>", lambda _event: self._estop_btn.configure(bg="#b71c1c"))
        self._estop_btn.bind("<Leave>", lambda _event: self._estop_btn.configure(bg="#c62828"))

        workflow_box = ttk.LabelFrame(panel, text="Workflow")
        workflow_box.pack(fill=tk.X, padx=6, pady=4)
        mode_row = ttk.Frame(workflow_box)
        mode_row.pack(fill=tk.X, padx=6, pady=(6, 4))
        ttk.Label(mode_row, text="Mode:").pack(side=tk.LEFT)
        self._interaction_enabled = False
        self._control_widgets: List[tk.Widget] = []
        self._ctrl_mode_var = tk.StringVar(value="joint")
        rb_joint = ttk.Radiobutton(
            mode_row,
            text="Whole-Hand PID",
            variable=self._ctrl_mode_var,
            value="joint",
            command=self._switch_control_mode,
        )
        rb_joint.pack(side=tk.LEFT, padx=4)
        rb_motor = ttk.Radiobutton(
            mode_row,
            text="Single-Motor Open-Loop",
            variable=self._ctrl_mode_var,
            value="motor",
            command=self._switch_control_mode,
        )
        rb_motor.pack(side=tk.LEFT, padx=4)
        rb_test = ttk.Radiobutton(
            mode_row,
            text="Test",
            variable=self._ctrl_mode_var,
            value="test",
            command=self._switch_control_mode,
        )
        rb_test.pack(side=tk.LEFT, padx=4)
        self._interaction_btn = ttk.Button(mode_row, text="Hand Manual: OFF", command=self._toggle_interaction_control)
        self._interaction_btn.pack(side=tk.RIGHT, padx=2)

        self._table_section = ttk.LabelFrame(panel, text=f"Channel Table (J{ENCODER_COUNT} / M{MOTOR_COUNT})")
        self._table_section.pack(fill=tk.BOTH, expand=True, padx=6, pady=(4, 6))
        self._batch_action_row = ttk.Frame(self._table_section)
        self._batch_action_row.pack(fill=tk.X, padx=4, pady=(4, 2))
        self._open_pose_btn = ttk.Button(
            self._batch_action_row,
            text="Open Pose",
            width=11,
            command=self._apply_preset_open,
        )
        self._open_pose_btn.pack(side=tk.LEFT, padx=(0, 4))
        self._fist_pose_btn = ttk.Button(
            self._batch_action_row,
            text="Fist Pose",
            width=11,
            command=self._apply_preset_fist,
        )
        self._fist_pose_btn.pack(side=tk.LEFT, padx=4)
        self._zero_pose_btn = ttk.Button(
            self._batch_action_row,
            text="Zero Pose",
            width=11,
            command=self._apply_preset_zero,
        )
        self._zero_pose_btn.pack(side=tk.LEFT, padx=4)
        self._send_all_btn = ttk.Button(self._batch_action_row, text="Send All", command=self._apply_all_angles)
        self._send_all_btn.pack(side=tk.LEFT, padx=(8, 0))
        self._ctrl_body = ttk.Frame(self._table_section)
        self._ctrl_body.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        self._joint_panel = ttk.Frame(self._ctrl_body)
        self._motor_panel = ttk.Frame(self._ctrl_body)
        self._test_panel = ttk.Frame(self._ctrl_body)

        self._joint_curr_vars: List[tk.StringVar] = []
        self._joint_target_vars: List[tk.DoubleVar] = []
        self._joint_sliders: List[ttk.Scale] = []
        self._joint_target_user_set: List[bool] = []
        self._joint_status_dots: List[Tuple[tk.Canvas, int]] = []
        self._joint_tie_sign_vars: List[tk.StringVar] = []
        self._joint_tie_x1_vars: List[tk.StringVar] = []
        self._joint_tie_motor_abs_vars: List[tk.StringVar] = []
        self._joint_tie_x1_live_vars: List[tk.StringVar] = []
        self._joint_target_initialized = False
        self._motor_curr_vars: List[tk.StringVar] = []
        self._motor_joint_angle_vars: List[tk.StringVar] = []
        self._motor_target_vars: List[tk.IntVar] = []
        self._motor_target_user_set: List[bool] = []
        self._motor_target_initialized = False
        self._motor_abs_pos: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_session_origin_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_dir_hint: List[int] = [0] * MOTOR_COUNT
        self._motor_dir_hint_pending_clear: List[bool] = [False] * MOTOR_COUNT
        self._motor_dir_hint_stable_count: List[int] = [0] * MOTOR_COUNT
        self._motor_last_slider: List[Optional[int]] = [None] * MOTOR_COUNT
        self._motor_slider_dragging: List[bool] = [False] * MOTOR_COUNT
        self._motor_first_drag_limit_active: List[bool] = [True] * MOTOR_COUNT
        self._motor_last_sent_abs: List[Optional[int]] = [None] * MOTOR_COUNT

        self._build_mode_table(self._joint_panel, is_motor=False)
        self._build_mode_table(self._motor_panel, is_motor=True)
        self._setup_test_panel(self._test_panel)
        # 初始化为关闭：禁用交互输入
        for w in self._control_widgets:
            try:
                w.configure(state="disabled")
            except Exception:
                pass
        self._update_manual_action_buttons()
        self._switch_control_mode()
        self._ui_scale_bucket = -1
        self._apply_responsive_ui_scale()

    def _open_joint_rt_window(self) -> None:
        win = getattr(self, "_joint_rt_window", None)
        if win is not None:
            try:
                if win.winfo_exists():
                    if getattr(self, "_joint_rt_canvas", None) is None:
                        self._setup_joint_realtime_plot(win)
                    win.deiconify()
                    win.lift()
                    win.focus_set()
                    return
            except Exception:
                pass

        win = tk.Toplevel(self.root)
        win.title("Joint Real-Time Curve")
        win.geometry("760x360")
        win.protocol("WM_DELETE_WINDOW", self._on_joint_rt_window_close)
        self._joint_rt_window = win
        self._setup_joint_realtime_plot(win)

    def _on_joint_rt_window_close(self) -> None:
        self._destroy_joint_rt_plot()

    def _setup_joint_realtime_plot(self, parent) -> None:
        self._destroy_joint_rt_plot(close_window=False)
        box = ttk.LabelFrame(parent, text="Joint Real-Time Curve")
        box.pack(fill=tk.BOTH, expand=True, padx=6, pady=(2, 4))

        top = ttk.Frame(box)
        top.pack(fill=tk.X, padx=6, pady=(6, 2))
        ttk.Label(top, text="Joints:").pack(side=tk.LEFT)
        values = ["None"] + [f"J{i:02d}" for i in range(ENCODER_COUNT)]
        if len(getattr(self, "_joint_rt_joint_vars", [])) != JOINT_RT_MAX_PLOT_JOINTS:
            self._joint_rt_joint_vars = [tk.StringVar(value="None") for _ in range(JOINT_RT_MAX_PLOT_JOINTS)]
        for i, var in enumerate(self._joint_rt_joint_vars):
            var.set(self._normalize_joint_rt_value(var.get()))
            combo = ttk.Combobox(
                top,
                textvariable=var,
                values=values,
                state="readonly",
                width=6,
            )
            combo.pack(side=tk.LEFT, padx=(4 if i == 0 else 2, 2))
            combo.bind("<<ComboboxSelected>>", lambda _e, slot=i: self._on_joint_rt_selection_changed(slot))
        ttk.Button(top, text="Clear", width=8, command=self._on_joint_rt_clear).pack(side=tk.LEFT)

        self._joint_rt_fig = plt.Figure(figsize=(7.2, 3.0), dpi=100)
        self._joint_rt_ax = self._joint_rt_fig.add_subplot(111)
        self._joint_rt_canvas = FigureCanvasTkAgg(self._joint_rt_fig, box)
        self._joint_rt_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))

        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)

    def _destroy_joint_rt_plot(self, close_window: bool = True) -> None:
        canvas = getattr(self, "_joint_rt_canvas", None)
        if canvas is not None:
            try:
                widget = canvas.get_tk_widget()
                widget.destroy()
            except Exception:
                pass
        fig = getattr(self, "_joint_rt_fig", None)
        if fig is not None:
            try:
                plt.close(fig)
            except Exception:
                pass
        self._joint_rt_canvas = None
        self._joint_rt_fig = None
        self._joint_rt_ax = None
        self._joint_rt_line_target = None
        self._joint_rt_line_actual = None
        if close_window:
            win = getattr(self, "_joint_rt_window", None)
            self._joint_rt_window = None
            if win is not None:
                try:
                    if win.winfo_exists():
                        win.destroy()
                except Exception:
                    pass

    @staticmethod
    def _normalize_joint_rt_value(value: str) -> str:
        text = str(value or "").strip().upper()
        if text in ("", "NONE", "N", "-"):
            return "None"
        if text.startswith("J"):
            try:
                idx = int(text[1:])
            except ValueError:
                return "None"
            if 0 <= idx < ENCODER_COUNT:
                return f"J{idx:02d}"
        return "None"

    def _selected_joint_rt_indices(self) -> List[int]:
        selected: List[int] = []
        for var in getattr(self, "_joint_rt_joint_vars", []):
            norm = self._normalize_joint_rt_value(var.get())
            var.set(norm)
            if norm == "None":
                continue
            idx = int(norm[1:])
            if idx not in selected:
                selected.append(idx)
        return selected[:JOINT_RT_MAX_PLOT_JOINTS]

    def _on_joint_rt_selection_changed(self, slot: Optional[int] = None) -> None:
        if slot is not None and 0 <= int(slot) < len(self._joint_rt_joint_vars):
            slot_idx = int(slot)
            current = self._normalize_joint_rt_value(self._joint_rt_joint_vars[slot_idx].get())
            self._joint_rt_joint_vars[slot_idx].set(current)
            if current != "None":
                for i, var in enumerate(self._joint_rt_joint_vars):
                    if i == slot_idx:
                        continue
                    if self._normalize_joint_rt_value(var.get()) == current:
                        self._joint_rt_joint_vars[slot_idx].set("None")
                        if hasattr(self, "status_var"):
                            self.status_var.set(f"Plot selection duplicate joint: {current} already selected")
                        break
        selected = self._selected_joint_rt_indices()
        if selected:
            self._joint_rt_joint_index = int(selected[0])
            self._joint_rt_joint_var.set(f"J{self._joint_rt_joint_index:02d}")
        else:
            self._joint_rt_joint_index = 0
            self._joint_rt_joint_var.set("J00")
        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)

    def _on_joint_rt_clear(self) -> None:
        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)

    def _clear_joint_rt_history(self) -> None:
        self._joint_rt_series_t.clear()
        self._joint_rt_series_actual.clear()
        self._joint_rt_series_target.clear()
        self._joint_rt_t0 = 0.0
        self._joint_rt_last_plot_ts = 0.0

    def _ensure_joint_rt_series(self, joint_idx: int) -> None:
        if joint_idx not in self._joint_rt_series_t:
            self._joint_rt_series_t[joint_idx] = deque(maxlen=1800)
            self._joint_rt_series_actual[joint_idx] = deque(maxlen=1800)
            self._joint_rt_series_target[joint_idx] = deque(maxlen=1800)

    def _append_joint_rt_sample(self, s: HandModel) -> None:
        selected = self._selected_joint_rt_indices()
        if not selected:
            return

        now = time.time()
        if self._joint_rt_t0 <= 0.0:
            self._joint_rt_t0 = now
        t_rel = now - self._joint_rt_t0

        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        targets: Optional[List[float]] = None
        if mode in ("joint", "test"):
            try:
                targets = self._resolved_joint_targets()
            except Exception:
                targets = None

        for idx in selected:
            self._ensure_joint_rt_series(idx)
            actual = float("nan")
            if idx < len(getattr(s, "angles", [])):
                try:
                    actual = float(s.angles[idx])
                except Exception:
                    actual = float("nan")
            target = float("nan")
            if targets is not None and idx < len(targets):
                try:
                    target = float(targets[idx])
                except Exception:
                    target = float("nan")
            self._joint_rt_series_t[idx].append(float(t_rel))
            self._joint_rt_series_actual[idx].append(actual)
            self._joint_rt_series_target[idx].append(target)

    def _update_joint_rt_plot(self, force: bool = False) -> None:
        ax = getattr(self, "_joint_rt_ax", None)
        canvas = getattr(self, "_joint_rt_canvas", None)
        if ax is None or canvas is None:
            return

        now = time.time()
        if not force and (now - float(getattr(self, "_joint_rt_last_plot_ts", 0.0))) < float(self._joint_rt_plot_interval):
            return
        self._joint_rt_last_plot_ts = now

        ax.clear()
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.set_ylabel("Angle (deg)", fontsize=9)
        ax.grid(True, alpha=0.28)

        selected = self._selected_joint_rt_indices()
        x_max = 0.0
        vals: List[float] = []
        plotted = False
        cmap = plt.get_cmap("tab10")
        for idx in selected:
            t = list(self._joint_rt_series_t.get(idx, []))
            y_actual = list(self._joint_rt_series_actual.get(idx, []))
            y_target = list(self._joint_rt_series_target.get(idx, []))
            if not t:
                continue
            plotted = True
            x_max = max(x_max, float(t[-1]))
            base_color = cmap(idx % 10)
            ax.plot(
                t,
                y_actual,
                linestyle="-",
                linewidth=1.8,
                color=base_color,
                label=f"J{idx:02d} Actual",
            )
            ax.plot(
                t,
                y_target,
                linestyle="--",
                linewidth=1.6,
                color=base_color,
                alpha=0.55,
                label=f"J{idx:02d} Target",
            )
            for v in y_target:
                if np.isfinite(v):
                    vals.append(float(v))
            for v in y_actual:
                if np.isfinite(v):
                    vals.append(float(v))

        if x_max > 0.0:
            x_min = max(0.0, x_max - float(self._joint_rt_window_sec))
            ax.set_xlim(x_min, max(x_min + 0.8, x_max + 0.05))
        else:
            ax.set_xlim(0.0, float(self._joint_rt_window_sec))

        if vals:
            y_min = min(vals)
            y_max = max(vals)
            if y_min == y_max:
                y_max = y_min + 1.0
            margin = (y_max - y_min) * 0.12
            ax.set_ylim(y_min - margin, y_max + margin)
        else:
            ax.set_ylim(-1.0, 1.0)

        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        suffix = " (target N/A in motor mode)" if mode == "motor" else ""
        if selected:
            selection_label = ", ".join([f"J{i:02d}" for i in selected])
            ax.set_title(f"Target vs Actual: {selection_label}{suffix}", fontsize=9)
        else:
            ax.set_title(f"Target vs Actual: None selected{suffix}", fontsize=9)
        if plotted:
            ax.legend(loc="upper right", fontsize=8, ncol=2)
        canvas.draw_idle()

    def _setup_teleop_control_panel(self, parent):
        """遥操作模式下的专用右侧控制栏"""
        panel = ttk.LabelFrame(parent, text="Teleoperation Control")
        panel.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # (1) 遥操作子模式选择
        mode_row = ttk.Frame(panel)
        mode_row.pack(fill=tk.X, padx=6, pady=(8, 4))
        ttk.Label(mode_row, text="Teleop Mode:").pack(side=tk.LEFT)
        self._teleop_mode_var = tk.StringVar(value="Mode 1: Visual Retargeting")
        self._teleop_mode_combo = ttk.Combobox(
            mode_row,
            textvariable=self._teleop_mode_var,
            state="readonly",
            width=28,
            values=["Mode 1: Visual Retargeting", "Mode 2: Rokoko Glove"],
        )
        self._teleop_mode_combo.pack(side=tk.LEFT, padx=4)

        # (2) 启动/停止遥操开关
        toggle_row = ttk.Frame(panel)
        toggle_row.pack(fill=tk.X, padx=6, pady=8)
        self._teleop_enable_var = tk.BooleanVar(value=False)
        self._teleop_toggle_btn = ttk.Checkbutton(
            toggle_row,
            text="Start Teleop",
            variable=self._teleop_enable_var,
            command=self._on_toggle_teleop,
        )
        self._teleop_toggle_btn.pack(side=tk.LEFT)

        hint = ttk.Label(
            panel,
            text="Select teleop mode, then turn switch ON to start and OFF to stop.",
            font=("", 9),
        )
        hint.pack(fill=tk.X, padx=6, pady=(4, 8))

    def _on_toggle_teleop(self):
        enabled = bool(self._teleop_enable_var.get())
        mode_name = self._teleop_mode_var.get() if hasattr(self, "_teleop_mode_var") else ""
        if enabled:
            if hasattr(self, "_teleop_toggle_btn"):
                self._teleop_toggle_btn.config(text="Stop Teleop")
            self._emit_action(f"Teleop started [{mode_name}]")
            self.status_var.set(f"Teleop started: {mode_name}")
        else:
            if hasattr(self, "_teleop_toggle_btn"):
                self._teleop_toggle_btn.config(text="Start Teleop")
            self._emit_action("Teleop stopped")
            self.status_var.set("Teleop stopped")

    def _setup_algorithm_control_panel(self, parent):
        """算法模式下的专用右侧控制栏"""
        panel = ttk.LabelFrame(parent, text="Algorithm Control")
        panel.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        top = ttk.Frame(panel)
        top.pack(fill=tk.X, padx=6, pady=(8, 4))
        self._alg_running = not self.controller.is_paused()
        self._alg_toggle_btn = ttk.Button(
            top,
            text="Stop Algorithm" if self._alg_running else "Start Algorithm",
            command=self._on_toggle_algorithm,
        )
        self._alg_toggle_btn.pack(side=tk.LEFT)

        ttk.Label(panel, text="Algorithm Angle Output:").pack(anchor="w", padx=6, pady=(6, 2))
        self._alg_output = tk.Text(panel, height=18, wrap="none")
        self._alg_output.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        self._alg_output.configure(state="disabled")
        self._alg_last_print = 0.0

    def _on_toggle_algorithm(self):
        paused = self.controller.toggle_pause()
        self._alg_running = not paused
        if self._alg_running:
            self._alg_toggle_btn.config(text="Stop Algorithm")
            self.status_var.set("Algorithm started")
            self._emit_action("Algorithm started")
        else:
            self._alg_toggle_btn.config(text="Start Algorithm")
            self.status_var.set("Algorithm stopped")
            self._emit_action("Algorithm stopped")

    def _append_algorithm_output(self, text: str):
        if not hasattr(self, "_alg_output"):
            return
        self._alg_output.configure(state="normal")
        self._alg_output.insert("end", text + "\n")
        # 保留最近 300 行，防止无限增长
        line_count = int(float(self._alg_output.index("end-1c").split(".")[0]))
        if line_count > 300:
            self._alg_output.delete("1.0", f"{line_count - 300}.0")
        self._alg_output.see("end")
        self._alg_output.configure(state="disabled")

    def _build_mode_table(self, parent, is_motor: bool):
        wrap = ttk.Frame(parent)
        wrap.pack(fill=tk.BOTH, expand=True)
        canvas = tk.Canvas(wrap, highlightthickness=0)
        scroll_y = ttk.Scrollbar(wrap, orient=tk.VERTICAL, command=canvas.yview)
        scroll_x = ttk.Scrollbar(parent, orient=tk.HORIZONTAL, command=canvas.xview)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
        canvas.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        inner = ttk.Frame(canvas)
        inner_window = canvas.create_window((0, 0), window=inner, anchor="nw")

        table_scroll_job = {"id": None}

        def _flush_table_scrollregion() -> None:
            table_scroll_job["id"] = None
            try:
                canvas.configure(scrollregion=canvas.bbox("all"))
            except tk.TclError:
                pass

        def _on_inner_configure(_event=None) -> None:
            job = table_scroll_job["id"]
            if job is not None:
                try:
                    self.root.after_cancel(job)
                except tk.TclError:
                    pass
            try:
                table_scroll_job["id"] = self.root.after(80, _flush_table_scrollregion)
            except tk.TclError:
                table_scroll_job["id"] = None

        inner.bind("<Configure>", _on_inner_configure, add="+")
        _on_inner_configure()
        self._bind_canvas_mousewheel(canvas)

        header = ttk.Frame(inner)
        header.pack(fill=tk.X, padx=2, pady=(2, 4))
        if is_motor:
            ttk.Label(header, text="Channel").grid(row=0, column=0, padx=2, sticky="w")
            ttk.Label(header, text="当前(0–4095)").grid(row=0, column=1, padx=2, sticky="w")
            ttk.Label(header, text="目标(0–4095)").grid(row=0, column=2, padx=2, sticky="w")
            ttk.Label(header, text="Action").grid(row=0, column=3, padx=2, sticky="w")
            ttk.Label(header, text="Step / Raw Slider").grid(row=0, column=4, columnspan=3, padx=2, sticky="w")
            ttk.Label(header, text="当前关节角(°)").grid(row=0, column=7, padx=2, sticky="w")
        else:
            ttk.Label(header, text="Channel").grid(row=0, column=0, padx=2, sticky="w")
            ttk.Label(header, text="Encoder").grid(row=0, column=1, padx=2, sticky="w")
            ttk.Label(header, text="Current").grid(row=0, column=2, padx=2, sticky="w")
            ttk.Label(header, text="Min").grid(row=0, column=3, padx=2, sticky="w")
            ttk.Label(header, text="Target").grid(row=0, column=4, padx=2, sticky="w")
            ttk.Label(header, text="Max").grid(row=0, column=5, padx=2, sticky="w")
            ttk.Label(header, text="Action").grid(row=0, column=6, padx=2, sticky="w")
            ttk.Label(header, text="Target Slider").grid(row=0, column=7, padx=2, sticky="w")
            ttk.Label(header, text="tie").grid(row=0, column=8, padx=2, sticky="w")
            ttk.Label(header, text="+/-").grid(row=0, column=9, padx=2, sticky="w")
            ttk.Label(header, text="x1(abs)").grid(row=0, column=10, padx=2, sticky="w")
            ttk.Label(header, text="ServoAbs(now)").grid(row=0, column=11, padx=2, sticky="w")
            ttk.Label(header, text="x1(now)").grid(row=0, column=12, padx=2, sticky="w")

        channel_count = MOTOR_COUNT if is_motor else ENCODER_COUNT
        for i in range(channel_count):
            row = ttk.Frame(inner)
            row.pack(fill=tk.X, padx=2, pady=1)
            label_prefix = "M" if is_motor else "J"
            ttk.Label(row, text=f"{label_prefix}{i:02d}", width=4).grid(row=0, column=0, padx=1, sticky="w")

            curr = tk.StringVar(value="-")

            if is_motor:
                row.columnconfigure(5, weight=1, minsize=220)
                ttk.Label(row, textvariable=curr, width=8).grid(row=0, column=1, padx=1, sticky="w")
                target = tk.IntVar(value=0)
                joint_angle = tk.StringVar(value="-")
                self._motor_curr_vars.append(curr)
                self._motor_joint_angle_vars.append(joint_angle)
                self._motor_target_vars.append(target)
                self._motor_target_user_set.append(False)
                ent = ttk.Entry(row, textvariable=target, width=8)
                ent.grid(row=0, column=2, padx=1, sticky="w")
                ent.bind("<KeyRelease>", lambda _e, idx=i: self._mark_motor_target_user_set(idx))
                ent.bind("<FocusOut>", lambda _e, idx=i: self._mark_motor_target_user_set(idx))
                btn = ttk.Button(row, text="Send", width=5, command=lambda idx=i: self._confirm_motor_row(idx))
                btn.grid(row=0, column=3, padx=1, sticky="w")
                self._motor_run_buttons.append(btn)
                minus_btn = ttk.Button(row, text="-", width=3, command=lambda idx=i: self._step_motor_target(idx, -1))
                minus_btn.grid(row=0, column=4, padx=(1, 0), sticky="w")
                sld = ttk.Scale(
                    row,
                    from_=MOTOR_TARGET_SCALE_LO,
                    to=MOTOR_TARGET_SCALE_HI,
                    variable=target,
                    orient=tk.HORIZONTAL,
                    length=220,
                    command=lambda value, idx=i: self._on_motor_slider_change(idx, value),
                )
                sld.grid(row=0, column=5, padx=1, sticky="we")
                sld.bind("<ButtonPress-1>", lambda e, idx=i: self._on_motor_slider_press(idx, e), add="+")
                sld.bind("<ButtonRelease-1>", lambda e, idx=i: self._on_motor_slider_release(idx, e), add="+")
                plus_btn = ttk.Button(row, text="+", width=3, command=lambda idx=i: self._step_motor_target(idx, 1))
                plus_btn.grid(row=0, column=6, padx=(0, 1), sticky="w")
                ttk.Label(row, textvariable=joint_angle, width=12).grid(row=0, column=7, padx=(6, 1), sticky="w")
                self._control_widgets.extend([ent, btn, minus_btn, sld, plus_btn])
            else:
                row.columnconfigure(7, weight=1, minsize=220)
                dot_canvas = tk.Canvas(row, width=12, height=12, highlightthickness=0, bd=0)
                dot_item = dot_canvas.create_oval(2, 2, 10, 10, fill="#9aa0a6", outline="#9aa0a6")
                dot_canvas.grid(row=0, column=1, padx=2, sticky="w")
                self._joint_status_dots.append((dot_canvas, dot_item))

                ttk.Label(row, textvariable=curr, width=8).grid(row=0, column=2, padx=1, sticky="w")
                joint_min = self._joint_target_min_deg(i)
                joint_max = self._joint_target_max_deg(i)
                target = tk.DoubleVar(value=joint_min)
                self._joint_curr_vars.append(curr)
                self._joint_target_vars.append(target)
                self._joint_target_user_set.append(False)
                ttk.Label(row, text=f"{joint_min:.2f}", width=6).grid(row=0, column=3, padx=1, sticky="w")
                ent = ttk.Entry(row, textvariable=target, width=8)
                ent.grid(row=0, column=4, padx=1, sticky="w")
                ent.bind("<KeyRelease>", lambda _e, idx=i: self._mark_joint_target_user_set(idx))
                ent.bind("<FocusOut>", lambda _e, idx=i: self._mark_joint_target_user_set(idx))
                ttk.Label(row, text=f"{joint_max:.2f}", width=6).grid(row=0, column=5, padx=1, sticky="w")
                btn = ttk.Button(row, text="Send", width=5, command=lambda idx=i: self._confirm_joint_row(idx))
                btn.grid(row=0, column=6, padx=1, sticky="w")
                self._joint_run_buttons.append(btn)
                sld = ttk.Scale(
                    row,
                    from_=joint_min,
                    to=joint_max,
                    variable=target,
                    orient=tk.HORIZONTAL,
                    length=220,
                    command=lambda value, idx=i: self._on_joint_slider_change(idx, value),
                )
                sld.grid(row=0, column=7, padx=1, sticky="we")
                self._joint_sliders.append(sld)
                ttk.Label(row, text="tie", width=4).grid(row=0, column=8, padx=(8, 2), sticky="w")
                tie_sign_cfg = self._tendon_cfg[i] if i < len(self._tendon_cfg) else {"enabled": False, "pull_sign": 1, "x1_abs": 0}
                tie_sign = "+" if bool(tie_sign_cfg.get("enabled", False)) and int(tie_sign_cfg.get("pull_sign", 1)) >= 0 else (
                    "-" if bool(tie_sign_cfg.get("enabled", False)) else ""
                )
                tie_sign_var = tk.StringVar(value=tie_sign)
                tie_x1_var = tk.StringVar(value=str(int(tie_sign_cfg.get("x1_abs", 0))))
                self._joint_tie_sign_vars.append(tie_sign_var)
                self._joint_tie_x1_vars.append(tie_x1_var)
                tie_sign_ent = ttk.Entry(row, textvariable=tie_sign_var, width=3)
                tie_sign_ent.grid(row=0, column=9, padx=1, sticky="w")
                tie_x1_ent = ttk.Entry(row, textvariable=tie_x1_var, width=8)
                tie_x1_ent.grid(row=0, column=10, padx=1, sticky="w")
                tie_motor_abs_live_var = tk.StringVar(value="-")
                tie_x1_live_var = tk.StringVar(value="-")
                self._joint_tie_motor_abs_vars.append(tie_motor_abs_live_var)
                self._joint_tie_x1_live_vars.append(tie_x1_live_var)
                ttk.Label(row, textvariable=tie_motor_abs_live_var, width=12).grid(
                    row=0, column=11, padx=(6, 1), sticky="w"
                )
                ttk.Label(row, textvariable=tie_x1_live_var, width=10).grid(
                    row=0, column=12, padx=1, sticky="w"
                )
                tie_sign_ent.bind("<FocusOut>", lambda _e, idx=i: self._on_joint_tie_field_commit(idx))
                tie_x1_ent.bind("<FocusOut>", lambda _e, idx=i: self._on_joint_tie_field_commit(idx))
                tie_sign_ent.bind("<Return>", lambda _e, idx=i: self._on_joint_tie_field_commit(idx))
                tie_x1_ent.bind("<Return>", lambda _e, idx=i: self._on_joint_tie_field_commit(idx))
                self._control_widgets.extend([ent, btn, sld])

    def _setup_test_panel(self, parent) -> None:
        box = ttk.LabelFrame(parent, text="Joint Sine Test")
        box.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        ttk.Label(
            box,
            text="Configure up to 5 joints (None allowed). Start Test sends periodic sine targets in joint PID mode.",
            font=("", 9),
        ).pack(fill=tk.X, padx=6, pady=(6, 4))

        grid = ttk.Frame(box)
        grid.pack(fill=tk.X, padx=6, pady=(2, 4))
        ttk.Label(grid, text="#", width=2).grid(row=0, column=0, padx=2, sticky="w")
        ttk.Label(grid, text="Joint", width=9).grid(row=0, column=1, padx=2, sticky="w")
        ttk.Label(grid, text="Min(deg)", width=10).grid(row=0, column=2, padx=2, sticky="w")
        ttk.Label(grid, text="Max(deg)", width=10).grid(row=0, column=3, padx=2, sticky="w")
        ttk.Label(grid, text="Freq(Hz)", width=9).grid(row=0, column=4, padx=2, sticky="w")
        ttk.Label(grid, text="Phase(deg)", width=11).grid(row=0, column=5, padx=2, sticky="w")

        joint_choices = ["None"] + [f"J{i:02d}" for i in range(ENCODER_COUNT)]
        self._test_joint_vars = []
        self._test_min_vars = []
        self._test_max_vars = []
        self._test_freq_vars = []
        self._test_phase_vars = []
        for row_idx in range(TEST_ROW_COUNT):
            ttk.Label(grid, text=f"{row_idx + 1}", width=2).grid(row=row_idx + 1, column=0, padx=2, pady=1, sticky="w")
            joint_var = tk.StringVar(value="None")
            min_var = tk.StringVar(value="0.0")
            max_var = tk.StringVar(value="10.0")
            freq_var = tk.StringVar(value=f"{TEST_DEFAULT_FREQ_HZ:.2f}")
            phase_var = tk.StringVar(value=f"{TEST_DEFAULT_PHASE_DEG:.1f}")
            self._test_joint_vars.append(joint_var)
            self._test_min_vars.append(min_var)
            self._test_max_vars.append(max_var)
            self._test_freq_vars.append(freq_var)
            self._test_phase_vars.append(phase_var)

            joint_combo = ttk.Combobox(grid, textvariable=joint_var, values=joint_choices, state="readonly", width=8)
            joint_combo.grid(row=row_idx + 1, column=1, padx=2, pady=1, sticky="w")
            joint_combo.bind("<<ComboboxSelected>>", lambda _e, idx=row_idx: self._on_test_joint_changed(idx))
            min_ent = ttk.Entry(grid, textvariable=min_var, width=11)
            min_ent.grid(row=row_idx + 1, column=2, padx=2, pady=1, sticky="w")
            max_ent = ttk.Entry(grid, textvariable=max_var, width=11)
            max_ent.grid(row=row_idx + 1, column=3, padx=2, pady=1, sticky="w")
            freq_ent = ttk.Entry(grid, textvariable=freq_var, width=10)
            freq_ent.grid(row=row_idx + 1, column=4, padx=2, pady=1, sticky="w")
            phase_ent = ttk.Entry(grid, textvariable=phase_var, width=11)
            phase_ent.grid(row=row_idx + 1, column=5, padx=2, pady=1, sticky="w")
            self._control_widgets.extend([joint_combo, min_ent, max_ent, freq_ent, phase_ent])

        action_row = ttk.Frame(box)
        action_row.pack(fill=tk.X, padx=6, pady=(6, 6))
        self._test_start_btn = ttk.Button(action_row, text="Start Test", width=11, command=self._on_start_test)
        self._test_start_btn.pack(side=tk.LEFT, padx=(0, 6))
        self._test_stop_btn = ttk.Button(action_row, text="Stop Test", width=11, command=self._on_stop_test)
        self._test_stop_btn.pack(side=tk.LEFT)
        ttk.Label(action_row, text="Initial Hold(s)").pack(side=tk.LEFT, padx=(10, 4))
        hold_ent = ttk.Entry(action_row, textvariable=self._test_hold_s_var, width=8)
        hold_ent.pack(side=tk.LEFT)
        self._control_widgets.append(hold_ent)
        self._set_test_button_states()

    @staticmethod
    def _normalize_test_joint_value(value: str) -> str:
        text = (value or "").strip().upper()
        if text in ("", "NONE", "N", "-"):
            return "None"
        if text.startswith("J"):
            try:
                idx = int(text[1:])
            except ValueError:
                return "None"
            if 0 <= idx < ENCODER_COUNT:
                return f"J{idx:02d}"
        return "None"

    def _on_test_joint_changed(self, row_idx: int) -> None:
        if row_idx < 0 or row_idx >= len(self._test_joint_vars):
            return
        current = self._normalize_test_joint_value(self._test_joint_vars[row_idx].get())
        self._test_joint_vars[row_idx].set(current)
        if current == "None":
            return
        for i, var in enumerate(self._test_joint_vars):
            if i == row_idx:
                continue
            if self._normalize_test_joint_value(var.get()) == current:
                self._test_joint_vars[row_idx].set("None")
                if hasattr(self, "status_var"):
                    self.status_var.set(f"Test config duplicate joint: {current} already selected")
                return

    def _validate_test_rows(self, show_message: bool = True) -> Optional[List[Tuple[int, float, float, float, float]]]:
        active: List[Tuple[int, float, float, float, float]] = []
        selected = set()
        n = min(
            TEST_ROW_COUNT,
            len(self._test_joint_vars),
            len(self._test_min_vars),
            len(self._test_max_vars),
            len(self._test_freq_vars),
            len(self._test_phase_vars),
        )
        for row_idx in range(n):
            joint_text = self._normalize_test_joint_value(self._test_joint_vars[row_idx].get())
            self._test_joint_vars[row_idx].set(joint_text)
            if joint_text == "None":
                continue
            joint_idx = int(joint_text[1:])
            if joint_idx in selected:
                msg = f"Test row {row_idx + 1}: duplicated {joint_text}"
                if show_message:
                    messagebox.showwarning("Test Config", msg)
                return None
            selected.add(joint_idx)

            try:
                raw_min = float(self._test_min_vars[row_idx].get())
                raw_max = float(self._test_max_vars[row_idx].get())
                freq = float(self._test_freq_vars[row_idx].get())
                phase_deg = float(self._test_phase_vars[row_idx].get())
            except (TypeError, ValueError, tk.TclError):
                msg = f"Test row {row_idx + 1}: invalid numeric input"
                if show_message:
                    messagebox.showwarning("Test Config", msg)
                return None

            if not np.isfinite(raw_min) or not np.isfinite(raw_max) or not np.isfinite(freq) or not np.isfinite(phase_deg):
                msg = f"Test row {row_idx + 1}: min/max/freq/phase must be finite numbers"
                if show_message:
                    messagebox.showwarning("Test Config", msg)
                return None
            if freq <= 0.0:
                msg = f"Test row {row_idx + 1}: frequency must be > 0"
                if show_message:
                    messagebox.showwarning("Test Config", msg)
                return None
            if phase_deg < 0.0 or phase_deg > 360.0:
                msg = f"Test row {row_idx + 1}: phase must be in [0, 360] deg"
                if show_message:
                    messagebox.showwarning("Test Config", msg)
                return None

            lo_raw = min(raw_min, raw_max)
            hi_raw = max(raw_min, raw_max)
            lo = self._clamp_joint_target_deg(joint_idx, lo_raw)
            hi = self._clamp_joint_target_deg(joint_idx, hi_raw)
            if hi < lo:
                lo, hi = hi, lo
            self._test_min_vars[row_idx].set(f"{lo:.2f}")
            self._test_max_vars[row_idx].set(f"{hi:.2f}")
            self._test_freq_vars[row_idx].set(f"{freq:.4g}")
            self._test_phase_vars[row_idx].set(f"{phase_deg:.4g}")
            active.append((joint_idx, lo, hi, float(freq), float(phase_deg)))

        if not active:
            if show_message:
                messagebox.showwarning("Test Config", "Select at least one valid joint (not None).")
            return None
        return active

    def _set_test_button_states(self) -> None:
        start_btn = getattr(self, "_test_start_btn", None)
        stop_btn = getattr(self, "_test_stop_btn", None)
        if start_btn is not None:
            try:
                start_btn.configure(state="disabled" if self._test_running else "normal")
            except Exception:
                pass
        if stop_btn is not None:
            try:
                stop_btn.configure(state="normal" if self._test_running else "disabled")
            except Exception:
                pass

    def _schedule_test_tick(self) -> None:
        if not self._test_running:
            return
        self._test_after_job = self.root.after(int(self._test_tick_ms), self._test_tick)

    def _on_start_test(self) -> None:
        if self._test_running:
            return
        if not self._interaction_enabled:
            if hasattr(self, "status_var"):
                self.status_var.set("Start Test blocked: Hand Manual must be ON")
            return
        if not self._guard_joint_command_started():
            return
        active = self._validate_test_rows(show_message=True)
        if not active:
            return
        try:
            hold_s = float(self._test_hold_s_var.get())
        except (TypeError, ValueError, tk.TclError):
            messagebox.showwarning("Test Config", "Initial Hold(s) must be a valid number.")
            return
        if (not np.isfinite(hold_s)) or hold_s < 0.0:
            messagebox.showwarning("Test Config", "Initial Hold(s) must be a finite number >= 0.")
            return
        self._test_hold_s = float(hold_s)
        self._test_hold_s_var.set(f"{self._test_hold_s:.4g}")

        self._test_active_rows = active
        self._test_running = True
        self._test_t0 = time.time()
        self._set_test_button_states()
        self._emit_action("Test started")
        if hasattr(self, "status_var"):
            self.status_var.set("Test running")
        self._test_tick()

    def _on_stop_test(self, reason: str = "stopped by test", update_status: bool = True) -> None:
        was_running = bool(self._test_running)
        self._test_running = False
        job = getattr(self, "_test_after_job", None)
        self._test_after_job = None
        if job is not None:
            try:
                self.root.after_cancel(job)
            except Exception:
                pass
        self._test_active_rows = []
        self._set_test_button_states()
        if self._data_logger.is_running():
            self._stop_data_recording(reason=reason, keep_switch=True)
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(False)
        if was_running:
            self._emit_action(f"Test stopped ({reason})")
        if update_status and hasattr(self, "status_var"):
            self.status_var.set("Test stopped")

    def _test_tick(self) -> None:
        if not self._test_running:
            return
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        if mode != "test":
            self._on_stop_test(reason="stopped on mode switch", update_status=False)
            return
        if not self._interaction_enabled:
            self._on_stop_test(reason="stopped: Hand Manual OFF")
            return
        if not self.controller.is_started():
            self._on_stop_test(reason="stopped: press Enable to start controller")
            return
        if not self._test_active_rows:
            self._on_stop_test(reason="stopped: no active test rows")
            return

        elapsed = max(0.0, time.time() - float(self._test_t0))
        angles = self._resolved_joint_targets()
        for joint_idx, lo, hi, freq, phase_deg in self._test_active_rows:
            center = 0.5 * (lo + hi)
            amp = 0.5 * (hi - lo)
            phi = np.deg2rad(float(phase_deg))
            initial_target = center + amp * float(np.sin(phi))
            if elapsed < float(self._test_hold_s):
                target = initial_target
            else:
                run_t = elapsed - float(self._test_hold_s)
                target = center + amp * float(np.sin((2.0 * np.pi * float(freq) * run_t) + phi))
            target = self._clamp_joint_target_deg(joint_idx, target)
            angles[joint_idx] = target
            if joint_idx < len(self._joint_target_vars):
                self._joint_target_vars[joint_idx].set(target)
            if joint_idx < len(self._joint_target_user_set):
                self._joint_target_user_set[joint_idx] = True

        try:
            self._send_joint_targets_with_tendon_guard(angles, source="Test", live=True)
        except Exception as exc:
            self._on_stop_test(reason=f"stopped by send error: {exc}")
            return
        self._schedule_test_tick()

    def _switch_control_mode(self):
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        prev_mode = getattr(self, "_ctrl_mode_last", mode)
        if prev_mode == "test" and mode != "test":
            self._on_stop_test(reason="stopped on mode switch", update_status=False)
        self._clear_run_states()
        self._joint_panel.pack_forget()
        self._motor_panel.pack_forget()
        if hasattr(self, "_test_panel"):
            self._test_panel.pack_forget()
        if mode == "motor":
            if hasattr(self, "_motor_dir_hint"):
                self._motor_dir_hint = [0] * MOTOR_COUNT
            if hasattr(self, "_motor_dir_hint_pending_clear"):
                self._motor_dir_hint_pending_clear = [False] * MOTOR_COUNT
            if hasattr(self, "_motor_dir_hint_stable_count"):
                self._motor_dir_hint_stable_count = [0] * MOTOR_COUNT
            if hasattr(self, "_motor_first_drag_limit_active"):
                self._motor_first_drag_limit_active = [True] * MOTOR_COUNT
            self._motor_panel.pack(fill=tk.BOTH, expand=True)
            self._refresh_motor_feedback_view(sync_target=True)
            if hasattr(self, "_table_section"):
                self._table_section.configure(text=f"Motor Channels (M{MOTOR_COUNT})")
        elif mode == "test":
            if hasattr(self, "_test_panel"):
                self._test_panel.pack(fill=tk.BOTH, expand=True)
            if hasattr(self, "_table_section"):
                self._table_section.configure(text="Joint Sine Test")
        else:
            self._joint_panel.pack(fill=tk.BOTH, expand=True)
            self._refresh_joint_feedback_view(sync_target=True)
            if hasattr(self, "_table_section"):
                self._table_section.configure(text=f"Joint Channels (J{ENCODER_COUNT})")
        if prev_mode in ("joint", "test") and mode == "motor":
            self._stop_joint_pid_on_motor_switch()
        self._ctrl_mode_last = mode
        self._update_manual_action_buttons()
        self._apply_interaction_control_policy()

    def _stop_joint_pid_on_motor_switch(self) -> None:
        if not self._is_comm_connected():
            return
        was_started = bool(self.controller.is_started())
        if was_started:
            try:
                self.controller.stop()
            except Exception:
                pass
        cleared = int(self.controller.invalidate_joint_commands())
        self._refresh_start_indicator()
        action = "Auto stop: joint PID halted on motor mode switch"
        if cleared > 0:
            action = f"{action}; cleared {cleared} pending joint command(s)"
        self._emit_action(action)
        if hasattr(self, "status_var"):
            base = "Switched to motor mode: old joint commands invalidated"
            if was_started:
                base = f"{base}, joint PID stopped"
            if cleared > 0:
                base = f"{base} (cleared={cleared})"
            self.status_var.set(base)

    def _mark_joint_target_user_set(self, idx: int) -> None:
        if 0 <= idx < len(self._joint_target_user_set):
            self._joint_target_user_set[idx] = True

    def _mark_motor_target_user_set(self, idx: int) -> None:
        if 0 <= idx < len(self._motor_target_user_set):
            self._motor_target_user_set[idx] = True

    def _joint_target_max_deg(self, idx: int) -> float:
        if 0 <= idx < len(JOINT_TARGET_MAX_DEG_BY_INDEX):
            return float(JOINT_TARGET_MAX_DEG_BY_INDEX[idx])
        return float(JOINT_TARGET_MAX_DEG_BY_INDEX[-1])

    def _joint_target_min_deg(self, idx: int) -> float:
        if 0 <= idx < len(JOINT_TARGET_MIN_DEG_BY_INDEX):
            return float(JOINT_TARGET_MIN_DEG_BY_INDEX[idx])
        return float(JOINT_TARGET_MIN_DEG_BY_INDEX[-1])

    def _clamp_joint_target_deg(self, idx: int, value: float) -> float:
        try:
            v = float(value)
        except Exception:
            v = self._joint_target_min_deg(idx)
        if not np.isfinite(v):
            v = self._joint_target_min_deg(idx)
        vmin = self._joint_target_min_deg(idx)
        vmax = self._joint_target_max_deg(idx)
        if vmin > vmax:
            vmin, vmax = vmax, vmin
        if v < vmin:
            return vmin
        if v > vmax:
            return vmax
        return v

    def _joint_feedback_target(self, idx: int) -> float:
        with self._state_lock:
            s = self._state
        if s and bool(getattr(s, "has_sensor_data", False)) and idx < len(s.angles):
            return float(s.angles[idx])
        if idx < len(self._joint_target_vars):
            try:
                return float(self._joint_target_vars[idx].get())
            except Exception:
                return self._joint_target_min_deg(idx)
        return self._joint_target_min_deg(idx)

    def _motor_feedback_target(self, idx: int) -> int:
        with self._state_lock:
            s = self._state
        if s and bool(getattr(s, "has_servo_raw_data", False)) and idx < len(s.servo_raw_positions):
            return int(self._motor_target_from_servo_feedback(s.servo_raw_positions[idx]))
        if idx < len(self._motor_target_vars):
            try:
                return int(self._motor_target_vars[idx].get())
            except Exception:
                return 0
        return 0

    def _resolved_joint_targets(self) -> List[float]:
        targets: List[float] = []
        for i in range(ENCODER_COUNT):
            if i < len(self._joint_target_vars) and i < len(self._joint_target_user_set) and self._joint_target_user_set[i]:
                try:
                    raw = float(self._joint_target_vars[i].get())
                except Exception:
                    raw = self._joint_feedback_target(i)
            else:
                raw = self._joint_feedback_target(i)
            clamped = self._clamp_joint_target_deg(i, raw)
            if i < len(self._joint_target_vars):
                try:
                    self._joint_target_vars[i].set(clamped)
                except Exception:
                    pass
            targets.append(clamped)
        return targets

    @staticmethod
    def _default_tendon_cfg() -> List[dict]:
        return [{"enabled": False, "pull_sign": 1, "x1_abs": 0} for _ in range(ENCODER_COUNT)]

    def _load_tendon_manual_cfg(self) -> List[dict]:
        defaults = self._default_tendon_cfg()
        p = getattr(self, "_tendon_cfg_path", "")
        if not p or not os.path.isfile(p):
            return defaults
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return defaults
        joints = data.get("joints", [])
        if not isinstance(joints, list):
            return defaults

        out = []
        for i in range(ENCODER_COUNT):
            item = joints[i] if i < len(joints) and isinstance(joints[i], dict) else {}
            try:
                sign_raw = int(item.get("pull_sign", 1))
            except (TypeError, ValueError):
                sign_raw = 1
            sign = 1 if sign_raw >= 0 else -1
            try:
                x1 = int(item.get("x1_abs", 0))
            except (TypeError, ValueError):
                x1 = 0
            x1 = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, x1))
            enabled = bool(item.get("enabled", False))
            out.append({"enabled": enabled, "pull_sign": sign, "x1_abs": x1})
        return out

    def _save_tendon_manual_cfg(self) -> bool:
        p = getattr(self, "_tendon_cfg_path", "")
        if not p:
            return False
        os.makedirs(os.path.dirname(p), exist_ok=True)
        joints = []
        for i in range(ENCODER_COUNT):
            item = self._tendon_cfg[i] if i < len(self._tendon_cfg) else {}
            try:
                sign_raw = int(item.get("pull_sign", 1))
            except (TypeError, ValueError):
                sign_raw = 1
            sign = 1 if sign_raw >= 0 else -1
            try:
                x1 = int(item.get("x1_abs", 0))
            except (TypeError, ValueError):
                x1 = 0
            x1 = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, x1))
            enabled = bool(item.get("enabled", False))
            joints.append({"enabled": enabled, "pull_sign": sign, "x1_abs": x1})
        payload = {
            "comment": "Manual tendon calibration (+/- pull direction and x1 absolute anchor).",
            "joints": joints,
        }
        try:
            with open(p, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
            return True
        except OSError:
            return False

    @staticmethod
    def _parse_tie_sign_text(text: str) -> int:
        t = (text or "").strip()
        if t in ("+", "+1", "1"):
            return 1
        if t in ("-", "-1"):
            return -1
        return 0

    def _on_joint_tie_field_commit(self, joint_idx: int) -> None:
        if joint_idx < 0 or joint_idx >= ENCODER_COUNT:
            return
        if joint_idx >= len(self._joint_tie_sign_vars) or joint_idx >= len(self._joint_tie_x1_vars):
            return
        sign_text = self._joint_tie_sign_vars[joint_idx].get()
        sign = self._parse_tie_sign_text(sign_text)
        if sign > 0:
            self._joint_tie_sign_vars[joint_idx].set("+")
        elif sign < 0:
            self._joint_tie_sign_vars[joint_idx].set("-")
        else:
            self._joint_tie_sign_vars[joint_idx].set("")

        try:
            x1 = int(float(self._joint_tie_x1_vars[joint_idx].get()))
        except (TypeError, ValueError, tk.TclError):
            x1 = 0
        x1 = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, x1))
        self._joint_tie_x1_vars[joint_idx].set(str(x1))

        if joint_idx >= len(self._tendon_cfg):
            self._tendon_cfg.extend(
                [{"enabled": False, "pull_sign": 1, "x1_abs": 0} for _ in range(joint_idx - len(self._tendon_cfg) + 1)]
            )
        self._tendon_cfg[joint_idx] = {"enabled": sign != 0, "pull_sign": (1 if sign >= 0 else -1), "x1_abs": x1}
        saved = self._save_tendon_manual_cfg()
        if hasattr(self, "status_var"):
            if not saved:
                self.status_var.set(f"tie save failed: J{joint_idx}")
            elif sign == 0:
                self.status_var.set(f"tie saved: J{joint_idx} disabled, x1={x1}")
            else:
                self.status_var.set(f"tie saved: J{joint_idx} sign={'+' if sign > 0 else '-'} x1={x1}")
        self._sync_tendon_guard_to_lower(force=True)

    def _refresh_joint_tie_cfg_from_ui(self) -> None:
        n = min(ENCODER_COUNT, len(self._joint_tie_sign_vars), len(self._joint_tie_x1_vars))
        for i in range(n):
            sign = self._parse_tie_sign_text(self._joint_tie_sign_vars[i].get())
            try:
                x1 = int(float(self._joint_tie_x1_vars[i].get()))
            except (TypeError, ValueError, tk.TclError):
                x1 = 0
            x1 = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, x1))
            if i >= len(self._tendon_cfg):
                self._tendon_cfg.extend(
                    [{"enabled": False, "pull_sign": 1, "x1_abs": 0} for _ in range(i - len(self._tendon_cfg) + 1)]
                )
            self._tendon_cfg[i] = {"enabled": sign != 0, "pull_sign": (1 if sign >= 0 else -1), "x1_abs": x1}

    def _build_tendon_guard_payload(self) -> List[Tuple[bool, int, int]]:
        payload: List[Tuple[bool, int, int]] = []
        for i in range(ENCODER_COUNT):
            cfg = self._tendon_cfg[i] if i < len(self._tendon_cfg) else {}
            enabled = bool(cfg.get("enabled", False))
            sign = 1 if int(cfg.get("pull_sign", 1)) >= 0 else -1
            try:
                x1 = int(cfg.get("x1_abs", 0))
            except (TypeError, ValueError):
                x1 = 0
            x1 = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, x1))
            payload.append((enabled, sign, x1))
        return payload

    def _sync_tendon_guard_to_lower(self, force: bool = False) -> None:
        self._refresh_joint_tie_cfg_from_ui()
        payload = self._build_tendon_guard_payload()
        fingerprint = tuple((1 if e else 0, int(s), int(x)) for e, s, x in payload)
        if (not force) and (fingerprint == self._last_tendon_guard_sent):
            return
        if not self._is_comm_connected():
            return
        try:
            self.controller.set_tendon_guard_config(payload)
            self._last_tendon_guard_sent = fingerprint
        except Exception:
            pass

    @staticmethod
    def _joint_primary_motor_channel(joint_index: int) -> int:
        if joint_index < 0 or joint_index >= ENCODER_COUNT:
            return -1
        if joint_index >= len(JOINT_TO_PRIMARY_MOTOR_INDEX):
            return -1
        return int(JOINT_TO_PRIMARY_MOTOR_INDEX[joint_index])

    def _send_joint_targets_with_tendon_guard(self,
                                              angles: List[float],
                                              source: str = "",
                                              live: bool = False) -> None:
        self._refresh_joint_tie_cfg_from_ui()
        self._sync_tendon_guard_to_lower(force=False)
        guarded = list(angles[:ENCODER_COUNT])
        if len(guarded) < ENCODER_COUNT:
            guarded.extend([0.0] * (ENCODER_COUNT - len(guarded)))

        with self._state_lock:
            s = self._state

        blocked: List[int] = []
        if (
            s
            and bool(getattr(s, "has_sensor_data", False))
            and bool(getattr(s, "has_servo_angle_data", False))
        ):
            for joint_idx in range(ENCODER_COUNT):
                cfg = self._tendon_cfg[joint_idx] if joint_idx < len(self._tendon_cfg) else None
                if not cfg or not bool(cfg.get("enabled", False)):
                    continue
                sign = 1 if int(cfg.get("pull_sign", 1)) >= 0 else -1
                x1_abs = int(cfg.get("x1_abs", 0))
                motor_ch = self._joint_primary_motor_channel(joint_idx)
                if motor_ch < 0:
                    continue
                if (
                    joint_idx >= len(getattr(s, "angles", []))
                    or motor_ch >= len(getattr(s, "servo_angles", []))
                    or motor_ch >= len(getattr(s, "servo_online", []))
                    or not s.servo_online[motor_ch]
                ):
                    continue

                cur_joint_deg = float(s.angles[joint_idx])
                target_joint_deg = float(guarded[joint_idx])
                release_cmd = target_joint_deg < (cur_joint_deg - self._tendon_release_angle_eps_deg)
                if not release_cmd:
                    continue

                abs_now = int(s.servo_angles[motor_ch])
                hit_boundary = (
                    (sign > 0 and abs_now <= (x1_abs + self._tendon_release_margin_counts))
                    or (sign < 0 and abs_now >= (x1_abs - self._tendon_release_margin_counts))
                )
                if hit_boundary:
                    guarded[joint_idx] = cur_joint_deg
                    blocked.append(joint_idx)

        if blocked:
            now = time.time()
            if now - self._tendon_guard_last_warn_ts >= self._tendon_guard_warn_interval_s:
                self._tendon_guard_last_warn_ts = now
                if hasattr(self, "status_var"):
                    blk = ", ".join([f"J{i}" for i in blocked])
                    suffix = f" ({source})" if source else ""
                    self.status_var.set(f"Tendon guard blocked release on {blk}{suffix}")

        if live:
            self.controller.set_target_angles_live(guarded)
        else:
            self.controller.set_target_angles(guarded)

    def _refresh_joint_feedback_view(self, sync_target: bool = False) -> None:
        if not hasattr(self, "_joint_curr_vars"):
            return
        with self._state_lock:
            s = self._state
        if not s:
            return

        # Keep tie runtime display aligned with what command send path will parse.
        self._refresh_joint_tie_cfg_from_ui()
        joint_has_data = bool(getattr(s, "has_sensor_data", False))
        motor_has_raw = bool(getattr(s, "has_servo_raw_data", False))
        motor_has_abs = bool(getattr(s, "has_servo_angle_data", False))
        for i in range(min(ENCODER_COUNT, len(self._joint_curr_vars))):
            fallback_text = "-"
            motor_ch = self._joint_primary_motor_channel(i)
            if (
                motor_has_raw
                and motor_ch >= 0
                and motor_ch < len(getattr(s, "servo_raw_positions", []))
                and motor_ch < len(getattr(s, "servo_raw_online", []))
                and bool(s.servo_raw_online[motor_ch])
            ):
                fallback_text = f"{self._motor_target_from_servo_feedback(s.servo_raw_positions[motor_ch]):4d}"
            elif (
                motor_has_abs
                and motor_ch >= 0
                and motor_ch < len(getattr(s, "servo_angles", []))
                and motor_ch < len(getattr(s, "servo_online", []))
                and bool(s.servo_online[motor_ch])
            ):
                fallback_text = f"{self._slider_from_abs_pos(int(s.servo_angles[motor_ch])):4d}"

            encoder_valid = (
                joint_has_data
                and i < len(getattr(s, "angles", []))
                and i < len(getattr(s, "encoders", []))
                and (not bool(s.encoders[i].error))
            )
            if fallback_text != "-":
                self._joint_curr_vars[i].set(fallback_text)
            elif encoder_valid:
                self._joint_curr_vars[i].set(f"{s.angles[i]:6.1f}")
            else:
                self._joint_curr_vars[i].set("-")

            if hasattr(self, "_joint_status_dots"):
                if joint_has_data and i < len(s.encoders):
                    self._set_joint_encoder_dot(i, not bool(s.encoders[i].error))
                else:
                    self._set_joint_encoder_dot(i, None)

            if i < len(getattr(self, "_joint_tie_motor_abs_vars", [])):
                motor_abs_text = "-"
                motor_ch = self._joint_primary_motor_channel(i)
                if (
                    motor_has_abs
                    and motor_ch >= 0
                    and motor_ch < len(getattr(s, "servo_angles", []))
                    and motor_ch < len(getattr(s, "servo_online", []))
                    and bool(s.servo_online[motor_ch])
                ):
                    motor_abs_text = str(int(s.servo_angles[motor_ch]))
                self._joint_tie_motor_abs_vars[i].set(motor_abs_text)

            if i < len(getattr(self, "_joint_tie_x1_live_vars", [])):
                x1_live_text = "-"
                if i < len(getattr(self, "_tendon_cfg", [])):
                    cfg = self._tendon_cfg[i]
                    try:
                        x1 = int(cfg.get("x1_abs", 0))
                    except Exception:
                        x1 = 0
                    sign = 1 if int(cfg.get("pull_sign", 1)) >= 0 else -1
                    if bool(cfg.get("enabled", False)):
                        x1_live_text = f"{'+' if sign > 0 else '-'}{x1}"
                    else:
                        x1_live_text = f"off {x1}"
                self._joint_tie_x1_live_vars[i].set(x1_live_text)

        if joint_has_data and sync_target:
            for i in range(min(ENCODER_COUNT, len(s.angles), len(self._joint_target_vars))):
                self._joint_target_vars[i].set(self._clamp_joint_target_deg(i, float(s.angles[i])))
                if i < len(self._joint_target_user_set):
                    self._joint_target_user_set[i] = False
            self._joint_target_initialized = True

    def _refresh_motor_feedback_view(self, sync_target: bool = False) -> None:
        if not hasattr(self, "_motor_curr_vars"):
            return
        with self._state_lock:
            s = self._state
        if not s:
            return

        motor_has_raw = bool(getattr(s, "has_servo_raw_data", False))
        motor_has_abs = bool(getattr(s, "has_servo_angle_data", False))
        for i in range(min(MOTOR_COUNT, len(self._motor_curr_vars))):
            if (
                motor_has_raw
                and i < len(s.servo_raw_positions)
                and i < len(s.servo_raw_online)
                and s.servo_raw_online[i]
            ):
                st = self._motor_target_from_servo_feedback(s.servo_raw_positions[i])
                self._motor_curr_vars[i].set(f"{st:4d}")
            elif motor_has_abs and i < len(s.servo_angles) and i < len(s.servo_online) and s.servo_online[i]:
                ab = self._clamp_motor_abs_cmd(int(s.servo_angles[i]))
                st = self._slider_from_abs_pos(ab)
                self._motor_curr_vars[i].set(f"{st:4d}")
            else:
                self._motor_curr_vars[i].set("-")

            if i < len(self._motor_joint_angle_vars):
                joint_idx = MOTOR_TO_JOINT_INDEX[i] if i < len(MOTOR_TO_JOINT_INDEX) else -1
                if (
                    joint_idx >= 0
                    and bool(getattr(s, "has_sensor_data", False))
                    and joint_idx < len(s.angles)
                ):
                    self._motor_joint_angle_vars[i].set(f"J{joint_idx:02d}: {s.angles[joint_idx]:5.1f}")
                else:
                    self._motor_joint_angle_vars[i].set("-")

        # 仅首帧 sync_target：一次性用反馈对齐 abs + 滑条（之后不再自动改滑条）。
        if (motor_has_raw or motor_has_abs) and sync_target:
            if bool(getattr(s, "has_servo_angle_data", False)):
                for i in range(min(MOTOR_COUNT, len(self._motor_target_vars))):
                    if i < len(s.servo_online) and s.servo_online[i] and i < len(s.servo_angles):
                        dev_abs = self._clamp_motor_abs_cmd(int(s.servo_angles[i]))
                        self._motor_session_origin_abs[i] = dev_abs
                        self._motor_abs_pos[i] = 0
                        sl = self._slider_from_abs_pos(dev_abs)
                        self._motor_last_slider[i] = sl
                        self._motor_target_vars[i].set(sl)
                    if i < len(self._motor_target_user_set):
                        self._motor_target_user_set[i] = False
                self._motor_target_initialized = True
                self._seed_motor_last_sent_after_sync(s)
            else:
                for i in range(min(MOTOR_COUNT, len(s.servo_raw_positions), len(self._motor_target_vars))):
                    if i < len(s.servo_raw_online) and s.servo_raw_online[i]:
                        single = self._motor_target_from_servo_feedback(s.servo_raw_positions[i])
                        dev_abs = self._clamp_motor_abs_cmd(single)
                        self._motor_session_origin_abs[i] = dev_abs
                        self._motor_abs_pos[i] = 0
                        self._motor_last_slider[i] = single
                        self._motor_target_vars[i].set(single)
                    if i < len(self._motor_target_user_set):
                        self._motor_target_user_set[i] = False
                self._motor_target_initialized = True
                self._seed_motor_last_sent_after_sync(s)

        # 一次性方向覆盖：到位稳定后自动清零，继续等待下一次 +/-。
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        if mode == "motor":
            for i in range(MOTOR_COUNT):
                if i >= len(self._motor_dir_hint):
                    break
                if self._motor_dir_hint[i] == 0:
                    if i < len(self._motor_dir_hint_pending_clear):
                        self._motor_dir_hint_pending_clear[i] = False
                    if i < len(self._motor_dir_hint_stable_count):
                        self._motor_dir_hint_stable_count[i] = 0
                    continue
                if i >= len(self._motor_dir_hint_pending_clear) or not self._motor_dir_hint_pending_clear[i]:
                    continue
                if i >= len(self._motor_target_vars):
                    continue

                cur_single = None
                if (
                    motor_has_raw
                    and i < len(s.servo_raw_positions)
                    and i < len(s.servo_raw_online)
                    and s.servo_raw_online[i]
                ):
                    cur_single = self._motor_target_from_servo_feedback(s.servo_raw_positions[i])
                elif (
                    motor_has_abs
                    and i < len(s.servo_angles)
                    and i < len(s.servo_online)
                    and s.servo_online[i]
                ):
                    cur_single = self._slider_from_abs_pos(int(s.servo_angles[i]))

                if cur_single is None:
                    self._motor_dir_hint_stable_count[i] = 0
                    continue

                tgt_single = self._read_motor_target_slider_int(i)
                if self._single_turn_diff(cur_single, tgt_single) <= MOTOR_DIR_HINT_CLEAR_TOL:
                    self._motor_dir_hint_stable_count[i] += 1
                else:
                    self._motor_dir_hint_stable_count[i] = 0

                if self._motor_dir_hint_stable_count[i] >= MOTOR_DIR_HINT_CLEAR_STABLE_FRAMES:
                    self._motor_dir_hint[i] = 0
                    self._motor_dir_hint_pending_clear[i] = False
                    self._motor_dir_hint_stable_count[i] = 0

    def _toggle_interaction_control(self):
        self._interaction_enabled = not self._interaction_enabled
        if self._interaction_enabled and hasattr(self, "status_var"):
            self.status_var.set("Interaction enabled")
        self._interaction_btn.config(text=f"Arm Manual: {'ON' if self._interaction_enabled else 'OFF'}")
        state = "normal" if self._interaction_enabled else "disabled"
        for w in self._control_widgets:
            try:
                w.configure(state=state)
            except Exception:
                pass
        self._update_manual_action_buttons()
        self._apply_interaction_control_policy()

    def _apply_interaction_control_policy(self):
        """
        交互控制策略：
        - Interaction=ON 且 Joint Control: PID 使能 + 关节角控制模式
        - Motor Control: PID 失能 + 直接电机控制模式
        - Interaction=OFF 且 Joint Control: 保持关节角模式，但 PID 关闭
        """
        if not hasattr(self, "_ctrl_mode_var"):
            return
        mode = self._ctrl_mode_var.get()
        if mode == "motor":
            self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
            self.controller.set_pid_control(False)
        else:
            self.controller.set_control_mode(ControlMode.JOINT_ANGLE)
            self.controller.set_pid_control(bool(getattr(self, "_interaction_enabled", False)))

    def _clear_run_states(self) -> None:
        self._joint_run_states = [False] * ENCODER_COUNT
        self._motor_run_states = [False] * MOTOR_COUNT
        for btn in getattr(self, "_joint_run_buttons", []):
            try:
                btn.configure(text="Send")
            except Exception:
                pass
        for btn in getattr(self, "_motor_run_buttons", []):
            try:
                btn.configure(text="Send")
            except Exception:
                pass

    def _set_row_running(self, is_motor: bool, idx: int, running: bool) -> None:
        states = self._motor_run_states if is_motor else self._joint_run_states
        buttons = self._motor_run_buttons if is_motor else self._joint_run_buttons
        if 0 <= idx < len(states):
            states[idx] = bool(running)
        if 0 <= idx < len(buttons):
            try:
                buttons[idx].configure(text="Hold" if running else "Send")
            except Exception:
                pass

    def _update_manual_action_buttons(self) -> None:
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        interaction_on = bool(getattr(self, "_interaction_enabled", False))
        joint_state = "normal" if mode == "joint" else "disabled"
        send_all_state = "normal" if interaction_on else "disabled"
        for btn_name in ("_open_pose_btn", "_fist_pose_btn", "_zero_pose_btn"):
            btn = getattr(self, btn_name, None)
            if btn is not None:
                btn.configure(state=joint_state)
        if hasattr(self, "_open_pose_btn"):
            self._open_pose_btn.pack_configure(padx=(0, 4))
        if hasattr(self, "_send_all_btn"):
            self._send_all_btn.configure(state=send_all_state)
            self._send_all_btn.pack_configure(padx=(8, 0))

    def _hold_joint_row_at_feedback(self, idx: int) -> None:
        if not self._guard_joint_command_started():
            return
        if idx < len(self._joint_target_vars):
            self._joint_target_vars[idx].set(
                self._clamp_joint_target_deg(idx, self._joint_feedback_target(idx))
            )
        if idx < len(self._joint_target_user_set):
            self._joint_target_user_set[idx] = False
        targets = self._resolved_joint_targets()
        self._send_joint_targets_with_tendon_guard(targets, source=f"Hold J{idx}")

    def _hold_motor_row_at_feedback(self, idx: int) -> None:
        with self._state_lock:
            s = self._state
        if (
            s
            and bool(getattr(s, "has_servo_angle_data", False))
            and idx < len(s.servo_angles)
            and idx < len(s.servo_online)
            and s.servo_online[idx]
        ):
            dev_abs = self._clamp_motor_abs_cmd(int(s.servo_angles[idx]))
            if idx < len(self._motor_session_origin_abs) and self._motor_session_origin_abs[idx] is None:
                self._motor_session_origin_abs[idx] = dev_abs
            origin = self._motor_session_origin_abs[idx] if idx < len(self._motor_session_origin_abs) else 0
            self._motor_abs_pos[idx] = self._clamp_motor_abs_cmd(dev_abs - origin)
            sl = self._slider_from_abs_pos(dev_abs)
            self._motor_last_slider[idx] = sl
            if idx < len(self._motor_target_vars):
                self._motor_target_vars[idx].set(sl)
        elif idx < len(self._motor_target_vars):
            self._motor_target_vars[idx].set(self._motor_feedback_target(idx))
            self._motor_resync_one_channel(idx)
        if idx < len(self._motor_target_user_set):
            self._motor_target_user_set[idx] = False
        self._send_motor_abs_from_ui(f"Hold M{idx}")

    def _confirm_joint_row(self, idx: int):
        if not self._interaction_enabled:
            return
        if not self._guard_joint_command_started():
            return
        try:
            if idx < len(self._joint_run_states) and self._joint_run_states[idx]:
                self._hold_joint_row_at_feedback(idx)
                self._set_row_running(False, idx, False)
                self.status_var.set(f"Channel held at current feedback (J{idx})")
            else:
                self._mark_joint_target_user_set(idx)
                targets = self._resolved_joint_targets()
                self._send_joint_targets_with_tendon_guard(targets, source=f"Run J{idx}")
                self._set_row_running(False, idx, True)
                self.status_var.set(f"Joint command sent (J{idx})")
        except Exception as e:
            messagebox.showerror("Send Failed", f"Joint control send failed: {e}")

    def _confirm_motor_row(self, idx: int):
        try:
            if idx < len(self._motor_run_states) and self._motor_run_states[idx]:
                self._hold_motor_row_at_feedback(idx)
                self._set_row_running(True, idx, False)
                self.status_var.set(f"Channel held at current feedback (M{idx})")
            else:
                self._mark_motor_target_user_set(idx)
                self._sync_motor_abs_from_slider_before_send(idx)
                self._send_motor_abs_from_ui(f"Run M{idx}")
                self._set_row_running(True, idx, True)
                self.status_var.set(f"Motor absolute command sent (M{idx})")
        except Exception as e:
            messagebox.showerror("Send Failed", f"Motor absolute send failed: {e}")

    def _on_joint_slider_change(self, idx: int, value: str):
        """拖动关节拉动条时实时下发，无需点击确认"""
        if not self._interaction_enabled:
            return
        try:
            clamped = self._clamp_joint_target_deg(idx, float(value))
            self._joint_target_vars[idx].set(clamped)
            self._mark_joint_target_user_set(idx)
            if not self._guard_joint_command_started():
                return
            targets = self._resolved_joint_targets()
            # 滑条跟随走高频通道，避免快速拖动时命令堆积/跳变。
            self._send_joint_targets_with_tendon_guard(targets, source=f"Slider J{idx}", live=True)
        except Exception:
            pass

    def _on_motor_slider_change(self, idx: int, value: str):
        """滑条=用户目标：锁定当前会话圈，按该圈内单圈位直接映射。"""
        if not self._interaction_enabled:
            return
        with self._state_lock:
            s = self._state
        if s is None or not (
            bool(getattr(s, "has_servo_angle_data", False))
            or bool(getattr(s, "has_servo_raw_data", False))
        ):
            if hasattr(self, "status_var"):
                self.status_var.set("Slider ignored: waiting for motor feedback")
            return
        try:
            new_s = int(float(value))
        except (TypeError, ValueError):
            return
        if idx < 0 or idx >= MOTOR_COUNT:
            return
        if self._motor_abs_pos[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            return
        last = self._motor_last_slider[idx]
        drag_sign = 0
        if last is not None:
            d = int(new_s) - int(last)
            if d > 0:
                drag_sign = 1
            elif d < 0:
                drag_sign = -1
        mapped = self._session_abs_from_slider_locked_turn(idx, new_s, drag_sign)
        if mapped is None:
            return
        if idx < len(self._motor_first_drag_limit_active) and self._motor_first_drag_limit_active[idx]:
            mapped = max(-MOTOR_TARGET_SCALE_HI, min(MOTOR_TARGET_SCALE_HI, int(mapped)))
        if last is None:
            self._motor_abs_pos[idx] = mapped
            self._motor_last_slider[idx] = new_s
            self._motor_target_vars[idx].set(new_s)
            self._mark_motor_target_user_set(idx)
            self._send_motor_abs_from_ui(f"Slider M{idx}")
            if idx < len(self._motor_first_drag_limit_active):
                self._motor_first_drag_limit_active[idx] = False
            return
        d = new_s - int(last)
        if abs(d) > MOTOR_JUMP_RESYNC_THRESH:
            if self._motor_slider_dragging[idx]:
                return
            self._motor_resync_one_channel(idx)
            return
        self._motor_abs_pos[idx] = mapped
        self._motor_last_slider[idx] = new_s
        self._motor_target_vars[idx].set(new_s)
        self._mark_motor_target_user_set(idx)
        self._send_motor_abs_from_ui(f"Slider M{idx}")
        if idx < len(self._motor_first_drag_limit_active):
            self._motor_first_drag_limit_active[idx] = False

    def _step_motor_target(self, idx: int, delta: int) -> None:
        """± 仅在边界激活：+ 仅 4095，- 仅 0，并写入一次性方向提示。"""
        if not self._interaction_enabled:
            return
        if idx < 0 or idx >= len(self._motor_target_vars):
            return
        if self._motor_abs_pos[idx] is None:
            self._motor_resync_one_channel(idx)
        if self._motor_abs_pos[idx] is None:
            return
        cur_sl = self._read_motor_target_slider_int(idx)
        try:
            if int(delta) > 0:
                if cur_sl < MOTOR_TARGET_SCALE_HI:
                    return
                new_sl = MOTOR_TARGET_SCALE_LO
                d_abs = 1
            elif int(delta) < 0:
                if cur_sl > MOTOR_TARGET_SCALE_LO:
                    return
                new_sl = MOTOR_TARGET_SCALE_HI
                d_abs = -1
            else:
                return
            self._motor_abs_pos[idx] = self._clamp_motor_abs_cmd(
                int(self._motor_abs_pos[idx]) + d_abs
            )
            if idx < len(self._motor_dir_hint):
                self._motor_dir_hint[idx] = 1 if d_abs > 0 else -1
            if idx < len(self._motor_dir_hint_pending_clear):
                self._motor_dir_hint_pending_clear[idx] = True
            if idx < len(self._motor_dir_hint_stable_count):
                self._motor_dir_hint_stable_count[idx] = 0
            self._motor_target_vars[idx].set(new_sl)
            self._motor_last_slider[idx] = new_sl
            self._mark_motor_target_user_set(idx)
            label = "Step+" if delta > 0 else "Step-"
            self._send_motor_abs_from_ui(f"{label} M{idx}")
        except Exception as e:
            messagebox.showerror("Send Failed", f"Motor step failed: {e}")

    def _set_joint_encoder_dot(self, joint_index: int, is_connected: Optional[bool]) -> None:
        if not hasattr(self, "_joint_status_dots"):
            return
        if joint_index < 0 or joint_index >= len(self._joint_status_dots):
            return

        if is_connected is True:
            color = "#19a55a"
        elif is_connected is False:
            color = "#d84343"
        else:
            color = "#9aa0a6"

        canvas, dot_item = self._joint_status_dots[joint_index]
        try:
            canvas.itemconfig(dot_item, fill=color, outline=color)
        except Exception:
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
        self._last_tendon_guard_sent = None
        self._joint_target_user_set = [False] * len(getattr(self, "_joint_target_vars", []))
        self._motor_target_user_set = [False] * len(getattr(self, "_motor_target_vars", []))
        self._clear_run_states()
        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)
        neutral = [0.0] * ENCODER_COUNT
        if hasattr(self, "_joint_curr_vars"):
            for v in self._joint_curr_vars:
                v.set("-")
        if hasattr(self, "_joint_tie_motor_abs_vars"):
            for v in self._joint_tie_motor_abs_vars:
                v.set("-")
        if hasattr(self, "_joint_tie_x1_live_vars"):
            for v in self._joint_tie_x1_live_vars:
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
        if hasattr(self, "ax"):
            self._draw_hand_model(neutral, None)
        if hasattr(self, "_tactile_dot_scatters"):
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

    def _show_servo_status_dialog(self) -> None:
        """PACKET_TYPE_FAULT_STATUS / 关节调试扩展字段等（不改变触觉与相机区）。"""
        with self._state_lock:
            s = self._state
        lines: List[str] = []
        lines.append(f"下位机已 start: {'是' if self.controller.is_started() else '否'}")
        if s is None:
            lines.append("尚无设备状态（未 Start 或无回调数据）。")
        else:
            fl = [str(i) for i, f in enumerate(getattr(s, "servo_overload_fault", []) or []) if f]
            lines.append(f"过载锁存 (motor 索引): {', '.join(fl) if fl else '无'}")
            rfl = [f"J{i}" for i, f in enumerate(getattr(s, "joint_reverse_release_fault", []) or []) if f]
            lines.append(f"反绕保护 (joint): {', '.join(rfl) if rfl else '无'}")
            cv = getattr(s, "joint_debug_cmd_valid", None) or []
            cp = getattr(s, "joint_debug_cmd_target_pos", None) or []
            n_ok = sum(1 for v in cv if v)
            lines.append(f"关节调试 cmd 有效: {n_ok}/{ENCODER_COUNT}")
            show_j = [0, 1, 2, 8, 9, 16, 20]
            for j in show_j:
                if j < len(cv) and j < len(cp) and bool(cv[j]):
                    lines.append(f"  J{j:02d} cmd_target_pos = {int(cp[j])}")
        messagebox.showinfo("伺服 / 协议状态", "\n".join(lines))

    def _on_btn_start(self):
        port = self._get_port_from_main_combo()
        if not self._is_comm_connected():
            if not port:
                messagebox.showwarning(
                    "No Port",
                    "Please select a serial port in the Port dropdown first.",
                )
                return
            if not self.controller.initialize(port):
                messagebox.showerror("Connection Failed", f"Cannot open port: {port}")
                return
            self._device_running = True
            self._emit_action(f"Port connected: {port}")
            self._on_comm_ready()
        self.controller.start()
        self._refresh_start_indicator()
        self._emit_action("Click [Start]")
        if hasattr(self, "status_var"):
            self.status_var.set("Motor start command sent")
        self._maybe_redraw()

    def _on_btn_stop(self):
        self._emit_action("Click [Stop]")
        if not self._is_comm_connected():
            self._refresh_start_indicator()
            return
        try:
            self.controller.stop()
        except Exception:
            pass
        self._refresh_start_indicator()
        if hasattr(self, "status_var"):
            self.status_var.set("Motor stop command sent")

    def _on_btn_reset(self):
        self._emit_action("Click [Reset]")
        self.controller.reset()
        self._refresh_start_indicator()

    def _on_btn_home(self):
        if hasattr(self, "_ctrl_mode_var") and self._ctrl_mode_var.get() != "joint":
            self._ctrl_mode_var.set("joint")
            self._switch_control_mode()
        if not self._interaction_enabled:
            if hasattr(self, "status_var"):
                self.status_var.set("Home requires manual control to be armed")
            return
        self._emit_action("Click [Home]")
        self._apply_preset_zero()
        self._apply_all_angles()
        if hasattr(self, "status_var"):
            self.status_var.set("Home pose sent")

    def _apply_preset_open(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Preset: Open]")
        angles = _preset_20rad_to_21deg(_OPEN_20_RAD)
        for i, a in enumerate(angles):
            if i < len(self._joint_target_vars):
                self._joint_target_vars[i].set(self._clamp_joint_target_deg(i, round(a, 1)))
                if i < len(self._joint_target_user_set):
                    self._joint_target_user_set[i] = True
        self._draw_hand_model(angles, None)

    def _apply_preset_fist(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Preset: Fist]")
        angles = _preset_20rad_to_21deg(_FIST_20_RAD)
        for i, a in enumerate(angles):
            if i < len(self._joint_target_vars):
                self._joint_target_vars[i].set(self._clamp_joint_target_deg(i, round(a, 1)))
                if i < len(self._joint_target_user_set):
                    self._joint_target_user_set[i] = True
        self._draw_hand_model(angles, None)

    def _apply_preset_zero(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Zero All]")
        for i, v in enumerate(self._joint_target_vars):
            v.set(self._clamp_joint_target_deg(i, 0.0))
            if i < len(self._joint_target_user_set):
                self._joint_target_user_set[i] = True
        self._draw_hand_model([0.0] * ENCODER_COUNT, None)

    def _on_calibrate(self):
        self._emit_action("Click [Calibrate]")

        def progress(current: int, total: int, msg: str):
            self.root.after(0, lambda: self.status_var.set(f"Calibration: {current + 1}/{total} {msg}"))

        def done(success: bool, zero_raw):
            self.root.after(0, lambda: self._calibrate_done(success, zero_raw))

        self.controller.calibrate(progress_cb=progress, done_cb=done)
        self.status_var.set("Calibration in progress...")

    def _calibrate_done(self, success: bool, zero_raw):
        if success:
            messagebox.showinfo("Calibration Completed", "Saved to calib_deg.json and sent to lower controller")
            self.status_var.set("Calibration completed")
        else:
            messagebox.showerror("Calibration Failed", "Calibration failed, please retry")

    def _apply_all_angles(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Send All]")
        try:
            mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
            if mode == "motor":
                for mi in range(MOTOR_COUNT):
                    self._sync_motor_abs_from_slider_before_send(mi)
                self._send_motor_abs_from_ui("Send All")
                self.status_var.set(f"Sent {MOTOR_COUNT} motor absolute targets")
            else:
                if not self._guard_joint_command_started():
                    return
                angles = self._resolved_joint_targets()
                self._send_joint_targets_with_tendon_guard(angles, source="Send All")
                self.status_var.set(f"Sent {ENCODER_COUNT} joint targets")
        except ValueError as e:
            messagebox.showerror("Parameter Error", str(e))
        except Exception as e:
            messagebox.showerror("Send Failed", f"{type(e).__name__}: {e}")

    def _setup_status_bar(self):
        ttk.Separator(self.root, orient=tk.HORIZONTAL).pack(side=tk.BOTTOM, fill=tk.X)
        bar = ttk.Frame(self.root, padding=(8, 3))
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        comm = getattr(self.controller, "comm", None)
        connected = bool(
            comm
            and getattr(comm, "serial", None)
            and getattr(comm.serial, "is_open", False)
        )
        self.status_var = tk.StringVar(value="Disconnected" if not connected else "Ready")
        ttk.Label(bar, textvariable=self.status_var, anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._delay_var = tk.StringVar()
        self._overload_var = tk.StringVar(value="伺服过载: —")
        self._overload_lbl = tk.Label(
            bar,
            textvariable=self._overload_var,
            fg="gray35",
            font=("Segoe UI" if os.name == "nt" else "", 9),
        )
        self._overload_lbl.pack(side=tk.RIGHT, padx=(0, 10))
        self._release_fault_var = tk.StringVar(value="反绕保护: 无")
        self._release_fault_lbl = tk.Label(
            bar,
            textvariable=self._release_fault_var,
            fg="gray35",
            font=("Segoe UI" if os.name == "nt" else "", 9),
        )
        self._release_fault_lbl.pack(side=tk.RIGHT, padx=(0, 10))
        ttk.Label(bar, textvariable=self._delay_var).pack(side=tk.RIGHT)

    def _update_display(self):
        if not self._is_comm_connected():
            return
        now = time.time()
        with self._state_lock:
            s = self._state
        # 无连接时 _state 为 None，用默认状态；触觉由 _get_tactile_display_state(fake) 决定
        if not s:
            s = HandModel()

        feedback_interval = float(getattr(self, "_feedback_draw_interval", 0.06))
        do_feedback_update = (now - float(getattr(self, "_last_feedback_draw", 0.0))) >= feedback_interval
        if do_feedback_update:
            if hasattr(self, "_joint_curr_vars"):
                self._refresh_joint_feedback_view(sync_target=not self._joint_target_initialized)
            if hasattr(self, "_motor_curr_vars"):
                self._refresh_motor_feedback_view(sync_target=not self._motor_target_initialized)
            self._last_feedback_draw = now

        # 电机模式不以 0x03 角度重算 abs（避免拖一路时邻路 abs 被陈旧角度改掉、整包误触发）。

        need_pose_draw = self._is_pose_3d_enabled() and (now - self._last_pose_draw) >= self._pose_draw_interval
        need_tactile_draw = (now - self._last_tactile_draw) >= self._tactile_draw_interval
        tactile_state = None
        if need_pose_draw or need_tactile_draw:
            tactile_state = self._get_tactile_display_state(s, fake=self._should_use_fake_tactile(s))
        if need_pose_draw:
            self._draw_hand_model(s.angles, tactile_state)
            self._last_pose_draw = now
        if need_tactile_draw:
            self._update_tactile_matrix_widget(tactile_state)
            self._last_tactile_draw = now
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
