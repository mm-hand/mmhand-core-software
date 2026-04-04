# ui_main.py - 灵巧手图形界面（含启动时的端口/模式选择）

import os
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap, Normalize
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from typing import Callable, List, Optional, Tuple
import threading
import time

from protocol import ENCODER_COUNT, MOTOR_COUNT, ControlMode
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
        self.root.title("Dexterous Hand - Connection & Mode")
        self.root.geometry("336x224")
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
        title = f"{ENCODER_COUNT}-DOF Dexterous Hand Control System"
        if title_suffix:
            title = f"{title} - {title_suffix}"
        self.root.title(title)
        self.root.geometry("1260x720")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        controller.register_update_callback(self._on_state_update)
        self._state: Optional[HandModel] = None
        self._state_lock = threading.Lock()
        self._last_draw = 0.0
        self._draw_interval = 0.1  # 100ms
        # 0=X, 1=Y, 2=Z；可多选，触觉区按选中个数并排显示对应热力图
        self._tactile_axes_selected: set = {0, 1, 2}
        self._tactile_fake_mode = True  # False=真实数据/no data，True=显示生成数据
        # 主触觉区为仅传感 inferno 色标（见 _make_tactile_sensors_only_cmap_norm）；子图占位仍用全图热力图色标
        self._camera1_cap = None
        self._camera2_cap = None
        self._camera_frame_left_buf = None
        self._camera_frame_right_buf = None
        self._camera_buf_lock = threading.Lock()
        self._camera_thread = None
        self._camera_stop = threading.Event()
        # Start：串口连接（若未连）+ 下位机 start + 手部姿态/触觉等；相机在 Device 中单独选，与 Start 无关
        self._device_running = False

        self._setup_ui()
        self._setup_device_menu()
        self._start_camera_poll()
        self._start_display_poll()
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
            self.mode_select_var.set(mode_name)
            self._emit_action("Click [Apply Port/Mode]")
            self._apply_port_mode_in_main()

    def _list_available_cameras(self) -> List[int]:
        """Probe indices 0-4 for available cameras (avoids OpenCV out-of-bound on some systems)."""
        try:
            import cv2
        except ImportError:
            return []
        available = []
        for idx in range(5):
            try:
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        available.append(idx)
            except Exception:
                pass
        return available

    def _refresh_camera_menus(self) -> None:
        """Refresh camera combo dropdown values (Device > Refresh)."""
        self._update_camera_combo_values()

    def _on_camera_selected(self, index: Optional[int], slot: int) -> None:
        """Slot 1=Left widget, 2=Right widget."""
        label = "None" if index is None else f"Camera {index}"
        if slot == 1:
            self._camera1_var.set(label)
        else:
            self._camera2_var.set(label)
        self._emit_action(f"Camera {slot} selected: {index if index is not None else 'None'}")
        threading.Thread(target=lambda: self._set_camera_capture(slot, index), daemon=True).start()

    def _set_camera_capture(self, slot: int, index: Optional[int]) -> None:
        """Open or release camera for slot (1=Left, 2=Right)."""
        try:
            import cv2
        except ImportError:
            return
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

        if index is not None:
            try:
                cap = cv2.VideoCapture(index)
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
            time.sleep(0.033)

    def _start_display_poll(self) -> None:
        """设备 Start 后定期刷新；未 Start 时不刷新（触觉 fake 亦仅在运行中）"""
        def poll() -> None:
            try:
                if not getattr(self, "_device_running", False):
                    self.root.after(200, poll)
                    return
                if not self._is_comm_connected():
                    self._device_running = False
                    self._clear_live_displays()
                    self.root.after(200, poll)
                    return
                with self._state_lock:
                    has_state = self._state is not None
                need_redraw = not has_state or getattr(self, "_tactile_fake_mode", False)
                if need_redraw:
                    self._maybe_redraw()
                self.root.after(200, poll)
            except tk.TclError:
                pass
        self.root.after(200, poll)

    def _is_comm_connected(self) -> bool:
        comm = getattr(self.controller, "comm", None)
        return bool(
            comm
            and getattr(comm, "serial", None)
            and getattr(comm.serial, "is_open", False)
        )

    def _should_use_realtime_pose(self) -> bool:
        # Runtime policy:
        # - No port/connection: use txt pose.
        # - Connected but not started: use txt pose.
        # - Connected and started: use realtime angles.
        return self._is_comm_connected() and bool(getattr(self, "_device_running", False))

    def _start_camera_poll(self) -> None:
        """Main-thread poll: copy buffers to display (non-blocking)."""

        def poll() -> None:
            try:
                with self._camera_buf_lock:
                    left = self._camera_frame_left_buf
                    right = self._camera_frame_right_buf
                self._camera_frame_left = left
                self._camera_frame_right = right
                if hasattr(self, "_cam_left_canvas"):
                    self._update_camera_widget()
            except Exception:
                pass
            try:
                self.root.after(33, poll)
            except tk.TclError:
                pass

        self.root.after(33, poll)

    def _emit_action(self, msg: str) -> None:
        """在按钮响应中上报操作（如监控模式下直接打印）"""
        if self._action_callback:
            try:
                self._action_callback(msg)
            except Exception:
                pass

    def _on_close(self):
        stop = getattr(self, "_camera_stop", None)
        if stop is not None:
            stop.set()
        thr = getattr(self, "_camera_thread", None)
        if thr is not None:
            thr.join(timeout=1.0)
            self._camera_thread = None
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
        self._ports_cache = list_ports()
        choices = ["No Connection"] + [f"{d} ({desc})" for d, desc in self._ports_cache]
        if hasattr(self, "port_combo_main"):
            self.port_combo_main["values"] = choices
        if hasattr(self, "port_select_var") and (not self.port_select_var.get()) and choices:
            self.port_select_var.set(choices[0])
        if hasattr(self, "_port_menu"):
            self._refresh_port_menu()

    def _parse_mode_str(self, mode_str: str) -> Mode:
        if mode_str in ("遥操作", "Teleoperation"):
            return Mode.TELEOP
        if mode_str in ("算法", "Algorithm"):
            return Mode.ALGORITHM
        return Mode.MONITOR

    def _apply_port_mode_in_main(self):
        """在主窗口直接应用端口/模式，不再弹独立启动窗口"""
        mode_str = self.mode_select_var.get() or "Monitor"
        mode = self._parse_mode_str(mode_str)

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
        old_mode = self._parse_mode_str(self.current_mode or "Monitor")
        if old_port == port and old_mode == mode:
            return

        # 复用现有 controller：仅当端口变化时重连
        reused_controller = self.controller
        if old_port != port:
            reused_controller.shutdown()
            if port:
                if not reused_controller.initialize(port):
                    messagebox.showerror("Connection Failed", f"Cannot open port: {port}")
                    return

        # 解绑旧窗口回调，避免切换后回调叠加
        try:
            if self._on_state_update in reused_controller.update_callbacks:
                reused_controller.update_callbacks.remove(self._on_state_update)
        except Exception:
            pass
        self.root.destroy()
        run(mode, reused_controller, port=port, gui=True)

    def _on_state_update(self, state: HandModel):
        if not getattr(self, "_device_running", False):
            return
        with self._state_lock:
            self._state = state
        self.root.after(0, self._maybe_redraw)

    def _maybe_redraw(self):
        if not getattr(self, "_device_running", False):
            return
        now = time.time()
        if now - self._last_draw < self._draw_interval:
            return
        self._last_draw = now
        self._update_display()

    def _setup_ui(self):
        # Top bar: port selection + control buttons
        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, padx=6, pady=4)
        refresh_btn = tk.Button(top, text="↻", width=2, command=self._refresh_port_options_main, cursor="hand2")
        refresh_btn.pack(side=tk.LEFT, padx=(0, 4))
        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_select_var = tk.StringVar(value="")
        self.port_combo_main = ttk.Combobox(top, textvariable=self.port_select_var, width=26, state="readonly")
        self.port_combo_main.pack(side=tk.LEFT, padx=4)
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
        ttk.Button(top, text="Reset", command=self._on_btn_reset).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="伺服状态", command=self._show_servo_status_dialog).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="Calibrate", command=self._on_calibrate).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="Stop", command=self._on_btn_stop).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="Start", command=self._on_btn_start).pack(side=tk.RIGHT, padx=2)

        # weight：窗口被拉大时“多出来的宽度”按权重分给各格；不保证首屏分割线就是 2:1
        # 若要与 weight 一致，需在布局完成后设 sash（见 _set_initial_main_sash）
        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)
        self._main_paned = main
        self._main_left_weight = 2
        self._main_right_weight = 1

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
        if self.current_mode in ("遥操作", "Teleoperation"):
            self._setup_teleop_control_panel(right)
        elif self.current_mode in ("算法", "Algorithm"):
            self._setup_algorithm_control_panel(right)
        else:
            self._setup_control_panel(right)

        self._setup_status_bar()
        self.root.after(100, self._set_initial_main_sash)

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

    def _setup_3d_display(self, parent):
        pose_frame = ttk.Frame(parent)
        pose_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 2))
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
        self._pose_3d_dist_fix_idle_pending = False
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
        if event.widget is not getattr(self, "_pose_canvas_widget", None):
            return
        w, h = int(event.width), int(event.height)
        if w < 24 or h < 24:
            return
        key = (w, h)
        if key == getattr(self, "_pose_canvas_size", None):
            return
        self._pose_canvas_size = key
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

    def _on_pose_3d_mpl_scroll(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None):
            return
        self._pose_view_dirty = True
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
        self._pose_view_dirty = True
        if self._pose_3d_dist_fix_idle_pending:
            return
        self._pose_3d_dist_fix_idle_pending = True

        def fix_after_mplot3d() -> None:
            self._pose_3d_dist_fix_idle_pending = False
            apply_mplot3d_camera_zoom(self.ax, dist=self._pose_3d_target_dist)
            self.canvas.draw_idle()

        self.root.after_idle(fix_after_mplot3d)

    def _on_pose_3d_mpl_release(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None) or event.button != 1:
            return
        self._pose_view_dirty = True

        def restore() -> None:
            apply_mplot3d_camera_zoom(self.ax, dist=self._pose_3d_target_dist)
            self.canvas.draw_idle()

        self.root.after(1, restore)

    def _toggle_tactile_axis(self, axis: int):
        labels = ["X", "Y", "Z"]
        if axis in self._tactile_axes_selected:
            self._tactile_axes_selected.discard(axis)
            self._emit_action(f"Tactile axis {labels[axis]} OFF")
        else:
            self._tactile_axes_selected.add(axis)
            self._emit_action(f"Tactile axis {labels[axis]} ON")
        self._tactile_fig_sel_sig = None  # 强制重建子图布局
        self._refresh_tactile_axis_button_states()
        with self._state_lock:
            s = self._state
        tactile_state = self._get_tactile_display_state(s, fake=self._tactile_fake_mode)
        if s:
            self._draw_hand_model(s.angles, tactile_state)
        self._update_tactile_matrix_widget(tactile_state)

    def _on_tactile_fake_toggle(self):
        self._tactile_fake_mode = bool(self._tactile_fake_var.get())
        with self._state_lock:
            s = self._state
        if not s:
            s = HandModel()
        tactile_state = self._get_tactile_display_state(s, fake=self._tactile_fake_mode)
        self._draw_hand_model(s.angles, tactile_state)
        self._update_tactile_matrix_widget(tactile_state)

    def _refresh_tactile_axis_button_states(self):
        if not hasattr(self, "_tactile_axis_buttons"):
            return
        for i, btn in enumerate(self._tactile_axis_buttons):
            selected = i in self._tactile_axes_selected
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
        """固定 1×3 子图（X | Y | Z），未选中的方向保留占位格，不随选中数缩放。"""
        if not hasattr(self, "_tactile_fig"):
            return
        self._clear_tactile_anatomy_overlay()
        self._tactile_fig.clear()
        self._tactile_subplot_imgs = []
        self._tactile_no_data_texts = []
        axes_arr = self._tactile_fig.subplots(1, 3, squeeze=False, sharey=True)
        axes_row = axes_arr[0]
        spec = TACTILE_STITCH_SPEC
        mat0 = np.full((spec.height, spec.width), np.nan, dtype=float)

        for i in range(3):
            ax = axes_row[i]
            on = i in self._tactile_axes_selected
            if on:
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
                self._tactile_no_data_texts.append(nd)
            else:
                im = ax.imshow(
                    mat0,
                    cmap="gray",
                    aspect="auto",
                    vmin=0.25,
                    vmax=0.35,
                    interpolation="nearest",
                )
                ax.text(
                    0.5,
                    0.5,
                    "未选择",
                    transform=ax.transAxes,
                    ha="center",
                    va="center",
                    fontsize=10,
                    color="silver",
                )
                self._tactile_no_data_texts.append(None)

            # 去掉各通道子图外框（spines），仅保留热力图内容
            ax.axis("off")
            self._tactile_subplot_imgs.append(im)
        # tight_layout 会给子图留边并拉开间距；无坐标轴时用手动边距更紧凑
        self._tactile_fig.subplots_adjust(left=0.01, right=0.99, bottom=0.02, top=0.98, wspace=0.06)

    def _setup_tactile_matrix_widget(self, parent):
        """触觉空间矩阵：15 个传感器按 5 指 x 3 骨段拼成一个热力图"""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # 标题与触觉轴按钮同一行
        self._tactile_row = ttk.Frame(frame)
        self._tactile_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(self._tactile_row, text="Axes (multi):").pack(side=tk.LEFT, padx=2)
        self._tactile_fake_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            self._tactile_row,
            text="Fake",
            variable=self._tactile_fake_var,
            command=self._on_tactile_fake_toggle,
        ).pack(side=tk.LEFT, padx=4)
        self._tactile_axis_buttons = []
        for axis, label in enumerate(["X", "Y", "Z"]):
            btn = tk.Button(
                self._tactile_row,
                text=label,
                width=3,
                command=lambda a=axis: self._toggle_tactile_axis(a),
            )
            btn.pack(side=tk.LEFT, padx=2)
            self._tactile_axis_buttons.append(btn)
        self._refresh_tactile_axis_button_states()

        self._tactile_fig = plt.Figure(figsize=(7.8, 4.0))
        self._tactile_canvas = FigureCanvasTkAgg(self._tactile_fig, frame)
        self._tactile_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._tactile_fig_sel_sig = None
        self._tactile_subplot_imgs: List = []
        self._tactile_no_data_texts: List = []
        self._rebuild_tactile_figure()
        self._tactile_fig_sel_sig = frozenset(self._tactile_axes_selected)
        self._tactile_anatomy_artists: List = []
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
        """将原触觉窗格拆分为左右两栏：左触觉矩阵，右相机图像（触觉更窄，相机更宽）"""
        split = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        split.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        left_frame = ttk.Frame(split)
        split.add(left_frame, weight=1)
        self._setup_tactile_matrix_widget(left_frame)

        cam_frame = ttk.LabelFrame(split, text="")
        split.add(cam_frame, weight=2)
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

        self._update_camera_combo_values()

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
        choices = ["None"] + [f"Camera {i}" for i in self._list_available_cameras()]
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
        sig = frozenset(self._tactile_axes_selected)
        if sig != getattr(self, "_tactile_fig_sel_sig", object()):
            self._rebuild_tactile_figure()
            self._tactile_fig_sel_sig = sig

        self._clear_tactile_anatomy_overlay()

        for ax_i, (im, nd_txt) in enumerate(
            zip(self._tactile_subplot_imgs, self._tactile_no_data_texts)
        ):
            if ax_i not in self._tactile_axes_selected or nd_txt is None:
                continue
            mat = self._build_tactile_spatial_matrix(state, ax_i)
            tactile_pos = mat[np.isfinite(mat) & (mat >= 0.0)]
            vmax = float(np.max(tactile_pos)) if tactile_pos.size > 0 else 0.0
            vmax = max(vmax, 1.0)
            cmap_u, norm_u = _make_tactile_sensors_only_cmap_norm(vmax)
            im.set_data(np.ma.masked_invalid(mat))
            im.set_cmap(cmap_u)
            im.set_norm(norm_u)
            has_data = bool(
                state
                and state.tactile_data
                and tactile_pos.size > 0
                and (np.max(tactile_pos) > 1e-9)
            )
            nd_txt.set_visible(not has_data)
            self._draw_tactile_anatomy_overlay_sensors_only(im.axes, mat)
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
        _ = state
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
        panel = ttk.LabelFrame(parent, text=f"Interactive Control (J{ENCODER_COUNT} / M{MOTOR_COUNT})")
        panel.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        mode_row = ttk.Frame(panel)
        mode_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(mode_row, text="Mode:").pack(side=tk.LEFT)
        self._interaction_enabled = False
        self._control_widgets: List[tk.Widget] = []
        self._ctrl_mode_var = tk.StringVar(value="joint")
        rb_joint = ttk.Radiobutton(mode_row, text="Joint Control", variable=self._ctrl_mode_var, value="joint",
                                   command=self._switch_control_mode)
        rb_joint.pack(side=tk.LEFT, padx=4)
        rb_motor = ttk.Radiobutton(mode_row, text="Motor Control", variable=self._ctrl_mode_var, value="motor",
                                   command=self._switch_control_mode)
        rb_motor.pack(side=tk.LEFT, padx=4)
        # Keep mode switchers always enabled, even when Interaction is OFF,
        # so users can still switch between Joint/Motor views to inspect data.
        self._interaction_btn = ttk.Button(mode_row, text="Interaction: OFF", command=self._toggle_interaction_control)
        self._interaction_btn.pack(side=tk.RIGHT, padx=2)

        self._ctrl_body = ttk.Frame(panel)
        self._ctrl_body.pack(fill=tk.BOTH, expand=True)

        self._joint_panel = ttk.Frame(self._ctrl_body)
        self._motor_panel = ttk.Frame(self._ctrl_body)

        self._joint_curr_vars: List[tk.StringVar] = []
        self._joint_target_vars: List[tk.DoubleVar] = []
        self._joint_status_dots: List[Tuple[tk.Canvas, int]] = []
        self._motor_curr_vars: List[tk.StringVar] = []
        self._motor_target_vars: List[tk.IntVar] = []
        self._motor_target_initialized = False

        self._build_mode_table(self._joint_panel, is_motor=False)
        self._build_mode_table(self._motor_panel, is_motor=True)
        # 初始化为关闭：禁用交互输入
        for w in self._control_widgets:
            try:
                w.configure(state="disabled")
            except Exception:
                pass
        self._switch_control_mode()

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
        canvas.create_window((0, 0), window=inner, anchor="nw")

        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<MouseWheel>", lambda e: canvas.yview_scroll(int(-(e.delta if hasattr(e, 'delta') else (120 if getattr(e, 'num', 0) == 4 else -120)) / 120), "units"))
        canvas.bind("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        header = ttk.Frame(inner)
        header.pack(fill=tk.X, padx=2, pady=(2, 4))
        if is_motor:
            ttk.Label(header, text="Channel").grid(row=0, column=0, padx=2, sticky="w")
            ttk.Label(header, text="Current").grid(row=0, column=1, padx=2, sticky="w")
            ttk.Label(header, text="Target").grid(row=0, column=2, padx=2, sticky="w")
            ttk.Label(header, text="Confirm").grid(row=0, column=3, padx=2, sticky="w")
            ttk.Label(header, text="Target Slider").grid(row=0, column=4, padx=2, sticky="w")
        else:
            ttk.Label(header, text="Channel").grid(row=0, column=0, padx=2, sticky="w")
            ttk.Label(header, text="Encoder").grid(row=0, column=1, padx=2, sticky="w")
            ttk.Label(header, text="Current").grid(row=0, column=2, padx=2, sticky="w")
            ttk.Label(header, text="Target").grid(row=0, column=3, padx=2, sticky="w")
            ttk.Label(header, text="Confirm").grid(row=0, column=4, padx=2, sticky="w")
            ttk.Label(header, text="Target Slider").grid(row=0, column=5, padx=2, sticky="w")

        channel_count = MOTOR_COUNT if is_motor else ENCODER_COUNT
        for i in range(channel_count):
            row = ttk.Frame(inner)
            row.pack(fill=tk.X, padx=2, pady=1)
            label_prefix = "M" if is_motor else "J"
            ttk.Label(row, text=f"{label_prefix}{i:02d}", width=4).grid(row=0, column=0, padx=1, sticky="w")

            curr = tk.StringVar(value="-")

            if is_motor:
                ttk.Label(row, textvariable=curr, width=8).grid(row=0, column=1, padx=1, sticky="w")
                target = tk.IntVar(value=0)
                self._motor_curr_vars.append(curr)
                self._motor_target_vars.append(target)
                ent = ttk.Entry(row, textvariable=target, width=8)
                ent.grid(row=0, column=2, padx=1, sticky="w")
                btn = ttk.Button(row, text="OK", width=4, command=lambda idx=i: self._confirm_motor_row(idx))
                btn.grid(row=0, column=3, padx=1, sticky="w")
                sld = ttk.Scale(
                    row,
                    from_=0,
                    to=16383,
                    variable=target,
                    orient=tk.HORIZONTAL,
                    length=130,
                    command=lambda value, idx=i: self._on_motor_slider_change(idx, value),
                )
                sld.grid(row=0, column=4, padx=1, sticky="we")
                self._control_widgets.extend([ent, btn, sld])
            else:
                dot_canvas = tk.Canvas(row, width=12, height=12, highlightthickness=0, bd=0)
                dot_item = dot_canvas.create_oval(2, 2, 10, 10, fill="#9aa0a6", outline="#9aa0a6")
                dot_canvas.grid(row=0, column=1, padx=2, sticky="w")
                self._joint_status_dots.append((dot_canvas, dot_item))

                ttk.Label(row, textvariable=curr, width=8).grid(row=0, column=2, padx=1, sticky="w")
                target = tk.DoubleVar(value=0.0)
                self._joint_curr_vars.append(curr)
                self._joint_target_vars.append(target)
                ent = ttk.Entry(row, textvariable=target, width=8)
                ent.grid(row=0, column=3, padx=1, sticky="w")
                btn = ttk.Button(row, text="OK", width=4, command=lambda idx=i: self._confirm_joint_row(idx))
                btn.grid(row=0, column=4, padx=1, sticky="w")
                sld = ttk.Scale(
                    row,
                    from_=-90,
                    to=90,
                    variable=target,
                    orient=tk.HORIZONTAL,
                    length=130,
                    command=lambda value, idx=i: self._on_joint_slider_change(idx, value),
                )
                sld.grid(row=0, column=5, padx=1, sticky="we")
                self._control_widgets.extend([ent, btn, sld])

    def _switch_control_mode(self):
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        self._joint_panel.pack_forget()
        self._motor_panel.pack_forget()
        if mode == "motor":
            self._motor_panel.pack(fill=tk.BOTH, expand=True)
        else:
            self._joint_panel.pack(fill=tk.BOTH, expand=True)
        self._apply_interaction_control_policy()

    def _toggle_interaction_control(self):
        self._interaction_enabled = not self._interaction_enabled
        self._interaction_btn.config(text=f"Interaction: {'ON' if self._interaction_enabled else 'OFF'}")
        state = "normal" if self._interaction_enabled else "disabled"
        for w in self._control_widgets:
            try:
                w.configure(state=state)
            except Exception:
                pass
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

    def _confirm_joint_row(self, idx: int):
        if not self._interaction_enabled:
            return
        try:
            targets = [float(v.get()) for v in self._joint_target_vars]
            self.controller.set_target_angles(targets)
            self.status_var.set(f"Joint targets sent (trigger J{idx})")
        except Exception as e:
            messagebox.showerror("Send Failed", f"Joint control send failed: {e}")

    def _confirm_motor_row(self, idx: int):
        if not self._interaction_enabled:
            return
        try:
            targets = [int(v.get()) for v in self._motor_target_vars]
            self.controller.set_motor_positions_raw(targets)
            self.status_var.set(f"Motor targets sent (trigger M{idx})")
        except Exception as e:
            messagebox.showerror("Send Failed", f"Motor control send failed: {e}")

    def _on_joint_slider_change(self, idx: int, value: str):
        """拖动关节拉动条时实时下发，无需点击确认"""
        if not self._interaction_enabled:
            return
        try:
            self._joint_target_vars[idx].set(float(value))
            targets = [float(v.get()) for v in self._joint_target_vars]
            self.controller.set_target_angles(targets)
        except Exception:
            pass

    def _on_motor_slider_change(self, idx: int, value: str):
        """拖动电机拉动条时实时下发，无需点击确认"""
        if not self._interaction_enabled:
            return
        try:
            self._motor_target_vars[idx].set(int(float(value)))
            targets = [int(v.get()) for v in self._motor_target_vars]
            self.controller.set_motor_positions_raw(targets)
        except Exception:
            pass

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
        """Stop 后或未 Start：清空关节读数、3D 手与触觉；相机单独由 Device 菜单选择，不受此影响。"""
        with self._state_lock:
            self._state = None
        self._motor_target_initialized = False
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
        if hasattr(self, "ax"):
            self._draw_hand_model(neutral, None)
        if hasattr(self, "_tactile_subplot_imgs"):
            empty = HandModel()
            tactile_state = self._get_tactile_display_state(empty, fake=self._tactile_fake_mode)
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
        if getattr(self, "_device_running", False):
            return
        port = self._get_port_from_main_combo()
        connected = self._is_comm_connected()
        if not connected:
            if not port:
                messagebox.showwarning(
                    "No Port",
                    "Please select a serial port in the Port dropdown, then press Start.",
                )
                return
            if not self.controller.initialize(port):
                messagebox.showerror("Connection Failed", f"Cannot open port: {port}")
                return
        self.controller.start()
        self._device_running = True
        self._emit_action("Click [Start]")
        if hasattr(self, "status_var"):
            self.status_var.set("Running…")
        self._maybe_redraw()

    def _on_btn_stop(self):
        self._emit_action("Click [Stop]")
        if not getattr(self, "_device_running", False):
            return
        try:
            self.controller.stop()
        except Exception:
            pass
        try:
            self.controller.shutdown()
        except Exception:
            pass
        self._device_running = False
        self.root.after(0, self._clear_live_displays)

    def _on_btn_reset(self):
        self._emit_action("Click [Reset]")
        self.controller.reset()

    def _apply_preset_open(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Preset: Open]")
        angles = _preset_20rad_to_21deg(_OPEN_20_RAD)
        for i, a in enumerate(angles):
            if i < len(self._joint_target_vars):
                self._joint_target_vars[i].set(round(a, 1))
        self._draw_hand_model(angles, None)

    def _apply_preset_fist(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Preset: Fist]")
        angles = _preset_20rad_to_21deg(_FIST_20_RAD)
        for i, a in enumerate(angles):
            if i < len(self._joint_target_vars):
                self._joint_target_vars[i].set(round(a, 1))
        self._draw_hand_model(angles, None)

    def _apply_preset_zero(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Zero All]")
        for v in self._joint_target_vars:
            v.set(0.0)
        self._draw_hand_model([0.0] * ENCODER_COUNT, None)

    def _on_calibrate(self):
        self._emit_action("Click [Calibrate]")
        if not self.controller.is_started():
            messagebox.showwarning("Calibration Blocked", "Please click Start before calibration.")
            self.status_var.set("Calibration blocked: click Start first")
            return

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

    def _on_pause(self):
        paused = self.controller.toggle_pause()
        self._emit_action("Click [Resume]" if paused else "Click [Pause]")
        self._pause_btn.config(text="Resume" if paused else "Pause")
        with self._state_lock:
            s = self._state
        if s:
            self._update_status_text(s, paused)
        elif paused:
            self.status_var.set("[Paused] Waiting for data...")

    def _apply_all_angles(self):
        if not self._interaction_enabled:
            return
        self._emit_action("Click [Apply All Angles]")
        try:
            angles = [self._joint_target_vars[i].get() for i in range(min(ENCODER_COUNT, len(self._joint_target_vars)))]
            if len(angles) < ENCODER_COUNT:
                angles.extend([0.0] * (ENCODER_COUNT - len(angles)))
            self.controller.set_target_angles(angles)
            self.status_var.set(f"Sent {ENCODER_COUNT} target joint angles")
        except ValueError as e:
            messagebox.showerror("Parameter Error", str(e))
        except Exception as e:
            messagebox.showerror("Send Failed", f"{type(e).__name__}: {e}")

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
        if not getattr(self, "_device_running", False):
            return
        with self._state_lock:
            s = self._state
        # 无连接时 _state 为 None，用默认状态；触觉由 _get_tactile_display_state(fake) 决定
        if not s:
            s = HandModel()

        if hasattr(self, "_joint_curr_vars"):
            joint_has_data = bool(getattr(s, "has_sensor_data", False))
            for i in range(min(ENCODER_COUNT, len(self._joint_curr_vars))):
                if joint_has_data and i < len(s.angles):
                    a = s.angles[i]
                    self._joint_curr_vars[i].set(f"{a:6.1f}")
                else:
                    self._joint_curr_vars[i].set("-")

                if hasattr(self, "_joint_status_dots"):
                    if joint_has_data and i < len(s.encoders):
                        self._set_joint_encoder_dot(i, not bool(s.encoders[i].error))
                    else:
                        self._set_joint_encoder_dot(i, None)

        if hasattr(self, "_motor_curr_vars"):
            motor_has_data = bool(getattr(s, "has_servo_angle_data", False))
            for i in range(min(MOTOR_COUNT, len(self._motor_curr_vars))):
                if motor_has_data and i < len(s.servo_angles):
                    ang = s.servo_angles[i]
                    self._motor_curr_vars[i].set(f"{ang:6d}")
                else:
                    self._motor_curr_vars[i].set("-")
            if motor_has_data and not self._motor_target_initialized:
                for i in range(min(MOTOR_COUNT, len(s.servo_angles))):
                    if i < len(self._motor_target_vars):
                        self._motor_target_vars[i].set(int(s.servo_angles[i]))
                self._motor_target_initialized = True

        tactile_state = self._get_tactile_display_state(s, fake=self._tactile_fake_mode)
        self._draw_hand_model(s.angles, tactile_state)
        self._update_tactile_matrix_widget(tactile_state)
        if self.current_mode in ("算法", "Algorithm") and hasattr(self, "_alg_output"):
            now = time.time()
            if now - getattr(self, "_alg_last_print", 0.0) >= 0.2:
                self._alg_last_print = now
                line = " ".join([f"J{i}:{s.angles[i]:.1f}" for i in range(min(ENCODER_COUNT, len(s.angles)))])
                self._append_algorithm_output(line)

        # 触觉轴按钮始终显示
        if hasattr(self, "_tactile_row") and not self._tactile_row.winfo_ismapped():
            self._tactile_row.pack(fill=tk.X, padx=2, pady=2)

        err_count = sum(1 for m in s.motors if m.error)
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
