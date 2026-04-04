# ui_main.py - 灵巧手图形界面（含启动时的端口/模式选择）

import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from typing import Callable, List, Optional, Tuple
import threading
import time

from protocol import ENCODER_COUNT, MOTOR_COUNT, ControlMode
from data_models import HandModel, FingerTactile
from core_logic import HandController
from comm_layer import list_ports
from modes import Mode, run
from hand_visualizer_ui import HandVisualizerUI

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
    """灵巧手图形界面 - 实时显示与控制"""

    def __init__(
        self,
        controller: HandController,
        title_suffix: str = "",
        current_port: Optional[str] = None,
        current_mode: Optional[str] = None,
        on_close_extra: Optional[Callable[[], None]] = None,
        action_callback: Optional[Callable[[str], None]] = None,
    ):
        self.controller = controller
        self.current_port = current_port or ""
        _mode_map = {"监控": "Monitor", "遥操作": "Teleoperation", "算法": "Algorithm"}
        self.current_mode = _mode_map.get(current_mode or "", current_mode or "")
        self._on_close_extra = on_close_extra
        self._action_callback = action_callback

        self.root = tk.Tk()
        title = f"{ENCODER_COUNT}-DOF Dexterous Hand Control System"
        if title_suffix:
            title = f"{title} - {title_suffix}"
        self.root.title(title)
        self.root.geometry("1160x720")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        controller.register_update_callback(self._on_state_update)
        self._state: Optional[HandModel] = None
        self._state_lock = threading.Lock()
        self._last_draw = 0.0
        self._draw_interval = 0.1  # 100ms
        self._hand_visualizer = HandVisualizerUI()
        self._tactile_axis = 2  # 0=X, 1=Y, 2=Z(压力) 默认

        self._setup_ui()

    def _emit_action(self, msg: str) -> None:
        """在按钮响应中上报操作（如监控模式下直接打印）"""
        if self._action_callback:
            try:
                self._action_callback(msg)
            except Exception:
                pass

    def _on_close(self):
        # 避免关闭后残留状态回调
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

    def _on_btn_reopen(self):
        self._emit_action("Click [Apply Port/Mode]")
        self._apply_port_mode_in_main()

    def _refresh_port_options_main(self):
        self._ports_cache = list_ports()
        choices = ["No Connection"] + [f"{d} ({desc})" for d, desc in self._ports_cache]
        if hasattr(self, "port_combo_main"):
            self.port_combo_main["values"] = choices
        if not self.port_select_var.get() and choices:
            self.port_select_var.set(choices[0])

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
        with self._state_lock:
            self._state = state
        self.root.after(0, self._maybe_redraw)

    def _maybe_redraw(self):
        now = time.time()
        if now - self._last_draw < self._draw_interval:
            return
        self._last_draw = now
        self._update_display()

    def _setup_ui(self):
        # 顶部：端口与模式信息 + 总控按钮（同一行）
        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(top, text="Refresh Ports", command=self._refresh_port_options_main).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_select_var = tk.StringVar(value="")
        self.port_combo_main = ttk.Combobox(top, textvariable=self.port_select_var, width=26, state="readonly")
        self.port_combo_main.pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text="Mode:").pack(side=tk.LEFT, padx=(8, 0))
        self.mode_select_var = tk.StringVar(value=self.current_mode or "Monitor")
        self.mode_combo_main = ttk.Combobox(
            top,
            textvariable=self.mode_select_var,
            width=10,
            state="readonly",
            values=["Monitor", "Teleoperation", "Algorithm"],
        )
        self.mode_combo_main.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Apply Port/Mode", command=self._on_btn_reopen).pack(side=tk.LEFT, padx=(4, 8))
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
        ttk.Button(top, text="Start", command=self._on_btn_start).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="Stop", command=self._on_btn_stop).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="Calibrate", command=self._on_calibrate).pack(side=tk.RIGHT, padx=2)
        ttk.Button(top, text="Reset", command=self._on_btn_reset).pack(side=tk.RIGHT, padx=2)

        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        main.add(left, weight=2)
        self._setup_3d_display(left)
        self._setup_tactile_and_camera_widgets(left)

        right = ttk.Frame(main)
        main.add(right, weight=1)
        if self.current_mode in ("遥操作", "Teleoperation"):
            self._setup_teleop_control_panel(right)
        elif self.current_mode in ("算法", "Algorithm"):
            self._setup_algorithm_control_panel(right)
        else:
            self._setup_control_panel(right)

        self._setup_status_bar()

    def _setup_3d_display(self, parent):
        from mpl_toolkits.mplot3d import Axes3D

        self.fig = plt.Figure(figsize=(5, 3.2))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=False)
        self._draw_hand_model([0.0] * ENCODER_COUNT, None)

    def _setup_tactile_axis_buttons(self, parent):
        self._tactile_row = ttk.Frame(parent)
        self._tactile_row.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(self._tactile_row, text="Tactile Axis:").pack(side=tk.LEFT, padx=2)
        self._tactile_axis_buttons = []
        for axis, label in enumerate(["X", "Y", "Z"]):
            btn = tk.Button(
                self._tactile_row,
                text=label,
                width=3,
                command=lambda a=axis: self._set_tactile_axis(a),
            )
            btn.pack(side=tk.LEFT, padx=2)
            self._tactile_axis_buttons.append(btn)
        self._refresh_tactile_axis_button_states()

    def _set_tactile_axis(self, axis: int):
        labels = ["X", "Y", "Z"]
        self._emit_action(f"Click [Tactile Axis {labels[axis]}]")
        self._tactile_axis = axis
        self._refresh_tactile_axis_button_states()
        with self._state_lock:
            s = self._state
        if s:
            self._draw_hand_model(s.angles, s)
        self._update_tactile_matrix_widget(s)

    def _refresh_tactile_axis_button_states(self):
        if not hasattr(self, "_tactile_axis_buttons"):
            return
        for i, btn in enumerate(self._tactile_axis_buttons):
            selected = i == self._tactile_axis
            btn.configure(relief=tk.SUNKEN if selected else tk.RAISED, bd=2 if selected else 1)

    def _setup_tactile_matrix_widget(self, parent):
        """触觉空间矩阵：15 个传感器按 5 指 x 3 骨段拼成一个热力图"""
        frame = ttk.LabelFrame(parent, text="")
        frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        # 标题与触觉轴按钮同一行
        self._tactile_row = ttk.Frame(frame)
        self._tactile_row.pack(fill=tk.X, padx=2, pady=2)
        ttk.Label(self._tactile_row, text="Tactile Mode").pack(side=tk.LEFT, padx=2)
        self._tactile_axis_buttons = []
        for axis, label in enumerate(["X", "Y", "Z"]):
            btn = tk.Button(
                self._tactile_row,
                text=label,
                width=3,
                command=lambda a=axis: self._set_tactile_axis(a),
            )
            btn.pack(side=tk.LEFT, padx=2)
            self._tactile_axis_buttons.append(btn)
        self._refresh_tactile_axis_button_states()

        self._tactile_fig = plt.Figure(figsize=(6, 3.2))
        self._tactile_ax = self._tactile_fig.add_subplot(111)
        self._tactile_canvas = FigureCanvasTkAgg(self._tactile_fig, frame)
        self._tactile_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        mat0 = np.zeros((12, 20), dtype=float)  # 3 段 x 4 行, 5 指 x 4 列
        self._tactile_img = self._tactile_ax.imshow(
            mat0, cmap="inferno", aspect="auto", vmin=0.0, vmax=1.0
        )
        self._tactile_ax.set_xticks([1.5, 5.5, 9.5, 13.5, 17.5])
        self._tactile_ax.set_xticklabels(["thumb", "index", "middle", "ring", "pinky"], fontsize=8)
        self._tactile_ax.set_yticks([1.5, 5.5, 9.5])
        self._tactile_ax.set_yticklabels(["distal", "middle", "proximal"], fontsize=8)
        self._tactile_ax.set_title("Axis: Z", fontsize=9)
        self._tactile_ax.tick_params(length=0)
        self._tactile_no_data_text = self._tactile_ax.text(
            0.5,
            0.5,
            "No tactile data",
            transform=self._tactile_ax.transAxes,
            ha="center",
            va="center",
            fontsize=10,
            color="white",
            alpha=0.85,
        )
        self._tactile_canvas.draw_idle()

    def _setup_tactile_and_camera_widgets(self, parent):
        """将原触觉窗格拆分为左右两栏：左触觉矩阵，右相机图像"""
        split = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        split.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        left_frame = ttk.Frame(split)
        split.add(left_frame, weight=1)
        self._setup_tactile_matrix_widget(left_frame)

        cam_frame = ttk.LabelFrame(split, text="")
        split.add(cam_frame, weight=1)
        self._setup_camera_widget(cam_frame)

    def _setup_camera_widget(self, parent):
        """相机图像窗格：上下分为左相机/右相机"""
        self._camera_frame_left = None
        self._camera_frame_right = None

        split = ttk.PanedWindow(parent, orient=tk.VERTICAL)
        split.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        left_box = ttk.LabelFrame(split, text="Left Camera")
        right_box = ttk.LabelFrame(split, text="Right Camera")
        split.add(left_box, weight=1)
        split.add(right_box, weight=1)

        self._cam_left_fig = plt.Figure(figsize=(4, 1.6))
        self._cam_left_ax = self._cam_left_fig.add_subplot(111)
        self._cam_left_canvas = FigureCanvasTkAgg(self._cam_left_fig, left_box)
        self._cam_left_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._cam_right_fig = plt.Figure(figsize=(4, 1.6))
        self._cam_right_ax = self._cam_right_fig.add_subplot(111)
        self._cam_right_canvas = FigureCanvasTkAgg(self._cam_right_fig, right_box)
        self._cam_right_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        placeholder = np.zeros((240, 320, 3), dtype=float)
        self._cam_left_img = self._cam_left_ax.imshow(placeholder)
        self._cam_left_ax.set_axis_off()
        self._cam_left_no_data_text = self._cam_left_ax.text(
            0.5, 0.5, "No left camera", transform=self._cam_left_ax.transAxes,
            ha="center", va="center", fontsize=10, color="white", alpha=0.9,
        )

        self._cam_right_img = self._cam_right_ax.imshow(placeholder)
        self._cam_right_ax.set_axis_off()
        self._cam_right_no_data_text = self._cam_right_ax.text(
            0.5, 0.5, "No right camera", transform=self._cam_right_ax.transAxes,
            ha="center", va="center", fontsize=10, color="white", alpha=0.9,
        )
        self._cam_left_canvas.draw_idle()
        self._cam_right_canvas.draw_idle()

    def set_camera_frame_left(self, frame: np.ndarray) -> None:
        """外部可调用：设置左相机帧（RGB/BGR/灰度均可）"""
        self._camera_frame_left = frame

    def set_camera_frame_right(self, frame: np.ndarray) -> None:
        """外部可调用：设置右相机帧（RGB/BGR/灰度均可）"""
        self._camera_frame_right = frame

    def set_camera_frame(self, frame: np.ndarray) -> None:
        """兼容旧接口：默认写入左相机帧"""
        self._camera_frame_left = frame

    def _update_single_camera_widget(self, frame, img_artist, no_data_text, canvas):
        if frame is None:
            no_data_text.set_visible(True)
            canvas.draw_idle()
            return
        img = np.asarray(frame)
        if img.ndim == 2:
            img_artist.set_data(img)
            img_artist.set_cmap("gray")
        elif img.ndim == 3 and img.shape[2] >= 3:
            img_artist.set_data(img[:, :, :3])
            img_artist.set_cmap(None)
        else:
            no_data_text.set_visible(True)
            canvas.draw_idle()
            return
        no_data_text.set_visible(False)
        canvas.draw_idle()

    def _update_camera_widget(self):
        if not hasattr(self, "_cam_left_img"):
            return
        self._update_single_camera_widget(
            self._camera_frame_left, self._cam_left_img, self._cam_left_no_data_text, self._cam_left_canvas
        )
        self._update_single_camera_widget(
            self._camera_frame_right, self._cam_right_img, self._cam_right_no_data_text, self._cam_right_canvas
        )

    def _build_tactile_spatial_matrix(self, state: Optional[HandModel]) -> np.ndarray:
        """
        将 15 个触觉传感器按空间关系拼接为单一矩阵：
        - 列方向：thumb -> pinky（每个传感器 4 列）
        - 行方向：distal -> proximal（每个传感器 4 行）
        """
        sensor_rows, sensor_cols = 4, 4
        mat = np.zeros((sensor_rows * 3, sensor_cols * 5), dtype=float)
        if not state or not state.tactile_data:
            return mat

        for finger_idx in range(min(5, len(state.tactile_data))):
            fd = state.tactile_data[finger_idx]
            if not fd.sensors:
                continue
            for seg in range(min(3, len(fd.sensors))):
                s = fd.sensors[seg]
                cf = getattr(s, "contact_forces", None)
                if cf is None or not hasattr(cf, "shape") or len(cf.shape) < 3:
                    continue
                axis = min(self._tactile_axis, cf.shape[2] - 1)
                grid = np.array(cf[:, :, axis], dtype=float)
                grid = np.clip(grid, 0, None)

                block = np.zeros((sensor_rows, sensor_cols), dtype=float)
                h = min(sensor_rows, grid.shape[0])
                w = min(sensor_cols, grid.shape[1])
                block[:h, :w] = grid[:h, :w]

                row_block = 2 - seg  # seg=0 近端，放到底部；seg=2 远端，放顶部
                r0 = row_block * sensor_rows
                c0 = finger_idx * sensor_cols
                mat[r0 : r0 + sensor_rows, c0 : c0 + sensor_cols] = block
        return mat

    def _update_tactile_matrix_widget(self, state: Optional[HandModel]):
        if not hasattr(self, "_tactile_img"):
            return
        mat = self._build_tactile_spatial_matrix(state)
        vmax = float(np.max(mat)) if mat.size > 0 else 0.0
        vmax = max(vmax, 1.0)
        self._tactile_img.set_data(mat)
        self._tactile_img.set_clim(0.0, vmax)
        axis_label = ["X", "Y", "Z"][self._tactile_axis]
        self._tactile_ax.set_title(f"Axis: {axis_label}", fontsize=9)
        has_data = bool(state and state.tactile_data and np.max(mat) > 0)
        self._tactile_no_data_text.set_visible(not has_data)
        self._tactile_canvas.draw_idle()

    def _get_tactile_values_15(self, state: Optional[HandModel]) -> List[float]:
        """15 个触觉面对应的标量值（按当前轴 X/Y/Z），无数据时返回 0"""
        out = [0.0] * 15
        if not state or not state.tactile_data:
            return out
        for finger_idx in range(min(5, len(state.tactile_data))):
            fd = state.tactile_data[finger_idx]
            if not fd.sensors:
                continue
            for seg in range(min(3, len(fd.sensors))):
                idx = finger_idx * 3 + seg
                if idx >= 15:
                    break
                r = getattr(fd.sensors[seg], "resultant", None)
                if r is not None and hasattr(r, "__len__") and len(r) > self._tactile_axis:
                    out[idx] = float(np.clip(r[self._tactile_axis], 0, None))
        return out

    def _draw_hand_model(self, angles: List[float], state: Optional[HandModel]):
        """根据 hand_geo 与关节角度绘制手部姿态（plot_hand 风格），可选触觉着色"""
        from mpl_toolkits.mplot3d import Axes3D

        self.ax.clear()
        vis = self._hand_visualizer
        vis.set_angles_from_motors(angles)
        tactile = self._get_tactile_values_15(state)
        vis.draw(
            self.ax,
            tactile_values=tactile,
            show_tactile=bool(state and state.tactile_data),
        )
        self.fig.canvas.draw_idle()

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
        ttk.Label(header, text="Channel").grid(row=0, column=0, padx=2, sticky="w")
        ttk.Label(header, text="Current").grid(row=0, column=1, padx=2, sticky="w")
        ttk.Label(header, text="Target").grid(row=0, column=2, padx=2, sticky="w")
        ttk.Label(header, text="Confirm").grid(row=0, column=3, padx=2, sticky="w")
        ttk.Label(header, text="Target Slider").grid(row=0, column=4, padx=2, sticky="w")

        channel_count = MOTOR_COUNT if is_motor else ENCODER_COUNT
        for i in range(channel_count):
            row = ttk.Frame(inner)
            row.pack(fill=tk.X, padx=2, pady=1)
            label_prefix = "M" if is_motor else "J"
            ttk.Label(row, text=f"{label_prefix}{i:02d}", width=4).grid(row=0, column=0, padx=1, sticky="w")

            curr = tk.StringVar(value="-")
            ttk.Label(row, textvariable=curr, width=8).grid(row=0, column=1, padx=1, sticky="w")

            if is_motor:
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
                target = tk.DoubleVar(value=0.0)
                self._joint_curr_vars.append(curr)
                self._joint_target_vars.append(target)
                ent = ttk.Entry(row, textvariable=target, width=8)
                ent.grid(row=0, column=2, padx=1, sticky="w")
                btn = ttk.Button(row, text="OK", width=4, command=lambda idx=i: self._confirm_joint_row(idx))
                btn.grid(row=0, column=3, padx=1, sticky="w")
                sld = ttk.Scale(
                    row,
                    from_=-90,
                    to=90,
                    variable=target,
                    orient=tk.HORIZONTAL,
                    length=130,
                    command=lambda value, idx=i: self._on_joint_slider_change(idx, value),
                )
                sld.grid(row=0, column=4, padx=1, sticky="we")
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

    def _on_btn_start(self):
        self._emit_action("Click [Start]")
        self.controller.start()

    def _on_btn_stop(self):
        self._emit_action("Click [Stop]")
        self.controller.stop()

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
        connected = self.controller.comm and getattr(
            self.controller.comm.serial, "is_open", False
        )
        self.status_var = tk.StringVar(value="Disconnected" if not connected else "Ready")
        ttk.Label(bar, textvariable=self.status_var).pack(side=tk.LEFT)
        self._delay_var = tk.StringVar()
        ttk.Label(bar, textvariable=self._delay_var).pack(side=tk.RIGHT)

    def _update_display(self):
        with self._state_lock:
            s = self._state
        if not s:
            return

        if hasattr(self, "_joint_curr_vars"):
            joint_has_data = bool(getattr(s, "has_sensor_data", False))
            for i in range(min(ENCODER_COUNT, len(self._joint_curr_vars))):
                if joint_has_data and i < len(s.angles):
                    a = s.angles[i]
                    self._joint_curr_vars[i].set(f"{a:6.1f}")
                else:
                    self._joint_curr_vars[i].set("-")

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

        self._draw_hand_model(s.angles, s)
        self._update_tactile_matrix_widget(s)
        self._update_camera_widget()
        if self.current_mode in ("算法", "Algorithm") and hasattr(self, "_alg_output"):
            now = time.time()
            if now - getattr(self, "_alg_last_print", 0.0) >= 0.2:
                self._alg_last_print = now
                line = " ".join([f"J{i}:{s.angles[i]:.1f}" for i in range(min(ENCODER_COUNT, len(s.angles)))])
                self._append_algorithm_output(line)

        # 有触觉数据时显示触觉轴按钮
        if hasattr(self, "_tactile_row"):
            has_tactile = bool(s.tactile_data)
            if has_tactile and not self._tactile_row.winfo_ismapped():
                self._tactile_row.pack(fill=tk.X, padx=4, pady=2)
            elif not has_tactile and self._tactile_row.winfo_ismapped():
                self._tactile_row.pack_forget()

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

    def run(self):
        self.root.mainloop()
