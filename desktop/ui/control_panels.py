import os
import json
import math
import time
from typing import List, Optional, Tuple
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np

from data_models import HandModel
from protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    ControlMode,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
)

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
MOTOR_TO_JOINT_INDEX = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20
]
JOINT_TO_PRIMARY_MOTOR_INDEX = [-1] * ENCODER_COUNT
for _mi, _ji in enumerate(MOTOR_TO_JOINT_INDEX):
    if 0 <= _ji < ENCODER_COUNT and JOINT_TO_PRIMARY_MOTOR_INDEX[_ji] < 0:
        JOINT_TO_PRIMARY_MOTOR_INDEX[_ji] = _mi
DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_OPEN_20_RAD = (
    [0.1, 0.0, 0.0, 0.0], [0.0, 0.2, 0.1, 0.0], [0.0, 0.1, 0.1, 0.0],
    [0.0, 0.1, 0.1, 0.0], [-0.1, 0.1, 0.1, 0.0],
)
_FIST_20_RAD = (
    [0.6, 0.4, 0.7, 0.9], [0.0, 0.8, 0.9, 1.0], [0.0, 0.8, 0.9, 1.0],
    [0.0, 0.8, 0.9, 1.0], [0.0, 0.8, 0.9, 1.0],
)
_ANGLE_MAP = [[3, 0, 1, 2], [4, 5, 6, 7], [8, 9, 10, 11], [12, 13, 14, 15], [16, 17, 18, 19]]


def _preset_20rad_to_21deg(preset_20: tuple) -> List[float]:
    out = [0.0] * ENCODER_COUNT
    flat = preset_20
    for i, indices in enumerate(_ANGLE_MAP):
        for j, midx in enumerate(indices):
            if 0 <= midx < ENCODER_COUNT:
                out[midx] = math.degrees(flat[i][j])
    return out

class ControlPanelsMixin:
    def _guard_joint_command_started(self) -> bool:
        if self.controller.is_started():
            return True
        if hasattr(self, "status_var"):
            self.status_var.set("Joint command blocked: press Enable first")
        self._refresh_start_indicator()
        return False

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
        try:
            table_bg = ttk.Style(parent).lookup("TFrame", "background") or parent.winfo_toplevel().cget("bg")
        except Exception:
            table_bg = "#f0f0f0"
        canvas = tk.Canvas(
            wrap,
            highlightthickness=0,
            borderwidth=0,
            relief=tk.FLAT,
            background=table_bg,
        )

        def _mark_table_scroll() -> None:
            self._table_scroll_active_until = time.time() + 0.18
            job = getattr(canvas, "_table_scroll_repaint_job", None)
            if job is not None:
                try:
                    self.root.after_cancel(job)
                except tk.TclError:
                    pass

            def repaint() -> None:
                canvas._table_scroll_repaint_job = None
                try:
                    canvas.update_idletasks()
                except tk.TclError:
                    pass

            canvas._table_scroll_repaint_job = self.root.after(35, repaint)

        def _yview(*args) -> None:
            _mark_table_scroll()
            canvas.yview(*args)

        def _xview(*args) -> None:
            _mark_table_scroll()
            canvas.xview(*args)

        scroll_y = ttk.Scrollbar(wrap, orient=tk.VERTICAL, command=_yview)
        scroll_x = ttk.Scrollbar(parent, orient=tk.HORIZONTAL, command=_xview)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
        scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
        canvas.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        inner = ttk.Frame(canvas)
        inner_window = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _refresh_scroll_region(_event=None) -> None:
            try:
                req_w = int(inner.winfo_reqwidth())
                canvas_w = int(canvas.winfo_width())
                canvas.itemconfigure(inner_window, width=max(req_w, canvas_w))
                canvas.configure(scrollregion=canvas.bbox("all"))
            except tk.TclError:
                pass

        def _wheel(event) -> str:
            _mark_table_scroll()
            delta = int(-(event.delta if hasattr(event, "delta") else (120 if getattr(event, "num", 0) == 4 else -120)) / 120)
            canvas.yview_scroll(delta, "units")
            return "break"

        def _button_wheel(units: int):
            def do(_event) -> str:
                _mark_table_scroll()
                canvas.yview_scroll(units, "units")
                return "break"
            return do

        inner.bind("<Configure>", _refresh_scroll_region)
        canvas.bind("<Configure>", _refresh_scroll_region)
        canvas.bind("<MouseWheel>", _wheel)
        canvas.bind("<Button-4>", _button_wheel(-1))
        canvas.bind("<Button-5>", _button_wheel(1))
        scroll_y.bind("<ButtonPress-1>", lambda _e: _mark_table_scroll(), add="+")
        scroll_y.bind("<B1-Motion>", lambda _e: _mark_table_scroll(), add="+")
        scroll_y.bind("<ButtonRelease-1>", lambda _e: _mark_table_scroll(), add="+")
        scroll_x.bind("<ButtonPress-1>", lambda _e: _mark_table_scroll(), add="+")
        scroll_x.bind("<B1-Motion>", lambda _e: _mark_table_scroll(), add="+")
        scroll_x.bind("<ButtonRelease-1>", lambda _e: _mark_table_scroll(), add="+")

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
                    length=130,
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
                    length=130,
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

        ff_box = ttk.LabelFrame(parent, text="Feedforward Test")
        ff_box.pack(fill=tk.X, expand=False, padx=2, pady=(0, 2))
        ttk.Label(
            ff_box,
            text="Step through a fixed joint-angle sequence with constant hold time per point.",
            font=("", 9),
        ).pack(fill=tk.X, padx=6, pady=(6, 4))

        ff_grid = ttk.Frame(ff_box)
        ff_grid.pack(fill=tk.X, padx=6, pady=(2, 4))
        ttk.Label(ff_grid, text="Joint", width=9).grid(row=0, column=0, padx=2, sticky="w")
        ttk.Label(ff_grid, text="Targets(deg)", width=18).grid(row=0, column=1, padx=2, sticky="w")
        ttk.Label(ff_grid, text="Step Time(s)", width=12).grid(row=0, column=2, padx=2, sticky="w")

        ff_joint_choices = ["None"] + [f"J{i:02d}" for i in range(ENCODER_COUNT)]
        ff_joint_combo = ttk.Combobox(
            ff_grid,
            textvariable=self._ff_test_joint_var,
            values=ff_joint_choices,
            state="readonly",
            width=8,
        )
        ff_joint_combo.grid(row=1, column=0, padx=2, pady=1, sticky="w")
        ff_joint_combo.bind("<<ComboboxSelected>>", lambda _e: self._ff_test_joint_var.set(
            self._normalize_test_joint_value(self._ff_test_joint_var.get())
        ))
        ff_targets_ent = ttk.Entry(ff_grid, textvariable=self._ff_test_targets_var, width=34)
        ff_targets_ent.grid(row=1, column=1, padx=2, pady=1, sticky="we")
        ff_step_ent = ttk.Entry(ff_grid, textvariable=self._ff_test_step_s_var, width=10)
        ff_step_ent.grid(row=1, column=2, padx=2, pady=1, sticky="w")
        ff_grid.columnconfigure(1, weight=1)
        self._control_widgets.extend([ff_joint_combo, ff_targets_ent, ff_step_ent])

        ff_action_row = ttk.Frame(ff_box)
        ff_action_row.pack(fill=tk.X, padx=6, pady=(4, 6))
        self._ff_test_start_btn = ttk.Button(
            ff_action_row,
            text="Start Feedforward Test",
            width=22,
            command=self._on_start_ff_test,
        )
        self._ff_test_start_btn.pack(side=tk.LEFT, padx=(0, 6))
        self._ff_test_stop_btn = ttk.Button(
            ff_action_row,
            text="Stop Feedforward Test",
            width=22,
            command=self._on_stop_ff_test,
        )
        self._ff_test_stop_btn.pack(side=tk.LEFT)

        tri_box = ttk.LabelFrame(parent, text="Triangle Wave Test")
        tri_box.pack(fill=tk.X, expand=False, padx=2, pady=(0, 2))
        ttk.Label(
            tri_box,
            text="Run one joint with a periodic triangle-wave target in joint PID mode.",
            font=("", 9),
        ).pack(fill=tk.X, padx=6, pady=(6, 4))

        tri_grid = ttk.Frame(tri_box)
        tri_grid.pack(fill=tk.X, padx=6, pady=(2, 4))
        ttk.Label(tri_grid, text="Joint", width=9).grid(row=0, column=0, padx=2, sticky="w")
        ttk.Label(tri_grid, text="Min(deg)", width=10).grid(row=0, column=1, padx=2, sticky="w")
        ttk.Label(tri_grid, text="Max(deg)", width=10).grid(row=0, column=2, padx=2, sticky="w")
        ttk.Label(tri_grid, text="Freq(Hz)", width=9).grid(row=0, column=3, padx=2, sticky="w")

        tri_joint_combo = ttk.Combobox(
            tri_grid,
            textvariable=self._tri_test_joint_var,
            values=joint_choices,
            state="readonly",
            width=8,
        )
        tri_joint_combo.grid(row=1, column=0, padx=2, pady=1, sticky="w")
        tri_joint_combo.bind("<<ComboboxSelected>>", lambda _e: self._tri_test_joint_var.set(
            self._normalize_test_joint_value(self._tri_test_joint_var.get())
        ))
        tri_min_ent = ttk.Entry(tri_grid, textvariable=self._tri_test_min_var, width=11)
        tri_min_ent.grid(row=1, column=1, padx=2, pady=1, sticky="w")
        tri_max_ent = ttk.Entry(tri_grid, textvariable=self._tri_test_max_var, width=11)
        tri_max_ent.grid(row=1, column=2, padx=2, pady=1, sticky="w")
        tri_freq_ent = ttk.Entry(tri_grid, textvariable=self._tri_test_freq_var, width=10)
        tri_freq_ent.grid(row=1, column=3, padx=2, pady=1, sticky="w")
        self._control_widgets.extend([tri_joint_combo, tri_min_ent, tri_max_ent, tri_freq_ent])

        tri_action_row = ttk.Frame(tri_box)
        tri_action_row.pack(fill=tk.X, padx=6, pady=(4, 6))
        self._tri_test_start_btn = ttk.Button(
            tri_action_row,
            text="Start Triangle Test",
            width=22,
            command=self._on_start_tri_test,
        )
        self._tri_test_start_btn.pack(side=tk.LEFT, padx=(0, 6))
        self._tri_test_stop_btn = ttk.Button(
            tri_action_row,
            text="Stop Triangle Test",
            width=22,
            command=self._on_stop_tri_test,
        )
        self._tri_test_stop_btn.pack(side=tk.LEFT)
        self._set_test_button_states()

        local_box = ttk.LabelFrame(parent, text="Local Test Mode")
        local_box.pack(fill=tk.X, expand=False, padx=2, pady=(0, 2))
        ttk.Label(
            local_box,
            text="Generate sine wave targets locally on device. Eliminates serial latency for pure controller testing.",
            font=("", 9),
        ).pack(fill=tk.X, padx=6, pady=(6, 4))

        local_grid = ttk.Frame(local_box)
        local_grid.pack(fill=tk.X, padx=6, pady=(2, 4))
        ttk.Label(local_grid, text="Freq(Hz)", width=10).grid(row=0, column=0, padx=2, sticky="w")
        ttk.Label(local_grid, text="Amp(deg)", width=10).grid(row=0, column=1, padx=2, sticky="w")

        self._local_freq_var = tk.StringVar(value="0.5")
        self._local_amp_var = tk.StringVar(value="20.0")
        freq_ent = ttk.Entry(local_grid, textvariable=self._local_freq_var, width=11)
        freq_ent.grid(row=1, column=0, padx=2, pady=1, sticky="w")
        amp_ent = ttk.Entry(local_grid, textvariable=self._local_amp_var, width=11)
        amp_ent.grid(row=1, column=1, padx=2, pady=1, sticky="w")
        self._control_widgets.extend([freq_ent, amp_ent])

        local_action_row = ttk.Frame(local_box)
        local_action_row.pack(fill=tk.X, padx=6, pady=(4, 6))
        self._local_start_btn = ttk.Button(
            local_action_row,
            text="Start Local Test",
            width=18,
            command=self._on_start_local_test,
        )
        self._local_start_btn.pack(side=tk.LEFT, padx=(0, 6))
        self._local_stop_btn = ttk.Button(
            local_action_row,
            text="Stop Local Test",
            width=18,
            command=self._on_stop_local_test,
        )
        self._local_stop_btn.pack(side=tk.LEFT)

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
        ff_start_btn = getattr(self, "_ff_test_start_btn", None)
        ff_stop_btn = getattr(self, "_ff_test_stop_btn", None)
        tri_start_btn = getattr(self, "_tri_test_start_btn", None)
        tri_stop_btn = getattr(self, "_tri_test_stop_btn", None)
        local_start_btn = getattr(self, "_local_start_btn", None)
        local_stop_btn = getattr(self, "_local_stop_btn", None)
        any_running = bool(
            self._test_running or
            self._ff_test_running or
            self._tri_test_running or
            self._local_test_running
        )
        if start_btn is not None:
            try:
                start_btn.configure(state="disabled" if any_running else "normal")
            except Exception:
                pass
        if stop_btn is not None:
            try:
                stop_btn.configure(state="normal" if self._test_running else "disabled")
            except Exception:
                pass
        if ff_start_btn is not None:
            try:
                ff_start_btn.configure(state="disabled" if any_running else "normal")
            except Exception:
                pass
        if ff_stop_btn is not None:
            try:
                ff_stop_btn.configure(state="normal" if self._ff_test_running else "disabled")
            except Exception:
                pass
        if tri_start_btn is not None:
            try:
                tri_start_btn.configure(state="disabled" if any_running else "normal")
            except Exception:
                pass
        if tri_stop_btn is not None:
            try:
                tri_stop_btn.configure(state="normal" if self._tri_test_running else "disabled")
            except Exception:
                pass
        if local_start_btn is not None:
            try:
                local_start_btn.configure(state="disabled" if any_running else "normal")
            except Exception:
                pass
        if local_stop_btn is not None:
            try:
                local_stop_btn.configure(state="normal" if self._local_test_running else "disabled")
            except Exception:
                pass

    def _schedule_test_tick(self) -> None:
        if not self._test_running:
            return
        self._test_after_job = self.root.after(int(self._test_tick_ms), self._test_tick)

    def _on_start_test(self) -> None:
        if self._test_running:
            return
        if self._ff_test_running:
            self._on_stop_ff_test(reason="restarted by sine test", update_status=False)
        if self._tri_test_running:
            self._on_stop_tri_test(reason="restarted by sine test", update_status=False)
        if self._local_test_running:
            self._on_stop_local_test(reason="restarted by sine test", update_status=False)
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

        try:
            if self._data_logger.is_running():
                self._stop_data_recording(reason="restart by test", keep_switch=True)
            self._start_data_recording()
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(True)
        except Exception as exc:
            messagebox.showerror("Start Test Failed", f"Cannot start recording: {exc}")
            return

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

    def _parse_ff_targets_text(raw_text: str) -> Optional[List[float]]:
        text = str(raw_text or "").replace(",", " ")
        tokens = [tok for tok in text.split() if tok]
        if not tokens:
            return None
        values: List[float] = []
        for tok in tokens:
            try:
                val = float(tok)
            except (TypeError, ValueError):
                return None
            if not np.isfinite(val):
                return None
            values.append(float(val))
        return values if values else None

    def _validate_ff_test_config(self, show_message: bool = True) -> Optional[Tuple[int, List[float], float]]:
        joint_text = self._normalize_test_joint_value(self._ff_test_joint_var.get())
        self._ff_test_joint_var.set(joint_text)
        if joint_text == "None":
            if show_message:
                messagebox.showwarning("Feedforward Test", "Select one valid joint.")
            return None
        joint_idx = int(joint_text[1:])

        targets = self._parse_ff_targets_text(self._ff_test_targets_var.get())
        if not targets:
            if show_message:
                messagebox.showwarning("Feedforward Test", "Targets(deg) must contain at least one valid number.")
            return None

        try:
            step_s = float(self._ff_test_step_s_var.get())
        except (TypeError, ValueError, tk.TclError):
            if show_message:
                messagebox.showwarning("Feedforward Test", "Step Time(s) must be a valid number.")
            return None
        if (not np.isfinite(step_s)) or step_s <= 0.0:
            if show_message:
                messagebox.showwarning("Feedforward Test", "Step Time(s) must be a finite number > 0.")
            return None

        clamped = [self._clamp_joint_target_deg(joint_idx, target) for target in targets]
        self._ff_test_targets_var.set(", ".join(f"{target:.2f}" for target in clamped))
        self._ff_test_step_s_var.set(f"{step_s:.4g}")
        return joint_idx, clamped, float(step_s)

    def _schedule_ff_test_tick(self) -> None:
        if not self._ff_test_running:
            return
        self._ff_test_after_job = self.root.after(int(self._test_tick_ms), self._ff_test_tick)

    def _on_start_ff_test(self) -> None:
        if self._ff_test_running:
            return
        if self._test_running:
            self._on_stop_test(reason="restarted by feedforward test", update_status=False)
        if self._tri_test_running:
            self._on_stop_tri_test(reason="restarted by feedforward test", update_status=False)
        if self._local_test_running:
            self._on_stop_local_test(reason="restarted by feedforward test", update_status=False)
        if not self._interaction_enabled:
            if hasattr(self, "status_var"):
                self.status_var.set("Feedforward Test blocked: Hand Manual must be ON")
            return
        if not self._guard_joint_command_started():
            return
        validated = self._validate_ff_test_config(show_message=True)
        if not validated:
            return
        joint_idx, targets, step_s = validated

        try:
            if self._data_logger.is_running():
                self._stop_data_recording(reason="restart by feedforward test", keep_switch=True)
            self._start_data_recording()
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(True)
        except Exception as exc:
            messagebox.showerror("Start Feedforward Test Failed", f"Cannot start recording: {exc}")
            return

        self._ff_test_joint_idx = int(joint_idx)
        self._ff_test_targets = list(targets)
        self._ff_test_step_s = float(step_s)
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = time.time()
        self._ff_test_running = True
        self._set_test_button_states()
        self._emit_action(f"Feedforward Test started on J{joint_idx:02d}")
        if hasattr(self, "status_var"):
            self.status_var.set(f"Feedforward Test running on J{joint_idx:02d}")
        self._ff_test_tick()

    def _on_stop_ff_test(self, reason: str = "stopped by feedforward test", update_status: bool = True) -> None:
        was_running = bool(self._ff_test_running)
        self._ff_test_running = False
        job = getattr(self, "_ff_test_after_job", None)
        self._ff_test_after_job = None
        if job is not None:
            try:
                self.root.after_cancel(job)
            except Exception:
                pass
        self._ff_test_targets = []
        self._ff_test_idx = 0
        self._ff_test_t_accum = 0.0
        self._ff_test_last_ts = 0.0
        self._set_test_button_states()
        if self._data_logger.is_running():
            self._stop_data_recording(reason=reason, keep_switch=True)
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(False)
        if was_running:
            self._emit_action(f"Feedforward Test stopped ({reason})")
        if update_status and hasattr(self, "status_var"):
            self.status_var.set("Feedforward Test stopped")

    def _validate_tri_test_config(self, show_message: bool = True) -> Optional[Tuple[int, float, float, float]]:
        joint_text = self._normalize_test_joint_value(self._tri_test_joint_var.get())
        self._tri_test_joint_var.set(joint_text)
        if joint_text == "None":
            if show_message:
                messagebox.showwarning("Triangle Wave Test", "Select one valid joint.")
            return None
        joint_idx = int(joint_text[1:])

        try:
            raw_min = float(self._tri_test_min_var.get())
            raw_max = float(self._tri_test_max_var.get())
            freq = float(self._tri_test_freq_var.get())
        except (TypeError, ValueError, tk.TclError):
            if show_message:
                messagebox.showwarning("Triangle Wave Test", "Min/Max/Freq must be valid numbers.")
            return None

        if not np.isfinite(raw_min) or not np.isfinite(raw_max) or not np.isfinite(freq):
            if show_message:
                messagebox.showwarning("Triangle Wave Test", "Min/Max/Freq must be finite numbers.")
            return None
        if freq <= 0.0:
            if show_message:
                messagebox.showwarning("Triangle Wave Test", "Frequency must be > 0.")
            return None

        lo = self._clamp_joint_target_deg(joint_idx, min(raw_min, raw_max))
        hi = self._clamp_joint_target_deg(joint_idx, max(raw_min, raw_max))
        if hi < lo:
            lo, hi = hi, lo

        self._tri_test_min_var.set(f"{lo:.2f}")
        self._tri_test_max_var.set(f"{hi:.2f}")
        self._tri_test_freq_var.set(f"{freq:.4g}")
        return joint_idx, float(lo), float(hi), float(freq)

    def _schedule_tri_test_tick(self) -> None:
        if not self._tri_test_running:
            return
        self._tri_test_after_job = self.root.after(int(self._test_tick_ms), self._tri_test_tick)

    def _on_start_tri_test(self) -> None:
        if self._tri_test_running:
            return
        if self._test_running:
            self._on_stop_test(reason="restarted by triangle test", update_status=False)
        if self._ff_test_running:
            self._on_stop_ff_test(reason="restarted by triangle test", update_status=False)
        if self._local_test_running:
            self._on_stop_local_test(reason="restarted by triangle test", update_status=False)
        if not self._interaction_enabled:
            if hasattr(self, "status_var"):
                self.status_var.set("Triangle Wave Test blocked: Hand Manual must be ON")
            return
        if not self._guard_joint_command_started():
            return
        validated = self._validate_tri_test_config(show_message=True)
        if not validated:
            return
        joint_idx, lo, hi, freq = validated

        try:
            if self._data_logger.is_running():
                self._stop_data_recording(reason="restart by triangle test", keep_switch=True)
            self._start_data_recording()
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(True)
        except Exception as exc:
            messagebox.showerror("Start Triangle Test Failed", f"Cannot start recording: {exc}")
            return

        self._tri_test_joint_idx = int(joint_idx)
        self._tri_test_min_deg = float(lo)
        self._tri_test_max_deg = float(hi)
        self._tri_test_freq_hz = float(freq)
        self._tri_test_t0 = time.time()
        self._tri_test_running = True
        self._set_test_button_states()
        self._emit_action(f"Triangle Wave Test started on J{joint_idx:02d}")
        if hasattr(self, "status_var"):
            self.status_var.set(f"Triangle Wave Test running on J{joint_idx:02d}")
        self._tri_test_tick()

    def _on_stop_tri_test(self, reason: str = "stopped by triangle test", update_status: bool = True) -> None:
        was_running = bool(self._tri_test_running)
        self._tri_test_running = False
        job = getattr(self, "_tri_test_after_job", None)
        self._tri_test_after_job = None
        if job is not None:
            try:
                self.root.after_cancel(job)
            except Exception:
                pass
        self._tri_test_t0 = 0.0
        self._set_test_button_states()
        if self._data_logger.is_running():
            self._stop_data_recording(reason=reason, keep_switch=True)
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(False)
        if was_running:
            self._emit_action(f"Triangle Wave Test stopped ({reason})")
        if update_status and hasattr(self, "status_var"):
            self.status_var.set("Triangle Wave Test stopped")

    def _on_start_local_test(self) -> None:
        if self._local_test_running:
            return
        if self._test_running:
            self._on_stop_test(reason="restarted by local test", update_status=False)
        if self._ff_test_running:
            self._on_stop_ff_test(reason="restarted by local test", update_status=False)
        if self._tri_test_running:
            self._on_stop_tri_test(reason="restarted by local test", update_status=False)
        if not self._interaction_enabled:
            if hasattr(self, "status_var"):
                self.status_var.set("Local Test blocked: Hand Manual must be ON")
            return
        if not self._guard_joint_command_started():
            return

        try:
            freq_hz = float(self._local_freq_var.get())
            amplitude_deg = float(self._local_amp_var.get())
        except (TypeError, ValueError, tk.TclError):
            messagebox.showwarning("Local Test", "Freq(Hz) and Amp(deg) must be valid numbers.")
            return

        if (not np.isfinite(freq_hz)) or freq_hz <= 0.0:
            messagebox.showwarning("Local Test", "Freq(Hz) must be a finite number > 0.")
            return
        if not np.isfinite(amplitude_deg):
            messagebox.showwarning("Local Test", "Amp(deg) must be a finite number.")
            return

        self._local_freq_var.set(f"{freq_hz:.4g}")
        self._local_amp_var.set(f"{amplitude_deg:.4g}")

        try:
            if self._data_logger.is_running():
                self._stop_data_recording(reason="restart by local test", keep_switch=True)
            self._start_data_recording()
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(True)
            self.controller.set_local_test_params(float(freq_hz), float(amplitude_deg))
            self.controller.start_local_test()
        except Exception as exc:
            messagebox.showerror("Start Local Test Failed", f"{type(exc).__name__}: {exc}")
            return

        self._local_test_running = True
        self._set_test_button_states()
        self._emit_action(
            f"Local Test started (freq={float(freq_hz):.4g} Hz, amp={float(amplitude_deg):.4g} deg)"
        )
        if hasattr(self, "status_var"):
            self.status_var.set("Local Test running")

    def _on_stop_local_test(self, reason: str = "stopped by local test", update_status: bool = True) -> None:
        was_running = bool(self._local_test_running)
        self._local_test_running = False
        try:
            if self._is_comm_connected():
                self.controller.stop_local_test()
        except Exception:
            pass
        self._set_test_button_states()
        if self._data_logger.is_running():
            self._stop_data_recording(reason=reason, keep_switch=True)
            if hasattr(self, "_save_data_var"):
                self._save_data_var.set(False)
        if was_running:
            self._emit_action(f"Local Test stopped ({reason})")
        if update_status and hasattr(self, "status_var"):
            self.status_var.set("Local Test stopped")

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

    def _ff_test_tick(self) -> None:
        if not self._ff_test_running:
            return
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        if mode != "test":
            self._on_stop_ff_test(reason="stopped on mode switch", update_status=False)
            return
        if not self._interaction_enabled:
            self._on_stop_ff_test(reason="stopped: Hand Manual OFF")
            return
        if not self.controller.is_started():
            self._on_stop_ff_test(reason="stopped: press Enable to start controller")
            return
        if not self._ff_test_targets:
            self._on_stop_ff_test(reason="stopped: no feedforward targets")
            return

        now = time.time()
        dt = 0.0 if self._ff_test_last_ts <= 0.0 else max(0.0, now - self._ff_test_last_ts)
        self._ff_test_last_ts = now
        self._ff_test_t_accum += dt
        while self._ff_test_t_accum >= self._ff_test_step_s and self._ff_test_idx < (len(self._ff_test_targets) - 1):
            self._ff_test_t_accum -= self._ff_test_step_s
            self._ff_test_idx += 1

        target = self._clamp_joint_target_deg(
            self._ff_test_joint_idx,
            self._ff_test_targets[self._ff_test_idx],
        )
        angles = self._resolved_joint_targets()
        angles[self._ff_test_joint_idx] = target
        if self._ff_test_joint_idx < len(self._joint_target_vars):
            self._joint_target_vars[self._ff_test_joint_idx].set(target)
        if self._ff_test_joint_idx < len(self._joint_target_user_set):
            self._joint_target_user_set[self._ff_test_joint_idx] = True

        try:
            self._send_joint_targets_with_tendon_guard(angles, source="Feedforward Test", live=True)
        except Exception as exc:
            self._on_stop_ff_test(reason=f"stopped by send error: {exc}")
            return
        self._schedule_ff_test_tick()

    def _tri_test_tick(self) -> None:
        if not self._tri_test_running:
            return
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        if mode != "test":
            self._on_stop_tri_test(reason="stopped on mode switch", update_status=False)
            return
        if not self._interaction_enabled:
            self._on_stop_tri_test(reason="stopped: Hand Manual OFF")
            return
        if not self.controller.is_started():
            self._on_stop_tri_test(reason="stopped: press Enable to start controller")
            return

        elapsed = max(0.0, time.time() - float(self._tri_test_t0))
        phase = elapsed * float(self._tri_test_freq_hz)
        frac = phase - np.floor(phase)
        tri = 1.0 - 4.0 * abs(float(frac) - 0.5)
        center = 0.5 * (float(self._tri_test_min_deg) + float(self._tri_test_max_deg))
        amp = 0.5 * (float(self._tri_test_max_deg) - float(self._tri_test_min_deg))
        target = self._clamp_joint_target_deg(self._tri_test_joint_idx, center + amp * tri)

        angles = self._resolved_joint_targets()
        angles[self._tri_test_joint_idx] = target
        if self._tri_test_joint_idx < len(self._joint_target_vars):
            self._joint_target_vars[self._tri_test_joint_idx].set(target)
        if self._tri_test_joint_idx < len(self._joint_target_user_set):
            self._joint_target_user_set[self._tri_test_joint_idx] = True

        try:
            self._send_joint_targets_with_tendon_guard(angles, source="Triangle Wave Test", live=True)
        except Exception as exc:
            self._on_stop_tri_test(reason=f"stopped by send error: {exc}")
            return
        self._schedule_tri_test_tick()

    def _switch_control_mode(self):
        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        prev_mode = getattr(self, "_ctrl_mode_last", mode)
        if prev_mode == "test" and mode != "test":
            self._on_stop_test(reason="stopped on mode switch", update_status=False)
            self._on_stop_ff_test(reason="stopped on mode switch", update_status=False)
            self._on_stop_tri_test(reason="stopped on mode switch", update_status=False)
            self._on_stop_local_test(reason="stopped on mode switch", update_status=False)
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
                self._table_section.configure(text="Joint Tests")
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

        joint_has_data = bool(getattr(s, "has_sensor_data", False))
        for i in range(min(ENCODER_COUNT, len(self._joint_curr_vars))):
            if joint_has_data and i < len(s.angles):
                self._joint_curr_vars[i].set(f"{s.angles[i]:6.1f}")
            else:
                self._joint_curr_vars[i].set("-")

            if hasattr(self, "_joint_status_dots"):
                if joint_has_data and i < len(s.encoders):
                    self._set_joint_encoder_dot(i, not bool(s.encoders[i].error))
                else:
                    self._set_joint_encoder_dot(i, None)

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
        if getattr(self, "_test_running", False):
            self._on_stop_test(reason="stopped by E-STOP", update_status=False)
        if getattr(self, "_ff_test_running", False):
            self._on_stop_ff_test(reason="stopped by E-STOP", update_status=False)
        if getattr(self, "_tri_test_running", False):
            self._on_stop_tri_test(reason="stopped by E-STOP", update_status=False)
        if getattr(self, "_local_test_running", False):
            self._on_stop_local_test(reason="stopped by E-STOP", update_status=False)
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

