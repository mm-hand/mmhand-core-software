"""cli_monitor.py - MMHand Desktop 模块契约。

职责：提供无图形界面的设备连接、状态查看和诊断命令。
线程：在进程主线程执行；仅负责创建和关闭 CLI、Qt 及可选 ROS2 生命周期。
输入：命令行参数、环境变量和启动配置。
输出：已装配的应用服务、进程退出码和启动诊断。
状态所有权：只拥有启动期配置与生命周期句柄，不拥有设备运行状态。
禁止依赖：禁止实现协议解析、控制算法和设备安全判断，统一委托 core/服务层。
"""
import os
import time
import importlib
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

from desktop.core.comm_layer import choose_serial_port_interactive
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import ENCODER_COUNT
from desktop.ros2.joint_targets import DEFAULT_JOINT_TARGET_TOPIC, Ros2JointTargetBridge
from desktop.ros2.raw_ik_target import RawIkTargetBridge

try:
    _colorama = importlib.import_module("colorama")
    Back = _colorama.Back
    Fore = _colorama.Fore
    Style = _colorama.Style
    _colorama.init(autoreset=True)
except Exception:
    class _DummyColor:
        def __getattr__(self, _name):
            return ""
    Fore = Back = Style = _DummyColor()

DEBUG_JOINTS = [4, 5, 6, 7]
PLOT_MAX_POINTS = 1000
PLOT_WINDOW_SEC = 10.0


class _KeyReader:
    """Windows non-blocking keyboard reader for single char."""

    def __init__(self):
        self._msvcrt = None
        try:
            import msvcrt  # type: ignore
            self._msvcrt = msvcrt
        except Exception:
            self._msvcrt = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return None

    def read_key(self) -> Optional[str]:
        if self._msvcrt is None or not self._msvcrt.kbhit():
            return None
        b = self._msvcrt.getch()
        try:
            return b.decode("utf-8", errors="ignore").lower()
        except Exception:
            return None


def _fmt_sensor_cell(index: int, raw: int, error: bool) -> str:
    if error:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} DISCONNECT {Style.RESET_ALL}"
    deg = raw * 360.0 / 16384.0
    color = Fore.CYAN if index % 2 == 0 else Fore.WHITE
    return f"[{index:02d}] {color}{raw:6d} ({deg:7.2f} deg){Style.RESET_ALL}"


def _fmt_servo_cell(index: int, angle: int, online: bool) -> str:
    if not online:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} OFFLINE {Style.RESET_ALL}"
    color = Fore.GREEN if index % 2 == 0 else Fore.YELLOW
    return f"[{index:02d}] {color}{angle:7d}{Style.RESET_ALL}"


def _fmt_telem_cell(index: int, speed: int, load: int, voltage: int, temp: int, online: bool) -> str:
    if not online:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} OFFLINE {Style.RESET_ALL}"
    color = Fore.MAGENTA if index % 2 == 0 else Fore.CYAN
    return f"[{index:02d}] {color}S:{speed:6d} L:{load:6d} V:{voltage / 10.0:4.1f} T:{temp:3d}{Style.RESET_ALL}"


def _render_calib_status(calib_status: str) -> str:
    if calib_status == "PENDING":
        return f"{Fore.YELLOW}Sending calibration request...{Style.RESET_ALL}"
    if calib_status == "SUCCESS":
        return f"{Back.GREEN}{Fore.WHITE} Calibration SUCCESS {Style.RESET_ALL}"
    if calib_status == "WARN":
        return f"{Back.YELLOW}{Fore.BLACK} Calibration WARN {Style.RESET_ALL}"
    if calib_status == "FAILED":
        return f"{Back.RED}{Fore.WHITE} Calibration FAILED {Style.RESET_ALL}"
    if calib_status == "NO_CALIB":
        return f"{Fore.YELLOW}NO_CALIB{Style.RESET_ALL}"
    return "IDLE"


def _snapshot_backend_state(backend: HandBackend) -> Dict[str, object]:
    s = backend.get_state_snapshot()
    return {
        "host_rx_time": s.host_rx_time,
        "p4_time_ms": s.p4_time_ms if s.has_p4_time else None,
        "calib_status": s.calib_status,
        "enc_raw": [e.raw for e in s.encoders],
        "enc_err": [e.error for e in s.encoders],
        "servo_angles": list(s.servo_angles),
        "servo_online": list(s.servo_online),
        "servo_speed": list(s.servo_speed),
        "servo_load": list(s.servo_load),
        "servo_voltage": list(s.servo_voltage),
        "servo_temp": list(s.servo_temperature),
        "servo_telem_online": list(s.servo_telem_online),
        "servo_overload_fault": list(s.servo_overload_fault),
        "servo_overload_trigger_load": list(getattr(s, "servo_overload_trigger_load", [])),
    }


def _print_console(backend: HandBackend, connected_port: str, plot_status: str, ros_status: str = "") -> None:
    snap = _snapshot_backend_state(backend)
    last_update = float(snap["timestamp"])
    latency_ms = (time.time() - last_update) * 1000.0 if last_update > 0 else 9999.0
    rows = (ENCODER_COUNT + 2) // 3

    os.system("cls")
    print(f"{Back.BLUE}{Fore.WHITE}  ServoBoard Monitor  {Style.RESET_ALL}")
    port_text = f"{Fore.GREEN}Connected: {connected_port}{Style.RESET_ALL}"
    latency_color = Fore.GREEN if latency_ms < 100 else (Fore.YELLOW if latency_ms < 500 else Fore.RED)
    plot_color = Fore.GREEN if "ENABLED" in plot_status else Fore.YELLOW
    status_line = (
        f"Status: {port_text} | Latency: {latency_color}{latency_ms:.0f} ms{Style.RESET_ALL}"
        f" | Plot: {plot_color}{plot_status}{Style.RESET_ALL}"
    )
    if ros_status:
        status_line += f" | ROS2: {ros_status}"
    print(status_line)
    print("-" * 88)

    print("Raw magnetic encoder data (count | degree):")
    enc_raw = snap["enc_raw"]
    enc_err = snap["enc_err"]
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(_fmt_sensor_cell(idx, int(enc_raw[idx]), bool(enc_err[idx])))
        print("   ".join(cells))
    print("-" * 88)

    print("Servo absolute positions:")
    servo_angles = snap["servo_angles"]
    servo_online = snap["servo_online"]
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(_fmt_servo_cell(idx, int(servo_angles[idx]), bool(servo_online[idx])))
        print("   ".join(cells))
    print("-" * 88)

    print("Servo telemetry (speed/load/voltage/temp):")
    servo_speed = snap["servo_speed"]
    servo_load = snap["servo_load"]
    servo_voltage = snap["servo_voltage"]
    servo_temp = snap["servo_temp"]
    telem_online = snap["servo_telem_online"]
    overload_fault = snap["servo_overload_fault"]
    overload_trigger_load = snap["servo_overload_trigger_load"]
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(
                    _fmt_telem_cell(
                        idx,
                        int(servo_speed[idx]),
                        int(servo_load[idx]),
                        int(servo_voltage[idx]),
                        int(servo_temp[idx]),
                        bool(telem_online[idx]),
                    )
                )
        print("   ".join(cells))
    active_fault_idx = [
        f"M{i:02d}:{int(overload_trigger_load[i]) if i < len(overload_trigger_load) and overload_trigger_load[i] is not None else '?'}"
        for i, f in enumerate(overload_fault)
        if bool(f)
    ]
    print(f"Overload latch: {', '.join(active_fault_idx) if active_fault_idx else 'none'}")
    print("-" * 88)

    print("Debug joints (packet 0x04):")
    jd_valid = snap["jd_valid"]
    jd_target = snap["jd_target"]
    jd_actual = snap["jd_actual"]
    jd_l1 = snap["jd_l1"]
    jd_cmd_valid = snap["jd_cmd_valid"]
    jd_cmd_pos = snap["jd_cmd_pos"]
    for idx in [j for j in DEBUG_JOINTS if 0 <= j < ENCODER_COUNT]:
        cmd_text = f"{int(jd_cmd_pos[idx]):6d}" if bool(jd_cmd_valid[idx]) else "  N/A "
        print(
            f"J{idx:02d} valid={int(bool(jd_valid[idx]))} "
            f"target={float(jd_target[idx]):8.3f} actual={float(jd_actual[idx]):8.3f} "
            f"loop1={float(jd_l1[idx]):8.3f} cmd={cmd_text}"
        )
    print("-" * 88)
    print(
        "Controls: 's' start, 'i' RAW IK enable, 'o' RAW IK disable, "
        "'x' stop, 'r' reset, 'c' calibrate, 'q' quit."
    )
    print(f"Calibration: {_render_calib_status(str(snap['calib_status']))}")


@dataclass
class _JointPlotBuffers:
    t: Dict[int, Deque[float]] = field(default_factory=lambda: {j: deque(maxlen=PLOT_MAX_POINTS) for j in DEBUG_JOINTS})
    target: Dict[int, Deque[float]] = field(default_factory=lambda: {j: deque(maxlen=PLOT_MAX_POINTS) for j in DEBUG_JOINTS})
    actual: Dict[int, Deque[float]] = field(default_factory=lambda: {j: deque(maxlen=PLOT_MAX_POINTS) for j in DEBUG_JOINTS})
    t0: Dict[int, float] = field(default_factory=lambda: {j: 0.0 for j in DEBUG_JOINTS})


class _JointPlotter:
    def __init__(self):
        import matplotlib.pyplot as plt

        self._plt = plt
        self._buf = _JointPlotBuffers()
        self._plt.ion()
        self.fig, self.ax = self._plt.subplots()
        self.lines: Dict[int, Tuple[object, object]] = {}
        for idx in DEBUG_JOINTS:
            (line_t,) = self.ax.plot([], [], linestyle="--", label=f"J{idx} Target")
            (line_a,) = self.ax.plot([], [], linestyle="-", label=f"J{idx} Actual")
            self.lines[idx] = (line_t, line_a)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Angle (deg)")
        self.ax.set_title("Debug joints target vs actual")
        self.ax.grid(True)
        self.ax.legend(loc="upper right", ncol=2, fontsize=8)
        self._plt.show(block=False)

    def update(self, backend: HandBackend) -> None:
        snap = _snapshot_backend_state(backend)
        now = time.time()
        jd_valid = snap["jd_valid"]
        jd_target = snap["jd_target"]
        jd_actual = snap["jd_actual"]

        for j in DEBUG_JOINTS:
            if self._buf.t0[j] == 0.0:
                self._buf.t0[j] = now
            self._buf.t[j].append(now - self._buf.t0[j])
            self._buf.target[j].append(float(jd_target[j]))
            self._buf.actual[j].append(float(jd_actual[j]) if bool(jd_valid[j]) else float("nan"))

            line_t, line_a = self.lines[j]
            t = list(self._buf.t[j])
            line_t.set_data(t, list(self._buf.target[j]))
            line_a.set_data(t, list(self._buf.actual[j]))

        all_t = [v for j in DEBUG_JOINTS for v in self._buf.t[j]]
        if all_t:
            t_max = max(all_t)
            t_min = max(0.0, t_max - PLOT_WINDOW_SEC)
            self.ax.set_xlim(t_min, t_max + 0.1)

        vals: List[float] = []
        for j in DEBUG_JOINTS:
            vals.extend([v for v in self._buf.target[j] if v == v])
            vals.extend([v for v in self._buf.actual[j] if v == v])
        if vals:
            y_min, y_max = min(vals), max(vals)
            if y_min == y_max:
                y_max = y_min + 1.0
            margin = (y_max - y_min) * 0.1
            self.ax.set_ylim(y_min - margin, y_max + margin)

        self.fig.canvas.draw_idle()
        self._plt.pause(0.001)


def _mark_calibration_pending(backend: HandBackend) -> None:
    try:
        with backend.state_lock:
            backend.current_state.calib_status = "PENDING"
    except Exception:
        pass


def _make_ros2_bridge(
    backend: HandBackend,
    *,
    ros2_input: str,
    ros_topic: str,
    auto_start: bool,
    raw_ik_space: str = "legacy_calibrated_deg",
):
    if ros2_input == "raw_ik":
        return RawIkTargetBridge(
            backend,
            topic=ros_topic,
            auto_start=auto_start,
            input_space=raw_ik_space,
        )
    return Ros2JointTargetBridge(backend, topic=ros_topic, auto_start=auto_start)


def run_cli_monitor(
    *,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    no_plot: bool = False,
    ros2_enabled: bool = False,
    ros2_input: str = "joint_target",
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
    auto_start: bool = False,
    ros2_raw_ik_space: str = "legacy_calibrated_deg",
    backend: Optional[HandBackend] = None,
    key_reader_factory: Callable[[], _KeyReader] = _KeyReader,
    max_iterations: Optional[int] = None,
    render: bool = True,
    sleep_s: float = 0.06,
) -> int:
    selected_port = port or choose_serial_port_interactive()
    if not selected_port:
        return 1

    backend = backend or HandBackend()
    if not backend.connect(selected_port):
        print(f"Failed to connect to {selected_port}.")
        return 2

    print(f"Connecting to {selected_port} ...")
    backend.set_pid_control(bool(ros2_enabled))
    if tactile_port:
        if backend.connect_tactile_serial(tactile_port):
            print(f"Connected tactile serial: {tactile_port}")
        else:
            print(f"Failed to connect tactile serial: {tactile_port}")
    if auto_start:
        backend.start()

    plotter = None
    plot_status = "DISABLED (--no-plot)" if no_plot else "DISABLED"
    if not no_plot and render:
        try:
            plotter = _JointPlotter()
            plot_status = "ENABLED"
        except Exception as exc:
            plotter = None
            plot_status = f"DISABLED ({exc})"

    ros_bridge = None
    ros_status = "OFF"
    if ros2_enabled:
        ros_bridge = _make_ros2_bridge(
            backend,
            ros2_input=ros2_input,
            ros_topic=ros_topic,
            auto_start=auto_start,
            raw_ik_space=ros2_raw_ik_space,
        )
        try:
            ros_bridge.start()
            ros_status = f"ON {ros2_input} {ros_topic}"
            if auto_start:
                print("Controller auto-started; RAW IK output remains disabled")
        except Exception as exc:
            print(f"ROS2 disabled: {exc}")
            ros_bridge = None
            ros_status = f"ERROR {exc}"

    iterations = 0
    try:
        with key_reader_factory() as key_reader:
            while True:
                if render:
                    _print_console(backend, selected_port, plot_status, ros_status)
                if plotter is not None:
                    plotter.update(backend)

                key = key_reader.read_key()
                if key == "q":
                    break
                if key == "s":
                    backend.start()
                if key == "i" and ros_bridge is not None:
                    # CLI 与 GUI 使用同一独立授权门；普通 start 不得放行遥操。
                    ok, message = backend.set_raw_ik_output_enabled(True)
                    print(message if ok else f"RAW IK enable blocked: {message}")
                if key == "o" and ros_bridge is not None:
                    _, message = backend.set_raw_ik_output_enabled(False)
                    print(message)
                if key == "x":
                    backend.stop()
                if key == "r":
                    backend.reset()
                if key == "c":
                    if backend.request_firmware_calibration():
                        _mark_calibration_pending(backend)
                iterations += 1
                if max_iterations is not None and iterations >= max_iterations:
                    break
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass
    finally:
        if ros_bridge is not None:
            ros_bridge.stop()
        backend.shutdown()
        print("\nMonitor exited.")
    return 0
