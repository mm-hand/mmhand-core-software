import os
import sys
import time
import select
import importlib
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

from comm_layer import choose_serial_port_interactive
from core_logic import HandController
from protocol import ENCODER_COUNT

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
    """Cross-platform non-blocking keyboard reader for single char."""

    def __init__(self):
        self._is_windows = os.name == "nt"
        self._fd = None
        self._old_term = None
        self._msvcrt = None
        if self._is_windows:
            try:
                import msvcrt  # type: ignore
                self._msvcrt = msvcrt
            except Exception:
                self._msvcrt = None

    def __enter__(self):
        if not self._is_windows:
            try:
                import termios
                import tty
                self._fd = sys.stdin.fileno()
                self._old_term = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
            except Exception:
                self._fd = None
                self._old_term = None
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self._is_windows and self._fd is not None and self._old_term is not None:
            try:
                import termios
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass

    def read_key(self) -> Optional[str]:
        if self._is_windows and self._msvcrt is not None:
            if self._msvcrt.kbhit():
                b = self._msvcrt.getch()
                try:
                    return b.decode("utf-8", errors="ignore").lower()
                except Exception:
                    return None
            return None

        if self._fd is None:
            return None
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = sys.stdin.read(1)
                return ch.lower()
        except Exception:
            return None
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
    if calib_status == "FAILED":
        return f"{Back.RED}{Fore.WHITE} Calibration FAILED {Style.RESET_ALL}"
    if calib_status == "NO_CALIB":
        return f"{Fore.YELLOW}NO_CALIB{Style.RESET_ALL}"
    return "IDLE"


def _snapshot_controller_state(controller: HandController) -> Dict[str, object]:
    with controller.state_lock:
        s = controller.current_state
        return {
            "timestamp": s.timestamp,
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
            "jd_valid": list(s.joint_debug_valid),
            "jd_target": list(s.joint_debug_target_deg),
            "jd_actual": list(s.joint_debug_actual_deg),
            "jd_l1": list(s.joint_debug_loop1_output),
            "jd_l2a": list(s.joint_debug_loop2_actual),
            "jd_l2o": list(s.joint_debug_loop2_output),
            "jd_cmd_valid": list(s.joint_debug_cmd_valid),
            "jd_cmd_pos": list(s.joint_debug_cmd_target_pos),
        }


def _print_console(controller: HandController, connected_port: str, plot_status: str) -> None:
    snap = _snapshot_controller_state(controller)
    last_update = float(snap["timestamp"])
    latency_ms = (time.time() - last_update) * 1000.0 if last_update > 0 else 9999.0
    rows = (ENCODER_COUNT + 2) // 3

    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()
    print(f"{Back.BLUE}{Fore.WHITE}  ServoBoard Monitor  {Style.RESET_ALL}")
    port_text = f"{Fore.GREEN}Connected: {connected_port}{Style.RESET_ALL}"
    latency_color = Fore.GREEN if latency_ms < 100 else (Fore.YELLOW if latency_ms < 500 else Fore.RED)
    plot_color = Fore.GREEN if "ENABLED" in plot_status else Fore.YELLOW
    print(
        f"Status: {port_text} | Latency: {latency_color}{latency_ms:.0f} ms{Style.RESET_ALL}"
        f" | Plot: {plot_color}{plot_status}{Style.RESET_ALL}"
    )
    print("-" * 88)

    print("Mapped encoder data (count | degree):")
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
    active_fault_idx = [str(i) for i, f in enumerate(overload_fault) if bool(f)]
    print(f"Overload latch: {', '.join(active_fault_idx) if active_fault_idx else 'none'}")
    print("-" * 88)

    print("Debug joints (packet 0x04):")
    jd_valid = snap["jd_valid"]
    jd_target = snap["jd_target"]
    jd_actual = snap["jd_actual"]
    jd_l1 = snap["jd_l1"]
    jd_l2a = snap["jd_l2a"]
    jd_l2o = snap["jd_l2o"]
    jd_cmd_valid = snap["jd_cmd_valid"]
    jd_cmd_pos = snap["jd_cmd_pos"]
    for idx in [j for j in DEBUG_JOINTS if 0 <= j < ENCODER_COUNT]:
        cmd_text = f"{int(jd_cmd_pos[idx]):6d}" if bool(jd_cmd_valid[idx]) else "  N/A "
        print(
            f"J{idx:02d} valid={int(bool(jd_valid[idx]))} "
            f"target={float(jd_target[idx]):8.3f} actual={float(jd_actual[idx]):8.3f} "
            f"loop1={float(jd_l1[idx]):8.3f} loop2a={float(jd_l2a[idx]):8.3f} "
            f"loop2o={float(jd_l2o[idx]):8.3f} cmd={cmd_text}"
        )
    print("-" * 88)
    print("Controls: 's' start, 'x' stop, 'r' reset, 'c' calibrate, 'q' quit.")
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

    def update(self, controller: HandController) -> None:
        snap = _snapshot_controller_state(controller)
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


def run_development_mode(*, no_plot: bool = False) -> None:
    selected_port = choose_serial_port_interactive()
    if not selected_port:
        return

    controller = HandController()
    if not controller.initialize(selected_port):
        print(f"Failed to connect to {selected_port}.")
        return

    print(f"Connecting to {selected_port} ...")
    controller.set_pid_control(False)
    controller.start()

    plotter = None
    plot_status = "DISABLED (--no-plot)" if no_plot else "DISABLED"
    if not no_plot:
        try:
            plotter = _JointPlotter()
            plot_status = "ENABLED"
        except Exception as exc:
            plotter = None
            plot_status = f"DISABLED ({exc})"

    try:
        with _KeyReader() as key_reader:
            while True:
                _print_console(controller, selected_port, plot_status)
                if plotter is not None:
                    plotter.update(controller)

                key = key_reader.read_key()
                if key == "q":
                    break
                if key == "s":
                    controller.start()
                if key == "x":
                    controller.stop()
                if key == "r":
                    controller.reset()
                if key == "c":
                    if controller.comm and controller.comm.serial and controller.comm.serial.is_open:
                        controller.comm.send_command("calibrate")
                        with controller.state_lock:
                            controller.current_state.calib_status = "PENDING"
                time.sleep(0.06)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        print("\nMonitor exited.")
