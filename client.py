import os
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional

import serial
import serial.tools.list_ports

try:
    import msvcrt
except ImportError:
    msvcrt = None

try:
    from colorama import Back, Fore, Style, init

    init(autoreset=True)
except ImportError:
    class _DummyColor:
        def __getattr__(self, _name):
            return ""

    Fore = Back = Style = _DummyColor()


TARGET_BAUDRATE = 921600
ENCODER_COUNT = 21
DEBUG_JOINTS = [4, 5, 6, 7]
PLOT_JOINTS = [4, 5, 6, 7]

PACKET_HEADER = 0xFE
PACKET_TAIL = 0xFF
PACKET_TYPE_SENSOR = 0x01
PACKET_TYPE_CALIB_ACK = 0x02
PACKET_TYPE_SERVO_ANGLE = 0x03
PACKET_TYPE_SERVO_TELEM = 0x05
PACKET_TYPE_JOINT1_DEBUG = 0x04
PACKET_TYPE_PROTO_ACK = 0x06

CMD_CALIBRATE = b"\xCA"
CMD_SENSOR_STREAM_MODE = 0xD1
DISCONNECT_SENTINEL = 0x7FFF

SENSOR_STREAM_MODE_LEGACY_U16_WRAP = 0
SENSOR_STREAM_MODE_SIGNED_I16 = 1
PROTO_ACK_STATUS_OK = 0


def parse_stream_mode_request(argv: List[str]) -> Optional[int]:
    requested: Optional[int] = SENSOR_STREAM_MODE_SIGNED_I16
    for arg in argv:
        if not arg.startswith("--stream-mode="):
            continue
        mode_text = arg.split("=", 1)[1].strip().lower()
        if mode_text == "signed":
            requested = SENSOR_STREAM_MODE_SIGNED_I16
        elif mode_text == "legacy":
            requested = SENSOR_STREAM_MODE_LEGACY_U16_WRAP
        elif mode_text == "off":
            requested = None
        else:
            print(f"Unknown --stream-mode value '{mode_text}', fallback to 'signed'.")
            requested = SENSOR_STREAM_MODE_SIGNED_I16
    return requested


REQUESTED_STREAM_MODE = parse_stream_mode_request(sys.argv[1:])

PLOT_MAX_POINTS = 1000
PLOT_WINDOW_SEC = 10.0
PLOT_ENABLED = "--no-plot" not in sys.argv
PLOT_STATUS = "DISABLED"
HAS_PLOT = False
if PLOT_ENABLED:
    try:
        import matplotlib

        try:
            backend = matplotlib.get_backend()
            if backend and backend.lower() == "agg":
                matplotlib.use("TkAgg")
        except Exception:
            matplotlib.use("TkAgg")

        import matplotlib.pyplot as plt

        HAS_PLOT = True
        PLOT_STATUS = f"ENABLED ({matplotlib.get_backend()})"
    except Exception as exc:
        PLOT_ENABLED = False
        HAS_PLOT = False
        PLOT_STATUS = f"DISABLED ({exc})"
else:
    PLOT_STATUS = "DISABLED (--no-plot)"

def _plot_joint_indices() -> List[int]:
    return [idx for idx in PLOT_JOINTS if 0 <= idx < ENCODER_COUNT]


def _make_plot_buffers() -> Dict[int, Deque[float]]:
    return {idx: deque(maxlen=PLOT_MAX_POINTS) for idx in _plot_joint_indices()}


def _make_plot_start_times() -> Dict[int, float]:
    return {idx: 0.0 for idx in _plot_joint_indices()}


@dataclass
class JointDebugInfo:
    valid: bool = False
    target_deg: float = 0.0
    mag_actual_deg: float = 0.0
    loop1_output: float = 0.0
    loop2_actual: float = 0.0
    loop2_output: float = 0.0
    last_update: float = 0.0


@dataclass
class MachineState:
    mapped_counts: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    mapped_disconnect: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)

    servo_angles: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    servo_online: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)

    servo_speed: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    servo_load: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    servo_voltage: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    servo_temperature: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    servo_telem_online: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)

    joint_debug: List[JointDebugInfo] = field(
        default_factory=lambda: [JointDebugInfo() for _ in range(ENCODER_COUNT)]
    )

    plot_time: Dict[int, Deque[float]] = field(default_factory=_make_plot_buffers)
    plot_target: Dict[int, Deque[float]] = field(default_factory=_make_plot_buffers)
    plot_actual: Dict[int, Deque[float]] = field(default_factory=_make_plot_buffers)
    plot_start_time: Dict[int, float] = field(default_factory=_make_plot_start_times)

    calib_status: str = "IDLE"
    calib_timestamp: float = 0.0

    last_update: float = 0.0
    connected_port: Optional[str] = None
    running: bool = True
    ser: Optional[serial.Serial] = None

    lock: threading.Lock = field(default_factory=threading.Lock)


state = MachineState()


def process_sensor_packet(payload: bytes) -> None:
    if len(payload) != ENCODER_COUNT * 2:
        return

    with state.lock:
        for i in range(ENCODER_COUNT):
            raw_u16 = (payload[i * 2] << 8) | payload[i * 2 + 1]
            if raw_u16 == DISCONNECT_SENTINEL:
                state.mapped_disconnect[i] = True
                state.mapped_counts[i] = 0
            else:
                state.mapped_disconnect[i] = False
                state.mapped_counts[i] = struct.unpack(">h", payload[i * 2:i * 2 + 2])[0]

        state.last_update = time.time()


def process_calib_ack_packet(payload: bytes) -> None:
    if len(payload) < 1:
        return

    code = payload[0]
    with state.lock:
        if code == 1:
            state.calib_status = "PENDING"
        elif code == 2:
            state.calib_status = "SUCCESS"
            state.calib_timestamp = time.time()
        elif code == 3:
            state.calib_status = "FAILED"
            state.calib_timestamp = time.time()


def process_proto_ack_packet(payload: bytes) -> None:
    if len(payload) < 3:
        return
    ack_cmd, applied_mode, status = payload[0], payload[1], payload[2]
    if ack_cmd != CMD_SENSOR_STREAM_MODE:
        return
    if applied_mode == SENSOR_STREAM_MODE_SIGNED_I16:
        mode_name = "signed"
    elif applied_mode == SENSOR_STREAM_MODE_LEGACY_U16_WRAP:
        mode_name = "legacy"
    else:
        mode_name = str(applied_mode)
    if status == PROTO_ACK_STATUS_OK:
        print(f"\nProtocol ACK: sensor stream mode = {mode_name}")
    else:
        print(f"\nProtocol ACK warning: request rejected, applied={mode_name}, status={status}")


def process_servo_angle_packet(payload: bytes) -> None:
    expected_len = ENCODER_COUNT * 4 + ENCODER_COUNT
    if len(payload) != expected_len:
        return

    with state.lock:
        for i in range(ENCODER_COUNT):
            state.servo_angles[i] = struct.unpack(">i", payload[i * 4:i * 4 + 4])[0]

        online_offset = ENCODER_COUNT * 4
        for i in range(ENCODER_COUNT):
            state.servo_online[i] = payload[online_offset + i] == 1

        state.last_update = time.time()


def process_servo_telem_packet(payload: bytes) -> None:
    expected_len = ENCODER_COUNT * 7
    if len(payload) != expected_len:
        return

    with state.lock:
        for i in range(ENCODER_COUNT):
            base = i * 7
            state.servo_speed[i] = struct.unpack(">h", payload[base:base + 2])[0]
            state.servo_load[i] = struct.unpack(">h", payload[base + 2:base + 4])[0]
            state.servo_voltage[i] = payload[base + 4]
            state.servo_temperature[i] = payload[base + 5]
            state.servo_telem_online[i] = payload[base + 6] == 1

        state.last_update = time.time()


def process_joint1_debug_packet(payload: bytes) -> None:
    expected_len = 2 + 5 * 4
    if len(payload) != expected_len:
        return

    joint_index, valid, target_deg, mag_actual_deg, loop1_output, loop2_actual, loop2_output = struct.unpack(
        ">BBfffff", payload
    )

    now = time.time()
    with state.lock:
        if joint_index < ENCODER_COUNT:
            jd = state.joint_debug[joint_index]
            jd.valid = valid == 1
            jd.target_deg = target_deg
            jd.mag_actual_deg = mag_actual_deg
            jd.loop1_output = loop1_output
            jd.loop2_actual = loop2_actual
            jd.loop2_output = loop2_output
            jd.last_update = now
        state.last_update = now

        if joint_index in state.plot_time:
            if state.plot_start_time.get(joint_index, 0.0) == 0.0:
                state.plot_start_time[joint_index] = now
            t = now - state.plot_start_time[joint_index]
            actual_val = mag_actual_deg if valid == 1 else float("nan")
            state.plot_time[joint_index].append(t)
            state.plot_target[joint_index].append(target_deg)
            state.plot_actual[joint_index].append(actual_val)


def parse_stream(buffer: bytearray) -> bytearray:
    while len(buffer) >= 4:
        try:
            header_index = buffer.index(PACKET_HEADER)
        except ValueError:
            return bytearray()

        if header_index > 0:
            buffer = buffer[header_index:]

        if len(buffer) < 2:
            break

        payload_len = buffer[1]
        frame_len = payload_len + 2
        if len(buffer) < frame_len:
            break

        frame = buffer[:frame_len]
        buffer = buffer[frame_len:]

        if frame[-1] != PACKET_TAIL:
            continue

        packet_type = frame[2]
        payload = frame[3:-1]

        if packet_type == PACKET_TYPE_SENSOR:
            process_sensor_packet(payload)
        elif packet_type == PACKET_TYPE_CALIB_ACK:
            process_calib_ack_packet(payload)
        elif packet_type == PACKET_TYPE_SERVO_ANGLE:
            process_servo_angle_packet(payload)
        elif packet_type == PACKET_TYPE_SERVO_TELEM:
            process_servo_telem_packet(payload)
        elif packet_type == PACKET_TYPE_JOINT1_DEBUG:
            process_joint1_debug_packet(payload)
        elif packet_type == PACKET_TYPE_PROTO_ACK:
            process_proto_ack_packet(payload)

    return buffer


def serial_thread(port_name: str) -> None:
    try:
        ser = serial.Serial(port_name, TARGET_BAUDRATE, timeout=0.1)
        with state.lock:
            state.ser = ser
            state.connected_port = port_name

        if REQUESTED_STREAM_MODE is not None:
            ser.write(bytes([CMD_SENSOR_STREAM_MODE, REQUESTED_STREAM_MODE & 0xFF]))

        buffer = bytearray()
        while state.running:
            if ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))
                buffer = parse_stream(buffer)
            else:
                time.sleep(0.005)
    except Exception as exc:
        with state.lock:
            state.connected_port = None
            state.running = False
        print(f"\nSerial error: {exc}")


def format_mapped_cell(index: int, mapped_count: int, disconnected: bool) -> str:
    if disconnected:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} DISCONNECT {Style.RESET_ALL}"

    deg = mapped_count * 360.0 / 16384.0
    color = Fore.CYAN if index % 2 == 0 else Fore.WHITE
    return f"[{index:02d}] {color}{mapped_count:6d} ({deg:7.2f} deg){Style.RESET_ALL}"


def format_servo_cell(index: int, angle: int, online: bool) -> str:
    if not online:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} OFFLINE {Style.RESET_ALL}"

    color = Fore.GREEN if index % 2 == 0 else Fore.YELLOW
    return f"[{index:02d}] {color}{angle:7d}{Style.RESET_ALL}"


def format_telemetry_cell(index: int,
                          speed: int,
                          load: int,
                          voltage: int,
                          temperature: int,
                          online: bool) -> str:
    if not online:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} OFFLINE {Style.RESET_ALL}"

    color = Fore.MAGENTA if index % 2 == 0 else Fore.CYAN
    v = voltage / 10.0
    return f"[{index:02d}] {color}S:{speed:6d} L:{load:6d} V:{v:4.1f} T:{temperature:3d}{Style.RESET_ALL}"


def render_calib_status(calib_status: str, calib_timestamp: float) -> str:
    elapsed = time.time() - calib_timestamp

    if calib_status == "IDLE":
        return f"{Fore.WHITE}Waiting command...{Style.RESET_ALL}"
    if calib_status == "PENDING":
        return f"{Fore.YELLOW}Sending calibration request...{Style.RESET_ALL}"
    if calib_status == "SUCCESS":
        if elapsed < 5.0:
            return f"{Back.GREEN}{Fore.WHITE} Calibration SUCCESS {Style.RESET_ALL}"
        return ""
    if calib_status == "FAILED":
        if elapsed < 5.0:
            return f"{Back.RED}{Fore.WHITE} Calibration FAILED {Style.RESET_ALL}"
        return ""
    return ""


if HAS_PLOT:
    class JointPlotter:
        def __init__(self) -> None:
            plt.ion()
            self.fig, self.ax = plt.subplots()
            self.fig.canvas.mpl_connect("close_event", self._on_close)
            self.plot_joints = _plot_joint_indices()
            self.lines = {}
            colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
            for idx, joint in enumerate(self.plot_joints):
                color = colors[idx % len(colors)] if colors else None
                (line_target,) = self.ax.plot(
                    [], [], label=f"J{joint} Target", linestyle="--", color=color
                )
                (line_actual,) = self.ax.plot(
                    [], [], label=f"J{joint} Actual", linestyle="-", color=color
                )
                self.lines[joint] = (line_target, line_actual)
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("Angle (deg)")
            if self.plot_joints:
                joints_text = ",".join(str(j) for j in self.plot_joints)
                self.ax.set_title(f"Joints {joints_text} Target vs Actual")
            else:
                self.ax.set_title("Joint Target vs Actual")
            self.ax.grid(True)
            self.ax.legend(loc="upper right", ncol=2, fontsize=8)
            plt.show(block=False)

        def _on_close(self, _event) -> None:
            state.running = False

        def update(self) -> None:
            with state.lock:
                series = {
                    idx: (
                        list(state.plot_time.get(idx, [])),
                        list(state.plot_target.get(idx, [])),
                        list(state.plot_actual.get(idx, [])),
                    )
                    for idx in self.plot_joints
                }

            all_t = [v for t, _, _ in series.values() for v in t]
            if not all_t:
                plt.pause(0.001)
                return
            for idx in self.plot_joints:
                t, y_target, y_actual = series.get(idx, ([], [], []))
                line_target, line_actual = self.lines[idx]
                line_target.set_data(t, y_target)
                line_actual.set_data(t, y_actual)

            t_max = max(all_t)
            t_min = max(0.0, t_max - PLOT_WINDOW_SEC)
            self.ax.set_xlim(t_min, t_max + 0.1)

            vals = []
            for _, y_target, y_actual in series.values():
                for v in y_target + y_actual:
                    if v == v:
                        vals.append(v)
            if vals:
                y_min = min(vals)
                y_max = max(vals)
            else:
                y_min, y_max = 0.0, 1.0

            if y_min == y_max:
                y_max = y_min + 1.0

            margin = (y_max - y_min) * 0.1
            self.ax.set_ylim(y_min - margin, y_max + margin)

            self.fig.canvas.draw_idle()
            plt.pause(0.001)


def print_ui() -> None:
    with state.lock:
        connected_port = state.connected_port
        last_update = state.last_update
        mapped_counts = list(state.mapped_counts)
        mapped_disconnect = list(state.mapped_disconnect)
        servo_angles = list(state.servo_angles)
        servo_online = list(state.servo_online)
        servo_speed = list(state.servo_speed)
        servo_load = list(state.servo_load)
        servo_voltage = list(state.servo_voltage)
        servo_temperature = list(state.servo_temperature)
        servo_telem_online = list(state.servo_telem_online)
        joint_debug = [
            JointDebugInfo(
                valid=state.joint_debug[i].valid,
                target_deg=state.joint_debug[i].target_deg,
                mag_actual_deg=state.joint_debug[i].mag_actual_deg,
                loop1_output=state.joint_debug[i].loop1_output,
                loop2_actual=state.joint_debug[i].loop2_actual,
                loop2_output=state.joint_debug[i].loop2_output,
                last_update=state.joint_debug[i].last_update,
            )
            for i in range(ENCODER_COUNT)
        ]
        calib_status = state.calib_status
        calib_timestamp = state.calib_timestamp

    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()

    title = f"{Back.BLUE}{Fore.WHITE}  ServoBoard Monitor  {Style.RESET_ALL}"
    print(title)

    port_text = (
        f"{Fore.GREEN}Connected: {connected_port}{Style.RESET_ALL}"
        if connected_port
        else f"{Fore.RED}Disconnected{Style.RESET_ALL}"
    )

    latency_ms = (time.time() - last_update) * 1000.0 if last_update > 0 else 9999.0
    latency_color = Fore.GREEN if latency_ms < 100 else Fore.YELLOW if latency_ms < 500 else Fore.RED
    plot_color = Fore.GREEN if HAS_PLOT and PLOT_ENABLED else Fore.YELLOW
    print(
        f"Status: {port_text} | Latency: {latency_color}{latency_ms:.0f} ms{Style.RESET_ALL}"
        f" | Plot: {plot_color}{PLOT_STATUS}{Style.RESET_ALL}"
    )
    print("-" * 88)

    print(f"{Style.BRIGHT}Mapped encoder data (count | degree):{Style.RESET_ALL}")
    rows = (ENCODER_COUNT + 2) // 3
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(format_mapped_cell(idx, mapped_counts[idx], mapped_disconnect[idx]))
        print("   ".join(cells))

    print("-" * 88)

    print(f"{Style.BRIGHT}Servo absolute positions:{Style.RESET_ALL}")
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(format_servo_cell(idx, servo_angles[idx], servo_online[idx]))
        print("   ".join(cells))

    print("-" * 88)

    print(f"{Style.BRIGHT}Servo telemetry (speed/load/voltage/temp):{Style.RESET_ALL}")
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(
                    format_telemetry_cell(
                        idx,
                        servo_speed[idx],
                        servo_load[idx],
                        servo_voltage[idx],
                        servo_temperature[idx],
                        servo_telem_online[idx],
                    )
                )
        print("   ".join(cells))

    print("-" * 88)

    print(f"{Style.BRIGHT}Debug joints (packet 0x04):{Style.RESET_ALL}")
    debug_indices = [idx for idx in DEBUG_JOINTS if idx < ENCODER_COUNT]
    if not debug_indices:
        print("  (no debug joints configured)")
    else:
        label_width = 14
        col_width = 18

        def color_pad(text: str, color: str) -> str:
            padded = f"{text:<{col_width}}"
            if color:
                return f"{color}{padded}{Style.RESET_ALL}"
            return padded

        header_cells = [color_pad(f"joint {idx:02d}", Style.BRIGHT) for idx in debug_indices]
        print(f"{'':<{label_width}} " + "  ".join(header_cells))

        status_cells = []
        target_cells = []
        actual_cells = []
        loop1_cells = []
        loop2a_cells = []
        loop2o_cells = []
        latency_cells = []

        for idx in debug_indices:
            jd = joint_debug[idx]
            debug_age_ms = (time.time() - jd.last_update) * 1000.0 if jd.last_update > 0 else 9999.0
            debug_status = "VALID" if jd.valid else "INVALID"
            debug_status_color = Fore.GREEN if jd.valid else Fore.RED

            status_cells.append(color_pad(f"{debug_status:<7}", debug_status_color))
            target_cells.append(color_pad(f"{jd.target_deg:10.4f}", ""))
            actual_cells.append(color_pad(f"{jd.mag_actual_deg:10.4f}", ""))
            loop1_cells.append(color_pad(f"{jd.loop1_output:10.4f}", ""))
            loop2a_cells.append(color_pad(f"{jd.loop2_actual:10.4f}", ""))
            loop2o_cells.append(color_pad(f"{jd.loop2_output:10.4f}", ""))
            latency_cells.append(color_pad(f"{debug_age_ms:9.1f} ms", ""))

        print(f"{'status':<{label_width}} " + "  ".join(status_cells))
        print(f"{'target_deg':<{label_width}} " + "  ".join(target_cells))
        print(f"{'actual_deg':<{label_width}} " + "  ".join(actual_cells))
        print(f"{'loop1_output':<{label_width}} " + "  ".join(loop1_cells))
        print(f"{'loop2_actual':<{label_width}} " + "  ".join(loop2a_cells))
        print(f"{'loop2_output':<{label_width}} " + "  ".join(loop2o_cells))
        print(f"{'debug_latency':<{label_width}} " + "  ".join(latency_cells))

    print("-" * 88)
    print("Controls: press 'c' to send calibration command, press 'q' to quit.")

    status_line = render_calib_status(calib_status, calib_timestamp)
    print(f"Calibration: {status_line}")


def choose_serial_port() -> Optional[str]:
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Scanning serial ports...")

        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print(f"{Fore.RED}No serial port detected.{Style.RESET_ALL}")
            print("Press Enter to retry, or Ctrl+C to exit.")
            try:
                input()
            except KeyboardInterrupt:
                return None
            continue

        print("\nAvailable ports:")
        for idx, port in enumerate(ports):
            print(f"  [{idx}] {port.device} ({port.description})")

        try:
            user_input = input(
                f"\nSelect index [0-{len(ports) - 1}] or press Enter for [0]: "
            ).strip()
            if user_input == "":
                return ports[0].device

            index = int(user_input)
            if 0 <= index < len(ports):
                return ports[index].device

            print(f"{Fore.YELLOW}Invalid index.{Style.RESET_ALL}")
            time.sleep(1)
        except ValueError:
            print(f"{Fore.YELLOW}Please input a number.{Style.RESET_ALL}")
            time.sleep(1)
        except KeyboardInterrupt:
            return None


def handle_keyboard_input() -> None:
    if os.name == "nt" and msvcrt is not None and msvcrt.kbhit():
        key = msvcrt.getch().lower()
        if key == b"q":
            state.running = False
        elif key == b"c":
            with state.lock:
                ser = state.ser
            if ser and ser.is_open:
                ser.write(CMD_CALIBRATE)
                with state.lock:
                    state.calib_status = "PENDING"
                    state.calib_timestamp = time.time()


def main() -> None:
    selected_port = choose_serial_port()
    if not selected_port:
        return

    print(f"Connecting to {selected_port} ...")
    worker = threading.Thread(target=serial_thread, args=(selected_port,), daemon=True)
    worker.start()

    try:
        plotter = None
        if PLOT_ENABLED and HAS_PLOT:
            plotter = JointPlotter()

        while state.running:
            print_ui()
            if plotter:
                plotter.update()
            handle_keyboard_input()
            time.sleep(0.06)
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        with state.lock:
            if state.ser and state.ser.is_open:
                state.ser.close()
        print("\nMonitor exited.")


if __name__ == "__main__":
    main()
