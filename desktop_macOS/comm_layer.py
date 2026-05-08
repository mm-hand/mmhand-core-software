import os
import queue
import threading
import time
from typing import List, Optional, Union

import serial
import serial.tools.list_ports

from data_models import HandModel, MotorState
from protocol import (
    BAUDRATE,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PACKET_TYPE_FAULT_STATUS,
    PACKET_TYPE_JOINT_DEBUG,
    PACKET_TYPE_RELEASE_FAULT,
    PACKET_TYPE_SERVO_RAW,
    RX_POLL_SLEEP,
    RX_QUEUE_MAXSIZE,
    SERIAL_TIMEOUT,
    TX_QUEUE_GET_TIMEOUT,
    TX_QUEUE_MAXSIZE,
    build_angle_cmd,
    build_calib_data_cmd,
    build_calibrate_cmd,
    build_local_test_params_cmd,
    build_local_test_start_cmd,
    build_local_test_stop_cmd,
    build_motor_pos_cmd,
    build_motor_pos_abs_cmd,
    build_motor_pos_sweep_cmd,
    build_stream_mode_cmd,
    build_reset_cmd,
    build_start_cmd,
    build_stop_cmd,
    CMD_SENSOR_STREAM_MODE,
    parse_calib_ack,
    parse_fault_status_packet,
    parse_release_fault_packet,
    parse_frame,
    parse_joint_debug_packet,
    parse_proto_ack,
    parse_sensor_packet,
    parse_servo_angle_packet,
    parse_servo_raw_packet,
    parse_servo_telem_packet,
    PROTO_ACK_STATUS_OK,
    SENSOR_STREAM_MODE_SIGNED_I16,
)
from run_data_logger import RunDataLogger

# String command / HandModel / tuple command.
SendCmd = Union[str, HandModel, tuple]


def list_ports() -> List[tuple]:
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


def is_valid_port(port: str) -> bool:
    if not port or not port.strip():
        return False
    available = [p[0] for p in list_ports()]
    return port.strip() in available


def choose_serial_port_interactive() -> Optional[str]:
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Scanning serial ports...")
        ports = list_ports()
        if not ports:
            print("No serial port detected.")
            print("Press Enter to retry, or Ctrl+C to exit.")
            try:
                input()
            except KeyboardInterrupt:
                return None
            continue

        print("\nAvailable ports:")
        for idx, (device, desc) in enumerate(ports):
            print(f"  [{idx}] {device} ({desc})")

        try:
            user_input = input(
                f"\nSelect index [0-{len(ports) - 1}] or press Enter for [0]: "
            ).strip()
            if user_input == "":
                return ports[0][0]
            index = int(user_input)
            if 0 <= index < len(ports):
                return ports[index][0]
            print("Invalid index.")
            time.sleep(1.0)
        except ValueError:
            print("Please input a number.")
            time.sleep(1.0)
        except KeyboardInterrupt:
            return None


class LowerComputerComm:
    """Serial communication with lower controller."""

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.port = (port or "").strip()
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.rx_queue: queue.Queue = queue.Queue(maxsize=RX_QUEUE_MAXSIZE)
        self.tx_queue: queue.Queue = queue.Queue(maxsize=TX_QUEUE_MAXSIZE)
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._last_model: Optional[HandModel] = None
        self._stream_mode_requested = SENSOR_STREAM_MODE_SIGNED_I16
        self._stream_mode_applied: Optional[int] = None
        self._stream_mode_ack_pending = False
        self._stream_mode_ack_deadline = 0.0
        self._stream_mode_ack_warned = False
        self._rx_diag_last_print = time.time()
        self._rx_diag_total_packets = 0
        self._rx_diag_joint_debug_packets = 0
        self._rx_diag_sensor_packets = 0
        self._rx_diag_bytes = 0
        self._tx_diag_last_print = time.time()
        self._tx_diag_total_cmds = 0
        self._tx_diag_angle_live_cmds = 0
        self._tx_diag_bytes = 0
        self._joint_debug_packet_logger = RunDataLogger(flush_every=20)

        if self.port:
            if not is_valid_port(self.port):
                raise ValueError(f"端口 '{self.port}' 不在当前可用串口列表中")
            if not self.connect():
                raise RuntimeError(f"无法打开端口 '{self.port}'")

    def connect(self) -> bool:
        if self.serial and getattr(self.serial, "is_open", False):
            return True
        if not self.port:
            return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=SERIAL_TIMEOUT)
            self.running = True
            self._rx_buffer = bytearray()
            self._reset_rx_diag()
            self._reset_tx_diag()
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._rx_thread.start()
            self._tx_thread.start()
            self._stream_mode_applied = None
            self._stream_mode_ack_pending = True
            self._stream_mode_ack_warned = False
            self._stream_mode_ack_deadline = time.time() + 2.0
            self.send_command(("stream_mode", self._stream_mode_requested))
            return True
        except Exception as exc:
            print(f"连接失败: {exc}")
            return False

    def disconnect(self):
        self.running = False
        self.stop_joint_debug_packet_log()
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None

    def start_joint_debug_packet_log(self, path: str) -> None:
        fieldnames = [
            "host_ts_wall",
            "device_ts_ms",
            "joint_index",
            "valid",
            "target_deg",
            "actual_deg",
            "loop1_output",
            "loop2_actual",
            "loop2_output",
            "cmd_valid",
            "cmd_target_pos",
        ]
        self._joint_debug_packet_logger.start(path, fieldnames=fieldnames)

    def stop_joint_debug_packet_log(self) -> None:
        self._joint_debug_packet_logger.stop()

    def _log_joint_debug_packet(
        self,
        host_ts_wall: float,
        joint_index: int,
        valid: bool,
        target_deg: float,
        actual_deg: float,
        loop1_out: float,
        loop2_act: float,
        loop2_out: float,
        cmd_valid: bool,
        cmd_target_pos: int,
        device_timestamp_ms: int,
    ) -> None:
        if not self._joint_debug_packet_logger.is_running():
            return
        self._joint_debug_packet_logger.write_row(
            {
                "host_ts_wall": f"{float(host_ts_wall):.6f}",
                "device_ts_ms": int(device_timestamp_ms),
                "joint_index": int(joint_index),
                "valid": 1 if valid else 0,
                "target_deg": float(target_deg),
                "actual_deg": float(actual_deg),
                "loop1_output": float(loop1_out),
                "loop2_actual": float(loop2_act),
                "loop2_output": float(loop2_out),
                "cmd_valid": 1 if cmd_valid else 0,
                "cmd_target_pos": int(cmd_target_pos),
            }
        )

    def _reset_rx_diag(self) -> None:
        self._rx_diag_last_print = time.time()
        self._rx_diag_total_packets = 0
        self._rx_diag_joint_debug_packets = 0
        self._rx_diag_sensor_packets = 0
        self._rx_diag_bytes = 0

    def _reset_tx_diag(self) -> None:
        self._tx_diag_last_print = time.time()
        self._tx_diag_total_cmds = 0
        self._tx_diag_angle_live_cmds = 0
        self._tx_diag_bytes = 0

    def _update_rx_diag(self, packets: List[tuple], chunk_len: int) -> None:
        self._rx_diag_bytes += int(chunk_len)
        self._rx_diag_total_packets += len(packets)
        for pkt_type, _payload in packets:
            if pkt_type == PACKET_TYPE_JOINT_DEBUG:
                self._rx_diag_joint_debug_packets += 1
            elif pkt_type == 0x01:
                self._rx_diag_sensor_packets += 1

    def _maybe_print_rx_diag(self) -> None:
        now = time.time()
        elapsed = now - self._rx_diag_last_print
        if elapsed < 1.0:
            return
        total_rate = self._rx_diag_total_packets / elapsed
        joint_debug_rate = self._rx_diag_joint_debug_packets / elapsed
        sensor_rate = self._rx_diag_sensor_packets / elapsed
        byte_rate = self._rx_diag_bytes / elapsed
        print(
            "[RX diag] "
            f"total={total_rate:.1f} pkt/s "
            f"jointDebug={joint_debug_rate:.1f} pkt/s "
            f"sensor={sensor_rate:.1f} pkt/s "
            f"bytes={byte_rate:.0f} B/s"
        )
        self._reset_rx_diag()

    def _update_tx_diag(self, cmd: SendCmd, data_len: int) -> None:
        self._tx_diag_total_cmds += 1
        self._tx_diag_bytes += int(data_len)
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "angle_live":
            self._tx_diag_angle_live_cmds += 1

    def _maybe_print_tx_diag(self) -> None:
        now = time.time()
        elapsed = now - self._tx_diag_last_print
        if elapsed < 1.0:
            return
        total_rate = self._tx_diag_total_cmds / elapsed
        angle_live_rate = self._tx_diag_angle_live_cmds / elapsed
        byte_rate = self._tx_diag_bytes / elapsed
        print(
            "[TX diag] "
            f"total={total_rate:.1f} cmd/s "
            f"angle_live={angle_live_rate:.1f} cmd/s "
            f"bytes={byte_rate:.0f} B/s"
        )
        self._reset_tx_diag()

    def _rx_loop(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    chunk = self.serial.read(self.serial.in_waiting)
                    self._rx_buffer.extend(chunk)
                    packets, self._rx_buffer = parse_frame(bytes(self._rx_buffer))
                    self._update_rx_diag(packets, len(chunk))
                    model = self._process_packets(packets)
                    if model:
                        try:
                            self.rx_queue.put_nowait(model)
                        except queue.Full:
                            pass
                else:
                    time.sleep(RX_POLL_SLEEP)
                self._check_stream_mode_ack_timeout()
                self._maybe_print_rx_diag()
            except Exception as exc:
                print(f"RX error: {exc}")

    def _check_stream_mode_ack_timeout(self):
        if self._stream_mode_ack_pending and not self._stream_mode_ack_warned:
            if time.time() >= self._stream_mode_ack_deadline:
                print("Protocol warning: no stream-mode ACK from firmware; continue with current decoding.")
                self._stream_mode_ack_warned = True
                self._stream_mode_ack_pending = False

    def _process_packets(self, packets: List[tuple]) -> Optional[HandModel]:
        model = HandModel()
        if self._last_model:
            model.copy_state_from(self._last_model)
            model.calib_status = self._last_model.calib_status
            model.has_sensor_data = bool(self._last_model.has_sensor_data)
            model.has_servo_angle_data = bool(self._last_model.has_servo_angle_data)
            model.has_servo_raw_data = bool(self._last_model.has_servo_raw_data)

        model.timestamp = time.time()
        emitted = False

        for pkt_type, payload in packets:
            if pkt_type == 0x01:
                angles, errors = parse_sensor_packet(payload)
                if angles:
                    for i in range(min(ENCODER_COUNT, len(angles), len(model.encoders))):
                        model.encoders[i].encoder_id = i
                        model.encoders[i].raw = angles[i]
                        model.encoders[i].error = errors[i] if i < len(errors) else False

                    for i in range(min(ENCODER_COUNT, len(model.motors))):
                        model.motors[i].motor_id = i
                        model.motors[i].error = errors[i] if i < len(errors) else False
                        model.motors[i].state = (
                            MotorState.ERROR if model.motors[i].error else MotorState.RUNNING
                        )

                    model.has_sensor_data = True
                    emitted = True

            elif pkt_type == 0x02:
                code = parse_calib_ack(payload)
                if code == 1:
                    model.calib_status = "PENDING"
                elif code == 2:
                    model.calib_status = "SUCCESS"
                elif code == 3:
                    model.calib_status = "FAILED"
                emitted = True

            elif pkt_type == 0x03:
                parsed = parse_servo_angle_packet(payload)
                if parsed is not None:
                    angles, online_flags = parsed
                    for i in range(min(MOTOR_COUNT, len(angles))):
                        model.servo_angles[i] = int(angles[i])
                        model.servo_online[i] = bool(online_flags[i]) if i < len(online_flags) else False
                    model.has_servo_angle_data = True
                    emitted = True

            elif pkt_type == PACKET_TYPE_SERVO_RAW:
                parsed = parse_servo_raw_packet(payload)
                if parsed is not None:
                    raw_positions, online_flags = parsed
                    for i in range(min(MOTOR_COUNT, len(raw_positions))):
                        model.servo_raw_positions[i] = int(raw_positions[i])
                        model.servo_raw_online[i] = bool(online_flags[i]) if i < len(online_flags) else False
                    model.has_servo_raw_data = True
                    emitted = True

            elif pkt_type == 0x05:
                parsed = parse_servo_telem_packet(payload)
                if parsed is not None:
                    speeds, loads, volts, temps, online = parsed
                    for i in range(min(MOTOR_COUNT, len(speeds))):
                        model.servo_speed[i] = int(speeds[i])
                        model.servo_load[i] = int(loads[i])
                        model.servo_voltage[i] = int(volts[i])
                        model.servo_temperature[i] = int(temps[i])
                        model.servo_telem_online[i] = bool(online[i]) if i < len(online) else False
                    emitted = True

            elif pkt_type == 0x04:
                parsed = parse_joint_debug_packet(payload)
                if parsed is not None:
                    (
                        joint_index,
                        valid,
                        target_deg,
                        actual_deg,
                        loop1_out,
                        loop2_act,
                        loop2_out,
                        cmd_valid,
                        cmd_target_pos,
                        device_timestamp_ms,
                    ) = parsed
                    self._log_joint_debug_packet(
                        host_ts_wall=time.time(),
                        joint_index=joint_index,
                        valid=valid,
                        target_deg=target_deg,
                        actual_deg=actual_deg,
                        loop1_out=loop1_out,
                        loop2_act=loop2_act,
                        loop2_out=loop2_out,
                        cmd_valid=cmd_valid,
                        cmd_target_pos=cmd_target_pos,
                        device_timestamp_ms=device_timestamp_ms,
                    )
                    if 0 <= joint_index < ENCODER_COUNT:
                        model.joint_debug_valid[joint_index] = valid
                        model.joint_debug_target_deg[joint_index] = float(target_deg)
                        model.joint_debug_actual_deg[joint_index] = float(actual_deg)
                        model.joint_debug_loop1_output[joint_index] = float(loop1_out)
                        model.joint_debug_loop2_actual[joint_index] = float(loop2_act)
                        model.joint_debug_loop2_output[joint_index] = float(loop2_out)
                        model.joint_debug_cmd_valid[joint_index] = bool(cmd_valid)
                        model.joint_debug_cmd_target_pos[joint_index] = int(cmd_target_pos)
                        model.joint_debug_timestamp_ms[joint_index] = int(device_timestamp_ms)
                    emitted = True

            elif pkt_type == PACKET_TYPE_FAULT_STATUS:
                fault_bitmap = parse_fault_status_packet(payload)
                if fault_bitmap is not None:
                    for i in range(MOTOR_COUNT):
                        model.servo_overload_fault[i] = ((fault_bitmap >> i) & 0x01) != 0
                    emitted = True

            elif pkt_type == PACKET_TYPE_RELEASE_FAULT:
                release_fault_bitmap = parse_release_fault_packet(payload)
                if release_fault_bitmap is not None:
                    for i in range(ENCODER_COUNT):
                        model.joint_reverse_release_fault[i] = ((release_fault_bitmap >> i) & 0x01) != 0
                    emitted = True

            elif pkt_type == 0x06:
                parsed = parse_proto_ack(payload)
                if parsed is not None:
                    ack_cmd, applied_mode, status = parsed
                    if ack_cmd == CMD_SENSOR_STREAM_MODE:
                        self._stream_mode_applied = int(applied_mode)
                        self._stream_mode_ack_pending = False
                        if status != PROTO_ACK_STATUS_OK and not self._stream_mode_ack_warned:
                            print(
                                f"Protocol warning: stream-mode rejected "
                                f"(status={status}, applied={applied_mode})."
                            )
                            self._stream_mode_ack_warned = True

        self._last_model = model
        return model if emitted else None

    def send_command(self, cmd: SendCmd):
        if cmd == "stop":
            self._enqueue_stop_immediately()
            return
        if self._is_high_rate_command(cmd):
            self._enqueue_latest_high_rate(cmd)
            return
        try:
            self.tx_queue.put_nowait(cmd)
        except queue.Full:
            pass

    def _enqueue_stop_immediately(self) -> None:
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            pending.clear()
            pending.append("stop")
            self.tx_queue.unfinished_tasks += 1
            self.tx_queue.not_empty.notify()

    @staticmethod
    def _is_high_rate_command(cmd: SendCmd) -> bool:
        return isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] in (
            "motor_pos_sweep",
            "motor_pos_abs",
            "angle_live",
        )

    @staticmethod
    def _is_joint_command(cmd: SendCmd) -> bool:
        if isinstance(cmd, HandModel):
            return True
        return isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "angle_live"

    def invalidate_pending_joint_commands(self) -> int:
        """Drop pending joint-control commands from TX queue."""
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            kept = [item for item in pending if not self._is_joint_command(item)]
            removed = len(pending) - len(kept)
            if removed <= 0:
                return 0
            pending.clear()
            pending.extend(kept)
            self.tx_queue.not_full.notify_all()
            return removed

    def _enqueue_latest_high_rate(self, cmd: SendCmd):
        # Keep only the latest pending high-rate command to avoid UI backlog.
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            kept = [item for item in pending if not self._is_high_rate_command(item)]
            pending.clear()
            pending.extend(kept)

            maxsize = self.tx_queue.maxsize
            if maxsize > 0 and len(pending) >= maxsize:
                return

            pending.append(cmd)
            self.tx_queue.unfinished_tasks += 1
            self.tx_queue.not_empty.notify()

    def _tx_loop(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                cmd = self.tx_queue.get(timeout=TX_QUEUE_GET_TIMEOUT)
                data = self._encode_command(cmd)
                if data:
                    with self._tx_lock:
                        self.serial.write(data)
                    self._update_tx_diag(cmd, len(data))
                self._maybe_print_tx_diag()
            except queue.Empty:
                self._maybe_print_tx_diag()
            except Exception as exc:
                print(f"TX error: {exc}")

    def _encode_command(self, cmd: SendCmd) -> Optional[bytes]:
        if cmd == "calibrate":
            return build_calibrate_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "calib_data":
            return build_calib_data_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_pos":
            return build_motor_pos_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_pos_sweep":
            return build_motor_pos_sweep_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_pos_abs":
            return build_motor_pos_abs_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "angle_live":
            return build_angle_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "stream_mode":
            return build_stream_mode_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "local_test_params":
            return build_local_test_params_cmd(cmd[1], cmd[2])
        if cmd == "local_test_start":
            return build_local_test_start_cmd()
        if cmd == "local_test_stop":
            return build_local_test_stop_cmd()
        if cmd == "start":
            return build_start_cmd()
        if cmd == "stop":
            return build_stop_cmd()
        if cmd == "reset":
            return build_reset_cmd()
        if isinstance(cmd, HandModel):
            angles = cmd.target_angles
            if len(angles) != ENCODER_COUNT:
                return None
            return build_angle_cmd(list(angles))
        return None
