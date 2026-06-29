import os
import queue
import threading
import time
from typing import Dict, List, Optional, Union

import numpy as np
import serial
import serial.tools.list_ports

from data_models import FingerTactile, HandModel, MotorState, TactileSensor, tactile_segment_shape
from protocol import (
    BAUDRATE,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PACKET_TYPE_CALIB_DIAG,
    PACKET_TYPE_FAULT_STATUS,
    PACKET_TYPE_JOINT_DEBUG,
    PACKET_TYPE_RELEASE_FAULT,
    PACKET_TYPE_ENCODER_RAW,
    PACKET_TYPE_JOINT_CALIB_SNAPSHOT,
    PACKET_TYPE_MOTOR_ABS_LIMIT,
    PACKET_TYPE_SERVO_RAW,
    PACKET_TYPE_SYSTEM_STATUS,
    PACKET_TYPE_TACTILE,
    PACKET_TYPE_TACTILE_FULL,
    PROTOCOL_HEADER,
    PROTOCOL_TAIL,
    RX_POLL_SLEEP,
    RX_QUEUE_MAXSIZE,
    SERIAL_TIMEOUT,
    TX_QUEUE_GET_TIMEOUT,
    TX_QUEUE_MAXSIZE,
    build_angle_cmd,
    build_active_scope_cmd,
    build_calib_data_cmd,
    build_calib_load_joint_cmd,
    build_calib_load_limits_cmd,
    build_calibrate_cmd,
    build_clear_calibration_cmd,
    build_local_test_params_cmd,
    build_local_test_start_cmd,
    build_local_test_stop_cmd,
    build_host_heartbeat_cmd,
    build_motor_abs_limit_lock_cmd,
    build_motor_pos_cmd,
    build_motor_pos_abs_cmd,
    build_motor_pos_abs_one_cmd,
    build_motor_pos_sweep_cmd,
    build_multiturn_nvs_control_cmd,
    build_pause_cmd,
    build_restore_multiturn_cmd,
    build_run_feedback_cmd,
    build_side_sway_adjust_cmd,
    build_stream_mode_cmd,
    build_reset_cmd,
    build_start_cmd,
    build_stop_cmd,
    build_tendon_guard_cmd,
    CMD_CALIBRATE,
    CMD_CLEAR_CALIBRATION,
    CMD_CALIB_LOAD_JOINT,
    CMD_CALIB_LOAD_LIMITS,
    CMD_ACTIVE_SCOPE,
    CMD_CALIB_DATA,
    CMD_START,
    CMD_STOP,
    CMD_RESET,
    CMD_RUN_FEEDBACK,
    CMD_SIDE_SWAY_ADJUST,
    CMD_PAUSE,
    CMD_RESTORE_MULTITURN,
    CMD_MOTOR_ABS_LIMIT_LOCK,
    CMD_MULTITURN_NVS_CONTROL,
    CMD_SENSOR_STREAM_MODE,
    SYSTEM_STATE_UNCONTROLLABLE,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_PAUSED,
    SYSTEM_FAULT_CALIBRATION,
    parse_calib_diag_packet,
    parse_calib_ack,
    parse_fault_status_packet,
    parse_release_fault_packet,
    parse_joint_debug_packet,
    parse_joint_calib_snapshot_packet,
    parse_motor_abs_limit_packet,
    parse_proto_ack,
    parse_sensor_packet,
    parse_servo_angle_packet,
    parse_servo_raw_packet,
    parse_servo_telem_packet,
    parse_system_status_packet,
    parse_tactile_full_fragment,
    parse_tactile_packet,
    TACTILE_GROUP_COUNT,
    TACTILE_FULL_BYTES,
    TACTILE_SENSOR_PER_GROUP,
    TACTILE_SENSOR_TAXEL_COUNTS,
    crc16_ccitt,
    PROTO_ACK_STATUS_OK,
    PROTO_ACK_STATUS_QUEUED,
    SENSOR_STREAM_MODE_SIGNED_I16,
)
from run_data_logger import RunDataLogger
from session_log import log_session

# String command / HandModel / tuple command.
SendCmd = Union[str, HandModel, tuple]
# 固件连续丢失两次 0.5s heartbeat 后，会认为上位机离线并保存 NVS。
HOST_HEARTBEAT_INTERVAL_S = 0.5
TACTILE_ZERO_NOISE_FLOOR = 3
TACTILE_ZERO_HOLD_FRAMES = 6
TACTILE_HOLD_MIN_MAGNITUDE = 12
TACTILE_FULL_STALE_AFTER_S = 1.0
TACTILE_SERIAL_OFFLINE_AFTER_S = 0.6
TACTILE_MODULE_OFFLINE_AFTER_S = 0.6
TERMINAL_LOG_LEVEL_ERROR = 1
TERMINAL_LOG_LEVEL_WARN = 2
TERMINAL_LOG_LEVEL_INFO = 3
TERMINAL_LOG_LEVEL_DEBUG = 4
TERMINAL_LOG_LEVEL_TRACE = 5
TERMINAL_DIAG_ENABLE = False
TERMINAL_DIAG_INTERVAL_S = 20.0
SESSION_DIAG_INTERVAL_S = 20.0
FIRMWARE_LOG_TO_TERMINAL = False
FIRMWARE_LOG_TERMINAL_LEVEL = TERMINAL_LOG_LEVEL_WARN
FIRMWARE_LOG_FILTER_PREFIXES = ("[RX diag]", "[TX diag]")
FIRMWARE_LOG_TRACE_MARKERS = (
    " TRACE",
    "_TRACE",
    " trace ",
    "_WAIT",
    " CHECK_",
    "RawZ",
)
FIRMWARE_LOG_DEBUG_MARKERS = (
    " DBG",
    "[CAN RX DBG]",
    "[S3 CAN DBG]",
    "[Tactile]",
)
FIRMWARE_LOG_WARN_MARKERS = (
    "WARN",
    "WARNING",
    "OFFLINE",
    "HOTPLUG",
    "reject",
    "rejected",
    "timeout",
)
FIRMWARE_LOG_ERROR_MARKERS = (
    "FAIL",
    "FAILED",
    "FATAL",
    "ERROR",
    "failed",
    "error",
)

# S3 tactile raw group order follows the physical SPI order:
# G0=thumb(SPI1), G1=index(SPI2), G2=middle(SPI3), G3=ring(SPI4), G4=pinky(SPI5).
# The UI/model order is also thumb, index, middle, ring, pinky.
TACTILE_UI_GROUP_FROM_RAW = (0, 1, 2, 3, 4)
TACTILE_SERIAL_FULL_PACKET_TYPES = (0x0E, PACKET_TYPE_TACTILE_FULL)


def _synthetic_contact_grid(rows: int, cols: int, fx: int, fy: int, fz: int) -> np.ndarray:
    """Project one resultant force sample into a tactile-looking taxel field."""
    y, x = np.mgrid[0:rows, 0:cols]
    cx = (cols - 1) / 2.0
    cy = (rows - 1) / 2.0

    pressure = float(max(0, fz))
    if pressure <= 3.0:
        pressure = 0.0

    if pressure > 0.0:
        direction_scale = max(24.0, min(96.0, pressure))
        cx += max(-0.48, min(0.48, float(fx) / direction_scale)) * max(1.0, (cols - 1) / 2.0)
        cy += max(-0.48, min(0.48, float(fy) / direction_scale)) * max(1.0, (rows - 1) / 2.0)

    normalized = min(1.0, pressure / 180.0)
    sigma_x = max(0.62, cols * (0.13 + 0.11 * normalized))
    sigma_y = max(0.62, rows * (0.15 + 0.13 * normalized))
    core = np.exp(-(((x - cx) ** 2) / (2.0 * sigma_x * sigma_x) + ((y - cy) ** 2) / (2.0 * sigma_y * sigma_y)))
    skirt = np.exp(-(((x - cx) ** 2) / (2.0 * (sigma_x * 2.2) ** 2) + ((y - cy) ** 2) / (2.0 * (sigma_y * 2.2) ** 2)))
    weight = core + 0.18 * normalized * skirt
    max_weight = float(np.max(weight))
    if max_weight > 1e-9:
        weight = weight / max_weight

    contact = np.zeros((rows, cols, 3), dtype=float)
    contact[:, :, 0] = float(fx) * weight
    contact[:, :, 1] = float(fy) * weight
    contact[:, :, 2] = pressure * weight
    return contact


def _build_tactile_fingers(groups: List[List[List[int]]]) -> List[FingerTactile]:
    raw_fingers: List[FingerTactile] = []
    for group in groups:
        sensors: List[TactileSensor] = []
        for sensor_idx in range(3):
            if sensor_idx < len(group):
                fx, fy, fz = group[sensor_idx]
            else:
                fx, fy, fz = 0, 0, 0

            if sensor_idx == 0:
                rows, cols = tactile_segment_shape(0)
            else:
                rows, cols = tactile_segment_shape(2)

            contact = _synthetic_contact_grid(rows, cols, int(fx), int(fy), int(fz))
            resultant = np.array([float(fx), float(fy), float(fz)], dtype=float)
            sensors.append(TactileSensor(contact_forces=contact, resultant=resultant))
        raw_fingers.append(FingerTactile(sensors=sensors))
    return [
        raw_fingers[idx] if idx < len(raw_fingers) else FingerTactile(sensors=[])
        for idx in TACTILE_UI_GROUP_FROM_RAW
    ]


def _u8_to_i8(value: int) -> int:
    value = int(value) & 0xFF
    return value - 256 if value >= 128 else value


def _summary_groups_all_zero(summary_groups: List[List[List[int]]]) -> bool:
    for group in summary_groups:
        for force in group:
            if len(force) >= 3 and (int(force[0]) != 0 or int(force[1]) != 0 or int(force[2]) != 0):
                return False
    return True


def _build_tactile_fingers_from_full(
    raw: bytes,
    summary_groups: Optional[List[List[List[int]]]] = None,
) -> List[FingerTactile]:
    raw_fingers: List[FingerTactile] = []
    cursor = 0
    for group_idx in range(5):
        sensors: List[TactileSensor] = []
        for sensor_idx, taxel_count in enumerate(TACTILE_SENSOR_TAXEL_COUNTS):
            rows, cols = tactile_segment_shape(0 if sensor_idx == 0 else 2)
            raw_contact = np.zeros((rows, cols, 3), dtype=float)
            raw_unsigned_contact = np.zeros((rows, cols, 3), dtype=float)
            flat = raw_contact.reshape((-1, 3))
            flat_unsigned = raw_unsigned_contact.reshape((-1, 3))
            for taxel_idx in range(min(taxel_count, flat.shape[0])):
                if cursor + 2 >= len(raw):
                    break
                flat_unsigned[taxel_idx, 0] = float(raw[cursor])
                flat_unsigned[taxel_idx, 1] = float(raw[cursor + 1])
                flat_unsigned[taxel_idx, 2] = float(raw[cursor + 2])
                flat[taxel_idx, 0] = float(_u8_to_i8(raw[cursor]))
                flat[taxel_idx, 1] = float(_u8_to_i8(raw[cursor + 1]))
                flat[taxel_idx, 2] = float(raw[cursor + 2])
                cursor += 3
            contact = raw_contact.copy()

            if summary_groups and group_idx < len(summary_groups) and sensor_idx < len(summary_groups[group_idx]):
                fx, fy, fz = summary_groups[group_idx][sensor_idx]
                resultant = np.array([float(fx), float(fy), float(fz)], dtype=float)
            else:
                resultant = np.array(
                    [
                        float(np.sum(contact[:, :, 0])),
                        float(np.sum(contact[:, :, 1])),
                        float(np.sum(contact[:, :, 2])),
                    ],
                    dtype=float,
                )
            sensors.append(TactileSensor(
                contact_forces=contact,
                resultant=resultant,
                raw_contact_forces=raw_contact,
                raw_unsigned_contact_forces=raw_unsigned_contact,
            ))
        raw_fingers.append(FingerTactile(sensors=sensors))
    return [
        raw_fingers[idx] if idx < len(raw_fingers) else FingerTactile(sensors=[])
        for idx in TACTILE_UI_GROUP_FROM_RAW
    ]


def _tactile_axis_magnitude(values: List[int]) -> int:
    if not values or len(values) < 3:
        return 0
    return max(abs(int(values[0])), abs(int(values[1])), max(0, int(values[2])))


def list_ports() -> List[tuple]:
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


def normalize_serial_port_name(port: str) -> str:
    value = (port or "").strip()
    if value.startswith("\\\\.\\"):
        return value[4:]
    return value


def is_valid_port(port: str) -> bool:
    if not port or not port.strip():
        return False
    available = [p[0] for p in list_ports()]
    return normalize_serial_port_name(port) in available or port.strip() in available


class TactileSerialComm:
    """Read-only S3 tactile serial stream using the desktop binary protocol."""

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.port = (port or "").strip()
        self.baudrate = int(baudrate)
        self.serial: Optional[serial.Serial] = None
        self.rx_queue: queue.Queue = queue.Queue(maxsize=20)
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_buffer = bytearray()
        self._firmware_text_buffer = bytearray()
        self._last_model: Optional[HandModel] = None
        self._last_tactile_summary_groups: Optional[List[List[List[int]]]] = None
        self._last_tactile_summary_seq: Optional[int] = None
        self._tactile_full_seq: Optional[int] = None
        self._tactile_full_count = 0
        self._tactile_full_fragments: Dict[int, bytes] = {}
        self._tactile_full_summary_seq: Optional[int] = None
        self._tactile_full_crc16: Optional[int] = None
        self._tactile_full_started_ts = 0.0
        self._tactile_full_crc_errors = 0
        self._tactile_full_seq_mismatches = 0
        self._tactile_full_dropped = 0
        self._last_tactile_full_ts = 0.0
        self._last_tactile_rx_ts = 0.0
        self._tactile_module_last_rx_ts = [0.0] * (TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)
        self._tactile_offline_emitted = False
        self._rx_diag_last_print = time.time()
        self._rx_diag_total_packets = 0
        self._rx_diag_bytes = 0

    def connect(self) -> bool:
        if self.serial and getattr(self.serial, "is_open", False):
            return True
        if not self.port:
            return False
        try:
            if not is_valid_port(self.port):
                raise ValueError(f"tactile port '{self.port}' is not available")
            self.serial = serial.Serial()
            self.serial.port = self.port
            self.serial.baudrate = self.baudrate
            self.serial.timeout = SERIAL_TIMEOUT
            self.serial.dtr = False
            self.serial.rts = False
            self.serial.open()
            self.serial.setDTR(False)
            self.serial.setRTS(False)
            self.running = True
            self._rx_buffer = bytearray()
            self._firmware_text_buffer = bytearray()
            self._last_model = None
            self._clear_queue(self.rx_queue)
            self._reset_tactile_full_reassembly()
            self._last_tactile_summary_groups = None
            self._last_tactile_summary_seq = None
            self._last_tactile_full_ts = 0.0
            self._last_tactile_rx_ts = 0.0
            self._tactile_module_last_rx_ts = [0.0] * (TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)
            self._tactile_offline_emitted = False
            self._rx_diag_last_print = time.time()
            self._rx_diag_total_packets = 0
            self._rx_diag_bytes = 0
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            return True
        except Exception as exc:
            print(f"Tactile serial connect failed: {exc}")
            return False

    def disconnect(self) -> None:
        self.running = False
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
        thread = self._rx_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._rx_thread = None

    def get_latest_model(self) -> Optional[HandModel]:
        latest = None
        while True:
            try:
                latest = self.rx_queue.get_nowait()
                self.rx_queue.task_done()
            except queue.Empty:
                break
        stale = self._offline_model_if_stale()
        if stale is not None:
            latest = stale
        return latest

    def is_connected(self) -> bool:
        return bool(self.running and self.serial and getattr(self.serial, "is_open", False))

    def _mark_serial_offline(self) -> None:
        self.running = False
        try:
            if self.serial and getattr(self.serial, "is_open", False):
                self.serial.close()
        except Exception:
            pass

    def _offline_model_if_stale(self) -> Optional[HandModel]:
        if self._last_tactile_rx_ts <= 0.0:
            return None
        if self._tactile_offline_emitted:
            return None
        if (time.time() - self._last_tactile_rx_ts) <= TACTILE_SERIAL_OFFLINE_AFTER_S:
            return None
        model = HandModel()
        if self._last_model is not None:
            model.copy_state_from(self._last_model)
        model.tactile_data = []
        model.tactile_online = False
        model.tactile_last_rx_ts = self._last_tactile_rx_ts
        model.tactile_module_online = [False] * (TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)
        model.tactile_module_last_rx_ts = list(self._tactile_module_last_rx_ts)
        model.timestamp = time.time()
        self._last_model = model
        self._tactile_offline_emitted = True
        return model

    @staticmethod
    def _summary_sensor_has_tactile_data(values: Optional[List[int]]) -> bool:
        if not values:
            return False
        padded = list(values[:3])
        while len(padded) < 3:
            padded.append(0)
        return (
            abs(int(padded[0])) > TACTILE_ZERO_NOISE_FLOOR or
            abs(int(padded[1])) > TACTILE_ZERO_NOISE_FLOOR or
            int(padded[2]) > TACTILE_ZERO_NOISE_FLOOR
        )

    @staticmethod
    def _full_sensor_has_tactile_data(raw: bytes, cursor: int, taxel_count: int) -> bool:
        end = min(len(raw), int(cursor) + int(taxel_count) * 3)
        idx = int(cursor)
        while idx + 2 < end:
            x = _u8_to_i8(raw[idx])
            y = _u8_to_i8(raw[idx + 1])
            z = int(raw[idx + 2])
            if (
                abs(int(x)) > TACTILE_ZERO_NOISE_FLOOR or
                abs(int(y)) > TACTILE_ZERO_NOISE_FLOOR or
                z > TACTILE_ZERO_NOISE_FLOOR
            ):
                return True
            idx += 3
        return False

    def _mark_tactile_modules_seen(
        self,
        groups: Optional[List[List[List[int]]]] = None,
        full_raw: Optional[bytes] = None,
    ) -> None:
        now = time.time()
        self._last_tactile_rx_ts = now
        self._tactile_offline_emitted = False
        full_active = [[False] * TACTILE_SENSOR_PER_GROUP for _ in range(TACTILE_GROUP_COUNT)]
        if full_raw is not None:
            cursor = 0
            raw_bytes = bytes(full_raw)
            for group_idx in range(TACTILE_GROUP_COUNT):
                for sensor_idx in range(TACTILE_SENSOR_PER_GROUP):
                    taxel_count = (
                        TACTILE_SENSOR_TAXEL_COUNTS[sensor_idx]
                        if sensor_idx < len(TACTILE_SENSOR_TAXEL_COUNTS)
                        else 0
                    )
                    full_active[group_idx][sensor_idx] = self._full_sensor_has_tactile_data(
                        raw_bytes,
                        cursor,
                        int(taxel_count),
                    )
                    cursor += int(taxel_count) * 3
        for group_idx in range(TACTILE_GROUP_COUNT):
            src_group = groups[group_idx] if groups and group_idx < len(groups) else None
            for sensor_idx in range(TACTILE_SENSOR_PER_GROUP):
                if src_group is not None and sensor_idx >= len(src_group):
                    continue
                summary_active = (
                    src_group is not None and
                    sensor_idx < len(src_group) and
                    self._summary_sensor_has_tactile_data(src_group[sensor_idx])
                )
                if not summary_active and not full_active[group_idx][sensor_idx]:
                    continue
                self._tactile_module_last_rx_ts[
                    group_idx * TACTILE_SENSOR_PER_GROUP + sensor_idx
                ] = now

    def _apply_tactile_status(self, model: HandModel) -> None:
        now = time.time()
        online = self._last_tactile_rx_ts > 0.0 and (
            now - self._last_tactile_rx_ts
        ) <= TACTILE_SERIAL_OFFLINE_AFTER_S
        model.tactile_online = bool(online)
        model.tactile_last_rx_ts = float(self._last_tactile_rx_ts)
        module_online: List[bool] = []
        for ts in self._tactile_module_last_rx_ts:
            module_online.append(bool(online and ts > 0.0 and (now - ts) <= TACTILE_MODULE_OFFLINE_AFTER_S))
        model.tactile_module_online = module_online
        model.tactile_module_last_rx_ts = list(self._tactile_module_last_rx_ts)

    @staticmethod
    def _clear_queue(q: queue.Queue) -> None:
        with q.mutex:
            q.queue.clear()
            q.unfinished_tasks = 0
            q.not_full.notify_all()

    def _put_latest_model(self, model: HandModel) -> None:
        try:
            self.rx_queue.put_nowait(model)
            return
        except queue.Full:
            pass
        try:
            self.rx_queue.get_nowait()
            self.rx_queue.task_done()
        except queue.Empty:
            pass
        try:
            self.rx_queue.put_nowait(model)
        except queue.Full:
            pass

    def _reset_tactile_full_reassembly(self) -> None:
        self._tactile_full_seq = None
        self._tactile_full_count = 0
        self._tactile_full_fragments = {}
        self._tactile_full_summary_seq = None
        self._tactile_full_crc16 = None
        self._tactile_full_started_ts = 0.0

    def _accept_tactile_full_fragment(
        self,
        seq: int,
        frag_idx: int,
        frag_count: int,
        chunk: bytes,
        summary_seq: Optional[int] = None,
        full_crc: Optional[int] = None,
    ) -> Optional[tuple]:
        now = time.time()
        if self._tactile_full_started_ts and (now - self._tactile_full_started_ts) > 0.5:
            self._tactile_full_dropped += 1
            self._reset_tactile_full_reassembly()

        if frag_count <= 0 or frag_count > 64 or frag_idx < 0 or frag_idx >= frag_count:
            self._tactile_full_dropped += 1
            return None

        if (
            self._tactile_full_seq != seq or
            self._tactile_full_count != frag_count or
            self._tactile_full_summary_seq != summary_seq or
            self._tactile_full_crc16 != full_crc or
            (frag_idx == 0 and self._tactile_full_fragments)
        ):
            self._tactile_full_seq = seq
            self._tactile_full_count = frag_count
            self._tactile_full_fragments = {}
            self._tactile_full_summary_seq = summary_seq
            self._tactile_full_crc16 = full_crc
            self._tactile_full_started_ts = now

        chunk_bytes = bytes(chunk)
        existing = self._tactile_full_fragments.get(int(frag_idx))
        if existing is not None and existing != chunk_bytes:
            self._tactile_full_dropped += 1
            self._reset_tactile_full_reassembly()
            return None

        self._tactile_full_fragments[int(frag_idx)] = chunk_bytes
        if len(self._tactile_full_fragments) < frag_count:
            return None
        if any(idx not in self._tactile_full_fragments for idx in range(frag_count)):
            return None

        raw = b"".join(self._tactile_full_fragments[idx] for idx in range(frag_count))
        accepted_summary_seq = self._tactile_full_summary_seq
        accepted_crc = self._tactile_full_crc16
        self._reset_tactile_full_reassembly()
        if len(raw) != TACTILE_FULL_BYTES:
            self._tactile_full_dropped += 1
            return None
        if accepted_crc is not None and crc16_ccitt(raw) != accepted_crc:
            self._tactile_full_crc_errors += 1
            return None
        return raw, accepted_summary_seq, accepted_crc

    def _rx_loop(self) -> None:
        while self.running and self.serial and self.serial.is_open:
            try:
                waiting = self.serial.in_waiting
                if waiting:
                    chunk = self.serial.read(waiting)
                    self._rx_buffer.extend(chunk)
                    packets = self._extract_packets_and_firmware_logs()
                    self._rx_diag_total_packets += len(packets)
                    self._rx_diag_bytes += len(chunk)
                    model = self._process_tactile_packets(packets)
                    if model:
                        self._put_latest_model(model)
                    self._maybe_print_rx_diag()
                else:
                    time.sleep(RX_POLL_SLEEP)
            except Exception as exc:
                if self.running:
                    print(f"Tactile RX error: {exc}")
                self._mark_serial_offline()

    def _append_firmware_log_byte(self, value: int) -> None:
        if value in (9, 10, 13) or 32 <= value <= 126:
            self._firmware_text_buffer.append(value)
            if value in (10, 13):
                self._flush_firmware_log_text()
        elif self._firmware_text_buffer and self._firmware_text_buffer[-1] not in (10, 13, 32):
            self._firmware_text_buffer.append(32)

    def _flush_firmware_log_text(self) -> None:
        self._firmware_text_buffer.clear()

    def _extract_packets_and_firmware_logs(self) -> List[tuple]:
        packets: List[tuple] = []
        buffer = self._rx_buffer
        while buffer:
            if buffer[0] != PROTOCOL_HEADER:
                self._append_firmware_log_byte(buffer.pop(0))
                continue

            if len(buffer) < 4:
                break

            frame_len = 2 + buffer[1]
            if frame_len < 4:
                self._append_firmware_log_byte(buffer.pop(0))
                continue

            if len(buffer) < frame_len:
                break

            if buffer[frame_len - 1] == PROTOCOL_TAIL:
                self._flush_firmware_log_text()
                pkt_type = buffer[2]
                payload = bytes(buffer[3:frame_len - 1])
                packets.append((pkt_type, payload))
                del buffer[:frame_len]
            else:
                self._append_firmware_log_byte(buffer.pop(0))
        return packets

    @staticmethod
    def _stabilize_tactile_groups(groups: List[List[List[int]]]) -> List[List[List[int]]]:
        stable: List[List[List[int]]] = []
        for gi in range(5):
            group_out: List[List[int]] = []
            src_group = groups[gi] if gi < len(groups) else []
            for si in range(3):
                incoming = list(src_group[si]) if si < len(src_group) else [0, 0, 0]
                while len(incoming) < 3:
                    incoming.append(0)
                group_out.append([int(incoming[0]), int(incoming[1]), int(incoming[2])])
            stable.append(group_out)
        return stable

    def _process_tactile_packets(self, packets: List[tuple]) -> Optional[HandModel]:
        if not packets:
            return None
        model = HandModel()
        if self._last_model is not None:
            model.copy_state_from(self._last_model)
        model.timestamp = time.time()
        emitted = False

        for pkt_type, payload in packets:
            if pkt_type == PACKET_TYPE_TACTILE:
                parsed = parse_tactile_packet(payload)
                if parsed is None:
                    continue
                seq, groups = parsed
                stable_groups = self._stabilize_tactile_groups(groups)
                self._mark_tactile_modules_seen(stable_groups)
                self._last_tactile_summary_groups = stable_groups
                self._last_tactile_summary_seq = int(seq)
                model.tactile_summary_seq = int(seq)
                model.tactile_full_crc_errors = self._tactile_full_crc_errors
                model.tactile_full_seq_mismatches = self._tactile_full_seq_mismatches
                model.tactile_full_dropped = self._tactile_full_dropped
                if _summary_groups_all_zero(stable_groups):
                    self._last_tactile_full_ts = 0.0
                model.tactile_data = _build_tactile_fingers(stable_groups)
                emitted = True

            elif pkt_type in TACTILE_SERIAL_FULL_PACKET_TYPES:
                parsed = parse_tactile_full_fragment(payload)
                if parsed is None:
                    continue
                seq, frag_idx, frag_count, chunk, summary_seq, full_crc = parsed
                accepted = self._accept_tactile_full_fragment(
                    seq,
                    frag_idx,
                    frag_count,
                    chunk,
                    summary_seq,
                    full_crc,
                )
                if accepted is None:
                    continue
                raw, accepted_summary_seq, accepted_crc = accepted
                if (
                    accepted_summary_seq is not None and
                    self._last_tactile_summary_seq != accepted_summary_seq
                ):
                    self._tactile_full_seq_mismatches += 1
                    model.tactile_full_seq_mismatches = self._tactile_full_seq_mismatches
                    emitted = True
                    continue
                model.tactile_data = _build_tactile_fingers_from_full(
                    raw,
                    self._last_tactile_summary_groups,
                )
                self._mark_tactile_modules_seen(self._last_tactile_summary_groups, raw)
                model.tactile_full_seq = int(seq)
                model.tactile_full_summary_seq = (
                    int(accepted_summary_seq)
                    if accepted_summary_seq is not None
                    else None
                )
                model.tactile_full_crc = (
                    int(accepted_crc)
                    if accepted_crc is not None
                    else None
                )
                model.tactile_full_crc_errors = self._tactile_full_crc_errors
                model.tactile_full_seq_mismatches = self._tactile_full_seq_mismatches
                model.tactile_full_dropped = self._tactile_full_dropped
                self._last_tactile_full_ts = time.time()
                emitted = True

        if emitted:
            self._apply_tactile_status(model)
            self._last_model = model
            return model
        return None

    def _maybe_print_rx_diag(self) -> None:
        now = time.time()
        elapsed = now - self._rx_diag_last_print
        if elapsed < TERMINAL_DIAG_INTERVAL_S:
            return
        packet_rate = self._rx_diag_total_packets / elapsed
        byte_rate = self._rx_diag_bytes / elapsed
        log_session(
            "Tactile RX diag",
            f"packets={packet_rate:.1f} pkt/s bytes={byte_rate:.0f} B/s "
            f"crc_err={self._tactile_full_crc_errors} "
            f"seq_mismatch={self._tactile_full_seq_mismatches} "
            f"dropped={self._tactile_full_dropped}",
        )
        self._rx_diag_last_print = now
        self._rx_diag_total_packets = 0
        self._rx_diag_bytes = 0



def choose_serial_port_interactive() -> Optional[str]:
    while True:
        os.system("cls")
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
    """
    ===========================================================================
    LowerComputerComm - 底层通信与协议解析引擎
    ===========================================================================
    定位：
    这是上位机最底层的硬件交互层。所有的业务逻辑最终都会转换为指令经过本类发往串口；
    下位机所有的状态回传也都会先经过本类的解析，再向上传递。

    核心机制：
    1. 异步双线程架构：
       - TX 线程 (_tx_loop)：阻塞等待 `tx_queue`，一旦有指令对象，就通过 `_encode_command`
         封包为大端序二进制流，并调用 `serial.write` 下发。
       - RX 线程 (_rx_loop)：轮询串口读取字节流，处理“粘包”和串口断帧问题。提取出
         合法的包后，交给 `_process_packets` 根据包头 ID (pkt_type) 进行 `struct.unpack`
         解包。
    2. 高频命令截流 (High-rate throttling)：
       - 对于滑动条拖动等产生的海量实时命令（如 `angle_live`, `motor_pos_abs`），
         通过 `_enqueue_latest_high_rate` 方法清除队列中积压的旧历史指令，仅保留最新一帧。
         这极大地防止了 UI 事件堆积导致的手臂运动延迟（Latency）。
    ===========================================================================
    """

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
        self._ack_lock = threading.Condition()
        self._rx_buffer = bytearray()
        self._firmware_text_buffer = bytearray()
        self._last_model: Optional[HandModel] = None
        self._last_proto_ack: Optional[tuple] = None
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
        self._last_heartbeat_tx = 0.0
        self._heartbeat_enabled = False
        self._joint_debug_packet_logger = RunDataLogger(flush_every=20)
        self._tactile_hold_groups: Optional[List[List[List[int]]]] = None
        self._tactile_zero_counts: List[List[int]] = [
            [0 for _ in range(3)] for _ in range(5)
        ]
        self._last_tactile_summary_groups: Optional[List[List[List[int]]]] = None
        self._last_tactile_summary_seq: Optional[int] = None
        self._tactile_full_seq: Optional[int] = None
        self._tactile_full_count = 0
        self._tactile_full_fragments: Dict[int, bytes] = {}
        self._tactile_full_summary_seq: Optional[int] = None
        self._tactile_full_crc16: Optional[int] = None
        self._tactile_full_started_ts = 0.0
        self._tactile_full_crc_errors = 0
        self._tactile_full_seq_mismatches = 0
        self._tactile_full_dropped = 0
        self._last_tactile_full_ts = 0.0

        if self.port:
            if not is_valid_port(self.port):
                raise ValueError(f"serial port '{self.port}' is not available")
            if not self.connect():
                raise RuntimeError(f"无法打开端口 '{self.port}'")

    def connect(self) -> bool:
        if self.serial and getattr(self.serial, "is_open", False):
            log_session("COMM", f"serial connected port={self.port} baud={self.baudrate}")
            return True
        if not self.port:
            return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=SERIAL_TIMEOUT)
            self.running = True
            self._rx_buffer = bytearray()
            self._firmware_text_buffer = bytearray()
            self._last_model = None
            self._last_proto_ack = None
            self._clear_queue(self.rx_queue)
            self._clear_queue(self.tx_queue)
            self._reset_rx_diag()
            self._reset_tx_diag()
            self._last_heartbeat_tx = 0.0
            self._heartbeat_enabled = False
            self._tactile_hold_groups = None
            self._tactile_zero_counts = [[0 for _ in range(3)] for _ in range(5)]
            self._last_tactile_summary_groups = None
            self._last_tactile_summary_seq = None
            self._last_tactile_full_ts = 0.0
            self._reset_tactile_full_reassembly()
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._rx_thread.start()
            self._tx_thread.start()
            self._request_stream_mode_negotiation()
            return True
        except Exception as exc:
            log_session("COMM", f"serial connect failed port={self.port}: {exc}")
            print(f"连接失败: {exc}")
            return False

    @staticmethod
    def _clear_queue(q: queue.Queue) -> None:
        with q.mutex:
            q.queue.clear()
            q.unfinished_tasks = 0
            q.not_full.notify_all()

    def disconnect(self):
        self.running = False
        self.stop_joint_debug_packet_log()
        log_session("COMM", "serial disconnect requested")
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None
        rx_thread = self._rx_thread
        tx_thread = self._tx_thread
        if rx_thread and rx_thread.is_alive():
            rx_thread.join(timeout=1.0)
        if tx_thread and tx_thread.is_alive():
            tx_thread.join(timeout=1.0)
        self._rx_thread = None
        self._tx_thread = None
        log_session("COMM", "serial disconnected")

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

    def _reset_tactile_full_reassembly(self) -> None:
        self._tactile_full_seq = None
        self._tactile_full_count = 0
        self._tactile_full_fragments = {}
        self._tactile_full_summary_seq = None
        self._tactile_full_crc16 = None
        self._tactile_full_started_ts = 0.0

    def _accept_tactile_full_fragment(
        self,
        seq: int,
        frag_idx: int,
        frag_count: int,
        chunk: bytes,
        summary_seq: Optional[int] = None,
        full_crc: Optional[int] = None,
    ) -> Optional[tuple]:
        now = time.time()
        if self._tactile_full_started_ts and (now - self._tactile_full_started_ts) > 0.5:
            self._tactile_full_dropped += 1
            self._reset_tactile_full_reassembly()

        if frag_count <= 0 or frag_count > 64 or frag_idx < 0 or frag_idx >= frag_count:
            self._tactile_full_dropped += 1
            return None

        if (
            self._tactile_full_seq != seq or
            self._tactile_full_count != frag_count or
            self._tactile_full_summary_seq != summary_seq or
            self._tactile_full_crc16 != full_crc or
            (frag_idx == 0 and self._tactile_full_fragments)
        ):
            self._tactile_full_seq = seq
            self._tactile_full_count = frag_count
            self._tactile_full_fragments = {}
            self._tactile_full_summary_seq = summary_seq
            self._tactile_full_crc16 = full_crc
            self._tactile_full_started_ts = now

        chunk_bytes = bytes(chunk)
        existing = self._tactile_full_fragments.get(int(frag_idx))
        if existing is not None and existing != chunk_bytes:
            self._tactile_full_dropped += 1
            self._reset_tactile_full_reassembly()
            return None

        self._tactile_full_fragments[int(frag_idx)] = chunk_bytes
        if len(self._tactile_full_fragments) < frag_count:
            return None
        if any(idx not in self._tactile_full_fragments for idx in range(frag_count)):
            return None

        raw = b"".join(self._tactile_full_fragments[idx] for idx in range(frag_count))
        accepted_summary_seq = self._tactile_full_summary_seq
        accepted_crc = self._tactile_full_crc16
        self._reset_tactile_full_reassembly()
        if len(raw) != TACTILE_FULL_BYTES:
            self._tactile_full_dropped += 1
            return None
        if accepted_crc is not None and crc16_ccitt(raw) != accepted_crc:
            self._tactile_full_crc_errors += 1
            return None
        return raw, accepted_summary_seq, accepted_crc

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
        if elapsed < TERMINAL_DIAG_INTERVAL_S:
            return
        total_rate = self._rx_diag_total_packets / elapsed
        joint_debug_rate = self._rx_diag_joint_debug_packets / elapsed
        sensor_rate = self._rx_diag_sensor_packets / elapsed
        byte_rate = self._rx_diag_bytes / elapsed
        diag_msg = (
            f"total={total_rate:.1f} pkt/s "
            f"jointDebug={joint_debug_rate:.1f} pkt/s "
            f"sensor={sensor_rate:.1f} pkt/s "
            f"bytes={byte_rate:.0f} B/s"
        )
        log_session("RX diag", diag_msg)
        if TERMINAL_DIAG_ENABLE:
            print(f"[RX diag] {diag_msg}")
        self._reset_rx_diag()

    def _update_tx_diag(self, cmd: SendCmd, data_len: int) -> None:
        self._tx_diag_total_cmds += 1
        self._tx_diag_bytes += int(data_len)
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "angle_live":
            self._tx_diag_angle_live_cmds += 1

    def _maybe_print_tx_diag(self) -> None:
        now = time.time()
        elapsed = now - self._tx_diag_last_print
        if elapsed < TERMINAL_DIAG_INTERVAL_S:
            return
        total_rate = self._tx_diag_total_cmds / elapsed
        angle_live_rate = self._tx_diag_angle_live_cmds / elapsed
        byte_rate = self._tx_diag_bytes / elapsed
        diag_msg = (
            f"total={total_rate:.1f} cmd/s "
            f"angle_live={angle_live_rate:.1f} cmd/s "
            f"bytes={byte_rate:.0f} B/s"
        )
        log_session("TX diag", diag_msg)
        if TERMINAL_DIAG_ENABLE:
            print(f"[TX diag] {diag_msg}")
        self._reset_tx_diag()

    def _rx_loop(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    chunk = self.serial.read(self.serial.in_waiting)
                    self._rx_buffer.extend(chunk)
                    packets = self._extract_packets_and_firmware_logs()
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
                log_session("COMM", f"RX error: {exc}")
                print(f"RX error: {exc}")

    def _append_firmware_log_byte(self, value: int) -> None:
        if value in (9, 10, 13) or 32 <= value <= 126:
            self._firmware_text_buffer.append(value)
            if value in (10, 13):
                self._flush_firmware_log_text()
        elif self._firmware_text_buffer and self._firmware_text_buffer[-1] not in (10, 13, 32):
            self._firmware_text_buffer.append(32)

    def _flush_firmware_log_text(self) -> None:
        if not self._firmware_text_buffer:
            return
        text = bytes(self._firmware_text_buffer).decode("utf-8", errors="ignore")
        self._firmware_text_buffer.clear()
        if not text:
            return
        stripped = text.strip()
        if stripped:
            log_session("FW", stripped)
        if "<<<SYS_READY>>>" in stripped:
            # P4 may reboot after the first frame was sent; negotiate again
            # only after the firmware reports that its tasks are ready.
            self._request_stream_mode_negotiation()
        if not FIRMWARE_LOG_TO_TERMINAL:
            return
        if stripped.startswith(FIRMWARE_LOG_FILTER_PREFIXES):
            return
        if self._firmware_log_level(stripped) > FIRMWARE_LOG_TERMINAL_LEVEL:
            return
        print(text, end="", flush=True)

    @staticmethod
    def _firmware_log_level(stripped: str) -> int:
        if any(marker in stripped for marker in FIRMWARE_LOG_ERROR_MARKERS):
            return TERMINAL_LOG_LEVEL_ERROR
        if any(marker in stripped for marker in FIRMWARE_LOG_WARN_MARKERS):
            return TERMINAL_LOG_LEVEL_WARN
        if any(marker in stripped for marker in FIRMWARE_LOG_TRACE_MARKERS):
            return TERMINAL_LOG_LEVEL_TRACE
        if any(marker in stripped for marker in FIRMWARE_LOG_DEBUG_MARKERS):
            return TERMINAL_LOG_LEVEL_DEBUG
        return TERMINAL_LOG_LEVEL_INFO

    def _extract_packets_and_firmware_logs(self) -> List[tuple]:
        packets: List[tuple] = []
        buffer = self._rx_buffer
        while buffer:
            if buffer[0] != PROTOCOL_HEADER:
                self._append_firmware_log_byte(buffer.pop(0))
                continue

            if len(buffer) < 4:
                break

            frame_len = 2 + buffer[1]
            if frame_len < 4:
                self._append_firmware_log_byte(buffer.pop(0))
                continue

            if len(buffer) < frame_len:
                break

            if buffer[frame_len - 1] == PROTOCOL_TAIL:
                self._flush_firmware_log_text()
                pkt_type = buffer[2]
                payload = bytes(buffer[3:frame_len - 1])
                packets.append((pkt_type, payload))
                del buffer[:frame_len]
            else:
                self._append_firmware_log_byte(buffer.pop(0))
        return packets

    def _request_stream_mode_negotiation(self) -> None:
        if not self.running:
            return
        self._stream_mode_applied = None
        self._stream_mode_ack_pending = True
        self._stream_mode_ack_warned = False
        self._stream_mode_ack_deadline = time.time() + 2.0
        self.send_command(("stream_mode", self._stream_mode_requested))

    def _check_stream_mode_ack_timeout(self):
        if self._stream_mode_ack_pending and not self._stream_mode_ack_warned:
            if time.time() >= self._stream_mode_ack_deadline:
                log_session("PROTO", "no stream-mode ACK from firmware; continue with current decoding")
                print("Protocol warning: no stream-mode ACK from firmware; continue with current decoding.")
                self._stream_mode_ack_warned = True
                self._stream_mode_ack_pending = False

    def _stabilize_tactile_groups(self, groups: List[List[List[int]]]) -> List[List[List[int]]]:
        stable: List[List[List[int]]] = []
        if self._tactile_hold_groups is None:
            self._tactile_hold_groups = [
                [[0, 0, 0] for _ in range(3)] for _ in range(5)
            ]

        for gi in range(5):
            group_out: List[List[int]] = []
            src_group = groups[gi] if gi < len(groups) else []
            for si in range(3):
                incoming = list(src_group[si]) if si < len(src_group) else [0, 0, 0]
                while len(incoming) < 3:
                    incoming.append(0)
                incoming = [int(incoming[0]), int(incoming[1]), int(incoming[2])]
                held = self._tactile_hold_groups[gi][si]

                incoming_active = _tactile_axis_magnitude(incoming) > TACTILE_ZERO_NOISE_FLOOR
                held_mag = _tactile_axis_magnitude(held)
                if incoming_active:
                    out = incoming
                    self._tactile_zero_counts[gi][si] = 0
                elif held_mag >= TACTILE_HOLD_MIN_MAGNITUDE and self._tactile_zero_counts[gi][si] < TACTILE_ZERO_HOLD_FRAMES:
                    out = list(held)
                    self._tactile_zero_counts[gi][si] += 1
                else:
                    out = [0, 0, 0]
                    if self._tactile_zero_counts[gi][si] < 255:
                        self._tactile_zero_counts[gi][si] += 1

                self._tactile_hold_groups[gi][si] = list(out)
                group_out.append(out)
            stable.append(group_out)
        return stable

    def _process_packets(self, packets: List[tuple]) -> Optional[HandModel]:
        if packets:
            self._heartbeat_enabled = True
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
                        model.encoders[i].zero_raw = 0
                        model.encoders[i].error = errors[i] if i < len(errors) else False

                    for i in range(min(ENCODER_COUNT, len(model.motors))):
                        model.motors[i].motor_id = i
                        model.motors[i].error = errors[i] if i < len(errors) else False
                        model.motors[i].state = (
                            MotorState.ERROR if model.motors[i].error else MotorState.RUNNING
                        )

                    model.has_sensor_data = True
                    emitted = True

            elif pkt_type == PACKET_TYPE_ENCODER_RAW:
                raw_values, errors = parse_sensor_packet(payload)
                if raw_values:
                    for i in range(min(ENCODER_COUNT, len(raw_values), len(model.encoders))):
                        model.encoders[i].raw_sensor = raw_values[i]
                        model.encoders[i].error = errors[i] if i < len(errors) else False
                    emitted = True

            elif pkt_type == 0x02:
                code = parse_calib_ack(payload)
                if code == 1:
                    model.calib_status = "RUNNING"
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
                    raw_positions, hardware_positions, online_flags, has_hardware_positions = parsed
                    for i in range(min(MOTOR_COUNT, len(raw_positions))):
                        model.servo_raw_positions[i] = int(raw_positions[i])
                        if has_hardware_positions and i < len(hardware_positions):
                            model.servo_hardware_positions[i] = int(hardware_positions[i])
                        model.servo_raw_online[i] = bool(online_flags[i]) if i < len(online_flags) else False
                    model.has_servo_raw_data = True
                    model.has_servo_hardware_data = bool(has_hardware_positions)
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
                parsed_fault = parse_fault_status_packet(payload)
                if parsed_fault is not None:
                    fault_bitmap, trigger_loads = parsed_fault
                    for i in range(MOTOR_COUNT):
                        faulted = ((fault_bitmap >> i) & 0x01) != 0
                        model.servo_overload_fault[i] = faulted
                        if i < len(trigger_loads):
                            model.servo_overload_trigger_load[i] = trigger_loads[i] if faulted else None
                    model.has_fault_status_data = True
                    model.fault_status_last_rx_ts = time.time()
                    emitted = True

            elif pkt_type == PACKET_TYPE_RELEASE_FAULT:
                release_fault_bitmap = parse_release_fault_packet(payload)
                if release_fault_bitmap is not None:
                    for i in range(ENCODER_COUNT):
                        model.joint_reverse_release_fault[i] = ((release_fault_bitmap >> i) & 0x01) != 0
                    emitted = True

            elif pkt_type == PACKET_TYPE_SYSTEM_STATUS:
                parsed = parse_system_status_packet(payload)
                if parsed is not None:
                    (
                        system_state,
                        control_enabled,
                        servo_target_owner,
                        control_mode,
                        calib_status,
                        system_fault_bitmap,
                        state_transition_count,
                        multi_turn_unknown_bitmap,
                        multiturn_nvs_enabled,
                        multiturn_nvs_snapshot_valid,
                        active_motor_mask,
                        active_joint_mask,
                        require_can_feedback,
                        has_active_scope_status,
                        servo_online_bitmap,
                        encoder_online_bitmap,
                        can_link_online,
                        has_connectivity_status,
                        encoder_warn_bitmap,
                        encoder_fault_bitmap,
                        has_encoder_health_status,
                    ) = parsed
                    model.system_state = int(system_state)
                    model.control_enabled = bool(control_enabled)
                    model.servo_target_owner = int(servo_target_owner)
                    model.control_mode = int(control_mode)
                    model.system_fault_bitmap = int(system_fault_bitmap)
                    model.state_transition_count = int(state_transition_count)
                    model.multi_turn_unknown_bitmap = int(multi_turn_unknown_bitmap)
                    model.multiturn_nvs_enabled = bool(multiturn_nvs_enabled)
                    model.multiturn_nvs_snapshot_valid = bool(multiturn_nvs_snapshot_valid)
                    model.active_motor_mask = int(active_motor_mask)
                    model.active_joint_mask = int(active_joint_mask)
                    model.require_can_feedback = bool(require_can_feedback)
                    model.has_active_scope_status = bool(has_active_scope_status)
                    model.servo_online_bitmap = int(servo_online_bitmap)
                    model.encoder_online_bitmap = int(encoder_online_bitmap)
                    model.encoder_warn_bitmap = int(encoder_warn_bitmap)
                    model.encoder_fault_bitmap = int(encoder_fault_bitmap)
                    model.can_link_online = bool(can_link_online)
                    model.has_connectivity_status = bool(has_connectivity_status)
                    model.has_encoder_health_status = bool(has_encoder_health_status)
                    model.has_system_status_data = True
                    model.system_status_last_rx_ts = time.time()
                    if calib_status == 0:
                        if (
                            model.calib_status in ("PENDING", "RUNNING") and
                            model.system_state != SYSTEM_STATE_CALIBRATION_RUNNING
                        ):
                            if (model.system_fault_bitmap & SYSTEM_FAULT_CALIBRATION) != 0:
                                model.calib_status = "FAILED"
                            elif model.system_state in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY):
                                model.calib_status = "SUCCESS"
                            else:
                                model.calib_status = "IDLE"
                        else:
                            model.calib_status = "IDLE"
                    elif calib_status == 1:
                        model.calib_status = "RUNNING"
                    elif calib_status == 2:
                        model.calib_status = "SUCCESS"
                    elif calib_status == 3:
                        model.calib_status = "FAILED"
                    emitted = True

            elif pkt_type == PACKET_TYPE_MOTOR_ABS_LIMIT:
                parsed = parse_motor_abs_limit_packet(payload)
                if parsed is not None:
                    valid_bitmap, locked_bitmap, mins, maxs = parsed
                    model.motor_abs_limit_valid_bitmap = int(valid_bitmap)
                    model.motor_abs_limit_locked_bitmap = int(locked_bitmap)
                    for i in range(min(MOTOR_COUNT, len(mins), len(maxs))):
                        model.motor_abs_limit_min[i] = int(mins[i])
                        model.motor_abs_limit_max[i] = int(maxs[i])
                    emitted = True

            elif pkt_type == PACKET_TYPE_JOINT_CALIB_SNAPSHOT:
                parsed = parse_joint_calib_snapshot_packet(payload)
                if parsed is not None:
                    joint_idx = int(parsed.get("joint", -1))
                    if 0 <= joint_idx < ENCODER_COUNT:
                        model.joint_calib_results[joint_idx] = dict(parsed)
                        emitted = True

            elif pkt_type == PACKET_TYPE_CALIB_DIAG:
                parsed = parse_calib_diag_packet(payload)
                if parsed is not None:
                    model.calib_diag_valid = bool(parsed.get("valid", False))
                    model.calib_diag_reason = int(parsed.get("reason", 0) or 0)
                    model.calib_diag_phase = int(parsed.get("phase", 0) or 0)
                    model.calib_diag_joint = int(parsed.get("joint", 0xFF) or 0)
                    model.calib_diag_motor = int(parsed.get("motor", 0xFF) or 0)
                    model.calib_diag_value_a = int(parsed.get("valueA", 0) or 0)
                    model.calib_diag_value_b = int(parsed.get("valueB", 0) or 0)
                    model.calib_diag_value_c = int(parsed.get("valueC", 0) or 0)
                    model.calib_diag_bitmap = int(parsed.get("bitmap", 0) or 0)
                    model.calib_diag_timestamp_ms = int(parsed.get("timestamp_ms", 0) or 0)
                    emitted = True

            elif pkt_type == PACKET_TYPE_TACTILE:
                parsed = parse_tactile_packet(payload)
                if parsed is not None:
                    _seq, groups = parsed
                    stable_groups = self._stabilize_tactile_groups(groups)
                    self._last_tactile_summary_groups = stable_groups
                    self._last_tactile_summary_seq = int(_seq)
                    model.tactile_summary_seq = int(_seq)
                    model.tactile_full_crc_errors = self._tactile_full_crc_errors
                    model.tactile_full_seq_mismatches = self._tactile_full_seq_mismatches
                    model.tactile_full_dropped = self._tactile_full_dropped
                    if _summary_groups_all_zero(stable_groups):
                        self._last_tactile_full_ts = 0.0
                        model.tactile_data = _build_tactile_fingers(stable_groups)
                    elif (
                        not model.tactile_data or
                        (time.time() - self._last_tactile_full_ts) > TACTILE_FULL_STALE_AFTER_S
                    ):
                        model.tactile_data = _build_tactile_fingers(stable_groups)
                    emitted = True

            elif pkt_type == PACKET_TYPE_TACTILE_FULL:
                parsed = parse_tactile_full_fragment(payload)
                if parsed is not None:
                    seq, frag_idx, frag_count, chunk, summary_seq, full_crc = parsed
                    accepted = self._accept_tactile_full_fragment(
                        seq,
                        frag_idx,
                        frag_count,
                        chunk,
                        summary_seq,
                        full_crc,
                    )
                    if accepted is not None:
                        raw, accepted_summary_seq, accepted_crc = accepted
                        if (
                            accepted_summary_seq is not None and
                            self._last_tactile_summary_seq != accepted_summary_seq
                        ):
                            self._tactile_full_seq_mismatches += 1
                            model.tactile_full_seq_mismatches = self._tactile_full_seq_mismatches
                            emitted = True
                            continue
                        model.tactile_data = _build_tactile_fingers_from_full(
                            raw,
                            self._last_tactile_summary_groups,
                        )
                        model.tactile_full_seq = int(seq)
                        model.tactile_full_summary_seq = (
                            int(accepted_summary_seq)
                            if accepted_summary_seq is not None
                            else None
                        )
                        model.tactile_full_crc = (
                            int(accepted_crc)
                            if accepted_crc is not None
                            else None
                        )
                        model.tactile_full_crc_errors = self._tactile_full_crc_errors
                        model.tactile_full_seq_mismatches = self._tactile_full_seq_mismatches
                        model.tactile_full_dropped = self._tactile_full_dropped
                        self._last_tactile_full_ts = time.time()
                        emitted = True

            elif pkt_type == 0x06:
                parsed = parse_proto_ack(payload)
                if parsed is not None:
                    ack_cmd, applied_mode, status = parsed
                    log_session(
                        "PROTO",
                        f"ACK cmd=0x{int(ack_cmd):02X} mode={int(applied_mode)} status={int(status)}",
                    )
                    if ack_cmd == CMD_SENSOR_STREAM_MODE:
                        self._stream_mode_applied = int(applied_mode)
                        self._stream_mode_ack_pending = False
                        if status != PROTO_ACK_STATUS_OK and not self._stream_mode_ack_warned:
                            print(
                                f"Protocol warning: stream-mode rejected "
                                f"(status={status}, applied={applied_mode})."
                            )
                            self._stream_mode_ack_warned = True
                    with self._ack_lock:
                        self._last_proto_ack = (int(ack_cmd), int(applied_mode), int(status), time.time())
                        self._ack_lock.notify_all()
                    if (
                        ack_cmd in (CMD_STOP, CMD_RESET) and
                        status in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED)
                    ):
                        model.control_enabled = False
                        model.system_state = SYSTEM_STATE_UNCONTROLLABLE
                        if model.calib_status in ("PENDING", "RUNNING"):
                            model.calib_status = "IDLE"
                    if ack_cmd in (
                        CMD_START,
                        CMD_STOP,
                        CMD_RESET,
                        CMD_CALIBRATE,
                        CMD_CALIB_DATA,
                        CMD_ACTIVE_SCOPE,
                        CMD_RUN_FEEDBACK,
                        CMD_PAUSE,
                        CMD_RESTORE_MULTITURN,
                        CMD_MOTOR_ABS_LIMIT_LOCK,
                        CMD_MULTITURN_NVS_CONTROL,
                        CMD_CLEAR_CALIBRATION,
                    ) and status == PROTO_ACK_STATUS_OK:
                        model.system_state = int(applied_mode)
                        if ack_cmd == CMD_START:
                            model.control_enabled = False
                        elif ack_cmd == CMD_STOP:
                            model.control_enabled = False
                            model.system_state = SYSTEM_STATE_UNCONTROLLABLE
                            if model.calib_status in ("PENDING", "RUNNING"):
                                model.calib_status = "IDLE"
                        elif ack_cmd == CMD_RESET:
                            model.control_enabled = False
                            model.system_state = SYSTEM_STATE_UNCONTROLLABLE
                            if model.calib_status in ("PENDING", "RUNNING"):
                                model.calib_status = "IDLE"
                        elif ack_cmd == CMD_CALIBRATE:
                            model.control_enabled = False
                            model.system_state = SYSTEM_STATE_CALIBRATION_RUNNING
                        elif ack_cmd == CMD_CALIB_DATA:
                            model.control_enabled = False
                        elif ack_cmd == CMD_ACTIVE_SCOPE:
                            pass
                        elif ack_cmd == CMD_RUN_FEEDBACK:
                            model.control_enabled = int(applied_mode) == SYSTEM_STATE_RUN_FEEDBACK
                        elif ack_cmd == CMD_PAUSE:
                            model.control_enabled = False
                            model.system_state = SYSTEM_STATE_PAUSED
                        elif ack_cmd == CMD_RESTORE_MULTITURN:
                            pass
                        elif ack_cmd == CMD_MOTOR_ABS_LIMIT_LOCK:
                            pass
                        elif ack_cmd == CMD_MULTITURN_NVS_CONTROL:
                            pass
                        elif ack_cmd == CMD_CLEAR_CALIBRATION:
                            model.calib_status = "IDLE"
                            model.motor_abs_limit_valid_bitmap = 0
                            model.motor_abs_limit_locked_bitmap = 0
                            model.joint_calib_results = [None] * ENCODER_COUNT
                    emitted = True

        self._apply_connectivity_visibility(model)
        self._last_model = model
        return model if emitted else None

    @staticmethod
    def _apply_connectivity_visibility(model: HandModel) -> None:
        if bool(getattr(model, "has_connectivity_status", False)):
            servo_mask = int(getattr(model, "servo_online_bitmap", 0) or 0)
            for i in range(MOTOR_COUNT):
                online = (servo_mask & (1 << i)) != 0
                if i < len(model.servo_online):
                    model.servo_online[i] = online
                if i < len(model.servo_raw_online):
                    model.servo_raw_online[i] = online
                if i < len(model.servo_telem_online):
                    model.servo_telem_online[i] = online
                if i < len(model.motors):
                    model.motors[i].error = not online
                    model.motors[i].state = MotorState.RUNNING if online else MotorState.ERROR

        if bool(getattr(model, "has_encoder_health_status", False)):
            online_mask = int(getattr(model, "encoder_online_bitmap", 0) or 0)
            fault_mask = int(getattr(model, "encoder_fault_bitmap", 0) or 0)
            for i in range(min(ENCODER_COUNT, len(model.encoders))):
                usable = ((online_mask & (1 << i)) != 0) and ((fault_mask & (1 << i)) == 0)
                model.encoders[i].error = not usable

    def send_command(self, cmd: SendCmd):
        if cmd == "stop":
            self._enqueue_stop_immediately()
            return
        if cmd == "pause":
            self._enqueue_pause_immediately()
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

    def _enqueue_pause_immediately(self) -> None:
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            kept = [item for item in pending if not self._is_high_rate_command(item)]
            pending.clear()
            pending.extend(kept)
            pending.append("pause")
            self.tx_queue.unfinished_tasks += 1
            self.tx_queue.not_empty.notify()

    @staticmethod
    def _is_high_rate_command(cmd: SendCmd) -> bool:
        return isinstance(cmd, tuple) and cmd[0] in (
            "motor_pos_sweep",
            "motor_pos_abs",
            "motor_pos_abs_one",
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

    def wait_for_ack(
        self,
        cmd: int,
        timeout_s: float = 1.0,
        received_after: Optional[float] = None,
    ) -> Optional[tuple]:
        deadline = time.time() + max(0.0, float(timeout_s))
        with self._ack_lock:
            while True:
                ack = self._last_proto_ack
                ack_is_new = (
                    received_after is None or
                    (ack is not None and len(ack) >= 4 and float(ack[3]) >= float(received_after))
                )
                if ack is not None and int(ack[0]) == int(cmd) and ack_is_new:
                    return ack
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._ack_lock.wait(timeout=remaining)

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
                self._maybe_send_host_heartbeat()
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
                log_session("COMM", f"TX error: {exc}")
                print(f"TX error: {exc}")

    def _maybe_send_host_heartbeat(self) -> None:
        """Send a lightweight online heartbeat without using the normal command queue."""
        now = time.time()
        if now - self._last_heartbeat_tx < HOST_HEARTBEAT_INTERVAL_S:
            return
        if not (self.serial and self.serial.is_open):
            return
        if not self._heartbeat_enabled:
            return
        data = build_host_heartbeat_cmd()
        with self._tx_lock:
            self.serial.write(data)
        self._last_heartbeat_tx = now
        self._update_tx_diag("host_heartbeat", len(data))

    def _encode_command(self, cmd: SendCmd) -> Optional[bytes]:
        if cmd == "calibrate":
            return build_calibrate_cmd()
        if cmd == "side_sway_adjust":
            return build_side_sway_adjust_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "calib_data":
            return build_calib_data_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 4 and cmd[0] == "active_scope":
            return build_active_scope_cmd(cmd[1], cmd[2], bool(cmd[3]))
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "restore_multiturn":
            return build_restore_multiturn_cmd(cmd[1], cmd[2])
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "motor_abs_limit_lock":
            return build_motor_abs_limit_lock_cmd(cmd[1], bool(cmd[2]))
        if isinstance(cmd, tuple) and len(cmd) == 5 and cmd[0] == "calib_load_limits":
            return build_calib_load_limits_cmd(cmd[1], cmd[2], cmd[3], cmd[4])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "calib_load_joint":
            return build_calib_load_joint_cmd(cmd[1])
        if cmd == "clear_calibration":
            return build_clear_calibration_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "multiturn_nvs_control":
            return build_multiturn_nvs_control_cmd(bool(cmd[1]))
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_pos":
            return build_motor_pos_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_pos_sweep":
            return build_motor_pos_sweep_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_pos_abs":
            return build_motor_pos_abs_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "motor_pos_abs_one":
            return build_motor_pos_abs_one_cmd(cmd[1], cmd[2])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "angle_live":
            return build_angle_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "stream_mode":
            return build_stream_mode_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "local_test_params":
            return build_local_test_params_cmd(cmd[1], cmd[2])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "tendon_guard":
            return build_tendon_guard_cmd(cmd[1])
        if cmd == "local_test_start":
            return build_local_test_start_cmd()
        if cmd == "local_test_stop":
            return build_local_test_stop_cmd()
        if cmd == "start":
            return build_start_cmd()
        if cmd == "run_feedback":
            return build_run_feedback_cmd()
        if cmd == "pause":
            return build_pause_cmd()
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
