from __future__ import annotations

import os
import queue
import threading
import time
from typing import Callable, Dict, List, Optional, Union

import numpy as np
try:
    import serial
    import serial.tools.list_ports
except ModuleNotFoundError:  # pragma: no cover - depends on local environment
    serial = None

from .auto_calib_logger import AutoCalibLogger
from .data_models import FingerTactile, HandModel, MotorState, TactileSensor, tactile_segment_shape
from .protocol import (
    BAUDRATE,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PACKET_TYPE_CALIB_DIAG,
    PACKET_TYPE_FAULT_STATUS,
    PACKET_TYPE_JOINT_DEBUG,
    PACKET_TYPE_ENCODER_RAW,
    PACKET_TYPE_JOINT_CALIB_SNAPSHOT,
    PACKET_TYPE_MANUAL_ENCODER_REFERENCE,
    PACKET_TYPE_MOTOR_ABS_LIMIT,
    PACKET_TYPE_SERVO_LOGICAL_RANGE,
    PACKET_TYPE_SERVO_RAW,
    PACKET_TYPE_SYSTEM_STATUS,
    PACKET_TYPE_TASK_DIAG,
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
    build_masked_angle_cmd,
    build_active_scope_cmd,
    build_calib_load_joint_cmd,
    build_calib_load_limits_cmd,
    build_calibrate_cmd,
    build_clear_calibration_cmd,
    build_clear_calibration_scope_cmd,
    build_servo_reset_zero_cmd,
    build_host_heartbeat_cmd,
    build_motor_abs_limit_lock_cmd,
    build_motor_pos_cmd,
    build_motor_pos_abs_cmd,
    build_motor_pos_abs_one_cmd,
    build_motor_pos_sweep_cmd,
    build_multiturn_nvs_control_cmd,
    build_manual_encoder_reference_cmd,
    build_clear_manual_encoder_reference_cmd,
    build_manual_encoder_query_cmd,
    build_manual_encoder_single_cmd,
    build_rope_init_cmd,
    build_pause_cmd,
    build_restore_multiturn_cmd,
    build_run_feedback_cmd,
    build_reset_cmd,
    build_start_cmd,
    build_stop_cmd,
    CMD_CALIBRATE,
    CMD_CLEAR_CALIBRATION,
    CMD_CLEAR_CALIBRATION_SCOPE,
    CMD_SERVO_RESET_ZERO,
    CMD_CALIB_LOAD_JOINT,
    CMD_CALIB_LOAD_LIMITS,
    CMD_ACTIVE_SCOPE,
    CMD_START,
    CMD_STOP,
    CMD_RESET,
    CMD_RUN_FEEDBACK,
    CMD_PAUSE,
    CMD_RESTORE_MULTITURN,
    CMD_MOTOR_ABS_LIMIT_LOCK,
    CMD_MULTITURN_NVS_CONTROL,
    CMD_MANUAL_ENCODER_REFERENCE,
    CMD_MANUAL_ENCODER_QUERY,
    CMD_MANUAL_ENCODER_SINGLE,
    CMD_ROPE_INIT,
    CMD_ANGLE_CTRL_MASKED,
    SYSTEM_STATE_UNCONTROLLABLE,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_PAUSED,
    SYSTEM_FAULT_CALIBRATION,
    parse_calib_diag_packet,
    parse_calib_ack,
    parse_fault_status_packet,
    parse_joint_debug_packet,
    parse_joint_calib_snapshot_packet,
    parse_manual_encoder_reference_packet,
    parse_motor_abs_limit_packet,
    parse_servo_logical_range_packet,
    parse_proto_ack,
    parse_sensor_packet,
    parse_servo_angle_packet,
    parse_servo_raw_packet,
    parse_servo_telem_packet,
    parse_system_status_packet,
    parse_task_diag_packet,
    parse_tactile_full_fragment,
    parse_tactile_packet,
    split_p4_timestamp,
    TACTILE_GROUP_COUNT,
    TACTILE_FULL_BYTES,
    TACTILE_SENSOR_PER_GROUP,
    TACTILE_SENSOR_TAXEL_COUNTS,
    crc16_ccitt,
    PROTO_ACK_STATUS_OK,
    PROTO_ACK_STATUS_QUEUED,
)
from .run_data_logger import RunDataLogger
from .session_log import log_session

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
MAPPED_ANGLE_COUNTS_PER_DEG = 16384.0 / 360.0


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
    if serial is None:
        return []
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
        if serial is None:
            print("Missing Python dependency: pyserial. Install it with: python -m pip install pyserial")
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
        model.host_rx_time = time.time()
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
        model.host_rx_time = time.time()
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
        self._offline_lock = threading.Lock()
        self._ack_lock = threading.Condition()
        self._rx_buffer = bytearray()
        self._firmware_text_buffer = bytearray()
        self._last_model: Optional[HandModel] = None
        self._last_proto_ack: Optional[tuple] = None
        self._proto_ack_sequence = 0
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
        self._offline_reported = False
        self._joint_debug_packet_logger = RunDataLogger(flush_every=20)
        self._joint_debug_packet_callback: Optional[Callable[[dict], None]] = None
        self._auto_calib_logger = AutoCalibLogger(self.port)
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
        if serial is None:
            raise RuntimeError("Missing Python dependency: pyserial. Install it with: python -m pip install pyserial")
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=SERIAL_TIMEOUT)
            self.running = True
            self._offline_reported = False
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
            auto_calib_path = self._auto_calib_logger.start(self.port)
            log_session("AUTO_CALIB", f"log started {auto_calib_path}")
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._rx_thread.start()
            self._tx_thread.start()
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
        self._heartbeat_enabled = False
        self.stop_joint_debug_packet_log()
        auto_calib_path = self._auto_calib_logger.close()
        if auto_calib_path:
            log_session("AUTO_CALIB", f"log saved {auto_calib_path}")
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

    def is_connected(self) -> bool:
        return bool(self.running and self.serial and getattr(self.serial, "is_open", False))

    def _build_offline_model(self) -> HandModel:
        model = HandModel()
        if self._last_model is not None:
            model.copy_state_from(self._last_model)
        model.host_rx_time = time.time()
        model.calib_status = "DISCONNECTED"
        model.has_system_status_data = False
        model.has_feedback_collision_status = False
        model.feedback_collision_armed_bitmap = 0
        model.feedback_collision_hold_bitmap = 0
        model.has_connectivity_status = False
        model.has_encoder_health_status = False
        model.has_servo_angle_data = False
        model.has_servo_raw_data = False
        model.has_servo_flash_raw_data = False
        model.has_fault_status_data = False
        model.servo_online_bitmap = 0
        model.encoder_online_bitmap = 0
        model.encoder_warn_bitmap = 0
        model.encoder_fault_bitmap = 0
        model.can_link_online = False
        model.servo_online = [False] * MOTOR_COUNT
        model.servo_raw_online = [False] * MOTOR_COUNT
        model.servo_telem_online = [False] * MOTOR_COUNT
        for motor in model.motors:
            motor.error = True
            motor.state = MotorState.ERROR
        model.tactile_online = False
        model.tactile_module_online = [False] * len(model.tactile_module_online)
        self._last_model = model
        return model

    def _push_rx_model(self, model: HandModel) -> None:
        try:
            self.rx_queue.put_nowait(model)
        except queue.Full:
            try:
                self.rx_queue.get_nowait()
                self.rx_queue.task_done()
            except queue.Empty:
                pass
            try:
                self.rx_queue.put_nowait(model)
            except queue.Full:
                pass

    def _mark_comm_offline(self, reason: str, exc: Optional[BaseException] = None) -> None:
        with self._offline_lock:
            if self._offline_reported:
                return
            self._offline_reported = True
            detail = f"{reason}: {exc}" if exc is not None else reason
            log_session("COMM", f"serial offline {detail}")
            print(f"Serial offline: {detail}")
            self.running = False
            self._heartbeat_enabled = False
            with self._ack_lock:
                self._last_proto_ack = None
                self._ack_lock.notify_all()
            try:
                if self.serial and getattr(self.serial, "is_open", False):
                    self.serial.close()
            except Exception as close_exc:
                log_session("COMM", f"serial close after offline failed: {close_exc}")
            self.serial = None
            self._push_rx_model(self._build_offline_model())

    def start_joint_debug_packet_log(self, path: str, row_callback: Optional[Callable[[dict], None]] = None) -> None:
        fieldnames = [
            "host_time",
            "p4_time_ms",
            "joint_index",
            "valid",
            "target_deg",
            "actual_deg",
            "loop1_output",
            "cmd_valid",
            "cmd_target_pos",
        ]
        self._joint_debug_packet_callback = row_callback
        if path:
            self._joint_debug_packet_logger.start(path, fieldnames=fieldnames)

    def stop_joint_debug_packet_log(self) -> None:
        self._joint_debug_packet_logger.stop()
        self._joint_debug_packet_callback = None

    def _log_joint_debug_packet(
        self,
        host_ts_wall: float,
        joint_index: int,
        valid: bool,
        target_deg: float,
        actual_deg: float,
        loop1_out: float,
        cmd_valid: bool,
        cmd_target_pos: int,
        p4_time_ms: int,
    ) -> None:
        row = {
            "host_time": float(host_ts_wall),
            "p4_time_ms": int(p4_time_ms),
            "joint_index": int(joint_index),
            "valid": 1 if valid else 0,
            "target_deg": float(target_deg),
            "actual_deg": float(actual_deg),
            "loop1_output": float(loop1_out),
            "cmd_valid": 1 if cmd_valid else 0,
            "cmd_target_pos": int(cmd_target_pos),
        }
        if self._joint_debug_packet_logger.is_running():
            self._joint_debug_packet_logger.write_row(row)
        callback = self._joint_debug_packet_callback
        if callback is not None:
            try:
                callback(dict(row))
            except Exception:
                pass

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
        if isinstance(cmd, tuple) and cmd[0] in ("angle_live", "angle_live_masked"):
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
                self._maybe_print_rx_diag()
            except Exception as exc:
                log_session("COMM", f"RX error: {exc}")
                print(f"RX error: {exc}")
                self._mark_comm_offline("RX error", exc)
                break

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
            if (
                "[AUTO CALIB]" in stripped
                or "[BOOT CALIB]" in stripped
                or "[CALIB_TX_APPLY]" in stripped
            ):
                self._auto_calib_logger.log("FW", stripped)
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
            model.has_encoder_raw_data = bool(getattr(self._last_model, "has_encoder_raw_data", False))
            model.encoder_raw_last_rx_ts = float(
                getattr(self._last_model, "encoder_raw_last_rx_ts", 0.0) or 0.0
            )
            model.has_servo_angle_data = bool(self._last_model.has_servo_angle_data)
            model.has_servo_raw_data = bool(self._last_model.has_servo_raw_data)

        model.host_rx_time = time.time()
        emitted = False

        for pkt_type, payload in packets:
            # 当前协议统一把 P4 millis() 放在业务负载末尾。解析业务字段前先剥离，
            # 并把它登记为本次状态更新对应的下位机时间。
            payload, packet_p4_time_ms = split_p4_timestamp(pkt_type, payload)
            if packet_p4_time_ms is not None:
                model.p4_time_ms = int(packet_p4_time_ms)
                model.has_p4_time = True
            if pkt_type == 0x01:
                angles, errors = parse_sensor_packet(payload)
                if angles:
                    for i in range(min(ENCODER_COUNT, len(angles), len(model.encoders))):
                        model.encoders[i].encoder_id = i
                        angle_error = errors[i] if i < len(errors) else False
                        model.encoders[i].angle_error = angle_error
                        if not angle_error:
                            # P4 validFlags is authoritative for mapped-angle validity.
                            # Values outside the calibrated workspace are still real
                            # feedback and must not leave a stale angle frozen in the UI.
                            model.encoders[i].raw = angles[i]
                            model.encoders[i].angle_out_of_workspace = (
                                self._mapped_angle_out_of_workspace(i, int(angles[i]), model)
                            )
                        else:
                            model.encoders[i].angle_out_of_workspace = False
                        model.encoders[i].zero_raw = 0
                        model.encoders[i].error = angle_error

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
                        raw_error = errors[i] if i < len(errors) else False
                        model.encoders[i].raw_error = raw_error
                        if not raw_error:
                            model.encoders[i].raw_sensor = raw_values[i]
                        model.encoders[i].error = raw_error
                    # Raw CAN magnetic encoder data remains meaningful before joint calibration.
                    model.has_encoder_raw_data = True
                    model.encoder_raw_last_rx_ts = time.time()
                    emitted = True

            elif pkt_type == 0x02:
                code = parse_calib_ack(payload)
                if code == 1:
                    model.calib_status = "RUNNING"
                elif code == 2:
                    model.calib_status = "SUCCESS"
                elif code == 3:
                    model.calib_status = "FAILED"
                elif code == 4:
                    model.calib_status = "WARN"
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
                    raw_positions, flash_raw_positions, online_flags = parsed
                    for i in range(min(MOTOR_COUNT, len(raw_positions))):
                        model.servo_raw_positions[i] = int(raw_positions[i])
                        if i < len(flash_raw_positions):
                            model.servo_flash_raw_positions[i] = int(
                                flash_raw_positions[i]
                            )
                        model.servo_raw_online[i] = bool(online_flags[i]) if i < len(online_flags) else False
                    model.has_servo_raw_data = True
                    model.has_servo_flash_raw_data = True
                    emitted = True

            elif pkt_type == 0x05:
                parsed = parse_servo_telem_packet(payload)
                if parsed is not None:
                    speeds, loads, currents, volts, temps, online = parsed
                    for i in range(min(MOTOR_COUNT, len(speeds))):
                        model.servo_speed[i] = int(speeds[i])
                        model.servo_load[i] = int(loads[i])
                        model.servo_current[i] = int(currents[i])
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
                        cmd_valid=cmd_valid,
                        cmd_target_pos=cmd_target_pos,
                        p4_time_ms=device_timestamp_ms,
                    )
                    model.p4_time_ms = int(device_timestamp_ms)
                    model.has_p4_time = True
                    if 0 <= joint_index < ENCODER_COUNT:
                        model.joint_debug_valid[joint_index] = valid
                        model.joint_debug_target_deg[joint_index] = float(target_deg)
                        model.joint_debug_actual_deg[joint_index] = float(actual_deg)
                        model.joint_debug_loop1_output[joint_index] = float(loop1_out)
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
                        multiturn_nvs_save_request_token,
                        multiturn_nvs_save_result_token,
                        multiturn_nvs_save_valid_mask,
                        multiturn_nvs_save_fail_count,
                        has_multiturn_nvs_status,
                        manual_reference_valid_bitmap,
                        manual_reference_crc,
                        manual_reference_nvs_sequence,
                        manual_reference_nvs_valid,
                        has_manual_reference_status,
                        auto_calibration_valid_bitmap,
                        manual_reference_save_status,
                        has_manual_v2_status,
                        rope_init_status,
                        rope_init_reason,
                        rope_init_sequence,
                        rope_init_joint_mask,
                        rope_init_timestamp_ms,
                        pending_direct_generation,
                        applied_direct_generation,
                        direct_reject_reason,
                        has_motion_result_status,
                        servo_turn_overflow_bitmap,
                        servo_reset_zero_request_token,
                        servo_reset_zero_result_token,
                        servo_reset_zero_success_mask,
                        servo_reset_zero_failed_mask,
                        servo_reset_zero_invalidated_joint_mask,
                        servo_reset_zero_status,
                        has_servo_reset_zero_status,
                        feedback_admission_sequence,
                        feedback_admission_status,
                        feedback_reject_reason,
                        has_feedback_admission_status,
                        feedback_collision_armed_bitmap,
                        feedback_collision_hold_bitmap,
                        feedback_collision_transition_count,
                        has_feedback_collision_status,
                        calibration_warning_bitmap,
                        calibration_auto_min_offset_counts,
                        has_calibration_warning_status,
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
                    model.multiturn_nvs_save_request_token = int(multiturn_nvs_save_request_token)
                    model.multiturn_nvs_save_result_token = int(multiturn_nvs_save_result_token)
                    model.multiturn_nvs_save_valid_mask = int(multiturn_nvs_save_valid_mask)
                    model.multiturn_nvs_save_fail_count = int(multiturn_nvs_save_fail_count)
                    model.has_multiturn_nvs_status = bool(has_multiturn_nvs_status)
                    model.manual_reference_valid_bitmap = int(manual_reference_valid_bitmap)
                    model.manual_reference_crc = int(manual_reference_crc)
                    model.manual_reference_nvs_sequence = int(manual_reference_nvs_sequence)
                    model.manual_reference_nvs_valid = bool(manual_reference_nvs_valid)
                    model.has_manual_reference_status = bool(has_manual_reference_status)
                    if has_manual_v2_status:
                        model.auto_calibration_valid_bitmap = int(
                            auto_calibration_valid_bitmap
                        )
                        model.manual_reference_save_status = int(
                            manual_reference_save_status
                        )
                    if has_motion_result_status:
                        model.rope_init_status = int(rope_init_status)
                        model.rope_init_reason = int(rope_init_reason)
                        model.rope_init_sequence = int(rope_init_sequence)
                        model.rope_init_joint_mask = int(rope_init_joint_mask)
                        model.rope_init_timestamp_ms = int(rope_init_timestamp_ms)
                        model.pending_direct_generation = int(pending_direct_generation)
                        model.applied_direct_generation = int(applied_direct_generation)
                        model.direct_reject_reason = int(direct_reject_reason)
                        model.has_motion_result_status = True
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
                    model.calibration_warning_bitmap = int(
                        calibration_warning_bitmap
                    )
                    model.calibration_auto_min_offset_counts = list(
                        calibration_auto_min_offset_counts
                    )
                    model.has_calibration_warning_status = bool(
                        has_calibration_warning_status
                    )
                    model.feedback_admission_sequence = int(feedback_admission_sequence)
                    model.feedback_admission_status = int(feedback_admission_status)
                    model.feedback_reject_reason = int(feedback_reject_reason)
                    model.has_feedback_admission_status = bool(
                        has_feedback_admission_status
                    )
                    model.feedback_collision_armed_bitmap = int(
                        feedback_collision_armed_bitmap
                    )
                    model.feedback_collision_hold_bitmap = int(
                        feedback_collision_hold_bitmap
                    )
                    model.feedback_collision_transition_count = int(
                        feedback_collision_transition_count
                    )
                    model.has_feedback_collision_status = bool(
                        has_feedback_collision_status
                    )
                    model.servo_turn_overflow_bitmap = int(
                        servo_turn_overflow_bitmap
                    )
                    model.servo_reset_zero_request_token = int(
                        servo_reset_zero_request_token
                    )
                    model.servo_reset_zero_result_token = int(
                        servo_reset_zero_result_token
                    )
                    model.servo_reset_zero_success_mask = int(
                        servo_reset_zero_success_mask
                    )
                    model.servo_reset_zero_failed_mask = int(
                        servo_reset_zero_failed_mask
                    )
                    model.servo_reset_zero_invalidated_joint_mask = int(
                        servo_reset_zero_invalidated_joint_mask
                    )
                    model.servo_reset_zero_status = int(
                        servo_reset_zero_status
                    )
                    model.has_servo_reset_zero_status = bool(
                        has_servo_reset_zero_status
                    )
                    model.has_system_status_data = True
                    model.system_status_last_rx_ts = time.time()
                    if calib_status == 0:
                        if (
                            model.calib_status in ("PENDING", "RUNNING") and
                            model.system_state != SYSTEM_STATE_CALIBRATION_RUNNING
                        ):
                            if (model.system_fault_bitmap & SYSTEM_FAULT_CALIBRATION) != 0:
                                model.calib_status = "FAILED"
                            elif model.system_state in (
                                SYSTEM_STATE_IDLE,
                                SYSTEM_STATE_READY,
                                SYSTEM_STATE_RECOVERY_REQUIRED,
                            ):
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
                    elif calib_status == 4:
                        model.calib_status = "WARN"
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

            elif pkt_type == PACKET_TYPE_SERVO_LOGICAL_RANGE:
                parsed = parse_servo_logical_range_packet(payload)
                if parsed is not None:
                    valid_bitmap, mins, maxs = parsed
                    model.servo_logical_range_valid_bitmap = int(valid_bitmap)
                    for i in range(min(MOTOR_COUNT, len(mins), len(maxs))):
                        model.servo_logical_range_min[i] = int(mins[i])
                        model.servo_logical_range_max[i] = int(maxs[i])
                    model.has_servo_logical_range_status = True
                    emitted = True

            elif pkt_type == PACKET_TYPE_MANUAL_ENCODER_REFERENCE:
                parsed = parse_manual_encoder_reference_packet(payload)
                if parsed is not None:
                    model.manual_reference_valid_bitmap = int(parsed["valid_bitmap"])
                    model.manual_reference_crc = int(parsed["reference_crc"])
                    model.manual_reference_nvs_sequence = int(parsed["nvs_sequence"])
                    model.manual_reference_nvs_valid = bool(parsed["nvs_valid"])
                    model.auto_calibration_valid_bitmap = int(
                        parsed.get("auto_valid_bitmap", 0)
                    )
                    model.manual_reference_save_status = int(
                        parsed.get("save_status", 0)
                    )
                    model.manual_reference_last_rx_time = time.time()
                    model.has_manual_reference_status = True
                    model.manual_reference_min_raw = list(parsed["min_raw"])
                    model.manual_reference_max_raw = list(parsed["max_raw"])
                    model.manual_reference_encoder_directions = list(
                        parsed.get("encoder_directions", [0] * ENCODER_COUNT)
                    )
                    model.has_manual_reference_encoder_directions = bool(
                        parsed.get("has_encoder_directions", False)
                    )
                    emitted = True

            elif pkt_type == PACKET_TYPE_JOINT_CALIB_SNAPSHOT:
                parsed = parse_joint_calib_snapshot_packet(payload)
                if parsed is not None:
                    joint_idx = int(parsed.get("joint", -1))
                    if 0 <= joint_idx < ENCODER_COUNT:
                        new_result = dict(parsed)
                        old_result = model.joint_calib_results[joint_idx]
                        if old_result != new_result:
                            # The cached mapped count may have been produced by the
                            # previous calibration. Wait for one fresh P4 angle frame
                            # before showing Current again.
                            model.encoders[joint_idx].raw = 0
                            model.encoders[joint_idx].angle_error = True
                            model.encoders[joint_idx].angle_out_of_workspace = False
                        model.joint_calib_results[joint_idx] = new_result
                        emitted = True

            elif pkt_type == PACKET_TYPE_CALIB_DIAG:
                parsed = parse_calib_diag_packet(payload)
                if parsed is not None:
                    self._log_auto_calib_diag(parsed)
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
                    model.p4_time_ms = model.calib_diag_timestamp_ms
                    model.has_p4_time = True
                    emitted = True

            elif pkt_type == PACKET_TYPE_TASK_DIAG:
                parsed = parse_task_diag_packet(payload)
                if parsed is not None:
                    task_id = int(parsed.get("task_id", -1) or -1)
                    if task_id >= 0:
                        now = time.time()
                        parsed["host_rx_ts"] = now
                        model.task_runtime[task_id] = dict(parsed)
                        model.task_runtime_last_rx_ts = now
                        model.p4_time_ms = int(parsed.get("device_timestamp_ms", 0) or 0)
                        model.has_p4_time = True
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
                    if ack_cmd in (
                        CMD_CALIBRATE,
                        CMD_CLEAR_CALIBRATION,
                        CMD_CLEAR_CALIBRATION_SCOPE,
                        CMD_SERVO_RESET_ZERO,
                    ):
                        self._auto_calib_logger.log(
                            "ACK",
                            f"cmd=0x{int(ack_cmd):02X} mode={int(applied_mode)} status={int(status)}",
                        )
                    log_session(
                        "PROTO",
                        f"ACK cmd=0x{int(ack_cmd):02X} mode={int(applied_mode)} status={int(status)}",
                    )
                    with self._ack_lock:
                        self._last_proto_ack = (int(ack_cmd), int(applied_mode), int(status), time.time())
                        self._ack_lock.notify_all()
                    # ACK 序号只在收到新 ACK 时递增，UI 可据此只记录一次 P4 执行事件。
                    self._proto_ack_sequence += 1
                    model.proto_ack_sequence = self._proto_ack_sequence
                    model.proto_ack_cmd = int(ack_cmd)
                    model.proto_ack_mode = int(applied_mode)
                    model.proto_ack_status = int(status)
                    model.proto_ack_p4_time_ms = int(packet_p4_time_ms or 0)
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
                        CMD_ACTIVE_SCOPE,
                        CMD_RUN_FEEDBACK,
                        CMD_PAUSE,
                        CMD_RESTORE_MULTITURN,
                        CMD_MOTOR_ABS_LIMIT_LOCK,
                        CMD_MULTITURN_NVS_CONTROL,
                        CMD_MANUAL_ENCODER_REFERENCE,
                        CMD_MANUAL_ENCODER_SINGLE,
                        CMD_CLEAR_CALIBRATION,
                        CMD_CLEAR_CALIBRATION_SCOPE,
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
                        elif ack_cmd in (
                            CMD_MANUAL_ENCODER_REFERENCE,
                            CMD_MANUAL_ENCODER_SINGLE,
                        ):
                            model.calib_status = "IDLE"
                            model.motor_abs_limit_valid_bitmap = 0
                            model.motor_abs_limit_locked_bitmap = 0
                            model.joint_calib_results = [None] * ENCODER_COUNT
                        elif ack_cmd == CMD_CLEAR_CALIBRATION:
                            model.calib_status = "IDLE"
                            model.motor_abs_limit_valid_bitmap = 0
                            model.motor_abs_limit_locked_bitmap = 0
                            model.joint_calib_results = [None] * ENCODER_COUNT
                            for encoder in model.encoders:
                                encoder.raw = 0
                                encoder.angle_error = True
                                encoder.angle_out_of_workspace = False
                        elif ack_cmd == CMD_CLEAR_CALIBRATION_SCOPE:
                            # 局部清除的准确 joint/motor mask 由后续 P4 状态包
                            # 回读，不能像全局清除一样在通信层把 21 路全部归零。
                            pass
                        elif ack_cmd == CMD_SERVO_RESET_ZERO:
                            pass
                    emitted = True

        self._apply_connectivity_visibility(model)
        self._last_model = model
        return model if emitted else None

    @staticmethod
    def _mapped_angle_out_of_workspace(joint_idx: int, value: int, model: HandModel) -> bool:
        if joint_idx < 0 or joint_idx >= ENCODER_COUNT:
            return False
        results = getattr(model, "joint_calib_results", []) or []
        if joint_idx >= len(results):
            return False
        result = results[joint_idx]
        if not isinstance(result, dict) or not bool(result.get("success", False)):
            return False
        try:
            angle_min = float(result.get("angleMin", 0.0) or 0.0)
            angle_max = float(result.get("angleMax", 0.0) or 0.0)
        except (TypeError, ValueError):
            return False
        if not np.isfinite(angle_min) or not np.isfinite(angle_max) or angle_min >= angle_max:
            return False
        min_count = int(round(angle_min * MAPPED_ANGLE_COUNTS_PER_DEG))
        max_count = int(round(angle_max * MAPPED_ANGLE_COUNTS_PER_DEG))
        return int(value) < min_count or int(value) > max_count

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

    def send_command(self, cmd: SendCmd) -> bool:
        if not self.is_connected():
            log_session("COMM", f"drop TX while offline cmd={cmd!r}")
            return False
        if cmd in ("calibrate", "clear_calibration") or (
            isinstance(cmd, tuple) and len(cmd) == 2 and
            cmd[0] == "clear_calibration_scope"
        ):
            self._auto_calib_logger.log("TX", str(cmd))
        elif isinstance(cmd, tuple) and len(cmd) >= 1 and cmd[0] in (
            "active_scope",
            "calib_load_limits",
            "calib_load_joint",
        ):
            self._auto_calib_logger.log("TX", repr(cmd))
        if cmd == "stop":
            self._enqueue_stop_immediately()
            return True
        if cmd == "pause":
            self._enqueue_pause_immediately()
            return True
        if self._is_high_rate_command(cmd):
            return self._enqueue_latest_high_rate(cmd)
        try:
            self.tx_queue.put_nowait(cmd)
            return True
        except queue.Full:
            log_session("COMM", f"drop TX queue full cmd={cmd!r}")
            return False

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
            "joint_target_run",
            "angle_live_masked",
            "joint_target_run_masked",
        )

    @staticmethod
    def _is_joint_command(cmd: SendCmd) -> bool:
        if isinstance(cmd, HandModel):
            return True
        return (
            isinstance(cmd, tuple) and cmd[0] in (
                "angle_live",
                "joint_target_run",
                "angle_live_masked",
                "joint_target_run_masked",
            )
        )

    def invalidate_pending_joint_commands(self) -> int:
        """Drop pending joint targets and their feedback ownership request."""
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            kept = [
                item for item in pending
                if not self._is_joint_command(item) and item != "run_feedback"
            ]
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

    def _enqueue_latest_high_rate(self, cmd: SendCmd) -> bool:
        # Keep only the latest pending high-rate command to avoid UI backlog.
        # For joint live control, the target must remain ahead of its paired
        # RUN_FEEDBACK request. Otherwise replacing an older angle_live packet
        # can move the new target behind RUN_FEEDBACK, letting P4 start and
        # finish a one-shot run against the stale target.
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            paired_joint_command = (
                isinstance(cmd, tuple) and cmd[0] in (
                    "joint_target_run",
                    "joint_target_run_masked",
                )
            )
            kept = []
            for item in pending:
                drop_item = self._is_high_rate_command(item)
                if paired_joint_command:
                    drop_item = drop_item or self._is_joint_command(item) or item == "run_feedback"
                if not drop_item:
                    kept.append(item)
            pending.clear()
            pending.extend(kept)

            maxsize = self.tx_queue.maxsize
            if maxsize > 0 and len(pending) >= maxsize:
                log_session("COMM", f"drop high-rate TX queue full cmd={cmd!r}")
                return False

            if isinstance(cmd, tuple) and cmd[0] in ("angle_live", "angle_live_masked"):
                for index, pending_cmd in enumerate(pending):
                    if pending_cmd == "run_feedback":
                        pending.insert(index, cmd)
                        break
                else:
                    pending.append(cmd)
            else:
                pending.append(cmd)
            self.tx_queue.unfinished_tasks += 1
            self.tx_queue.not_empty.notify()
            return True

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
                self._mark_comm_offline("TX error", exc)
                break

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

    def _log_auto_calib_diag(self, parsed: dict) -> None:
        if parsed is None:
            return
        self._auto_calib_logger.log(
            "CALIB_DIAG",
            (
                f"valid={int(bool(parsed.get('valid', False)))} "
                f"reason={int(parsed.get('reason', 0) or 0)} "
                f"phase={int(parsed.get('phase', 0) or 0)} "
                f"joint={int(parsed.get('joint', 0xFF) or 0)} "
                f"motor={int(parsed.get('motor', 0xFF) or 0)} "
                f"valueA={int(parsed.get('valueA', 0) or 0)} "
                f"valueB={int(parsed.get('valueB', 0) or 0)} "
                f"valueC={int(parsed.get('valueC', 0) or 0)} "
                f"bitmap=0x{int(parsed.get('bitmap', 0) or 0):08X} "
                f"device_ms={int(parsed.get('timestamp_ms', 0) or 0)}"
            ),
        )

    def _encode_command(self, cmd: SendCmd) -> Optional[bytes]:
        if cmd == "calibrate":
            return build_calibrate_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 4 and cmd[0] == "active_scope":
            return build_active_scope_cmd(cmd[1], cmd[2], bool(cmd[3]))
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "restore_multiturn":
            return build_restore_multiturn_cmd(cmd[1], cmd[2])
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "motor_abs_limit_lock":
            return build_motor_abs_limit_lock_cmd(cmd[1], bool(cmd[2]))
        if isinstance(cmd, tuple) and len(cmd) == 6 and cmd[0] == "calib_load_limits":
            return build_calib_load_limits_cmd(
                cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]
            )
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "calib_load_joint":
            return build_calib_load_joint_cmd(cmd[1])
        if cmd == "clear_calibration":
            return build_clear_calibration_cmd()
        if (
            isinstance(cmd, tuple) and len(cmd) == 2
            and cmd[0] == "clear_calibration_scope"
        ):
            return build_clear_calibration_scope_cmd(cmd[1])
        if (
            isinstance(cmd, tuple) and len(cmd) == 2
            and cmd[0] == "servo_reset_zero"
        ):
            return build_servo_reset_zero_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "multiturn_nvs_control":
            return build_multiturn_nvs_control_cmd(bool(cmd[1]))
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "manual_encoder_reference":
            return build_manual_encoder_reference_cmd(cmd[1], cmd[2])
        if cmd == "clear_manual_encoder_reference":
            return build_clear_manual_encoder_reference_cmd()
        if cmd == "query_manual_encoder_reference":
            return build_manual_encoder_query_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 4 and cmd[0] == "manual_encoder_single":
            return build_manual_encoder_single_cmd(cmd[1], cmd[2], cmd[3])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "rope_init":
            return build_rope_init_cmd(cmd[1])
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
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "angle_live_masked":
            return build_masked_angle_cmd(cmd[1], cmd[2])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "joint_target_run":
            # 目标角与负反馈所有权请求合为一次串口写入，避免只送达其中一条。
            return build_angle_cmd(cmd[1]) + build_run_feedback_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "joint_target_run_masked":
            return build_masked_angle_cmd(cmd[1], cmd[2]) + build_run_feedback_cmd()
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
