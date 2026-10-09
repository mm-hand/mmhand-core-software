"""comm_layer.py - MMHand Desktop 模块契约。

职责：保留历史通信 API，并编排已拆分的串口、解帧、路由、状态归并和 ACK 组件。
线程：RX/TX 线程由兼容门面创建；各通信组件不自行创建线程。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：SerialTransport 持有串口，FrameDecoder 持有半帧，StateReducer 持有 HandModel，AckTracker 持有 ACK。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
from __future__ import annotations

import os
import queue
import threading
import time
from typing import Dict, List, Optional, Union

import numpy as np
try:
    import serial
    import serial.tools.list_ports
except ModuleNotFoundError:  # pragma: no cover - depends on local environment
    serial = None

from .auto_calib_logger import AutoCalibLogger
from . import protocol as protocol_codec
from .communication import (
    AckTracker,
    EventChannel,
    FrameDecoder,
    P4PacketRouter,
    SerialTransport,
    StateReducer,
)
from .data_models import (
    FingerTactile,
    HandModel,
    MotorState,
    TACTILE_DATA_LEVEL_FULL,
    TACTILE_DATA_LEVEL_NONE,
    TACTILE_DATA_LEVEL_SUMMARY,
    TACTILE_SOURCE_P4_CAN_RELAY,
    TACTILE_SOURCE_S3_SERIAL,
    TactileSensor,
    tactile_segment_shape,
)
from .protocol import (
    BAUDRATE,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PACKET_TYPE_CALIB_DIAG,
    PACKET_TYPE_CONTROL_TRACE,
    PACKET_TYPE_FAULT_STATUS,
    PACKET_TYPE_ENCODER_RAW,
    PACKET_TYPE_JOINT_CALIB_SNAPSHOT,
    PACKET_TYPE_MANUAL_ENCODER_REFERENCE,
    PACKET_TYPE_MOTOR_ABS_LIMIT,
    PACKET_TYPE_MULTITURN_RESTORE_RESULT,
    PACKET_TYPE_SERVO_REBASE_ZERO_RESULT,
    PACKET_TYPE_DIRECT_TARGET_RESULT,
    PACKET_TYPE_PRODUCT_FINGERPRINT,
    PACKET_TYPE_SERVO_ROUTE_STATUS,
    PACKET_TYPE_SAFETY_EVENT,
    PACKET_TYPE_SENSOR,
    PACKET_TYPE_SERVO_ANGLE,
    PACKET_TYPE_SERVO_LOGICAL_RANGE,
    PACKET_TYPE_SERVO_RAW,
    PACKET_TYPE_SERVO_TELEM,
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
    build_servo_rebase_zero_cmd,
    build_motor_direct_release_cmd,
    build_host_heartbeat_cmd,
    build_motor_pos_abs_cmd,
    build_motor_pos_abs_one_cmd,
    build_manual_encoder_reference_cmd,
    build_clear_manual_encoder_reference_cmd,
    build_manual_encoder_query_cmd,
    build_manual_encoder_single_cmd,
    build_state_sync_request_cmd,
    build_task_diag_subscribe_cmd,
    build_control_trace_subscribe_cmd,
    build_rope_init_cmd,
    build_pause_cmd,
    build_restore_multiturn_cmd,
    build_run_feedback_cmd,
    build_reset_cmd,
    build_start_cmd,
    build_stop_cmd,
    parse_tactile_full_fragment,
    parse_tactile_packet,
    split_p4_timestamp,
    TACTILE_GROUP_COUNT,
    TACTILE_FULL_BYTES,
    TACTILE_SENSOR_PER_GROUP,
    TACTILE_SENSOR_TAXEL_COUNTS,
    crc16_ccitt,
)
from .session_log import log_session

# String command / HandModel / tuple command.
SendCmd = Union[str, HandModel, tuple]
# 固件连续丢失心跳后会撤销上位机控制租约；桌面 JSON 仍独立保存多圈锚点。
HOST_HEARTBEAT_INTERVAL_S = 0.5
TACTILE_ZERO_NOISE_FLOOR = 3
TACTILE_ZERO_HOLD_FRAMES = 6
TACTILE_HOLD_MIN_MAGNITUDE = 12
TACTILE_FULL_STALE_AFTER_S = 1.0
TACTILE_SERIAL_OFFLINE_AFTER_S = 0.6
TACTILE_P4_RELAY_OFFLINE_AFTER_S = 0.2
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
TACTILE_SERIAL_DIAG_PACKET_TYPE = 0x10
TACTILE_SERIAL_DIAG_VERSION = 1
TACTILE_SERIAL_DIAG_MODULE_COUNT = TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP
TACTILE_SERIAL_DIAG_PAYLOAD_BYTES = 1 + 4 + 8 * 2 + 4 * TACTILE_SERIAL_DIAG_MODULE_COUNT * 2 + 3 * TACTILE_SERIAL_DIAG_MODULE_COUNT
MAPPED_ANGLE_COUNTS_PER_DEG = 16384.0 / 360.0

# P4PacketRouter 只负责“包号 -> 稳定语义名”。当前状态归并仍按 packet_type
# 保持完全兼容，语义名用于诊断和后续把各分支进一步下沉到独立 reducer。
P4_PACKET_KINDS = {
    PACKET_TYPE_SENSOR: "sensor",
    PACKET_TYPE_ENCODER_RAW: "encoder_raw",
    PACKET_TYPE_SERVO_ANGLE: "servo_abs",
    PACKET_TYPE_SERVO_RAW: "servo_raw",
    PACKET_TYPE_SERVO_TELEM: "servo_telemetry",
    PACKET_TYPE_FAULT_STATUS: "fault_status",
    PACKET_TYPE_SYSTEM_STATUS: "system_status",
    PACKET_TYPE_PRODUCT_FINGERPRINT: "product_fingerprint",
    PACKET_TYPE_SERVO_ROUTE_STATUS: "servo_route_status",
    PACKET_TYPE_SAFETY_EVENT: "safety_event",
    PACKET_TYPE_MULTITURN_RESTORE_RESULT: "multiturn_restore_result",
    PACKET_TYPE_SERVO_REBASE_ZERO_RESULT: "servo_rebase_zero_result",
    PACKET_TYPE_DIRECT_TARGET_RESULT: "direct_target_result",
    PACKET_TYPE_MOTOR_ABS_LIMIT: "motor_abs_limit",
    PACKET_TYPE_SERVO_LOGICAL_RANGE: "servo_logical_range",
    PACKET_TYPE_MANUAL_ENCODER_REFERENCE: "manual_reference",
    PACKET_TYPE_JOINT_CALIB_SNAPSHOT: "joint_calibration_snapshot",
    PACKET_TYPE_CALIB_DIAG: "calibration_diagnostic",
    PACKET_TYPE_CONTROL_TRACE: "control_trace",
    PACKET_TYPE_TASK_DIAG: "task_diagnostic",
    PACKET_TYPE_TACTILE: "tactile_summary",
    PACKET_TYPE_TACTILE_FULL: "tactile_full",
    TACTILE_SERIAL_DIAG_PACKET_TYPE: "tactile_diagnostic",
    0x06: "protocol_ack",
}


def _parse_tactile_diagnostic_packet(payload: bytes) -> Optional[dict]:
    if len(payload) != TACTILE_SERIAL_DIAG_PAYLOAD_BYTES:
        return None
    cursor = 0
    version = int(payload[cursor])
    cursor += 1
    if version != TACTILE_SERIAL_DIAG_VERSION:
        return None

    def read_u16() -> int:
        nonlocal cursor
        value = int.from_bytes(payload[cursor:cursor + 2], "little")
        cursor += 2
        return value

    def read_u32() -> int:
        nonlocal cursor
        value = int.from_bytes(payload[cursor:cursor + 4], "little")
        cursor += 4
        return value

    def read_u16_array() -> List[int]:
        return [read_u16() for _ in range(TACTILE_SERIAL_DIAG_MODULE_COUNT)]

    result = {"version": version, "frame_count": read_u32()}
    for name in (
        "enabled_mask",
        "summary_ok_mask",
        "full_ok_mask",
        "command_fail_mask",
        "response_fail_mask",
        "crc_fail_mask",
        "status_nonzero_mask",
        "summary_retry_recovered_mask",
    ):
        result[name] = read_u16()
    for name in (
        "summary_success_count",
        "summary_failure_count",
        "full_success_count",
        "full_failure_count",
    ):
        result[name] = read_u16_array()
    for name in (
        "summary_last_fail_reason",
        "full_last_fail_reason",
        "module_last_status",
    ):
        result[name] = list(payload[cursor:cursor + TACTILE_SERIAL_DIAG_MODULE_COUNT])
        cursor += TACTILE_SERIAL_DIAG_MODULE_COUNT
    return result if cursor == len(payload) else None


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
    """独立 S3 触觉串口兼容门面，只接收触觉数据，不发送控制命令。"""

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.port = (port or "").strip()
        self.baudrate = int(baudrate)
        serial_factory = serial.Serial if serial is not None else None
        self._transport = SerialTransport(
            self.port,
            self.baudrate,
            SERIAL_TIMEOUT,
            serial_factory,
        )
        self._frame_decoder = FrameDecoder(
            PROTOCOL_HEADER,
            PROTOCOL_TAIL,
            log_byte_callback=self._append_firmware_log_byte,
            flush_log_callback=self._flush_firmware_log_text,
        )
        self.rx_queue: queue.Queue = queue.Queue(maxsize=20)
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
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
        self._last_tactile_module_valid_mask = 0
        self._tactile_module_last_rx_ts = [0.0] * (TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)
        self._tactile_offline_emitted = False
        self._last_tactile_diag_log_at = 0.0
        self._last_tactile_diag_signature = None
        self._rx_diag_last_print = time.time()
        self._rx_diag_total_packets = 0
        self._rx_diag_bytes = 0

    def connect(self) -> bool:
        if self._transport.is_open:
            return True
        if not self.port:
            return False
        if serial is None:
            print("Missing Python dependency: pyserial. Install it with: python -m pip install pyserial")
            return False
        try:
            if not is_valid_port(self.port):
                raise ValueError(f"tactile port '{self.port}' is not available")
            if not self._transport.open():
                log_session("TACTILE", f"serial open failed port={self.port}")
                return False
            handle = self._transport.serial
            log_session(
                "TACTILE",
                f"serial opened port={self.port} baud={self.baudrate} "
                f"dtr={int(bool(getattr(handle, 'dtr', False)))} "
                f"rts={int(bool(getattr(handle, 'rts', False)))}",
            )
            self.running = True
            self._frame_decoder.reset()
            self._firmware_text_buffer = bytearray()
            self._last_model = None
            self._clear_queue(self.rx_queue)
            self._reset_tactile_full_reassembly()
            self._last_tactile_summary_groups = None
            self._last_tactile_summary_seq = None
            self._last_tactile_full_ts = 0.0
            self._last_tactile_rx_ts = 0.0
            self._last_tactile_module_valid_mask = 0
            self._tactile_module_last_rx_ts = [0.0] * (TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)
            self._tactile_offline_emitted = False
            self._last_tactile_diag_log_at = 0.0
            self._last_tactile_diag_signature = None
            self._rx_diag_last_print = time.time()
            self._rx_diag_total_packets = 0
            self._rx_diag_bytes = 0
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            return True
        except Exception as exc:
            try:
                self._transport.close()
            except Exception:
                pass
            log_session("TACTILE", f"serial open failed port={self.port}: {exc}")
            print(f"Tactile serial connect failed: {exc}")
            return False

    def disconnect(self) -> None:
        self.running = False
        try:
            self._transport.close()
        except Exception:
            pass
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
        return bool(self.running and self._transport.is_open)

    def _mark_serial_offline(self) -> None:
        self.running = False
        try:
            self._transport.close()
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
        model.tactile_source = TACTILE_SOURCE_S3_SERIAL
        model.tactile_data_level = TACTILE_DATA_LEVEL_NONE
        model.tactile_module_valid_mask = 0
        model.tactile_online = False
        model.tactile_last_rx_ts = self._last_tactile_rx_ts
        model.tactile_module_online = [False] * (TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)
        model.tactile_module_last_rx_ts = list(self._tactile_module_last_rx_ts)
        model.host_rx_time = time.time()
        self._last_model = model
        self._tactile_offline_emitted = True
        return model

    def _mark_tactile_summary_received(self, module_valid_mask: int) -> None:
        """记录S3摘要链路和模块有效性；零受力不再被当成模块离线。"""
        now = time.time()
        self._last_tactile_rx_ts = now
        self._tactile_offline_emitted = False
        valid_mask = int(module_valid_mask) & ((1 << (TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)) - 1)
        self._last_tactile_module_valid_mask = valid_mask
        for module_index in range(TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP):
            if valid_mask & (1 << module_index):
                self._tactile_module_last_rx_ts[module_index] = now

    def _apply_tactile_status(self, model: HandModel) -> None:
        now = time.time()
        online = self._last_tactile_rx_ts > 0.0 and (
            now - self._last_tactile_rx_ts
        ) <= TACTILE_SERIAL_OFFLINE_AFTER_S
        valid_mask = self._last_tactile_module_valid_mask if online else 0
        model.tactile_source = TACTILE_SOURCE_S3_SERIAL
        model.tactile_module_valid_mask = int(valid_mask)
        model.tactile_online = bool(online)
        model.tactile_last_rx_ts = float(self._last_tactile_rx_ts)
        model.tactile_module_online = [
            bool(valid_mask & (1 << module_index))
            for module_index in range(TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP)
        ]
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
        while self.running and self._transport.is_open:
            try:
                chunk = self._transport.read_available()
                if chunk:
                    packets = self._frame_decoder.feed(chunk)
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
                    log_session("TACTILE", f"RX error port={self.port}: {exc}")
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
        sink = getattr(self, "recording_packet_sink", None)
        if sink is not None:
            sink("s3", packets)
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
                seq, groups, module_valid_mask = parsed
                stable_groups = self._stabilize_tactile_groups(groups)
                first_summary = self._last_tactile_rx_ts <= 0.0
                self._mark_tactile_summary_received(module_valid_mask)
                if first_summary:
                    log_session(
                        "TACTILE",
                        f"first summary port={self.port} seq={int(seq)} "
                        f"mask=0x{int(module_valid_mask) & 0x7FFF:04X}",
                    )
                self._last_tactile_summary_groups = stable_groups
                self._last_tactile_summary_seq = int(seq)
                model.tactile_summary_seq = int(seq)
                model.tactile_source = TACTILE_SOURCE_S3_SERIAL
                model.tactile_module_valid_mask = int(module_valid_mask)
                model.tactile_full_crc_errors = self._tactile_full_crc_errors
                model.tactile_full_seq_mismatches = self._tactile_full_seq_mismatches
                model.tactile_full_dropped = self._tactile_full_dropped
                if _summary_groups_all_zero(stable_groups):
                    self._last_tactile_full_ts = 0.0
                full_is_fresh = (
                    bool(model.tactile_data) and
                    (time.time() - self._last_tactile_full_ts) <= TACTILE_FULL_STALE_AFTER_S
                )
                if not full_is_fresh:
                    model.tactile_data = _build_tactile_fingers(stable_groups)
                    model.tactile_data_level = TACTILE_DATA_LEVEL_SUMMARY
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
                model.tactile_source = TACTILE_SOURCE_S3_SERIAL
                model.tactile_data_level = TACTILE_DATA_LEVEL_FULL
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

            elif pkt_type == TACTILE_SERIAL_DIAG_PACKET_TYPE:
                diag = _parse_tactile_diagnostic_packet(payload)
                if diag is None:
                    continue
                now = time.time()
                model.tactile_diag_version = int(diag["version"])
                model.tactile_diag_frame_count = int(diag["frame_count"])
                for field in (
                    "enabled_mask",
                    "summary_ok_mask",
                    "full_ok_mask",
                    "command_fail_mask",
                    "response_fail_mask",
                    "crc_fail_mask",
                    "status_nonzero_mask",
                    "summary_retry_recovered_mask",
                ):
                    setattr(model, f"tactile_diag_{field}", int(diag[field]))
                for field in (
                    "summary_success_count",
                    "summary_failure_count",
                    "full_success_count",
                    "full_failure_count",
                    "summary_last_fail_reason",
                    "full_last_fail_reason",
                    "module_last_status",
                ):
                    setattr(model, f"tactile_diag_{field}", list(diag[field]))
                model.tactile_diag_last_rx_ts = now
                signature = (
                    int(diag["summary_ok_mask"]),
                    int(diag["full_ok_mask"]),
                    int(diag["command_fail_mask"]),
                    int(diag["response_fail_mask"]),
                    int(diag["crc_fail_mask"]),
                )
                if signature != self._last_tactile_diag_signature or now - self._last_tactile_diag_log_at >= 10.0:
                    log_session(
                        "TACTILE_SPI",
                        f"frame={int(diag['frame_count'])} "
                        f"summary=0x{int(diag['summary_ok_mask']) & 0x7FFF:04X} "
                        f"full=0x{int(diag['full_ok_mask']) & 0x7FFF:04X} "
                        f"cmd=0x{int(diag['command_fail_mask']) & 0x7FFF:04X} "
                        f"rsp=0x{int(diag['response_fail_mask']) & 0x7FFF:04X} "
                        f"crc=0x{int(diag['crc_fail_mask']) & 0x7FFF:04X} "
                        f"retry=0x{int(diag['summary_retry_recovered_mask']) & 0x7FFF:04X}",
                    )
                    self._last_tactile_diag_signature = signature
                    self._last_tactile_diag_log_at = now
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
    """P4 通信兼容门面。

    旧调用方仍通过本类连接设备、提交命令、等待 ACK 和取得 ``HandModel``。内部 RX
    链路已固定为 ``SerialTransport -> FrameDecoder -> P4PacketRouter ->
    StateReducer``，TX 链路保留原有高频命令去重和命令编码，最终统一写入
    ``SerialTransport``。本类不得重新持有串口、半帧、ACK 或最新模型的真实状态。
    """

    # 触觉数据仍由兼容门面持有重组缓存；把纯函数显式暴露给 StateReducer，
    # 避免新核心模块反向导入 comm_layer 形成循环依赖。后续触觉链路拆分时可整体迁走。
    _summary_groups_all_zero = staticmethod(_summary_groups_all_zero)
    _build_tactile_fingers = staticmethod(_build_tactile_fingers)
    _build_tactile_fingers_from_full = staticmethod(_build_tactile_fingers_from_full)
    _tactile_full_stale_after_s = TACTILE_FULL_STALE_AFTER_S

    @staticmethod
    def _parse_protocol_payload(parser_name: str, payload: bytes):
        """从唯一协议模块取得 parser，并保留临时覆盖 parser 的能力。

        生产解析器统一归属 ``protocol.py``；局部同名函数只作为调试覆盖点。
        这样通信拆分后不会因漏写某个重复 import 让整条 RX 链路掉线。
        """
        parser = globals().get(str(parser_name))
        if not callable(parser):
            parser = getattr(protocol_codec, str(parser_name), None)
        if not callable(parser):
            raise RuntimeError(f"protocol parser is unavailable: {parser_name}")
        return parser(payload)

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.port = (port or "").strip()
        self.baudrate = baudrate
        serial_factory = serial.Serial if serial is not None else None
        # 五个通信组件各自拥有单一状态，LowerComputerComm 仅负责编排和旧 API 兼容。
        self._transport = SerialTransport(
            self.port,
            self.baudrate,
            SERIAL_TIMEOUT,
            serial_factory,
        )
        self._frame_decoder = FrameDecoder(
            PROTOCOL_HEADER,
            PROTOCOL_TAIL,
            log_byte_callback=self._append_firmware_log_byte,
            flush_log_callback=self._flush_firmware_log_text,
        )
        self._ack_tracker = AckTracker()
        self._safety_event_channel: EventChannel[dict] = EventChannel()
        self._packet_router = P4PacketRouter(P4_PACKET_KINDS, split_p4_timestamp)
        self._state_reducer = StateReducer(
            self._packet_router,
            self._ack_tracker,
            self._safety_event_channel,
        )
        self.rx_queue: queue.Queue = queue.Queue(maxsize=RX_QUEUE_MAXSIZE)
        self.tx_queue: queue.Queue = queue.Queue(maxsize=TX_QUEUE_MAXSIZE)
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._offline_lock = threading.Lock()
        self._firmware_text_buffer = bytearray()
        self._rx_diag_last_print = time.time()
        self._rx_diag_total_packets = 0
        self._rx_diag_sensor_packets = 0
        self._rx_diag_bytes = 0
        self._tx_diag_last_print = time.time()
        self._tx_diag_total_cmds = 0
        self._tx_diag_angle_live_cmds = 0
        self._tx_diag_bytes = 0
        self._last_heartbeat_tx = 0.0
        self._heartbeat_enabled = False
        self._offline_reported = False
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
                raise RuntimeError(f"Unable to open port '{self.port}'")

    def connect(self) -> bool:
        if self._transport.is_open:
            log_session("COMM", f"serial connected port={self.port} baud={self.baudrate}")
            return True
        if not self.port:
            return False
        if serial is None:
            raise RuntimeError("Missing Python dependency: pyserial. Install it with: python -m pip install pyserial")
        try:
            if not self._transport.open():
                return False
            self.running = True
            self._offline_reported = False
            self._frame_decoder.reset()
            self._firmware_text_buffer = bytearray()
            self._state_reducer.reset()
            self._ack_tracker.reset()
            self._safety_event_channel.reset()
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
            try:
                self._transport.close()
            except Exception:
                pass
            log_session("COMM", f"serial connect failed port={self.port}: {exc}")
            print(f"Connection failed: {exc}")
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
        auto_calib_path = self._auto_calib_logger.close()
        if auto_calib_path:
            log_session("AUTO_CALIB", f"log saved {auto_calib_path}")
        log_session("COMM", "serial disconnect requested")
        self._transport.close()
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
        return bool(self.running and self._transport.is_open)

    def _build_offline_model(self) -> HandModel:
        model = HandModel()
        previous = self._state_reducer.last_model
        if previous is not None:
            model.copy_state_from(previous)
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
        model.has_encoder_raw_data = False
        model.encoder_raw_p4_time_ms = 0
        model.servo_telem_p4_time_ms = 0
        model.servo_telem_last_rx_ts = 0.0
        model.has_direct_target_result = False
        model.direct_target_result_rx_ts = 0.0
        model.servo_logical_range_last_rx_ts = 0.0
        model.servo_logical_range_p4_time_ms = 0
        model.has_fault_status_data = False
        model.servo_online_bitmap = 0
        model.has_servo_route_status = False
        model.servo_route_controllable_bitmap = 0
        model.servo_route_conflict_bitmap = 0
        model.servo_bus_overload_mask = 0
        model.servo_route_scan_state = 0
        model.servo_route_scan_progress = 0
        model.servo_bus_device_count = [0] * 4
        model.servo_route_bus = [0xFF] * MOTOR_COUNT
        model.servo_route_state = [0] * MOTOR_COUNT
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
        self._state_reducer.last_model = model
        return model

    def _push_rx_model(self, model: HandModel) -> None:
        """发布最新状态；邮箱已有旧状态时先丢旧再写新，禁止形成状态积压。"""
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

    def drain_ack_events(self) -> list[dict]:
        """排空 ACK 事件通道；命令等待仍由 AckTracker 的 Condition 独立完成。"""
        return self._ack_tracker.drain_events()

    def drain_safety_events(self) -> list[dict]:
        """排空安全事件通道，避免状态邮箱覆盖时遗漏规则证据。"""
        return self._safety_event_channel.drain()

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
            self._ack_tracker.reset(notify_waiters=True)
            try:
                self._transport.close()
            except Exception as close_exc:
                log_session("COMM", f"serial close after offline failed: {close_exc}")
            self._push_rx_model(self._build_offline_model())

    def _reset_rx_diag(self) -> None:
        self._rx_diag_last_print = time.time()
        self._rx_diag_total_packets = 0
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
            if pkt_type == 0x01:
                self._rx_diag_sensor_packets += 1

    def _maybe_print_rx_diag(self) -> None:
        now = time.time()
        elapsed = now - self._rx_diag_last_print
        if elapsed < TERMINAL_DIAG_INTERVAL_S:
            return
        total_rate = self._rx_diag_total_packets / elapsed
        sensor_rate = self._rx_diag_sensor_packets / elapsed
        byte_rate = self._rx_diag_bytes / elapsed
        diag_msg = (
            f"total={total_rate:.1f} pkt/s "
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
        while self.running and self._transport.is_open:
            try:
                chunk = self._transport.read_available()
                if chunk:
                    packets = self._frame_decoder.feed(chunk)
                    self._update_rx_diag(packets, len(chunk))
                    model = self._process_packets(packets)
                    if model:
                        self._push_rx_model(model)
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
        """???????????????? StateReducer ???"""
        sink = getattr(self, "recording_packet_sink", None)
        if sink is not None:
            sink("p4", packets)
        return self._state_reducer.process_packets(self, packets)

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
        # P4仍有其他遥测包到达时，在统一收尾阶段根据摘要时间判定触觉中继超时。
        # 链路在线与模块有效必须分开：即使15位均为0，刚收到摘要也说明CAN中继在线。
        if (
            getattr(model, "tactile_source", "") == TACTILE_SOURCE_P4_CAN_RELAY and
            bool(getattr(model, "tactile_online", False))
        ):
            last_rx = float(getattr(model, "tactile_last_rx_ts", 0.0) or 0.0)
            if last_rx <= 0.0 or (time.time() - last_rx) > TACTILE_P4_RELAY_OFFLINE_AFTER_S:
                model.tactile_online = False
                model.tactile_module_valid_mask = 0
                model.tactile_module_online = [False] * 15

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

    def invalidate_pending_direct_commands(self) -> int:
        """Discard old motion intentions, without touching recovery data."""
        with self.tx_queue.mutex:
            pending = self.tx_queue.queue
            kept = [item for item in pending if item != "start" and not (
                isinstance(item, tuple) and item[0] in (
                    "motor_pos_abs", "motor_pos_abs_one",
                    "motor_direct_release",
                )
            )]
            removed = len(pending) - len(kept)
            pending.clear()
            pending.extend(kept)
            self.tx_queue.not_full.notify_all()
            return removed

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
        return self._ack_tracker.wait_for(cmd, timeout_s, received_after)

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
        while self.running and self._transport.is_open:
            try:
                self._maybe_send_host_heartbeat()
                cmd = self.tx_queue.get(timeout=TX_QUEUE_GET_TIMEOUT)
                data = self._encode_command(cmd)
                if data:
                    self._transport.write(data)
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
        if not self._transport.is_open:
            return
        if not self._heartbeat_enabled:
            return
        data = build_host_heartbeat_cmd()
        self._transport.write(data)
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
        if isinstance(cmd, tuple) and len(cmd) == 5 and cmd[0] == "calib_load_limits":
            return build_calib_load_limits_cmd(
                cmd[1], cmd[2], cmd[3], cmd[4]
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
        if (
            isinstance(cmd, tuple) and len(cmd) == 2
            and cmd[0] == "servo_rebase_zero"
        ):
            return build_servo_rebase_zero_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 6 and cmd[0] == "motor_pos_abs":
            return build_motor_pos_abs_cmd(
                cmd[3], token=cmd[1], motor_mask=cmd[2], speed=cmd[4], acc=cmd[5])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_direct_release":
            return build_motor_direct_release_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 3 and cmd[0] == "manual_encoder_reference":
            return build_manual_encoder_reference_cmd(cmd[1], cmd[2])
        if cmd == "clear_manual_encoder_reference":
            return build_clear_manual_encoder_reference_cmd()
        if cmd == "query_manual_encoder_reference":
            return build_manual_encoder_query_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "state_sync":
            return build_state_sync_request_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "task_diag_subscribe":
            return build_task_diag_subscribe_cmd(bool(cmd[1]))
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "control_trace_subscribe":
            return build_control_trace_subscribe_cmd(bool(cmd[1]))
        if isinstance(cmd, tuple) and len(cmd) == 4 and cmd[0] == "manual_encoder_single":
            return build_manual_encoder_single_cmd(cmd[1], cmd[2], cmd[3])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "rope_init":
            return build_rope_init_cmd(cmd[1])
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
