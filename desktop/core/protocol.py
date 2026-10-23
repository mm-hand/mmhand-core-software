"""protocol.py - MMHand Desktop 模块契约。

职责：编码下行命令帧并集中暴露生成的协议常量。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
import struct
import json
from enum import Enum
from typing import List, Optional, Sequence, Tuple

# 协议编号、21/27 维度、固定 payload 长度和时间戳规则均由唯一 schema 生成。
# 本文件只保留字段编解码和业务数据模型，禁止重新定义同名协议常量。
from .generated_upper_protocol import *  # noqa: F403


class ControlMode(Enum):
    JOINT_ANGLE = 0
    DIRECT_MOTOR = 1


CALIB_WARN_NONE = 0x00
CALIB_WARN_AUTO_MIN_OFFSET = 0x01
CALIB_WARN_AUTO_MAX_SHORTFALL = 0x02
CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH = 0x04

TASK_RUNTIME_UPPER_COMM = 0
TASK_RUNTIME_STATE_MACHINE = 1
TASK_RUNTIME_CAN_COMM = 2
TASK_RUNTIME_SERVO_COMM = 3
TASK_RUNTIME_CONTROL = 4
TASK_RUNTIME_AUTO_CALIB = 5
TASK_RUNTIME_MONITOR = 6
TASK_RUNTIME_NAMES = {
    TASK_RUNTIME_UPPER_COMM: "UpperComm",
    TASK_RUNTIME_STATE_MACHINE: "StateMachine",
    TASK_RUNTIME_CAN_COMM: "CAN",
    TASK_RUNTIME_SERVO_COMM: "Servo",
    TASK_RUNTIME_CONTROL: "Control",
    TASK_RUNTIME_AUTO_CALIB: "AutoCalibration",
    TASK_RUNTIME_MONITOR: "TaskMonitor",
}

SYSTEM_STATE_UNCONTROLLABLE = 0
SYSTEM_STATE_IDLE = 1
SYSTEM_STATE_RUN_FEEDBACK = 2
SYSTEM_STATE_CALIBRATION_RUNNING = 4
SYSTEM_STATE_READY = 5
SYSTEM_STATE_RUN_DIRECT = 6
SYSTEM_STATE_ROPE_INIT_RUNNING = 8
SYSTEM_STATE_RECOVERY_REQUIRED = 9

ROPE_INIT_STATUS_IDLE = 0
ROPE_INIT_STATUS_RUNNING = 1
ROPE_INIT_STATUS_SUCCESS = 2
ROPE_INIT_STATUS_FAILED = 3

ROPE_INIT_REASON_NONE = 0
ROPE_INIT_REASON_TIMEOUT = 1
ROPE_INIT_REASON_FEEDBACK_STALE = 2
ROPE_INIT_REASON_SERVO_OFFLINE = 3
ROPE_INIT_REASON_MULTITURN_UNKNOWN = 4
ROPE_INIT_REASON_MANUAL_REQUIRED = 5
ROPE_INIT_REASON_AUTO_REQUIRED = 6
ROPE_INIT_REASON_TARGET_PUBLISH_FAILED = 7
ROPE_INIT_REASON_SYSTEM_FAULT = 8
ROPE_INIT_REASON_CANCELLED = 9

# RUN_DIRECT 结果由 SYSTEM_STATUS 中的 generation/reject_reason 异步确认。
# 数值与 P4 domain/SystemTypes.h 保持一致，不能重新编号。
DIRECT_REJECT_NONE = 0
DIRECT_REJECT_STATE = 1
DIRECT_REJECT_HARDWARE = 2
DIRECT_REJECT_MULTITURN = 3
DIRECT_REJECT_EVENT_QUEUE = 4
DIRECT_REJECT_STALE = 5
DIRECT_REJECT_FAULT = 6

# RUN_FEEDBACK admission is asynchronous: the command ACK only means that P4
# queued the request.  These fields report the eventual StateMachine result.
FEEDBACK_ADMISSION_IDLE = 0
FEEDBACK_ADMISSION_PENDING = 1
FEEDBACK_ADMISSION_ACCEPTED = 2
FEEDBACK_ADMISSION_REJECTED = 3

FEEDBACK_REJECT_NONE = 0
FEEDBACK_REJECT_MANUAL_REFERENCE = 1
FEEDBACK_REJECT_STATE = 2
FEEDBACK_REJECT_WORKSPACE = 3
FEEDBACK_REJECT_MULTITURN = 4
FEEDBACK_REJECT_ABS_LIMIT = 5
FEEDBACK_REJECT_MODE_SWITCH = 6
FEEDBACK_REJECT_EVENT_QUEUE = 7

SYSTEM_FAULT_CAN_OFFLINE = 1 << 0
SYSTEM_FAULT_JOINT16_DUAL = 1 << 1
SYSTEM_FAULT_OVERLOAD = 1 << 2
# Fault bitmap bit 3 is reserved by the P4 protocol.
SYSTEM_FAULT_SERVO_OFFLINE = 1 << 4
SYSTEM_FAULT_CALIBRATION = 1 << 5
SYSTEM_FAULT_MULTITURN_UNKNOWN = 1 << 6

CALIB_DIAG_REASON_NONE = 0
CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY = 1
CALIB_DIAG_REASON_ENCODER_ERROR = 5
CALIB_DIAG_REASON_SERVO_OFFLINE = 6
CALIB_DIAG_REASON_OVERLOAD = 8
CALIB_DIAG_REASON_ZERO_CROSS_INVALID = 13
CALIB_DIAG_REASON_WRAP_TOO_LARGE = 14
CALIB_DIAG_REASON_ABS_LIMIT_INVALID = 15
CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED = 16
CALIB_DIAG_REASON_FEEDBACK_LOST = 17
CALIB_DIAG_REASON_TASK_START_FAILED = 18
CALIB_DIAG_REASON_CONTROL_LOST = 20
CALIB_DIAG_REASON_HOLD_TARGET_FAILED = 21
CALIB_DIAG_REASON_JOINT_LIST_INVALID = 22
CALIB_DIAG_REASON_DUPLICATE_JOINT = 23
CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED = 24
CALIB_DIAG_REASON_JOINT_STEP_FAILED = 25
CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT = 26
CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS = 27
CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID = 28
CALIB_DIAG_REASON_RESULT_SPAN_INVALID = 29
CALIB_DIAG_REASON_RESULT_BUILD_FAILED = 30
CALIB_DIAG_REASON_RESULT_COMMIT_FAILED = 31
CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED = 32
CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT = 33
CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED = 34

CALIB_DIAG_PHASE_NONE = 0
CALIB_DIAG_PHASE_INIT = 1
CALIB_DIAG_PHASE_FIND_MAX = 2
CALIB_DIAG_PHASE_RELEASE = 3
CALIB_DIAG_PHASE_RETURN_ZERO = 4
CALIB_DIAG_PHASE_DUAL_PRELOAD = 5
CALIB_DIAG_PHASE_DUAL_EXCHANGE = 6
CALIB_DIAG_PHASE_SEQUENCE = 7

SERVO_TARGET_OWNER_NONE = 0
SERVO_TARGET_OWNER_CONTROL = 1
SERVO_TARGET_OWNER_CALIBRATION = 2

# 动态舵机路由状态；数值必须与 P4 ServoRouteState 保持一致。
SERVO_ROUTE_UNKNOWN = 0
SERVO_ROUTE_PROBING = 1
SERVO_ROUTE_ONLINE = 2
SERVO_ROUTE_OFFLINE = 3
SERVO_ROUTE_CONFLICT = 4
SERVO_ROUTE_BUS_OVERLOADED = 5

# 与协议无关的控制域配置。
DUAL_TENDON_JOINT_INDICES = (0, 4, 8, 12, 16, 20)
TENDON_LUT_ACTUATOR_COUNT = 2
TENDON_LUT_MAX_SAMPLES = 24
TACTILE_GROUP_COUNT = 5
TACTILE_SENSOR_PER_GROUP = 3
TACTILE_AXIS_COUNT = 3
TACTILE_SUMMARY_BYTES = TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_COUNT
TACTILE_TIP_TAXELS = 52
TACTILE_PAD_TAXELS = 25
TACTILE_SENSOR_TAXEL_COUNTS = (TACTILE_TIP_TAXELS, TACTILE_PAD_TAXELS, TACTILE_PAD_TAXELS)
TACTILE_FULL_BYTES = TACTILE_GROUP_COUNT * sum(TACTILE_SENSOR_TAXEL_COUNTS) * TACTILE_AXIS_COUNT
TACTILE_FULL_PACKET_MAGIC = b"T2"


def tendon_lut_field(result: dict, field_name: str, default=None):
    """读取当前唯一的 Tendon LUT 字段，不再兼容旧键名。"""
    return result.get(field_name, default)
HAND_PROFILE = "v2"
VALID_MOTOR_MASK = (1 << MOTOR_COUNT) - 1


def directed_encoder_delta(from_raw: int, to_raw: int, direction: int) -> Optional[int]:
    """Mirror P4 calibrationDirectedEncoderDelta for raw 14-bit endpoints."""
    if int(direction) not in (-1, 1):
        return None
    start = int(from_raw) & (ENCODER_RAW_MODULO - 1)
    end = int(to_raw) & (ENCODER_RAW_MODULO - 1)
    if int(direction) > 0:
        return (end - start) % ENCODER_RAW_MODULO
    return (start - end) % ENCODER_RAW_MODULO

CALIB_LOAD_THRESHOLD = 0.3
ERROR_VAL_FLAG = 0xFFFF

# Serial settings
# P4 运行期 USB-UART 波特率；必须与固件 BoardConfig.h 保持一致。
BAUDRATE = 921600
SERIAL_TIMEOUT = 0.1
# 连续状态是 latest-value 语义：积压旧状态没有价值，只会放大 UI 状态年龄。
RX_QUEUE_MAXSIZE = 1
TX_QUEUE_MAXSIZE = 100
RX_POLL_SLEEP = 0.005
TX_QUEUE_GET_TIMEOUT = 0.1


def _build_command_frame(cmd: int, payload: bytes = b"") -> bytes:
    """按 P4 下行帧格式编码命令，并用生成长度表阻止协议漂移。

    帧格式为 FE | LEN | CMD | PAYLOAD | FF；LEN 统计 CMD、PAYLOAD 和尾字节，
    不包含帧头与 LEN 自身。所有当前下行命令均为定长 payload。
    """
    payload = bytes(payload)
    # 下行命令全部是定长协议。这里使用同一份 schema 生成的长度表做最后校验，
    # 可在串口发送前发现 C++/Python 字段增删不同步，而不是等 P4 静默拒绝。
    expected_payload_bytes = COMMAND_PAYLOAD_LENGTHS.get(int(cmd))
    if expected_payload_bytes is None:
        raise ValueError(f"unknown upper command: 0x{int(cmd) & 0xFF:02X}")
    if len(payload) != expected_payload_bytes:
        raise ValueError(
            f"command 0x{int(cmd) & 0xFF:02X} payload length mismatch: "
            f"expected {expected_payload_bytes}, got {len(payload)}"
        )
    # Symmetric downstream frame: [0xFE][LEN][CMD][PAYLOAD][0xFF]
    # LEN = CMD byte + payload + tail byte.
    frame_len = 1 + len(payload) + 1
    return bytes([PROTOCOL_HEADER, frame_len & 0xFF, cmd & 0xFF]) + payload + bytes([PROTOCOL_TAIL])


def split_p4_timestamp(pkt_type: int, payload: bytes) -> Tuple[bytes, Optional[int]]:
    """从当前上行包尾剥离 P4 毫秒时间戳。

    新协议不再把上位机接收时间冒充下位机时间。应带时间戳的包若长度不足，
    返回空负载，让调用方拒绝损坏或旧格式数据包。
    """
    if int(pkt_type) not in P4_TRAILING_TIMESTAMP_PACKET_TYPES:
        return bytes(payload), None
    if len(payload) < P4_TIMESTAMP_BYTES:
        return b"", None
    return (
        bytes(payload[:-P4_TIMESTAMP_BYTES]),
        struct.unpack(">I", payload[-P4_TIMESTAMP_BYTES:])[0],
    )


def parse_sensor_packet(payload: bytes) -> Tuple[List[int], List[bool]]:
    """Parse 21 raw magnetic encoder counts from the firmware sensor packet."""
    angles: List[int] = []
    errors: List[bool] = []
    if len(payload) != ENCODER_COUNT * 2:
        return angles, errors

    for i in range(ENCODER_COUNT):
        raw_u16 = (payload[i * 2] << 8) | payload[i * 2 + 1]
        is_error = (raw_u16 == DISCONNECT_SENTINEL)
        errors.append(is_error)
        if is_error:
            angles.append(0)
        else:
            angles.append(struct.unpack(">h", payload[i * 2:i * 2 + 2])[0])
    return angles, errors


def parse_proto_ack(payload: bytes) -> Optional[Tuple[int, int, int]]:
    if len(payload) != 3:
        return None
    return payload[0], payload[1], payload[2]


def parse_system_status_packet(payload: bytes) -> Optional[tuple]:
    if len(payload) != 173:
        return None
    system_state = payload[0]
    control_enabled = payload[1]
    servo_target_owner = payload[2]
    control_mode = payload[3]
    calib_status = payload[4]
    system_fault_bitmap = struct.unpack('>I', payload[5:9])[0]
    state_transition_count = struct.unpack('>I', payload[9:13])[0]
    multi_turn_unknown_bitmap = struct.unpack('>I', payload[13:17])[0]
    active_motor_mask = struct.unpack('>I', payload[17:21])[0]
    active_joint_mask = struct.unpack('>I', payload[21:25])[0]
    require_can_feedback = payload[25]
    servo_online_bitmap = struct.unpack('>I', payload[26:30])[0]
    encoder_online_bitmap = struct.unpack('>I', payload[30:34])[0]
    can_link_online = payload[34]
    encoder_warn_bitmap = struct.unpack('>I', payload[35:39])[0]
    encoder_fault_bitmap = struct.unpack('>I', payload[39:43])[0]
    manual_reference_valid_bitmap = struct.unpack('>I', payload[43:47])[0]
    manual_reference_crc = struct.unpack('>I', payload[47:51])[0]
    manual_reference_revision = struct.unpack('>I', payload[51:55])[0]
    manual_reference_committed = payload[55]
    auto_calibration_valid_bitmap = struct.unpack('>I', payload[56:60])[0]
    manual_reference_commit_status = payload[60]
    rope_init_status = payload[61]
    rope_init_reason = payload[62]
    rope_init_sequence = struct.unpack('>I', payload[63:67])[0]
    rope_init_joint_mask = struct.unpack('>I', payload[67:71])[0]
    rope_init_timestamp_ms = struct.unpack('>I', payload[71:75])[0]
    pending_direct_generation = struct.unpack('>I', payload[75:79])[0]
    applied_direct_generation = struct.unpack('>I', payload[79:83])[0]
    direct_reject_reason = payload[83]
    calibration_warning_bitmap = struct.unpack('>I', payload[84:88])[0]
    calibration_auto_min_offset_counts = [struct.unpack('>h', payload[88 + 2 * i:90 + 2 * i])[0] for i in range(ENCODER_COUNT)]
    feedback_admission_sequence = struct.unpack('>I', payload[130:134])[0]
    feedback_admission_status = payload[134]
    feedback_reject_reason = payload[135]
    feedback_collision_hold_bitmap = struct.unpack('>I', payload[136:140])[0]
    feedback_collision_transition_count = struct.unpack('>I', payload[140:144])[0]
    feedback_collision_armed_bitmap = struct.unpack('>I', payload[144:148])[0]
    servo_turn_overflow_bitmap = struct.unpack('>I', payload[148:152])[0]
    servo_reset_zero_request_token = struct.unpack('>I', payload[152:156])[0]
    servo_reset_zero_result_token = struct.unpack('>I', payload[156:160])[0]
    servo_reset_zero_success_mask = struct.unpack('>I', payload[160:164])[0]
    servo_reset_zero_failed_mask = struct.unpack('>I', payload[164:168])[0]
    servo_reset_zero_invalidated_joint_mask = struct.unpack('>I', payload[168:172])[0]
    servo_reset_zero_status = payload[172]
    return (
        system_state,
        control_enabled,
        servo_target_owner,
        control_mode,
        calib_status,
        system_fault_bitmap,
        state_transition_count,
        multi_turn_unknown_bitmap,
        active_motor_mask,
        active_joint_mask,
        require_can_feedback,
        True,
        servo_online_bitmap,
        encoder_online_bitmap,
        can_link_online,
        True,
        encoder_warn_bitmap,
        encoder_fault_bitmap,
        True,
        manual_reference_valid_bitmap,
        manual_reference_crc,
        manual_reference_revision,
        manual_reference_committed,
        True,
        auto_calibration_valid_bitmap,
        manual_reference_commit_status,
        True,
        rope_init_status,
        rope_init_reason,
        rope_init_sequence,
        rope_init_joint_mask,
        rope_init_timestamp_ms,
        pending_direct_generation,
        applied_direct_generation,
        direct_reject_reason,
        True,
        servo_turn_overflow_bitmap,
        servo_reset_zero_request_token,
        servo_reset_zero_result_token,
        servo_reset_zero_success_mask,
        servo_reset_zero_failed_mask,
        servo_reset_zero_invalidated_joint_mask,
        servo_reset_zero_status,
        True,
        feedback_admission_sequence,
        feedback_admission_status,
        feedback_reject_reason,
        True,
        feedback_collision_armed_bitmap,
        feedback_collision_hold_bitmap,
        feedback_collision_transition_count,
        True,
        calibration_warning_bitmap,
        calibration_auto_min_offset_counts,
        True,
    )


def parse_motor_abs_limit_packet(payload: bytes) -> Optional[Tuple[int, List[int], List[int]]]:
    """解析标定 ABS 范围；运行边界由 LUT 决定，不再传输 LOCK 状态。"""
    required = 4 + MOTOR_COUNT * 8
    if len(payload) < required:
        return None
    valid_bitmap = struct.unpack(">I", payload[:4])[0]
    offset = 4
    mins: List[int] = []
    maxs: List[int] = []
    for _ in range(MOTOR_COUNT):
        mins.append(struct.unpack(">i", payload[offset:offset + 4])[0])
        maxs.append(struct.unpack(">i", payload[offset + 4:offset + 8])[0])
        offset += 8
    return valid_bitmap, mins, maxs


def parse_servo_logical_range_packet(
    payload: bytes,
) -> Optional[Tuple[int, List[int], List[int]]]:
    """解析27路舵机当前上电会话的独立逻辑行程。"""
    required = 4 + MOTOR_COUNT * 8
    if len(payload) < required:
        return None
    valid_bitmap = struct.unpack(">I", payload[:4])[0]
    offset = 4
    mins: List[int] = []
    maxs: List[int] = []
    for _ in range(MOTOR_COUNT):
        mins.append(struct.unpack(">i", payload[offset:offset + 4])[0])
        maxs.append(struct.unpack(">i", payload[offset + 4:offset + 8])[0])
        offset += 8
    return valid_bitmap, mins, maxs


def parse_manual_encoder_reference_packet(payload: bytes) -> Optional[dict]:
    """Parse only the current v3 Manual snapshot, including all encoder directions."""
    if len(payload) != 124 or payload[0] != 3:
        return None
    valid_bitmap, reference_crc, revision = struct.unpack_from('>III', payload, 1)
    auto_valid_bitmap = struct.unpack_from('>I', payload, 14)[0]
    directions = list(struct.unpack_from('>21b', payload, 103))
    if any(value not in (-1, 1) for value in directions):
        return None
    endpoints = struct.unpack_from('>42H', payload, 19)
    return dict(version=3, valid_bitmap=valid_bitmap, reference_crc=reference_crc,
                revision=revision, committed=bool(payload[13]),
                auto_valid_bitmap=auto_valid_bitmap, commit_status=payload[18],
                min_raw=list(endpoints[::2]), max_raw=list(endpoints[1::2]),
                encoder_directions=directions, has_encoder_directions=True)


def _unpack_float_be(data: bytes) -> float:
    return struct.unpack(">f", data)[0]


def parse_joint_calib_snapshot_packet(payload: bytes) -> Optional[dict]:
    """解析当前 V4 快照；P4 上行体在装配告警后附带设备时间戳。"""
    if len(payload) != 202:
        return None
    idx = 0
    joint = payload[idx]
    idx += 1
    if joint >= ENCODER_COUNT:
        return None
    result = {'joint': int(joint), 'success': bool(payload[idx])}
    idx += 1
    result['angleMax'] = _unpack_float_be(payload[idx:idx + 4])
    idx += 4
    for key in ('startEncoder', 'tightEncoder', 'zeroEncoder', 'encoderSpan'):
        result[key] = struct.unpack('>i', payload[idx:idx + 4])[0]
        idx += 4
    for key in ('startServoAbs', 'tightServoAbs', 'zeroServoAbs'):
        result[key] = struct.unpack('>i', payload[idx:idx + 4])[0]
        idx += 4
    for key in ('startDeg', 'tightDeg', 'totalDeg'):
        result[key] = _unpack_float_be(payload[idx:idx + 4])
        idx += 4
    result['scaled'] = bool(payload[idx])
    idx += 1
    result['encoderDirection'] = -1 if struct.unpack('>b', payload[idx:idx + 1])[0] < 0 else 1
    idx += 1
    result['tendonLutValid'] = bool(payload[idx])
    idx += 1
    result['tendonLutCount'] = int(payload[idx])
    idx += 1
    result['tendonLutMotor'] = [int(payload[idx]), int(payload[idx + 1])]
    idx += 2
    result['tendonLutAngleStartDeg'] = _unpack_float_be(payload[idx:idx + 4])
    idx += 4
    result['tendonLutAngleStepDeg'] = _unpack_float_be(payload[idx:idx + 4])
    idx += 4
    motor_abs: List[List[int]] = []
    for _actuator in range(TENDON_LUT_ACTUATOR_COUNT):
        anchor = struct.unpack('>i', payload[idx:idx + 4])[0]
        idx += 4
        row: List[int] = []
        for _sample in range(TENDON_LUT_MAX_SAMPLES):
            encoded = struct.unpack('>h', payload[idx:idx + 2])[0]
            idx += 2
            row.append(anchor + encoded)
        motor_abs.append(row)
    result['tendonLutMotorAbs'] = motor_abs
    result['mappingVersion'] = int(payload[idx])
    idx += 1
    result['angleMin'] = _unpack_float_be(payload[idx:idx + 4])
    idx += 4
    result['urdfConfigCrc'] = struct.unpack('>I', payload[idx:idx + 4])[0]
    idx += 4
    result['autoMinRaw'] = struct.unpack('>H', payload[idx:idx + 2])[0]
    idx += 2
    result['autoMaxRaw'] = struct.unpack('>H', payload[idx:idx + 2])[0]
    idx += 2
    result['manualReferenceCrc'] = struct.unpack('>I', payload[idx:idx + 4])[0]
    idx += 4
    # V4 wire 仍携带 currentMinDeg/currentMaxDeg 两个 float 保留槽；host 不再维护
    # current 概念，读取后立即跳过，工作区统一使用静态 URDF Min/Max。保持 idx
    # 前进 8 字节，后面的装配告警字段偏移和 202-byte payload 长度不变。
    _unpack_float_be(payload[idx:idx + 4])
    _unpack_float_be(payload[idx + 4:idx + 8])
    idx += 8
    result['hasManualScaledV4Fields'] = True
    result['assemblyWarningReason'] = CALIB_WARN_NONE
    result['autoMinOffsetCounts'] = 0
    result['autoMaxShortfallCounts'] = 0
    result['assemblyAutoSpanCounts'] = 0
    result['assemblyManualSpanCounts'] = 0
    result['assemblyUrdfSpanCounts'] = 0
    result['assemblySpanDifferenceCounts'] = 0
    result['hasAssemblyWarningFields'] = False
    result['hasAssemblySpanComparisonFields'] = False
    result['assemblyWarningReason'] = int(payload[idx])
    idx += 1
    result['autoMinOffsetCounts'] = struct.unpack('>h', payload[idx:idx + 2])[0]
    idx += 2
    result['autoMaxShortfallCounts'] = struct.unpack('>h', payload[idx:idx + 2])[0]
    idx += 2
    result['assemblyAutoSpanCounts'] = struct.unpack('>H', payload[idx:idx + 2])[0]
    idx += 2
    result['assemblyManualSpanCounts'] = struct.unpack('>H', payload[idx:idx + 2])[0]
    idx += 2
    result['assemblyUrdfSpanCounts'] = struct.unpack('>H', payload[idx:idx + 2])[0]
    idx += 2
    result['assemblySpanDifferenceCounts'] = struct.unpack('>H', payload[idx:idx + 2])[0]
    idx += 2
    result['hasAssemblyWarningFields'] = True
    result['hasAssemblySpanComparisonFields'] = True
    result['deviceTimestampMs'] = None
    return result


def parse_calib_diag_packet(payload: bytes) -> Optional[dict]:
    # 当前协议固定携带根因、诊断序号以及本轮成功/失败位图。
    # 拒绝旧短包，避免把缺失字段误解释为“没有成功关节”。
    if len(payload) != 37:
        return None
    idx = 0
    result = {
        "valid": bool(payload[idx]),
        "reason": int(payload[idx + 1]),
        "phase": int(payload[idx + 2]),
        "joint": int(payload[idx + 3]),
        "motor": int(payload[idx + 4]),
    }
    idx = 5
    result["valueA"] = struct.unpack(">i", payload[idx:idx + 4])[0]
    idx += 4
    result["valueB"] = struct.unpack(">i", payload[idx:idx + 4])[0]
    idx += 4
    result["valueC"] = struct.unpack(">i", payload[idx:idx + 4])[0]
    idx += 4
    result["bitmap"] = struct.unpack(">I", payload[idx:idx + 4])[0]
    idx += 4
    result["timestamp_ms"] = struct.unpack(">I", payload[idx:idx + 4])[0]
    idx += 4
    result["sequence"] = struct.unpack(">I", payload[idx:idx + 4])[0]
    idx += 4
    result["completed_joint_mask"] = struct.unpack(">I", payload[idx:idx + 4])[0]
    idx += 4
    result["failed_joint_mask"] = struct.unpack(">I", payload[idx:idx + 4])[0]
    return result


def parse_task_diag_packet(payload: bytes) -> Optional[dict]:
    """解析 P4 一秒窗口任务诊断包 v4；区分固定周期和事件提前唤醒。"""
    if len(payload) != 191 or payload[0] != 4:
        return None
    version = 4
    idx = 1
    task_id = payload[idx]
    idx += 1
    active = payload[idx] != 0
    idx += 1
    priority = payload[idx]
    idx += 1
    core_raw = payload[idx]
    core_id = core_raw - 256 if core_raw >= 128 else core_raw
    idx += 1

    def read_u32() -> int:
        nonlocal idx
        value = struct.unpack(">I", payload[idx:idx + 4])[0]
        idx += 4
        return value

    def read_u16() -> int:
        nonlocal idx
        value = struct.unpack(">H", payload[idx:idx + 2])[0]
        idx += 2
        return value

    result = {
        "version": version,
        "task_id": task_id,
        "task_name": TASK_RUNTIME_NAMES.get(task_id, f"Task{task_id}"),
        "active": active,
        "priority": priority,
        "core_id": core_id,
        "stack_allocated_bytes": read_u32(),
    }
    result["stack_current_free_bytes"] = read_u32()
    result["stack_min_free_bytes"] = read_u32()
    result.update({
        "loop_count": read_u32(),
        "max_gap_us": read_u32(),
        "max_exec_us": read_u32(),
        "window_sequence": read_u32(),
        "window_duration_us": read_u32(),
        "window_loop_count": read_u32(),
        "window_periodic_count": read_u32(),
        "window_event_count": read_u32(),
        "avg_exec_us": read_u32(),
        "p95_exec_us": read_u32(),
        "p99_exec_us": read_u32(),
        "window_max_exec_us": read_u32(),
        "cpu_permille": read_u16(),
        "overrun_count": read_u32(),
        "consecutive_overruns": read_u16(),
        "max_consecutive_overruns": read_u16(),
        "core0_idle_permille": read_u16(),
        "core1_idle_permille": read_u16(),
        "system_status_max_gap_ms": read_u32(),
        "stop_last_latency_us": read_u32(),
        "stop_max_latency_us": read_u32(),
        "device_timestamp_ms": read_u32(),
    })
    io_names = (
        "io_sample_count",
        "io_requested_count",
        "io_succeeded_count",
        "io_avg_us",
        "io_p95_us",
        "io_p99_us",
    )
    for name in io_names:
        result[name] = [read_u32() for _ in range(4)]
    return result


def parse_control_components_packet(payload: bytes) -> Optional[dict]:
    """Same-cycle raw PI/reversal components, before final output constraints."""
    if len(payload) != 210 or payload[0] != 1 or not 1 <= payload[5] <= 100:
        return None
    return dict(sequence=struct.unpack_from('>I', payload, 1)[0], divisor=payload[5],
                proportional_counts=list(struct.unpack_from('>27h', payload, 6)),
                integral_counts=list(struct.unpack_from('>27h', payload, 60)),
                reversal_counts=list(struct.unpack_from('>27h', payload, 114)),
                slow_deg=[None if value == 32767 else value / 100.0
                          for value in struct.unpack_from('>21h', payload, 168)])


def parse_control_detail_packet(payload: bytes) -> Optional[dict]:
    if len(payload) != 240:
        return None
    result = dict(zip(('sequence', 'servo_timestamp', 'control_dt_us', 'servo_valid_mask',
                       'recovery_applied_mask', 'p4_trace_dropped'), struct.unpack_from('>6I', payload)))
    result.update(actual_abs=list(struct.unpack_from('>27i', payload, 24)),
                  pid_counts=list(struct.unpack_from('>27h', payload, 132)),
                  recovery_counts=list(struct.unpack_from('>27h', payload, 186)))
    return result


def parse_control_trace_packet(payload: bytes) -> Optional[dict]:
    """解析同一 P4 控制周期采用的目标角、反馈角和最终舵机 CMD。"""
    header_size = 7 * 4
    angle_size = ENCODER_COUNT * 2
    motor_command_size = MOTOR_COUNT * 4
    if len(payload) != header_size + angle_size * 2 + motor_command_size:
        return None
    (
        sequence,
        target_token,
        target_accepted_ms,
        encoder_timestamp_ms,
        active_joint_mask,
        valid_joint_mask,
        motor_command_mask,
    ) = struct.unpack_from(">IIIIIII", payload, 0)
    target_raw = struct.unpack_from(f">{ENCODER_COUNT}h", payload, header_size)
    actual_raw = struct.unpack_from(
        f">{ENCODER_COUNT}h", payload, header_size + angle_size
    )
    motor_command_abs = struct.unpack_from(
        f">{MOTOR_COUNT}i", payload, header_size + angle_size * 2
    )
    return {
        "sequence": int(sequence),
        "target_token": int(target_token),
        "target_accepted_ms": int(target_accepted_ms),
        "encoder_timestamp_ms": int(encoder_timestamp_ms),
        "active_joint_mask": int(active_joint_mask),
        "valid_joint_mask": int(valid_joint_mask),
        "motor_command_mask": int(motor_command_mask),
        "target_deg": [float(value) / 100.0 for value in target_raw],
        "actual_deg": [float(value) / 100.0 for value in actual_raw],
        "motor_command_abs": [int(value) for value in motor_command_abs],
    }


def parse_product_fingerprint_packet(payload: bytes) -> Optional[dict]:
    """解析 P4 当前固件、参数、硬件映射和 URDF 的可复现实验指纹。"""
    if len(payload) != 36:
        return None
    values = struct.unpack(">BBBBIIIIIIQ", payload)
    return {
        "protocol_version": int(values[3]),
        "device_id": f"{values[10]:016X}" if values[10] else "",
        "schema_version": int(values[0]),
        "feedback_mapping_version": int(values[1]),
        "manual_scaled_mapping_version": int(values[2]),
        "control_model_crc": int(values[4]),
        "visual_model_crc": int(values[5]),
        "hardware_map_crc": int(values[6]),
        "parameter_crc": int(values[7]),
        "firmware_source_crc": int(values[8]),
        "structure_commit_crc": int(values[9]),
    }


def parse_servo_route_status_packet(payload: bytes) -> Optional[dict]:
    """解析 27 台舵机在四路 BUS 上的动态发现结果。

    P4 只上报已经完成提交的路由快照。扫描过程中 ``controllable_bitmap``
    会被清零，因此 UI 不得退回使用旧的固定 BUS 映射。
    """
    if len(payload) != 77:
        return None
    generation, valid_bitmap, controllable_bitmap, conflict_bitmap = struct.unpack(
        ">IIII", payload[:16]
    )
    return {
        "generation": int(generation),
        "valid_bitmap": int(valid_bitmap),
        "controllable_bitmap": int(controllable_bitmap),
        "conflict_bitmap": int(conflict_bitmap),
        "bus_overload_mask": int(payload[16]),
        "scan_state": int(payload[17]),
        "scan_progress": int(payload[18]),
        "bus_device_count": [int(value) for value in payload[19:23]],
        "route_bus": [int(value) for value in payload[23:50]],
        "route_state": [int(value) for value in payload[50:77]],
    }


def parse_safety_event_packet(payload: bytes) -> Optional[dict]:
    """解析 P4 去重后的结构化安全事件。

    ``device_timestamp_ms`` 由公共包路由器剥离，因此这里固定解析 22 字节业务字段。
    """
    if len(payload) != 22:
        return None
    (
        sequence,
        rule_id,
        motor_channel,
        joint_index,
        system_state,
        owner,
        requested_value,
        observed_value,
        overflow_count,
    ) = struct.unpack(">IHBBBBiiI", payload)
    return {
        "sequence": int(sequence),
        "rule_id": int(rule_id),
        "motor_channel": int(motor_channel),
        "joint_index": int(joint_index),
        "system_state": int(system_state),
        "owner": int(owner),
        "requested_value": int(requested_value),
        "observed_value": int(observed_value),
        "overflow_count": int(overflow_count),
    }


def parse_servo_telem_packet(
    payload: bytes,
) -> Optional[
    Tuple[List[int], List[int], List[int], List[int], List[int]]
]:
    # 每路固定 8 字节；在线状态不在 100 Hz 遥测中重复发送。
    if len(payload) != MOTOR_COUNT * 8:
        return None

    channel_count = MOTOR_COUNT

    speeds: List[int] = []
    loads: List[int] = []
    currents: List[int] = []
    volts: List[int] = []
    temps: List[int] = []

    for i in range(channel_count):
        base = i * 8
        speeds.append(struct.unpack(">h", payload[base:base + 2])[0])
        loads.append(struct.unpack(">h", payload[base + 2:base + 4])[0])
        currents.append(struct.unpack(">h", payload[base + 4:base + 6])[0])
        volts.append(payload[base + 6])
        temps.append(payload[base + 7])
    return speeds, loads, currents, volts, temps


def parse_servo_position_packet(payload: bytes) -> Optional[Tuple[List[int], List[int]]]:
    """Parse one merged packet: 27 raw int16 first, then 27 logical ABS int32."""
    if len(payload) != MOTOR_COUNT * 6:
        return None
    raw_positions = [
        struct.unpack(">h", payload[2 * i:2 * i + 2])[0]
        for i in range(MOTOR_COUNT)
    ]
    base = MOTOR_COUNT * 2
    angles = [
        struct.unpack(">i", payload[base + 4 * i:base + 4 * i + 4])[0]
        for i in range(MOTOR_COUNT)
    ]
    return raw_positions, angles


def parse_multiturn_restore_result_packet(payload: bytes) -> Optional[Tuple[int, int, int]]:
    """解析多圈恢复的异步最终 token、成功位图和拒绝位图。"""
    if len(payload) != 12:
        return None
    token, restored_mask, rejected_mask = struct.unpack(">III", payload)
    valid_mask = (1 << MOTOR_COUNT) - 1
    if (restored_mask | rejected_mask) & ~valid_mask or restored_mask & rejected_mask:
        return None
    return token, restored_mask, rejected_mask


def parse_servo_rebase_zero_result_packet(
    payload: bytes,
) -> Optional[Tuple[int, int, int]]:
    """解析硬件 ABS 设零结果；保留原帧长度，末尾保留区必须全零。"""
    if len(payload) != 12 + MOTOR_COUNT * 4:
        return None
    token, success_mask, failed_mask = struct.unpack_from(">III", payload)
    valid_mask = (1 << MOTOR_COUNT) - 1
    if (success_mask | failed_mask) & ~valid_mask or success_mask & failed_mask:
        return None
    if any(payload[12:]):
        return None
    return token, success_mask, failed_mask


def parse_direct_target_result_packet(
    payload: bytes,
) -> Optional[Tuple[int, int, int, int, int]]:
    """解析通用直控同步写结果；APPLIED 与物理到位是两个独立条件。"""
    if len(payload) != 17:
        return None
    token, requested, applied, rejected, status = struct.unpack(">IIIIB", payload)
    valid_mask = (1 << MOTOR_COUNT) - 1
    if (requested | applied | rejected) & ~valid_mask:
        return None
    if applied & rejected or (applied | rejected) != requested:
        return None
    if requested == 0:
        return (token, requested, applied, rejected, status) if not (token or status) else None
    if status not in (2, 3):
        return None
    if status == 2 and (applied != requested or rejected):
        return None
    if status == 3 and not rejected:
        return None
    return token, requested, applied, rejected, status


def parse_adaptive_kp_packet(payload: bytes) -> Optional[dict]:
    """Parse 21 per-joint adaptive Kp scales and effective gains (int16 x1000)."""
    if len(payload) != 1 + 4 * ENCODER_COUNT:
        return None
    scale = [
        struct.unpack(">h", payload[1 + 2 * i:3 + 2 * i])[0] / 1000.0
        for i in range(ENCODER_COUNT)
    ]
    base = 1 + 2 * ENCODER_COUNT
    effective = [
        struct.unpack(">h", payload[base + 2 * i:base + 2 * i + 2])[0] / 1000.0
        for i in range(ENCODER_COUNT)
    ]
    return {"enabled": bool(payload[0]), "scale": scale, "effective": effective}


def parse_fault_status_packet(payload: bytes) -> Optional[Tuple[int, List[Optional[int]]]]:
    if len(payload) != 4 + MOTOR_COUNT * 2:
        return None
    fault_bitmap = struct.unpack(">I", payload[:4])[0]
    trigger_loads: List[Optional[int]] = [None] * MOTOR_COUNT
    offset = 4
    for i in range(MOTOR_COUNT):
        trigger_loads[i] = struct.unpack(">h", payload[offset:offset + 2])[0]
        offset += 2
    return fault_bitmap, trigger_loads


def parse_tactile_packet(
    payload: bytes,
) -> Optional[Tuple[int, List[List[List[int]]], int]]:
    """解析统一触觉摘要：序号、45字节三维力和15位模块有效位图。"""
    expected_len = 1 + TACTILE_SUMMARY_BYTES + 2
    if len(payload) != expected_len:
        return None

    seq = payload[0]
    data = payload[1:1 + TACTILE_SUMMARY_BYTES]
    mask_offset = 1 + TACTILE_SUMMARY_BYTES
    module_valid_mask = struct.unpack(">H", payload[mask_offset:mask_offset + 2])[0]

    out: List[List[List[int]]] = []
    cursor = 0
    for _group in range(TACTILE_GROUP_COUNT):
        group_data: List[List[int]] = []
        for _sensor in range(TACTILE_SENSOR_PER_GROUP):
            fx = struct.unpack("b", bytes([data[cursor]]))[0]
            fy = struct.unpack("b", bytes([data[cursor + 1]]))[0]
            fz = int(data[cursor + 2])
            group_data.append([fx, fy, fz])
            cursor += TACTILE_AXIS_COUNT
        out.append(group_data)
    return seq, out, int(module_valid_mask)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= (int(byte) & 0xFF) << 8
        for _bit in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def parse_tactile_full_fragment(
    payload: bytes,
) -> Optional[Tuple[int, int, int, bytes, Optional[int], Optional[int]]]:
    if len(payload) >= 9 and payload[:2] == TACTILE_FULL_PACKET_MAGIC:
        seq = payload[2]
        summary_seq = payload[3]
        frag_idx = payload[4]
        frag_count = payload[5]
        chunk_len = payload[6]
        full_crc = payload[7] | (payload[8] << 8)
        if frag_count <= 0 or frag_idx >= frag_count:
            return None
        if len(payload) != 9 + chunk_len:
            return None
        return (
            seq,
            frag_idx,
            frag_count,
            bytes(payload[9:9 + chunk_len]),
            summary_seq,
            full_crc,
        )

    return None


def build_start_cmd() -> bytes:
    return _build_command_frame(CMD_START)


def build_stop_cmd() -> bytes:
    return _build_command_frame(CMD_STOP)


def build_reset_cmd() -> bytes:
    return _build_command_frame(CMD_RESET)


def build_active_scope_cmd(active_motor_mask: int, active_joint_mask: int, require_can_feedback: bool) -> bytes:
    payload = struct.pack(
        ">IIB",
        int(active_motor_mask) & 0xFFFFFFFF,
        int(active_joint_mask) & 0xFFFFFFFF,
        1 if require_can_feedback else 0,
    )
    return _build_command_frame(CMD_ACTIVE_SCOPE, payload)


def build_protocol_hello_cmd() -> bytes:
    """发送本地生成的版本和模型契约；握手不包含运动授权。"""
    from desktop.paths import PRODUCT_FINGERPRINT_PATH
    fingerprint = json.loads(PRODUCT_FINGERPRINT_PATH.read_text(encoding="utf-8"))
    payload = struct.pack(
        ">BII", HOST_PROTOCOL_VERSION,
        int(fingerprint["control_model_crc"], 16),
        int(fingerprint["hardware_map_crc"], 16),
    )
    return _build_command_frame(CMD_PROTOCOL_HELLO, payload)


def build_angle_cmd(angles: List[float], joint_mask: int | None = None) -> bytes:
    """Build the one angle command: little-endian mask followed by 21 floats.

    The mask only selects which active joints enter PID; unset joints keep their
    current servo ABS. ``None`` means all 21 joints.
    """
    if len(angles) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} joint angles")
    valid_mask = (1 << ENCODER_COUNT) - 1 if joint_mask is None else int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
    if valid_mask == 0:
        raise ValueError("angle command requires a non-zero joint mask")
    payload = struct.pack("<I", valid_mask)
    payload += struct.pack(f"<{ENCODER_COUNT}f", *angles)
    return _build_command_frame(CMD_ANGLE_CTRL, payload)


def build_masked_angle_cmd(angles: List[float], joint_mask: int) -> bytes:
    return build_angle_cmd(angles, joint_mask)


def build_run_feedback_cmd() -> bytes:
    return _build_command_frame(CMD_RUN_FEEDBACK, b"")






def build_restore_multiturn_cmd(active_motor_mask: int, motor_abs: List[int]) -> bytes:
    if len(motor_abs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor absolute values")
    payload = bytearray()
    payload.extend(struct.pack(">I", int(active_motor_mask) & 0xFFFFFFFF))
    for value in motor_abs:
        payload.extend(struct.pack(">i", int(value)))
    return _build_command_frame(CMD_RESTORE_MULTITURN, bytes(payload))


def _pack_float_be(value: float) -> bytes:
    return struct.pack(">f", float(value))


def build_calib_load_limits_cmd(
    update_bitmap: int,
    valid_bitmap: int,
    mins: List[int],
    maxs: List[int],
) -> bytes:
    """构造按电机作用域更新的 ABS Limit 命令，未选电机保持 P4 原值。"""
    if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor limit values")
    payload = bytearray()
    payload.extend(
        struct.pack(
            ">II",
            int(update_bitmap) & 0xFFFFFFFF,
            int(valid_bitmap) & 0xFFFFFFFF,
        )
    )
    for i in range(MOTOR_COUNT):
        payload.extend(struct.pack(">i", int(mins[i])))
        payload.extend(struct.pack(">i", int(maxs[i])))
    return _build_command_frame(CMD_CALIB_LOAD_LIMITS, bytes(payload))


def build_calib_load_joint_cmd(result: dict) -> bytes:
    joint = int(result.get("joint", -1))
    if joint < 0 or joint >= ENCODER_COUNT:
        raise ValueError(f"invalid joint index {joint}")
    if int(result.get("mappingVersion", 0)) != 4:
        raise ValueError(
            f"J{joint:02d} mapping version mismatch; current mapping v4 is required"
        )
    payload = bytearray()
    payload.append(joint & 0xFF)
    payload.append(1 if bool(result.get("success", False)) else 0)
    payload.extend(_pack_float_be(float(result.get("angleMax", 0.0))))
    for key in ("startEncoder", "tightEncoder", "zeroEncoder", "encoderSpan"):
        payload.extend(struct.pack(">i", int(result.get(key, 0))))
    for key in ("startServoAbs", "tightServoAbs", "zeroServoAbs"):
        payload.extend(struct.pack(">i", int(result.get(key, 0))))
    for key in ("startDeg", "tightDeg", "totalDeg"):
        payload.extend(_pack_float_be(float(result.get(key, 0.0))))
    payload.append(1 if bool(result.get("scaled", False)) else 0)
    direction = -1 if int(result.get("encoderDirection", 1)) < 0 else 1
    payload.extend(struct.pack(">b", direction))
    payload.append(1 if bool(tendon_lut_field(result, "tendonLutValid", False)) else 0)
    payload.append(int(tendon_lut_field(result, "tendonLutCount", 0)) & 0xFF)
    motors = list(tendon_lut_field(result, "tendonLutMotor", [0xFF, 0xFF]))
    while len(motors) < TENDON_LUT_ACTUATOR_COUNT:
        motors.append(0xFF)
    for i in range(TENDON_LUT_ACTUATOR_COUNT):
        payload.append(int(motors[i]) & 0xFF)
    payload.extend(_pack_float_be(float(tendon_lut_field(result, "tendonLutAngleStartDeg", 0.0))))
    payload.extend(_pack_float_be(float(tendon_lut_field(result, "tendonLutAngleStepDeg", 0.0))))
    motor_abs = tendon_lut_field(result, "tendonLutMotorAbs", [])
    for actuator in range(TENDON_LUT_ACTUATOR_COUNT):
        row = motor_abs[actuator] if actuator < len(motor_abs) else []
        valid_count = min(
            max(0, int(tendon_lut_field(result, "tendonLutCount", 0))),
            TENDON_LUT_MAX_SAMPLES,
            len(row),
        )
        if valid_count > 0:
            valid_values = [int(row[i]) for i in range(valid_count)]
            anchor = min(valid_values) + (
                max(valid_values) - min(valid_values)
            ) // 2
        else:
            anchor = 0
        payload.extend(struct.pack(">i", anchor))
        for sample in range(TENDON_LUT_MAX_SAMPLES):
            delta = int(row[sample]) - anchor if sample < valid_count else 0
            if delta < -32768 or delta > 32767:
                raise ValueError(
                    f"J{joint:02d} tendon LUT actuator {actuator} "
                    f"delta {delta} exceeds int16 compact range"
                )
            payload.extend(struct.pack(">h", delta))
    payload.append(int(result.get("mappingVersion", 0)) & 0xFF)
    payload.extend(_pack_float_be(float(result.get("angleMin", 0.0))))
    payload.extend(struct.pack(">I", int(result.get("urdfConfigCrc", 0)) & 0xFFFFFFFF))
    auto_min_raw = int(result.get("autoMinRaw", -1))
    auto_max_raw = int(result.get("autoMaxRaw", -1))
    if not 0 <= auto_min_raw < 16384 or not 0 <= auto_max_raw < 16384:
        raise ValueError(f"J{joint:02d} Auto raw must be in [0, 16383]")
    start_oriented = int(result.get("startEncoder", 0)) % 16384
    tight_oriented = int(result.get("tightEncoder", 0)) % 16384
    expected_auto_min_raw = (
        (-start_oriented) % 16384 if direction < 0 else start_oriented
    )
    expected_auto_max_raw = (
        (-tight_oriented) % 16384 if direction < 0 else tight_oriented
    )
    if (
        auto_min_raw != expected_auto_min_raw
        or auto_max_raw != expected_auto_max_raw
    ):
        raise ValueError(
            f"J{joint:02d} Auto raw endpoint contract mismatch: "
            f"snapshot={auto_min_raw}->{auto_max_raw}, "
            f"expected={expected_auto_min_raw}->{expected_auto_max_raw}; "
            "recalibration required"
        )
    payload.extend(struct.pack(">H", auto_min_raw))
    payload.extend(struct.pack(">H", auto_max_raw))
    payload.extend(struct.pack(">I", int(result.get("manualReferenceCrc", 0)) & 0xFFFFFFFF))
    # 下行仍写 8 字节 currentMinDeg/currentMaxDeg 保留槽（固定 0.0）；host 已删除
    # current 概念，保留槽只为维持 CMD_CALIB_LOAD_JOINT 的 189-byte wire 兼容。
    payload.extend(_pack_float_be(0.0))
    payload.extend(_pack_float_be(0.0))
    return _build_command_frame(CMD_CALIB_LOAD_JOINT, bytes(payload))


def build_host_heartbeat_cmd() -> bytes:
    """界面打开时维持下位机 host-online 计时器。"""
    return _build_command_frame(CMD_HOST_HEARTBEAT, b"")


def build_clear_calibration_cmd() -> bytes:
    return _build_command_frame(CMD_CLEAR_CALIBRATION, b"")


def build_clear_calibration_scope_cmd(motor_mask: int) -> bytes:
    """只清除指定电机所属关节的 Auto/LUT/ABS Limit，保留其他关节。"""
    motor_mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
    if motor_mask == 0:
        raise ValueError("scoped calibration clear requires a non-zero motor mask")
    return _build_command_frame(
        CMD_CLEAR_CALIBRATION_SCOPE,
        struct.pack(">I", motor_mask),
    )


def build_servo_reset_zero_cmd(motor_mask: int) -> bytes:
    """标定前请求 P4 归零所选舵机的硬件圈计数。"""
    motor_mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
    if motor_mask == 0:
        raise ValueError("calibration turn reset requires a non-zero motor mask")
    return _build_command_frame(
        CMD_SERVO_RESET_ZERO,
        struct.pack(">I", motor_mask),
    )


def build_servo_rebase_zero_cmd(motor_mask: int) -> bytes:
    """Calibration-page alias of the one motor ABS zero command."""
    return build_motor_abs_zero_cmd(motor_mask)


def build_motor_abs_zero_cmd(motor_mask: int) -> bytes:
    """Direct 页将所选电机当前实际位置设为 ABS 0；不发运动目标。"""
    mask = int(motor_mask)
    if mask <= 0 or mask & ~((1 << MOTOR_COUNT) - 1):
        raise ValueError("motor ABS zero requires a non-zero valid motor mask")
    return _build_command_frame(CMD_MOTOR_ABS_ZERO, struct.pack(">I", mask))


def build_motor_direct_release_cmd(token: int = 0) -> bytes:
    """结束普通直控并保持扭矩；零匹配当前代际，非零匹配执行事务。"""
    token = int(token)
    if not 0 <= token <= 0xFFFFFFFF:
        raise ValueError("direct release token must be uint32")
    return _build_command_frame(CMD_MOTOR_DIRECT_RELEASE, struct.pack(">I", token))


def build_manual_encoder_reference_cmd(
    manual_min_raw: Sequence[int],
    manual_max_raw: Sequence[int],
) -> bytes:
    if len(manual_min_raw) != ENCODER_COUNT or len(manual_max_raw) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} Manual Min/Max pairs")
    payload = bytearray([1])
    for joint in range(ENCODER_COUNT):
        min_raw = int(manual_min_raw[joint])
        max_raw = int(manual_max_raw[joint])
        if not 0 <= min_raw < 16384 or not 0 <= max_raw < 16384:
            raise ValueError(f"J{joint:02d} Manual raw must be in [0, 16383]")
        payload.extend(struct.pack(">HH", min_raw, max_raw))
    return _build_command_frame(CMD_MANUAL_ENCODER_REFERENCE, bytes(payload))


def build_clear_manual_encoder_reference_cmd() -> bytes:
    # Firmware uses one fixed command length, so clear carries zero-filled pairs.
    payload = bytes([0]) + bytes(ENCODER_COUNT * 4)
    return _build_command_frame(CMD_MANUAL_ENCODER_REFERENCE, payload)


def build_state_sync_request_cmd(sync_mask: int = STATE_SYNC_ALL) -> bytes:
    """请求 P4 发送指定静态状态；用于连接握手和用户显式刷新。"""
    mask = int(sync_mask)
    if mask < 0 or (mask & ~int(STATE_SYNC_ALL)) != 0:
        raise ValueError(f"invalid state sync mask: 0x{mask:X}")
    return _build_command_frame(CMD_STATE_SYNC_REQUEST, struct.pack(">I", mask))


def build_diagnostics_subscribe_cmd(flags: int, divisor: int = 5) -> bytes:
    """Subscribe diagnostics: bit0 task runtime, bit1 control trace."""
    flags = int(flags)
    if isinstance(divisor, bool) or int(divisor) != divisor or not 1 <= divisor <= 100:
        raise ValueError("Diagnostics divisor must be 1..100")
    if not 0 <= flags <= 3:
        raise ValueError("Diagnostics flags must be 0..3")
    return _build_command_frame(CMD_DIAGNOSTICS_SUBSCRIBE, bytes([flags, divisor]))


def build_manual_encoder_single_cmd(joint: int, min_raw: int, max_raw: int) -> bytes:
    joint = int(joint)
    min_raw = int(min_raw)
    max_raw = int(max_raw)
    if not 0 <= joint < ENCODER_COUNT:
        raise ValueError(f"invalid joint index {joint}")
    if not 0 <= min_raw < 16384 or not 0 <= max_raw < 16384:
        raise ValueError(f"J{joint:02d} Manual raw must be in [0, 16383]")
    return _build_command_frame(
        CMD_MANUAL_ENCODER_SINGLE,
        struct.pack(">BHH", joint, min_raw, max_raw),
    )


def build_rope_init_cmd(active_joint_mask: int) -> bytes:
    mask = int(active_joint_mask) & ((1 << ENCODER_COUNT) - 1)
    if mask == 0:
        raise ValueError("Rope Init requires a non-zero active joint mask")
    return _build_command_frame(CMD_ROPE_INIT, struct.pack(">I", mask))


def manual_encoder_reference_crc(
    manual_min_raw: Sequence[int],
    manual_max_raw: Sequence[int],
) -> int:
    """Match P4's segmented IEEE CRC32 over all 21 endpoint pairs."""
    if len(manual_min_raw) != ENCODER_COUNT or len(manual_max_raw) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} Manual Min/Max pairs")
    crc = 0
    for joint in range(ENCODER_COUNT):
        data = bytes((
            joint,
            (int(manual_min_raw[joint]) >> 8) & 0xFF,
            int(manual_min_raw[joint]) & 0xFF,
            (int(manual_max_raw[joint]) >> 8) & 0xFF,
            int(manual_max_raw[joint]) & 0xFF,
        ))
        value = (~crc) & 0xFFFFFFFF
        for byte in data:
            value ^= byte
            for _ in range(8):
                value = (value >> 1) ^ (0xEDB88320 if value & 1 else 0)
        crc = (~value) & 0xFFFFFFFF
    return crc


def manual_encoder_reference_entry_crc(
    joint: int,
    manual_min_raw: int,
    manual_max_raw: int,
) -> int:
    """Match P4's per-joint IEEE CRC32 binding for one Manual pair."""
    joint = int(joint)
    min_raw = int(manual_min_raw)
    max_raw = int(manual_max_raw)
    if not 0 <= joint < ENCODER_COUNT:
        raise ValueError(f"joint must be in [0, {ENCODER_COUNT - 1}]")
    if not 0 <= min_raw < ENCODER_RAW_MODULO:
        raise ValueError("Manual Min must be in [0, 16383]")
    if not 0 <= max_raw < ENCODER_RAW_MODULO:
        raise ValueError("Manual Max must be in [0, 16383]")
    data = bytes((
        joint,
        (min_raw >> 8) & 0xFF,
        min_raw & 0xFF,
        (max_raw >> 8) & 0xFF,
        max_raw & 0xFF,
    ))
    value = 0xFFFFFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xEDB88320 if value & 1 else 0)
    return (~value) & 0xFFFFFFFF


# P4 多圈 ABS 使用 int32 传输；实际可达范围由下位机按舵机硬件行程
# 上报和校验。
MOTOR_ABS_CMD_MIN = -(2**31)
MOTOR_ABS_CMD_MAX = 2**31 - 1


def build_motor_pos_abs_cmd(
    motor_abs: Sequence[int], *, token: int = 0, motor_mask: int = 0,
    speed: int = 400, acc: int = 254,
) -> bytes:
    """统一直控帧；mask=0 使用当前 scope，token=0 不跟踪执行回执。"""
    if len(motor_abs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor absolute values")
    token, motor_mask, speed, acc = map(int, (token, motor_mask, speed, acc))
    if not 0 <= token <= 0xFFFFFFFF:
        raise ValueError("direct token must be uint32")
    if motor_mask < 0 or motor_mask & ~((1 << MOTOR_COUNT) - 1):
        raise ValueError("direct motor mask must fit the 27 motor channels")
    values = [int(value) for value in motor_abs]
    if any(value < MOTOR_ABS_CMD_MIN or value > MOTOR_ABS_CMD_MAX for value in values):
        raise ValueError("direct targets must be int32")
    if not 0 <= speed <= 0xFFFF or not 0 <= acc <= 0xFF:
        raise ValueError("direct speed/acc must be uint16/uint8")
    payload = struct.pack(f">II{MOTOR_COUNT}iHB", token, motor_mask, *values, speed, acc)
    return _build_command_frame(CMD_MOTOR_POS_ABS, payload)


def build_motor_pos_abs_one_cmd(motor_index: int, motor_abs: int) -> bytes:
    """Single-motor convenience over the one general direct command."""
    idx = int(motor_index)
    if idx < 0 or idx >= MOTOR_COUNT:
        raise ValueError(f"motor index out of range: {idx}")
    values = [0] * MOTOR_COUNT
    values[idx] = int(motor_abs)
    return build_motor_pos_abs_cmd(values, motor_mask=1 << idx)
