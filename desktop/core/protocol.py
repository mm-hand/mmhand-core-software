import struct
from enum import Enum
from typing import List, Optional, Sequence, Tuple


class ControlMode(Enum):
    JOINT_ANGLE = 0
    DIRECT_MOTOR = 1


PROTOCOL_HEADER = 0xFE
PROTOCOL_TAIL = 0xFF

# Upstream packet types (device -> desktop)
PACKET_TYPE_SENSOR = 0x01
PACKET_TYPE_CALIB_ACK = 0x02
PACKET_TYPE_SERVO_ANGLE = 0x03
PACKET_TYPE_JOINT_DEBUG = 0x04
PACKET_TYPE_SERVO_TELEM = 0x05
PACKET_TYPE_PROTO_ACK = 0x06
PACKET_TYPE_FAULT_STATUS = 0x07
PACKET_TYPE_SERVO_RAW = 0x09
PACKET_TYPE_TACTILE = 0x0A
PACKET_TYPE_ENCODER_RAW = 0x0B
PACKET_TYPE_SYSTEM_STATUS = 0x0C
PACKET_TYPE_MOTOR_ABS_LIMIT = 0x0D
PACKET_TYPE_JOINT_CALIB_SNAPSHOT = 0x0E
PACKET_TYPE_TACTILE_FULL = 0x0F
PACKET_TYPE_CALIB_DIAG = 0x10
PACKET_TYPE_TASK_DIAG = 0x11
PACKET_TYPE_MANUAL_ENCODER_REFERENCE = 0x12

CALIB_WARN_NONE = 0x00
CALIB_WARN_AUTO_MIN_OFFSET = 0x01
CALIB_WARN_AUTO_MAX_SHORTFALL = 0x02
CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH = 0x04
# Compatibility alias for callers that still import the original name.
CALIB_WARN_AUTO_SPAN_RANGE = CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH

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

# Downstream commands (desktop -> device)
CMD_CALIBRATE = 0xCA
CMD_ANGLE_CTRL = 0xCB
CMD_START = 0xCC
CMD_STOP = 0xCD
CMD_RESET = 0xCE
CMD_CALIB_DATA = 0xCF
CMD_MOTOR_POS = 0xD0
CMD_SENSOR_STREAM_MODE = 0xD1
CMD_MOTOR_POS_SWEEP = 0xD2
CMD_MOTOR_POS_ABS = 0xD3
CMD_MOTOR_POS_ABS_ONE = 0xDE
CMD_ACTIVE_SCOPE = 0xD7
CMD_RUN_FEEDBACK = 0xD8
CMD_PAUSE = 0xD9
CMD_RESTORE_MULTITURN = 0xDA
CMD_MOTOR_ABS_LIMIT_LOCK = 0xDB
# 上位机在线心跳；固件用它判断桌面端是否已经关闭。
CMD_HOST_HEARTBEAT = 0xDC
CMD_MULTITURN_NVS_CONTROL = 0xDD
CMD_CALIB_LOAD_JOINT = 0xDF
CMD_CALIB_LOAD_LIMITS = 0xE0
CMD_LOCAL_TEST_PARAMS = 0xE1
CMD_LOCAL_TEST_START = 0xE2
CMD_LOCAL_TEST_STOP = 0xE3
CMD_CLEAR_CALIBRATION = 0xE5
CMD_MANUAL_ENCODER_REFERENCE = 0xE6
CMD_MANUAL_ENCODER_QUERY = 0xE7
CMD_MANUAL_ENCODER_SINGLE = 0xE8
CMD_ROPE_INIT = 0xE9

SENSOR_STREAM_MODE_LEGACY_U16_WRAP = 0
SENSOR_STREAM_MODE_SIGNED_I16 = 1

PROTO_ACK_STATUS_OK = 0
PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1
PROTO_ACK_STATUS_QUEUED = 2
PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED = 3
PROTO_ACK_STATUS_INVALID_DATA = 4
PROTO_ACK_STATUS_NVS_FAILURE = 5
PROTO_ACK_STATUS_MANUAL_CRC_MISMATCH = 6
PROTO_ACK_STATUS_LEGACY_SNAPSHOT_REJECTED = 7

SYSTEM_STATE_UNCONTROLLABLE = 0
SYSTEM_STATE_IDLE = 1
SYSTEM_STATE_RUN_FEEDBACK = 2
SYSTEM_STATE_CALIBRATION_RUNNING = 4
SYSTEM_STATE_READY = 5
SYSTEM_STATE_RUN_DIRECT = 6
SYSTEM_STATE_PAUSED = 7
SYSTEM_STATE_ROPE_INIT_RUNNING = 8

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
ROPE_INIT_REASON_TASK_START_FAILED = 10

SYSTEM_FAULT_CAN_OFFLINE = 1 << 0
SYSTEM_FAULT_JOINT16_DUAL = 1 << 1
SYSTEM_FAULT_OVERLOAD = 1 << 2
# Fault bitmap bit 3 is reserved by the P4 protocol.
SYSTEM_FAULT_SERVO_OFFLINE = 1 << 4
SYSTEM_FAULT_CALIBRATION = 1 << 5
SYSTEM_FAULT_MULTITURN_UNKNOWN = 1 << 6

CALIB_DIAG_REASON_NONE = 0
CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY = 1
CALIB_DIAG_REASON_CAN_FEEDBACK_TIMEOUT = 2
CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT = 3
CALIB_DIAG_REASON_TELEMETRY_TIMEOUT = 4
CALIB_DIAG_REASON_ENCODER_ERROR = 5
CALIB_DIAG_REASON_SERVO_OFFLINE = 6
CALIB_DIAG_REASON_TURN_UNKNOWN = 7
CALIB_DIAG_REASON_OVERLOAD = 8
CALIB_DIAG_REASON_LOAD_NOT_REACHED = 9
CALIB_DIAG_REASON_ANGLE_NOT_MOVING = 10
CALIB_DIAG_REASON_WRONG_DIRECTION = 11
CALIB_DIAG_REASON_ZERO_RETURN_TIMEOUT = 12
CALIB_DIAG_REASON_ZERO_CROSS_INVALID = 13
CALIB_DIAG_REASON_WRAP_TOO_LARGE = 14
CALIB_DIAG_REASON_ABS_LIMIT_INVALID = 15
CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED = 16
CALIB_DIAG_REASON_FEEDBACK_LOST = 17
CALIB_DIAG_REASON_TASK_START_FAILED = 18
CALIB_DIAG_REASON_INTERNAL_ERROR = 19
CALIB_DIAG_REASON_CONTROL_LOST = 20
CALIB_DIAG_REASON_HOLD_TARGET_FAILED = 21
CALIB_DIAG_REASON_JOINT_LIST_INVALID = 22
CALIB_DIAG_REASON_DUPLICATE_JOINT = 23
CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED = 24
CALIB_DIAG_REASON_JOINT_STEP_FAILED = 25

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

# Decoupled dimensions
ENCODER_COUNT = 21
MOTOR_COUNT = 27
ENCODER_RAW_MODULO = 16384
DUAL_TENDON_JOINT_INDICES = (0, 4, 8, 12, 16, 20)
DUAL_TENDON_LUT_ACTUATOR_COUNT = 2
DUAL_TENDON_LUT_MAX_SAMPLES = 24
TACTILE_GROUP_COUNT = 5
TACTILE_SENSOR_PER_GROUP = 3
TACTILE_AXIS_COUNT = 3
TACTILE_SUMMARY_BYTES = TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_COUNT
TACTILE_TIP_TAXELS = 25
TACTILE_PAD_TAXELS = 52
TACTILE_SENSOR_TAXEL_COUNTS = (TACTILE_PAD_TAXELS, TACTILE_TIP_TAXELS, TACTILE_TIP_TAXELS)
TACTILE_FULL_BYTES = TACTILE_GROUP_COUNT * sum(TACTILE_SENSOR_TAXEL_COUNTS) * TACTILE_AXIS_COUNT
TACTILE_FULL_PACKET_MAGIC = b"T2"
HAND_PROFILE = "v2"
VALID_MOTOR_COUNT_V1 = 22
VALID_MOTOR_MASK_V1 = (1 << VALID_MOTOR_COUNT_V1) - 1
VALID_MOTOR_MASK_V2 = (1 << MOTOR_COUNT) - 1
VALID_MOTOR_MASK = VALID_MOTOR_MASK_V2 if HAND_PROFILE.lower() == "v2" else VALID_MOTOR_MASK_V1


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
DISCONNECT_SENTINEL = 0x7FFF

# Serial settings
BAUDRATE = 921600
SERIAL_TIMEOUT = 0.1
RX_QUEUE_MAXSIZE = 200
TX_QUEUE_MAXSIZE = 100
RX_POLL_SLEEP = 0.005
TX_QUEUE_GET_TIMEOUT = 0.1


def _build_command_frame(cmd: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    # Symmetric downstream frame: [0xFE][LEN][CMD][PAYLOAD][0xFF]
    # LEN = CMD byte + payload + tail byte.
    frame_len = 1 + len(payload) + 1
    return bytes([PROTOCOL_HEADER, frame_len & 0xFF, cmd & 0xFF]) + payload + bytes([PROTOCOL_TAIL])


def parse_sensor_packet(payload: bytes) -> Tuple[List[int], List[bool]]:
    """Parse 21 raw magnetic encoder counts from the firmware sensor packet."""
    angles: List[int] = []
    errors: List[bool] = []
    if len(payload) < ENCODER_COUNT * 2:
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
    if len(payload) < 3:
        return None
    return payload[0], payload[1], payload[2]


def parse_system_status_packet(payload: bytes) -> Optional[tuple]:
    if len(payload) < 13:
        return None
    system_state = payload[0]
    control_enabled = payload[1]
    servo_target_owner = payload[2]
    control_mode = payload[3]
    calib_status = payload[4]
    system_fault_bitmap = struct.unpack(">I", payload[5:9])[0]
    state_transition_count = struct.unpack(">I", payload[9:13])[0]
    multi_turn_unknown_bitmap = struct.unpack(">I", payload[13:17])[0] if len(payload) >= 17 else 0
    multiturn_nvs_enabled = payload[17] if len(payload) >= 18 else 1
    multiturn_nvs_snapshot_valid = payload[18] if len(payload) >= 19 else 0
    has_active_scope_status = len(payload) >= 28
    active_motor_mask = struct.unpack(">I", payload[19:23])[0] if has_active_scope_status else 0
    active_joint_mask = struct.unpack(">I", payload[23:27])[0] if has_active_scope_status else 0
    require_can_feedback = payload[27] if has_active_scope_status else 0
    has_connectivity_status = len(payload) >= 37
    servo_online_bitmap = struct.unpack(">I", payload[28:32])[0] if has_connectivity_status else 0
    encoder_online_bitmap = struct.unpack(">I", payload[32:36])[0] if has_connectivity_status else 0
    can_link_online = payload[36] if has_connectivity_status else 0
    has_encoder_health_status = len(payload) >= 45
    encoder_warn_bitmap = struct.unpack(">I", payload[37:41])[0] if has_encoder_health_status else 0
    encoder_fault_bitmap = struct.unpack(">I", payload[41:45])[0] if has_encoder_health_status else 0
    has_multiturn_nvs_status = len(payload) >= 61
    multiturn_nvs_save_request_token = (
        struct.unpack(">I", payload[45:49])[0] if has_multiturn_nvs_status else 0
    )
    multiturn_nvs_save_result_token = (
        struct.unpack(">I", payload[49:53])[0] if has_multiturn_nvs_status else 0
    )
    multiturn_nvs_save_valid_mask = (
        struct.unpack(">I", payload[53:57])[0] if has_multiturn_nvs_status else 0
    )
    multiturn_nvs_save_fail_count = (
        struct.unpack(">I", payload[57:61])[0] if has_multiturn_nvs_status else 0
    )
    has_manual_reference_status = len(payload) >= 74
    manual_reference_valid_bitmap = (
        struct.unpack(">I", payload[61:65])[0] if has_manual_reference_status else 0
    )
    manual_reference_crc = (
        struct.unpack(">I", payload[65:69])[0] if has_manual_reference_status else 0
    )
    manual_reference_nvs_sequence = (
        struct.unpack(">I", payload[69:73])[0] if has_manual_reference_status else 0
    )
    manual_reference_nvs_valid = (
        payload[73] if has_manual_reference_status else 0
    )
    has_manual_v2_status = len(payload) >= 79
    auto_calibration_valid_bitmap = (
        struct.unpack(">I", payload[74:78])[0] if has_manual_v2_status else 0
    )
    manual_reference_save_status = payload[78] if has_manual_v2_status else 0
    has_motion_result_status = len(payload) >= 102
    rope_init_status = payload[79] if has_motion_result_status else ROPE_INIT_STATUS_IDLE
    rope_init_reason = payload[80] if has_motion_result_status else ROPE_INIT_REASON_NONE
    rope_init_sequence = (
        struct.unpack(">I", payload[81:85])[0] if has_motion_result_status else 0
    )
    rope_init_joint_mask = (
        struct.unpack(">I", payload[85:89])[0] if has_motion_result_status else 0
    )
    rope_init_timestamp_ms = (
        struct.unpack(">I", payload[89:93])[0] if has_motion_result_status else 0
    )
    pending_direct_generation = (
        struct.unpack(">I", payload[93:97])[0] if has_motion_result_status else 0
    )
    applied_direct_generation = (
        struct.unpack(">I", payload[97:101])[0] if has_motion_result_status else 0
    )
    direct_reject_reason = payload[101] if has_motion_result_status else 0
    has_calibration_warning_status = len(payload) >= 148
    calibration_warning_bitmap = (
        struct.unpack(">I", payload[102:106])[0]
        if has_calibration_warning_status else 0
    )
    calibration_auto_min_offset_counts = (
        [
            struct.unpack(">h", payload[106 + 2 * i:108 + 2 * i])[0]
            for i in range(ENCODER_COUNT)
        ]
        if has_calibration_warning_status else [0] * ENCODER_COUNT
    )
    return (
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
        calibration_warning_bitmap,
        calibration_auto_min_offset_counts,
        has_calibration_warning_status,
    )


def parse_motor_abs_limit_packet(payload: bytes) -> Optional[Tuple[int, int, List[int], List[int]]]:
    required = 4 + MOTOR_COUNT * 4
    if len(payload) < required:
        return None
    valid_bitmap = struct.unpack(">I", payload[:4])[0]
    if len(payload) >= 8 + MOTOR_COUNT * 4:
        locked_bitmap = struct.unpack(">I", payload[4:8])[0]
        offset = 8
    else:
        # Backward compatibility with older firmware: default to unlocked.
        locked_bitmap = 0
        offset = 4
    mins: List[int] = []
    maxs: List[int] = []
    for _ in range(MOTOR_COUNT):
        mins.append(struct.unpack(">h", payload[offset:offset + 2])[0])
        maxs.append(struct.unpack(">h", payload[offset + 2:offset + 4])[0])
        offset += 4
    return valid_bitmap, locked_bitmap, mins, maxs


def parse_manual_encoder_reference_packet(payload: bytes) -> Optional[dict]:
    """Parse a full or partial Manual Min/Max snapshot stored by P4."""
    if not payload or payload[0] not in (1, 2, 3):
        return None
    version = int(payload[0])
    required = 1 + 4 + 4 + 4 + 1 + ENCODER_COUNT * 4
    if version >= 2:
        required += 5
    if version >= 3:
        required += ENCODER_COUNT
    if len(payload) < required:
        return None

    valid_bitmap = struct.unpack(">I", payload[1:5])[0]
    reference_crc = struct.unpack(">I", payload[5:9])[0]
    nvs_sequence = struct.unpack(">I", payload[9:13])[0]
    nvs_valid = bool(payload[13])
    offset = 14
    auto_valid_bitmap = 0
    save_status = 0
    if version >= 2:
        auto_valid_bitmap = struct.unpack(">I", payload[offset:offset + 4])[0]
        offset += 4
        save_status = int(payload[offset])
        offset += 1
    min_raw: List[int] = []
    max_raw: List[int] = []
    for _ in range(ENCODER_COUNT):
        min_raw.append(struct.unpack(">H", payload[offset:offset + 2])[0])
        max_raw.append(struct.unpack(">H", payload[offset + 2:offset + 4])[0])
        offset += 4
    encoder_directions: List[int] = [0] * ENCODER_COUNT
    if version >= 3:
        encoder_directions = [
            -1 if struct.unpack(">b", payload[offset + joint:offset + joint + 1])[0] < 0 else 1
            for joint in range(ENCODER_COUNT)
        ]
    return {
        "version": version,
        "valid_bitmap": valid_bitmap,
        "reference_crc": reference_crc,
        "nvs_sequence": nvs_sequence,
        "nvs_valid": nvs_valid,
        "auto_valid_bitmap": auto_valid_bitmap,
        "save_status": save_status,
        "min_raw": min_raw,
        "max_raw": max_raw,
        "encoder_directions": encoder_directions,
        "has_encoder_directions": version >= 3,
    }


def _unpack_float_be(data: bytes) -> float:
    return struct.unpack(">f", data)[0]


def parse_joint_calib_snapshot_packet(payload: bytes) -> Optional[dict]:
    # One joint per frame keeps the existing 1-byte LEN protocol unchanged.
    if len(payload) < 54:
        return None
    idx = 0
    joint = payload[idx]
    idx += 1
    if joint >= ENCODER_COUNT:
        return None

    result = {
        "joint": int(joint),
        "success": bool(payload[idx]),
    }
    idx += 1
    for key in ("encoderMin", "encoderMax", "offset"):
        result[key] = struct.unpack(">i", payload[idx:idx + 4])[0]
        idx += 4
    result["angleMax"] = _unpack_float_be(payload[idx:idx + 4])
    idx += 4
    for key in ("startEncoder", "tightEncoder", "zeroEncoder", "encoderSpan"):
        result[key] = struct.unpack(">i", payload[idx:idx + 4])[0]
        idx += 4
    for key in ("startServoAbs", "tightServoAbs", "zeroServoAbs"):
        result[key] = struct.unpack(">h", payload[idx:idx + 2])[0]
        idx += 2
    for key in ("startDeg", "tightDeg", "totalDeg"):
        result[key] = _unpack_float_be(payload[idx:idx + 4])
        idx += 4
    result["scaled"] = bool(payload[idx])
    idx += 1
    result["encoderDirection"] = -1 if struct.unpack(">b", payload[idx:idx + 1])[0] < 0 else 1
    idx += 1
    result["dualTendonLutValid"] = False
    result["dualTendonLutCount"] = 0
    result["dualTendonLutMotor"] = [0xFF, 0xFF]
    result["dualTendonLutAngleStartDeg"] = 0.0
    result["dualTendonLutAngleStepDeg"] = 0.0
    result["dualTendonLutMotorAbs"] = [
        [0] * DUAL_TENDON_LUT_MAX_SAMPLES
        for _ in range(DUAL_TENDON_LUT_ACTUATOR_COUNT)
    ]
    result["mappingVersion"] = 0
    result["encoderAtUrdfMin"] = 0
    result["encoderAtUrdfMax"] = 0
    result["angleMin"] = 0.0
    result["urdfConfigCrc"] = 0
    result["autoMinRaw"] = 0
    result["autoMaxRaw"] = 0
    result["manualReferenceCrc"] = 0
    result["currentMinDeg"] = 0.0
    result["currentMaxDeg"] = 0.0
    result["hasManualScaledV3Fields"] = False
    result["assemblyWarningReason"] = CALIB_WARN_NONE
    result["autoMinOffsetCounts"] = 0
    result["autoMaxShortfallCounts"] = 0
    result["assemblyAutoSpanCounts"] = 0
    result["assemblyManualSpanCounts"] = 0
    result["assemblyUrdfSpanCounts"] = 0
    result["assemblySpanDifferenceCounts"] = 0
    result["hasAssemblyWarningFields"] = False
    result["hasAssemblySpanComparisonFields"] = False
    extended_len = 54 + 1 + 1 + 2 + 4 + 4 + (
        DUAL_TENDON_LUT_ACTUATOR_COUNT * DUAL_TENDON_LUT_MAX_SAMPLES * 2
    )
    if len(payload) >= extended_len:
        result["dualTendonLutValid"] = bool(payload[idx])
        idx += 1
        result["dualTendonLutCount"] = int(payload[idx])
        idx += 1
        result["dualTendonLutMotor"] = [int(payload[idx]), int(payload[idx + 1])]
        idx += 2
        result["dualTendonLutAngleStartDeg"] = _unpack_float_be(payload[idx:idx + 4])
        idx += 4
        result["dualTendonLutAngleStepDeg"] = _unpack_float_be(payload[idx:idx + 4])
        idx += 4
        motor_abs: List[List[int]] = []
        for _actuator in range(DUAL_TENDON_LUT_ACTUATOR_COUNT):
            row: List[int] = []
            for _sample in range(DUAL_TENDON_LUT_MAX_SAMPLES):
                row.append(struct.unpack(">h", payload[idx:idx + 2])[0])
                idx += 2
            motor_abs.append(row)
        result["dualTendonLutMotorAbs"] = motor_abs
        v2_len = extended_len + 17
        if len(payload) >= v2_len:
            result["mappingVersion"] = int(payload[idx])
            idx += 1
            result["encoderAtUrdfMin"] = struct.unpack(">i", payload[idx:idx + 4])[0]
            idx += 4
            result["encoderAtUrdfMax"] = struct.unpack(">i", payload[idx:idx + 4])[0]
            idx += 4
            result["angleMin"] = _unpack_float_be(payload[idx:idx + 4])
            idx += 4
            result["urdfConfigCrc"] = struct.unpack(">I", payload[idx:idx + 4])[0]
            idx += 4
            v3_len = v2_len + 16
            if len(payload) >= v3_len:
                result["autoMinRaw"] = struct.unpack(">H", payload[idx:idx + 2])[0]
                idx += 2
                result["autoMaxRaw"] = struct.unpack(">H", payload[idx:idx + 2])[0]
                idx += 2
                result["manualReferenceCrc"] = struct.unpack(">I", payload[idx:idx + 4])[0]
                idx += 4
                result["currentMinDeg"] = _unpack_float_be(payload[idx:idx + 4])
                idx += 4
                result["currentMaxDeg"] = _unpack_float_be(payload[idx:idx + 4])
                idx += 4
                result["hasManualScaledV3Fields"] = True
                warning_len = v3_len + 7
                if len(payload) >= warning_len:
                    result["assemblyWarningReason"] = int(payload[idx])
                    idx += 1
                    result["autoMinOffsetCounts"] = struct.unpack(
                        ">h", payload[idx:idx + 2]
                    )[0]
                    idx += 2
                    result["autoMaxShortfallCounts"] = struct.unpack(
                        ">h", payload[idx:idx + 2]
                    )[0]
                    idx += 2
                    result["assemblyAutoSpanCounts"] = struct.unpack(
                        ">H", payload[idx:idx + 2]
                    )[0]
                    idx += 2
                    result["hasAssemblyWarningFields"] = True
                    span_comparison_len = warning_len + 6
                    if len(payload) >= span_comparison_len:
                        result["assemblyManualSpanCounts"] = struct.unpack(
                            ">H", payload[idx:idx + 2]
                        )[0]
                        idx += 2
                        result["assemblyUrdfSpanCounts"] = struct.unpack(
                            ">H", payload[idx:idx + 2]
                        )[0]
                        idx += 2
                        result["assemblySpanDifferenceCounts"] = struct.unpack(
                            ">H", payload[idx:idx + 2]
                        )[0]
                        idx += 2
                        result["hasAssemblySpanComparisonFields"] = True
    return result


def parse_calib_diag_packet(payload: bytes) -> Optional[dict]:
    if len(payload) < 25:
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
    return result


def parse_task_diag_packet(payload: bytes) -> Optional[dict]:
    if len(payload) < 53 or payload[0] not in (1, 2):
        return None
    version = int(payload[0])
    if version == 2 and len(payload) < 57:
        return None
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
    if version >= 2:
        result["stack_current_free_bytes"] = read_u32()
        result["stack_min_free_bytes"] = read_u32()
    else:
        min_free = read_u32()
        result["stack_current_free_bytes"] = min_free
        result["stack_min_free_bytes"] = min_free
    result.update({
        "loop_count": read_u32(),
        "max_gap_us": read_u32(),
        "max_exec_us": read_u32(),
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
    return result


def parse_calib_ack(payload: bytes) -> Optional[int]:
    if len(payload) < 1:
        return None
    return payload[0]


def parse_servo_angle_packet(payload: bytes) -> Optional[Tuple[List[int], List[bool]]]:
    # Some firmware revisions upload fewer channels while newer desktop code reserves 27.
    # Accept any payload that matches N * (4-byte angle + 1-byte online flag).
    if not payload or (len(payload) % 5) != 0:
        return None

    channel_count = len(payload) // 5
    if channel_count <= 0:
        return None

    angles: List[int] = []
    for i in range(channel_count):
        start = i * 4
        angles.append(struct.unpack(">i", payload[start:start + 4])[0])

    online_flags: List[bool] = []
    base = channel_count * 4
    for i in range(channel_count):
        online_flags.append(payload[base + i] == 1)
    return angles, online_flags


def parse_servo_telem_packet(
    payload: bytes,
) -> Optional[Tuple[List[int], List[int], List[int], List[int], List[bool]]]:
    if not payload or (len(payload) % 7) != 0:
        return None

    channel_count = len(payload) // 7
    if channel_count <= 0:
        return None

    speeds: List[int] = []
    loads: List[int] = []
    volts: List[int] = []
    temps: List[int] = []
    online: List[bool] = []

    for i in range(channel_count):
        base = i * 7
        speeds.append(struct.unpack(">h", payload[base:base + 2])[0])
        loads.append(struct.unpack(">h", payload[base + 2:base + 4])[0])
        volts.append(payload[base + 4])
        temps.append(payload[base + 5])
        online.append(payload[base + 6] == 1)
    return speeds, loads, volts, temps, online


def parse_servo_raw_packet(payload: bytes) -> Optional[Tuple[List[int], List[int], List[bool], bool]]:
    if not payload:
        return None

    if (len(payload) % 7) == 0:
        channel_count = len(payload) // 7
        has_hardware_positions = True
    elif (len(payload) % 3) == 0:
        channel_count = len(payload) // 3
        has_hardware_positions = False
    else:
        return None
    if channel_count <= 0:
        return None

    raw_positions: List[int] = []
    for i in range(channel_count):
        start = i * 2
        raw_positions.append(struct.unpack(">h", payload[start:start + 2])[0])

    hardware_positions: List[int] = []
    hw_base = channel_count * 2
    online_base = channel_count * 6 if has_hardware_positions else channel_count * 2
    if has_hardware_positions:
        for i in range(channel_count):
            start = hw_base + i * 4
            hardware_positions.append(struct.unpack(">i", payload[start:start + 4])[0])

    online_flags: List[bool] = []
    for i in range(channel_count):
        online_flags.append(payload[online_base + i] == 1)
    return raw_positions, hardware_positions, online_flags, has_hardware_positions


def parse_fault_status_packet(payload: bytes) -> Optional[Tuple[int, List[Optional[int]]]]:
    if len(payload) < 4:
        return None
    fault_bitmap = struct.unpack(">I", payload[:4])[0]
    trigger_loads: List[Optional[int]] = [None] * MOTOR_COUNT
    if len(payload) >= 4 + MOTOR_COUNT * 2:
        offset = 4
        for i in range(MOTOR_COUNT):
            trigger_loads[i] = struct.unpack(">h", payload[offset:offset + 2])[0]
            offset += 2
    return fault_bitmap, trigger_loads


def parse_tactile_packet(payload: bytes) -> Optional[Tuple[int, List[List[List[int]]]]]:
    if len(payload) == (1 + TACTILE_SUMMARY_BYTES):
        seq = payload[0]
        data = payload[1:]
    elif len(payload) == TACTILE_SUMMARY_BYTES:
        seq = 0
        data = payload
    else:
        return None

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
    return seq, out


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

    if len(payload) < 4:
        return None
    seq = payload[0]
    frag_idx = payload[1]
    frag_count = payload[2]
    chunk_len = payload[3]
    if frag_count <= 0 or frag_idx >= frag_count:
        return None
    if len(payload) != 4 + chunk_len:
        return None
    return seq, frag_idx, frag_count, bytes(payload[4:4 + chunk_len]), None, None


def parse_joint_debug_packet(
    payload: bytes,
) -> Optional[Tuple[int, bool, float, float, float, float, float, bool, int, int]]:
    legacy_len = 2 + 5 * 4
    cmd_len = legacy_len + 1 + 2
    cmd_ts_len = cmd_len + 4
    if len(payload) not in (legacy_len, cmd_len, cmd_ts_len):
        return None

    joint_index, valid, target_deg, actual_deg, loop1_out, loop2_act, loop2_out = struct.unpack(
        ">BBfffff", payload[:legacy_len]
    )
    if len(payload) >= cmd_len:
        cmd_valid = payload[legacy_len] == 1
        cmd_target_pos = struct.unpack(">h", payload[legacy_len + 1 : legacy_len + 3])[0]
    else:
        cmd_valid = False
        cmd_target_pos = 0

    if len(payload) == cmd_ts_len:
        device_timestamp_ms = struct.unpack(">I", payload[cmd_len : cmd_len + 4])[0]
    else:
        device_timestamp_ms = 0

    return (
        joint_index,
        valid == 1,
        target_deg,
        actual_deg,
        loop1_out,
        loop2_act,
        loop2_out,
        cmd_valid,
        cmd_target_pos,
        device_timestamp_ms,
    )


def parse_frame(data: bytes) -> Tuple[List[Tuple[int, bytes]], bytearray]:
    result: List[Tuple[int, bytes]] = []
    buffer = bytearray(data)
    while len(buffer) >= 4:
        try:
            head_idx = buffer.index(PROTOCOL_HEADER)
        except ValueError:
            break

        buffer = buffer[head_idx:]
        if len(buffer) < 4:
            break

        payload_len = buffer[1]
        frame_len = 2 + payload_len
        if len(buffer) < frame_len:
            break

        frame = buffer[:frame_len]
        if frame[-1] != PROTOCOL_TAIL:
            buffer = buffer[1:]
            continue

        pkt_type = frame[2]
        payload = bytes(frame[3:-1])
        result.append((pkt_type, payload))
        buffer = buffer[frame_len:]

    return result, buffer


def build_calibrate_cmd() -> bytes:
    return _build_command_frame(CMD_CALIBRATE)


def build_start_cmd() -> bytes:
    return _build_command_frame(CMD_START)


def build_stop_cmd() -> bytes:
    return _build_command_frame(CMD_STOP)


def build_reset_cmd() -> bytes:
    return _build_command_frame(CMD_RESET)


def build_calib_data_cmd(zero_encoder_raw: List[int]) -> bytes:
    if len(zero_encoder_raw) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} encoder zero values")
    vals = [float(v) for v in zero_encoder_raw]
    payload = struct.pack(f"<{ENCODER_COUNT}f", *vals)
    return _build_command_frame(CMD_CALIB_DATA, payload)


def build_active_scope_cmd(active_motor_mask: int, active_joint_mask: int, require_can_feedback: bool) -> bytes:
    payload = struct.pack(
        ">IIB",
        int(active_motor_mask) & 0xFFFFFFFF,
        int(active_joint_mask) & 0xFFFFFFFF,
        1 if require_can_feedback else 0,
    )
    return _build_command_frame(CMD_ACTIVE_SCOPE, payload)


def build_angle_cmd(angles: List[float]) -> bytes:
    if len(angles) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} joint angles")
    payload = struct.pack(f"<{ENCODER_COUNT}f", *angles)
    return _build_command_frame(CMD_ANGLE_CTRL, payload)


def build_run_feedback_cmd() -> bytes:
    return _build_command_frame(CMD_RUN_FEEDBACK, b"")


def build_pause_cmd() -> bytes:
    return _build_command_frame(CMD_PAUSE, b"")


def build_restore_multiturn_cmd(active_motor_mask: int, motor_abs: List[int]) -> bytes:
    if len(motor_abs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor absolute values")
    payload = bytearray()
    payload.extend(struct.pack(">I", int(active_motor_mask) & 0xFFFFFFFF))
    for value in motor_abs:
        v = int(value)
        if v < MOTOR_ABS_CMD_MIN:
            v = MOTOR_ABS_CMD_MIN
        elif v > MOTOR_ABS_CMD_MAX:
            v = MOTOR_ABS_CMD_MAX
        payload.extend(struct.pack(">h", v))
    return _build_command_frame(CMD_RESTORE_MULTITURN, bytes(payload))


def build_motor_abs_limit_lock_cmd(active_motor_mask: int, lock_enabled: bool) -> bytes:
    payload = struct.pack(
        ">BI",
        1 if lock_enabled else 0,
        int(active_motor_mask) & 0xFFFFFFFF,
    )
    return _build_command_frame(CMD_MOTOR_ABS_LIMIT_LOCK, payload)


def _pack_float_be(value: float) -> bytes:
    return struct.pack(">f", float(value))


def _clamp_i16(value: int) -> int:
    v = int(value)
    if v < -32768:
        return -32768
    if v > 32767:
        return 32767
    return v


def build_calib_load_limits_cmd(valid_bitmap: int, locked_bitmap: int, mins: List[int], maxs: List[int]) -> bytes:
    if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor limit values")
    payload = bytearray()
    payload.extend(struct.pack(">II", int(valid_bitmap) & 0xFFFFFFFF, int(locked_bitmap) & 0xFFFFFFFF))
    for i in range(MOTOR_COUNT):
        payload.extend(struct.pack(">h", _clamp_i16(mins[i])))
        payload.extend(struct.pack(">h", _clamp_i16(maxs[i])))
    return _build_command_frame(CMD_CALIB_LOAD_LIMITS, bytes(payload))


def build_calib_load_joint_cmd(result: dict) -> bytes:
    joint = int(result.get("joint", -1))
    if joint < 0 or joint >= ENCODER_COUNT:
        raise ValueError(f"invalid joint index {joint}")
    if int(result.get("mappingVersion", 0)) != 3:
        raise ValueError(
            f"J{joint:02d} legacy calibration snapshot is view-only; v3 is required"
        )
    payload = bytearray()
    payload.append(joint & 0xFF)
    payload.append(1 if bool(result.get("success", False)) else 0)
    for key in ("encoderMin", "encoderMax", "offset"):
        payload.extend(struct.pack(">i", int(result.get(key, 0))))
    payload.extend(_pack_float_be(float(result.get("angleMax", 0.0))))
    for key in ("startEncoder", "tightEncoder", "zeroEncoder", "encoderSpan"):
        payload.extend(struct.pack(">i", int(result.get(key, 0))))
    for key in ("startServoAbs", "tightServoAbs", "zeroServoAbs"):
        payload.extend(struct.pack(">h", _clamp_i16(result.get(key, 0))))
    for key in ("startDeg", "tightDeg", "totalDeg"):
        payload.extend(_pack_float_be(float(result.get(key, 0.0))))
    payload.append(1 if bool(result.get("scaled", False)) else 0)
    direction = -1 if int(result.get("encoderDirection", 1)) < 0 else 1
    payload.extend(struct.pack(">b", direction))
    payload.append(1 if bool(result.get("dualTendonLutValid", False)) else 0)
    payload.append(int(result.get("dualTendonLutCount", 0)) & 0xFF)
    motors = list(result.get("dualTendonLutMotor", [0xFF, 0xFF]))
    while len(motors) < DUAL_TENDON_LUT_ACTUATOR_COUNT:
        motors.append(0xFF)
    for i in range(DUAL_TENDON_LUT_ACTUATOR_COUNT):
        payload.append(int(motors[i]) & 0xFF)
    payload.extend(_pack_float_be(float(result.get("dualTendonLutAngleStartDeg", 0.0))))
    payload.extend(_pack_float_be(float(result.get("dualTendonLutAngleStepDeg", 0.0))))
    motor_abs = result.get("dualTendonLutMotorAbs", [])
    for actuator in range(DUAL_TENDON_LUT_ACTUATOR_COUNT):
        row = motor_abs[actuator] if actuator < len(motor_abs) else []
        for sample in range(DUAL_TENDON_LUT_MAX_SAMPLES):
            value = row[sample] if sample < len(row) else 0
            payload.extend(struct.pack(">h", _clamp_i16(value)))
    payload.append(int(result.get("mappingVersion", 0)) & 0xFF)
    payload.extend(struct.pack(">i", int(result.get("encoderAtUrdfMin", 0))))
    payload.extend(struct.pack(">i", int(result.get("encoderAtUrdfMax", 0))))
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
    payload.extend(_pack_float_be(float(result.get("currentMinDeg", 0.0))))
    payload.extend(_pack_float_be(float(result.get("currentMaxDeg", 0.0))))
    return _build_command_frame(CMD_CALIB_LOAD_JOINT, bytes(payload))


def build_host_heartbeat_cmd() -> bytes:
    """界面打开时维持下位机 host-online 计时器。"""
    return _build_command_frame(CMD_HOST_HEARTBEAT, b"")


def build_multiturn_nvs_control_cmd(enabled: bool) -> bytes:
    # False means disable and clear old snapshots; True only enables future saves.
    return _build_command_frame(CMD_MULTITURN_NVS_CONTROL, bytes([1 if enabled else 0]))


def build_clear_calibration_cmd() -> bytes:
    return _build_command_frame(CMD_CLEAR_CALIBRATION, b"")


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


def build_manual_encoder_query_cmd() -> bytes:
    return _build_command_frame(CMD_MANUAL_ENCODER_QUERY, b"")


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


def build_motor_pos_cmd(motor_pos_raw: List[int]) -> bytes:
    if len(motor_pos_raw) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor raw values")
    payload = bytearray()
    for value in motor_pos_raw:
        raw = int(value) & 0xFFFF
        payload.append((raw >> 8) & 0xFF)
        payload.append(raw & 0xFF)
    return _build_command_frame(CMD_MOTOR_POS, bytes(payload))


def build_motor_pos_sweep_cmd(motor_pos_raw: List[int]) -> bytes:
    if len(motor_pos_raw) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor raw values")
    payload = bytearray()
    for value in motor_pos_raw:
        raw = int(value) & 0xFFFF
        payload.append((raw >> 8) & 0xFF)
        payload.append(raw & 0xFF)
    return _build_command_frame(CMD_MOTOR_POS_SWEEP, bytes(payload))


# 与下位机 clampServoPos 一致的多圈绝对位置（int16 大端每路）
MOTOR_ABS_CMD_MIN = -30719
MOTOR_ABS_CMD_MAX = 30719


def build_motor_pos_abs_cmd(motor_abs: List[int]) -> bytes:
    if len(motor_abs) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor absolute values")
    payload = bytearray()
    for value in motor_abs:
        v = int(value)
        if v < MOTOR_ABS_CMD_MIN:
            v = MOTOR_ABS_CMD_MIN
        elif v > MOTOR_ABS_CMD_MAX:
            v = MOTOR_ABS_CMD_MAX
        payload.extend(struct.pack(">h", v))
    return _build_command_frame(CMD_MOTOR_POS_ABS, bytes(payload))


def build_motor_pos_abs_one_cmd(motor_index: int, motor_abs: int) -> bytes:
    idx = int(motor_index)
    if idx < 0 or idx >= MOTOR_COUNT:
        raise ValueError(f"motor index out of range: {idx}")
    v = int(motor_abs)
    if v < MOTOR_ABS_CMD_MIN:
        v = MOTOR_ABS_CMD_MIN
    elif v > MOTOR_ABS_CMD_MAX:
        v = MOTOR_ABS_CMD_MAX
    payload = bytes([idx & 0xFF]) + struct.pack(">h", v)
    return _build_command_frame(CMD_MOTOR_POS_ABS_ONE, payload)


def build_stream_mode_cmd(mode: int) -> bytes:
    mode_int = int(mode)
    if mode_int not in (SENSOR_STREAM_MODE_LEGACY_U16_WRAP, SENSOR_STREAM_MODE_SIGNED_I16):
        raise ValueError("unsupported sensor stream mode")
    return _build_command_frame(CMD_SENSOR_STREAM_MODE, bytes([mode_int & 0xFF]))


def build_local_test_params_cmd(freq_hz: float, amplitude_deg: float) -> bytes:
    payload = struct.pack("<ff", freq_hz, amplitude_deg)
    return _build_command_frame(CMD_LOCAL_TEST_PARAMS, payload)


def build_local_test_start_cmd() -> bytes:
    return _build_command_frame(CMD_LOCAL_TEST_START, b"")


def build_local_test_stop_cmd() -> bytes:
    return _build_command_frame(CMD_LOCAL_TEST_STOP, b"")
