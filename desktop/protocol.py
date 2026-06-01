import struct
from enum import Enum
from typing import List, Optional, Tuple


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
PACKET_TYPE_RELEASE_FAULT = 0x08
PACKET_TYPE_SERVO_RAW = 0x09
PACKET_TYPE_TACTILE = 0x0A
PACKET_TYPE_ENCODER_RAW = 0x0B
PACKET_TYPE_SYSTEM_STATUS = 0x0C
PACKET_TYPE_MOTOR_ABS_LIMIT = 0x0D

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
CMD_LOCAL_TEST_PARAMS = 0xD4
CMD_LOCAL_TEST_START = 0xD5
CMD_LOCAL_TEST_STOP = 0xD6
CMD_ACTIVE_SCOPE = 0xD7
CMD_RUN_FEEDBACK = 0xD8
CMD_PAUSE = 0xD9
CMD_RESTORE_MULTITURN = 0xDA
CMD_MOTOR_ABS_LIMIT_LOCK = 0xDB
# 上位机在线心跳；固件用它判断桌面端是否已经关闭。
CMD_HOST_HEARTBEAT = 0xDC
CMD_MULTITURN_NVS_CONTROL = 0xDD

SENSOR_STREAM_MODE_LEGACY_U16_WRAP = 0
SENSOR_STREAM_MODE_SIGNED_I16 = 1

PROTO_ACK_STATUS_OK = 0
PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1
PROTO_ACK_STATUS_QUEUED = 2

SYSTEM_STATE_UNCONTROLLABLE = 0
SYSTEM_STATE_IDLE = 1
SYSTEM_STATE_RUN_FEEDBACK = 2
SYSTEM_STATE_CALIBRATION_RUNNING = 4
SYSTEM_STATE_READY = 5
SYSTEM_STATE_RUN_DIRECT = 6
SYSTEM_STATE_PAUSED = 7

SYSTEM_FAULT_CAN_OFFLINE = 1 << 0
SYSTEM_FAULT_JOINT16_DUAL = 1 << 1
SYSTEM_FAULT_OVERLOAD = 1 << 2
SYSTEM_FAULT_RELEASE_GUARD = 1 << 3
SYSTEM_FAULT_SERVO_OFFLINE = 1 << 4
SYSTEM_FAULT_CALIBRATION = 1 << 5

SERVO_TARGET_OWNER_NONE = 0
SERVO_TARGET_OWNER_CONTROL = 1
SERVO_TARGET_OWNER_CALIBRATION = 2

# Decoupled dimensions
ENCODER_COUNT = 21
MOTOR_COUNT = 27
TACTILE_GROUP_COUNT = 5
TACTILE_SENSOR_PER_GROUP = 3
TACTILE_AXIS_COUNT = 3
TACTILE_SUMMARY_BYTES = TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_COUNT
HAND_PROFILE = "v2"
VALID_MOTOR_COUNT_V1 = 22
VALID_MOTOR_MASK_V1 = (1 << VALID_MOTOR_COUNT_V1) - 1
VALID_MOTOR_MASK_V2 = (1 << MOTOR_COUNT) - 1
VALID_MOTOR_MASK = VALID_MOTOR_MASK_V2 if HAND_PROFILE.lower() == "v2" else VALID_MOTOR_MASK_V1

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


def parse_system_status_packet(payload: bytes) -> Optional[Tuple[int, int, int, int, int, int, int, int, int, int]]:
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
        # Backward compatibility with older firmware: valid limits were always locked.
        locked_bitmap = valid_bitmap
        offset = 4
    mins: List[int] = []
    maxs: List[int] = []
    for _ in range(MOTOR_COUNT):
        mins.append(struct.unpack(">h", payload[offset:offset + 2])[0])
        maxs.append(struct.unpack(">h", payload[offset + 2:offset + 4])[0])
        offset += 4
    return valid_bitmap, locked_bitmap, mins, maxs


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


def parse_servo_raw_packet(payload: bytes) -> Optional[Tuple[List[int], List[bool]]]:
    if not payload or (len(payload) % 3) != 0:
        return None

    channel_count = len(payload) // 3
    if channel_count <= 0:
        return None

    raw_positions: List[int] = []
    for i in range(channel_count):
        start = i * 2
        raw_positions.append(struct.unpack(">h", payload[start:start + 2])[0])

    online_flags: List[bool] = []
    base = channel_count * 2
    for i in range(channel_count):
        online_flags.append(payload[base + i] == 1)
    return raw_positions, online_flags


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


def parse_release_fault_packet(payload: bytes) -> Optional[int]:
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


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


def build_host_heartbeat_cmd() -> bytes:
    """界面打开时维持下位机 host-online 计时器。"""
    return _build_command_frame(CMD_HOST_HEARTBEAT, b"")


def build_multiturn_nvs_control_cmd(enabled: bool) -> bytes:
    # False means disable and clear old snapshots; True only enables future saves.
    return _build_command_frame(CMD_MULTITURN_NVS_CONTROL, bytes([1 if enabled else 0]))


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
