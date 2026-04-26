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
CMD_TENDON_GUARD = 0xD4

SENSOR_STREAM_MODE_LEGACY_U16_WRAP = 0
SENSOR_STREAM_MODE_SIGNED_I16 = 1

PROTO_ACK_STATUS_OK = 0
PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1

# Decoupled dimensions
ENCODER_COUNT = 21
MOTOR_COUNT = 22
TACTILE_GROUP_COUNT = 5
TACTILE_SENSOR_PER_GROUP = 3
TACTILE_AXIS_COUNT = 3
TACTILE_SUMMARY_BYTES = TACTILE_GROUP_COUNT * TACTILE_SENSOR_PER_GROUP * TACTILE_AXIS_COUNT

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


def parse_calib_ack(payload: bytes) -> Optional[int]:
    if len(payload) < 1:
        return None
    return payload[0]


def parse_servo_angle_packet(payload: bytes) -> Optional[Tuple[List[int], List[bool]]]:
    # Some firmware revisions upload 21 channels while newer desktop code expects 22.
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


def parse_fault_status_packet(payload: bytes) -> Optional[int]:
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


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
) -> Optional[Tuple[int, bool, float, float, float, float, float, bool, int]]:
    legacy_len = 2 + 5 * 4
    extended_len = legacy_len + 1 + 2
    if len(payload) not in (legacy_len, extended_len):
        return None

    joint_index, valid, target_deg, actual_deg, loop1_out, loop2_act, loop2_out = struct.unpack(
        ">BBfffff", payload[:legacy_len]
    )
    if len(payload) == extended_len:
        cmd_valid = payload[legacy_len] == 1
        cmd_target_pos = struct.unpack(">h", payload[legacy_len + 1 : legacy_len + 3])[0]
    else:
        cmd_valid = False
        cmd_target_pos = 0

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


def build_angle_cmd(angles: List[float]) -> bytes:
    if len(angles) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} joint angles")
    payload = struct.pack(f"<{ENCODER_COUNT}f", *angles)
    return _build_command_frame(CMD_ANGLE_CTRL, payload)


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


def build_tendon_guard_cmd(configs: List[Tuple[bool, int, int]]) -> bytes:
    """
    Build CMD_TENDON_GUARD payload.

    Per joint (4 bytes):
    - enabled: uint8 (0/1)
    - sign: int8 (+1 / -1, other values normalized)
    - x1_abs: int16 big-endian, clamped to motor absolute command range
    """
    if len(configs) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} tendon guard configs")

    payload = bytearray()
    for enabled, sign_raw, x1_raw in configs:
        enabled_u8 = 1 if bool(enabled) else 0
        sign_i8 = 1 if int(sign_raw) >= 0 else -1
        x1 = int(x1_raw)
        if x1 < MOTOR_ABS_CMD_MIN:
            x1 = MOTOR_ABS_CMD_MIN
        elif x1 > MOTOR_ABS_CMD_MAX:
            x1 = MOTOR_ABS_CMD_MAX

        payload.append(enabled_u8 & 0xFF)
        payload.extend(struct.pack("b", sign_i8))
        payload.extend(struct.pack(">h", x1))

    return _build_command_frame(CMD_TENDON_GUARD, bytes(payload))
