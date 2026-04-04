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

# Downstream commands (desktop -> device)
CMD_CALIBRATE = 0xCA
CMD_ANGLE_CTRL = 0xCB
CMD_START = 0xCC
CMD_STOP = 0xCD
CMD_RESET = 0xCE
CMD_CALIB_DATA = 0xCF
CMD_MOTOR_POS = 0xD0
CMD_SENSOR_STREAM_MODE = 0xD1

SENSOR_STREAM_MODE_LEGACY_U16_WRAP = 0
SENSOR_STREAM_MODE_SIGNED_I16 = 1

PROTO_ACK_STATUS_OK = 0
PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1

# Decoupled dimensions
ENCODER_COUNT = 21
MOTOR_COUNT = 22

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
    expected_len = MOTOR_COUNT * 4 + MOTOR_COUNT
    if len(payload) != expected_len:
        return None

    angles: List[int] = []
    for i in range(MOTOR_COUNT):
        start = i * 4
        angles.append(struct.unpack(">i", payload[start:start + 4])[0])

    online_flags: List[bool] = []
    base = MOTOR_COUNT * 4
    for i in range(MOTOR_COUNT):
        online_flags.append(payload[base + i] == 1)
    return angles, online_flags


def parse_servo_telem_packet(
    payload: bytes,
) -> Optional[Tuple[List[int], List[int], List[int], List[int], List[bool]]]:
    expected_len = MOTOR_COUNT * 7
    if len(payload) != expected_len:
        return None

    speeds: List[int] = []
    loads: List[int] = []
    volts: List[int] = []
    temps: List[int] = []
    online: List[bool] = []

    for i in range(MOTOR_COUNT):
        base = i * 7
        speeds.append(struct.unpack(">h", payload[base:base + 2])[0])
        loads.append(struct.unpack(">h", payload[base + 2:base + 4])[0])
        volts.append(payload[base + 4])
        temps.append(payload[base + 5])
        online.append(payload[base + 6] == 1)
    return speeds, loads, volts, temps, online


def parse_fault_status_packet(payload: bytes) -> Optional[int]:
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


def parse_release_fault_packet(payload: bytes) -> Optional[int]:
    if len(payload) != 4:
        return None
    return struct.unpack(">I", payload)[0]


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
    return bytes([CMD_CALIBRATE])


def build_start_cmd() -> bytes:
    return bytes([CMD_START])


def build_stop_cmd() -> bytes:
    return bytes([CMD_STOP])


def build_reset_cmd() -> bytes:
    return bytes([CMD_RESET])


def build_calib_data_cmd(zero_encoder_raw: List[int]) -> bytes:
    if len(zero_encoder_raw) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} encoder zero values")
    vals = [float(v) for v in zero_encoder_raw]
    payload = struct.pack(f"<{ENCODER_COUNT}f", *vals)
    return bytes([CMD_CALIB_DATA]) + payload


def build_angle_cmd(angles: List[float]) -> bytes:
    if len(angles) != ENCODER_COUNT:
        raise ValueError(f"need exactly {ENCODER_COUNT} joint angles")
    payload = struct.pack(f"<{ENCODER_COUNT}f", *angles)
    return bytes([CMD_ANGLE_CTRL]) + payload


def build_motor_pos_cmd(motor_pos_raw: List[int]) -> bytes:
    if len(motor_pos_raw) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} motor raw values")
    payload = bytearray()
    for value in motor_pos_raw:
        raw = int(value) & 0xFFFF
        payload.append((raw >> 8) & 0xFF)
        payload.append(raw & 0xFF)
    return bytes([CMD_MOTOR_POS]) + bytes(payload)


def build_stream_mode_cmd(mode: int) -> bytes:
    mode_int = int(mode)
    if mode_int not in (SENSOR_STREAM_MODE_LEGACY_U16_WRAP, SENSOR_STREAM_MODE_SIGNED_I16):
        raise ValueError("unsupported sensor stream mode")
    return bytes([CMD_SENSOR_STREAM_MODE, mode_int & 0xFF])
