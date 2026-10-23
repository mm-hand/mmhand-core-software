"""锁定当前唯一 Auto/LUT V4 快照长度，防止旧映射字段重新进入运行协议。"""

import struct

from desktop.core.generated_upper_protocol import (
    CMD_CALIB_LOAD_JOINT,
    COMMAND_PAYLOAD_LENGTHS,
)
from desktop.core.protocol import (
    build_calib_load_joint_cmd,
    manual_encoder_reference_crc,
    manual_encoder_reference_entry_crc,
    parse_joint_calib_snapshot_packet,
)


def _minimal_v4_result() -> dict:
    return {
        "joint": 0,
        "success": True,
        "mappingVersion": 4,
        "angleMax": 11.0,
        "angleMin": -45.0,
        "startEncoder": 100,
        "tightEncoder": 200,
        "zeroEncoder": 100,
        "encoderSpan": 100,
        "startServoAbs": 1000,
        "tightServoAbs": 1200,
        "zeroServoAbs": 1000,
        "startDeg": 0.0,
        "tightDeg": 90.0,
        "totalDeg": 90.0,
        "scaled": True,
        "encoderDirection": 1,
        "tendonLutValid": False,
        "tendonLutCount": 0,
        "tendonLutMotor": [0xFF, 0xFF],
        "tendonLutAngleStartDeg": 0.0,
        "tendonLutAngleStepDeg": 0.0,
        "tendonLutMotorAbs": [[], []],
        "urdfConfigCrc": 1,
        "autoMinRaw": 100,
        "autoMaxRaw": 200,
        "manualReferenceCrc": 2,
        "currentMinDeg": -45.0,
        "currentMaxDeg": 11.0,
    }


def test_calib_load_joint_uses_only_v4_payload() -> None:
    frame = build_calib_load_joint_cmd(_minimal_v4_result())

    assert COMMAND_PAYLOAD_LENGTHS[CMD_CALIB_LOAD_JOINT] == 189
    assert len(frame) == 193
    assert len(frame[3:-1]) == 189


def test_snapshot_parser_rejects_removed_v2_lengths() -> None:
    assert parse_joint_calib_snapshot_packet(bytes(209)) is None
    assert parse_joint_calib_snapshot_packet(bytes(222)) is None


def test_manual_entry_crc_matches_p4_contract_vectors() -> None:
    """锁定 P4 与 desktop 共用的 IEEE CRC32，禁止退化成普通哈希。"""
    assert manual_encoder_reference_entry_crc(14, 1793, 13972) == 2608455130
    assert manual_encoder_reference_entry_crc(15, 8182, 13553) == 2362513690

    mins = [0] * 21
    maxs = [0] * 21
    mins[12:16] = [14047, 11518, 1793, 8182]
    maxs[12:16] = [11152, 6831, 13972, 13553]
    assert manual_encoder_reference_crc(mins, maxs) == 0x0D47316D


def test_full_5_degree_lut_round_trips_through_the_v4_load_frame() -> None:
    result = _minimal_v4_result()
    rows = [[1000 + 37 * index for index in range(24)],
            [5000 - 41 * index for index in range(24)]]
    result.update(
        tendonLutValid=True,
        tendonLutCount=24,
        tendonLutMotor=[0, 1],
        tendonLutAngleStartDeg=-45.0,
        tendonLutAngleStepDeg=5.0,
        tendonLutMotorAbs=rows,
    )
    payload = build_calib_load_joint_cmd(result)[3:-1]
    assert len(payload) == 189
    # 布局：... scaled(1) direction(1) lutValid(1) count(1) motors(2) start(4) step(4)
    count_offset = 1 + 1 + 4 + 4 * 4 + 3 * 4 + 3 * 4 + 1 + 1 + 1
    assert payload[count_offset] == 24
    assert payload[count_offset + 1] == 0 and payload[count_offset + 2] == 1
    # LUT 块 = 2 路 × (int32 anchor + 24 × int16 delta)，其后是 25 字节尾部。
    block = payload[len(payload) - 25 - 2 * 52:len(payload) - 25]
    for actuator in range(2):
        base = actuator * 52
        anchor = struct.unpack(">i", block[base:base + 4])[0]
        decoded = [
            anchor + struct.unpack(">h", block[base + 4 + 2 * index:base + 6 + 2 * index])[0]
            for index in range(24)
        ]
        assert decoded == rows[actuator]
