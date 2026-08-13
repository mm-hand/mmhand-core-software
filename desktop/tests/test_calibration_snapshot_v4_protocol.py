"""锁定当前唯一 Auto/LUT V4 快照长度，防止旧映射字段重新进入运行协议。"""

from desktop.core.generated_upper_protocol import (
    CMD_CALIB_LOAD_JOINT,
    COMMAND_PAYLOAD_LENGTHS,
)
from desktop.core.protocol import (
    build_calib_load_joint_cmd,
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
