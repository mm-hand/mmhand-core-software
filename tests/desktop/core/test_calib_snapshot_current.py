import struct

import pytest

from desktop.core.angle_mapping import urdf_manual_scaled_mapping_metadata
from desktop.core.protocol import (
    CALIB_WARN_AUTO_MAX_SHORTFALL,
    CALIB_WARN_AUTO_MIN_OFFSET,
    CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
    CMD_CALIB_LOAD_LIMITS,
    CMD_CALIB_LOAD_JOINT,
    MOTOR_COUNT,
    TENDON_LUT_ACTUATOR_COUNT,
    TENDON_LUT_MAX_SAMPLES,
    PROTOCOL_HEADER,
    PROTOCOL_TAIL,
    build_calib_load_joint_cmd,
    build_calib_load_limits_cmd,
    parse_joint_calib_snapshot_packet,
)


def test_motor_limit_load_is_scoped_to_update_bitmap():
    mins = [-1000 - i for i in range(MOTOR_COUNT)]
    maxs = [2000 + i for i in range(MOTOR_COUNT)]
    update_mask = (1 << 5) | (1 << 6)
    valid_mask = update_mask | (1 << 1)
    locked_mask = 1 << 6

    frame = build_calib_load_limits_cmd(
        update_mask,
        valid_mask,
        locked_mask,
        mins,
        maxs,
    )

    assert frame[0] == PROTOCOL_HEADER
    assert frame[2] == CMD_CALIB_LOAD_LIMITS
    assert frame[-1] == PROTOCOL_TAIL
    payload = frame[3:-1]
    assert struct.unpack(">III", payload[:12]) == (
        update_mask,
        valid_mask,
        locked_mask,
    )
    assert len(payload) == 12 + MOTOR_COUNT * 8


def test_joint_calibration_current_protocol_round_trip():
    mapping_version, mapping_crc = urdf_manual_scaled_mapping_metadata()
    result = {
        "joint": 4,
        "success": True,
        "encoderMin": 1200,
        "encoderMax": 4600,
        "offset": 1400,
        "angleMax": 63.0,
        "startEncoder": 1200,
        "tightEncoder": 4600,
        "zeroEncoder": 1400,
        "encoderSpan": 3400,
        "startServoAbs": 100,
        "tightServoAbs": 3500,
        "zeroServoAbs": 300,
        "startDeg": -5.0,
        "tightDeg": 69.7,
        "totalDeg": 74.7,
        "scaled": True,
        "encoderDirection": 1,
        "tendonLutValid": True,
        "tendonLutCount": 3,
        "tendonLutMotor": [5, 6],
        "tendonLutAngleStartDeg": -5.0,
        "tendonLutAngleStepDeg": 5.0,
        "tendonLutMotorAbs": [
            list(range(TENDON_LUT_MAX_SAMPLES))
            for _ in range(TENDON_LUT_ACTUATOR_COUNT)
        ],
        "mappingVersion": mapping_version,
        "encoderAtUrdfMin": 1200,
        "encoderAtUrdfMax": 4600,
        "angleMin": 0.0,
        "urdfConfigCrc": mapping_crc,
        "autoMinRaw": 1200,
        "autoMaxRaw": 4600,
        "manualReferenceCrc": 0x12345678,
        "currentMinDeg": 0.0,
        "currentMaxDeg": 63.0,
    }

    frame = build_calib_load_joint_cmd(result)
    assert frame[0] == PROTOCOL_HEADER
    assert frame[2] == CMD_CALIB_LOAD_JOINT
    assert frame[-1] == PROTOCOL_TAIL

    parsed = parse_joint_calib_snapshot_packet(frame[3:-1])
    assert parsed is not None
    assert parsed["joint"] == result["joint"]
    assert parsed["mappingVersion"] == mapping_version
    assert parsed["encoderAtUrdfMin"] == 1200
    assert parsed["encoderAtUrdfMax"] == 4600
    assert parsed["angleMin"] == 0.0
    assert parsed["angleMax"] == 63.0
    assert parsed["urdfConfigCrc"] == mapping_crc
    assert parsed["autoMinRaw"] == 1200
    assert parsed["autoMaxRaw"] == 4600
    assert parsed["manualReferenceCrc"] == 0x12345678
    assert parsed["currentMinDeg"] == 0.0
    assert parsed["currentMaxDeg"] == 63.0
    assert parsed["hasManualScaledV3Fields"] is True
    assert parsed["hasAssemblyWarningFields"] is False
    assert parsed["tendonLutValid"] is True
    assert parsed["tendonLutCount"] == 3
    assert parsed["tendonLutMotor"] == [5, 6]
    assert parsed["tendonLutMotorAbs"][0][:3] == [0, 1, 2]

def test_joint_calibration_preserves_large_logical_abs():
    """舵机单独掉电重建 offset 后，logical ABS 超过 int16 仍须无损恢复。"""
    mapping_version, mapping_crc = urdf_manual_scaled_mapping_metadata()
    result = {
        "joint": 4,
        "success": True,
        "encoderDirection": 1,
        "mappingVersion": mapping_version,
        "urdfConfigCrc": mapping_crc,
        "autoMinRaw": 100,
        "autoMaxRaw": 200,
        "startEncoder": 100,
        "tightEncoder": 200,
        "startServoAbs": 40000,
        "tightServoAbs": 43000,
        "zeroServoAbs": 41000,
        "tendonLutValid": True,
        "tendonLutCount": 3,
        "tendonLutMotor": [5, 6],
        "tendonLutMotorAbs": [
            [40000, 41000, 43000],
            [-42000, -41000, -39000],
        ],
    }

    frame = build_calib_load_joint_cmd(result)
    parsed = parse_joint_calib_snapshot_packet(frame[3:-1])

    assert parsed is not None
    assert parsed["startServoAbs"] == 40000
    assert parsed["tightServoAbs"] == 43000
    assert parsed["zeroServoAbs"] == 41000
    assert parsed["tendonLutMotorAbs"][0][:3] == [40000, 41000, 43000]
    assert parsed["tendonLutMotorAbs"][1][:3] == [-42000, -41000, -39000]


def test_legacy_joint_payload_lengths_are_rejected():
    assert parse_joint_calib_snapshot_packet(bytes(54)) is None
    assert parse_joint_calib_snapshot_packet(bytes(162)) is None
    assert parse_joint_calib_snapshot_packet(bytes(179)) is None
    assert parse_joint_calib_snapshot_packet(bytes(195)) is None


def test_joint_calibration_snapshot_parses_optional_warning_details():
    mapping_version, mapping_crc = urdf_manual_scaled_mapping_metadata()
    result = {
        "joint": 12,
        "success": True,
        "encoderMin": 1000,
        "encoderMax": 3400,
        "offset": 1000,
        "angleMax": 65.0,
        "startEncoder": 1000,
        "tightEncoder": 3400,
        "zeroEncoder": 1100,
        "encoderSpan": 2400,
        "startServoAbs": 100,
        "tightServoAbs": 2500,
        "zeroServoAbs": 200,
        "startDeg": 0.0,
        "tightDeg": 65.0,
        "totalDeg": 65.0,
        "scaled": True,
        "encoderDirection": 1,
        "mappingVersion": mapping_version,
        "encoderAtUrdfMin": 1000,
        "encoderAtUrdfMax": 3400,
        "angleMin": 0.0,
        "urdfConfigCrc": mapping_crc,
        "autoMinRaw": 1000,
        "autoMaxRaw": 3400,
        "manualReferenceCrc": 0x12345678,
        "currentMinDeg": 0.0,
        "currentMaxDeg": 65.0,
    }
    frame = build_calib_load_joint_cmd(result)
    payload = bytearray(frame[3:-1])
    reason = (
        CALIB_WARN_AUTO_MIN_OFFSET |
        CALIB_WARN_AUTO_MAX_SHORTFALL |
        CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH
    )
    payload.extend(struct.pack(">BhhHHHH", reason, 412, 326, 2184, 3400, 2957, 443))

    parsed = parse_joint_calib_snapshot_packet(bytes(payload))

    assert parsed is not None
    assert parsed["hasAssemblyWarningFields"] is True
    assert parsed["assemblyWarningReason"] == reason
    assert parsed["autoMinOffsetCounts"] == 412
    assert parsed["autoMaxShortfallCounts"] == 326
    assert parsed["assemblyAutoSpanCounts"] == 2184
    assert parsed["assemblyManualSpanCounts"] == 3400
    assert parsed["assemblyUrdfSpanCounts"] == 2957
    assert parsed["assemblySpanDifferenceCounts"] == 443
    assert parsed["hasAssemblySpanComparisonFields"] is True


def test_direction_negative_snapshot_requires_physical_raw_endpoints():
    mapping_version, mapping_crc = urdf_manual_scaled_mapping_metadata()
    result = {
        "joint": 14,
        "success": True,
        "encoderMin": 6398,
        "encoderMax": 10598,
        "offset": 6626,
        "angleMax": 88.0,
        "startEncoder": 6398,
        "tightEncoder": 10598,
        "zeroEncoder": 6626,
        "encoderSpan": 4200,
        "startServoAbs": 100,
        "tightServoAbs": 4300,
        "zeroServoAbs": 300,
        "startDeg": -9.0,
        "tightDeg": 88.0,
        "totalDeg": 97.0,
        "scaled": True,
        "encoderDirection": -1,
        "mappingVersion": mapping_version,
        "encoderAtUrdfMin": 6398,
        "encoderAtUrdfMax": 10598,
        "angleMin": -8.609,
        "urdfConfigCrc": mapping_crc,
        "autoMinRaw": 9986,
        "autoMaxRaw": 5786,
        "manualReferenceCrc": 0x12345678,
        "currentMinDeg": -8.609,
        "currentMaxDeg": 88.0,
    }

    frame = build_calib_load_joint_cmd(result)
    assert frame[2] == CMD_CALIB_LOAD_JOINT

    result["autoMinRaw"] = 6398
    result["autoMaxRaw"] = 10598
    with pytest.raises(ValueError, match="endpoint contract mismatch"):
        build_calib_load_joint_cmd(result)
