"""Original P4 V4 mapping and raw-sample LUT arithmetic regression tests."""

import math
import struct
import zlib

import pytest

from desktop.core.host_calibration.config import (
    AutoCalibrationParams, kTendonRawSampleCapacity,
)
from desktop.core.host_calibration.math import (
    TendonRawFitBuffer, build_calibration_result, build_tendon_lut,
    encoder_count_to_deg, f32,
    forward_encoder_delta, manual_entry_crc, map_manual_scaled_encoder_to_urdf,
    round_to_int32, tendon_lut_sample_angle, tendon_motor_abs_limits,
    wrap_encoder_delta,
)


class MathIO:
    def __init__(self, limits=(0.0, 83.0), endpoints=(16000, 4000)):
        self._limits = limits
        self.reference = {"valid": True, "minRaw": endpoints[0], "maxRaw": endpoints[1]}

    def urdf_limits(self, joint):
        return self._limits

    def urdf_config_crc(self):
        return 0x2BC44010

    def manual_reference(self, joint):
        return self.reference


def test_original_default_parameters():
    assert vars(AutoCalibrationParams()) == {
        "tightenSpeedCountsPerSec": 400, "servoSpeed": 400, "servoAcc": 20,
        "loopMs": 20, "windowMs": 200, "traceMs": 200,
        "windowServoDeltaCounts": 50, "zeroReachAbsTolerance": 200,
        "stallActualDeg": 0.75, "tightLoadHigh": 250,
    }


def test_forward_encoder_delta_cross_wrap():
    assert forward_encoder_delta(16000, 400) == 784
    assert forward_encoder_delta(400, 16000) == 15600
    assert wrap_encoder_delta(8192, 0) == 8192
    assert wrap_encoder_delta(0, 8192) == -8192


def test_rounding_and_encoder_degrees_match_single_precision_p4():
    assert round_to_int32(2.5) == 3
    assert round_to_int32(-2.5) == -3
    assert encoder_count_to_deg(34) == f32(f32(f32(34) * 360.0) / 16384.0)
    assert encoder_count_to_deg(34) <= 0.75
    assert encoder_count_to_deg(35) > 0.75


def test_manual_crc_matches_big_endian_five_byte_contract():
    io = MathIO()
    assert manual_entry_crc(1, io.reference) == zlib.crc32(struct.pack(">BHH", 1, 16000, 4000))


def test_full_manual_mapping_preserved_and_auto_endpoints_do_not_rescale():
    io = MathIO()
    result = build_calibration_result(io, 1, 16200, 0, 3500, -100, 400, 2000)
    assert result["mappingVersion"] == 4
    # host 结果只保留静态 URDF/angle 字段，currentMin/Max 概念已删除。
    assert (result["angleMin"], result["angleMax"]) == (0.0, 83.0)
    assert "currentMinDeg" not in result
    assert "currentMaxDeg" not in result
    assert result["autoMinRaw"] == 16200
    assert result["autoMaxRaw"] == 3500
    assert result["manualSpanCounts"] == 4384
    assert result["autoSpanCounts"] == 3684
    assert result["zeroEncoder"] != 0
    at_min = map_manual_scaled_encoder_to_urdf(io, 1, result, 16000)
    at_max = map_manual_scaled_encoder_to_urdf(io, 1, result, 4000)
    assert at_min["urdf_deg"] == 0.0
    assert at_max["urdf_deg"] == 83.0
    below = map_manual_scaled_encoder_to_urdf(io, 1, result, 15990)
    above = map_manual_scaled_encoder_to_urdf(io, 1, result, 4010)
    assert below["outside_counts"] == -10 and not below["feedback_ready"]
    assert above["outside_counts"] == 10 and above["feedback_ready"]
    assert below["urdf_deg"] == encoder_count_to_deg(-10)
    assert above["urdf_deg"] == f32(83.0 + encoder_count_to_deg(10))
    io.reference["maxRaw"] = 4001
    assert map_manual_scaled_encoder_to_urdf(io, 1, result, 4000) is None


def test_buffer_capacity_sampling_threshold_and_duplicate_order():
    raw = TendonRawFitBuffer()
    assert raw.append(16000, 0, 0)
    raw.next_travel_deg = 5.0
    raw.maybe_append(4.89, 16100, 50)
    assert raw.count == 1
    raw.maybe_append(4.9, 16100, 50)
    assert raw.count == 2 and raw.next_travel_deg == 10.0
    raw.maybe_append(21.0, 16300, 100)
    assert raw.count == 3 and raw.next_travel_deg == 25.0
    raw.maybe_append(math.nan, 0, 200)
    assert raw.count == 3
    for index in range(kTendonRawSampleCapacity - raw.count):
        assert raw.append(index, index, index)
    assert raw.count == 72
    last = raw.samples[-1]
    assert not raw.append(last.encoder, last.primary_abs, last.secondary_abs)


def test_build_tendon_lut_keeps_the_full_5_degree_table():
    io = MathIO(limits=(0.0, 83.0), endpoints=(16000, 4000))
    result = build_calibration_result(io, 1, 16000, 0, 4000, 0, 1000, 2000)
    assert result
    raw = TendonRawFitBuffer()
    # 物理 raw 16000/1808/4000 对应 URDF 0°/41.5°/83°。
    for encoder, primary in ((16000, 0), (1808, 1000), (4000, 2000)):
        assert raw.append(encoder, primary, 0)
    assert build_tendon_lut(io, 1, result, 2, 255, 1, raw)
    assert result["tendonLutValid"] is True
    assert result["tendonLutCount"] == 18
    assert result["tendonLutAngleStartDeg"] == 0.0
    assert result["tendonLutAngleStepDeg"] == 5.0
    row = result["tendonLutMotorAbs"][0]
    # 线性样本 -> 每 5° 一个点；末点 85° 按静态 angleMax 83° 解释。
    assert row[0] == 0
    assert row[8] == 964
    assert row[17] == 2000
    assert result["tendonLutMotorAbs"][1] == [0] * 24
    assert tendon_lut_sample_angle(result, 1) == 5.0
    assert tendon_lut_sample_angle(result, 17) == 83.0


def test_tendon_limits_use_full_table_endpoints_with_margins():
    # 两端陡、中间平的整表（J03 实测形状）：工作空间取整表端点，不取中段直线。
    result = {
        "success": True,
        "scaled": True,
        "tendonLutValid": True,
        "tendonLutCount": 5,
        "tendonLutAngleStartDeg": 0.0,
        "tendonLutAngleStepDeg": 5.0,
        "angleMin": 0.0,
        "angleMax": 20.0,
        "tendonLutMotorAbs": [[7600, 8100, 8900, 11200, 11900], [0] * 24],
    }
    assert tendon_motor_abs_limits(result, 0, 0, 500, 1000) == (7100, 12900)
    assert tendon_motor_abs_limits(result, 0, 1, 500, 1000) == (7100, 12900)
    result["tendonLutMotorAbs"][0] = [11900, 11200, 8900, 8100, 7600]
    assert tendon_motor_abs_limits(result, 0, 0, 500, 1000) == (7100, 12900)


def test_tendon_limits_extend_past_fitted_line_instead_of_measured_tight_stop():
    result = {
        "success": True,
        "scaled": True,
        "angleMin": 0.0,
        "angleMax": 83.0,
        "tendonLutValid": True,
        "tendonLutCount": 2,
        "tendonLutAngleStartDeg": 0.0,
        "tendonLutAngleStepDeg": 83.0,
        "tendonLutMotorAbs": [[4000, 14000], [0, 0]],
    }
    # 不记录实测 tight 停点：两端都按拟合直线外延同样的计数。
    assert tendon_motor_abs_limits(result, 0, 2, 500, 1000) == (3500, 15000)
    assert tendon_motor_abs_limits(result, 0, 3, 500, 1000) == (3500, 15000)


def test_auto_result_rejects_short_and_near_full_circle_span():
    io = MathIO()
    assert build_calibration_result(io, 1, 0, 0, 100, 0, 500, 1000) is None
    assert build_calibration_result(io, 1, 0, 0, 16340, 0, 500, 1000) is None
