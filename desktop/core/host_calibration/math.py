"""math.py - MMHand Desktop 模块契约。

职责：镜像 P4 CalibrationMath.cpp、CalibrationLutBuilder.cpp 的纯数学与 LUT。
线程：无线程；采样缓冲由一个校准流程独占。
输入：14 bit 磁编、logical ABS 样本、Manual 端点和生成的 URDF 契约。
输出：与原通信协议相同的 V4 标定结果字典。
状态所有权：仅修改调用者传入的结果及本地采样缓冲。
禁止依赖：禁止依赖 NiceGUI/UI、串口、ROS2 或 Replay。
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass, field

from .config import (
    ENCODER_COUNT, TENDON_LUT_MAX_SAMPLES, kTendonRawSampleCapacity,
    kTendonFitSampleStepDeg, kMinTravelDeg, kMaxTravelDeg, kLutStartDeg,
    kLutZeroDeg, kCalibrationEncoderModulo, kCalibrationEncoderHalfTurn,
)


def f32(value: float) -> float:
    """在原 C++ float 运算边界取单精度，避免阈值和舍入偏移。"""
    return struct.unpack("=f", struct.pack("=f", float(value)))[0]


def round_to_int32(value: float) -> int:
    value = f32(value)
    return int(f32(value + (0.5 if value >= 0.0 else -0.5)))


def wrap_encoder_count(value: int) -> int:
    return int(value) % kCalibrationEncoderModulo


def orient_encoder_raw(raw: int, direction: int) -> int:
    value = int(raw) & (kCalibrationEncoderModulo - 1)
    return (-value if direction < 0 else value) & (kCalibrationEncoderModulo - 1)


def forward_encoder_delta(start: int, end: int) -> int:
    return wrap_encoder_count(end - start)


def directed_encoder_delta(start_raw: int, end_raw: int, direction: int) -> int:
    if direction not in (-1, 1):
        return -1
    start, end = start_raw & 16383, end_raw & 16383
    return wrap_encoder_count((end - start) if direction > 0 else (start - end))


def wrap_encoder_delta(target: int, actual: int) -> int:
    delta = target - actual
    while delta > kCalibrationEncoderHalfTurn:
        delta -= kCalibrationEncoderModulo
    while delta < -kCalibrationEncoderHalfTurn:
        delta += kCalibrationEncoderModulo
    return delta


def encoder_count_to_deg(count: int) -> float:
    return f32(f32(f32(count) * 360.0) / f32(kCalibrationEncoderModulo))


def deg_to_encoder_count(deg: float) -> int:
    scaled = f32(f32(f32(deg) * f32(kCalibrationEncoderModulo)) / 360.0)
    return round_to_int32(scaled)


def signed_encoder_delta(start: int, end: int) -> int:
    return wrap_encoder_delta(end, start)


def encoder_for_positive_travel(encoder: int, travel_sign: int) -> int:
    return wrap_encoder_count(encoder if travel_sign >= 0 else kCalibrationEncoderModulo - encoder)


def manual_entry_crc(joint: int, reference: dict) -> int:
    lo, hi = _manual_endpoints(reference)
    return zlib.crc32(struct.pack(">BHH", joint, lo, hi)) & 0xFFFFFFFF


def _manual_endpoints(reference: dict) -> tuple[int, int]:
    return (int(reference.get("minRaw", reference.get("min_raw", -1))),
            int(reference.get("maxRaw", reference.get("max_raw", -1))))


def apply_manual_scaled_mapping(io, joint: int, result: dict,
                                auto_min_raw: int, auto_max_raw: int,
                                direction: int) -> bool:
    manual = io.manual_reference(joint)
    lo, hi = _manual_endpoints(manual)
    if (not 0 <= joint < ENCODER_COUNT or not manual.get("valid", False)
            or not all(0 <= value < 16384 for value in (lo, hi, auto_min_raw, auto_max_raw))):
        return False
    manual_span = directed_encoder_delta(lo, hi, direction)
    auto_span = directed_encoder_delta(auto_min_raw, auto_max_raw, direction)
    if not 0 < manual_span < 16384 or not 0 < auto_span < 16384:
        return False
    urdf_min, urdf_max = (f32(v) for v in io.urdf_limits(joint))
    span = f32(urdf_max - urdf_min)
    per_count = f32(span / f32(manual_span))
    if (not all(math.isfinite(v) for v in (span, per_count, urdf_min, urdf_max))
            or span <= 0.0 or per_count <= 0.0):
        return False
    result.update(mappingVersion=4, autoMinRaw=auto_min_raw, autoMaxRaw=auto_max_raw,
                  manualSpanCounts=manual_span, autoSpanCounts=auto_span,
                  manualReferenceCrc=manual_entry_crc(joint, manual),
                  angleMin=urdf_min, angleMax=urdf_max,
                  urdfConfigCrc=int(io.urdf_config_crc()), encoderDirection=direction)
    return True


def map_manual_scaled_encoder_to_urdf(io, joint: int, result: dict,
                                    raw: int, direction: int) -> dict | None:
    """原 mapManualScaledEncoderToUrdf；Auto 不参与反馈角比例。"""
    if not 0 <= joint < ENCODER_COUNT:
        return None
    manual = io.manual_reference(joint)
    lo, hi = _manual_endpoints(manual)
    urdf_min, urdf_max = (f32(v) for v in io.urdf_limits(joint))
    stored_span = int(result.get("manualSpanCounts", 0))
    if (result.get("mappingVersion") != 4
            or int(result.get("urdfConfigCrc", -1)) != int(io.urdf_config_crc())
            or not manual.get("valid", False)
            or not all(0 <= value < 16384 for value in (lo, hi, raw))
            or not 0 < stored_span < 16384):
        return None
    # host 结果只保留静态 URDF/angle 字段；旧结果里多余的 currentMinDeg/
    # currentMaxDeg 不再参与反馈映射校验。
    values = [float(result.get(key, math.nan))
              for key in ("angleMin", "angleMax")]
    if (not all(math.isfinite(v) for v in values)
            or values[1] <= values[0]
            or any(abs(f32(v - expected)) > f32(0.01)
                   for v, expected in zip(values, (urdf_min, urdf_max)))):
        return None
    span = directed_encoder_delta(lo, hi, direction)
    if not 0 < span < 16384 or span != stored_span:
        return None
    forward = directed_encoder_delta(lo, raw, direction)
    coordinate = forward
    if forward > span:
        above, below = forward - span, 16384 - forward
        coordinate = -below if below < above else forward
    below = max(0, -coordinate)
    above = max(0, coordinate - span)
    if coordinate < 0:
        mapped = f32(urdf_min + encoder_count_to_deg(coordinate))
    elif coordinate <= span:
        deg_per_count = f32(f32(urdf_max - urdf_min) / f32(span))
        mapped = f32(urdf_min + f32(f32(coordinate) * deg_per_count))
    else:
        mapped = f32(urdf_max + encoder_count_to_deg(coordinate - span))
    if not math.isfinite(mapped):
        return None
    return {"valid": True, "feedback_ready": below == 0,
            "outside_counts": -below if below else above,
            "below_min_counts": below, "above_max_counts": above, "urdf_deg": mapped}


@dataclass
class TendonRawFitSample:
    encoder: int
    primary_abs: int
    secondary_abs: int


@dataclass
class TendonRawFitBuffer:
    samples: list[TendonRawFitSample] = field(default_factory=list)
    next_travel_deg: float = 0.0

    @property
    def count(self) -> int:
        return len(self.samples)

    def reset(self) -> None:
        self.samples.clear()
        self.next_travel_deg = 0.0

    def append(self, encoder: int, primary_abs: int, secondary_abs: int = 0) -> bool:
        # 原函数先判容量，再判重复；缓冲满时重复末项也返回 false。
        if self.count >= kTendonRawSampleCapacity:
            return False
        if self.samples:
            previous = self.samples[-1]
            if (previous.encoder, previous.primary_abs, previous.secondary_abs) == (
                    encoder, primary_abs, secondary_abs):
                return True
        self.samples.append(TendonRawFitSample(wrap_encoder_count(encoder), primary_abs, secondary_abs))
        return True

    def maybe_append(self, travel_deg: float, encoder: int,
                     primary_abs: int, secondary_abs: int = 0) -> None:
        travel_deg = f32(travel_deg)
        if not math.isfinite(travel_deg) or f32(travel_deg + f32(0.1)) < self.next_travel_deg:
            return
        while f32(travel_deg + f32(0.1)) >= self.next_travel_deg:
            self.next_travel_deg = f32(self.next_travel_deg + kTendonFitSampleStepDeg)
        self.append(encoder, primary_abs, secondary_abs)


def build_calibration_result(io, joint: int, taut_encoder: int,
                             zero_encoder: int, tight_encoder: int,
                             taut_abs: int, zero_abs: int, tight_abs: int) -> dict | None:
    # 与 P4 相同，zero_encoder 输入保留接口但用跨度公式重算。
    del zero_encoder
    span = forward_encoder_delta(taut_encoder, tight_encoder)
    total_deg = encoder_count_to_deg(span)
    if (span <= 0 or not math.isfinite(total_deg)
            or total_deg < kMinTravelDeg or total_deg >= kMaxTravelDeg):
        return None
    zero_fraction = f32(f32(kLutZeroDeg - kLutStartDeg) / total_deg)
    if zero_fraction <= 0.0 or zero_fraction >= 1.0:
        return None
    computed_zero = wrap_encoder_count(taut_encoder + round_to_int32(f32(f32(span) * zero_fraction)))
    result = {"joint": joint, "success": True, "scaled": True,
              "startEncoder": taut_encoder, "tightEncoder": tight_encoder,
              "zeroEncoder": computed_zero, "encoderSpan": span,
              "startServoAbs": taut_abs, "tightServoAbs": tight_abs,
              "zeroServoAbs": zero_abs, "startDeg": kLutStartDeg,
              "tightDeg": f32(kLutStartDeg + total_deg), "totalDeg": total_deg,
              "angleMax": f32(kLutStartDeg + total_deg)}
    direction = io.direction(joint)
    auto_min = orient_encoder_raw(wrap_encoder_count(taut_encoder), direction)
    auto_max = orient_encoder_raw(wrap_encoder_count(tight_encoder), direction)
    if not apply_manual_scaled_mapping(io, joint, result, auto_min, auto_max, direction):
        return None
    return result


def _interpolate_abs(target: float, samples: list[tuple[float, int, int]], actuator: int) -> int | None:
    if len(samples) < 2:
        return None
    if target <= samples[0][0]:
        left, right = samples[:2]
    else:
        left, right = samples[-2:]
        for index in range(1, len(samples)):
            if target <= samples[index][0]:
                left, right = samples[index - 1:index + 1]
                break
    span = f32(right[0] - left[0])
    if span <= 0:
        return None
    # C++ 边界外用 slope，区间内先计算 t；保留两种计算顺序。
    if target <= samples[0][0] or target > samples[-1][0]:
        anchor = left if target <= samples[0][0] else right
        slope = f32(f32(f32(right[actuator]) - f32(left[actuator])) / span)
        value = f32(f32(anchor[actuator]) + f32(f32(target - anchor[0]) * slope))
    else:
        t = f32(f32(target - left[0]) / span)
        value = f32(f32(left[actuator]) + f32(f32(f32(right[actuator]) - f32(left[actuator])) * t))
    return round_to_int32(value)


def tendon_lut_sample_angle(result: dict, index: int) -> float:
    """Return one LUT sample angle; the remainder point is the static URDF Max.

    Mirrors the P4 ``tendonLutSampleAngleDeg`` rule so the host and firmware
    never disagree about the end sample of the 5-degree table.
    """
    first = f32(result.get("tendonLutAngleStartDeg", 0.0))
    step = f32(result.get("tendonLutAngleStepDeg", 0.0))
    nominal = f32(first + f32(step * f32(index)))
    angle_max = f32(result.get("angleMax", math.nan))
    if (index == int(result.get("tendonLutCount", 0)) - 1
            and math.isfinite(angle_max) and first < angle_max < nominal):
        return angle_max
    return nominal


def extend_motor_abs_limits(motor: int, lo: int, hi: int,
                           release_margin: int, tight_margin: int) -> tuple[int, int] | None:
    """Extend an ABS envelope outward by the two configured margins."""
    from .config import shorten_sign, motor_apply_tendon_delta
    sign = shorten_sign(motor)
    if not sign or lo > hi:
        return None
    release_abs = lo if sign > 0 else hi
    tight_abs = hi if sign > 0 else lo
    release_with_margin = motor_apply_tendon_delta(motor, release_abs, -release_margin)
    tight_with_margin = motor_apply_tendon_delta(motor, tight_abs, tight_margin)
    return min(release_with_margin, tight_with_margin), max(release_with_margin, tight_with_margin)


def tendon_motor_abs_limits(result: dict, actuator: int, motor: int,
                            release_margin: int, tight_margin: int) -> tuple[int, int] | None:
    """LUT endpoint + configurable extension; never trust a measured tight stop."""
    envelope = tendon_motor_abs_range(result, actuator)
    if envelope is None:
        return None
    return extend_motor_abs_limits(motor, *envelope, release_margin, tight_margin)


def build_tendon_lut(io, joint: int, result: dict, primary: int,
                     secondary: int, actuator_count: int, raw: TendonRawFitBuffer) -> bool:
    if not result or not 0 <= joint < ENCODER_COUNT or not 1 <= actuator_count <= 2:
        return False
    urdf_min, urdf_max = (f32(v) for v in io.urdf_limits(joint))
    result.update(tendonLutValid=False, tendonLutCount=0, tendonLutMotor=[255, 255],
                  tendonLutMotorAbs=[[0] * TENDON_LUT_MAX_SAMPLES for _ in range(2)],
                  tendonLutAngleStartDeg=urdf_min, tendonLutAngleStepDeg=kTendonFitSampleStepDeg)
    if (not result.get("success") or not result.get("scaled")
            or result.get("encoderSpan", 0) <= 0 or result.get("totalDeg", 0) <= 0 or raw.count < 3):
        return False
    samples: list[tuple[float, int, int]] = []
    for sample in raw.samples:
        physical_raw = orient_encoder_raw(wrap_encoder_count(sample.encoder), io.direction(joint))
        mapped = map_manual_scaled_encoder_to_urdf(io, joint, result, physical_raw, io.direction(joint))
        angle = mapped["urdf_deg"] if mapped else math.nan
        if (not math.isfinite(angle) or angle < f32(urdf_min - 180.0)
                or angle > f32(urdf_max + 180.0) or len(samples) >= kTendonRawSampleCapacity):
            continue
        entry = (angle, sample.primary_abs, sample.secondary_abs)
        # 原 C++ 先移动固定数组，再检测前一点是否相近。不能改成 sorted /
        # bisect 后去重，否则插入相近中间点时后续样本与固件不同。
        index = len(samples)
        samples.append(entry)
        while index > 0 and samples[index - 1][0] > angle:
            samples[index] = samples[index - 1]
            index -= 1
        if index > 0 and abs(f32(samples[index - 1][0] - angle)) < f32(0.2):
            samples[index - 1] = entry
            samples.pop()
        else:
            samples[index] = entry
    if len(samples) < 3 or f32(samples[-1][0] - samples[0][0]) < f32(kTendonFitSampleStepDeg * 2.0):
        return False
    span = f32(urdf_max - urdf_min)
    full_steps = math.floor(f32(f32(span + f32(0.001)) / kTendonFitSampleStepDeg))
    remainder = f32(span - f32(f32(full_steps) * kTendonFitSampleStepDeg))
    count = full_steps + 1 + (1 if remainder > f32(0.001) else 0)
    if not 3 <= count <= TENDON_LUT_MAX_SAMPLES:
        return False
    result["tendonLutMotor"] = [primary, secondary if actuator_count > 1 else 255]
    for index in range(count):
        target = urdf_max if index == count - 1 else f32(urdf_min + f32(kTendonFitSampleStepDeg * f32(index)))
        for actuator in range(actuator_count):
            value = _interpolate_abs(target, samples, actuator + 1)
            if value is None:
                return False
            result["tendonLutMotorAbs"][actuator][index] = value
    result["tendonLutCount"] = count
    result["tendonLutValid"] = True
    return True


def tendon_motor_abs_range(result: dict, actuator: int) -> tuple[int, int] | None:
    count = int(result.get("tendonLutCount", 0))
    if (not 0 <= actuator < 2 or not result.get("success") or not result.get("scaled")
            or not result.get("tendonLutValid") or not 2 <= count <= TENDON_LUT_MAX_SAMPLES
            or not math.isfinite(result.get("tendonLutAngleStartDeg", math.nan))
            or not math.isfinite(result.get("tendonLutAngleStepDeg", math.nan))
            or result["tendonLutAngleStepDeg"] <= 0):
        return None
    values = result["tendonLutMotorAbs"][actuator][:count]
    return min(values), max(values)
