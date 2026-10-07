"""Tendon LUT resampling migrated from CalibrationLutBuilder.cpp.

职责：将 Manual 映射后的单腱或双腱样本重采样为固定角步长 LUT。
线程：仅在标定专用工作线程调用，不执行阻塞操作。
输入：URDF 角度、单/双电机 ABS 样本和 URDF 限位。
输出：真实端点角度和对应电机 ABS 表，安装由标定事务负责。
状态所有权：纯计算，不持有设备状态或持久化缓存。
禁止依赖：禁止依赖 Qt、串口、整机状态机和硬件执行接口。
"""

from bisect import bisect_right
from dataclasses import dataclass
import math

import numpy as np

from .protocol import TENDON_LUT_MAX_SAMPLES


class CalibrationLutError(ValueError):
    pass


@dataclass(frozen=True)
class TendonLut:
    angles_degrees: tuple[float, ...]
    motor_abs_rows: tuple[tuple[int, ...], ...]
    nominal_step_degrees: float = 5.0


def _rounded_abs(value):
    """Match production float32 roundToInt32, including negative half counts."""
    value = np.float32(value)
    if not np.isfinite(value):
        raise CalibrationLutError("non-finite interpolated ABS")
    rounded = math.trunc(float(np.float32(value + np.float32(0.5 if value >= 0 else -0.5))))
    if not -(1 << 31) <= rounded < 1 << 31:
        raise CalibrationLutError("interpolated ABS outside int32")
    return rounded


def _interpolate(target, angles, values):
    if target <= angles[0] or target > angles[-1]:
        lower = 0 if target <= angles[0] else len(angles) - 2
        upper = lower + 1
        span = np.float32(angles[upper] - angles[lower])
        if span <= 0:
            raise CalibrationLutError("non-increasing sample angle")
        slope = np.float32(np.float32(np.float32(values[upper]) - np.float32(values[lower])) / span)
        base = lower if target <= angles[0] else upper
        return _rounded_abs(np.float32(np.float32(values[base]) +
            np.float32(np.float32(target - angles[base]) * slope)))
    position = bisect_right(angles, target)
    upper = max(1, min(position, len(angles) - 1))
    lower = upper - 1
    span = np.float32(angles[upper] - angles[lower])
    if span <= 0:
        raise CalibrationLutError("non-increasing sample angle")
    fraction = np.float32(np.float32(target - angles[lower]) / span)
    delta = np.float32(np.float32(values[upper]) - np.float32(values[lower]))
    return _rounded_abs(np.float32(np.float32(values[lower]) + np.float32(delta * fraction)))


def manual_encoder_to_urdf(raw, *, manual_min_raw, manual_max_raw,
                           direction, urdf_min_degrees, urdf_max_degrees):
    """Map physical encoder raw using Manual; extrapolate outside at 360/16384."""
    if direction not in (-1, 1) or any(
            not isinstance(value, (int, np.integer)) or isinstance(value, bool)
            or not 0 <= value < 16384 for value in (raw, manual_min_raw, manual_max_raw)):
        raise CalibrationLutError("invalid Manual encoder coordinate")
    minimum, maximum = np.float32(urdf_min_degrees), np.float32(urdf_max_degrees)
    span = ((manual_max_raw - manual_min_raw) * direction) % 16384
    if (not 0 < span < 8192 or not np.isfinite(minimum)
            or not np.isfinite(maximum) or minimum >= maximum):
        raise CalibrationLutError("invalid Manual span or URDF range")
    coordinate = ((raw - manual_min_raw) * direction) % 16384
    if coordinate > span and 16384 - coordinate < coordinate - span:
        coordinate -= 16384
    if coordinate < 0:
        delta = np.float32(np.float32(coordinate) * np.float32(360.0) / np.float32(16384))
        return float(np.float32(minimum + delta))
    if coordinate <= span:
        scale = np.float32(np.float32(maximum - minimum) / np.float32(span))
        return float(np.float32(minimum + np.float32(np.float32(coordinate) * scale)))
    delta = np.float32(np.float32(coordinate - span) * np.float32(360.0) / np.float32(16384))
    return float(np.float32(maximum + delta))


def resample_tendon_lut(samples, *, urdf_min_degrees, urdf_max_degrees,
                       actuator_count):
    """Preserve original 5-degree interpolation and end-slope extrapolation.

    Each sample is ``(URDF degrees, (primary ABS, [secondary ABS]))``.
    The caller must first validate Manual and convert physical encoder raw into
    URDF degrees. This function never infers that mapping from Auto span.
    """
    if actuator_count not in (1, 2):
        raise CalibrationLutError("one or two actuators required")
    minimum, maximum = np.float32(urdf_min_degrees), np.float32(urdf_max_degrees)
    if not np.isfinite(minimum) or not np.isfinite(maximum) or minimum >= maximum:
        raise CalibrationLutError("invalid URDF range")
    angles, rows = [], [[] for _ in range(actuator_count)]
    for angle, abs_values in list(samples)[:TENDON_LUT_MAX_SAMPLES * 3]:
        angle = np.float32(angle)
        if (not np.isfinite(angle) or angle < minimum - 180.0
                or angle > maximum + 180.0):
            continue
        if len(abs_values) != actuator_count or any(
                not isinstance(value, (int, np.integer)) or isinstance(value, bool)
                or not -(1 << 31) <= int(value) < 1 << 31 for value in abs_values):
            raise CalibrationLutError("invalid motor ABS sample")
        insert_at = bisect_right(angles, angle)
        if insert_at > 0 and abs(float(angles[insert_at - 1] - angle)) < 0.2:
            angles[insert_at - 1] = angle
            for motor in range(actuator_count):
                rows[motor][insert_at - 1] = int(abs_values[motor])
        else:
            angles.insert(insert_at, angle)
            for motor in range(actuator_count):
                rows[motor].insert(insert_at, int(abs_values[motor]))
    if len(angles) < 3 or angles[-1] - angles[0] < 10.0:
        raise CalibrationLutError("insufficient samples or angular travel")
    span = np.float32(maximum - minimum)
    full_steps = math.floor(float(np.float32(np.float32(span + np.float32(0.001)) / np.float32(5.0))))
    remainder = np.float32(span - np.float32(full_steps * 5.0))
    count = full_steps + 1 + int(remainder > 0.001)
    if not 3 <= count <= TENDON_LUT_MAX_SAMPLES:
        raise CalibrationLutError("LUT sample count outside protocol capacity")
    targets = [np.float32(minimum + np.float32(index * 5.0)) for index in range(count)]
    targets[-1] = maximum
    fitted = tuple(tuple(_interpolate(target, angles, row) for target in targets) for row in rows)
    return TendonLut(tuple(float(angle) for angle in targets), fitted)
