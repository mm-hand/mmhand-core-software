"""Fit raw=f(t) and ABS=g(t), then sample both on the same P4 time axis.

The fit is local piecewise linear interpolation. It neither predicts outside
measured intervals nor bridges invalid feedback or excessive source gaps.
Physical 14-bit raw is unwrapped before interpolation and oriented afterward.
Dual tendon ABS channels always share the same original servo position frame.

职责：保持本模块既有数据与算法契约。
线程：调用方线程，遵守现有锁边界。
输入：类型化状态和数据。
输出：诊断或计算结果。
状态所有权：既有后端拥有设备真值。
禁止依赖：禁止依赖 UI 或直接操作串口。
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Iterable

from ..communication.feedback_history import FeedbackFrame


class TimeAlignmentError(RuntimeError):
    """No trustworthy common raw/ABS time domain can be reconstructed."""


@dataclass(frozen=True, slots=True)
class AlignedFeedbackSample:
    encoder: int
    primary_abs: int
    secondary_abs: int
    timestamp_ms: int
    raw_bracket_ms: tuple[int, int]
    abs_bracket_ms: tuple[int, int]

    @property
    def raw_gap_ms(self) -> int:
        return (self.raw_bracket_ms[1] - self.raw_bracket_ms[0]) & 0xFFFFFFFF

    @property
    def abs_gap_ms(self) -> int:
        return (self.abs_bracket_ms[1] - self.abs_bracket_ms[0]) & 0xFFFFFFFF


@dataclass(frozen=True, slots=True)
class AlignedFeedbackResult:
    samples: tuple[AlignedFeedbackSample, ...]
    raw_point_count: int
    abs_point_count: int
    overlap_start_ms: int
    overlap_end_ms: int
    max_raw_gap_ms: int
    max_abs_gap_ms: int


@dataclass(frozen=True, slots=True)
class _Segment:
    times: tuple[int, ...]
    values: tuple[tuple[int, ...], ...]


def _relative_time(timestamp: int, origin: int) -> int:
    # A calibration phase is much shorter than the uint32 half-period. Signed
    # modular subtraction also handles a late packet around clock rollover.
    return ((timestamp - origin + 0x80000000) & 0xFFFFFFFF) - 0x80000000


def _round_away(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def _segments(frames: tuple[FeedbackFrame, ...], kind: str, indices: tuple[int, ...],
              origin: int, max_gap: int) -> tuple[tuple[_Segment, ...], int]:
    required_mask = sum(1 << index for index in indices)
    # At one timestamp, the latest valid value wins, but any invalid event
    # remains a barrier. A later corrected packet cannot erase lost trust.
    by_time: dict[int, tuple[FeedbackFrame, bool]] = {}
    for frame in frames:
        if frame.kind != kind:
            continue
        valid = (len(frame.values) > max(indices)
                 and frame.valid_mask & required_mask == required_mask)
        if valid and kind == "encoder_raw":
            valid = 0 <= frame.values[indices[0]] < 16384
        time_ms = _relative_time(frame.p4_time_ms, origin)
        previous = by_time.get(time_ms)
        if previous is None:
            by_time[time_ms] = frame, valid
        else:
            latest = frame if frame.sequence >= previous[0].sequence else previous[0]
            by_time[time_ms] = latest, previous[1] and valid

    segments: list[_Segment] = []
    times: list[int] = []
    values: list[tuple[int, ...]] = []
    point_count = 0

    def close() -> None:
        if times:
            segments.append(_Segment(tuple(times), tuple(values)))
            times.clear()
            values.clear()

    for time_ms, (frame, valid) in sorted(by_time.items()):
        if not valid:
            close()
            continue
        if times and time_ms - times[-1] > max_gap:
            close()
        point = tuple(frame.values[index] for index in indices)
        if kind == "encoder_raw" and values:
            previous_raw = values[-1][0]
            delta = ((point[0] - previous_raw + 8192) % 16384) - 8192
            point = (previous_raw + delta,)
        times.append(time_ms)
        values.append(point)
        point_count += 1
    close()
    return tuple(segments), point_count


def _evaluate(segment: _Segment, time_ms: int) -> tuple[tuple[float, ...], tuple[int, int]]:
    upper = bisect_left(segment.times, time_ms)
    if upper < len(segment.times) and segment.times[upper] == time_ms:
        return tuple(float(value) for value in segment.values[upper]), (time_ms, time_ms)
    if upper == 0 or upper == len(segment.times):
        raise TimeAlignmentError("time alignment cannot extrapolate outside measured feedback")
    lower = upper - 1
    t0, t1 = segment.times[lower], segment.times[upper]
    fraction = (time_ms - t0) / (t1 - t0)
    values = tuple(a + (b - a) * fraction
                   for a, b in zip(segment.values[lower], segment.values[upper]))
    return values, (t0, t1)


def align_feedback(frames: Iterable[FeedbackFrame], joint: int,
                   motors: Iterable[int], direction: int, *,
                   raw_max_gap_ms: int = 300,
                   abs_max_gap_ms: int = 500) -> AlignedFeedbackResult:
    """Return common-time pairs from measured encoder and servo position tables.

    Source counts refer to valid distinct P4 timestamps for the requested joint
    and complete motor pair. Maximum gaps refer to interpolation brackets that
    were actually used. An absent common interval raises TimeAlignmentError;
    partially invalid phases retain only their trustworthy shared intervals.
    """
    motor_indices = tuple(int(motor) for motor in motors)
    if (joint < 0 or len(motor_indices) not in (1, 2)
            or any(motor < 0 for motor in motor_indices)
            or len(set(motor_indices)) != len(motor_indices)):
        raise ValueError("time alignment requires one joint and one or two distinct motors")
    if direction not in (-1, 1):
        raise ValueError("encoder direction must be -1 or 1")
    if raw_max_gap_ms <= 0 or abs_max_gap_ms <= 0:
        raise ValueError("maximum feedback gaps must be positive")
    packets = tuple(frames)
    if not packets:
        raise TimeAlignmentError("time alignment has no feedback history")
    origin = packets[0].p4_time_ms
    raw_segments, raw_count = _segments(
        packets, "encoder_raw", (joint,), origin, raw_max_gap_ms)
    abs_segments, abs_count = _segments(
        packets, "servo_abs", motor_indices, origin, abs_max_gap_ms)
    if raw_count < 2 or abs_count < 2:
        raise TimeAlignmentError(
            f"time alignment requires two valid timestamps per source: raw={raw_count} ABS={abs_count}")

    samples: list[AlignedFeedbackSample] = []
    raw_index = abs_index = 0
    while raw_index < len(raw_segments) and abs_index < len(abs_segments):
        raw_segment, abs_segment = raw_segments[raw_index], abs_segments[abs_index]
        start = max(raw_segment.times[0], abs_segment.times[0])
        end = min(raw_segment.times[-1], abs_segment.times[-1])
        if start <= end:
            sample_times = sorted({time_ms for time_ms in raw_segment.times + abs_segment.times
                                   if start <= time_ms <= end})
            for time_ms in sample_times:
                raw, raw_bracket = _evaluate(raw_segment, time_ms)
                absolute, abs_bracket = _evaluate(abs_segment, time_ms)
                physical_raw = _round_away(raw[0]) % 16384
                samples.append(AlignedFeedbackSample(
                    encoder=(physical_raw * direction) % 16384,
                    primary_abs=_round_away(absolute[0]),
                    secondary_abs=_round_away(absolute[1]) if len(absolute) == 2 else 0,
                    timestamp_ms=(origin + time_ms) & 0xFFFFFFFF,
                    raw_bracket_ms=tuple((origin + value) & 0xFFFFFFFF for value in raw_bracket),
                    abs_bracket_ms=tuple((origin + value) & 0xFFFFFFFF for value in abs_bracket),
                ))
        if raw_segment.times[-1] <= abs_segment.times[-1]:
            raw_index += 1
        else:
            abs_index += 1

    if not samples:
        raise TimeAlignmentError("time alignment has no common valid raw/ABS time interval")
    return AlignedFeedbackResult(
        samples=tuple(samples), raw_point_count=raw_count, abs_point_count=abs_count,
        overlap_start_ms=samples[0].timestamp_ms, overlap_end_ms=samples[-1].timestamp_ms,
        max_raw_gap_ms=max(sample.raw_gap_ms for sample in samples),
        max_abs_gap_ms=max(sample.abs_gap_ms for sample in samples),
    )
