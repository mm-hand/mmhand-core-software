"""Integral and fast-reversal autotuning core (pure, no hardware access).

Responsibilities:
- Build safe hold and back-and-forth target plans inside an audited workspace.
- Replay the P4 per-joint controller state machine from same-cycle trace rows.
- Score integral holds and reversal events; choose candidates by fixed rules.
- Evaluate the target-reversal detector offline for confirm_deg/confirm_ms.

State ownership: only in-memory data. No serial, UI, parameter files or devices.
Forbidden dependencies: serial/UI/ROS2 imports; callers own hardware access.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
import math
from typing import Iterable, Optional, Sequence

import numpy as np

TYPES = ('aa', 'mcp', 'pip', 'dip')
# Test trajectories must stay away from the MCP upper mechanical limit.
TEST_HIGH_CAP_DEG = {'mcp': 74.0}


def test_high_cap(joint_type: str, high: float) -> float:
    return min(float(high), TEST_HIGH_CAP_DEG.get(joint_type, float(high)))


REPRESENTATIVE_JOINTS = {'aa': 8, 'mcp': 9, 'pip': 10, 'dip': 11}
JOINT_TYPES = {joint: name for name, joint in REPRESENTATIVE_JOINTS.items()}
CANDIDATE_JOINTS = tuple(REPRESENTATIVE_JOINTS[name] for name in TYPES)

KI_CANDIDATES = (0.0, 0.1, 0.2)
CONFIRM_DEG_CANDIDATES = (0.3, 0.6, 1.0, 1.5)
CONFIRM_MS_CANDIDATES = (20, 60, 100)
TAKEUP_CANDIDATES = (0, 20, 40, 80)
EXIT_TRAVEL_CANDIDATES = (0.15, 0.25, 0.50)

# Fixed first-round integral gates from the supplied plan.
INTEGRAL_NOISE_DEG = 0.1
INTEGRAL_CONFIRM_MS = 200
INTEGRAL_LIMIT_COUNTS = 100.0
FAST_WINDOW_MS = 20.0
SLOW_WINDOW_MS = 500.0

# Normalisation tolerances (engineering limits, not safety limits).
TOL_SS_DEG = 0.5
TOL_JITTER_DEG = 0.2
TOL_OVERSHOOT_DEG = 1.5
TOL_CHATTER_COUNTS = 50.0
TOL_DEAD_S = 0.5
TOL_IAE_DEG_S = 1.0
TOL_REV_OVERSHOOT_DEG = 1.5
TAKEUP_TIMEOUT_S = 0.6
REVERSAL_MOVE_DEG = 0.2
REVERSAL_SUSTAIN_S = 0.05
REVERSAL_WINDOW_S = 1.5
DEFAULT_DETECTOR_MATCH_S = 0.5


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _sign(value: float) -> int:
    return 1 if value > 0.0001 else -1 if value < -0.0001 else 0


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _mean(values: Sequence[float]) -> float:
    return float(sum(values) / len(values)) if values else float('nan')


def _p90(values: Sequence[float]) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), 90)) if values else float('nan')


def _std(values: Sequence[float]) -> float:
    return float(np.std(np.asarray(values, dtype=float))) if values else float('nan')


@dataclass(frozen=True)
class PlanPoint:
    time_s: float
    target_deg: float
    segment: str
    kind: str                 # ramp | hold | settle
    position: Optional[float] = None
    approach: Optional[str] = None
    event: Optional[str] = None


@dataclass(frozen=True)
class TruthEvent:
    segment: str
    direction: int
    center_deg: float
    amplitude_deg: float
    speed: str
    needs_takeup: bool


@dataclass(frozen=True)
class DetectorEvent:
    t_s: float
    direction: int


@dataclass(frozen=True)
class DetectorStream:
    name: str
    t: np.ndarray
    target: np.ndarray
    truth: tuple
    false_positive_zone: bool = False
    takeup_relevant: bool = True


@dataclass(frozen=True)
class DetectorScore:
    confirm_deg: float
    confirm_ms: float
    tp: int
    fp: int
    fn: int
    fn_needs_takeup: int
    median_latency_s: Optional[float]
    p90_latency_s: Optional[float]
    fp_per_s: float
    valid: bool
    reason: str = ''


@dataclass
class IntegralTrial:
    joint_type: str
    joint: int
    ki: float
    valid: bool
    score: Optional[float]
    reason: str
    holds: list = field(default_factory=list)
    gate_fraction: float = 0.0
    clamp_fraction: float = 0.0
    saturation_fraction: float = 0.0
    i_change_counts: float = 0.0
    replay_mismatch: float = 0.0
    noise: float = 0.0


@dataclass
class ReversalTrial:
    joint_type: str
    joint: int
    takeup_counts: float
    exit_travel_deg: float
    valid: bool
    score: Optional[float]
    reason: str
    events: list = field(default_factory=list)
    worst_event_s: Optional[float] = None
    extra_travel_counts: float = 0.0
    saturation_fraction: float = 0.0
    takeup_timeout: bool = False
    aa_dual_issue: str = ''


# ---------------------------------------------------------------------------
# Plans

def build_validation_plan(low: float, high: float, *, speed_deg_s: float = 5.0,
                          hold_s: float = 2.0, sine_s: float = 8.0,
                          delta_deg: Optional[float] = None,
                          high_cap: Optional[float] = None) -> tuple:
    """Combined check: 50% hold, 0.2 Hz/5 deg sine, then a short back-and-forth."""
    if not (_finite(low) and _finite(high) and high > low):
        raise ValueError('invalid workspace')
    if high_cap is not None:
        high = min(high, float(high_cap))
        if high <= low:
            raise ValueError('high_cap leaves no test workspace')
    span = high - low
    center = 0.5 * (low + high)
    amplitude = min(5.0, 0.15 * span)
    delta = float(delta_deg) if delta_deg is not None else min(4.0, 0.07 * span)
    points: list = [PlanPoint(0.0, center, 'start', 'settle')]
    now = _hold(points, center, hold_s, 0.0, 'validate_hold', 0.5, 'center')
    count = max(2, int(sine_s / 0.01))
    for index in range(count + 1):
        elapsed = sine_s * index / count
        points.append(PlanPoint(now + elapsed,
                                center + amplitude * math.sin(2 * math.pi * 0.2 * elapsed),
                                'dyn_sine', 'ramp'))
    now += sine_s
    current = points[-1].target_deg
    for index, sign in enumerate((1, -1, 1)):
        target = center + sign * delta
        segment = f'val_rev_{index}'
        now = _ramp(points, current, target, speed_deg_s, now, segment)
        now = _hold(points, target, 0.35, now, segment + '_hold', 0.0, 'turn')
        current = target
    for point in points:
        if not (low - 1e-9 <= point.target_deg <= high + 1e-9):
            raise ValueError(f'validation target {point.target_deg} outside [{low},{high}]')
    truth: list = []
    previous_direction = 0
    previous_target = points[0].target_deg
    for point in points[1:]:
        direction = _sign(point.target_deg - previous_target)
        if direction and previous_direction and direction != previous_direction \
                and point.segment.startswith('val_rev'):
            truth.append(TruthEvent(segment=point.segment, direction=direction,
                                    center_deg=center, amplitude_deg=delta,
                                    speed='', needs_takeup=delta >= 1.0))
        if direction:
            previous_direction = direction
        previous_target = point.target_deg
    return points, truth


def _ramp(points: list, start_deg: float, end_deg: float, speed_deg_s: float,
          t0: float, segment: str, kind: str = 'ramp', **kwargs) -> float:
    speed = max(abs(float(speed_deg_s)), 0.5)
    distance = abs(end_deg - start_deg)
    duration = distance / speed
    if duration <= 1e-9:
        points.append(PlanPoint(t0, end_deg, segment, kind, **kwargs))
        return t0
    count = max(2, int(math.ceil(duration / 0.01)))
    for index in range(1, count + 1):
        fraction = index / count
        points.append(PlanPoint(t0 + duration * fraction,
                                start_deg + (end_deg - start_deg) * fraction,
                                segment, kind, **kwargs))
    return t0 + duration


def _hold(points: list, target_deg: float, hold_s: float, t0: float, segment: str,
          position: float, approach: str) -> float:
    count = max(2, int(math.ceil(hold_s / 0.01)))
    for index in range(count + 1):
        points.append(PlanPoint(t0 + hold_s * index / count, target_deg, segment,
                                'hold', position=position, approach=approach))
    return t0 + hold_s


def build_hold_plan(low: float, high: float, *, speed_deg_s: float = 5.0,
                    hold_s: float = 3.2, dt: float = 0.01,
                    high_cap: Optional[float] = None) -> list:
    """Six holds: 20/50/80 approached upward, then 80/50/20 approached downward."""
    if not (_finite(low) and _finite(high) and high > low):
        raise ValueError('invalid workspace')
    if high_cap is not None:
        high = min(high, float(high_cap))
        if high <= low:
            raise ValueError('high_cap leaves no test workspace')
    if hold_s <= 0 or speed_deg_s <= 0:
        raise ValueError('hold_s and speed must be positive')
    span = high - low
    if span < 8.0:
        raise ValueError('workspace too small for hold plan')
    p = lambda fraction: low + fraction * span
    points: list = []
    now = 0.0
    start = p(0.10)
    points.append(PlanPoint(0.0, start, 'start', 'settle'))
    now = _ramp(points, start, p(0.20), speed_deg_s, now, 'up-to-20')
    now = _hold(points, p(0.20), hold_s, now, 'hold_20_up', 0.20, 'up')
    now = _ramp(points, p(0.20), p(0.50), speed_deg_s, now, 'up-to-50')
    now = _hold(points, p(0.50), hold_s, now, 'hold_50_up', 0.50, 'up')
    now = _ramp(points, p(0.50), p(0.80), speed_deg_s, now, 'up-to-80')
    now = _hold(points, p(0.80), hold_s, now, 'hold_80_up', 0.80, 'up')
    # Approaching 80% from above gives a true extension-side approach.
    upper = min(high, p(0.80) + min(2.0, 0.20 * span))
    if upper > p(0.80) + 0.05:
        now = _ramp(points, p(0.80), upper, speed_deg_s, now, 'over-80')
        now = _ramp(points, upper, p(0.80), speed_deg_s, now, 'down-to-80')
    now = _hold(points, p(0.80), hold_s, now, 'hold_80_down', 0.80, 'down')
    now = _ramp(points, p(0.80), p(0.50), speed_deg_s, now, 'down-to-50')
    now = _hold(points, p(0.50), hold_s, now, 'hold_50_down', 0.50, 'down')
    now = _ramp(points, p(0.50), p(0.20), speed_deg_s, now, 'down-to-20')
    now = _hold(points, p(0.20), hold_s, now, 'hold_20_down', 0.20, 'down')
    for point in points:
        if not (low - 1e-9 <= point.target_deg <= high + 1e-9):
            raise ValueError(f'hold plan target {point.target_deg} outside [{low},{high}]')
        if not math.isfinite(point.time_s):
            raise ValueError('non-finite plan time')
    return points


def plan_duration(points: Sequence[PlanPoint]) -> float:
    return float(points[-1].time_s) if points else 0.0


def hold_labels(points: Sequence[PlanPoint]) -> list:
    result = []
    seen = set()
    for point in points:
        if point.kind == 'hold' and point.segment not in seen:
            seen.add(point.segment)
            result.append(dict(segment=point.segment, position=point.position,
                               approach=point.approach, target_deg=point.target_deg))
    return result


def build_reversal_plan(low: float, high: float, *, centers=(0.20, 0.80),
                        delta_deg: Optional[float] = None,
                        speed_deg_s: float = 5.0,
                        speeds: Sequence[tuple] = (('low', 5.0), ('high', 20.0)),
                        settle_s: float = 0.35,
                        high_cap: Optional[float] = None) -> tuple:
    """Back-and-forth plan around two centres at two speeds.

    The plan is continuous; truth events are derived from the designed target
    slope changes, not from the detector under test.
    """
    if not (_finite(low) and _finite(high) and high > low):
        raise ValueError('invalid workspace')
    if high_cap is not None:
        high = min(high, float(high_cap))
        if high <= low:
            raise ValueError('high_cap leaves no test workspace')
    span = high - low
    p = lambda fraction: low + fraction * span
    delta = float(delta_deg) if delta_deg is not None else min(4.0, 0.07 * span)
    if delta <= 0:
        raise ValueError('delta must be positive')
    points: list = []
    now = 0.0
    points.append(PlanPoint(0.0, p(centers[0]), 'start', 'settle'))
    for fraction in centers:
        center = p(fraction)
        now = _ramp(points, points[-1].target_deg, center, speed_deg_s, now,
                    f'center_{fraction:.2f}')
        now = _hold(points, center, settle_s, now, f'center_{fraction:.2f}_hold',
                    0.0, 'center')
        for speed_name, speed in speeds:
            if center + delta > high + 1e-9 or center - delta < low - 1e-9:
                raise ValueError(f'center {center:.2f} delta {delta:.2f} outside workspace')
            for sign in (1, -1):
                target = center + sign * delta
                opposite = center - sign * delta
                segment = f'rev_{fraction:.2f}_{speed_name}_{"pos" if sign > 0 else "neg"}'
                now = _ramp(points, points[-1].target_deg, target, speed, now, segment)
                now = _hold(points, target, settle_s, now, segment + '_hold', 0.0, 'turn')
                now = _ramp(points, target, opposite, speed, now, segment + '_return')
                now = _hold(points, opposite, settle_s, now, segment + '_return_hold', 0.0, 'turn')
                now = _ramp(points, opposite, center, speed, now, segment + '_center')
                now = _hold(points, center, settle_s, now, segment + '_center_hold', 0.0, 'center')
    for point in points:
        if not (low - 1e-9 <= point.target_deg <= high + 1e-9):
            raise ValueError(f'reversal plan target {point.target_deg} outside [{low},{high}]')
    # Ground truth: every target-slope change after an established direction.
    truth: list = []
    previous_direction = 0
    previous_target = points[0].target_deg
    for point in points[1:]:
        direction = _sign(point.target_deg - previous_target)
        if direction and previous_direction and direction != previous_direction:
            truth.append(TruthEvent(segment=point.segment, direction=direction,
                                    center_deg=points[0].target_deg,
                                    amplitude_deg=delta, speed='',
                                    needs_takeup=delta >= 1.0))
        if direction:
            previous_direction = direction
        previous_target = point.target_deg
    return points, truth


# ---------------------------------------------------------------------------
# Detector simulation

def simulate_detector(t: Sequence[float], target: Sequence[float], confirm_deg: float,
                      confirm_ms: float, start_ms: float = 0.0) -> list:
    """Exact target-direction state machine used by JointFeedbackController."""
    if len(t) != len(target) or len(t) < 2:
        return []
    if confirm_deg <= 0 or confirm_ms < 0:
        raise ValueError('invalid detector thresholds')
    t = np.asarray(t, dtype=float)
    target = np.asarray(target, dtype=float)
    events = []
    direction = 0
    candidate = 0
    anchor = float(target[0])
    since_ms = float(t[0] * 1000.0 + start_ms)
    last_target = float(target[0])
    for index in range(1, len(t)):
        delta = float(target[index]) - last_target
        step = _sign(delta)
        if step:
            if not direction:
                direction = step
            if step == direction:
                candidate = 0
            elif candidate != step:
                candidate = step
                anchor = last_target
                since_ms = float(t[index] * 1000.0 + start_ms)
        last_target = float(target[index])
        if candidate and (float(target[index]) - anchor) * candidate >= confirm_deg:
            if float(t[index] * 1000.0 + start_ms) - since_ms >= confirm_ms:
                direction = candidate
                candidate = 0
                events.append(DetectorEvent(float(t[index]), direction))
    return events


def _triangle_stream(name: str, center: float, amplitude: float, period_s: float,
                     duration_s: float, dt: float = 0.01, fp_zone: bool = False,
                     takeup_relevant: bool = True):
    t = np.arange(0.0, duration_s + dt * 0.5, dt)
    phase = (t % period_s) / period_s
    target = center + amplitude * np.where(phase < 0.5, -1.0 + 4.0 * phase,
                                           3.0 - 4.0 * phase)
    truth = []
    # Direction turns at the triangle peaks: phase 0.5 (falling) and 1.0 (rising).
    for k in range(2, int(duration_s / period_s * 4) + 1, 2):
        turn = k * period_s / 4.0
        if turn >= duration_s:
            continue
        direction = -1 if k % 4 == 2 else 1
        truth.append(DetectorEvent(turn, direction))
    return DetectorStream(name, t, target, tuple(truth), fp_zone, takeup_relevant)


def build_detector_suite() -> list:
    streams = [
        _triangle_stream('large-10deg-0.2hz', 0.0, 10.0, 5.0, 20.0),
        _triangle_stream('small-1.0deg', 0.0, 1.0, 2.0, 12.0),
        _triangle_stream('small-0.6deg', 0.0, 0.6, 2.0, 12.0,
                         takeup_relevant=False),
        _triangle_stream('slow-2deg-s', 0.0, 5.0, 10.0, 20.0),
        _triangle_stream('fast-20deg-s', 0.0, 5.0, 1.0, 10.0),
    ]
    # Static jitter: real teleoperation noise level around a stationary target.
    t = np.arange(0.0, 12.0, 0.01)
    rng = np.random.default_rng(20261021)
    jitter = 0.05 * np.sin(2 * math.pi * 7.0 * t) + rng.normal(0.0, 0.01, t.size)
    streams.append(DetectorStream('static-jitter-7hz', t, 10.0 + jitter, tuple(), True,
                                  takeup_relevant=False))
    # Short pseudo reversal: 0.2 deg excursion then resume; must not trigger takeup.
    t = np.arange(0.0, 8.0, 0.01)
    target = np.full_like(t, 20.0)
    pulse = (t >= 2.0) & (t < 2.05)
    target[pulse] -= 0.2
    pulse2 = (t >= 5.0) & (t < 5.05)
    target[pulse2] += 0.2
    streams.append(DetectorStream('pseudo-reversal-0.2deg', t, target, tuple(), True,
                                  takeup_relevant=False))
    return streams


def evaluate_detector(streams: Sequence[DetectorStream], confirm_deg: float,
                      confirm_ms: float, match_s: float = DEFAULT_DETECTOR_MATCH_S) -> dict:
    tp = fp = fn = fn_needs = 0
    latencies = []
    total_duration = sum(float(stream.t[-1] - stream.t[0]) for stream in streams
                         if len(stream.t)) or 1.0
    for stream in streams:
        detections = simulate_detector(stream.t, stream.target, confirm_deg, confirm_ms)
        used = [False] * len(detections)
        for truth in stream.truth:
            matched = None
            for index, detection in enumerate(detections):
                if used[index]:
                    continue
                if detection.direction != truth.direction:
                    continue
                if truth.t_s - 1e-9 <= detection.t_s <= truth.t_s + match_s:
                    matched = index
                    break
            if matched is None:
                fn += 1
                if stream.takeup_relevant:
                    fn_needs += 1
            else:
                used[matched] = True
                tp += 1
                latencies.append(detections[matched].t_s - truth.t_s)
        fp += sum(1 for used_flag in used if not used_flag)
    return dict(tp=tp, fp=fp, fn=fn, fn_needs=fn_needs,
                latencies=sorted(latencies), fp_per_s=float(fp / total_duration))


def evaluate_detector_grid(streams: Sequence[DetectorStream],
                           match_s: float = DEFAULT_DETECTOR_MATCH_S) -> list:
    results = []
    for confirm_deg in CONFIRM_DEG_CANDIDATES:
        for confirm_ms in CONFIRM_MS_CANDIDATES:
            summary = evaluate_detector(streams, confirm_deg, confirm_ms, match_s)
            latencies = summary['latencies']
            results.append(DetectorScore(
                confirm_deg=confirm_deg,
                confirm_ms=confirm_ms,
                tp=summary['tp'],
                fp=summary['fp'],
                fn=summary['fn'],
                fn_needs_takeup=summary['fn_needs'],
                median_latency_s=float(np.median(latencies)) if latencies else None,
                p90_latency_s=float(np.percentile(latencies, 90)) if latencies else None,
                fp_per_s=summary['fp_per_s'],
                valid=True,
                reason='',
            ))
    return results


def choose_detector(scores: Sequence[DetectorScore]) -> DetectorScore:
    if not scores:
        raise ValueError('no detector scores')
    def key(score: DetectorScore):
        fp = score.fp
        fn_needs = score.fn_needs_takeup
        fn_all = score.fn
        latency = score.p90_latency_s if score.p90_latency_s is not None else float('inf')
        return (fp, fn_needs, fn_all, latency, score.confirm_ms, -score.confirm_deg)
    return min(scores, key=key)


# ---------------------------------------------------------------------------
# Same-cycle replay of the P4 per-joint controller state

class _ReplayFilter:
    def __init__(self, fast_ms: float, slow_ms: float):
        self.fast_ms = fast_ms
        self.slow_ms = slow_ms
        self.samples = []
        self.first = 0
        self.last = 0
        self.count = 0

    def reset(self):
        self.samples = []
        self.first = self.last = self.count = 0

    def update(self, timestamp, angle, valid):
        if not valid or not timestamp or not _finite(angle):
            self.reset()
            return None
        if self.count and timestamp == self.last:
            return dict(fresh=False)
        if self.count and (int(timestamp) - int(self.last) <= 0 or timestamp - self.last > 600):
            self.reset()
        if not self.count:
            self.first = timestamp
        sample_dt = ((timestamp - self.last) * 0.001
                     if self.last and timestamp - self.last <= 100 else 0.0)
        self.last = timestamp
        self.samples.append((timestamp, float(angle)))
        if len(self.samples) > 256:
            self.samples.pop(0)
        self.count = len(self.samples)
        fast = [value for stamp, value in self.samples if timestamp - stamp < self.fast_ms]
        slow = [value for stamp, value in self.samples if timestamp - stamp < self.slow_ms]
        return dict(fresh=True,
                    fast=float(np.mean(fast)) if fast else float(angle),
                    slow=float(np.mean(slow)) if slow else float(angle),
                    ready=bool(fast and slow and timestamp - self.first >= self.slow_ms),
                    dt=float(sample_dt))


def replay_control_state(samples: Sequence[dict], params: dict) -> list:
    """Reconstruct phase/gate/detection from same-cycle trace rows.

    Required sample keys: t (P4 ms), target, actual, encoder_ms, slow,
    i_counts, reversal_counts, saturated.  Optional: motor_command_abs,
    motor_actual_abs, pid_counts.
    """
    fast_ms = float(params.get('fast_window_ms', FAST_WINDOW_MS))
    slow_ms = float(params.get('slow_window_ms', SLOW_WINDOW_MS))
    confirm_deg = float(params.get('confirm_deg', 1.0))
    confirm_ms = float(params.get('confirm_ms', 20))
    takeup = float(params.get('takeup_counts', 0.0))
    exit_travel = float(params.get('exit_travel_deg', 0.25))
    integral_limit = float(params.get('integral_limit_counts', INTEGRAL_LIMIT_COUNTS))
    noise = float(params.get('integral_noise_deg', INTEGRAL_NOISE_DEG))
    confirm_integral_ms = float(params.get('integral_confirm_ms', INTEGRAL_CONFIRM_MS))

    filt = _ReplayFilter(fast_ms, slow_ms)
    direction = 0
    candidate = 0
    anchor = 0.0
    candidate_since = 0.0
    phase = 'Tracking'
    last_target = None
    error_sign = 0
    error_since = 0.0
    response_anchor = 0.0
    saturated_prev = False
    output = []
    for sample in samples:
        t = float(sample['t'])
        target = float(sample['target'])
        actual = float(sample['actual'])
        estimate = filt.update(sample.get('encoder_ms', t), actual,
                               bool(sample.get('valid', True)) and _finite(actual))
        row = dict(sample)
        if estimate is None or not estimate.get('fresh'):
            row.update(control_phase=phase, integral_enabled=False,
                       integral_gate_reason='feedback_stale',
                       reversal_detected=False,
                       takeup_active=phase == 'Takeup',
                       integral_clamped=abs(float(sample.get('i_counts') or 0)) >= integral_limit - 1,
                       fast=float(actual), slow=float(sample.get('slow') or actual),
                       takeup_lifting=bool(sample.get('reversal_counts')))
            output.append(row)
            continue
        fast, slow_est, dt = estimate['fast'], estimate['slow'], estimate['dt']
        slow_value = sample.get('slow')
        if slow_value is not None and _finite(slow_value):
            slow_est = float(slow_value)
        if last_target is None:
            last_target = target
        delta = target - last_target
        step = _sign(delta)
        if step:
            if not direction:
                direction = step
            if step == direction:
                if phase == 'Pending':
                    phase = 'Tracking'
                candidate = 0
            elif candidate != step:
                candidate = step
                anchor = last_target
                candidate_since = t
                phase = 'Pending'
        last_target = target
        detected = False
        if candidate and (target - anchor) * candidate >= confirm_deg and \
                t - candidate_since >= confirm_ms:
            direction = candidate
            candidate = 0
            error_sign = 0
            error_since = t
            phase = 'Takeup' if takeup > 0 else 'Tracking'
            response_anchor = fast
            detected = True
        if phase == 'Takeup' and direction * (fast - response_anchor) >= exit_travel:
            phase = 'Tracking'
        error = target - fast
        slow_error = target - slow_est
        if _sign(error) == _sign(slow_error):
            integral_error = _sign(error) * max(0.0, abs(slow_error) - noise)
        else:
            integral_error = 0.0
        if error_sign != _sign(integral_error):
            error_sign = _sign(integral_error)
            error_since = t
        if not estimate['ready']:
            gate_reason = 'feedback_stale'
        elif saturated_prev:
            gate_reason = 'saturated'
        elif phase != 'Tracking':
            gate_reason = 'not_tracking'
        elif _sign(error) != _sign(slow_error):
            gate_reason = 'sign_mismatch'
        elif error_sign == 0:
            gate_reason = 'below_noise'
        elif t - error_since < confirm_integral_ms:
            gate_reason = 'confirm_pending'
        else:
            gate_reason = 'ok'
        integrate = gate_reason == 'ok' and dt > 0
        i_counts = sample.get('i_counts')
        clamped = (abs(float(i_counts)) >= integral_limit - 1) if i_counts is not None else False
        row.update(control_phase=phase,
                   integral_enabled=bool(integrate),
                   integral_gate_reason=gate_reason,
                   reversal_detected=detected,
                   takeup_active=phase == 'Takeup',
                   integral_clamped=clamped,
                   fast=fast,
                   slow=slow_est,
                   takeup_lifting=bool(sample.get('reversal_counts')))
        output.append(row)
        saturated_prev = bool(sample.get('saturated', False))
    return output


def replay_mismatch(replayed: Sequence[dict]) -> float:
    """Fraction of takeup-lifting rows the replay did not explain, plus detector misses.

    This is a diagnostic only; observed counts remain the selection evidence.
    """
    if not replayed:
        return 1.0
    checks = 0
    mismatches = 0
    for row in replayed:
        if row.get('takeup_lifting'):
            checks += 1
            if not row.get('takeup_active'):
                mismatches += 1
        if row.get('integral_clamped') and not row.get('integral_enabled'):
            # Clamp can persist after the gate closes; not a mismatch by itself.
            pass
    return float(mismatches / checks) if checks else 0.0


# ---------------------------------------------------------------------------
# Integral metrics and selection

def _tail(rows: Sequence[dict], seconds: float) -> list:
    if not rows:
        return []
    end = float(rows[-1]['t'])
    return [row for row in rows if float(row['t']) >= end - seconds * 1000.0]


def _motor_values(rows: Sequence[dict], motor: int, key: str) -> list:
    values = []
    for row in rows:
        commands = row.get(key)
        if isinstance(commands, (list, tuple)) and motor < len(commands):
            values.append(float(commands[motor]))
    return values


def _command_chatter(rows: Sequence[dict], motor: int) -> float:
    commands = _motor_values(rows, motor, 'motor_command_abs')
    if len(commands) < 2:
        return 0.0
    duration = max(1e-6, (float(rows[-1]['t']) - float(rows[0]['t'])) * 0.001)
    return float(sum(abs(b - a) for a, b in zip(commands, commands[1:])) / duration)


def integral_metrics(samples: Sequence[dict], points: Sequence[PlanPoint],
                     motors: Sequence[int], params: dict) -> IntegralTrial:
    replayed = replay_control_state(samples, params)
    labels = hold_labels(points)
    by_segment = {}
    for row in replayed:
        by_segment.setdefault(row.get('segment'), []).append(row)
    holds = []
    ss_values, score_terms, jitter_values, overshoot_values = [], [], [], []
    gate_values, clamp_values, sat_values = [], [], []
    for label in labels:
        rows = by_segment.get(label['segment'])
        if not rows or len(rows) < 20:
            holds.append(dict(segment=label['segment'], valid=False, reason='missing hold samples'))
            continue
        rows = sorted(rows, key=lambda row: float(row['t']))
        tail1 = _tail(rows, 1.0)
        tail2 = _tail(rows, 2.0)
        errors = [float(row['target']) - float(row['actual']) for row in rows]
        tail1_errors = [float(row['target']) - float(row['actual']) for row in tail1]
        tail2_errors = [float(row['target']) - float(row['actual']) for row in tail2]
        e_ss = abs(_mean(tail1_errors))
        e_hold = _mean([abs(value) for value in tail2_errors])
        s_hold = _std([float(row['actual']) for row in tail1])
        chatter = _command_chatter(tail2, int(motors[0]))
        gate_fraction = _mean([1.0 if row.get('integral_enabled') else 0.0 for row in rows])
        clamp_fraction = _mean([1.0 if row.get('integral_clamped') else 0.0 for row in rows])
        sat_fraction = _mean([1.0 if row.get('saturated') else 0.0 for row in rows])
        i_values = [float(row.get('i_counts') or 0.0) for row in rows]
        i_change = max(i_values) - min(i_values) if i_values else 0.0
        first_enable = next((float(row['t']) - float(rows[0]['t']) for row in rows
                             if row.get('integral_enabled')), None)
        hold = dict(segment=label['segment'], position=label['position'],
                    approach=label['approach'], valid=True,
                    e_ss_deg=e_ss, e_hold_deg=e_hold, jitter_deg=s_hold,
                    chatter_counts_s=chatter, gate_fraction=gate_fraction,
                    clamp_fraction=clamp_fraction,
                    saturation_fraction=sat_fraction,
                    i_change_counts=i_change,
                    first_integral_enable_s=(first_enable / 1000.0 if first_enable is not None else None),
                    trend=[abs(_mean([float(row['target']) - float(row['actual'])
                                      for row in rows if start <= float(row['t']) - float(rows[0]['t']) < end]))
                           for start, end in ((0, 1000), (1000, 2000), (2000, 3000))],
                    errors=errors)
        holds.append(hold)
        ss_values.append(e_ss / TOL_SS_DEG)
        jitter_values.append(s_hold / TOL_JITTER_DEG)
        gate_values.append(gate_fraction)
        clamp_values.append(clamp_fraction)
        sat_values.append(sat_fraction)
        score_terms.append(e_ss / TOL_SS_DEG)
    # Overshoot/rebound on release: the first segment after the hold is the
    # commanded departure. Rebound is motion opposite to that departure before
    # the joint reaches the next target; passed is overshoot beyond it.
    next_segment = {}
    next_target = {}
    for index, point in enumerate(points):
        if point.kind != 'hold' or point.segment in next_segment:
            continue
        for later in points[index + 1:]:
            if later.segment != point.segment:
                next_segment[point.segment] = later.segment
                next_target[point.segment] = float(later.target_deg)
                break
    for label in labels:
        rows = by_segment.get(label['segment'])
        if not rows or label['segment'] not in next_segment:
            overshoot_values.append(0.0)
            continue
        following = by_segment.get(next_segment[label['segment']])
        if not following:
            overshoot_values.append(0.0)
            continue
        hold_target = float(label['target_deg'])
        destination = next_target[label['segment']]
        direction = _sign(destination - hold_target)
        if not direction:
            overshoot_values.append(0.0)
            continue
        release = sorted(rows, key=lambda row: float(row['t']))[-5:] + \
            sorted(following, key=lambda row: float(row['t']))[:100]
        rebound = max((max(0.0, -direction * (float(row['actual']) - hold_target))
                       for row in release), default=0.0)
        passed = max((max(0.0, direction * (float(row['actual']) - destination))
                      for row in release), default=0.0)
        overshoot_values.append(max(rebound, passed) / TOL_OVERSHOOT_DEG)
    valid = all(hold.get('valid') for hold in holds) and len(holds) == len(labels) and len(labels) == 6
    reasons = []
    if not valid:
        reasons.append('missing/invalid holds')
    mean_ss = _mean(score_terms) if score_terms else float('nan')
    p90_ss = _p90(score_terms) if score_terms else float('nan')
    mean_jitter = _mean(jitter_values) if jitter_values else float('nan')
    mean_overshoot = _mean(overshoot_values) if overshoot_values else 0.0
    score = 0.50 * mean_ss + 0.25 * p90_ss + 0.15 * mean_jitter + 0.10 * mean_overshoot
    gate_fraction = _mean(gate_values) if gate_values else 0.0
    clamp_fraction = _mean(clamp_values) if clamp_values else 0.0
    sat_fraction = _mean(sat_values) if sat_values else 0.0
    ki = float(params.get('ki', 0.0))
    if ki > 0 and gate_fraction < 0.2:
        reasons.append('integral gate blocked (<20% of hold samples)')
    if clamp_fraction > 0.5 and mean_ss > TOL_SS_DEG:
        reasons.append('integral clamped with unresolved droop')
    if mean_jitter > 10.0 * TOL_JITTER_DEG:
        reasons.append('hold jitter exceeds limit')
    if sat_fraction > 0.3:
        reasons.append('sustained saturation')
    i_change = max((hold.get('i_change_counts', 0.0) for hold in holds if hold.get('valid')), default=0.0)
    if ki > 0 and i_change < 2.0:
        reasons.append('integral never accumulated')
    return IntegralTrial(joint_type='', joint=-1, ki=ki, valid=not reasons,
                         score=score if not reasons else None,
                         reason='; '.join(reasons), holds=holds,
                         gate_fraction=gate_fraction, clamp_fraction=clamp_fraction,
                         saturation_fraction=sat_fraction, i_change_counts=i_change,
                         replay_mismatch=replay_mismatch(replayed))


# ---------------------------------------------------------------------------
# Reversal metrics and selection

def reversal_metrics(samples: Sequence[dict], truth: Sequence[TruthEvent],
                     motors: Sequence[int], params: dict,
                     baseline_extra_counts: Optional[float] = None) -> ReversalTrial:
    replayed = replay_control_state(samples, params)
    rows = sorted(replayed, key=lambda row: float(row['t']))
    if not rows:
        return ReversalTrial(joint_type='', joint=-1,
                             takeup_counts=float(params.get('takeup_counts', 0)),
                             exit_travel_deg=float(params.get('exit_travel_deg', 0.25)),
                             valid=False, score=None, reason='no samples')
    t = np.asarray([float(row['t']) / 1000.0 for row in rows])
    q = np.asarray([float(row['actual']) for row in rows])
    target = np.asarray([float(row['target']) for row in rows])
    events = []
    dead_values, iae_values, overshoot_values = [], [], []
    extra_total = 0.0
    sat_values = []
    timeout = False
    for event in truth:
        indices = [index for index, row in enumerate(rows)
                   if row.get('segment') == event.segment]
        if not indices:
            events.append(dict(segment=event.segment, valid=False, reason='missing event samples'))
            continue
        start = min(indices)
        t0 = t[start]
        window = (t >= t0) & (t <= t0 + REVERSAL_WINDOW_S)
        if not np.any(window):
            events.append(dict(segment=event.segment, valid=False, reason='empty event window'))
            continue
        anchor = q[start]
        direction = event.direction
        response_s = None
        for index in range(start, len(t)):
            if t[index] > t0 + REVERSAL_WINDOW_S:
                break
            cumulative = direction * (q[index] - anchor)
            if cumulative < REVERSAL_MOVE_DEG:
                continue
            end = int(np.searchsorted(t, t[index] + REVERSAL_SUSTAIN_S))
            if end <= index:
                continue
            if np.all(direction * (q[index:end] - anchor) >= REVERSAL_MOVE_DEG):
                response_s = float(t[index] - t0)
                break
        if response_s is None:
            timeout = True
        iae = float(np.trapezoid(np.abs(target[window] - q[window]), t[window]))
        overshoot = float(np.max(np.maximum(0.0, direction * (q[window] - target[window]))))
        commands = _motor_values([row for row, keep in zip(rows, window) if keep],
                                 int(motors[0]), 'motor_command_abs')
        extra = float(sum(abs(b - a) for a, b in zip(commands, commands[1:])))
        extra_total += extra
        sat_fraction = _mean([1.0 if row.get('saturated') else 0.0
                              for row, keep in zip(rows, window) if keep])
        lifting = [float(row['t']) / 1000.0 for row, keep in zip(rows, window) if keep
                   and row.get('takeup_lifting')]
        takeup_duration = 0.0
        if lifting:
            end_times = [float(row['t']) / 1000.0 for row, keep in zip(rows, window) if keep
                         and not row.get('takeup_lifting') and float(row['t']) / 1000.0 > lifting[0]]
            takeup_duration = (min(end_times) - lifting[0]) if end_times else TAKEUP_TIMEOUT_S
        if takeup_duration > 0:
            timeout = timeout or takeup_duration >= TAKEUP_TIMEOUT_S
        events.append(dict(segment=event.segment, direction=direction,
                           valid=response_s is not None,
                           response_s=response_s, iae_deg_s=iae,
                           overshoot_deg=overshoot, extra_travel_counts=extra,
                           saturation_fraction=sat_fraction,
                           takeup_duration_s=takeup_duration,
                           needs_takeup=bool(event.needs_takeup)))
        if response_s is not None:
            dead_values.append(response_s / TOL_DEAD_S)
        else:
            dead_values.append(REVERSAL_WINDOW_S / TOL_DEAD_S)
        iae_values.append(iae / TOL_IAE_DEG_S)
        overshoot_values.append(overshoot / TOL_REV_OVERSHOOT_DEG)
        sat_values.append(sat_fraction)
    score = (0.40 * _mean(dead_values) + 0.30 * _mean(iae_values) +
             0.20 * _p90(iae_values) + 0.10 * _mean(overshoot_values))
    reasons = []
    if len(events) != len(truth):
        reasons.append('missing events')
    if timeout:
        reasons.append('takeup/reversal timeout')
    if _mean(sat_values) > 0.2:
        reasons.append('saturation')
    extra_limit = (baseline_extra_counts + 300.0) if baseline_extra_counts is not None else 1e9
    if extra_total > extra_limit * max(1, len(truth)):
        reasons.append('excessive extra motor travel')
    return ReversalTrial(joint_type='', joint=-1,
                         takeup_counts=float(params.get('takeup_counts', 0)),
                         exit_travel_deg=float(params.get('exit_travel_deg', 0.25)),
                         valid=not reasons, score=score if not reasons else None,
                         reason='; '.join(reasons), events=events,
                         worst_event_s=max((event.get('response_s') or 0.0 for event in events), default=0.0),
                         extra_travel_counts=extra_total,
                         saturation_fraction=_mean(sat_values) if sat_values else 0.0,
                         takeup_timeout=timeout)


# ---------------------------------------------------------------------------
# Selection rules

def choose_integral(history: Sequence[IntegralTrial], noise: float = 0.02) -> dict:
    if not history:
        return dict(selected_ki=0.0, reason='no trials')
    baseline = next((trial for trial in history if trial.ki == 0.0 and trial.valid), None)
    if baseline is None:
        return dict(selected_ki=0.0, reason='no valid Ki=0 baseline')
    best = baseline
    best_reason = 'Ki=0 baseline; no candidate improved beyond noise'
    threshold = max(2.0 * max(noise, 0.0), 0.05)
    for trial in history:
        if trial.ki <= 0 or not trial.valid or trial.score is None:
            continue
        if trial.score + threshold < best.score:
            if trial.score < best.score:
                best = trial
                best_reason = (f'Ki={trial.ki} improves J_I by '
                               f'{baseline.score - trial.score:.3f} > {threshold:.3f}')
    return dict(selected_ki=best.ki, reason=best_reason,
                baseline=baseline, selected=best, threshold=threshold)


def takeup_candidates(history: Sequence[ReversalTrial]) -> list:
    """Group single-tendon and AA results by takeup/exit; a nonzero candidate is
    feasible only when both transmission classes passed."""
    groups: dict = {}
    for trial in history:
        key = (float(trial.takeup_counts), float(trial.exit_travel_deg), trial.joint_type)
        groups[key] = trial
    candidates = []
    for (takeup, exit_travel, _), trial in sorted(groups.items()):
        if (takeup, exit_travel) in [(c['takeup_counts'], c['exit_travel_deg']) for c in candidates]:
            continue
        single = [item for item in groups.values()
                  if item.takeup_counts == takeup and item.exit_travel_deg == exit_travel
                  and item.joint_type != 'aa']
        dual = [item for item in groups.values()
                if item.takeup_counts == takeup and item.exit_travel_deg == exit_travel
                and item.joint_type == 'aa']
        candidates.append(dict(takeup_counts=takeup, exit_travel_deg=exit_travel,
                               single=single, dual=dual))
    return candidates


def choose_takeup(history: Sequence[ReversalTrial], noise: float = 0.02) -> dict:
    groups = takeup_candidates(history)
    if not groups:
        return dict(selected_takeup=0.0, selected_exit=0.25, reason='no takeup trials')
    def group_score(group):
        trials = list(group['single']) + list(group['dual'])
        if not trials or any(not trial.valid or trial.score is None for trial in trials):
            return None
        return max(trial.score for trial in trials)
    baseline = next((g for g in groups if g['takeup_counts'] == 0 and g['exit_travel_deg'] == 0.25), None)
    baseline_score = group_score(baseline) if baseline else None
    # A global nonzero takeup is only defensible when the SAME Ki=0/takeup=0
    # baseline was measured on both a single-tendon and the AA joint.
    if baseline is None or not baseline['single'] or not baseline['dual'] or baseline_score is None:
        return dict(selected_takeup=0.0, selected_exit=0.25,
                    reason='missing valid single+AA takeup=0 baseline; keep takeup disabled',
                    baseline_score=baseline_score, threshold=0.0)
    threshold = max(2.0 * max(noise, 0.0), 0.05)
    best = None
    best_score = float('inf')
    for group in groups:
        if group['takeup_counts'] == 0:
            continue
        if not group['single'] or not group['dual']:
            continue
        score = group_score(group)
        if score is None:
            continue
        if baseline_score is not None and score + threshold >= baseline_score:
            continue
        if score < best_score - 1e-9:
            best, best_score = group, score
    if best is None:
        return dict(selected_takeup=0.0, selected_exit=0.25,
                    reason='no nonzero candidate beat baseline for both single and AA',
                    baseline_score=baseline_score, threshold=threshold)
    return dict(selected_takeup=float(best['takeup_counts']),
                selected_exit=float(best['exit_travel_deg']),
                reason=f'worst-joint J_R improved to {best_score:.3f}',
                baseline_score=baseline_score, threshold=threshold)


def validate_combined(baseline: dict, candidate: dict, noise: float = 0.02) -> tuple:
    """Compare per-type integral and reversal metrics against a Ki=0/takeup=0 baseline."""
    reasons = []
    tolerance = max(0.10, 2.0 * max(noise, 0.0))
    for joint_type in TYPES:
        base = baseline.get(joint_type, {})
        cand = candidate.get(joint_type, {})
        if not cand:
            reasons.append(f'{joint_type}: candidate missing')
            continue
        if cand.get('hard_failure'):
            reasons.append(f"{joint_type}: hard failure {cand['hard_failure']}")
            continue
        for key in ('integral_score', 'reversal_score'):
            base_value = base.get(key)
            cand_value = cand.get(key)
            if base_value is None or cand_value is None:
                continue
            if cand_value > base_value * (1.0 + tolerance) + 0.02:
                reasons.append(f'{joint_type}: {key} degraded {base_value:.3f}->{cand_value:.3f}')
        base_rmse = base.get('dynamic_rmse_deg')
        cand_rmse = cand.get('dynamic_rmse_deg')
        if base_rmse and cand_rmse and cand_rmse > base_rmse * 1.15 + 0.05:
            reasons.append(f'{joint_type}: dynamic RMSE degraded {base_rmse:.3f}->{cand_rmse:.3f}')
        base_phase = base.get('dynamic_phase_delay_s')
        cand_phase = cand.get('dynamic_phase_delay_s')
        if base_phase is not None and cand_phase is not None and cand_phase > base_phase * 1.2 + 0.02:
            reasons.append(f'{joint_type}: dynamic phase delay degraded')
    return (not reasons, reasons)
