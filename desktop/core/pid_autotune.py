"""pid_autotune.py - MMHand Desktop 模块契约。

职责：PID 自动整定的纯逻辑（复合轨迹、12+3 工况评分、离散 GP-EI），不打开串口。
线程：纯函数/对象，调用线程执行；不创建后台线程。
输入：安全角度范围、轨迹参数、采样行、评分阈值和候选网格。
输出：轨迹点、评分、硬失败标志和下一个候选。
状态所有权：只拥有优化器内存状态，不拥有设备状态或参数文件。
禁止依赖：禁止导入 Qt 控件、串口或直接操作下位机通信。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence

import numpy as np


POSITION_BINS = (0.20, 0.50, 0.80)
SPEED_BINS = ('low', 'high')
DIRECTIONS = ('flex', 'extend')
DYNAMIC_BINS = tuple(
    (position, speed, direction)
    for position in POSITION_BINS
    for speed in SPEED_BINS
    for direction in DIRECTIONS
)


@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    target_deg: float
    phase: str
    position_bin: Optional[float] = None
    speed_bin: Optional[str] = None
    direction: Optional[str] = None


@dataclass(frozen=True)
class ScoreThresholds:
    dynamic_rmse_deg: float = 1.0
    hold_bias_deg: float = 0.5
    hold_jitter_deg: float = 0.2
    overshoot_deg: float = 1.5
    motor_jitter_counts: float = 50.0

    def validate(self) -> None:
        for name in ('dynamic_rmse_deg', 'hold_bias_deg', 'hold_jitter_deg', 'overshoot_deg', 'motor_jitter_counts'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')


@dataclass(frozen=True)
class TrialScore:
    score: float
    dynamic_score: float
    hold_score: float
    overshoot_score: float
    motor_score: float
    dynamic_bins: int
    hold_bins: int
    missing_bins: tuple
    failures: tuple

    @property
    def valid(self) -> bool:
        return not self.failures and self.dynamic_bins == 12 and self.hold_bins == 3


@dataclass(frozen=True)
class GridPoint:
    kp: float
    kd: float


@dataclass
class Observation:
    point: GridPoint
    score: Optional[float]
    valid: bool
    reason: str = ''


def _bin_for_position(value: float) -> Optional[float]:
    if not math.isfinite(value):
        return None
    if value <= 0.35:
        return 0.20
    if value < 0.65:
        return 0.50
    return 0.80


def _move(start: float, end: float, speed: float, acceleration: float, start_time: float,
          dt: float, speed_bin: str, direction: str) -> list[TrajectoryPoint]:
    """Return a trapezoidal/point-to-point move; only constant-speed samples are scored."""
    distance = abs(end - start)
    if distance <= 1e-9:
        return []
    sign = 1.0 if end > start else -1.0
    speed = max(abs(speed), 1e-6)
    acceleration = max(abs(acceleration), 1e-6)
    ramp_time = speed / acceleration
    ramp_distance = 0.5 * acceleration * ramp_time * ramp_time
    if 2.0 * ramp_distance >= distance:
        peak_speed = math.sqrt(distance * acceleration)
        ramp_time = peak_speed / acceleration
        constant_time = 0.0
        speed = peak_speed
    else:
        constant_time = (distance - 2.0 * ramp_distance) / speed
    total_time = 2.0 * ramp_time + constant_time
    count = max(2, int(math.ceil(total_time / dt)))
    points = []
    for index in range(count + 1):
        local = min(total_time, index * dt)
        if local <= ramp_time:
            travelled = 0.5 * acceleration * local * local
            scoring = False
        elif local <= ramp_time + constant_time:
            travelled = 0.5 * acceleration * ramp_time * ramp_time + speed * (local - ramp_time)
            scoring = constant_time > 0.0
        else:
            remaining = total_time - local
            travelled = distance - 0.5 * acceleration * remaining * remaining
            scoring = False
        travelled = min(distance, max(0.0, travelled))
        target = start + sign * travelled
        phase = 'dynamic' if scoring else 'transition'
        points.append(TrajectoryPoint(
            time_s=start_time + local,
            target_deg=target,
            phase=phase,
            position_bin=_bin_for_position((target - start) / sign / distance if distance else 0.0) if scoring else None,
            speed_bin=speed_bin if scoring else None,
            direction=direction if scoring else None,
        ))
    return points


def build_composite_trajectory(q_lo: float, q_hi: float, *, v_low: float = 0.12,
                               v_high: float = 0.50, hold_s: float = 1.5,
                               dt: float = 0.01) -> list[TrajectoryPoint]:
    """3 positions x 2 speeds x 2 directions plus three holds, normalized to safe span."""
    if not (math.isfinite(q_lo) and math.isfinite(q_hi) and q_hi > q_lo):
        raise ValueError('invalid safe angle span')
    if not (0.0 < v_low < v_high <= 1.0):
        raise ValueError('v_low and v_high must be fractions of span with v_low < v_high')
    if hold_s <= 0 or dt <= 0:
        raise ValueError('hold_s and dt must be positive')
    span = q_hi - q_lo
    r = lambda value: q_lo + value * span
    acceleration = max(v_high * span * 2.0, 1.0)
    points: list[TrajectoryPoint] = []
    now = 0.0
    # Start from a repeatable low position and settle before the first scored move.
    points.extend(_move(r(0.05), r(0.10), v_low * span, acceleration, now, dt, 'low', 'flex'))
    now = points[-1].time_s if points else 0.0
    points.append(TrajectoryPoint(now + 0.5, r(0.10), 'transition'))
    now += 0.5
    for speed_bin, fraction in (('low', v_low), ('high', v_high)):
        speed = fraction * span
        for direction, start, end in (('flex', 0.10, 0.90), ('extend', 0.90, 0.10)):
            moved = _move(r(start), r(end), speed, acceleration, now, dt, speed_bin, direction)
            points.extend(moved)
            now = moved[-1].time_s if moved else now
            points.append(TrajectoryPoint(now + 0.2, r(end), 'transition'))
            now += 0.2
    # Three hold zones; scoring uses the tail of each hold in the runner.
    for position in (0.20, 0.50, 0.80):
        moved = _move(r(0.10), r(position), v_low * span, acceleration, now, dt, 'low', 'flex')
        points.extend(moved)
        now = moved[-1].time_s if moved else now
        start = now
        while now <= start + hold_s + 1e-9:
            points.append(TrajectoryPoint(now, r(position), 'hold', position_bin=position))
            now += dt
        now = points[-1].time_s
    # Return to the start without adding another scored condition.
    back = _move(r(0.80), r(0.10), v_low * span, acceleration, now, dt, 'low', 'extend')
    for point in back:
        points.append(TrajectoryPoint(point.time_s, point.target_deg, 'transition'))
    return points


def _rmse(rows: Iterable[dict]) -> float:
    values = [float(row['actual']) - float(row['target']) for row in rows]
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float('nan')


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else float('nan')


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, value))


def evaluate_trial(rows: Sequence[dict], thresholds: ScoreThresholds) -> TrialScore:
    """Score feedback rows merged with trajectory metadata.

    Required row keys: phase, target, actual.  Dynamic rows additionally need
    position_bin, speed_bin and direction.  Hold rows may include motor_cmd.
    """
    thresholds.validate()
    failures = []
    dynamic: dict = {}
    for row in rows:
        if row.get('phase') != 'dynamic':
            continue
        key = (row.get('position_bin'), row.get('speed_bin'), row.get('direction'))
        if key[0] is None or key[1] is None or key[2] is None:
            continue
        dynamic.setdefault(key, []).append(row)
    missing = tuple(key for key in DYNAMIC_BINS if key not in dynamic)
    dynamic_rmse = {}
    for key, values in dynamic.items():
        dynamic_rmse[key] = _rmse(values)
    if missing:
        failures.append(f'missing {len(missing)} dynamic bins')
    if any(not math.isfinite(value) for value in dynamic_rmse.values()):
        failures.append('non-finite dynamic RMSE')
    dynamic_scores = [_clip01(value / thresholds.dynamic_rmse_deg) for value in dynamic_rmse.values()]
    dynamic_score = 0.7 * _mean(dynamic_scores) + 0.3 * float(np.percentile(dynamic_scores, 90)) if dynamic_scores else 1.0
    # Hold metrics use the last 0.5 s of each hold zone.
    hold_scores, overshoot_scores, motor_scores = [], [], []
    for position in POSITION_BINS:
        hold = [row for row in rows if row.get('phase') == 'hold' and row.get('position_bin') == position]
        if not hold:
            continue
        hold.sort(key=lambda row: float(row.get('time', 0.0)))
        tail = hold[-max(1, int(0.5 / 0.01)):]
        bias = _mean([float(row['actual']) - float(row['target']) for row in tail])
        jitter = math.sqrt(_mean([(float(row['actual']) - float(row['target'])) ** 2 for row in tail]))
        hold_scores.append(_clip01(abs(bias) / thresholds.hold_bias_deg + jitter / thresholds.hold_jitter_deg))
        overshoot = max((abs(float(row['actual']) - float(row['target'])) for row in tail), default=0.0)
        overshoot_scores.append(_clip01(overshoot / thresholds.overshoot_deg))
        commands = [float(row['motor_cmd']) for row in tail if 'motor_cmd' in row]
        if len(commands) >= 2:
            motor_scores.append(_clip01((sum(abs(b - a) for a, b in zip(commands, commands[1:])) /
                                        max(1, len(commands) - 1)) / thresholds.motor_jitter_counts))
    if len(hold_scores) != 3:
        failures.append(f'missing {3 - len(hold_scores)} hold bins')
    hold_score = _mean(hold_scores) if hold_scores else 1.0
    overshoot_score = _mean(overshoot_scores) if overshoot_scores else 1.0
    motor_score = _mean(motor_scores) if motor_scores else 1.0
    score = 0.65 * dynamic_score + 0.20 * hold_score + 0.10 * overshoot_score + 0.05 * motor_score
    return TrialScore(score, dynamic_score, hold_score, overshoot_score, motor_score,
                      len(dynamic), len(hold_scores), missing, tuple(failures))


def _matern52(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    distance = np.sqrt(np.maximum(0.0, ((a[:, None, :] - b[None, :, :]) ** 2).sum(axis=2)))
    scaled = math.sqrt(5.0) * distance
    return (1.0 + scaled + scaled * scaled / 3.0) * np.exp(-scaled)


class DiscreteEI:
    """Small discrete GP-EI over Kp/Kd; no scikit-learn dependency."""

    def __init__(self, grid: Sequence[GridPoint], initial: Sequence[GridPoint], *, noise: float = 0.05,
                 xi: float = 0.02):
        self.grid = list(grid)
        if not self.grid:
            raise ValueError('grid must not be empty')
        self.initial = list(initial)
        self.noise = noise
        self.xi = xi
        self.observations: list[Observation] = []
        self._tried = set()

    def _index(self, point: GridPoint) -> tuple:
        for index, candidate in enumerate(self.grid):
            if abs(candidate.kp - point.kp) < 1e-9 and abs(candidate.kd - point.kd) < 1e-9:
                return index, candidate
        raise ValueError(f'point {point} is not in the grid')

    def observe(self, point: GridPoint, score: Optional[float], valid: bool, reason: str = '') -> None:
        index, point = self._index(point)
        self._tried.add(index)
        self.observations.append(Observation(point, score, valid, reason))

    def suggest(self) -> Optional[GridPoint]:
        for point in self.initial:
            index, point = self._index(point)
            if index not in self._tried:
                return point
        valid = [item for item in self.observations if item.valid and item.score is not None]
        if len(valid) < 3:
            for index, point in enumerate(self.grid):
                if index not in self._tried:
                    return point
            return None
        kp_values = np.array([point.kp for point in self.grid])
        kd_values = np.array([point.kd for point in self.grid])
        kp_span = max(1e-9, float(kp_values.max() - kp_values.min()))
        kd_span = max(1e-9, float(kd_values.max() - kd_values.min()))
        def normalize(points):
            values = np.array([[point.kp, point.kd] for point in points], dtype=float)
            return np.column_stack(((values[:, 0] - kp_values.min()) / kp_span,
                                    (values[:, 1] - kd_values.min()) / kd_span))
        train = normalize([item.point for item in valid])
        targets = np.array([item.score for item in valid], dtype=float)
        mean, std = float(targets.mean()), float(targets.std())
        if std < 1e-12:
            std = 1.0
        targets = (targets - mean) / std
        candidates = normalize(self.grid)
        best = float(targets.min())
        try:
            kernel = _matern52(train, train) + self.noise * np.eye(len(train))
            chol = np.linalg.cholesky(kernel)
            alpha = np.linalg.solve(chol.T, np.linalg.solve(chol, targets))
            cross = _matern52(candidates, train)
            mu = mean + std * (cross @ alpha)
            variance = std * std * np.maximum(1e-12, 1.0 - np.sum(
                np.linalg.solve(chol, cross.T) ** 2, axis=0))
            sigma = np.sqrt(variance)
            improvement = best - (mu - mean) / max(std, 1e-9) - self.xi
            z = improvement / sigma
            expected = improvement * _normal_cdf(z) + sigma * _normal_pdf(z)
        except (np.linalg.LinAlgError, ValueError, FloatingPointError):
            expected = np.zeros(len(self.grid))
        best_index = None
        best_value = -float('inf')
        for index in range(len(self.grid)):
            if index in self._tried:
                continue
            value = float(expected[index])
            if value > best_value:
                best_index, best_value = index, value
        if best_index is None:
            return None
        return self.grid[best_index]

    @property
    def best(self) -> Optional[Observation]:
        valid = [item for item in self.observations if item.valid and item.score is not None]
        return min(valid, key=lambda item: item.score) if valid else None


def _normal_pdf(value):
    values = np.asarray(value, dtype=float)
    return np.exp(-0.5 * values * values) / math.sqrt(2.0 * math.pi)


def _normal_cdf(value):
    values = np.asarray(value, dtype=float)
    return 0.5 * (1.0 + np.vectorize(math.erf)(values / math.sqrt(2.0)))
