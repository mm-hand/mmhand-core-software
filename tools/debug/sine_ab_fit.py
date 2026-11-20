#!/usr/bin/env python3
"""Fit a replay sine test recorded with fast-reversal OFF and ON.

职责：从两条录制目录提取单关节正弦响应，做最小二乘正弦拟合和换向指标对比。
线程：单进程离线；不访问串口、不修改参数。
输入：OFF/ON 录制目录、关节号、正弦频率。
输出：report.md、metrics.json、fit.png。
状态所有权：无设备状态；只读录制 CSV。
禁止依赖：禁止 UI、运动命令、参数写入。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path

import numpy as np

JOINT_CSV = ('02_joint_feedback.csv', '02_关节总表.csv')
EVENT_CSV = ('06_operation_events.csv', '06_操作事件总表.csv')


def _find(directory: Path, names: tuple[str, ...]) -> Path:
    for name in names:
        path = directory / name
        if path.is_file():
            return path
    raise FileNotFoundError(f'{directory}: none of {names}')


def _events(directory: Path) -> list[tuple[datetime, str]]:
    path = _find(directory, EVENT_CSV)
    rows = []
    with path.open(encoding='utf-8-sig', newline='') as handle:
        for row in csv.DictReader(handle):
            rows.append((datetime.fromisoformat(row['host_time']), row.get('detail') or ''))
    return rows


def _samples(directory: Path, joint: int) -> list[dict]:
    path = _find(directory, JOINT_CSV)
    prefix = f'J{joint:02d}'
    rows = []
    with path.open(encoding='utf-8-sig', newline='') as handle:
        for row in csv.DictReader(handle):
            if row.get(f'{prefix}_active') != '1':
                continue
            if row.get(f'{prefix}_current_valid') != '1' or not row.get(f'{prefix}_target_deg'):
                continue
            host = datetime.fromisoformat(row['host_time']).timestamp()
            rows.append(dict(t=host, q=float(row[f'{prefix}_current_deg']),
                             target=float(row[f'{prefix}_target_deg'])))
    if not rows:
        raise RuntimeError(f'{directory}: no valid {prefix} samples')
    rows.sort(key=lambda r: r['t'])
    return rows


def _window(rows: list[dict], events: list[tuple[datetime, str]]) -> tuple[float, float]:
    """Span of the replay episode that actually moves this joint.

    A long recording can hold several replays (J00 then J01); pick the episode
    with the most target reversals instead of assuming the latest one. Sample
    dropouts are ignored, and a joint release that steps the target by more than
    5 deg in one sample is not part of the sine.
    """
    starts = sorted(t.timestamp() for t, detail in events if 'Replay started' in detail)
    if starts:
        bounds = [(a, b) for a, b in zip(starts, starts[1:] + [starts[-1] + 180.0])]
    else:
        bounds = [(rows[0]['t'], rows[-1]['t'])]
    best, best_turns, best_first = None, -1, None
    for low, high in bounds:
        window_rows = [r for r in rows if low - 0.5 <= r['t'] <= high]
        changes, turns, previous = [], 0, 0
        for before, row in zip(window_rows, window_rows[1:]):
            dt = row['t'] - before['t']
            delta = row['target'] - before['target']
            if abs(delta) <= 1e-4:
                continue
            if abs(delta) > 5 and dt < 0.2:  # joint release step, not the replay
                continue
            changes.append((before['t'], row['t']))
            direction = 1 if delta > 0 else -1
            if previous and direction != previous:
                turns += 1
            previous = direction
        if changes and turns >= best_turns:
            best, best_turns, best_first = changes, turns, low
    if not best:
        raise RuntimeError('no moving target in recording')
    first = best[0][0]
    last = min(best[-1][1], first + 180.0)
    return first, last


def _sin_fit(t: np.ndarray, y: np.ndarray, frequency: float) -> dict:
    w = 2 * math.pi * frequency
    design = np.column_stack((np.sin(w * t), np.cos(w * t), np.ones(len(t))))
    coef, *_ = np.linalg.lstsq(design, y, rcond=None)
    fitted = design @ coef
    return dict(coef=coef, fitted=fitted, amplitude=float(math.hypot(*coef[:2])),
                residual=float(np.sqrt(np.mean((y - fitted) ** 2))))


def _peak_overshoot(rows: list[dict], frequency: float, hold_s: float = 0.6) -> list[float]:
    """How far the joint runs past each target extremum after the turn."""
    turns = []
    previous = 0
    for i in range(1, len(rows)):
        delta = rows[i]['target'] - rows[i - 1]['target']
        direction = 1 if delta > 1e-6 else -1 if delta < -1e-6 else 0
        if direction and previous and direction != previous:
            turns.append((rows[i - 1]['t'], -previous))  # new direction
        if direction:
            previous = direction
    values = []
    for turn, new_direction in turns:
        peak = rows[next(i for i in range(len(rows)) if rows[i]['t'] >= turn)]
        segment = [r for r in rows if turn - 0.05 <= r['t'] <= turn + hold_s]
        if not segment:
            continue
        if new_direction < 0:  # was rising, now falling: overshoot above peak
            value = max(r['q'] for r in segment) - peak['target']
        else:
            value = peak['target'] - min(r['q'] for r in segment)
        values.append(value)
    return values


def _reversal_events(rows: list[dict], threshold: float = 0.2) -> list[tuple[float, float | None]]:
    """(target reversal time, sustained threshold response time or None)."""
    turns = []
    previous = 0
    for i in range(1, len(rows)):
        delta = rows[i]['target'] - rows[i - 1]['target']
        direction = 1 if delta > 1e-6 else -1 if delta < -1e-6 else 0
        if direction and previous and direction != previous:
            turns.append((i, direction))
        if direction:
            previous = direction
    events = []
    for start, direction in turns:
        anchor = rows[start]['q']
        response = None
        for i in range(start, len(rows)):
            anchor = min(anchor, rows[i]['q']) if direction > 0 else max(anchor, rows[i]['q'])
            end = i + 1
            while end < len(rows) and rows[end]['t'] - rows[i]['t'] < 0.03:
                end += 1
            if all(direction * (rows[k]['q'] - anchor) >= threshold for k in range(i, end)):
                response = rows[i]['t']
                break
        events.append((rows[start]['t'], response))
    return events


def _reversal_delays(rows: list[dict], threshold: float = 0.2) -> list[float]:
    return [response - turn for turn, response in _reversal_events(rows, threshold)
            if response is not None]


def analyze(directory: Path, joint: int, frequency: float) -> dict:
    events = _events(directory)
    rows = _samples(directory, joint)
    first, last = _window(rows, events)
    segment = [r for r in rows if first <= r['t'] <= last]
    t = np.array([r['t'] - first for r in segment])
    q = np.array([r['q'] for r in segment])
    target = np.array([r['target'] for r in segment])
    discard = max(1.0, 1.0 / frequency)  # steady state: skip the first cycle
    keep = t >= discard
    if keep.sum() < 20:
        raise RuntimeError(f'{directory}: sine window too short ({len(segment)} samples)')
    actual = _sin_fit(t[keep], q[keep], frequency)
    reference = _sin_fit(t[keep], target[keep], frequency)
    phase = (math.atan2(reference['coef'][1], reference['coef'][0])
             - math.atan2(actual['coef'][1], actual['coef'][0]) + math.pi) % (2 * math.pi) - math.pi
    w = 2 * math.pi * frequency
    identifiable = actual['amplitude'] >= 0.15 and actual['amplitude'] >= 2 * actual['residual']
    error = q[keep] - target[keep]
    delays = _reversal_delays(segment)
    overshoot = _peak_overshoot(segment, frequency)
    return dict(
        directory=str(directory), joint=joint, frequency_hz=frequency,
        duration_s=float(last - first), samples=int(len(segment)),
        amplitude_ratio=actual['amplitude'] / reference['amplitude'] if reference['amplitude'] else None,
        phase_lag_deg=math.degrees(phase) if identifiable else None,
        phase_delay_ms=(phase / w * 1000) if identifiable else None,
        sine_residual_rmse_deg=actual['residual'],
        tracking_rmse_deg=float(np.sqrt(np.mean(error ** 2))),
        tracking_max_abs_deg=float(np.max(np.abs(error))),
        reversals=len(delays),
        reversal_delay_mean_ms=float(np.mean(delays) * 1000) if delays else None,
        reversal_delay_median_ms=float(np.median(delays) * 1000) if delays else None,
        reversal_delay_p95_ms=float(np.percentile(delays, 95) * 1000) if delays else None,
        peak_overshoot_median_deg=float(np.median(overshoot)) if overshoot else None,
        peak_overshoot_p95_deg=float(np.percentile(overshoot, 95)) if overshoot else None,
        trace=dict(t=[round(float(x), 4) for x in t[keep]], q=[round(float(x), 4) for x in q[keep]],
                   target=[round(float(x), 4) for x in target[keep]]),
    )


def _row(label: str, m: dict) -> str:
    def fmt(value, unit='', digits=2):
        return '—' if value is None else f'{value:.{digits}f}{unit}'
    return (f"| {label} | {m['duration_s']:.1f} s | {fmt(m['amplitude_ratio'], '', 3)} | "
            f"{fmt(m['phase_lag_deg'], '°', 1)} | {fmt(m['phase_delay_ms'], ' ms', 0)} | "
            f"{fmt(m['sine_residual_rmse_deg'], '°')} | {fmt(m['tracking_rmse_deg'], '°')} | "
            f"{fmt(m['tracking_max_abs_deg'], '°')} | {m['reversals']} | "
            f"{fmt(m['reversal_delay_mean_ms'], ' ms', 0)} | "
            f"{fmt(m['reversal_delay_median_ms'], ' ms', 0)} | "
            f"{fmt(m['peak_overshoot_p95_deg'], '°')} |")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--off', type=Path, required=True)
    parser.add_argument('--on', type=Path)
    parser.add_argument('--joint', type=int, default=0)
    parser.add_argument('--frequency', type=float, default=0.2)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    out = args.out or args.off.parent / f"sine_ab_{args.off.name}"
    out.mkdir(parents=True, exist_ok=True)

    runs = [('OFF', analyze(args.off, args.joint, args.frequency))]
    if args.on:
        runs.append(('ON', analyze(args.on, args.joint, args.frequency)))
    metrics = {label: {k: v for k, v in m.items() if k != 'trace'} for label, m in runs}
    (out / 'metrics.json').write_text(json.dumps(metrics, indent=1), encoding='utf-8')

    header = ("| run | window | amplitude ratio | phase lag | phase delay | sine residual RMSE | "
              "tracking RMSE | max abs error | reversals | reversal delay mean | median | peak overshoot p95 |")
    lines = [f"# J{args.joint:02d} {args.frequency:g} Hz sine fit", '',
             header, '|' + '---|' * 12]
    lines += [_row(label, m) for label, m in runs]
    (out / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for label, m in runs:
            trace = m['trace']
            axes[0].plot(trace['t'], trace['target'], '--', lw=1, color='0.5')
            axes[0].plot(trace['t'], trace['q'], lw=1.2, label=f"{label} measured")
            axes[1].plot(trace['t'], np.array(trace['q']) - np.array(trace['target']), lw=1, label=label)
        axes[0].set_ylabel('angle (deg)')
        axes[0].legend(loc='upper right')
        axes[0].set_title('target (dashed) vs measured')
        axes[1].set_ylabel('error (deg)')
        axes[1].set_xlabel('time (s)')
        axes[1].legend(loc='upper right')
        figure.tight_layout()
        figure.savefig(out / 'fit.png', dpi=120)
    except ImportError:
        pass
    print('\n'.join(lines))
    print(f'\nWrote {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
