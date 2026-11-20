#!/usr/bin/env python3
"""Export one recording's sine replays as plain CSV plus a metrics table.

Usage:
  python tools/debug/export_sine_runs.py --recording logs/recordings_x \
      --run 0=ON@2026-11-20T13:13:46.888 --run 1=ON@2026-11-20T13:18:34.177 \
      --run 1=OFF@2026-11-20T13:25:51.332 --out logs/sine_x/data
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sine_ab_fit import _events, _peak_overshoot, _reversal_events, _samples, _sin_fit


def episode(rows: list[dict], start: str, span: float = 90.0) -> list[dict]:
    low = datetime.fromisoformat(start).timestamp() - 0.5
    high = low + span
    changes = []
    for before, row in zip(rows, rows[1:]):
        if row['t'] < low or row['t'] > high:
            continue
        dt = row['t'] - before['t']
        delta = row['target'] - before['target']
        if abs(delta) > 1e-4 and not (abs(delta) > 5 and dt < 0.2):
            changes.append((before['t'], row['t']))
    if not changes:
        raise RuntimeError(f'no target movement after {start}')
    first, last = changes[0][0], changes[-1][1]
    return [r for r in rows if first <= r['t'] <= last]


def measure(segment: list[dict], frequency: float) -> dict:
    first = segment[0]['t']
    t = np.array([r['t'] - first for r in segment])
    q = np.array([r['q'] for r in segment])
    target = np.array([r['target'] for r in segment])
    keep = t >= max(1.0, 1.0 / frequency)
    actual = _sin_fit(t[keep], q[keep], frequency)
    reference = _sin_fit(t[keep], target[keep], frequency)
    phase = (math.atan2(reference['coef'][1], reference['coef'][0])
             - math.atan2(actual['coef'][1], actual['coef'][0]) + math.pi) % (2 * math.pi) - math.pi
    delays = [response - turn for turn, response in _reversal_events(segment) if response is not None]
    overshoot = _peak_overshoot(segment, frequency)
    error = q[keep] - target[keep]
    return dict(
        duration_s=round(t[-1], 3), samples=len(segment),
        amplitude_ratio=round(actual['amplitude'] / reference['amplitude'], 4),
        phase_lag_deg=round(math.degrees(phase), 2),
        phase_delay_ms=round(phase / (2 * math.pi * frequency) * 1000, 1),
        sine_residual_rmse_deg=round(actual['residual'], 3),
        tracking_rmse_deg=round(float(np.sqrt(np.mean(error ** 2))), 3),
        tracking_max_abs_deg=round(float(np.max(np.abs(error))), 3),
        reversals=len(delays),
        reversal_delay_mean_ms=round(float(np.mean(delays)) * 1000, 1) if delays else None,
        reversal_delay_median_ms=round(float(np.median(delays)) * 1000, 1) if delays else None,
        reversal_delay_p95_ms=round(float(np.percentile(delays, 95)) * 1000, 1) if delays else None,
        peak_overshoot_p95_deg=round(float(np.percentile(overshoot, 95)), 3) if overshoot else None,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--run', action='append', required=True,
                        help='JOINT=STATE@ISO_START, e.g. 0=ON@2026-11-20T13:13:46.888')
    parser.add_argument('--frequency', type=float, default=0.2)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    events = _events(args.recording)

    combined, metrics = [], []
    for spec in args.run:
        head, start = spec.split('@', 1)
        joint_text, state = head.split('=', 1)
        joint = int(joint_text)
        rows = _samples(args.recording, joint)
        segment = episode(rows, start)
        base = segment[0]['t']
        name = f'J{joint:02d}_{state}'
        path = args.out / f'{name}.csv'
        with path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['t_s', 'target_deg', 'measured_deg', 'error_deg'])
            for row in segment:
                t = row['t'] - base
                writer.writerow([f'{t:.4f}', f'{row["target"]:.4f}',
                                 f'{row["q"]:.4f}', f'{row["q"] - row["target"]:.4f}'])
                combined.append([name, joint, state, f'{t:.4f}', f'{row["target"]:.4f}',
                                 f'{row["q"]:.4f}', f'{row["q"] - row["target"]:.4f}'])
        result = measure(segment, args.frequency)
        result.update(run=name, joint=joint, state=state, recording=str(args.recording),
                      start=datetime.fromtimestamp(base).isoformat())
        metrics.append(result)
        print(name, result['duration_s'], 's', result['samples'], 'samples')

    with (args.out / 'sine_runs_long.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['run', 'joint', 'state', 't_s', 'target_deg', 'measured_deg', 'error_deg'])
        writer.writerows(combined)
    keys = list(metrics[0])
    with (args.out / 'metrics.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(metrics)
    (args.out / 'metrics.json').write_text(json.dumps(metrics, indent=1), encoding='utf-8')
    print('wrote', args.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
