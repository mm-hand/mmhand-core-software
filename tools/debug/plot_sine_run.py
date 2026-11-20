#!/usr/bin/env python3
"""Plot one recorded sine run (reference vs measured) with reversal delays."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sine_ab_fit import _events, _reversal_events, _samples, _window, analyze


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--joint', type=int, default=0)
    parser.add_argument('--frequency', type=float, default=0.2)
    parser.add_argument('--label', default='ON')
    parser.add_argument('--zoom', type=float, nargs=2, default=(15.0, 25.0))
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    metrics = analyze(args.recording, args.joint, args.frequency)
    rows = _samples(args.recording, args.joint)
    first, last = _window(rows, _events(args.recording))
    segment = [r for r in rows if first <= r['t'] <= last]
    t = np.array([r['t'] - first for r in segment])
    q = np.array([r['q'] for r in segment])
    target = np.array([r['target'] for r in segment])
    events = [(turn - first, None if response is None else response - first)
              for turn, response in _reversal_events(segment)]

    figure, (top, bottom) = plt.subplots(
        2, 1, figsize=(12, 7.5), gridspec_kw=dict(height_ratios=(2, 1)))
    for axes in (top, bottom):
        axes.plot(t, target, '--', color='0.45', lw=1.0, label='Reference')
        axes.plot(t, q, '-', color='#1f5fbf', lw=1.4, label='Measured')
        axes.grid(alpha=.25)
        axes.set_ylabel('Joint angle (deg)')
    top.set_xlim(0, t[-1])
    span = max(target.max(), q.max()) - min(target.min(), q.min())
    pad = max(4.0, 0.10 * span)
    low, high = min(target.min(), q.min()) - pad, max(target.max(), q.max()) + pad
    top.set_ylim(low, high)
    top.legend(loc='upper right', ncol=2, frameon=False)
    top.set_title(f'J{args.joint:02d}  {args.frequency:g} Hz sine  ·  fast reversal {args.label}', pad=26)
    top.axvspan(args.zoom[0], args.zoom[1], color='#f2c14e', alpha=.12, lw=0)
    summary = (f"amplitude ratio {metrics['amplitude_ratio']:.3f}   ·   "
               f"phase delay {metrics['phase_delay_ms']:.0f} ms   ·   "
               f"tracking RMSE {metrics['tracking_rmse_deg']:.2f}°   ·   "
               f"mean reversal delay {metrics['reversal_delay_mean_ms']:.0f} ms")
    top.text(.5, 1.005, summary, transform=top.transAxes, ha='center', va='bottom',
             fontsize=8.5, color='0.35')

    zoom_low, zoom_high = args.zoom
    mask = (t >= zoom_low) & (t <= zoom_high)
    bottom.set_xlim(zoom_low, zoom_high)
    bottom.set_ylim(low, high)
    bottom.set_xlabel('Time (s)')
    for turn, response in events:
        if not (zoom_low <= turn <= zoom_high):
            continue
        bottom.axvline(turn, color='0.5', lw=.8, ls=':')
        if response is not None:
            arrow = high - 0.55 * pad
            bottom.annotate('', xy=(response, arrow), xytext=(turn, arrow),
                            arrowprops=dict(arrowstyle='<->', color='#c0392b', lw=1))
            bottom.text((turn + response) / 2, arrow + 0.25 * pad,
                        f'{(response - turn) * 1000:.0f} ms', ha='center', fontsize=8, color='#c0392b')
    bottom.legend(loc='lower right', ncol=2, frameon=False, fontsize=8)
    bottom.set_title('reversal response (per-turn delay)', fontsize=10)
    figure.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.out, dpi=130)
    print(args.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
