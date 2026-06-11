#!/usr/bin/env python3
"""Plot GUI Feedforward Test step response from run_data CSV logs.

The upper-computer UI sends the test commands and records hand_run_*.csv.
This script only reads that CSV and draws target vs actual curves.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt


ENCODER_COUNT = 21
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def default_run_data_dir() -> str:
    return os.path.join(PROJECT_ROOT, "run_data")


def latest_run_csv(run_data_dir: str) -> str:
    if not os.path.isdir(run_data_dir):
        raise FileNotFoundError(f"run_data directory does not exist: {run_data_dir}")

    candidates: List[str] = []
    for name in os.listdir(run_data_dir):
        lower = name.lower()
        if not lower.endswith(".csv"):
            continue
        if lower.endswith("_joint_debug_packets.csv"):
            continue
        if lower.endswith("_load_slim.csv"):
            continue
        if not lower.startswith("hand_run_"):
            continue
        path = os.path.join(run_data_dir, name)
        if os.path.isfile(path):
            candidates.append(path)

    if not candidates:
        raise FileNotFoundError(f"No hand_run_*.csv found in: {run_data_dir}")
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def safe_float(value: object) -> float:
    try:
        out = float(str(value).strip())
        if math.isfinite(out):
            return out
    except Exception:
        pass
    return float("nan")


def read_rows(csv_path: str) -> Tuple[List[str], List[Dict[str, str]]]:
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    if not fieldnames:
        raise ValueError(f"CSV header is empty: {csv_path}")
    if not rows:
        raise ValueError(f"CSV has no data rows: {csv_path}")
    return fieldnames, rows


def parse_joint(text: str) -> int:
    raw = str(text).strip().upper()
    if raw.startswith("J"):
        raw = raw[1:]
    joint = int(raw)
    if not 0 <= joint < ENCODER_COUNT:
        raise argparse.ArgumentTypeError(f"joint must be J00..J{ENCODER_COUNT - 1:02d}")
    return joint


def parse_joints(text: str) -> List[int]:
    raw = str(text).strip().lower()
    if raw in ("all", "*"):
        return list(range(ENCODER_COUNT))
    joints: List[int] = []
    for part in raw.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        joints.append(parse_joint(part))
    if not joints:
        raise argparse.ArgumentTypeError("need at least one joint, for example J11 or all")
    return sorted(set(joints))


def host_time_s(rows: Sequence[Dict[str, str]]) -> List[float]:
    times = [safe_float(row.get("host_time", "")) for row in rows]
    finite = [t for t in times if math.isfinite(t)]
    if finite:
        t0 = finite[0]
        return [(t - t0) if math.isfinite(t) else float("nan") for t in times]
    return [i for i, _ in enumerate(rows)]


def finite_count(values: Iterable[float]) -> int:
    return sum(1 for v in values if math.isfinite(v))


def choose_actual_series(
    rows: Sequence[Dict[str, str]],
    joint: int,
    source: str,
) -> Tuple[List[float], str]:
    encoder_key = f"joint_deg_j{joint:02d}"
    debug_valid_key = f"joint_debug_valid_j{joint:02d}"
    debug_actual_key = f"joint_debug_actual_deg_j{joint:02d}"

    encoder = [safe_float(row.get(encoder_key, "")) for row in rows]
    debug: List[float] = []
    for row in rows:
        valid = str(row.get(debug_valid_key, "")).strip() == "1"
        value = safe_float(row.get(debug_actual_key, ""))
        debug.append(value if valid else float("nan"))

    if source == "encoder":
        return encoder, "encoder actual"
    if source == "debug":
        return debug, "debug actual"
    if finite_count(debug) >= 3:
        return debug, "debug actual"
    return encoder, "encoder actual"


def target_series(rows: Sequence[Dict[str, str]], joint: int) -> List[float]:
    target_key = f"joint_target_deg_j{joint:02d}"
    return [safe_float(row.get(target_key, "")) for row in rows]


def save_path_for(csv_path: str, joints: Sequence[int], out: str) -> str:
    if out:
        return os.path.abspath(out)
    base, _ext = os.path.splitext(os.path.abspath(csv_path))
    if len(joints) == 1:
        return f"{base}_J{joints[0]:02d}_step.png"
    return f"{base}_J{joints[0]:02d}-J{joints[-1]:02d}_step.png"


def plot_step_response(
    csv_path: str,
    joints: Sequence[int],
    source: str,
    out: str,
    show: bool,
) -> str:
    _fieldnames, rows = read_rows(csv_path)
    t = host_time_s(rows)
    save_path = save_path_for(csv_path, joints, out)

    if len(joints) == 1:
        joint = joints[0]
        target = target_series(rows, joint)
        actual, actual_label = choose_actual_series(rows, joint, source)
        if finite_count(target) == 0:
            raise ValueError(f"CSV has no target data for J{joint:02d}")
        if finite_count(actual) == 0:
            raise ValueError(f"CSV has no actual data for J{joint:02d}")

        plt.figure(figsize=(11, 5.5))
        plt.plot(t, target, label=f"J{joint:02d} target", linewidth=1.8, linestyle="--")
        plt.plot(t, actual, label=f"J{joint:02d} {actual_label}", linewidth=1.6)
        plt.xlabel("Host Time (s)")
        plt.ylabel("Angle (deg)")
        plt.title(f"J{joint:02d} Step Response - {os.path.basename(csv_path)}")
        plt.grid(True, alpha=0.3)
        plt.legend(loc="best")
        plt.tight_layout()
    else:
        cols = 3
        rows_n = math.ceil(len(joints) / cols)
        fig, axes = plt.subplots(rows_n, cols, figsize=(15, max(4.0, rows_n * 3.0)), sharex=True)
        flat_axes = list(axes.flat) if hasattr(axes, "flat") else [axes]
        for ax, joint in zip(flat_axes, joints):
            target = target_series(rows, joint)
            actual, actual_label = choose_actual_series(rows, joint, source)
            ax.plot(t, target, label="target", linewidth=1.2, linestyle="--")
            ax.plot(t, actual, label=actual_label, linewidth=1.1)
            ax.set_title(f"J{joint:02d}")
            ax.grid(True, alpha=0.25)
            ax.set_ylabel("deg")
        for ax in flat_axes[len(joints):]:
            ax.axis("off")
        flat_axes[0].legend(loc="best")
        for ax in flat_axes[-cols:]:
            ax.set_xlabel("Host Time (s)")
        fig.suptitle(f"Step Response - {os.path.basename(csv_path)}")
        fig.tight_layout()

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.savefig(save_path, dpi=150)
    print(f"Using CSV: {csv_path}")
    print(f"Saved figure: {save_path}")
    if show:
        plt.show()
    else:
        plt.close("all")
    return save_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot Feedforward Test step response from UI run_data CSV.")
    parser.add_argument("--csv", default="", help="CSV path. Defaults to latest run_data/hand_run_*.csv.")
    parser.add_argument("--run-data-dir", default=default_run_data_dir(), help="Directory used when --csv is omitted.")
    parser.add_argument("--joint", default="J11", help="Joint to plot, for example J11. Use all for all 21 joints.")
    parser.add_argument("--source", choices=["auto", "encoder", "debug"], default="auto", help="Actual-angle source.")
    parser.add_argument("--out", default="", help="Output PNG path. Defaults next to the CSV.")
    parser.add_argument("--show", action="store_true", help="Also open a matplotlib window.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        csv_path = os.path.abspath(args.csv) if args.csv else latest_run_csv(os.path.abspath(args.run_data_dir))
        joints = parse_joints(args.joint)
        plot_step_response(csv_path, joints, args.source, args.out, args.show)
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
