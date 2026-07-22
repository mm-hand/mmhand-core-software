#!/usr/bin/env python3
"""
Plot servo load data exported by the current Qt upper-computer Save Data flow.

Examples:
  python desktop/visualize/plot_servo_load_run.py --motors 4,23
  python desktop/visualize/plot_servo_load_run.py --joint 4 --save
  python desktop/visualize/plot_servo_load_run.py --csv run_data/hand_run_xxx.csv --motors M00,M01
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import Dict, Iterable, List, Optional, Sequence


DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(DESKTOP_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from desktop.core.protocol import MOTOR_COUNT  # noqa: E402
from desktop.core.motor_mapping import MOTOR_TO_JOINT_INDEX  # noqa: E402


def _default_run_data_dir() -> str:
    return os.path.join(PROJECT_ROOT, "run_data")


def _latest_csv_file(run_data_dir: str) -> str:
    if not os.path.isdir(run_data_dir):
        raise FileNotFoundError(f"run_data directory does not exist: {run_data_dir}")
    candidates = [
        os.path.join(run_data_dir, name)
        for name in os.listdir(run_data_dir)
        if name.lower().endswith(".csv") and os.path.isfile(os.path.join(run_data_dir, name))
    ]
    if not candidates:
        raise FileNotFoundError(f"No CSV file found in: {run_data_dir}")
    candidates.sort(key=lambda path: os.path.getmtime(path), reverse=True)
    return candidates[0]


def _safe_float(value) -> float:
    try:
        out = float(value)
        if math.isfinite(out):
            return out
    except Exception:
        pass
    return float("nan")


def _read_rows(csv_path: str) -> tuple[List[str], List[Dict[str, str]]]:
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def _parse_motor_token(token: str) -> int:
    token = str(token).strip().upper()
    if token.startswith("M"):
        token = token[1:]
    return int(token)


def _parse_motor_list(text: str) -> List[int]:
    motors: List[int] = []
    for piece in str(text or "").replace(";", ",").split(","):
        piece = piece.strip()
        if not piece:
            continue
        motor = _parse_motor_token(piece)
        if 0 <= motor < MOTOR_COUNT and motor not in motors:
            motors.append(motor)
    return motors


def _motors_for_joint(joint: Optional[int]) -> List[int]:
    if joint is None:
        return []
    return [
        motor
        for motor, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT])
        if int(mapped_joint) == int(joint)
    ]


def _resolve_motors(args) -> List[int]:
    motors = _parse_motor_list(args.motors)
    motors.extend(m for m in _motors_for_joint(args.joint) if m not in motors)
    if not motors:
        raise ValueError("Please provide --motors M05,M06 or --joint 4")
    return motors


def _time_series(rows: Sequence[Dict[str, str]]) -> List[float]:
    raw = [_safe_float(row.get("host_time", "")) for row in rows]
    finite = [value for value in raw if math.isfinite(value)]
    if not finite:
        return [float(i) for i in range(len(rows))]
    t0 = finite[0]
    out: List[float] = []
    last = 0.0
    for value in raw:
        if math.isfinite(value):
            last = value - t0
        out.append(last)
    return out


def _column_values(rows: Sequence[Dict[str, str]], key: str) -> List[float]:
    return [_safe_float(row.get(key, "")) for row in rows]


def _has_column(fieldnames: Iterable[str], key: str) -> bool:
    return key in set(fieldnames)


def _ensure_matplotlib():
    try:
        import matplotlib.pyplot as plt
        return plt
    except Exception as exc:
        raise RuntimeError("matplotlib is required for plotting. Install it in the desktop Python env.") from exc


def _plot(csv_path: str, fieldnames: List[str], rows: List[Dict[str, str]], motors: List[int], save_path: str) -> None:
    plt = _ensure_matplotlib()
    t = _time_series(rows)
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    if not isinstance(axes, (list, tuple)):
        axes = list(axes)

    for motor in motors:
        label = f"M{motor:02d}"
        load_mag_key = f"servo_load_mag_m{motor:02d}"
        load_key = load_mag_key if _has_column(fieldnames, load_mag_key) else f"servo_load_m{motor:02d}"
        abs_key = f"servo_angle_m{motor:02d}"
        raw_key = f"servo_raw_m{motor:02d}"
        target_key = f"motor_target_abs_m{motor:02d}"

        axes[0].plot(t, _column_values(rows, load_key), label=f"{label} {load_key}", linewidth=1.4)
        axes[1].plot(t, _column_values(rows, abs_key), label=f"{label} abs", linewidth=1.2)
        if _has_column(fieldnames, target_key):
            axes[1].plot(t, _column_values(rows, target_key), label=f"{label} target", linewidth=1.0, linestyle="--")
        axes[2].plot(t, _column_values(rows, raw_key), label=f"{label} raw", linewidth=1.2)

    title = os.path.basename(csv_path)
    if rows:
        scope = rows[0].get("active_scope_label", "")
        if scope:
            title = f"{title} | {scope}"
    axes[0].set_title(title)
    axes[0].set_ylabel("Load")
    axes[1].set_ylabel("Abs / target")
    axes[2].set_ylabel("Raw")
    axes[2].set_xlabel("Host time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved: {save_path}")
    else:
        plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot servo load curves from current run_data CSV.")
    parser.add_argument("--csv", default="", help="CSV path. Defaults to latest file under run_data.")
    parser.add_argument("--run-data-dir", default=_default_run_data_dir(), help="Directory used when --csv is omitted.")
    parser.add_argument("--motors", default="", help="Comma-separated motors, for example M05,M06 or 5,6.")
    parser.add_argument("--joint", type=int, default=None, help="Plot all motors mapped to this joint.")
    parser.add_argument("--save", nargs="?", const="AUTO", default="", help="Save PNG. Optional explicit output path.")
    args = parser.parse_args()

    try:
        csv_path = os.path.abspath(args.csv) if args.csv else _latest_csv_file(os.path.abspath(args.run_data_dir))
        fieldnames, rows = _read_rows(csv_path)
        motors = _resolve_motors(args)
        if args.save == "AUTO":
            motor_label = "_".join(f"M{m:02d}" for m in motors)
            save_path = os.path.splitext(csv_path)[0] + f"_{motor_label}_load.png"
        else:
            save_path = os.path.abspath(args.save) if args.save else ""
        print(f"Using CSV: {csv_path}")
        print(f"Motors: {', '.join(f'M{m:02d}' for m in motors)}")
        _plot(csv_path, fieldnames, rows, motors, save_path)
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
