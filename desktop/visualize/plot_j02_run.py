#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import List, Tuple

import matplotlib.pyplot as plt


def _default_run_data_dir() -> str:
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(project_root, "run_data")


def _latest_csv_file(run_data_dir: str) -> str:
    if not os.path.isdir(run_data_dir):
        raise FileNotFoundError(f"run_data directory not found: {run_data_dir}")

    candidates: List[str] = []
    for name in os.listdir(run_data_dir):
        if name.lower().endswith(".csv"):
            path = os.path.join(run_data_dir, name)
            if os.path.isfile(path):
                candidates.append(path)

    if not candidates:
        raise FileNotFoundError(f"No CSV file found: {run_data_dir}")

    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]


def _debug_csv_for(csv_path: str) -> str:
    if csv_path.lower().endswith("_joint_debug_packets.csv"):
        return csv_path
    base, _ext = os.path.splitext(os.path.abspath(csv_path))
    return f"{base}_joint_debug_packets.csv"


def _safe_float(value: object) -> float:
    try:
        out = float(value)
        if math.isfinite(out):
            return out
    except Exception:
        pass
    return float("nan")


def _load_debug_packet_series(csv_path: str, joint_index: int) -> Tuple[List[float], List[float], List[float], str]:
    device_ts: List[float] = []
    actual: List[float] = []
    target: List[float] = []
    with open(csv_path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(f"Empty CSV header: {csv_path}")
        required = {"device_ts_ms", "joint_index", "target_deg", "actual_deg"}
        missing = sorted(required.difference(reader.fieldnames))
        if missing:
            raise ValueError(f"Debug CSV missing columns: {missing}")
        for row in reader:
            try:
                if int(row.get("joint_index", -1)) != int(joint_index):
                    continue
            except Exception:
                continue
            ts = _safe_float(row.get("device_ts_ms", ""))
            tgt = _safe_float(row.get("target_deg", ""))
            act = _safe_float(row.get("actual_deg", ""))
            if ts <= 0 or not (math.isfinite(tgt) and math.isfinite(act)):
                continue
            if device_ts and ts <= device_ts[-1]:
                continue
            device_ts.append(ts)
            target.append(tgt)
            actual.append(act)

    if not device_ts:
        raise ValueError(f"No debug packet rows for J{joint_index:02d}: {csv_path}")
    t0 = device_ts[0]
    return [(ts - t0) / 1000.0 for ts in device_ts], actual, target, "Device Time (s)"


def _load_main_csv_debug_series(csv_path: str, joint_index: int) -> Tuple[List[float], List[float], List[float], str]:
    ts_key = f"joint_debug_timestamp_ms_j{joint_index:02d}"
    tgt_key = f"joint_debug_target_deg_j{joint_index:02d}"
    act_key = f"joint_debug_actual_deg_j{joint_index:02d}"
    device_ts: List[float] = []
    actual: List[float] = []
    target: List[float] = []
    with open(csv_path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(f"Empty CSV header: {csv_path}")
        required = {ts_key, tgt_key, act_key}
        missing = sorted(required.difference(reader.fieldnames))
        if missing:
            raise ValueError(f"Main CSV missing columns: {missing}")
        for row in reader:
            ts = _safe_float(row.get(ts_key, ""))
            tgt = _safe_float(row.get(tgt_key, ""))
            act = _safe_float(row.get(act_key, ""))
            if ts <= 0 or not (math.isfinite(tgt) and math.isfinite(act)):
                continue
            if device_ts and ts <= device_ts[-1]:
                continue
            device_ts.append(ts)
            target.append(tgt)
            actual.append(act)

    if not device_ts:
        raise ValueError(f"No main CSV debug rows for J{joint_index:02d}: {csv_path}")
    t0 = device_ts[0]
    return [(ts - t0) / 1000.0 for ts in device_ts], actual, target, "Device Time (s)"


def _load_series(csv_path: str, joint_index: int) -> Tuple[List[float], List[float], List[float], str, str]:
    debug_csv = _debug_csv_for(csv_path)
    if os.path.exists(debug_csv):
        t, actual, target, label = _load_debug_packet_series(debug_csv, joint_index)
        return t, actual, target, label, debug_csv
    t, actual, target, label = _load_main_csv_debug_series(csv_path, joint_index)
    return t, actual, target, label, csv_path


def _plot(csv_path: str, joint_index: int, save_path: str | None = None) -> None:
    t, actual, target, time_label, source_path = _load_series(csv_path, joint_index)

    plt.figure(figsize=(10, 4.8))
    plt.plot(t, target, label=f"J{joint_index:02d} Target from firmware debug", linewidth=1.8)
    plt.plot(t, actual, label=f"J{joint_index:02d} Actual from firmware debug", linewidth=1.8)
    plt.xlabel(time_label)
    plt.ylabel("Angle (deg)")
    plt.title(f"J{joint_index:02d} Debug Packet Curve: {os.path.basename(source_path)}")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved figure: {save_path}")

    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot joint target/actual angle from firmware debug packets")
    parser.add_argument("--csv", type=str, default=None, help="Runtime CSV or *_joint_debug_packets.csv path")
    parser.add_argument("--joint", type=int, default=2, help="Joint index, default J02")
    parser.add_argument("--run-data-dir", type=str, default=_default_run_data_dir())
    parser.add_argument("--save", type=str, default=None, help="Optional output PNG path")
    args = parser.parse_args()

    try:
        csv_path = os.path.abspath(args.csv) if args.csv else _latest_csv_file(os.path.abspath(args.run_data_dir))
        print(f"Using CSV: {csv_path}")
        _plot(csv_path, int(args.joint), save_path=args.save)
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
