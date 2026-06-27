#!/usr/bin/env python3
"""
绘制运行数据中 J02 的目标角度与当前角度曲线。

默认行为：
- 自动读取工程根目录 run_data/ 下“最新修改”的 CSV 文件；
- 横坐标：时间（秒，基于 ts_wall 相对首帧）；
- 纵坐标：角度（度）；
- 同时显示 J02 current/target 两条曲线。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import List, Tuple

import matplotlib.pyplot as plt


NA_MODE_MARKER = "NA_MODE"
JIDX = 2


def _default_run_data_dir() -> str:
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(project_root, "run_data")


def _latest_csv_file(run_data_dir: str) -> str:
    if not os.path.isdir(run_data_dir):
        raise FileNotFoundError(f"run_data 目录不存在: {run_data_dir}")

    candidates: List[str] = []
    for name in os.listdir(run_data_dir):
        if name.lower().endswith(".csv"):
            path = os.path.join(run_data_dir, name)
            if os.path.isfile(path):
                candidates.append(path)

    if not candidates:
        raise FileNotFoundError(f"未找到 CSV 文件: {run_data_dir}")

    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates[0]


def _safe_float(value: str) -> float:
    try:
        v = float(value)
        if math.isfinite(v):
            return v
    except Exception:
        pass
    return float("nan")


def _load_j02_series(csv_path: str) -> Tuple[List[float], List[float], List[float], str]:
    host_ts: List[float] = []
    host_current: List[float] = []
    host_target: List[float] = []
    debug_ts: List[float] = []
    debug_current: List[float] = []
    debug_target: List[float] = []

    curr_key = f"joint_curr_deg_j{JIDX:02d}"
    tgt_key = f"joint_target_deg_j{JIDX:02d}"
    dbg_ts_key = f"joint_debug_ts_ms_j{JIDX:02d}"
    dbg_tgt_key = f"joint_debug_target_deg_j{JIDX:02d}"
    dbg_act_key = f"joint_debug_actual_deg_j{JIDX:02d}"

    with open(csv_path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"CSV 表头为空: {csv_path}")
        required = {"ts_wall", curr_key, tgt_key}
        missing = [k for k in required if k not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV 缺少必要列: {missing}")

        for row in reader:
            t = _safe_float(str(row.get("ts_wall", "")))
            if not math.isfinite(t):
                continue

            c = _safe_float(str(row.get(curr_key, "")))
            raw_tgt = str(row.get(tgt_key, "")).strip()
            if raw_tgt == "" or raw_tgt == NA_MODE_MARKER:
                g = float("nan")
            else:
                g = _safe_float(raw_tgt)

            host_ts.append(t)
            host_current.append(c)
            host_target.append(g)

            dbg_t = _safe_float(str(row.get(dbg_ts_key, "")))
            dbg_c = _safe_float(str(row.get(dbg_act_key, "")))
            dbg_g = _safe_float(str(row.get(dbg_tgt_key, "")))
            if math.isfinite(dbg_t) and math.isfinite(dbg_c) and math.isfinite(dbg_g):
                if dbg_t > 0 and (not debug_ts or dbg_t > debug_ts[-1]):
                    debug_ts.append(dbg_t)
                    debug_current.append(dbg_c)
                    debug_target.append(dbg_g)

    if debug_ts:
        t0 = debug_ts[0]
        rel_t = [(t - t0) / 1000.0 for t in debug_ts]
        return rel_t, debug_current, debug_target, "Device Time (s)"

    if not host_ts:
        raise ValueError(f"CSV 中没有可用数据行: {csv_path}")

    t0 = host_ts[0]
    rel_t = [t - t0 for t in host_ts]
    return rel_t, host_current, host_target, "Host Time (s)"


def _plot_j02(csv_path: str, save_path: str | None = None) -> None:
    t, curr, tgt, time_label = _load_j02_series(csv_path)

    plt.figure(figsize=(10, 4.8))
    plt.plot(t, curr, label="J02 Current Angle", linewidth=1.8)
    plt.plot(t, tgt, label="J02 Target Angle", linewidth=1.8, linestyle="--")
    plt.xlabel(time_label)
    plt.ylabel("Angle (deg)")
    plt.title(f"J02 Runtime Curve: {os.path.basename(csv_path)}")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Saved figure: {save_path}")

    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot J02 current/target angle from runtime CSV")
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="指定 CSV 文件路径；不指定时自动读取 run_data 下最新文件",
    )
    parser.add_argument(
        "--run-data-dir",
        type=str,
        default=_default_run_data_dir(),
        help="run_data 目录（仅在未指定 --csv 时生效）",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="可选：保存图像路径（如 output.png）",
    )
    args = parser.parse_args()

    try:
        csv_path = os.path.abspath(args.csv) if args.csv else _latest_csv_file(os.path.abspath(args.run_data_dir))
        print(f"Using CSV: {csv_path}")
        _plot_j02(csv_path, save_path=args.save)
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
