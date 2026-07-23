#!/usr/bin/env python3
"""Analyze one single-motor joint PID test recorded by the desktop UI.

The script reads the multi-CSV session produced automatically by the Test tab.
It prefers the high-rate joint-debug packet stream and aligns the lower-rate
servo telemetry on the same time axis.  No command is sent to the hand.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np


ENCODER_COUNT = 21
MOTOR_COUNT = 27
PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class JointTrace:
    time_s: np.ndarray
    target_deg: np.ndarray
    actual_deg: np.ndarray
    loop1_output: np.ndarray
    source: str


@dataclass
class ServoTrace:
    time_s: np.ndarray
    target_abs: np.ndarray
    actual_abs: np.ndarray
    load: np.ndarray
    speed: np.ndarray
    motor_index: int


def safe_float(value: object) -> float:
    try:
        result = float(str(value).strip())
        return result if math.isfinite(result) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def safe_int(value: object, default: int = -1) -> int:
    try:
        return int(str(value).strip(), 0)
    except (TypeError, ValueError):
        return default


def parse_joint(value: str) -> int:
    text = str(value).strip().upper()
    if text.startswith("J"):
        text = text[1:]
    try:
        joint = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("joint must be J00..J20") from exc
    if not 0 <= joint < ENCODER_COUNT:
        raise argparse.ArgumentTypeError("joint must be J00..J20")
    return joint


def parse_motor(value: str) -> int:
    text = str(value).strip().upper()
    if text.startswith("M"):
        text = text[1:]
    try:
        motor = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("motor must be M00..M26") from exc
    if not 0 <= motor < MOTOR_COUNT:
        raise argparse.ArgumentTypeError("motor must be M00..M26")
    return motor


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def find_topic_csv(session_dir: Path, suffix: str) -> Optional[Path]:
    candidates = [
        path
        for path in session_dir.glob("*.csv")
        if path.name.lower().endswith(suffix.lower())
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_size)


def session_has_pid_data(path: Path) -> bool:
    if not path.is_dir():
        return False
    return any(path.glob("*joint_debug*.csv")) or any(path.glob("*_joint_state.csv"))


def session_has_joint_data(path: Path, joint: int) -> bool:
    for debug_path in path.glob("*joint_debug*.csv"):
        for row in read_csv_rows(debug_path):
            if safe_int(row.get("joint_index")) == joint and safe_int(row.get("valid"), 0) == 1:
                return True
    joint_path = find_topic_csv(path, "_joint_state.csv")
    if joint_path is not None:
        for row in read_csv_rows(joint_path):
            if (
                safe_int(row.get("joint_index")) == joint
                and math.isfinite(safe_float(row.get("joint_deg")))
                and math.isfinite(safe_float(row.get("joint_target_deg")))
            ):
                return True
    return False


def latest_session(run_data_dir: Path, joint: int) -> Path:
    if not run_data_dir.is_dir():
        raise FileNotFoundError(f"run_data directory does not exist: {run_data_dir}")
    candidates = [
        path
        for path in run_data_dir.iterdir()
        if session_has_pid_data(path) and session_has_joint_data(path, joint)
    ]
    if not candidates:
        raise FileNotFoundError(f"no recorded session containing J{joint:02d} found in: {run_data_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def resolve_session(value: str, run_data_dir: Path, joint: int) -> Path:
    if not value:
        return latest_session(run_data_dir, joint)
    path = Path(value).expanduser().resolve()
    if path.is_file():
        path = path.parent
    if not session_has_pid_data(path):
        raise FileNotFoundError(f"session contains no joint PID data: {path}")
    return path


def row_time(row: Dict[str, str]) -> float:
    host_time = str(row.get("host_time", "") or "").strip()
    if host_time:
        try:
            return datetime.fromisoformat(host_time).timestamp()
        except Exception:
            pass
    value = safe_float(row.get("host_ts_wall", ""))
    if math.isfinite(value):
        return value
    return float("nan")


def finite_arrays(*arrays: np.ndarray) -> Tuple[np.ndarray, ...]:
    if not arrays:
        return tuple()
    mask = np.ones(len(arrays[0]), dtype=bool)
    for array in arrays:
        mask &= np.isfinite(array)
    return tuple(array[mask] for array in arrays)


def deduplicate_time(columns: Sequence[np.ndarray]) -> Tuple[np.ndarray, ...]:
    if not columns or len(columns[0]) == 0:
        return tuple(columns)
    order = np.argsort(columns[0], kind="stable")
    sorted_columns = [column[order] for column in columns]
    _, unique_indices = np.unique(sorted_columns[0], return_index=True)
    unique_indices.sort()
    return tuple(column[unique_indices] for column in sorted_columns)


def load_debug_trace(session_dir: Path, joint: int) -> Optional[JointTrace]:
    candidates = sorted(
        session_dir.glob("*joint_debug*.csv"),
        key=lambda path: path.stat().st_size,
        reverse=True,
    )
    best: Optional[JointTrace] = None
    for path in candidates:
        rows = read_csv_rows(path)
        selected = [
            row
            for row in rows
            if safe_int(row.get("joint_index")) == joint
            and safe_int(row.get("valid"), 0) == 1
        ]
        if len(selected) < 3:
            continue
        time_values = np.asarray([row_time(row) for row in selected], dtype=float)
        target = np.asarray([safe_float(row.get("target_deg")) for row in selected], dtype=float)
        actual = np.asarray([safe_float(row.get("actual_deg")) for row in selected], dtype=float)
        loop1 = np.asarray([safe_float(row.get("loop1_output")) for row in selected], dtype=float)
        valid = np.isfinite(time_values) & np.isfinite(target) & np.isfinite(actual)
        if np.count_nonzero(valid) < 3:
            continue
        columns = deduplicate_time(
            (time_values[valid], target[valid], actual[valid], loop1[valid])
        )
        trace = JointTrace(*columns, source=path.name)
        if best is None or len(trace.time_s) > len(best.time_s):
            best = trace
    return best


def load_joint_state_trace(session_dir: Path, joint: int) -> Optional[JointTrace]:
    path = find_topic_csv(session_dir, "_joint_state.csv")
    if path is None:
        return None
    selected = [
        row for row in read_csv_rows(path) if safe_int(row.get("joint_index")) == joint
    ]
    if len(selected) < 3:
        return None
    time_values = np.asarray([row_time(row) for row in selected], dtype=float)
    target = np.asarray([safe_float(row.get("joint_target_deg")) for row in selected], dtype=float)
    actual = np.asarray([safe_float(row.get("joint_deg")) for row in selected], dtype=float)
    valid = np.isfinite(time_values) & np.isfinite(target) & np.isfinite(actual)
    if np.count_nonzero(valid) < 3:
        return None
    time_values, target, actual = deduplicate_time((time_values[valid], target[valid], actual[valid]))
    empty = np.full(len(time_values), np.nan)
    return JointTrace(time_values, target, actual, empty, path.name)


def load_joint_trace(session_dir: Path, joint: int) -> JointTrace:
    trace = load_debug_trace(session_dir, joint)
    if trace is None:
        trace = load_joint_state_trace(session_dir, joint)
    if trace is None:
        raise ValueError(f"session has no valid target/actual samples for J{joint:02d}")
    return trace


def motor_candidates(session_dir: Path, joint: int) -> List[int]:
    path = find_topic_csv(session_dir, "_servo_state.csv")
    if path is None:
        return []
    candidates = {
        safe_int(row.get("motor_index"))
        for row in read_csv_rows(path)
        if safe_int(row.get("joint_index")) == joint
        and safe_int(row.get("motor_index")) >= 0
        and (
            safe_int(row.get("online"), 0) == 1
            or safe_int(row.get("is_active_motor"), 0) == 1
        )
    }
    return sorted(candidates)


def resolve_motor(session_dir: Path, joint: int, requested: Optional[int]) -> int:
    if requested is not None:
        return requested
    candidates = motor_candidates(session_dir, joint)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise ValueError(
            f"cannot infer the motor for J{joint:02d}; pass --motor Mxx explicitly"
        )
    labels = ", ".join(f"M{motor:02d}" for motor in candidates)
    raise ValueError(
        f"J{joint:02d} has multiple motors ({labels}); this script is for a single-motor joint"
    )


def load_servo_trace(session_dir: Path, motor: int) -> Optional[ServoTrace]:
    path = find_topic_csv(session_dir, "_servo_state.csv")
    if path is None:
        return None
    selected = [
        row for row in read_csv_rows(path) if safe_int(row.get("motor_index")) == motor
    ]
    if len(selected) < 2:
        return None
    time_values = np.asarray([row_time(row) for row in selected], dtype=float)
    target = np.asarray([safe_float(row.get("motor_target_abs")) for row in selected], dtype=float)
    actual = np.asarray([safe_float(row.get("servo_abs")) for row in selected], dtype=float)
    load = np.asarray([safe_float(row.get("load")) for row in selected], dtype=float)
    speed = np.asarray([safe_float(row.get("speed")) for row in selected], dtype=float)
    valid = np.isfinite(time_values)
    if np.count_nonzero(valid) < 2:
        return None
    columns = deduplicate_time((time_values[valid], target[valid], actual[valid], load[valid], speed[valid]))
    return ServoTrace(*columns, motor_index=motor)


def trim_trace(trace: JointTrace, start_s: Optional[float], end_s: Optional[float]) -> JointTrace:
    origin = trace.time_s[0]
    relative = trace.time_s - origin
    mask = np.ones(len(relative), dtype=bool)
    if start_s is not None:
        mask &= relative >= start_s
    if end_s is not None:
        mask &= relative <= end_s
    if np.count_nonzero(mask) < 3:
        raise ValueError("selected time window contains fewer than three joint samples")
    return JointTrace(
        trace.time_s[mask],
        trace.target_deg[mask],
        trace.actual_deg[mask],
        trace.loop1_output[mask],
        trace.source,
    )


def median_sample_period(time_s: np.ndarray) -> float:
    diffs = np.diff(time_s)
    diffs = diffs[np.isfinite(diffs) & (diffs > 0.0)]
    return float(np.median(diffs)) if len(diffs) else float("nan")


def classify_target(target: np.ndarray) -> str:
    span = float(np.ptp(target))
    if span < 0.05:
        return "hold"
    rounded_unique = len(np.unique(np.round(target, 3)))
    changes = np.abs(np.diff(target)) > max(0.05, 0.02 * span)
    change_ratio = float(np.mean(changes)) if len(changes) else 0.0
    if rounded_unique <= 20 and change_ratio < 0.2:
        return "step"
    return "wave"


def first_crossing(time_s: np.ndarray, values: np.ndarray, threshold: float) -> float:
    indices = np.flatnonzero(values >= threshold)
    return float(time_s[indices[0]]) if len(indices) else float("nan")


def settling_time(
    time_s: np.ndarray,
    error: np.ndarray,
    tolerance_deg: float,
    hold_s: float,
) -> float:
    if len(time_s) < 2:
        return float("nan")
    for start in range(len(time_s)):
        end_time = time_s[start] + hold_s
        end = int(np.searchsorted(time_s, end_time, side="left"))
        if end >= len(time_s):
            break
        if np.all(np.abs(error[start : end + 1]) <= tolerance_deg):
            return float(time_s[start])
    return float("nan")


def detect_step_indices(target: np.ndarray) -> List[int]:
    if len(target) < 2:
        return []
    span = float(np.ptp(target))
    threshold = max(0.05, 0.02 * span)
    raw = list(np.flatnonzero(np.abs(np.diff(target)) > threshold) + 1)
    compact: List[int] = []
    for index in raw:
        if not compact or index - compact[-1] > 1:
            compact.append(int(index))
        else:
            compact[-1] = int(index)
    return compact


def analyze_steps(
    trace: JointTrace,
    tolerance_deg: float,
    settle_hold_s: float,
) -> List[Dict[str, object]]:
    indices = detect_step_indices(trace.target_deg)
    rows: List[Dict[str, object]] = []
    for step_number, start in enumerate(indices, start=1):
        end = indices[step_number] if step_number < len(indices) else len(trace.time_s)
        if end - start < 3:
            continue
        segment_time = trace.time_s[start:end] - trace.time_s[start]
        segment_actual = trace.actual_deg[start:end]
        new_target = float(np.median(trace.target_deg[start:end]))
        old_target = float(np.median(trace.target_deg[max(0, start - 3) : start]))
        step_size = new_target - old_target
        if abs(step_size) < 0.05:
            continue
        before_start = max(0, start - max(3, min(20, start)))
        initial_actual = float(np.median(trace.actual_deg[before_start:start]))
        direction = 1.0 if step_size > 0.0 else -1.0
        requested_travel = abs(new_target - initial_actual)
        progress = direction * (segment_actual - initial_actual)
        rise_10 = first_crossing(segment_time, progress, 0.1 * requested_travel)
        rise_90 = first_crossing(segment_time, progress, 0.9 * requested_travel)
        rise_time = rise_90 - rise_10 if math.isfinite(rise_10) and math.isfinite(rise_90) else float("nan")
        directed_overshoot = float(np.max(direction * (segment_actual - new_target)))
        overshoot = max(0.0, directed_overshoot)
        overshoot_pct = 100.0 * overshoot / abs(step_size)
        error = new_target - segment_actual
        settled = settling_time(segment_time, error, tolerance_deg, settle_hold_s)
        tail_count = max(3, int(round(0.25 * len(segment_actual))))
        tail_error = error[-tail_count:]
        rows.append(
            {
                "step": step_number,
                "time_s": float(trace.time_s[start] - trace.time_s[0]),
                "from_deg": old_target,
                "to_deg": new_target,
                "step_deg": step_size,
                "rise_time_10_90_s": rise_time,
                "overshoot_deg": overshoot,
                "overshoot_pct": overshoot_pct,
                "settling_time_s": settled,
                "steady_error_mean_deg": float(np.mean(tail_error)),
                "steady_error_std_deg": float(np.std(tail_error)),
                "steady_error_max_abs_deg": float(np.max(np.abs(tail_error))),
            }
        )
    return rows


def estimate_lag(time_s: np.ndarray, target: np.ndarray, actual: np.ndarray) -> float:
    if len(time_s) < 5:
        return float("nan")
    dt = median_sample_period(time_s)
    if not math.isfinite(dt) or dt <= 0.0:
        return float("nan")
    target_centered = target - np.mean(target)
    actual_centered = actual - np.mean(actual)
    if np.std(target_centered) < 1e-9 or np.std(actual_centered) < 1e-9:
        return float("nan")
    correlation = np.correlate(actual_centered, target_centered, mode="full")
    lags = np.arange(-len(target_centered) + 1, len(target_centered))
    max_lag_samples = max(1, int(round(min(2.0, 0.25 * (time_s[-1] - time_s[0])) / dt)))
    allowed = np.abs(lags) <= max_lag_samples
    lag = int(lags[allowed][np.argmax(correlation[allowed])])
    return float(lag * dt)


def common_metrics(
    trace: JointTrace,
    servo: Optional[ServoTrace],
    output_limit: float,
    tolerance_deg: float,
) -> Dict[str, object]:
    error = trace.target_deg - trace.actual_deg
    dt = median_sample_period(trace.time_s)
    metrics: Dict[str, object] = {
        "duration_s": float(trace.time_s[-1] - trace.time_s[0]),
        "joint_samples": int(len(trace.time_s)),
        "sample_period_ms": 1000.0 * dt,
        "sample_rate_hz": 1.0 / dt if math.isfinite(dt) and dt > 0 else float("nan"),
        "target_min_deg": float(np.min(trace.target_deg)),
        "target_max_deg": float(np.max(trace.target_deg)),
        "rmse_deg": float(np.sqrt(np.mean(error * error))),
        "mae_deg": float(np.mean(np.abs(error))),
        "max_abs_error_deg": float(np.max(np.abs(error))),
        "p95_abs_error_deg": float(np.percentile(np.abs(error), 95)),
        "mean_error_deg": float(np.mean(error)),
        "within_tolerance_ratio": float(np.mean(np.abs(error) <= tolerance_deg)),
        "target_type": classify_target(trace.target_deg),
        "source": trace.source,
    }
    finite_loop1 = trace.loop1_output[np.isfinite(trace.loop1_output)]
    if len(finite_loop1):
        metrics["pid_output_peak_counts"] = float(np.max(np.abs(finite_loop1)))
        metrics["pid_output_rms_counts"] = float(np.sqrt(np.mean(finite_loop1 * finite_loop1)))
        metrics["pid_output_saturation_ratio"] = float(
            np.mean(np.abs(finite_loop1) >= 0.98 * output_limit)
        )
    if metrics["target_type"] == "wave":
        target_std = float(np.std(trace.target_deg))
        actual_std = float(np.std(trace.actual_deg))
        metrics["wave_amplitude_ratio"] = actual_std / target_std if target_std > 1e-9 else float("nan")
        metrics["wave_lag_s"] = estimate_lag(trace.time_s, trace.target_deg, trace.actual_deg)
    if servo is not None:
        mask = (servo.time_s >= trace.time_s[0]) & (servo.time_s <= trace.time_s[-1])
        if np.count_nonzero(mask):
            loads = servo.load[mask]
            speeds = servo.speed[mask]
            loads = loads[np.isfinite(loads)]
            speeds = speeds[np.isfinite(speeds)]
            metrics["servo_samples"] = int(np.count_nonzero(mask))
            if len(loads):
                metrics["load_peak_abs"] = float(np.max(np.abs(loads)))
                metrics["load_mean_abs"] = float(np.mean(np.abs(loads)))
            if len(speeds):
                metrics["speed_peak_abs"] = float(np.max(np.abs(speeds)))
    return metrics


def metric_unit(name: str) -> str:
    if name.endswith("_ms"):
        return "ms"
    if name.endswith("_hz"):
        return "Hz"
    if name.endswith("_s"):
        return "s"
    if name.endswith("_deg"):
        return "deg"
    if name.endswith("_counts"):
        return "counts"
    if name.endswith("_ratio"):
        return "ratio"
    return ""


def display_value(value: object) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return "-" if not math.isfinite(value) else f"{value:.6g}"
    return str(value)


def write_summary_csv(path: Path, metrics: Dict[str, object]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value", "unit"])
        for name, value in metrics.items():
            writer.writerow([name, display_value(value), metric_unit(name)])


def write_step_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    fields = [
        "step",
        "time_s",
        "from_deg",
        "to_deg",
        "step_deg",
        "rise_time_10_90_s",
        "overshoot_deg",
        "overshoot_pct",
        "settling_time_s",
        "steady_error_mean_deg",
        "steady_error_std_deg",
        "steady_error_max_abs_deg",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: display_value(row.get(key, "")) for key in fields})


def tuning_notes(metrics: Dict[str, object], steps: Sequence[Dict[str, object]]) -> List[str]:
    notes: List[str] = []
    sample_rate = safe_float(metrics.get("sample_rate_hz"))
    if math.isfinite(sample_rate) and sample_rate < 20.0:
        notes.append("采样率低于 20 Hz，超调和上升时间只能作趋势判断；应优先检查 joint debug 数据是否有效。")
    saturation = safe_float(metrics.get("pid_output_saturation_ratio"))
    if math.isfinite(saturation) and saturation > 0.05:
        notes.append("PID 输出有较明显限幅，先减小测试阶跃或检查 Kp，限幅期间不能据此精确判断线性参数。")
    overshoots = [safe_float(row.get("overshoot_pct")) for row in steps]
    overshoots = [value for value in overshoots if math.isfinite(value)]
    if overshoots and max(overshoots) > 10.0:
        notes.append("阶跃超调超过 10%，Kp 可能偏大；也应同时检查机械回弹、绳张力和目标是否触及动态边界。")
    steady_std = [safe_float(row.get("steady_error_std_deg")) for row in steps]
    steady_std = [value for value in steady_std if math.isfinite(value)]
    if steady_std and max(steady_std) > 0.3:
        notes.append("稳态误差波动超过 0.3 deg，存在振荡或反馈噪声；在加 Ki 前先降低 Kp 或检查磁编滤波。")
    mean_error = abs(safe_float(metrics.get("mean_error_deg")))
    if math.isfinite(mean_error) and mean_error > 0.5:
        notes.append("平均稳态偏差超过 0.5 deg；先排除摩擦/死区和标定映射，再评估是否需要很小的 Ki。")
    if not notes:
        notes.append("未发现明显的超调、限幅或稳态振荡特征；仍需结合正反向多组阶跃确认。")
    notes.append("脚本只做数据诊断，不会自动修改固件 PID 参数。")
    return notes


def write_report(
    path: Path,
    session_dir: Path,
    joint: int,
    motor: int,
    metrics: Dict[str, object],
    steps: Sequence[Dict[str, object]],
    tolerance_deg: float,
) -> None:
    lines = [
        f"# J{joint:02d} / M{motor:02d} 单电机 PID 分析",
        "",
        f"- 数据目录：`{session_dir}`",
        f"- 角度数据源：`{metrics.get('source', '-')}`",
        f"- 测试类型：`{metrics.get('target_type', '-')}`",
        f"- 容差：`+-{tolerance_deg:g} deg`",
        "",
        "## 汇总",
        "",
        f"- RMSE：{display_value(metrics.get('rmse_deg'))} deg",
        f"- MAE：{display_value(metrics.get('mae_deg'))} deg",
        f"- 最大绝对误差：{display_value(metrics.get('max_abs_error_deg'))} deg",
        f"- 95% 绝对误差：{display_value(metrics.get('p95_abs_error_deg'))} deg",
        f"- 容差内比例：{display_value(100.0 * safe_float(metrics.get('within_tolerance_ratio')))}%",
        f"- PID 输出峰值：{display_value(metrics.get('pid_output_peak_counts'))} counts",
        f"- PID 输出限幅占比：{display_value(100.0 * safe_float(metrics.get('pid_output_saturation_ratio')))}%",
        f"- 最大绝对负载：{display_value(metrics.get('load_peak_abs'))}",
        "",
    ]
    if steps:
        lines.extend(
            [
                "## 阶跃结果",
                "",
                "| 序号 | 目标变化(deg) | 10-90%上升(s) | 超调(%) | 稳定时间(s) | 稳态最大误差(deg) |",
                "|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for row in steps:
            lines.append(
                "| {step} | {step_deg} | {rise} | {overshoot} | {settling} | {steady} |".format(
                    step=row["step"],
                    step_deg=display_value(row["step_deg"]),
                    rise=display_value(row["rise_time_10_90_s"]),
                    overshoot=display_value(row["overshoot_pct"]),
                    settling=display_value(row["settling_time_s"]),
                    steady=display_value(row["steady_error_max_abs_deg"]),
                )
            )
        lines.append("")
    lines.extend(["## 调参提示", ""])
    lines.extend(f"- {note}" for note in tuning_notes(metrics, steps))
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8-sig")


def plot_analysis(
    path: Path,
    trace: JointTrace,
    servo: Optional[ServoTrace],
    joint: int,
    motor: int,
    tolerance_deg: float,
    show: bool,
) -> None:
    t0 = trace.time_s[0]
    t = trace.time_s - t0
    error = trace.target_deg - trace.actual_deg
    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)

    axes[0].plot(t, trace.target_deg, label="target", color="#2563eb", linewidth=1.6)
    axes[0].plot(t, trace.actual_deg, label="actual", color="#16a34a", linewidth=1.3)
    axes[0].set_ylabel("angle (deg)")
    axes[0].legend(loc="best")

    axes[1].plot(t, error, label="target - actual", color="#dc2626", linewidth=1.2)
    axes[1].axhspan(-tolerance_deg, tolerance_deg, color="#22c55e", alpha=0.12, label=f"+-{tolerance_deg:g} deg")
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("error (deg)")
    axes[1].legend(loc="best")

    if np.any(np.isfinite(trace.loop1_output)):
        axes[2].plot(t, trace.loop1_output, label="outer PID output", color="#7c3aed", linewidth=1.1)
    axes[2].axhline(0.0, color="black", linewidth=0.7)
    axes[2].set_ylabel("PID output\n(counts)")
    pid_handles, pid_labels = axes[2].get_legend_handles_labels()
    if pid_handles:
        axes[2].legend(pid_handles, pid_labels, loc="best")

    if servo is not None:
        servo_t = servo.time_s - t0
        axes[3].scatter(servo_t, servo.actual_abs, label="telemetry ABS", color="#0f766e", s=10, alpha=0.55)
        axes[3].scatter(servo_t, servo.target_abs, label="telemetry target", color="#d97706", s=10, alpha=0.45)
    axes[3].set_ylabel("motor ABS\n(counts)")
    motor_handles, motor_labels = axes[3].get_legend_handles_labels()
    if motor_handles:
        axes[3].legend(motor_handles, motor_labels, loc="best")

    if servo is not None:
        servo_t = servo.time_s - t0
        axes[4].plot(servo_t, servo.load, label="load", color="#ef4444", linewidth=1.1)
        speed_axis = axes[4].twinx()
        speed_axis.plot(servo_t, servo.speed, label="speed", color="#0ea5e9", linewidth=1.0, alpha=0.8)
        speed_axis.set_ylabel("speed")
        lines = axes[4].get_lines() + speed_axis.get_lines()
        axes[4].legend(lines, [line.get_label() for line in lines], loc="best")
    axes[4].set_ylabel("load")
    axes[4].set_xlabel("time from analysis start (s)")

    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.suptitle(f"Single-motor PID analysis: J{joint:02d} / M{motor:02d}")
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(path, dpi=170)
    if show:
        plt.show()
    else:
        plt.close(fig)


def output_directory(session_dir: Path, joint: int, requested: str) -> Path:
    if requested:
        path = Path(requested).expanduser().resolve()
    else:
        path = session_dir / f"PID分析_J{joint:02d}"
    path.mkdir(parents=True, exist_ok=True)
    return path


def run_analysis(args: argparse.Namespace) -> Dict[str, Path]:
    run_data_dir = Path(args.run_data_dir).expanduser().resolve()
    session_dir = resolve_session(args.session, run_data_dir, args.joint)
    trace = trim_trace(load_joint_trace(session_dir, args.joint), args.start, args.end)
    motor = resolve_motor(session_dir, args.joint, args.motor)
    servo = load_servo_trace(session_dir, motor)
    metrics = common_metrics(trace, servo, args.output_limit, args.tolerance)
    steps = analyze_steps(trace, args.tolerance, args.settle_hold) if metrics["target_type"] == "step" else []
    out_dir = output_directory(session_dir, args.joint, args.out_dir)

    figure_path = out_dir / f"J{args.joint:02d}_M{motor:02d}_pid_response.png"
    summary_path = out_dir / f"J{args.joint:02d}_M{motor:02d}_pid_metrics.csv"
    steps_path = out_dir / f"J{args.joint:02d}_M{motor:02d}_pid_steps.csv"
    report_path = out_dir / f"J{args.joint:02d}_M{motor:02d}_pid_summary.md"
    plot_analysis(figure_path, trace, servo, args.joint, motor, args.tolerance, args.show)
    write_summary_csv(summary_path, metrics)
    write_step_csv(steps_path, steps)
    write_report(report_path, session_dir, args.joint, motor, metrics, steps, args.tolerance)

    print(f"Session: {session_dir}")
    print(f"Joint/Motor: J{args.joint:02d} / M{motor:02d}")
    print(f"Data source: {trace.source}")
    print(f"Target type: {metrics['target_type']}")
    print(f"RMSE: {display_value(metrics['rmse_deg'])} deg")
    print(f"Max abs error: {display_value(metrics['max_abs_error_deg'])} deg")
    print(f"Figure: {figure_path}")
    print(f"Metrics: {summary_path}")
    print(f"Steps: {steps_path}")
    print(f"Report: {report_path}")
    return {
        "figure": figure_path,
        "metrics": summary_path,
        "steps": steps_path,
        "report": report_path,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze one single-motor PID test recorded by the desktop Test tab."
    )
    parser.add_argument("--joint", required=True, type=parse_joint, help="joint, for example J10")
    parser.add_argument("--motor", type=parse_motor, default=None, help="optional motor override, for example M13")
    parser.add_argument("--session", default="", help="recording session directory; defaults to latest session")
    parser.add_argument("--run-data-dir", default=str(PROJECT_ROOT / "run_data"), help="run_data root")
    parser.add_argument("--start", type=float, default=None, help="analysis start in seconds relative to the first sample")
    parser.add_argument("--end", type=float, default=None, help="analysis end in seconds relative to the first sample")
    parser.add_argument("--tolerance", type=float, default=0.5, help="settling/error tolerance in degrees")
    parser.add_argument("--settle-hold", type=float, default=0.5, help="continuous in-band time required for settling")
    parser.add_argument("--output-limit", type=float, default=600.0, help="outer PID output limit in counts")
    parser.add_argument("--out-dir", default="", help="output directory; defaults inside the session")
    parser.add_argument("--show", action="store_true", help="open the interactive matplotlib window")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        run_analysis(args)
        return 0
    except Exception as exc:
        print(f"[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
