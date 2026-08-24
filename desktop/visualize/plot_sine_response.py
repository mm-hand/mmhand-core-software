#!/usr/bin/env python3
"""绘制 Test 页面正弦波的目标角、实际角、误差和舵机响应。

脚本只读取 run_data，不向 P4 发送命令。默认选择最近的正弦 Test 会话，
也可以显式指定一个或多个数据记录目录，适用于任意 J00..J20。
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
JOINT_COUNT = 21
MOTOR_COUNT = 27


@dataclass
class SineMetrics:
    session: Path
    frequency_hz: float
    target_amplitude_deg: float
    actual_amplitude_deg: float
    amplitude_ratio: float
    peak_to_peak_ratio: float
    phase_lag_deg: float
    phase_delay_ms: float
    midpoint_delay_ms: float
    correlation_delay_ms: float
    rmse_deg: float
    mae_deg: float
    max_abs_error_deg: float
    analysis_start_s: float
    trace_suspect: bool
    duration_s: float
    samples: int


def parse_joint(value: str) -> int:
    text = str(value).strip().upper().removeprefix("J")
    try:
        joint = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("joint must be J00..J20") from exc
    if not 0 <= joint < JOINT_COUNT:
        raise argparse.ArgumentTypeError("joint must be J00..J20")
    return joint


def parse_motor(value: str) -> int:
    text = str(value).strip().upper().removeprefix("M")
    try:
        motor = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("motor must be M00..M26") from exc
    if not 0 <= motor < MOTOR_COUNT:
        raise argparse.ArgumentTypeError("motor must be M00..M26")
    return motor


def safe_float(value: object) -> float:
    try:
        result = float(str(value).strip())
        return result if math.isfinite(result) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def row_time_ms(row: dict[str, str]) -> float:
    return safe_float(row.get("p4_time_ms", ""))


def is_sine_session(session: Path, joint: int) -> bool:
    joint_csv = session / "02_关节总表.csv"
    event_csv = session / "06_操作事件总表.csv"
    if not joint_csv.is_file() or not event_csv.is_file():
        return False
    events = read_rows(event_csv)
    if not any("Test started" in str(row.get("detail", "")) for row in events):
        return False
    rows = read_rows(joint_csv)
    target_key = f"J{joint:02d}_target_deg"
    targets = np.asarray([safe_float(row.get(target_key, "")) for row in rows])
    targets = targets[np.isfinite(targets)]
    return len(targets) >= 8 and float(np.ptp(targets)) >= 0.5


def find_latest_sessions(run_data: Path, joint: int, count: int) -> list[Path]:
    sessions = [
        path for path in run_data.iterdir()
        if path.is_dir() and path.name.startswith("数据记录_") and is_sine_session(path, joint)
    ]
    sessions.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    selected = sessions[:count]
    selected.reverse()
    return selected


def read_joint_trace(
    session: Path, joint: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rows = read_rows(session / "02_关节总表.csv")
    prefix = f"J{joint:02d}_"
    samples = []
    for row in rows:
        timestamp = row_time_ms(row)
        feedback_age_ms = safe_float(row.get("feedback_age_ms", "0"))
        target = safe_float(row.get(prefix + "target_deg", ""))
        actual = safe_float(row.get(prefix + "current_deg", ""))
        valid = int(safe_float(row.get(prefix + "current_valid", "0")) or 0)
        if math.isfinite(timestamp) and math.isfinite(target) and math.isfinite(actual) and valid:
            actual_timestamp = timestamp - (
                feedback_age_ms if math.isfinite(feedback_age_ms) else 0.0
            )
            samples.append((timestamp, target, actual_timestamp, actual))
    if len(samples) < 8:
        raise ValueError(f"{session.name}: J{joint:02d} has too few valid samples")
    data = np.asarray(samples, dtype=float)
    origin_ms = min(data[0, 0], data[0, 2])
    target_time_s = (data[:, 0] - origin_ms) / 1000.0
    actual_time_s = (data[:, 2] - origin_ms) / 1000.0
    return target_time_s, data[:, 1], actual_time_s, data[:, 3]


def discover_motors(session: Path, joint: int) -> list[int]:
    rows = read_rows(session / "03_舵机总表.csv")
    found = []
    for motor in range(MOTOR_COUNT):
        key = f"M{motor:02d}_joint_index"
        if any(int(safe_float(row.get(key, "-1")) or -1) == joint for row in rows[:20]):
            found.append(motor)
    return found


def read_servo_trace(
    session: Path, motor: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rows = read_rows(session / "03_舵机总表.csv")
    prefix = f"M{motor:02d}_"
    samples = []
    for row in rows:
        timestamp = row_time_ms(row)
        target = safe_float(row.get(prefix + "target_abs", ""))
        actual = safe_float(row.get(prefix + "abs", ""))
        load = safe_float(row.get(prefix + "load", ""))
        if math.isfinite(timestamp) and math.isfinite(target) and math.isfinite(actual):
            samples.append((timestamp, target, actual, load))
    if not samples:
        return tuple(np.asarray([], dtype=float) for _ in range(4))  # type: ignore[return-value]
    data = np.asarray(samples, dtype=float)
    return (data[:, 0] - data[0, 0]) / 1000.0, data[:, 1], data[:, 2], data[:, 3]


def session_label(session: Path) -> str:
    """返回适合绘图的 ASCII 会话标签，避免默认字体缺少中文字形。"""
    return session.name.removeprefix("数据记录_")


def estimate_frequency(time_s: np.ndarray, target: np.ndarray) -> float:
    duration = float(time_s[-1] - time_s[0])
    if duration <= 0.0:
        return float("nan")
    dt = float(np.median(np.diff(time_s)))
    if not math.isfinite(dt) or dt <= 0.0:
        return float("nan")
    uniform_t = np.arange(time_s[0], time_s[-1], dt)
    if len(uniform_t) < 8:
        return float("nan")
    signal = np.interp(uniform_t, time_s, target)
    signal -= float(np.mean(signal))
    frequencies = np.fft.rfftfreq(len(signal), dt)
    magnitude = np.abs(np.fft.rfft(signal))
    valid = (frequencies >= max(0.03, 0.5 / duration)) & (frequencies <= 10.0)
    if not np.any(valid):
        return float("nan")
    indices = np.flatnonzero(valid)
    coarse = float(frequencies[indices[int(np.argmax(magnitude[valid]))]])

    # FFT 频点间隔由记录时长决定，0.2 Hz 短记录容易被量化成 0.208 Hz。
    # 在粗略峰值附近用正弦最小二乘细化，避免频率误差继续污染增益和相位。
    lower = max(0.03, coarse * 0.75)
    upper = min(10.0, coarse * 1.25)
    candidates = np.linspace(lower, upper, 1001)
    best_frequency = coarse
    best_error = float("inf")
    for candidate in candidates:
        omega_t = 2.0 * math.pi * candidate * time_s
        design = np.column_stack(
            (np.ones_like(time_s), np.sin(omega_t), np.cos(omega_t))
        )
        coefficients = np.linalg.lstsq(design, target, rcond=None)[0]
        error = float(np.mean((target - design @ coefficients) ** 2))
        if error < best_error:
            best_error = error
            best_frequency = float(candidate)
    return best_frequency


def sine_analysis_start(
    time_s: np.ndarray, target: np.ndarray, frequency_hz: float
) -> float:
    """跳过进入 Test 时的保持段和首个过渡周期，只统计稳定周期。"""
    span = float(np.ptp(target))
    threshold = max(0.1, span * 0.01)
    moving = np.flatnonzero(np.abs(target - target[0]) >= threshold)
    motion_start = float(time_s[moving[0]]) if len(moving) else float(time_s[0])
    candidate = motion_start + 1.0 / frequency_hz
    # 短记录不足两个周期时不强行丢弃首周期。
    if float(time_s[-1] - candidate) < 1.5 / frequency_hz:
        return motion_start
    return candidate


def sine_fit(time_s: np.ndarray, values: np.ndarray, frequency_hz: float) -> tuple[float, float, float]:
    omega_t = 2.0 * math.pi * frequency_hz * time_s
    design = np.column_stack((np.ones_like(time_s), np.sin(omega_t), np.cos(omega_t)))
    offset, sin_coeff, cos_coeff = np.linalg.lstsq(design, values, rcond=None)[0]
    amplitude = float(math.hypot(sin_coeff, cos_coeff))
    phase_deg = float(math.degrees(math.atan2(cos_coeff, sin_coeff)))
    return float(offset), amplitude, phase_deg


def level_crossings(
    time_s: np.ndarray, values: np.ndarray, level: float, rising: bool
) -> np.ndarray:
    """用相邻采样线性插值计算指定方向的中线穿越时刻。"""
    crossings = []
    for index in range(len(values) - 1):
        delta = values[index + 1] - values[index]
        if delta == 0.0 or (delta > 0.0) != rising:
            continue
        before = values[index] - level
        after = values[index + 1] - level
        if before * after > 0.0:
            continue
        fraction = (level - values[index]) / delta
        crossings.append(
            time_s[index] + fraction * (time_s[index + 1] - time_s[index])
        )
    return np.asarray(crossings, dtype=float)


def midpoint_delay(
    target_time_s: np.ndarray,
    target: np.ndarray,
    actual_time_s: np.ndarray,
    actual: np.ndarray,
    frequency_hz: float,
) -> float:
    """统计上升/下降经过目标中线的配对延迟，降低平台段对相位的影响。"""
    level = 0.5 * (float(np.min(target)) + float(np.max(target)))
    delays = []
    max_delay = 0.4 / frequency_hz
    for rising in (True, False):
        target_crossings = level_crossings(target_time_s, target, level, rising)
        actual_crossings = level_crossings(actual_time_s, actual, level, rising)
        for target_crossing in target_crossings:
            candidates = actual_crossings[
                (actual_crossings >= target_crossing)
                & (actual_crossings <= target_crossing + max_delay)
            ]
            if len(candidates):
                delays.append(float(candidates[0] - target_crossing))
    return float(np.median(delays) * 1000.0) if delays else float("nan")


def correlation_delay(
    target_time_s: np.ndarray,
    target: np.ndarray,
    actual_time_s: np.ndarray,
    actual: np.ndarray,
    frequency_hz: float,
) -> float:
    """寻找归一化波形相关性最高的正延迟，作为非正弦响应的整体延迟。"""
    max_delay = 0.4 / frequency_hz
    start = max(float(target_time_s[0]), float(actual_time_s[0]))
    end = min(float(target_time_s[-1]), float(actual_time_s[-1])) - max_delay
    if end <= start:
        return float("nan")
    timeline = np.arange(start, end, 0.01)
    target_uniform = np.interp(timeline, target_time_s, target)
    target_zero_mean = target_uniform - float(np.mean(target_uniform))
    target_norm = float(np.linalg.norm(target_zero_mean))
    best_delay = 0.0
    best_correlation = -1.0
    for delay in np.arange(0.0, max_delay + 0.0005, 0.001):
        actual_uniform = np.interp(timeline + delay, actual_time_s, actual)
        actual_zero_mean = actual_uniform - float(np.mean(actual_uniform))
        denominator = target_norm * float(np.linalg.norm(actual_zero_mean))
        if denominator <= 0.0:
            continue
        correlation = float(np.dot(target_zero_mean, actual_zero_mean) / denominator)
        if correlation > best_correlation:
            best_correlation = correlation
            best_delay = float(delay)
    return best_delay * 1000.0


def compute_metrics(
    session: Path,
    target_time_s: np.ndarray,
    target: np.ndarray,
    actual_time_s: np.ndarray,
    actual: np.ndarray,
) -> SineMetrics:
    initial_frequency = estimate_frequency(target_time_s, target)
    initial_start = sine_analysis_start(target_time_s, target, initial_frequency)
    initial_mask = target_time_s >= initial_start
    frequency = estimate_frequency(target_time_s[initial_mask], target[initial_mask])
    analysis_start = sine_analysis_start(target_time_s, target, frequency)
    target_mask = target_time_s >= analysis_start
    actual_mask = actual_time_s >= analysis_start
    fit_target_time = target_time_s[target_mask]
    fit_target = target[target_mask]
    fit_actual_time = actual_time_s[actual_mask]
    fit_actual = actual[actual_mask]
    _, target_amp, target_phase = sine_fit(fit_target_time, fit_target, frequency)
    _, actual_amp, actual_phase = sine_fit(fit_actual_time, fit_actual, frequency)
    phase_lag = (target_phase - actual_phase + 180.0) % 360.0 - 180.0
    overlap = (
        (target_time_s >= analysis_start)
        & (target_time_s >= fit_actual_time[0])
        & (target_time_s <= fit_actual_time[-1])
    )
    compared_target = target[overlap]
    compared_actual = np.interp(
        target_time_s[overlap], fit_actual_time, fit_actual
    )
    error = compared_target - compared_actual
    midpoint_delay_ms = midpoint_delay(
        fit_target_time,
        fit_target,
        fit_actual_time,
        fit_actual,
        frequency,
    )
    correlation_delay_ms = correlation_delay(
        fit_target_time,
        fit_target,
        fit_actual_time,
        fit_actual,
        frequency,
    )
    trace_suspect = bool(
        float(np.max(fit_actual)) < float(np.min(fit_target))
        or float(np.min(fit_actual)) > float(np.max(fit_target))
    )
    return SineMetrics(
        session=session,
        frequency_hz=frequency,
        target_amplitude_deg=target_amp,
        actual_amplitude_deg=actual_amp,
        amplitude_ratio=actual_amp / target_amp if target_amp > 1e-9 else float("nan"),
        peak_to_peak_ratio=(
            float(np.ptp(fit_actual)) / float(np.ptp(fit_target))
            if float(np.ptp(fit_target)) > 1e-9
            else float("nan")
        ),
        phase_lag_deg=phase_lag,
        phase_delay_ms=phase_lag / (360.0 * frequency) * 1000.0,
        midpoint_delay_ms=midpoint_delay_ms,
        correlation_delay_ms=correlation_delay_ms,
        rmse_deg=float(np.sqrt(np.mean(error * error))),
        mae_deg=float(np.mean(np.abs(error))),
        max_abs_error_deg=float(np.max(np.abs(error))),
        analysis_start_s=analysis_start,
        trace_suspect=trace_suspect,
        duration_s=float(target_time_s[-1] - analysis_start),
        samples=len(compared_target),
    )


def plot_session(
    session: Path,
    joint: int,
    motors: Iterable[int],
    output_dir: Path,
) -> tuple[SineMetrics, Path]:
    target_time_s, target, actual_time_s, actual = read_joint_trace(session, joint)
    metrics = compute_metrics(session, target_time_s, target, actual_time_s, actual)
    error_time_s = target_time_s[
        (target_time_s >= metrics.analysis_start_s)
        & (target_time_s >= actual_time_s[0])
        & (target_time_s <= actual_time_s[-1])
    ]
    error = target[
        (target_time_s >= metrics.analysis_start_s)
        & (target_time_s >= actual_time_s[0])
        & (target_time_s <= actual_time_s[-1])
    ] - np.interp(error_time_s, actual_time_s, actual)
    motors = list(motors)
    figure, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    axes[0].plot(target_time_s, target, label="Target", linewidth=1.8)
    axes[0].plot(actual_time_s, actual, label="Actual", linewidth=1.5)
    axes[0].axvline(
        metrics.analysis_start_s,
        color="tab:gray",
        linestyle=":",
        linewidth=1.0,
        label="Metrics start",
    )
    axes[0].set_ylabel("Joint angle (deg)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(
        f"{session_label(session)} J{joint:02d}: {metrics.frequency_hz:.3f} Hz, "
        f"fundamental lag={metrics.phase_lag_deg:.1f} deg, "
        f"midpoint={metrics.midpoint_delay_ms:.0f} ms"
        + (" [TRACE SUSPECT]" if metrics.trace_suspect else "")
    )
    axes[1].plot(error_time_s, error, color="tab:red", linewidth=1.2)
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("Target - actual (deg)")
    axes[1].grid(True, alpha=0.3)

    for motor in motors:
        servo_t, servo_target, servo_actual, servo_load = read_servo_trace(session, motor)
        if len(servo_t) == 0:
            continue
        # 现有舍机总表的 target_abs 可能是进入负反馈前的旧值。
        # 只有它在本轮内确实变化时才画出，防止将陈旧快照误解为 PID 实时输出。
        if float(np.ptp(servo_target)) >= 1.0:
            axes[2].plot(servo_t, servo_target, "--", linewidth=1.0, label=f"M{motor:02d} target")
        axes[2].plot(servo_t, servo_actual, linewidth=1.2, label=f"M{motor:02d} actual")
        axes[3].plot(servo_t, servo_load, linewidth=1.1, label=f"M{motor:02d} load")
    axes[2].set_ylabel("Servo ABS (counts)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")
    axes[3].set_ylabel("Servo load")
    axes[3].set_xlabel("P4 elapsed time (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="best")
    figure.tight_layout(pad=1.4)
    figure.subplots_adjust(left=0.10, right=0.98)
    output_path = output_dir / f"{session.name}_J{joint:02d}_sine_response.png"
    figure.savefig(output_path, dpi=170, bbox_inches="tight")
    plt.close(figure)
    return metrics, output_path


def write_metrics(path: Path, metrics: list[SineMetrics]) -> None:
    fields = list(SineMetrics.__dataclass_fields__)
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for item in metrics:
            row = dict(item.__dict__)
            row["session"] = item.session.name
            writer.writerow(row)


def plot_comparison(
    traces: list[tuple[Path, np.ndarray, np.ndarray, np.ndarray, np.ndarray, SineMetrics]],
    joint: int,
    path: Path,
) -> None:
    figure, axes = plt.subplots(len(traces), 1, figsize=(14, 3.6 * len(traces)), squeeze=False)
    for axis, (session, target_time_s, target, actual_time_s, actual, metrics) in zip(axes[:, 0], traces):
        axis.plot(target_time_s, target, label="Target", linewidth=1.6)
        axis.plot(actual_time_s, actual, label="Actual", linewidth=1.3)
        axis.axvline(
            metrics.analysis_start_s,
            color="tab:gray",
            linestyle=":",
            linewidth=1.0,
        )
        axis.set_title(
            f"{session_label(session)}: f={metrics.frequency_hz:.3f} Hz, "
            f"lag={metrics.phase_lag_deg:.1f} deg/{metrics.phase_delay_ms:.0f} ms, "
            f"mid={metrics.midpoint_delay_ms:.0f} ms, "
            f"p2p gain={metrics.peak_to_peak_ratio:.3f}"
            + (" [TRACE SUSPECT]" if metrics.trace_suspect else "")
        )
        axis.set_ylabel(f"J{joint:02d} (deg)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1, 0].set_xlabel("P4 elapsed time (s)")
    figure.tight_layout(pad=1.4)
    figure.subplots_adjust(left=0.10, right=0.98)
    figure.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(figure)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot sine response from Test-tab CSV sessions.")
    parser.add_argument("--joint", required=True, type=parse_joint, help="joint, for example J15")
    parser.add_argument("--motor", action="append", type=parse_motor, help="optional Mxx; repeat for dual motors")
    parser.add_argument("--session", action="append", type=Path, help="session directory; may be repeated")
    parser.add_argument("--latest", type=int, default=1, help="latest matching sine sessions, default 1")
    parser.add_argument("--run-data-dir", type=Path, default=PROJECT_ROOT / "run_data")
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--show", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    sessions = [path.resolve() for path in (args.session or [])]
    if not sessions:
        sessions = find_latest_sessions(args.run_data_dir.resolve(), args.joint, max(1, args.latest))
    if not sessions:
        raise FileNotFoundError(f"no sine Test session found for J{args.joint:02d}")
    output_dir = args.out_dir or (
        args.run_data_dir.resolve() / f"正弦响应分析_{sessions[-1].name.removeprefix('数据记录_')}_J{args.joint:02d}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    all_metrics = []
    traces = []
    output_paths = []
    for session in sessions:
        motors = list(args.motor or discover_motors(session, args.joint))
        metrics, output_path = plot_session(session, args.joint, motors, output_dir)
        target_time_s, target, actual_time_s, actual = read_joint_trace(
            session, args.joint
        )
        all_metrics.append(metrics)
        traces.append(
            (session, target_time_s, target, actual_time_s, actual, metrics)
        )
        output_paths.append(output_path)
    metrics_path = output_dir / f"J{args.joint:02d}_sine_metrics.csv"
    comparison_path = output_dir / f"J{args.joint:02d}_sine_frequency_comparison.png"
    write_metrics(metrics_path, all_metrics)
    plot_comparison(traces, args.joint, comparison_path)
    for metrics in all_metrics:
        print(
            f"{metrics.session.name}: f={metrics.frequency_hz:.4f} Hz, "
            f"gain={metrics.amplitude_ratio:.4f}, lag={metrics.phase_lag_deg:.2f} deg, "
            f"delay={metrics.phase_delay_ms:.1f} ms, "
            f"midpoint={metrics.midpoint_delay_ms:.1f} ms, "
            f"xcorr={metrics.correlation_delay_ms:.1f} ms, "
            f"p2p_gain={metrics.peak_to_peak_ratio:.4f}, RMSE={metrics.rmse_deg:.3f} deg"
        )
    print(f"output: {output_dir}")
    if args.show:
        image = plt.imread(comparison_path)
        plt.figure(figsize=(14, 8))
        plt.imshow(image)
        plt.axis("off")
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
