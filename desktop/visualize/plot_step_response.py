#!/usr/bin/env python3
"""使用 Plotly 绘制 Test 页面阶跃响应及其舵机轨迹。

脚本只读取 ``run_data``，不向 P4 发送命令。目标角、反馈角、舵机 CMD、
实际 ABS 和负载全部沿用正弦脚本的统一 P4 时间轴。
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

try:
    from .plot_sine_response import (
        PROJECT_ROOT,
        SERVO_COLORS,
        discover_motors,
        parse_joint,
        parse_motor,
        read_joint_trace,
        read_motor_command_trace,
        read_servo_trace,
        session_label,
    )
except ImportError:
    from plot_sine_response import (  # type: ignore[no-redef]
        PROJECT_ROOT,
        SERVO_COLORS,
        discover_motors,
        parse_joint,
        parse_motor,
        read_joint_trace,
        read_motor_command_trace,
        read_servo_trace,
        session_label,
    )


@dataclass
class StepMetrics:
    session: Path
    step: int
    start_s: float
    from_deg: float
    to_deg: float
    step_deg: float
    response_delay_ms: float
    rise_time_ms: float
    settling_time_ms: float
    overshoot_deg: float
    overshoot_pct: float
    steady_error_deg: float
    mae_deg: float
    max_abs_error_deg: float
    samples: int


def _format_ms(value: float) -> str:
    return f"{value:.0f} ms" if math.isfinite(value) else "n/a"


def detect_step_indices(
    time_s: np.ndarray,
    target: np.ndarray,
    min_step_deg: float,
) -> list[int]:
    """找出目标突跳，并把 100 ms 内的连续改值合并成一次阶跃。"""
    if len(target) < 2:
        return []
    raw = np.flatnonzero(np.abs(np.diff(target)) >= min_step_deg) + 1
    compact: list[int] = []
    for index in raw:
        index = int(index)
        if compact and time_s[index] - time_s[compact[-1]] <= 0.1:
            compact[-1] = index
        else:
            compact.append(index)
    return compact


def _first_crossing(
    time_s: np.ndarray,
    values: np.ndarray,
    threshold: float,
) -> float:
    indices = np.flatnonzero(values >= threshold)
    if not len(indices):
        return float("nan")
    index = int(indices[0])
    if index == 0 or values[index] == values[index - 1]:
        return float(time_s[index])
    fraction = (threshold - values[index - 1]) / (
        values[index] - values[index - 1]
    )
    return float(
        time_s[index - 1]
        + fraction * (time_s[index] - time_s[index - 1])
    )


def _settling_time(
    time_s: np.ndarray,
    error: np.ndarray,
    tolerance_deg: float,
    hold_s: float,
) -> float:
    for start in range(len(time_s)):
        end = int(np.searchsorted(time_s, time_s[start] + hold_s, side="left"))
        if end >= len(time_s):
            break
        if np.all(np.abs(error[start : end + 1]) <= tolerance_deg):
            return float(time_s[start])
    return float("nan")


def analyze_steps(
    session: Path,
    target_time_s: np.ndarray,
    target: np.ndarray,
    actual_time_s: np.ndarray,
    actual: np.ndarray,
    min_step_deg: float,
    tolerance_deg: float,
    settle_hold_s: float,
) -> list[StepMetrics]:
    indices = detect_step_indices(target_time_s, target, min_step_deg)
    metrics: list[StepMetrics] = []
    for number, start in enumerate(indices, start=1):
        end_time = (
            float(target_time_s[indices[number]])
            if number < len(indices)
            else float(target_time_s[-1]) + 1e-9
        )
        segment_mask = (
            (actual_time_s >= target_time_s[start]) & (actual_time_s < end_time)
        )
        segment_time = actual_time_s[segment_mask]
        segment_actual = actual[segment_mask]
        if len(segment_actual) < 3:
            continue

        old_start = max(0, start - 5)
        old_target = float(np.median(target[old_start:start]))
        target_end = indices[number] if number < len(indices) else len(target)
        new_target = float(np.median(target[start:target_end]))
        step_deg = new_target - old_target
        if abs(step_deg) < min_step_deg:
            continue

        before_mask = (
            (actual_time_s < target_time_s[start])
            & (actual_time_s >= target_time_s[start] - 0.3)
        )
        initial_actual = (
            float(np.median(actual[before_mask]))
            if np.any(before_mask)
            else float(np.interp(target_time_s[start], actual_time_s, actual))
        )
        direction = 1.0 if step_deg > 0.0 else -1.0
        requested_travel = direction * (new_target - initial_actual)
        if requested_travel <= 0.0:
            requested_travel = abs(step_deg)
        relative_time = segment_time - target_time_s[start]
        progress = direction * (segment_actual - initial_actual)
        response_10 = _first_crossing(relative_time, progress, 0.1 * requested_travel)
        response_90 = _first_crossing(relative_time, progress, 0.9 * requested_travel)
        rise_time = (
            response_90 - response_10
            if math.isfinite(response_10) and math.isfinite(response_90)
            else float("nan")
        )
        error = new_target - segment_actual
        settled = _settling_time(
            relative_time, error, tolerance_deg, settle_hold_s
        )
        overshoot = max(
            0.0, float(np.max(direction * (segment_actual - new_target)))
        )
        tail_start = max(0, int(np.searchsorted(relative_time, relative_time[-1] - 0.5)))
        tail_error = error[tail_start:]
        metrics.append(
            StepMetrics(
                session=session,
                step=number,
                start_s=float(target_time_s[start]),
                from_deg=old_target,
                to_deg=new_target,
                step_deg=step_deg,
                response_delay_ms=response_10 * 1000.0,
                rise_time_ms=rise_time * 1000.0,
                settling_time_ms=settled * 1000.0,
                overshoot_deg=overshoot,
                overshoot_pct=100.0 * overshoot / abs(step_deg),
                steady_error_deg=float(np.mean(tail_error)),
                mae_deg=float(np.mean(np.abs(error))),
                max_abs_error_deg=float(np.max(np.abs(error))),
                samples=len(segment_actual),
            )
        )
    return metrics


def is_step_session(session: Path, joint: int, min_step_deg: float) -> bool:
    try:
        time_s, target, _actual_time_s, _actual, _origin = read_joint_trace(
            session, joint
        )
    except (OSError, ValueError):
        return False
    if not detect_step_indices(time_s, target, min_step_deg):
        return False
    diff = np.abs(np.diff(target))
    flat_threshold = min(0.05, 0.1 * min_step_deg)
    return bool(len(diff) and np.mean(diff <= flat_threshold) >= 0.7)


def find_latest_sessions(
    run_data: Path,
    joint: int,
    count: int,
    min_step_deg: float,
) -> list[Path]:
    sessions = [
        path
        for path in run_data.iterdir()
        if path.is_dir()
        and path.name.startswith("数据记录_")
        and is_step_session(path, joint, min_step_deg)
    ]
    sessions.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    return list(reversed(sessions[:count]))


def _mean_metric(steps: list[StepMetrics], name: str) -> float:
    values = [float(getattr(step, name)) for step in steps]
    values = [value for value in values if math.isfinite(value)]
    return float(np.mean(values)) if values else float("nan")


def _add_servo_traces(
    figure: go.Figure,
    session: Path,
    motors: Iterable[int],
    origin_ms: float,
) -> None:
    for offset, motor in enumerate(motors):
        color = SERVO_COLORS[offset % len(SERVO_COLORS)]
        servo_t, servo_abs, servo_load = read_servo_trace(session, motor, origin_ms)
        if not len(servo_t):
            continue
        command_t, command = read_motor_command_trace(session, motor, origin_ms)
        figure.add_trace(
            go.Scatter(
                x=servo_t,
                y=servo_abs,
                name=f"M{motor:02d} ABS",
                mode="lines",
                line={"color": color, "width": 1.2},
            ),
            row=1,
            col=1,
            secondary_y=True,
        )
        if len(command_t):
            figure.add_trace(
                go.Scatter(
                    x=command_t,
                    y=command,
                    name=f"M{motor:02d} CMD",
                    mode="lines",
                    line={"color": color, "dash": "dash"},
                ),
                row=1,
                col=1,
                secondary_y=True,
            )
            figure.add_trace(
                go.Scatter(
                    x=command_t,
                    y=command,
                    name=f"M{motor:02d} CMD",
                    mode="lines",
                    line={"color": color, "dash": "dash"},
                    showlegend=False,
                ),
                row=3,
                col=1,
            )
        figure.add_trace(
            go.Scatter(
                x=servo_t,
                y=servo_abs,
                name=f"M{motor:02d} actual ABS",
                mode="lines",
                line={"color": color},
                showlegend=False,
            ),
            row=3,
            col=1,
        )
        if np.any(np.isfinite(servo_load)):
            figure.add_trace(
                go.Scatter(
                    x=servo_t,
                    y=servo_load,
                    name=f"M{motor:02d} load",
                    mode="lines",
                    line={"color": color},
                ),
                row=4,
                col=1,
            )


def plot_session(
    session: Path,
    joint: int,
    motors: Iterable[int],
    output_dir: Path,
    min_step_deg: float,
    tolerance_deg: float,
    settle_hold_s: float,
) -> tuple[list[StepMetrics], Path]:
    target_t, target, actual_t, actual, origin_ms = read_joint_trace(session, joint)
    steps = analyze_steps(
        session,
        target_t,
        target,
        actual_t,
        actual,
        min_step_deg,
        tolerance_deg,
        settle_hold_s,
    )
    if not steps:
        raise ValueError(f"{session.name}: no valid J{joint:02d} step found")
    overlap = (target_t >= actual_t[0]) & (target_t <= actual_t[-1])
    error_t = target_t[overlap]
    error = target[overlap] - np.interp(error_t, actual_t, actual)
    title = (
        f"{session_label(session)} J{joint:02d}: {len(steps)} steps, "
        f"delay={_format_ms(_mean_metric(steps, 'response_delay_ms'))}, "
        f"rise={_format_ms(_mean_metric(steps, 'rise_time_ms'))}, "
        f"MAE={_mean_metric(steps, 'mae_deg'):.2f} deg"
    )
    figure = make_subplots(
        rows=4,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.055,
        row_heights=[0.34, 0.20, 0.24, 0.22],
        specs=[
            [{"secondary_y": True}],
            [{"secondary_y": False}],
            [{"secondary_y": False}],
            [{"secondary_y": False}],
        ],
    )
    figure.add_trace(
        go.Scatter(x=target_t, y=target, name="Target", mode="lines"), row=1, col=1
    )
    figure.add_trace(
        go.Scatter(x=actual_t, y=actual, name="Actual", mode="lines"), row=1, col=1
    )
    figure.add_trace(
        go.Scatter(
            x=error_t,
            y=error,
            name="Target - actual",
            mode="lines",
            line={"color": "#d62728"},
        ),
        row=2,
        col=1,
    )
    figure.add_hline(y=0.0, line_color="#333333", line_width=1, row=2, col=1)
    for level in (-tolerance_deg, tolerance_deg):
        figure.add_hline(
            y=level,
            line_color="#2ca02c",
            line_dash="dot",
            line_width=1,
            row=2,
            col=1,
        )
    for step in steps:
        figure.add_vline(
            x=step.start_s,
            line_dash="dot",
            line_color="#777777",
            row="all",
            col=1,
        )
        figure.add_annotation(
            x=step.start_s,
            y=step.to_deg,
            text=(
                f"Step {step.step}<br>delay {_format_ms(step.response_delay_ms)}"
                f"<br>rise {_format_ms(step.rise_time_ms)}"
            ),
            showarrow=True,
            arrowhead=2,
            row=1,
            col=1,
        )
    _add_servo_traces(figure, session, motors, origin_ms)
    figure.update_yaxes(title_text=f"J{joint:02d} (deg)", row=1, col=1, secondary_y=False)
    figure.update_yaxes(title_text="Servo ABS (counts)", row=1, col=1, secondary_y=True)
    figure.update_yaxes(title_text="Error (deg)", row=2, col=1)
    figure.update_yaxes(title_text="Servo ABS/CMD", row=3, col=1)
    figure.update_yaxes(title_text="Servo load", row=4, col=1)
    figure.update_xaxes(title_text="P4 elapsed time (s)", row=4, col=1)
    figure.update_layout(
        title=title,
        height=980,
        hovermode="x unified",
        template="plotly_white",
        legend={"orientation": "h", "y": 1.04, "x": 0.0},
        margin={"l": 85, "r": 90, "t": 110, "b": 65},
    )
    output_path = output_dir / f"{session.name}_J{joint:02d}_step_response.html"
    figure.write_html(output_path, include_plotlyjs=True, full_html=True)
    return steps, output_path


def write_metrics(path: Path, steps: Iterable[StepMetrics]) -> None:
    rows = []
    for step in steps:
        row = asdict(step)
        row["session"] = step.session.name
        rows.append(row)
    if not rows:
        return
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_comparison(
    traces: list[tuple[Path, np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    joint: int,
    path: Path,
    show: bool,
) -> None:
    figure = make_subplots(rows=len(traces), cols=1, shared_xaxes=False)
    for row, (session, target_t, target, actual_t, actual) in enumerate(traces, 1):
        figure.add_trace(
            go.Scatter(x=target_t, y=target, name="Target", mode="lines", showlegend=row == 1),
            row=row,
            col=1,
        )
        figure.add_trace(
            go.Scatter(x=actual_t, y=actual, name="Actual", mode="lines", showlegend=row == 1),
            row=row,
            col=1,
        )
        figure.add_annotation(
            xref="paper",
            yref=f"y{'' if row == 1 else row} domain",
            x=0.0,
            y=1.0,
            text=session_label(session),
            showarrow=False,
            xanchor="left",
            yanchor="bottom",
        )
        figure.update_yaxes(title_text=f"J{joint:02d} (deg)", row=row, col=1)
        figure.update_xaxes(title_text="P4 elapsed time (s)", row=row, col=1)
    figure.update_layout(
        title=f"J{joint:02d} step response comparison",
        height=max(500, 360 * len(traces)),
        hovermode="x unified",
        template="plotly_white",
        legend={"orientation": "h", "y": 1.04, "x": 0.0},
        margin={"l": 85, "r": 50, "t": 105, "b": 65},
    )
    figure.write_html(path, include_plotlyjs=True, full_html=True)
    if show:
        figure.show()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build interactive Plotly step-response reports from Test-tab CSV sessions."
    )
    parser.add_argument("--joint", required=True, type=parse_joint, help="joint, for example J14")
    parser.add_argument("--motor", action="append", type=parse_motor, help="optional Mxx; repeat for dual motors")
    parser.add_argument("--session", action="append", type=Path, help="session directory; may be repeated")
    parser.add_argument("--latest", type=int, default=1, help="latest matching step sessions, default 1")
    parser.add_argument("--min-step", type=float, default=0.5, help="minimum target jump in degrees")
    parser.add_argument("--tolerance", type=float, default=0.5, help="settling tolerance in degrees")
    parser.add_argument("--settle-hold", type=float, default=0.2, help="continuous in-band settling time in seconds")
    parser.add_argument("--run-data-dir", type=Path, default=PROJECT_ROOT / "run_data")
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--show", action="store_true", help="open the comparison HTML")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    sessions = [path.resolve() for path in (args.session or [])]
    if not sessions:
        sessions = find_latest_sessions(
            args.run_data_dir.resolve(),
            args.joint,
            max(1, args.latest),
            args.min_step,
        )
    if not sessions:
        raise FileNotFoundError(f"no step Test session found for J{args.joint:02d}")
    output_dir = args.out_dir or (
        args.run_data_dir.resolve()
        / f"阶跃响应分析_{sessions[-1].name.removeprefix('数据记录_')}_J{args.joint:02d}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    all_steps: list[StepMetrics] = []
    traces = []
    for session in sessions:
        motors = list(args.motor or discover_motors(session, args.joint))
        steps, _path = plot_session(
            session,
            args.joint,
            motors,
            output_dir,
            args.min_step,
            args.tolerance,
            args.settle_hold,
        )
        target_t, target, actual_t, actual, _origin = read_joint_trace(session, args.joint)
        all_steps.extend(steps)
        traces.append((session, target_t, target, actual_t, actual))
        for step in steps:
            print(
                f"{session.name} step {step.step}: {step.from_deg:.2f}->{step.to_deg:.2f} deg, "
                f"delay={_format_ms(step.response_delay_ms)}, rise={_format_ms(step.rise_time_ms)}, "
                f"settle={_format_ms(step.settling_time_ms)}, overshoot={step.overshoot_pct:.1f}%, "
                f"MAE={step.mae_deg:.3f} deg"
            )
    write_metrics(output_dir / f"J{args.joint:02d}_step_metrics.csv", all_steps)
    plot_comparison(
        traces,
        args.joint,
        output_dir / f"J{args.joint:02d}_step_response_comparison.html",
        args.show,
    )
    print(f"output: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
