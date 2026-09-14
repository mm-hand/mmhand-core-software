#!/usr/bin/env python3
"""将标定快照中的 Tendon LUT、局部斜率和控制增益绘制为 Plotly HTML。

职责：离线读取 ``calibration_bundle.mmcal.json``，复现 P4 对 LUT 样本角度的
解释，并展示单腱动态 Kp 或双腱固定 Kp；脚本不连接设备、不修改快照。
输入：V4/V6 标定快照，以及可选的 J00..J20 关节筛选。
输出：每个有效关节一个可缩放、悬停查看数据的独立 HTML 文件。
"""

from __future__ import annotations

import argparse
import json
import math
import re
import webbrowser
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SNAPSHOT = (
    PROJECT_ROOT / "desktop" / "resources" / "config" /
    "calibration_bundle.mmcal.json"
)
DEFAULT_CONTROL_CONFIG = (
    PROJECT_ROOT / "ESP32-P4" / "ServoBoardMain" / "src" /
    "config" / "ControlConfig.h"
)
JOINT_COUNT = 21
LUT_ACTUATOR_COUNT = 2


@dataclass(frozen=True)
class ControlGains:
    """与 P4 ``ControlConfig.h`` 同源的 LUT 增益参数。"""

    single_fallback_kp: float
    single_tight_gain: float
    single_release_gain: float
    single_large_error_gain: float
    single_blend_start_deg: float
    single_error_max_deg: float
    dual_kp: float


def parse_joint(value: str) -> int:
    """接受 J14 或 14，并统一返回 0..20 的关节索引。"""
    text = str(value).strip().upper().removeprefix("J")
    try:
        joint = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("joint must be J00..J20") from exc
    if not 0 <= joint < JOINT_COUNT:
        raise argparse.ArgumentTypeError("joint must be J00..J20")
    return joint


def _read_cpp_float(text: str, name: str) -> float:
    """读取简单的 ``constexpr float``，让图中参数始终跟随固件配置。"""
    pattern = rf"\b{name}\s*=\s*([-+]?\d+(?:\.\d+)?)(?:[eE][-+]?\d+)?f?\s*;"
    match = re.search(pattern, text)
    if not match:
        raise ValueError(f"missing {name} in ControlConfig.h")
    return float(match.group(1))


def load_control_gains(path: Path) -> ControlGains:
    """从固件配置读取增益，避免可视化脚本另存一份会漂移的常量。"""
    text = path.read_text(encoding="utf-8")
    release_gain = _read_cpp_float(text, "kSingleTendonLargeErrorLutGain")
    return ControlGains(
        single_fallback_kp=_read_cpp_float(text, "kSingleTendonKp"),
        single_tight_gain=_read_cpp_float(text, "kSingleTendonDynamicLutGain"),
        single_release_gain=release_gain,
        single_large_error_gain=release_gain,
        single_blend_start_deg=_read_cpp_float(
            text, "kSingleTendonDynamicKpBlendStartDeg"
        ),
        single_error_max_deg=_read_cpp_float(
            text, "kSingleTendonDynamicKpErrorMaxDeg"
        ),
        dual_kp=_read_cpp_float(text, "kDualTendonKp"),
    )


def load_calibration_results(path: Path) -> list[dict[str, Any] | None]:
    """读取快照中的 21 路结果；拒绝缺少正式结果数组的任意 JSON。"""
    payload = json.loads(path.read_text(encoding="utf-8"))
    results = payload.get("joint_calib_results")
    if not isinstance(results, list):
        raise ValueError("snapshot has no joint_calib_results array")
    return list(results)


def lut_sample_angles(result: dict[str, Any]) -> np.ndarray:
    """逐字复现 P4 的末点规则：最后一段可不足标称 5 度。"""
    count = int(result.get("tendonLutCount", 0))
    start = float(result.get("tendonLutAngleStartDeg", math.nan))
    step = float(result.get("tendonLutAngleStepDeg", math.nan))
    current_max = float(result.get("currentMaxDeg", math.nan))
    if count < 3 or count > 24 or not math.isfinite(start) or not step > 0.0:
        raise ValueError("invalid LUT count/start/step")
    angles = start + step * np.arange(count, dtype=float)
    if math.isfinite(current_max) and start < current_max < angles[-1]:
        angles[-1] = current_max
    if np.any(np.diff(angles) <= 0.0):
        raise ValueError("LUT sample angles are not strictly increasing")
    return angles


def valid_actuators(result: dict[str, Any]) -> list[tuple[int, int, np.ndarray]]:
    """返回有效的 ``(执行器行号, Mxx, ABS样本)``，单腱自然只返回一路。"""
    count = int(result.get("tendonLutCount", 0))
    motors = result.get("tendonLutMotor", [])
    rows = result.get("tendonLutMotorAbs", [])
    valid = []
    for actuator in range(LUT_ACTUATOR_COUNT):
        if actuator >= len(motors) or actuator >= len(rows):
            continue
        motor = int(motors[actuator])
        if not 0 <= motor < 27 or len(rows[actuator]) < count:
            continue
        values = np.asarray(rows[actuator][:count], dtype=float)
        if np.all(np.isfinite(values)):
            valid.append((actuator, motor, values))
    return valid


def _joint_is_usable(result: object, joint: int) -> bool:
    """只绘制本关节成功且带有效 Tendon LUT 的当前快照。"""
    return bool(
        isinstance(result, dict)
        and int(result.get("joint", joint)) == joint
        and result.get("success")
        and result.get("tendonLutValid")
        and int(result.get("tendonLutCount", 0)) >= 3
    )


def plot_joint_lut(
    result: dict[str, Any],
    gains: ControlGains,
    output_path: Path,
) -> Path:
    """绘制一个关节；斜率和 Kp 使用相邻实测点计算，不做平滑美化。"""
    joint = int(result["joint"])
    angles = lut_sample_angles(result)
    actuators = valid_actuators(result)
    if not actuators:
        raise ValueError(f"J{joint:02d} has no valid LUT actuator")

    figure = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        row_heights=[0.46, 0.27, 0.27],
        subplot_titles=(
            "LUT: URDF angle -> servo logical ABS",
            "Adjacent-point slope",
            "Near-target Kp estimate and large-error Kp",
        ),
    )
    colors = ("#0072B2", "#D55E00")
    angle_mid = (angles[:-1] + angles[1:]) * 0.5
    is_single = len(actuators) == 1

    for order, (_actuator, motor, motor_abs) in enumerate(actuators):
        color = colors[order % len(colors)]
        signed_slope = np.diff(motor_abs) / np.diff(angles)
        slope = np.abs(signed_slope)
        custom = np.column_stack((np.arange(len(angles)),))
        figure.add_trace(
            go.Scatter(
                x=angles,
                y=motor_abs,
                mode="lines+markers",
                name=f"M{motor:02d} LUT",
                line={"color": color, "width": 2},
                marker={"size": 7},
                customdata=custom,
                hovertemplate=(
                    "sample=%{customdata[0]:.0f}<br>angle=%{x:.3f} deg"
                    "<br>ABS=%{y:.0f} counts<extra></extra>"
                ),
            ),
            row=1,
            col=1,
        )
        figure.add_trace(
            go.Bar(
                x=angle_mid,
                y=signed_slope,
                width=np.diff(angles) * 0.82,
                name=f"M{motor:02d} signed slope",
                marker_color=color,
                opacity=0.72,
                hovertemplate=(
                    "segment center=%{x:.3f} deg"
                    "<br>slope=%{y:.3f} counts/deg<extra></extra>"
                ),
            ),
            row=2,
            col=1,
        )

        if is_single:
            # 固件近目标时按运动方向选择局部斜率增益；图中分别列出两条，
            # 大误差则使用整条 LUT 平均斜率，避免把局部异常点放大到全行程。
            tight_kp = gains.single_tight_gain * slope
            release_kp = gains.single_release_gain * slope
            average_slope = abs(float(motor_abs[-1] - motor_abs[0])) / float(
                angles[-1] - angles[0]
            )
            large_error_kp = gains.single_large_error_gain * average_slope
            figure.add_trace(
                go.Scatter(
                    x=angle_mid,
                    y=tight_kp,
                    mode="lines+markers",
                    name=f"tight Kp ({gains.single_tight_gain:g} x local slope)",
                    line={"color": "#CC3311", "width": 2},
                ),
                row=3,
                col=1,
            )
            figure.add_trace(
                go.Scatter(
                    x=angle_mid,
                    y=release_kp,
                    mode="lines+markers",
                    name=f"release Kp ({gains.single_release_gain:g} x local slope)",
                    line={"color": "#009988", "width": 2},
                ),
                row=3,
                col=1,
            )
            figure.add_hline(
                y=large_error_kp,
                line_dash="dash",
                line_color="#AA4499",
                annotation_text=f"|error| >= {gains.single_error_max_deg:g} deg: Kp={large_error_kp:.2f}",
                annotation_position="bottom right",
                row=3,
                col=1,
            )
            figure.add_hline(
                y=gains.single_fallback_kp,
                line_dash="dot",
                line_color="#666666",
                annotation_text=f"fallback Kp={gains.single_fallback_kp:g}",
                annotation_position="top right",
                row=3,
                col=1,
            )

    if not is_single:
        # 双腱 PID 的误差已经是 LUT 插值得到的 motor counts，因此 Kp 不再
        # 乘 counts/deg 斜率；这条水平线用于避免误读为双腱也采用动态 Kp。
        figure.add_hline(
            y=gains.dual_kp,
            line_color="#AA4499",
            annotation_text=f"dual-tendon fixed Kp={gains.dual_kp:g}",
            annotation_position="top right",
            row=3,
            col=1,
        )

    sample_count = len(angles)
    motor_names = ", ".join(f"M{motor:02d}" for _, motor, _ in actuators)
    figure.update_yaxes(title_text="logical ABS (counts)", row=1, col=1)
    figure.update_yaxes(title_text="signed counts/deg", zeroline=True, row=2, col=1)
    figure.update_yaxes(title_text="effective Kp", row=3, col=1)
    figure.update_xaxes(title_text="URDF angle (deg)", row=3, col=1)
    figure.update_layout(
        title=(
            f"J{joint:02d} Tendon LUT | {motor_names} | {sample_count} samples | "
            f"range {angles[0]:.2f}..{angles[-1]:.2f} deg"
        ),
        height=920,
        template="plotly_white",
        hovermode="x unified",
        legend={"orientation": "h", "y": 1.06, "x": 0.0},
        margin={"l": 85, "r": 55, "t": 110, "b": 65},
        barmode="group",
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.write_html(output_path, include_plotlyjs=True, full_html=True)
    return output_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Visualize calibration Tendon LUT and LUT-derived Kp with Plotly."
    )
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument(
        "--joint",
        action="append",
        type=parse_joint,
        help="J00..J20; repeat to select multiple joints, default is all valid LUTs",
    )
    parser.add_argument("--control-config", type=Path, default=DEFAULT_CONTROL_CONFIG)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--show", action="store_true", help="open generated HTML files")
    return parser


def selected_joints(
    results: list[dict[str, Any] | None], requested: Iterable[int] | None
) -> list[int]:
    """未指定关节时自动选择快照中全部有效 LUT。"""
    candidates = list(dict.fromkeys(requested)) if requested else list(range(JOINT_COUNT))
    return [
        joint for joint in candidates
        if joint < len(results) and _joint_is_usable(results[joint], joint)
    ]


def main() -> int:
    args = build_parser().parse_args()
    snapshot = args.snapshot.resolve()
    gains = load_control_gains(args.control_config.resolve())
    results = load_calibration_results(snapshot)
    joints = selected_joints(results, args.joint)
    if not joints:
        raise SystemExit("no usable Tendon LUT found for the requested joints")

    output_dir = (
        args.out_dir.resolve()
        if args.out_dir
        else PROJECT_ROOT / "run_data" / f"LUT可视化_{snapshot.stem}"
    )
    paths = []
    for joint in joints:
        result = results[joint]
        assert isinstance(result, dict)
        path = plot_joint_lut(
            result,
            gains,
            output_dir / f"J{joint:02d}_tendon_lut.html",
        )
        paths.append(path)
        print(f"J{joint:02d}: {path}")
    if args.show:
        for path in paths:
            webbrowser.open(path.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
