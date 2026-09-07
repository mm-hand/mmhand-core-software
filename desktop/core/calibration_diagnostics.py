"""自动标定诊断协议目录与用户可读格式化。"""

from dataclasses import dataclass
from typing import Callable, Dict, Tuple

from . import protocol


Formatter = Callable[[Tuple[int, int, int], int], str]


@dataclass(frozen=True)
class CalibrationDiagnosticSpec:
    title: str
    fields: Tuple[str, str, str, str]
    formatter: Formatter
    suggestion: str
    legacy: bool = False


def _hex(value: int) -> str:
    return f"0x{int(value) & 0xFFFFFFFF:08X}"


def _fields_formatter(
    fields: Tuple[str, str, str, str],
    kinds: Tuple[str, str, str, str],
) -> Formatter:
    def format_fields(values: Tuple[int, int, int], bitmap: int) -> str:
        raw = (*values, bitmap)
        rendered = [
            _hex(value) if kind == "hex" else str(value)
            for value, kind in zip(raw, kinds)
        ]
        return ", ".join(
            f"{name}={value}"
            for name, value in zip(fields, rendered)
            if name
        )

    return format_fields


def _format_from_spec(
    spec: CalibrationDiagnosticSpec,
    values: Tuple[int, int, int],
    bitmap: int,
) -> str:
    return spec.formatter(values, bitmap)


def _format_workspace(values: Tuple[int, int, int], bitmap: int) -> str:
    helper_joint, pose, stage = values
    helper = f"J{helper_joint:02d}" if 0 <= helper_joint < protocol.ENCODER_COUNT else "J?"
    pose_text = {
        0: "NONE",
        1: "POSE_A (primary release, secondary tighten)",
        2: "POSE_B (secondary release, primary tighten)",
    }.get(pose, str(pose))
    stage_text = {
        101: "preflight",
        102: "execute",
        103: "post-calibration pose",
    }.get(stage, str(stage))
    return (
        f"helper_joint={helper}, pose={pose_text}, stage={stage_text}, "
        f"motor_mask={_hex(bitmap)}"
    )


def _format_joint_step(_values: Tuple[int, int, int], _bitmap: int) -> str:
    return "仅使用本轮 completed/failed joint mask；无具体机械根因"


def _spec(
    title: str,
    fields: Tuple[str, str, str, str],
    suggestion: str,
    *,
    kinds: Tuple[str, str, str, str] = ("int", "int", "int", "hex"),
    formatter: Formatter | None = None,
    legacy: bool = False,
) -> CalibrationDiagnosticSpec:
    return CalibrationDiagnosticSpec(
        title, fields, formatter or _fields_formatter(fields, kinds), suggestion, legacy
    )


CALIBRATION_DIAGNOSTICS: Dict[int, CalibrationDiagnosticSpec] = {
    protocol.CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY: _spec(
        "没有可标定的 active 关节",
        ("active_joint_mask", "active_motor_mask", "", ""),
        "在 Active Scope 中选择需要标定的关节和电机后重试。",
        kinds=("hex", "hex", "int", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_ENCODER_ERROR: _spec(
        "磁编码器反馈无效",
        ("encoder_valid_mask", "online_joint_mask", "fault_joint_mask", "affected_joint_mask"),
        "检查对应磁编码器供电、SPI/CAN 接线、S3/P4 通信和传感器板状态。",
        kinds=("hex", "hex", "hex", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_SERVO_OFFLINE: _spec(
        "舵机反馈缺失",
        ("online_motor_mask", "configured_motor_mask", "", "affected_motor_mask"),
        "检查对应舵机电源、串口总线、ID 映射和接线。",
        kinds=("hex", "hex", "int", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_OVERLOAD: _spec(
        "标定中过载", ("load", "limit", "raw_load", "motor_mask"),
        "卸载机械外力，检查绳索是否过紧或机构卡住，Clear Fault 后重新标定。",
    ),
    protocol.CALIB_DIAG_REASON_ZERO_CROSS_INVALID: _spec(
        "零点推算无效", ("taut", "zero", "tight", "motor_mask"),
        "检查编码器方向、标定起始姿态和绳索预紧。",
    ),
    protocol.CALIB_DIAG_REASON_WRAP_TOO_LARGE: _spec(
        "编码器跨度接近整圈", ("encoder_span", "tight", "max", "motor_mask"),
        "检查磁编方向、映射和多圈锚点。",
    ),
    protocol.CALIB_DIAG_REASON_ABS_LIMIT_INVALID: _spec(
        "舵机目标超过安全绝对范围", ("command_abs", "min", "max", "motor_mask"),
        "确认多圈读数、舵机 ID、绳路和 ABS limit 后重新标定。",
    ),
    protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED: _spec(
        "标定目标发送失败", ("target_a", "target_b", "apply_status", "motor_mask"),
        "确认目标 owner、执行状态和舵机任务正常后重新标定。",
    ),
    protocol.CALIB_DIAG_REASON_FEEDBACK_LOST: _spec(
        "标定过程中反馈丢失",
        ("encoder_valid_mask", "servo_abs_valid_mask", "servo_raw_valid_mask", "affected_motor_mask"),
        "根据三个 valid mask 检查磁编、舵机 ABS/raw 反馈及对应总线。",
        kinds=("hex", "hex", "hex", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_TASK_START_FAILED: _spec(
        "标定任务启动失败", ("task_create_result", "", "", "selected_joint_mask"),
        "确认上一次标定已退出；若仍失败，检查任务栈和系统资源。",
    ),
    protocol.CALIB_DIAG_REASON_CONTROL_LOST: _spec(
        "标定控制权丢失", ("system_state", "owner", "control_enabled", "active_joint_mask"),
        "检查 STOP、Pause、Clear Fault、Enable 或其他控制流程是否打断标定。",
    ),
    protocol.CALIB_DIAG_REASON_HOLD_TARGET_FAILED: _spec(
        "保持其他关节失败", ("current_index", "joint_count", "batch_count", "selected_joint_mask"),
        "检查其他已选关节的舵机在线、Turn Known 和 ABS limit 状态。",
    ),
    protocol.CALIB_DIAG_REASON_JOINT_LIST_INVALID: _spec(
        "标定关节列表无效", ("joint_or_count", "max_joint_count", "index_or_has_list", "seen_joint_mask"),
        "重新选择 active scope；若重复出现，检查上位机与固件的关节数量。",
    ),
    protocol.CALIB_DIAG_REASON_DUPLICATE_JOINT: _spec(
        "标定关节重复", ("duplicate_index", "joint_count", "", "seen_joint_mask"),
        "重新选择关节并检查 active scope 到标定序列的构造。",
    ),
    protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED: _spec(
        "标定工作空间准备失败",
        ("helper_joint", "pose", "stage", "motor_mask"),
        "检查失败辅助关节的舵机在线、ABS/load 反馈和 Turn Known 状态。",
        formatter=_format_workspace,
    ),
    protocol.CALIB_DIAG_REASON_JOINT_STEP_FAILED: _spec(
        "部分关节标定失败", ("", "", "", ""),
        "这是调度层兜底；按 failed joint mask 重标，并查看此前固件日志中的具体根因。",
        formatter=_format_joint_step,
    ),
    protocol.CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT: _spec(
        "舵机热插拔恢复超时",
        ("online_mask", "configured_mask", "pending_mask", "unresolved_mask"),
        "检查 unresolved mask 对应舵机的电源、总线和多圈模式。",
        kinds=("hex", "hex", "hex", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS: _spec(
        "单电机关节松绳超过两圈",
        ("released_counts", "limit_counts", "next_released_counts", "motor_mask"),
        "检查绳索脱落、断裂、绕线方向或磁编未随机械运动。",
    ),
    protocol.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID: _spec(
        "关节与舵机映射无效", ("joint", "primary", "secondary", "motor_mask"),
        "检查 HardwareMap 中的 J/M、主副电机映射。",
    ),
    protocol.CALIB_DIAG_REASON_RESULT_SPAN_INVALID: _spec(
        "磁编有效行程无效", ("encoder_span", "endpoint_a", "endpoint_b", "motor_mask"),
        "检查磁编方向、机械端点以及 SPI/CAN 数据是否实时更新。",
    ),
    protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED: _spec(
        "标定结果或 LUT 构建失败", ("sample_count", "encoder_span", "lut_count", "motor_mask"),
        "检查 Manual 基准、端点、样本数量和磁编行程。",
    ),
    protocol.CALIB_DIAG_REASON_RESULT_COMMIT_FAILED: _spec(
        "标定结果提交失败", ("joint", "encoder_span", "lut_count", "motor_mask"),
        "检查关节索引、共享状态和运行时标定存储。",
    ),
    protocol.CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED: _spec(
        "舵机多圈可信状态提交失败", ("joint", "primary", "secondary", "motor_mask"),
        "检查对应舵机在线、多圈配置和恢复状态。",
    ),
    protocol.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT: _spec(
        "标定目标等待下位机执行确认超时",
        ("target_a", "target_b", "apply_status", "motor_mask"),
        "检查 ServoBus 是否阻塞、UART 是否被扫描占用以及 owner 是否仍为 Calibration。",
    ),
    protocol.CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED: _spec(
        "双腱换向阶段关节停止响应",
        ("load", "load_threshold", "encoder_delta", "motor_mask"),
        "检查主副腱预紧、收放方向、磁编反馈和机械卡滞。",
    ),
}


_LEGACY = {
    protocol.CALIB_DIAG_REASON_LEGACY_CAN_FEEDBACK_TIMEOUT: (
        "旧版 CAN feedback timeout", ("can_online", "sample_age_ms", "timeout_ms", "affected_joint_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_SERVO_FEEDBACK_TIMEOUT: (
        "旧版 servo feedback timeout", ("abs_ok", "raw_ok", "telemetry_ok", "affected_motor_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_TELEMETRY_TIMEOUT: (
        "旧版 telemetry timeout", ("abs_ok", "raw_ok", "telemetry_ok", "affected_motor_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_TURN_UNKNOWN: (
        "旧版 turn unknown", ("", "", "", "unknown_motor_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_LOAD_NOT_REACHED: (
        "旧版 load not reached", ("load", "threshold", "travel_counts", "motor_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_ANGLE_NOT_MOVING: (
        "旧版 angle not moving", ("angle_delta", "threshold", "stable_samples", "motor_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_WRONG_DIRECTION: (
        "旧版 wrong direction", ("actual_delta", "expected_sign", "threshold", "motor_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_ZERO_RETURN_TIMEOUT: (
        "旧版 zero return timeout", ("abs_error", "required_travel", "zero_deg_x100", "motor_mask")
    ),
    protocol.CALIB_DIAG_REASON_LEGACY_INTERNAL_ERROR: (
        "旧版 internal error", ("detail_a", "detail_b", "detail_c", "affected_mask")
    ),
}
for reason, (title, fields) in _LEGACY.items():
    CALIBRATION_DIAGNOSTICS[reason] = _spec(
        title,
        fields,
        "该诊断来自旧固件或历史日志；保留原始数据，并使用对应版本日志定位。",
        legacy=True,
    )


PHASE_NAMES = {
    protocol.CALIB_DIAG_PHASE_INIT: "初始化反馈检查",
    protocol.CALIB_DIAG_PHASE_FIND_MAX: "寻找最大拉紧位置",
    protocol.CALIB_DIAG_PHASE_RELEASE: "松绳/方向判断",
    protocol.CALIB_DIAG_PHASE_RETURN_ZERO: "回到零位",
    protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD: "双电机预紧",
    protocol.CALIB_DIAG_PHASE_DUAL_EXCHANGE: "双电机交替测试",
    protocol.CALIB_DIAG_PHASE_SEQUENCE: "标定任务调度",
}


def format_calibration_diagnostic(state: object) -> dict:
    valid = bool(getattr(state, "calib_diag_valid", False))
    reason = int(getattr(state, "calib_diag_reason", 0) or 0)
    phase = int(getattr(state, "calib_diag_phase", 0) or 0)
    joint = int(getattr(state, "calib_diag_joint", 0xFF) or 0)
    motor = int(getattr(state, "calib_diag_motor", 0xFF) or 0)
    values = tuple(
        int(getattr(state, f"calib_diag_value_{suffix}", 0) or 0)
        for suffix in ("a", "b", "c")
    )
    bitmap = int(getattr(state, "calib_diag_bitmap", 0) or 0)
    timestamp_ms = int(getattr(state, "calib_diag_timestamp_ms", 0) or 0)
    sequence = int(getattr(state, "calib_diag_sequence", 0) or 0)
    completed = int(getattr(state, "calib_diag_completed_joint_mask", 0) or 0)
    failed = int(getattr(state, "calib_diag_failed_joint_mask", 0) or 0)
    active = valid and reason != protocol.CALIB_DIAG_REASON_NONE
    spec = CALIBRATION_DIAGNOSTICS.get(reason)
    title = spec.title if active and spec else (f"标定诊断 reason={reason}" if active else "")
    lines = []
    suggestions = []
    if active:
        phase_text = PHASE_NAMES.get(phase, f"phase={phase}" if phase else "-")
        joint_text = f"J{joint:02d}" if 0 <= joint < protocol.ENCODER_COUNT else "J?"
        motor_text = f"M{motor:02d}" if 0 <= motor < protocol.MOTOR_COUNT else "M?"
        lines.extend((f"标定失败原因：{title}", f"阶段：{phase_text}", f"对象：{joint_text}, {motor_text}"))
        if completed or failed:
            lines.append(f"本轮结果：completed={_hex(completed)}, failed={_hex(failed)}")
        if spec:
            lines.append(f"数据：{_format_from_spec(spec, values, bitmap)}")
            suggestions.append(spec.suggestion)
        else:
            lines.append(
                f"数据：valueA={values[0]}, valueB={values[1]}, valueC={values[2]}, bitmap={_hex(bitmap)}"
            )
            suggestions.append("保留当前日志并按对应固件版本核对诊断协议。")
    return {
        "active": active,
        "title": title,
        "lines": lines,
        "suggestions": suggestions,
        "key": f"{sequence}:{reason}:{phase}:{joint}:{motor}:{values[0]}:{values[1]}:{values[2]}:{bitmap:08X}:{timestamp_ms}",
    }
