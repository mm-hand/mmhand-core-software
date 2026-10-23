"""自动标定诊断协议目录与用户可读格式化。

职责：将标定诊断和装配报警转换为用户提示。
线程：调用者线程执行，不创建线程。
输入：只读设备快照和诊断编号。
输出：问题原因、数值和下一步操作。
状态所有权：仅拥有静态诊断目录。
禁止依赖：禁止依赖 GUI、串口或设备控制实现。
"""

from dataclasses import dataclass
from typing import Callable, Dict, Tuple

from . import protocol
from .safety_rules import explain_rope_init_reason


Formatter = Callable[[Tuple[int, int, int], int], str]


@dataclass(frozen=True)
class CalibrationDiagnosticSpec:
    title: str
    fields: Tuple[str, str, str, str]
    formatter: Formatter
    suggestion: str


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
    return "uses completed/failed joint masks only; no specific mechanical cause was reported"


def _spec(
    title: str,
    fields: Tuple[str, str, str, str],
    suggestion: str,
    *,
    kinds: Tuple[str, str, str, str] = ("int", "int", "int", "hex"),
    formatter: Formatter | None = None,
) -> CalibrationDiagnosticSpec:
    return CalibrationDiagnosticSpec(
        title, fields, formatter or _fields_formatter(fields, kinds), suggestion
    )


CALIBRATION_DIAGNOSTICS: Dict[int, CalibrationDiagnosticSpec] = {
    protocol.CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY: _spec(
        "No active joints selected for calibration",
        ("active_joint_mask", "active_motor_mask", "", ""),
        "Select the required joints and motors in Active Scope, then retry.",
        kinds=("hex", "hex", "int", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_ENCODER_ERROR: _spec(
        "Invalid magnetic encoder feedback",
        ("encoder_valid_mask", "online_joint_mask", "fault_joint_mask", "affected_joint_mask"),
        "Check encoder power, SPI/CAN wiring, S3/P4 communication, and the sensor board.",
        kinds=("hex", "hex", "hex", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_SERVO_OFFLINE: _spec(
        "Servo feedback missing",
        ("online_motor_mask", "configured_motor_mask", "", "affected_motor_mask"),
        "Check servo power, UART bus, ID mapping, and wiring.",
        kinds=("hex", "hex", "int", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_OVERLOAD: _spec(
        "Overload during calibration", ("load", "limit", "raw_load", "motor_mask"),
        "Remove external load, check for an over-tight tendon or mechanical jam, clear the fault, and recalibrate.",
    ),
    protocol.CALIB_DIAG_REASON_ZERO_CROSS_INVALID: _spec(
        "Invalid zero-position estimate", ("taut", "zero", "tight", "motor_mask"),
        "Check encoder direction, the calibration start pose, and tendon preload.",
    ),
    protocol.CALIB_DIAG_REASON_WRAP_TOO_LARGE: _spec(
        "Encoder span is near a full turn", ("encoder_span", "tight", "max", "motor_mask"),
        "Check magnetic encoder direction, mapping, and the multi-turn anchor.",
    ),
    protocol.CALIB_DIAG_REASON_ABS_LIMIT_INVALID: _spec(
        "Servo target exceeds the safe absolute range", ("command_abs", "min", "max", "motor_mask"),
        "Verify the multi-turn reading, servo ID, tendon routing, and ABS limits before recalibrating.",
    ),
    protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED: _spec(
        "Calibration target publish failed", ("target_a", "target_b", "apply_status", "motor_mask"),
        "Verify target ownership, apply status, and the servo task before recalibrating.",
    ),
    protocol.CALIB_DIAG_REASON_FEEDBACK_LOST: _spec(
        "Feedback lost during calibration",
        ("encoder_valid_mask", "servo_abs_valid_mask", "servo_raw_valid_mask", "affected_motor_mask"),
        "Use the three valid masks to check encoder, servo ABS/raw feedback, and the affected buses.",
        kinds=("hex", "hex", "hex", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_TASK_START_FAILED: _spec(
        "Calibration task failed to start", ("task_create_result", "", "", "selected_joint_mask"),
        "Confirm the previous calibration has exited; if it still fails, check task stack and system resources.",
    ),
    protocol.CALIB_DIAG_REASON_CONTROL_LOST: _spec(
        "Calibration control ownership lost", ("system_state", "owner", "control_enabled", "active_joint_mask"),
        "Check whether STOP, Clear Faults, Enable, or another control flow interrupted calibration.",
    ),
    protocol.CALIB_DIAG_REASON_HOLD_TARGET_FAILED: _spec(
        "Failed to hold other joints", ("current_index", "joint_count", "batch_count", "selected_joint_mask"),
        "Check servo online state, Turn Known state, and ABS limits for the other selected joints.",
    ),
    protocol.CALIB_DIAG_REASON_JOINT_LIST_INVALID: _spec(
        "Invalid calibration joint list", ("joint_or_count", "max_joint_count", "index_or_has_list", "seen_joint_mask"),
        "Select the active scope again; if this repeats, check desktop and firmware joint counts.",
    ),
    protocol.CALIB_DIAG_REASON_DUPLICATE_JOINT: _spec(
        "Duplicate joint in calibration list", ("duplicate_index", "joint_count", "", "seen_joint_mask"),
        "Reselect the joints and check active-scope construction of the calibration sequence.",
    ),
    protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED: _spec(
        "Calibration workspace preparation failed",
        ("helper_joint", "pose", "stage", "motor_mask"),
        "Check servo online state, ABS/load feedback, and Turn Known state for the helper joint.",
        formatter=_format_workspace,
    ),
    protocol.CALIB_DIAG_REASON_JOINT_STEP_FAILED: _spec(
        "One or more joints failed calibration", ("", "", "", ""),
        "This is a scheduler fallback. Recalibrate the failed joint mask and inspect the preceding firmware log for the specific cause.",
        formatter=_format_joint_step,
    ),
    protocol.CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT: _spec(
        "Servo hot-plug recovery timed out",
        ("online_mask", "configured_mask", "pending_mask", "unresolved_mask"),
        "Check power, bus communication, and multi-turn mode for the unresolved motor mask.",
        kinds=("hex", "hex", "hex", "hex"),
    ),
    protocol.CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS: _spec(
        "Single-tendon release exceeded two turns",
        ("released_counts", "limit_counts", "next_released_counts", "motor_mask"),
        "Check for a detached or broken tendon, wrong winding direction, or encoder motion mismatch.",
    ),
    protocol.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID: _spec(
        "Invalid joint-to-servo mapping", ("joint", "primary", "secondary", "motor_mask"),
        "Check the J/M and primary/secondary motor mapping in HardwareMap.",
    ),
    protocol.CALIB_DIAG_REASON_RESULT_SPAN_INVALID: _spec(
        "Invalid magnetic encoder travel", ("encoder_span", "endpoint_a", "endpoint_b", "motor_mask"),
        "Check encoder direction, mechanical endpoints, and whether SPI/CAN data updates in real time.",
    ),
    protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED: _spec(
        "Calibration result or LUT build failed", ("sample_count", "encoder_span", "lut_count", "motor_mask"),
        "Check Manual references, endpoints, sample count, and encoder travel.",
    ),
    protocol.CALIB_DIAG_REASON_RESULT_COMMIT_FAILED: _spec(
        "Calibration result commit failed", ("joint", "encoder_span", "lut_count", "motor_mask"),
        "Check the joint index, shared state, and runtime calibration storage.",
    ),
    protocol.CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED: _spec(
        "Failed to commit trusted multi-turn state", ("joint", "primary", "secondary", "motor_mask"),
        "Check servo online state, multi-turn configuration, and recovery state.",
    ),
    protocol.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT: _spec(
        "Timed out waiting for calibration target confirmation",
        ("target_a", "target_b", "apply_status", "motor_mask"),
        "Check whether ServoBus is blocked, UART is occupied by scanning, and the owner is still Calibration.",
    ),
    protocol.CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED: _spec(
        "Joint stalled during dual-tendon exchange",
        ("load", "load_threshold", "encoder_delta", "motor_mask"),
        "Check primary/secondary tendon preload, winding directions, encoder feedback, and mechanical binding.",
    ),
}


PHASE_NAMES = {
    protocol.CALIB_DIAG_PHASE_INIT: "Initial feedback check",
    protocol.CALIB_DIAG_PHASE_FIND_MAX: "Find maximum tightened position",
    protocol.CALIB_DIAG_PHASE_RELEASE: "Tendon release and direction check",
    protocol.CALIB_DIAG_PHASE_RETURN_ZERO: "Return to zero",
    protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD: "Dual-motor preload",
    protocol.CALIB_DIAG_PHASE_DUAL_EXCHANGE: "Dual-motor exchange test",
    protocol.CALIB_DIAG_PHASE_SEQUENCE: "Calibration sequence",
}


def format_calibration_diagnostic(diag) -> dict:
    """把本轮标定失败记录 (reason, phase, joint, motor, a, b, c, bitmap) 变成可读提示。

    ``diag`` 为 None 表示本轮没有失败记录（未开始或已成功）。
    """
    if diag is None:
        reason, phase, joint, motor, values, bitmap = 0, 0, 0, 0, (0, 0, 0), 0
    else:
        reason, phase, joint, motor, a, b, c, bitmap = diag
        values = (a, b, c)
    active = diag is not None and reason != protocol.CALIB_DIAG_REASON_NONE
    spec = CALIBRATION_DIAGNOSTICS.get(reason)
    title = spec.title if active and spec else (f"Unknown calibration reason: {reason}" if active else "")
    lines = []
    suggestions = []
    if active:
        phase_text = PHASE_NAMES.get(phase, f"phase={phase}" if phase else "-")
        joint_text = f"J{joint:02d}" if 0 <= joint < protocol.ENCODER_COUNT else "J?"
        motor_text = f"M{motor:02d}" if 0 <= motor < protocol.MOTOR_COUNT else "M?"
        lines.extend((f"Cause: {title}", f"Phase: {phase_text}", f"Affected: {joint_text}, {motor_text}"))
        if spec:
            lines.append(f"Raw data: {_format_from_spec(spec, values, bitmap)}")
            suggestions.append(spec.suggestion)
        else:
            lines.append(
                f"Raw data: reason={reason}, valueA={values[0]}, valueB={values[1]}, "
                f"valueC={values[2]}, bitmap={_hex(bitmap)}"
            )
            suggestions.append("Preserve the current log and check the diagnostic protocol for the matching firmware version.")
    return {
        "active": active,
        "title": title,
        "lines": lines,
        "suggestions": suggestions,
        "key": f"{reason}:{phase}:{joint}:{motor}:{values[0]}:{values[1]}:{values[2]}:{bitmap:08X}",
    }


def format_calibration_warning(state: object, warning_mask: int) -> dict:
    """Format calibration assembly warning bits for UI display."""
    mask = int(warning_mask) & ((1 << protocol.ENCODER_COUNT) - 1)
    min_offsets = list(getattr(state, "calibration_auto_min_offset_counts", []) or [])
    results = list(getattr(state, "joint_calib_results", []) or [])
    sections = []
    event_items = []
    for joint in range(protocol.ENCODER_COUNT):
        if not (mask & (1 << joint)):
            continue
        result = results[joint] if joint < len(results) else None
        if not isinstance(result, dict):
            return {"ready": False}
        reason = int(result.get("assemblyWarningReason", 0) or 0)
        min_offset = int(min_offsets[joint]) if joint < len(min_offsets) else 0
        max_shortfall = auto_span = manual_span = urdf_span = span_difference = 0
        details = []
        return_cause = ('Possible cause: excessive tendon friction.'
                        if joint in protocol.DUAL_TENDON_JOINT_INDICES
                        else 'Possible cause: weak spring return.')
        if bool(result.get("hasAssemblyWarningFields", False)):
            min_offset = int(result.get("autoMinOffsetCounts", 0) or 0)
            max_shortfall = int(result.get("autoMaxShortfallCounts", 0) or 0)
            auto_span = int(result.get("assemblyAutoSpanCounts", 0) or 0)
            manual_span = int(result.get("assemblyManualSpanCounts", 0) or 0)
            urdf_span = int(result.get("assemblyUrdfSpanCounts", 0) or 0)
            span_difference = int(result.get("assemblySpanDifferenceCounts", 0) or 0)
            if reason & protocol.CALIB_WARN_AUTO_MIN_OFFSET:
                details.append(
                    f"Auto Min is {min_offset} counts inside Manual Min (limit: 350 counts). {return_cause}"
                )
            if reason & protocol.CALIB_WARN_AUTO_MAX_SHORTFALL:
                details.append(
                    f"Auto Max remains {max_shortfall} counts below Manual Max (limit: 300 counts). Possible cause: excessive tendon friction."
                )
            if reason & protocol.CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH:
                if bool(result.get("hasAssemblySpanComparisonFields", False)):
                    details.append(
                        f"Manual span={manual_span} counts, URDF span={urdf_span} counts, "
                        f"difference={span_difference} counts (limit: 360 counts). Possible magnetic encoder or joint installation warning."
                    )
                else:
                    details.append(f"Legacy firmware reported an Auto span warning: {auto_span} counts.")
        else:
            reason = protocol.CALIB_WARN_AUTO_MIN_OFFSET
            details.append(
                f"Auto Min is {min_offset} counts inside Manual Min (limit: 350 counts). {return_cause}"
            )
        known = (
            protocol.CALIB_WARN_AUTO_MIN_OFFSET
            | protocol.CALIB_WARN_AUTO_MAX_SHORTFALL
            | protocol.CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH
        )
        if reason & ~known:
            details.append(f"Unknown calibration warning bits: 0x{reason & ~known:02X}.")
        if not details:
            details.append(f"P4 reported warning bits 0x{reason:02X} without recognized detail.")
        sections.append(f"J{joint:02d}:\n" + "\n".join(f"- {item}" for item in details))
        event_items.append(
            f"J{joint:02d}:reason=0x{reason:02X},minOffset={min_offset},"
            f"maxShortfall={max_shortfall},autoSpan={auto_span},manualSpan={manual_span},"
            f"urdfSpan={urdf_span},spanDifference={span_difference}"
        )
    return {
        "ready": True,
        "title": "Calibration Assembly Warning",
        "message": (
            "Calibration was saved successfully, but possible finger assembly offsets were detected:\n\n"
            + "\n\n".join(sections)
            + "\n\nThis warning does not block READY or feedback control.\n"
            "Check tendon routing, assembly position, and mechanical endpoints for the affected joints."
        ),
        "event_detail": ", ".join(event_items),
    }


def format_calibration_batch_summary(records: list[dict]) -> dict:
    """Combine per-phase calibration warnings into one final UI message."""
    sections = []
    seen = set()
    for record in records:
        if not isinstance(record, dict):
            continue
        details = []
        summary = record.get("warning_summary")
        if isinstance(summary, dict) and summary.get("ready"):
            details.append(str(summary.get("message", "")))
        rope_reason = int(record.get("rope_init_reason", 0) or 0)
        if str(record.get("status", "")) == "WARN" and rope_reason:
            details.append(
                "Automatic Rope Init warning:\n- "
                + explain_rope_init_reason(rope_reason)
            )
        key = "\n".join(details)
        if not details or key in seen:
            continue
        seen.add(key)
        phase_index = int(record.get("phase_index", 0)) + 1
        phase_count = int(record.get("phase_count", 1))
        sections.append(f"Phase {phase_index}/{phase_count}:\n" + "\n\n".join(details))
    return {
        "active": bool(sections),
        "title": "Calibration Completed with Warnings",
        "message": "\n\n".join(sections),
    }
