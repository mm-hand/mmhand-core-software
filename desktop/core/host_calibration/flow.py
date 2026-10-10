"""flow.py - 与 P4 对应的自动标定会话和串行关节调度。

职责：复制反馈预检、机械顺序、单/双关节失败策略及 Rope Init 收尾。
线程：由 calibration_workflow 的单工作线程调用；不创建线程。
输入：不依赖串口/UI 的校准 IO、active scope 和原固件参数。
输出：逐关节提交结果、部分成功位图和会话终态。
状态所有权：仅拥有本轮关节顺序、完成/失败位图；实时反馈归 IO。
禁止依赖：禁止依赖 Qt/UI、HandController、串口对象或文件持久化。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from ..motor_mapping import MOTOR_TO_JOINT_INDEX
from ..protocol import (
    CALIB_DIAG_PHASE_SEQUENCE,
    CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
    CALIB_DIAG_REASON_RESULT_COMMIT_FAILED,
    DUAL_TENDON_JOINT_INDICES,
    ENCODER_COUNT,
    VALID_MOTOR_MASK,
)
from .config import default_auto_calibration_params
from .dual_motor import (
    build_dynamic_workspace_motor_mask,
    invalidate_dual_workspace_cache_joint,
    reset_dual_workspace_cache,
    run_dual,
    prepare_single_workspace,
)
from .single_motor import run_single


@dataclass(frozen=True)
class CalibrationSessionResult:
    success: bool
    completed_joint_mask: int = 0
    failed_joint_mask: int = 0
    ordered_joints: tuple[int, ...] = ()
    records: tuple[dict, ...] = field(default_factory=tuple)
    message: str = ""
    terminal_status: str = "FAILED"
    cancelled: bool = False


def calibration_joint_motor_mask(joint: int) -> int:
    return sum(
        1 << motor
        for motor, mapped in enumerate(MOTOR_TO_JOINT_INDEX)
        if int(mapped) == int(joint)
    ) & VALID_MOTOR_MASK


def auto_calibration_sequence_rank(joint: int) -> int:
    """CalibrationSequencePlanner.cpp 的原机械顺序。"""
    if joint < 16:
        return joint
    return {20: 16, 16: 17, 17: 18, 18: 19, 19: 20}.get(joint, joint)


def selected_joints(joint_mask: int, motor_mask: int) -> tuple[int, ...]:
    active = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
    if not active:
        for joint in range(ENCODER_COUNT):
            if calibration_joint_motor_mask(joint) & int(motor_mask):
                active |= 1 << joint
    return tuple(sorted(
        (joint for joint in range(ENCODER_COUNT) if active & (1 << joint)),
        key=auto_calibration_sequence_rank,
    ))


def _accepted(value) -> tuple[bool, str]:
    if isinstance(value, tuple):
        return bool(value[0]), str(value[1]) if len(value) > 1 else ""
    return bool(value), ""


def _servo_recovery_message(io, message):
    detail = getattr(io, "feedback_recovery_error", "")
    return f"{message}: {detail}" if detail else message


def _is_cancelled(io) -> bool:
    predicate = getattr(io, "is_cancelled", None)
    return bool(predicate()) if callable(predicate) else False


def run(io, joint_mask: int, motor_mask: int,
        on_joint: Callable[[int, dict], None], params=None) -> CalibrationSessionResult:
    """按固件原策略顺序运行；成功关节立即写回，后续失败不撤销它。"""
    joints = selected_joints(joint_mask, motor_mask)
    selected_mask = sum(1 << joint for joint in joints)
    selected_motors = 0
    for joint in joints:
        selected_motors |= calibration_joint_motor_mask(joint)
    completed = failed = 0
    records = []
    success = False
    message = ""
    cancelled = False
    terminal_status = "FAILED"
    reset_dual_workspace_cache(io)
    io.set_workspace_read_mask(0)
    io.session_state("PENDING", progress=0, joint=0xFF, completed=0, failed=0)
    try:
        if not joints or not selected_motors:
            io.diag(CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
                    CALIB_DIAG_PHASE_SEQUENCE, 0xFF, 0xFF,
                    int(joint_mask), int(motor_mask), 0, 0)
            message = "active scope has no joints"
        elif not io.wait_servo_recovery(selected_motors):
            message = _servo_recovery_message(io, "selected servo feedback recovery failed")
        elif not io.wait_encoder_recovery(selected_mask):
            message = "selected encoder feedback recovery failed"
        else:
            workspace_motors = build_dynamic_workspace_motor_mask(io, joints)
            if workspace_motors is None:
                message = "workspace feedback preflight failed"
            else:
                io.set_workspace_read_mask(workspace_motors)
                if not io.wait_servo_recovery(selected_motors | workspace_motors):
                    message = _servo_recovery_message(io, "workspace servo feedback recovery failed")
                else:
                    io.session_state("RUNNING", progress=0, joint=0xFF,
                                     completed=0, failed=0)
                    for index, joint in enumerate(joints):
                        joint_params = (params if params is not None else
                                        default_auto_calibration_params(joint))
                        if not io.owns_control():
                            cancelled = True
                            message = "calibration cancelled or control lost"
                            break
                        joint_motors = calibration_joint_motor_mask(joint)
                        io.session_state("RUNNING", progress=index * 100 // len(joints),
                                         joint=joint, completed=completed, failed=failed)
                        prepared, prepare_message = _accepted(
                            io.prepare_joint(joint, joint_motors)
                        )
                        if not prepared:
                            outcome = {
                                "success": False, "aborted": True,
                                "cancelled": _is_cancelled(io),
                                "reason": prepare_message or "joint ABS zero failed",
                                "result": None, "limits": {},
                            }
                        else:
                            # 当前关节原点改变后，不能使用此前辅助动作的旧 ABS cache。
                            invalidate_dual_workspace_cache_joint(io, joint)
                            if joint in DUAL_TENDON_JOINT_INDICES:
                                outcome = run_dual(
                                    io, joint, index, len(joints), selected_mask, joint_params
                                )
                            else:
                                workspace_ok = prepare_single_workspace(io, joint, joint_params)
                                outcome = run_single(
                                    io, joint, joint_params, joints, failed, index,
                                    selected_mask,
                                ) if workspace_ok else {
                                    'success': False, 'aborted': True,
                                    'reason': 'Workspace preparation failed; the batch was stopped', 'result': None, 'limits': {},
                                }
                            outcome = dict(outcome)
                            if bool(outcome.get("success")):
                                committed, commit_message = _accepted(io.commit_joint(
                                    joint, outcome.get("result"), outcome.get("limits", {})
                                ))
                                if not committed:
                                    if not _is_cancelled(io):
                                        io.diag(CALIB_DIAG_REASON_RESULT_COMMIT_FAILED,
                                                CALIB_DIAG_PHASE_SEQUENCE, joint, 0xFF,
                                                0, 0, 0, joint_motors)
                                    outcome.update(success=False, aborted=True,
                                                   reason=commit_message or
                                                   "joint calibration commit failed")
                                else:
                                    outcome["commit_message"] = commit_message
                        if bool(outcome.get("success")):
                            completed |= 1 << joint
                        else:
                            cancelled = _is_cancelled(io) or bool(outcome.get("cancelled"))
                            if cancelled:
                                outcome.update(aborted=True, cancelled=True)
                            else:
                                failed |= 1 << joint
                            message = str(outcome.get("reason") or
                                          f"J{joint:02d} calibration failed")
                            if cancelled:
                                message = f"J{joint:02d}: {message}"
                            elif hasattr(io, "failure_message"):
                                message = f"J{joint:02d}: " + io.failure_message(
                                    outcome.get("reason") or message)
                        outcome.update(
                            joint=joint, sequence_index=index,
                            sequence_count=len(joints), motor_mask=joint_motors,
                            completed_joint_mask=completed, failed_joint_mask=failed,
                            has_next_joint=(index + 1 < len(joints) and
                                not bool(outcome.get("aborted")) and
                                not (joint in DUAL_TENDON_JOINT_INDICES and
                                     not bool(outcome.get("success")))),
                        )
                        records.append(outcome)
                        io.session_state("RUNNING", progress=(index + 1) * 100 // len(joints),
                                         joint=joint, completed=completed, failed=failed)
                        on_joint(joint, outcome)
                        if (bool(outcome.get("aborted")) or
                            (joint in DUAL_TENDON_JOINT_INDICES and
                             not bool(outcome.get("success")))):
                            break
                    success = completed == selected_mask and failed == 0 and not cancelled
                    if success:
                        message = "all calibration joints completed"
        if _is_cancelled(io):
            cancelled = True
            success = False
            if "cancelled" not in message.lower():
                message = f"calibration cancelled: {message}" if message else "calibration cancelled"
        if not success and failed and not cancelled:
            # 具体状态机的诊断归 IO 保存，汇总不覆盖首个真实根因。
            if hasattr(io, "diag_if_empty"):
                io.diag_if_empty(CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                                 CALIB_DIAG_PHASE_SEQUENCE, 0xFF, 0xFF,
                                 0, 0, 0, 0)
    finally:
        io.set_workspace_read_mask(0)
        finish = io.finish_session(success, completed, failed)
        if success:
            ready, finish_message = _accepted(finish)
            has_warning = any(bool(record.get("warning")) or
                              bool((record.get("result") or {}).get("assemblyWarningReason"))
                              for record in records)
            terminal_status = "SUCCESS" if ready and not has_warning else "WARN"
            if finish_message:
                message += "; " + finish_message
        io.session_state(terminal_status, progress=100 if success else
                         len(records) * 100 // max(1, len(joints)),
                         joint=0xFF, completed=completed, failed=failed,
                         cancelled=cancelled)
    return CalibrationSessionResult(success, completed, failed, joints,
                                    tuple(records), message, terminal_status, cancelled)
