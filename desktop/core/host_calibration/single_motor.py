"""single_motor.py - 单腱关节:拉到 Max,放回 Min,构建结果。

职责：AA 基准摆位、两次端点搜索、V5 结果与 LUT、ABS 限位。
线程：由校准流程的工作线程串行运行；不创建线程。
输入：校准 IO 的当前反馈、运动参数和选中关节序列。
输出：完成/失败结果、原 V5 LUT 和本地 ABS 限位。
状态所有权：本模块只拥有单关节轨迹；端点判定由 sweep.py 完成。
禁止依赖：禁止依赖 NiceGUI/UI、串口实现、ROS2 或 Replay。

The old firmware state machine (tighten / pre-release sync / release counters)
is gone. A single-tendon joint is now two endpoint sweeps: wind to URDF Max,
then pay out and let the spring return it to URDF Min. Both endpoints are the
absolute joint angle freezing near the commanded end.
"""

from .. import protocol
from . import config
from .math import (
    TendonRawFitBuffer,
    apply_contact_refinement,
    build_calibration_result,
    build_tendon_lut,
    forward_encoder_delta,
    tendon_motor_abs_limits,
    tendon_motor_abs_range,
)
from .sweep import END_MAX, END_MIN, sweep_to_end
from .time_alignment import TimeAlignmentError


def _diag(io, reason, phase, joint, motor=255, a=0, b=0, c=0, bitmap=0):
    emit = getattr(io, 'diag_if_empty', None) or io.diag
    emit(reason, phase, joint, motor, a, b, c, bitmap)


def _fail(io, reason, phase, joint, motor=255, a=0, b=0, c=0, bitmap=0):
    _diag(io, reason, phase, joint, motor, a, b, c, bitmap)
    # 失去控制权要让整批停下；单关节失败只影响它自己。
    return {"success": False, "aborted": not io.owns_control(), "reason": reason,
            "result": None, "limits": {}}


def run_single(io, joint: int, params: config.AutoCalibrationParams,
               ordered_joints, failed_joint_mask: int, sequence_index: int,
               selected_joint_mask: int) -> dict:
    """Serial single-tendon calibration; caller handles the Manual/LUT commit."""
    prepared, message = io.prepare_finger_aa(joint)
    if not prepared:
        # 摆动基准摆不过去：本指剩下的关节也标不了，但不要拖累其他手指。
        return {"success": False, "aborted": False, "finger_blocked": True,
                "reason": message, "result": None, "limits": {}}
    motors = config.joint_motors(joint)
    if len(motors) != 1:
        return _fail(io, protocol.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                     protocol.CALIB_DIAG_PHASE_INIT, joint, 255, joint, len(motors), -1)
    motor = motors[0]
    setter = getattr(io, 'set_active_calibration_motors', None)
    if callable(setter):
        setter(1 << motor)
    io.clear_limits(joint)
    io.trace("SEQUENCE_SERIAL_BEGIN", joint, 255, sequence_index=sequence_index,
             joint_count=len(ordered_joints))

    def hold():
        return io.hold_targets_except(ordered_joints, joint, failed_joint_mask, params)

    tight = sweep_to_end(io, joint, END_MAX, motors=motors, pull=motor, release=None,
                         hold=hold, params=params,
                         endpoint_load=config.kTautLoadCounts,
                         timeout_ms=config.kMaxSweepTimeoutMs)
    if tight.point is None or tight.point.primary_abs is None:
        return _fail(io, tight.reason or protocol.CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                     protocol.CALIB_DIAG_PHASE_FIND_MAX, joint, motor)

    fit = TendonRawFitBuffer()
    window = None
    begin_sampling = getattr(io, "begin_lut_sampling", None)
    if callable(begin_sampling):
        try:
            window = begin_sampling(joint, motors)
            if window is None:
                raise TimeAlignmentError("time-aligned LUT sampling window is missing")
        except TimeAlignmentError as exc:
            io.trace("LUT_TIME_FAILED", joint, motor, reason=str(exc))
            return _fail(io, protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                         protocol.CALIB_DIAG_PHASE_FIND_MIN, joint, motor)
    else:
        fit.append(tight.point.encoder, tight.point.primary_abs, 0)
        fit.next_travel_deg = config.kTendonFitSampleStepDeg

    limit = sweep_to_end(io, joint, END_MIN, motors=motors, pull=None, release=motor,
                         fit=None if window is not None else fit, hold=hold, params=params,
                         near_deg=config.kSpringReturnToleranceDeg,
                         timeout_ms=config.kMinSweepTimeoutMs)
    if limit.point is None or limit.point.primary_abs is None:
        return _fail(io, limit.reason or protocol.CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                     protocol.CALIB_DIAG_PHASE_FIND_MIN, joint, motor)
    if window is not None:
        try:
            fit = io.finish_lut_sampling(window)
        except TimeAlignmentError as exc:
            io.trace("LUT_TIME_FAILED", joint, motor, reason=str(exc))
            return _fail(io, protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                         protocol.CALIB_DIAG_PHASE_FIND_MIN, joint, motor)
    else:
        fit.append(limit.point.encoder, limit.point.primary_abs, 0)

    span = forward_encoder_delta(limit.point.encoder, tight.point.encoder)
    io.trace("SINGLE_SPAN_CHECK", joint, motor, tight_encoder=tight.point.encoder,
             limit_encoder=limit.point.encoder, span=span,
             tight_deg=round(tight.point.angle, 3), limit_deg=round(limit.point.angle, 3))
    zero_abs = config.motor_apply_tendon_delta(motor, limit.point.primary_abs,
                                               config.kServoZeroPullCounts)
    result = build_calibration_result(io, joint, limit.point.encoder, 0, tight.point.encoder,
                                      limit.point.primary_abs, zero_abs, tight.point.primary_abs)
    if result is None:
        return _fail(io, protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                     protocol.CALIB_DIAG_PHASE_FIND_MIN, joint, motor,
                     fit.count, span, 0, 1 << motor)
    if not build_tendon_lut(io, joint, result, motor, 255, 1, fit):
        return _fail(io, protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                     protocol.CALIB_DIAG_PHASE_FIND_MIN, joint, motor,
                     fit.count, result["encoderSpan"], 0, 1 << motor)
    apply_contact_refinement(io, joint, result)
    if result["tendonLutMotor"][0] != motor or tendon_motor_abs_range(result, 0) is None:
        return _fail(io, protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                     protocol.CALIB_DIAG_PHASE_FIND_MIN, joint, motor,
                     result["tendonLutCount"], result["tendonLutMotor"][0], 0, 1 << motor)
    limit_abs = tendon_motor_abs_limits(
        result, 0, motor, *config.calibration_margins(io))
    if limit_abs is None:
        return _fail(io, protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                     protocol.CALIB_DIAG_PHASE_FIND_MIN, joint, motor,
                     result["tendonLutCount"], result["encoderSpan"], 0, 1 << motor)
    io.set_limit(motor, *limit_abs)
    io.trace("SINGLE_RESULT", joint, motor, span=span,
             tight_encoder=tight.point.encoder, limit_encoder=limit.point.encoder,
             tight_deg=round(tight.point.angle, 3), limit_deg=round(limit.point.angle, 3),
             tight_load=tight.point.load, limit_load=limit.point.load,
             lut_samples=result["tendonLutCount"], raw_samples=fit.count)
    io.trace("SEQUENCE_SERIAL_DONE", joint, motor)
    return {"success": True, "aborted": False, "reason": "",
            "result": result, "limits": {motor: limit_abs}}
