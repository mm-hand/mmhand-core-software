"""finish.py - 上位机执行旧标定 Rope Init 之后的双腱 LUT 零度收尾。

职责：使用本轮宿主结果计算零度目标，通过普通直控完成原收尾动作。
线程：由校准工作线程串行调用，不创建线程。
输入：校准 IO、本轮已完成关节掩码和宿主生成的 LUT。
输出：普通直控目标与收尾成功或失败消息。
状态所有权：只拥有本次收尾计划，实际控制状态和反馈由 IO 提供。
禁止依赖：禁止依赖 Qt/UI、串口对象和 P4 专用校准协议。
"""

from __future__ import annotations

import math

from ..protocol import DUAL_TENDON_JOINT_INDICES, ENCODER_COUNT, TENDON_LUT_MAX_SAMPLES
from .config import default_auto_calibration_params, joint_motors
from .math import f32, round_to_int32


_STEP_PERIOD_MS = 50
_SERVO_ACC = 254


def _zero_targets(result, motors):
    """镜像 TendonCalibrationLut.cpp 的单精度插值与 lroundf。"""
    if not isinstance(result, dict):
        return None
    try:
        count = int(result.get('tendonLutCount', 0))
        rows = result.get('tendonLutMotorAbs', [])
        if (not result.get('success') or not result.get('scaled') or
                not result.get('tendonLutValid') or
                int(result.get('mappingVersion', 0)) != 4 or
                not 3 <= count <= TENDON_LUT_MAX_SAMPLES or
                tuple(result.get('tendonLutMotor', ())) != tuple(motors) or
                len(rows) != 2 or any(len(row) < count for row in rows)):
            return None
        first = f32(result['tendonLutAngleStartDeg'])
        step = f32(result['tendonLutAngleStepDeg'])
        current_max = f32(result.get('currentMaxDeg', float('nan')))
        if not math.isfinite(first) or not math.isfinite(step) or step <= 0:
            return None

        def sample_angle(index):
            nominal = f32(first + f32(step * f32(index)))
            if (index == count - 1 and math.isfinite(current_max) and
                    first < current_max < nominal):
                return current_max
            return nominal

        last = sample_angle(count - 1)
        if not math.isfinite(last) or not first <= 0.0 <= last or last <= first:
            return None
        upper_index = 1
        if 0.0 >= last:
            upper_index = count - 1
        elif 0.0 > first:
            while upper_index < count - 1 and 0.0 > sample_angle(upper_index):
                upper_index += 1
        lower_index = upper_index - 1
        lower_deg, upper_deg = sample_angle(lower_index), sample_angle(upper_index)
        if upper_deg <= lower_deg:
            return None
        fraction = f32(f32(-lower_deg) / f32(upper_deg - lower_deg))
        targets = {}
        for actuator, motor in enumerate(motors):
            lower, upper = f32(rows[actuator][lower_index]), f32(rows[actuator][upper_index])
            target = round_to_int32(f32(lower + f32(f32(upper - lower) * fraction)))
            if not -0x80000000 <= target <= 0x7FFFFFFF:
                return None
            targets[motor] = target
        return targets
    except (KeyError, TypeError, ValueError, OverflowError, IndexError):
        return None


def finish_dual_zero(io, completed_joint_mask):
    """普通 Rope Init 成功后，按旧关节顺序发送一次 LUT(0°) 直控。"""
    selected = int(io.selected_joint_mask) & int(completed_joint_mask)
    host_result = getattr(io, 'calibration_result', None)
    # 本轮已提交的 host 结果是收尾数据源；不能等异步 P4 echo 才取得自己的 LUT。
    # 简单机械算法 fake port 可继续提供旧 snapshot 接口。
    results = None if callable(host_result) else io._state().joint_calib_results
    plan = []
    for joint in range(ENCODER_COUNT):
        if joint not in DUAL_TENDON_JOINT_INDICES or not selected & (1 << joint):
            continue
        motors = joint_motors(joint)
        result = host_result(joint) if callable(host_result) else (
            results[joint] if joint < len(results) else None)
        targets = _zero_targets(result, motors)
        if len(motors) != 2 or targets is None:
            return False, f'J{joint:02d} zero finish requires a valid dual LUT containing 0 degrees'
        plan.append((joint, targets))
    for joint, targets in plan:
        if not io.owns_control():
            return False, f'J{joint:02d} zero finish cancelled or control lost'
        started = io.now_ms()
        speed = default_auto_calibration_params(joint).servoSpeed
        if not io.publish(targets, speed, _SERVO_ACC):
            return False, f'J{joint:02d} zero finish target publish failed'
        remaining = _STEP_PERIOD_MS - (io.now_ms() - started)
        if remaining > 0:
            io.sleep_ms(remaining)
    if plan:
        if not io.owns_control():
            return False, 'zero finish cancelled or control lost before release'
        if not io._release_motion():
            return False, 'zero finish direct owner release failed'
    return True, 'Rope Init and dual LUT zero finish completed'
