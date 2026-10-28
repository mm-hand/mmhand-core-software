"""把标定包的绝对坐标平移到电机当前的多圈坐标系（不写电机）。

职责：计算整圈偏移并平移标定包的伺服 ABS 字段。
线程：由控制器在连接/导入线程调用，纯函数，不创建线程。
输入：标定包 payload、保存的原生 ABS 与电机当前 ABS。
输出：平移后的快照副本（越出 ±30715 时抛 ValueError）。
状态所有权：不拥有设备状态，只返回新字典，不改入参。
禁止依赖：禁止访问串口、UI 或改变硬件判据。
"""
from __future__ import annotations

import copy

TURN_COUNTS = 4096
ABS_COMMAND_LIMIT = 30715

_SERVO_ABS_FIELDS = ("startServoAbs", "tightServoAbs", "zeroServoAbs")


def turn_offsets(anchors, currents) -> list[int]:
    """每台电机丢了几圈：k = round((anchor - current) / 4096)。"""
    return [int(round((int(a) - int(c)) / TURN_COUNTS))
            for a, c in zip(anchors, currents)]


def shift_snapshot(snapshot: dict, offsets: dict[int, int]) -> dict:
    """返回把每个 motor 的绝对坐标减去 k*4096 的快照副本。

    只平移该电机有效的 ABS 字段；越出舵机命令范围（±30715）时抛 ValueError，
    由调用方拒绝自动对账并要求重标。
    """
    payload = copy.deepcopy(snapshot)
    shifts = {int(m): int(k) for m, k in offsets.items() if int(k) != 0}

    def shifted(motor: int, value) -> int:
        turns = shifts.get(motor, 0)
        if not turns:
            return int(value)
        result = int(value) - turns * TURN_COUNTS
        if result < -ABS_COMMAND_LIMIT or result > ABS_COMMAND_LIMIT:
            raise ValueError(
                f"M{motor:02d} abs {int(value)} shifted to {result} is outside "
                f"+/-{ABS_COMMAND_LIMIT}; recalibrate instead")
        return result

    def shift_list(key: str, valid_mask: int) -> None:
        values = payload.get(key)
        if not isinstance(values, list) or len(values) != 27:
            return
        payload[key] = [
            shifted(motor, value) if (valid_mask >> motor) & 1 else value
            for motor, value in enumerate(values)
        ]

    if isinstance(payload.get("multiturn_anchor_logical_abs"), list):
        shift_list("multiturn_anchor_logical_abs",
                   int(str(payload.get("multiturn_anchor_valid_bitmap", 0)), 0))
    shift_list("motor_abs_limit_min",
               int(str(payload.get("motor_abs_limit_valid_bitmap", 0)), 0))
    shift_list("motor_abs_limit_max",
               int(str(payload.get("motor_abs_limit_valid_bitmap", 0)), 0))
    for result in payload.get("joint_calib_results") or []:
        if not isinstance(result, dict):
            continue
        motors = result.get("tendonLutMotor") or []
        # 标量端点属于主电机，双电机LUT的两列分别平移。
        motor = motors[0] if motors else None
        if isinstance(motor, int) and 0 <= motor < 27:
            for field in _SERVO_ABS_FIELDS:
                if isinstance(result.get(field), int):
                    result[field] = shifted(motor, result[field])
        lut = result.get("tendonLutMotorAbs")
        if isinstance(lut, list):
            for column, motor in enumerate(motors):
                if not isinstance(motor, int) or motor >= 27:
                    continue
                if column < len(lut) and isinstance(lut[column], list):
                    lut[column] = [shifted(motor, value) for value in lut[column]]
    return payload
