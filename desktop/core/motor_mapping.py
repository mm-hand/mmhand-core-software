"""motor_mapping.py - MMHand Desktop 模块契约。

职责：维护 J00-J20 与 M00-M26 的稳定映射和位图换算。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""

from .protocol import ENCODER_COUNT, MOTOR_ABS_CMD_MAX, MOTOR_ABS_CMD_MIN, MOTOR_COUNT


MOTOR_TO_JOINT_INDEX = [
    0, 0, 1, 2, 3,
    4, 4, 5, 6, 7,
    8, 8, 9, 10, 11,
    12, 12, 13, 14, 15,
    16, 16, 17, 18, 19, 20, 20,
]
if len(MOTOR_TO_JOINT_INDEX) < MOTOR_COUNT:
    MOTOR_TO_JOINT_INDEX.extend([-1] * (MOTOR_COUNT - len(MOTOR_TO_JOINT_INDEX)))

JOINT_TO_PRIMARY_MOTOR_INDEX = [-1] * ENCODER_COUNT
for _motor_idx, _joint_idx in enumerate(MOTOR_TO_JOINT_INDEX):
    if 0 <= _joint_idx < ENCODER_COUNT and JOINT_TO_PRIMARY_MOTOR_INDEX[_joint_idx] < 0:
        JOINT_TO_PRIMARY_MOTOR_INDEX[_joint_idx] = _motor_idx

MOTOR_TENDON_SHORTEN_ABS_SIGN = [1] * MOTOR_COUNT


def clamp_motor_abs(value: int) -> int:
    return max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(value)))


def motor_tendon_shorten_abs_sign(motor_idx: int) -> int:
    if 0 <= int(motor_idx) < len(MOTOR_TENDON_SHORTEN_ABS_SIGN):
        return -1 if int(MOTOR_TENDON_SHORTEN_ABS_SIGN[int(motor_idx)]) < 0 else 1
    return 1


def motor_abs_after_tendon_action(current_abs: int, motor_idx: int, shorten_counts: int) -> int:
    sign = motor_tendon_shorten_abs_sign(motor_idx)
    return clamp_motor_abs(int(current_abs) + sign * int(shorten_counts))


def motor_target_from_servo_feedback(servo_raw_single: int) -> int:
    return max(0, min(4095, int(servo_raw_single) % 4096))
