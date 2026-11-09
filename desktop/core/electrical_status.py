"""Status-bar CAN verdict and per-joint encoder state, no device access.

职责：把连接、编码器快照和 S3 自检折算成 CAN 标签与每关节编码器状态。
线程：只读快照。
输入：P4 连接年龄、raw 新鲜度、每关节有效性、S3 逐指自检。
输出：CAN: OK/ERROR/UNKNOWN 与 21 个 encoder 状态。
禁止依赖：不得导入 UI、串口或发送设备命令。
"""


def can_state(*, p4_online, connected_for_s, has_raw, raw_age_s, grace_s=3.0, stale_s=1.0):
    """CAN is ok only while encoder raw frames keep arriving over it."""
    if not p4_online or connected_for_s < grace_s:
        return {'level': 'unknown', 'label': 'CAN: UNKNOWN', 'cause': None}
    if has_raw and raw_age_s is not None and 0 <= raw_age_s <= stale_s:
        return {'level': 'ok', 'label': 'CAN: OK', 'cause': None}
    return {'level': 'can_error', 'label': 'CAN: ERROR', 'cause': 'can_error'}


def joint_states(*, can_level, faults=(), invalid_joints=()):
    """CAN down hides everything; CAN ok maps each joint to ok/open/short/error."""
    if can_level == 'can_error':
        return ['can_error'] * 21
    if can_level != 'ok':
        return ['unknown'] * 21
    bad = set(int(joint) for joint in invalid_joints)
    return [faults[joint] or ('error' if joint in bad else 'ok') for joint in range(21)]
