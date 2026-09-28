"""ui_control_scope.py - MMHand Desktop 模块契约。

职责：管理关节/舵机选择范围和活动位图的 UI 规则。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""

from typing import List, Optional, Tuple

from desktop.core.protocol import ENCODER_COUNT, HAND_PROFILE, MOTOR_COUNT, VALID_MOTOR_MASK

from desktop.core.motor_mapping import MOTOR_TO_JOINT_INDEX


FINGER_JOINT_RANGES = [
    ("Pinky J00-J03", 0, 3),
    ("Ring J04-J07", 4, 7),
    ("Middle J08-J11", 8, 11),
    ("Index J12-J15", 12, 15),
    ("Thumb J16-J20", 16, 20),
]

FINGER_WORKFLOW_JOINT_ORDER = {
    # Thumb workflow order follows the mechanical calibration sequence.
    "Thumb J16-J20": [20, 16, 17, 18, 19],
}

# 四指侧摆双电机关节在自动标定时会互相让出机械工作空间。该集合必须与
# P4 的 kWorkspaceDualJoints 保持一致；拇指 J16/J20 不参与四指让位序列。
CALIBRATION_WORKSPACE_DUAL_JOINT_INDICES = (0, 4, 8, 12)


# 设备 active scope 是实际标定范围；该列表仅供尚未取得设备状态时的 UI 展示。
AUTO_CALIBRATION_JOINT_INDICES = [
    joint_idx
    for group_label, joint_start, joint_end in FINGER_JOINT_RANGES
    for joint_idx in FINGER_WORKFLOW_JOINT_ORDER.get(
        group_label, list(range(joint_start, joint_end + 1))
    )
]


def _motor_channel_valid(idx: int) -> bool:
    """Return whether a motor channel is present in the active hand profile."""
    return 0 <= idx < MOTOR_COUNT and ((VALID_MOTOR_MASK >> idx) & 0x1) != 0


class ControlScopeModel:
    """Pure scope model shared by control views and calibration workflows."""

    @staticmethod
    def joint_indices_for_group(group_label: str, joint_start: int, joint_end: int) -> List[int]:
        """Return numeric joint indices for one finger group."""
        del group_label
        return list(range(joint_start, joint_end + 1))

    @staticmethod
    def workflow_joint_indices_for_group(
        group_label: str,
        joint_start: int,
        joint_end: int,
    ) -> List[int]:
        """Return workflow-ordered joint indices for one finger group."""
        custom = FINGER_WORKFLOW_JOINT_ORDER.get(group_label)
        if custom:
            return [idx for idx in custom if joint_start <= idx <= joint_end]
        return ControlScopeModel.joint_indices_for_group(group_label, joint_start, joint_end)

    @staticmethod
    def joint_group_mask(group_label: str, joint_start: int, joint_end: int) -> int:
        """Return the joint bit mask for one finger group."""
        mask = 0
        for joint_idx in ControlScopeModel.joint_indices_for_group(
            group_label, joint_start, joint_end
        ):
            mask |= 1 << joint_idx
        return mask

    @staticmethod
    def ordered_joint_indices_from_mask(joint_mask: int) -> List[int]:
        """Return selected joints in workflow order."""
        ordered: List[int] = []
        seen = set()
        for group_label, joint_start, joint_end in FINGER_JOINT_RANGES:
            for joint_idx in ControlScopeModel.workflow_joint_indices_for_group(
                group_label, joint_start, joint_end
            ):
                if (int(joint_mask) & (1 << joint_idx)) != 0:
                    ordered.append(joint_idx)
                    seen.add(joint_idx)
        for joint_idx in range(ENCODER_COUNT):
            if joint_idx not in seen and (int(joint_mask) & (1 << joint_idx)) != 0:
                ordered.append(joint_idx)
        return ordered

    @staticmethod
    def joint_rows_by_finger_group() -> Tuple[List[int], List[Tuple[int, int, str]]]:
        """Return joint table rows grouped by finger."""
        rows: List[int] = []
        group_ranges: List[Tuple[int, int, str]] = []
        for group_label, joint_start, joint_end in FINGER_JOINT_RANGES:
            group_start_row = len(rows)
            rows.extend(
                ControlScopeModel.joint_indices_for_group(
                    group_label, joint_start, joint_end
                )
            )
            if len(rows) > group_start_row:
                group_ranges.append((group_start_row, len(rows) - 1, group_label))
        return rows, group_ranges

    @staticmethod
    def motor_rows_by_finger_group() -> Tuple[List[int], List[Tuple[int, int, str]]]:
        """Return motor table rows grouped by mapped finger."""
        rows: List[int] = []
        group_ranges: List[Tuple[int, int, str]] = []
        used = set()

        for group_label, joint_start, joint_end in FINGER_JOINT_RANGES:
            group_start_row = len(rows)
            for joint_idx in ControlScopeModel.joint_indices_for_group(
                group_label, joint_start, joint_end
            ):
                for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
                    if (
                        _motor_channel_valid(motor_idx)
                        and motor_idx not in used
                        and int(mapped_joint) == joint_idx
                    ):
                        rows.append(motor_idx)
                        used.add(motor_idx)
            if len(rows) > group_start_row:
                group_ranges.append((group_start_row, len(rows) - 1, group_label))

        for motor_idx in range(MOTOR_COUNT):
            if _motor_channel_valid(motor_idx) and motor_idx not in used:
                rows.append(motor_idx)
                used.add(motor_idx)
                group_ranges.append((len(rows) - 1, len(rows) - 1, f"Extra M{motor_idx:02d}"))

        for motor_idx in range(MOTOR_COUNT):
            if motor_idx not in used:
                rows.append(motor_idx)
                used.add(motor_idx)

        return rows, group_ranges

    @staticmethod
    def motors_by_joint_index() -> List[List[int]]:
        """Return valid motor channels grouped by joint index."""
        result = [[] for _ in range(ENCODER_COUNT)]
        for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
            if _motor_channel_valid(motor_idx) and 0 <= int(joint_idx) < ENCODER_COUNT:
                result[int(joint_idx)].append(motor_idx)
        return result

    @staticmethod
    def antagonist_motor_row_groups(
        motor_rows: List[int],
    ) -> List[Tuple[int, int, List[int], int]]:
        """Return row spans for paired antagonist motors."""
        groups: List[Tuple[int, int, List[int], int]] = []
        row = 0
        while row < len(motor_rows):
            motor_idx = int(motor_rows[row])
            joint_idx = int(MOTOR_TO_JOINT_INDEX[motor_idx]) if motor_idx < len(MOTOR_TO_JOINT_INDEX) else -1
            end = row + 1
            while end < len(motor_rows):
                next_motor = int(motor_rows[end])
                next_joint = (
                    int(MOTOR_TO_JOINT_INDEX[next_motor])
                    if next_motor < len(MOTOR_TO_JOINT_INDEX)
                    else -1
                )
                if next_joint != joint_idx:
                    break
                end += 1
            motors = [int(motor_rows[idx]) for idx in range(row, end)]
            if joint_idx >= 0 and len(motors) == 2 and all(_motor_channel_valid(idx) for idx in motors):
                groups.append((row, end - 1, motors, joint_idx))
            row = end
        return groups

    @staticmethod
    def scope_range(start: int, end: int, require_can: bool = True) -> Tuple[int, int, bool]:
        """Return motor mask, joint mask, and CAN requirement for a joint range."""
        motor_mask = 0
        joint_mask = 0
        for joint_idx in range(start, end + 1):
            if 0 <= joint_idx < ENCODER_COUNT:
                joint_mask |= 1 << joint_idx
        for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
            if _motor_channel_valid(motor_idx) and start <= int(joint_idx) <= end:
                motor_mask |= 1 << motor_idx
        return motor_mask, joint_mask, require_can

    @staticmethod
    def motor_mask_for_joint_mask(joint_mask: int) -> int:
        """Return the motor mask mapped from a joint mask."""
        mask = 0
        for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
            joint_idx = int(joint_idx)
            if (
                _motor_channel_valid(motor_idx)
                and 0 <= joint_idx < ENCODER_COUNT
                and (int(joint_mask) & (1 << joint_idx)) != 0
            ):
                mask |= 1 << motor_idx
        return mask

    @staticmethod
    def joint_mask_for_motor_mask(motor_mask: int) -> int:
        """根据正式 J/M 映射返回一组舵机影响的关节位图。"""
        mask = 0
        for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
            joint_idx = int(joint_idx)
            if (
                _motor_channel_valid(motor_idx)
                and (int(motor_mask) & (1 << motor_idx)) != 0
                and 0 <= joint_idx < ENCODER_COUNT
            ):
                mask |= 1 << joint_idx
        return mask

    @staticmethod
    def calibration_workspace_helper_reset_scope(
        selected_joint_mask: int,
        servo_online_bitmap: int,
        multiturn_unknown_bitmap: int,
    ) -> Tuple[int, int, int, int]:
        """规划标定前必须一并 Reset Zero 的四指工作空间辅助关节。

        返回 ``(helper_joint_mask, helper_motor_mask,
        partial_joint_mask, partial_motor_mask)``：
        - 辅助双电机全部离线时，P4 会跳过该手指，不加入复位范围；
        - 两台均在线且 Turn Known 时复用现有姿态，不额外复位；
        - 两台均在线但任一 Turn Unknown 时，两台一起复位，避免双腱坐标分裂；
        - 仅一台在线时拒绝整次标定，防止先复位部分电机后才发现无法让位。
        """
        selected_joint_mask = int(selected_joint_mask) & ((1 << ENCODER_COUNT) - 1)
        workspace_mask = sum(
            1 << joint_idx
            for joint_idx in CALIBRATION_WORKSPACE_DUAL_JOINT_INDICES
        )
        if (selected_joint_mask & workspace_mask) == 0:
            return 0, 0, 0, 0

        online_mask = int(servo_online_bitmap) & VALID_MOTOR_MASK
        unknown_mask = int(multiturn_unknown_bitmap) & VALID_MOTOR_MASK
        helper_joint_mask = 0
        helper_motor_mask = 0
        partial_joint_mask = 0
        partial_motor_mask = 0

        for joint_idx in CALIBRATION_WORKSPACE_DUAL_JOINT_INDICES:
            # 用户选中的目标关节已包含在 selected motor scope 中，这里只规划
            # 其他手指的让位电机，防止同一双电机对被重复分类。
            if selected_joint_mask & (1 << joint_idx):
                continue
            pair_mask = ControlScopeModel.motor_mask_for_joint_mask(1 << joint_idx)
            pair_online = online_mask & pair_mask
            if pair_mask == 0 or pair_online == 0:
                continue
            if pair_online != pair_mask:
                partial_joint_mask |= 1 << joint_idx
                partial_motor_mask |= pair_mask
                continue
            if unknown_mask & pair_mask:
                helper_joint_mask |= 1 << joint_idx
                helper_motor_mask |= pair_mask

        return (
            helper_joint_mask,
            helper_motor_mask,
            partial_joint_mask,
            partial_motor_mask,
        )
