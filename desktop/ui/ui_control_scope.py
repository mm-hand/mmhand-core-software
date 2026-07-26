"""Control scope and channel grouping shared by the control panel views."""

import re
from pathlib import Path
from typing import List, Optional, Tuple

from desktop.paths import PROJECT_ROOT
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


def _load_default_auto_calibration_joint_indices() -> List[int]:
    """Load the default auto-calibration joint order from firmware source when available."""
    fallback = [
        0, 1, 2, 3,
        4, 5, 6, 7,
        8, 9, 10, 11,
        12, 13, 14, 15,
        20, 16, 17, 18, 19,
    ]
    try:
        source_path = (
            PROJECT_ROOT
            / "ESP32-P4"
            / "ServoBoardMain"
            / "src"
            / "calibration"
            / "AutoCalibrationTask.cpp"
        )
        text = source_path.read_text(encoding="utf-8", errors="ignore")
        match = re.search(
            r"const\s+uint8_t\s+kDefaultAutoCalibrationJoints\s*\[\]\s*=\s*\{([^}]*)\}",
            text,
            re.MULTILINE,
        )
        if not match:
            return fallback
        joints = [int(token) for token in re.findall(r"\b\d+\b", match.group(1))]
        joints = [idx for idx in joints if 0 <= idx < ENCODER_COUNT]
        return joints or fallback
    except Exception:
        return fallback


AUTO_CALIBRATION_JOINT_INDICES = _load_default_auto_calibration_joint_indices()


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
