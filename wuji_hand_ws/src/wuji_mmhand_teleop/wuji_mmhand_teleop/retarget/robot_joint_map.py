"""Jxx 编号 <-> URDF joint 名 映射，以及从 URDF 派生的关节限位。

整个 retarget 链路里凡是涉及 URDF 关节限位的地方都通过本模块从
``URDFRobotHandModel`` 派生，避免把 URDF 里的 limit/origin 数值在多个文件里
重复硬编码。Jxx 编号本身只是逻辑约定，不是 URDF 值，所以作为常量保留在这里
作为唯一来源。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from wuji_mmhand_teleop.retarget.hand_retargeting.constants import DEFAULT_URDF_PATH
from wuji_mmhand_teleop.retarget.hand_retargeting.urdf_model import URDFRobotHandModel

ENCODER_COUNT = 21

# URDF joint 名 -> 逻辑关节编号 Jxx。
# 注意这不是电机 Mxx 映射；J16/J20 这类双电机关节由下位机再展开到物理电机。
SEMANTIC_JOINT_TO_ENCODER_INDEX: Dict[str, int] = {
    "little_aa_joint": 0,
    "mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint": 1,
    "mmhand_finger_4_distal_phalanx_1_pip_joint": 2,
    "mmhand_finger_4_fingertip_1_dip_joint": 3,
    "ring_aa_joint": 4,
    "mmhand_finger_3_proximal_phalanx_1_mcp_fe_joint": 5,
    "mmhand_finger_3_distal_phalanx_1_pip_joint": 6,
    "mmhand_finger_3_fingertip_1_dip_joint": 7,
    "middle_aa_joint": 8,
    "mmhand_finger_2_proximal_phalanx_1_mcp_fe_joint": 9,
    "mmhand_finger_2_distal_phalanx_1_pip_joint": 10,
    "mmhand_finger_2_fingertip_1_dip_joint": 11,
    "index_aa_joint": 12,
    "mmhand_finger_1_proximal_phalanx_1_mcp_fe_joint": 13,
    "mmhand_finger_1_distal_phalanx_1_pip_joint": 14,
    "mmhand_finger_1_fingertip_1_dip_joint": 15,
    # 拇指按当前机械手定义：CMC 是 J20，其余沿 URDF 链路顺序对应 J16-J19。
    "thumb_aa_joint": 16,
    "mmhand_thumb_1_mmhand_finger_5_proximal_phalanx_1_mcp_fe_joint": 17,
    "mmhand_thumb_1_mmhand_finger_5_distal_phalanx_1_pip_joint": 18,
    "mmhand_thumb_1_mmhand_finger_5_fingertip_1_dip_joint": 19,
    "thumb_cmc1_joint": 20,
}

# 旧版 "前缀-序号" URDF joint 名 -> Jxx 起始编号，兼容旧 URDF 命名。
URDF_PREFIX_TO_JOINT_START: Dict[str, int] = {
    "4": 0,
    "3": 4,
    "2": 8,
    "1": 12,
    "5": 16,
}

# 这些 Jxx 是侧摆/旋转根关节（AA/CMC/四指 root）。该集合只描述机械角色；
# 所有关节目标值都直接使用当前 URDF 的符号与零点。
SIGNED_ROOT_ENCODER_INDICES = frozenset({0, 4, 8, 12, 16, 20})


def is_signed_root_index(joint_idx: int) -> bool:
    return int(joint_idx) in SIGNED_ROOT_ENCODER_INDICES


def is_flexion_index(joint_idx: int) -> bool:
    return not is_signed_root_index(joint_idx)


def encoder_index_for_joint(joint_name: str) -> Optional[int]:
    """URDF joint 名 -> Jxx 编号；兼容语义名和旧版 '前缀-序号' 名。"""
    semantic_idx = SEMANTIC_JOINT_TO_ENCODER_INDEX.get(str(joint_name))
    if semantic_idx is not None:
        return semantic_idx if 0 <= semantic_idx < ENCODER_COUNT else None

    parts = str(joint_name).split("-")
    if len(parts) < 2 or parts[0] not in URDF_PREFIX_TO_JOINT_START or not parts[1].isdigit():
        return None
    idx = URDF_PREFIX_TO_JOINT_START[parts[0]] + int(parts[1]) - 1
    if not 0 <= idx < ENCODER_COUNT:
        return None
    return idx


def _urdf_joint_name_for_index(robot: URDFRobotHandModel, idx: int) -> Optional[str]:
    for joint_name, enc_idx in SEMANTIC_JOINT_TO_ENCODER_INDEX.items():
        if enc_idx == int(idx) and joint_name in robot.joint_specs:
            return joint_name
    return None


def build_encoder_target_limits_deg(robot: URDFRobotHandModel) -> List[Tuple[float, float]]:
    """返回 J00-J20 的直接 URDF 角度限位，不做隐藏的符号翻转。"""
    out: List[Tuple[float, float]] = [(0.0, 0.0)] * ENCODER_COUNT
    for joint_name, idx in SEMANTIC_JOINT_TO_ENCODER_INDEX.items():
        spec = robot.joint_specs.get(joint_name)
        if spec is None:
            continue
        lower_deg = math.degrees(float(spec.lower))
        upper_deg = math.degrees(float(spec.upper))
        out[idx] = (lower_deg, upper_deg)
    return out


def build_root_urdf_limits_deg(robot: URDFRobotHandModel) -> Dict[int, Tuple[float, float]]:
    """从 URDF 派生 6 个带符号根关节的 URDF 原始限位(度)。

    当前 target 限位与 URDF 限位取相同值，所以这个字典与
    ``build_encoder_target_limits_deg`` 在根关节上的结果一致；保留这个函数
    只是为了语义清晰。
    """
    out: Dict[int, Tuple[float, float]] = {}
    for idx in SIGNED_ROOT_ENCODER_INDICES:
        joint_name = _urdf_joint_name_for_index(robot, idx)
        if joint_name is None:
            continue
        spec = robot.joint_specs[joint_name]
        out[idx] = (math.degrees(float(spec.lower)), math.degrees(float(spec.upper)))
    return out


def default_robot_model(urdf_path: Optional[Path] = None) -> URDFRobotHandModel:
    return URDFRobotHandModel.from_urdf(Path(urdf_path or DEFAULT_URDF_PATH))


__all__ = [
    "ENCODER_COUNT",
    "SEMANTIC_JOINT_TO_ENCODER_INDEX",
    "URDF_PREFIX_TO_JOINT_START",
    "SIGNED_ROOT_ENCODER_INDICES",
    "build_encoder_target_limits_deg",
    "build_root_urdf_limits_deg",
    "default_robot_model",
    "encoder_index_for_joint",
    "is_flexion_index",
    "is_signed_root_index",
]
