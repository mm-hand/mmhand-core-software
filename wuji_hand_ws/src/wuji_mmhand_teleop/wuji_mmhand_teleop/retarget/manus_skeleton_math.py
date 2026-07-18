"""MANUS raw skeleton 解析共享数学与拓扑常量。

本模块集中存放 ``manus_calibration.py`` 和 ``manus_raw_skeleton.py`` 都需要的
MANUS skeleton node 解析函数和手指节点拓扑常量，使两个文件不再各自重复定义
同名函数。坐标几何通用工具(归一化等)继续复用 ``hand_retargeting.math_utils``。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np


# Wuji MediaPipe topology has exactly four points per finger.  Keep that
# topology intact instead of fabricating an extra intermediate node.
FINGER_RAW_NODE_KEYS: Dict[str, tuple[str, ...]] = {
    "thumb": ("cmc", "mcp", "ip", "tip"),
    "index": ("mcp", "pip", "dip", "tip"),
    "middle": ("mcp", "pip", "dip", "tip"),
    "ring": ("mcp", "pip", "dip", "tip"),
    "pinky": ("mcp", "pip", "dip", "tip"),
}

# 每根手指的 retarget 锚点节点：anchor -> tip 向量在该节点取。
FINGER_TARGET_ANCHOR_NODE: Dict[str, str] = {
    "thumb": "cmc",
    "index": "mcp",
    "middle": "mcp",
    "ring": "mcp",
    "pinky": "mcp",
}


def node_position(node: Any) -> Optional[np.ndarray]:
    """从 MANUS skeleton node 取位置 (3,)；非法返回 None。"""
    if not isinstance(node, dict):
        return None
    pos = node.get("pos")
    if not isinstance(pos, list) or len(pos) < 3:
        return None
    try:
        return np.array([float(pos[0]), float(pos[1]), float(pos[2])], dtype=float)
    except (TypeError, ValueError):
        return None


def node_rotation(node: Any) -> Optional[np.ndarray]:
    """从 MANUS skeleton node 取单位四元数 wxyz (4,)；非法返回 None。"""
    if not isinstance(node, dict):
        return None
    rot = node.get("rot")
    if not isinstance(rot, list) or len(rot) < 4:
        return None
    try:
        quat = np.array([float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])], dtype=float)
    except (TypeError, ValueError):
        return None
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8:
        return None
    return quat / norm


def quat_wxyz_to_rotation_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    """wxyz 四元数 -> 3x3 旋转矩阵。"""
    w, x, y, z = [float(v) for v in quat_wxyz]
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def orient_palm_z_axis(frame: Any, z_axis: np.ndarray) -> np.ndarray:
    """Orient the palm normal using the hand quaternion, matching desktop."""
    skeleton = getattr(frame, "skeleton", {}) or {}
    hand_rot = node_rotation(skeleton.get("hand") if isinstance(skeleton, dict) else None)
    if hand_rot is None:
        return np.asarray(z_axis, dtype=float)
    hand_normal = quat_wxyz_to_rotation_matrix(hand_rot)[:, 2]
    return -np.asarray(z_axis, dtype=float) if float(np.dot(z_axis, hand_normal)) < 0.0 else np.asarray(z_axis, dtype=float)


__all__ = [
    "FINGER_RAW_NODE_KEYS",
    "FINGER_TARGET_ANCHOR_NODE",
    "node_position",
    "node_rotation",
    "orient_palm_z_axis",
    "quat_wxyz_to_rotation_matrix",
]
