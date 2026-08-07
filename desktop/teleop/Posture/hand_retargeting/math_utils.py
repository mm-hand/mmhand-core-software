"""math_utils.py - MMHand Desktop 模块契约。

职责：提供重定向所需的向量、旋转和坐标变换函数。
线程：接收器可在 SDK/摄像头工作线程执行；纯映射和数学函数不创建线程。
输入：手套、骨架或视觉关键点，以及活动 URDF/标定配置。
输出：统一 J00-J20 目标、接收状态和可诊断质量信息。
状态所有权：接收器拥有最近输入帧；角度映射和模型配置按只读快照使用。
禁止依赖：禁止依赖 Qt/UI 或直接写串口，目标必须经过统一角度适配和后端门禁。
"""

from __future__ import annotations

import numpy as np


def parse_xyz(text: str) -> np.ndarray:
    return np.array([float(x) for x in text.split()], dtype=float)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize(vec: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < eps:
        return np.zeros_like(vec)
    return vec / norm


def angle_between(vec_a: np.ndarray, vec_b: np.ndarray) -> float:
    a = normalize(vec_a)
    b = normalize(vec_b)
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na < 1e-8 or nb < 1e-8:
        return 0.0
    cos_value = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.arccos(cos_value))


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)

    rot_x = np.array(
        [[1, 0, 0], [0, cx, -sx], [0, sx, cx]],
        dtype=float,
    )
    rot_y = np.array(
        [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]],
        dtype=float,
    )
    rot_z = np.array(
        [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]],
        dtype=float,
    )
    return rot_z @ rot_y @ rot_x


def axis_angle_rotation(axis: np.ndarray, theta: float) -> np.ndarray:
    axis_unit = normalize(axis)
    norm_axis = float(np.linalg.norm(axis_unit))
    if norm_axis < 1e-8 or abs(theta) < 1e-8:
        return np.eye(3, dtype=float)

    x, y, z = axis_unit
    c = np.cos(theta)
    s = np.sin(theta)
    one_c = 1.0 - c

    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )
