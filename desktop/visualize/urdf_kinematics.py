"""活动 URDF 的纯运动学与显示坐标工具。

本模块刻意不导入 matplotlib、Qt 或 pyqtgraph：桌面 Hand Pose、命令行绘图和
单元测试共享同一套 URDF 解析与正向运动学，避免 GUI 初始化时混入第二套 Qt 后端。
"""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

import numpy as np


def _make_display_transform() -> np.ndarray:
    """把 URDF 坐标转换为桌面查看坐标，保持手指朝上、掌心朝向观察者。"""
    transform = np.eye(4, dtype=float)
    ax = -np.pi / 2
    cx, sx = np.cos(ax), np.sin(ax)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ay = -np.pi / 2
    cy, sy = np.cos(ay), np.sin(ay)
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz_pi = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    transform[:3, :3] = rz_pi @ ry @ rx
    return transform


DISPLAY_TRANSFORM: np.ndarray = _make_display_transform()


def _parse_floats(text: str) -> np.ndarray:
    return np.array(
        [float(value) for value in re.split(r"[\s,]+", text.strip()) if value],
        dtype=float,
    )


def _rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _homogeneous_from_origin(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rpy_to_rotation(rpy)
    transform[:3, 3] = xyz
    return transform


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float).reshape(3)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        return np.eye(3, dtype=float)
    x, y, z = vector / norm
    cos_value, sin_value = np.cos(angle), np.sin(angle)
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3, dtype=float) + sin_value * cross + (1.0 - cos_value) * (cross @ cross)


def _parse_origin(element) -> Tuple[np.ndarray, np.ndarray]:
    origin = element.find("origin")
    if origin is None:
        return np.zeros(3, dtype=float), np.zeros(3, dtype=float)
    return (
        _parse_floats(origin.attrib.get("xyz", "0 0 0")),
        _parse_floats(origin.attrib.get("rpy", "0 0 0")),
    )


class UrdfKinematics:
    """只解析 URDF link/joint 树并计算正向运动学，不读取网格文件。"""

    def __init__(self, urdf_path: str):
        self.urdf_path = os.path.abspath(urdf_path)
        self.root_dir = os.path.dirname(self.urdf_path)
        robot = ET.parse(self.urdf_path).getroot()
        self.link_names = [
            name for link in robot.findall("link")
            if (name := link.attrib.get("name"))
        ]
        self.joints: List[dict] = []
        for joint in robot.findall("joint"):
            parent_element = joint.find("parent")
            child_element = joint.find("child")
            if parent_element is None or child_element is None:
                continue
            xyz, rpy = _parse_origin(joint)
            axis_element = joint.find("axis")
            limit_element = joint.find("limit")
            self.joints.append({
                "name": joint.attrib.get("name", ""),
                "type": joint.attrib.get("type", "fixed"),
                "parent": parent_element.attrib.get("link", ""),
                "child": child_element.attrib.get("link", ""),
                "xyz": xyz,
                "rpy": rpy,
                "axis": (
                    _parse_floats(axis_element.attrib.get("xyz", "0 0 1"))
                    if axis_element is not None else np.array([0.0, 0.0, 1.0])
                ),
                "limit": (
                    float(limit_element.attrib.get("lower", "0")),
                    float(limit_element.attrib.get("upper", "0")),
                ) if limit_element is not None else (0.0, 0.0),
            })
        self._children: Dict[str, List[dict]] = {}
        for joint in self.joints:
            self._children.setdefault(joint["parent"], []).append(joint)
        self.root_link = self._find_root()

    def _find_root(self) -> str:
        children = {joint["child"] for joint in self.joints}
        parents = {joint["parent"] for joint in self.joints}
        roots = parents - children
        return sorted(roots)[0] if roots else (self.link_names[0] if self.link_names else "base_link")

    def joint_by_name(self) -> Dict[str, dict]:
        return {joint["name"]: joint for joint in self.joints if joint["name"]}

    def compute_link_poses(self, joint_positions: Dict[str, float]) -> Dict[str, np.ndarray]:
        """返回每个 link 在 URDF 世界坐标中的 4×4 位姿。"""
        world: Dict[str, np.ndarray] = {self.root_link: np.eye(4, dtype=float)}

        def visit(parent: str, world_parent: np.ndarray) -> None:
            for joint in self._children.get(parent, []):
                origin = _homogeneous_from_origin(joint["xyz"], joint["rpy"])
                if joint["type"] == "revolute":
                    rotation = np.eye(4, dtype=float)
                    rotation[:3, :3] = _rodrigues(
                        joint["axis"], float(joint_positions.get(joint["name"], 0.0))
                    )
                    parent_child = origin @ rotation
                else:
                    parent_child = origin
                child = joint["child"]
                world_child = world_parent @ parent_child
                world[child] = world_child
                visit(child, world_child)

        visit(self.root_link, world[self.root_link])
        return world


def apply_display_transform(poses: Dict[str, np.ndarray], transform: np.ndarray) -> Dict[str, np.ndarray]:
    """把所有 link 位姿左乘桌面显示坐标变换。"""
    return {name: transform @ pose for name, pose in poses.items()}


__all__ = ["DISPLAY_TRANSFORM", "UrdfKinematics", "apply_display_transform"]
