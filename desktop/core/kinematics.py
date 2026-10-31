"""kinematics.py - V5 接触标定的最小 FK。

职责：解析 URDF，批量计算 15 个触觉 sensor link 的位姿，并支持关节原点几何修正。
线程：只读模型，可被任意线程共享；调用方各自传入 q。
输入：URDF 路径、每事件 21 关节角（度）、每关节修正矩阵。
输出：形状 (E, 15, 4, 4) 的 sensor link 变换。
状态所有权：只拥有解析后的不可变 URDF 树。
禁止依赖：禁止导入 UI、通信、协议或设备状态。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .angle_mapping import JOINT_SPECS
from .tactile_geometry import SENSOR_LINKS, SENSOR_ORDER

# SENSOR_LINKS/SENSOR_ORDER 由 tactile_geometry 统一维护，FK 与 UI 共用。


def _vec(text, default="0 0 0"):
    return np.fromstring(text if text is not None else default, sep=" ")


def _rodrigues(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    cross = np.array(((0, -axis[2], axis[1]),
                      (axis[2], 0, -axis[0]),
                      (-axis[1], axis[0], 0)))
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * cross @ cross


def _rodrigues_batch(axis, angles):
    axis = np.asarray(axis, float)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    cross = np.array(((0, -axis[2], axis[1]),
                      (axis[2], 0, -axis[0]),
                      (-axis[1], axis[0], 0)))
    count = len(angles)
    return (np.eye(3)[None]
            + np.sin(angles)[:, None, None] * cross[None]
            + (1.0 - np.cos(angles))[:, None, None] * (cross @ cross)[None])


def _origin_matrix(xyz, rpy):
    x, y, z = xyz
    roll, pitch, yaw = rpy
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rotation = np.array((
        (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
        (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
        (-sy, cy * sx, cy * cx),
    ))
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = (x, y, z)
    return matrix


origin_matrix = _origin_matrix


def decompose_origin(matrix):
    """把 4x4 变换按 URDF 约定拆成 (xyz, rpy)，供 JSON/UI 使用。"""
    rotation = np.asarray(matrix, float)[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return tuple(float(v) for v in np.asarray(matrix, float)[:3, 3]), (
        roll, pitch, yaw)


def perpendicular_basis(axis):
    """给关节轴构造稳定的两个垂直单位向量。"""
    axis = np.asarray(axis, float)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    helper = np.eye(3)[int(np.argmin(np.abs(axis)))]
    first = np.cross(axis, helper)
    first = first / max(np.linalg.norm(first), 1e-12)
    second = np.cross(axis, first)
    return first, second


class Kinematics:
    """只读 URDF 树 + 批量 FK。"""

    def __init__(self, urdf_path):
        root = ET.parse(Path(urdf_path)).getroot()
        joint_index = {JOINT_SPECS[index].urdf_joint_name: index
                       for index in range(len(JOINT_SPECS))}
        self.link_names = []
        link_index = {}

        def link(name):
            if name not in link_index:
                link_index[name] = len(self.link_names)
                self.link_names.append(name)
            return link_index[name]

        self.joints = []
        for node in root.findall("joint"):
            parent = link(node.find("parent").attrib["link"])
            child = link(node.find("child").attrib["link"])
            origin = node.find("origin")
            xyz = _vec(origin.get("xyz") if origin is not None else None)
            rpy = _vec(origin.get("rpy") if origin is not None else None)
            axis_node = node.find("axis")
            axis = _vec(axis_node.get("xyz") if axis_node is not None else None,
                        "1 0 0")
            self.joints.append({
                "name": node.attrib["name"],
                "parent": parent,
                "child": child,
                "origin": _origin_matrix(xyz, rpy),
                "axis": axis / max(np.linalg.norm(axis), 1e-12),
                "index": joint_index.get(node.attrib["name"]),
            })
        # URDF joint 顺序不保证拓扑序；从唯一根 link 遍历出 FK 顺序。
        children = {joint["child"] for joint in self.joints}
        roots = set(range(len(self.link_names))) - children
        if len(roots) != 1:
            raise ValueError("URDF must have exactly one root link")
        self.root = roots.pop()
        by_parent = {}
        for joint in self.joints:
            by_parent.setdefault(joint["parent"], []).append(joint)
        ordered, stack = [], [self.root]
        while stack:
            parent = stack.pop()
            for joint in by_parent.get(parent, []):
                ordered.append(joint)
                stack.append(joint["child"])
        if len(ordered) != len(self.joints):
            raise ValueError("URDF joint tree is disconnected")
        self.joints = ordered
        self.by_name = {joint["name"]: joint for joint in self.joints}
        self.names = tuple(JOINT_SPECS[index].urdf_joint_name
                           for index in range(len(JOINT_SPECS)))
        self.sensor_indices = np.asarray(
            [link_index[SENSOR_LINKS[sensor]] for sensor in SENSOR_ORDER],
            dtype=int)

    def correction_origin(self, joint_name, translation, alpha, beta):
        """由 3 平移 + 2 垂直轴旋转合成最终关节原点 T0·E。"""
        joint = self.by_name[joint_name]
        first, second = perpendicular_basis(joint["axis"])
        vector = first * alpha + second * beta
        norm = float(np.linalg.norm(vector))
        rotation = (_rodrigues(vector / norm, norm) if norm > 1e-12
                    else np.eye(3))
        error = np.eye(4)
        error[:3, :3] = rotation
        error[:3, 3] = np.asarray(translation, float)
        return joint["origin"] @ error

    def sensor_transforms(self, q_deg, origins=None):
        """返回 (E, 15, 4, 4)；q_deg 形状 (E, 21)。"""
        q = np.asarray(q_deg, float)
        if q.ndim == 1:
            q = q[None]
        count = q.shape[0]
        if q.shape[1] != len(self.names):
            raise ValueError(f"expected {len(self.names)} joint angles")
        transforms = [None] * len(self.link_names)
        transforms[self.root] = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
        radians = np.radians(q)
        for joint in self.joints:
            origin = joint["origin"]
            if origins and joint["name"] in origins:
                origin = origins[joint["name"]]
            child = transforms[joint["parent"]] @ origin
            index = joint["index"]
            if index is not None:
                motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
                motion[:, :3, :3] = _rodrigues_batch(joint["axis"], radians[:, index])
                child = child @ motion
            transforms[joint["child"]] = child
        return np.stack([transforms[index] for index in self.sensor_indices], axis=1)
