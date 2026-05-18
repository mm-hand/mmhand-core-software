"""URDF parsing and simplified forward kinematics for the robot hand."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .constants import ROBOT_FINGER_PREFIX
from .math_utils import axis_angle_rotation, clamp, make_transform, parse_xyz, rpy_to_rotation


@dataclass
class JointSpec:
    name: str
    joint_type: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


@dataclass
class FingerChain:
    name: str
    prefix: str
    revolute_joint_names: list[str]
    tip_joint_name: str
    base_origin: np.ndarray
    max_reach: float


class URDFRobotHandModel:
    """URDF 机械手模型，提供关节信息与简化正向运动学。"""

    def __init__(
        self,
        joint_specs: dict[str, JointSpec],
        finger_chains: dict[str, FingerChain],
        revolute_joint_order: list[str],
    ) -> None:
        self.joint_specs = joint_specs
        self.finger_chains = finger_chains
        self.revolute_joint_order = revolute_joint_order

    @classmethod
    def from_urdf(cls, urdf_path: Path) -> "URDFRobotHandModel":
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        root = ET.parse(urdf_path).getroot()
        joint_specs: dict[str, JointSpec] = {}

        for joint_elem in root.findall("joint"):
            name = joint_elem.attrib["name"]
            joint_type = joint_elem.attrib.get("type", "fixed")

            origin_elem = joint_elem.find("origin")
            origin_xyz = (
                parse_xyz(origin_elem.attrib.get("xyz", "0 0 0"))
                if origin_elem is not None
                else np.zeros(3, dtype=float)
            )
            origin_rpy = (
                parse_xyz(origin_elem.attrib.get("rpy", "0 0 0"))
                if origin_elem is not None
                else np.zeros(3, dtype=float)
            )

            axis_elem = joint_elem.find("axis")
            axis = (
                parse_xyz(axis_elem.attrib.get("xyz", "0 0 1"))
                if axis_elem is not None
                else np.array([0.0, 0.0, 1.0], dtype=float)
            )

            limit_elem = joint_elem.find("limit")
            lower = float(limit_elem.attrib.get("lower", "0")) if limit_elem is not None else 0.0
            upper = float(limit_elem.attrib.get("upper", "0")) if limit_elem is not None else 0.0

            joint_specs[name] = JointSpec(
                name=name,
                joint_type=joint_type,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                axis=axis,
                lower=lower,
                upper=upper,
            )

        finger_chains: dict[str, FingerChain] = {}
        revolute_joint_order: list[str] = []

        for finger_name, prefix in ROBOT_FINGER_PREFIX.items():
            revolute_joints = [
                name
                for name, spec in joint_specs.items()
                if spec.joint_type == "revolute"
                and name.startswith(f"{prefix}-")
                and name.split("-")[1].isdigit()
            ]
            revolute_joints.sort(key=lambda joint_name: int(joint_name.split("-")[1]))
            if not revolute_joints:
                continue

            tip_joint_name = f"{prefix}-tip"
            if tip_joint_name not in joint_specs:
                raise ValueError(f"Missing fixed tip joint '{tip_joint_name}' in URDF")

            base_origin = joint_specs[revolute_joints[0]].origin_xyz.copy()
            max_reach = 0.0
            for joint_name in revolute_joints[1:]:
                max_reach += float(np.linalg.norm(joint_specs[joint_name].origin_xyz))
            max_reach += float(np.linalg.norm(joint_specs[tip_joint_name].origin_xyz))

            finger_chains[finger_name] = FingerChain(
                name=finger_name,
                prefix=prefix,
                revolute_joint_names=revolute_joints,
                tip_joint_name=tip_joint_name,
                base_origin=base_origin,
                max_reach=max_reach,
            )
            revolute_joint_order.extend(revolute_joints)

        revolute_joint_order.sort(
            key=lambda name: (int(name.split("-")[0]), int(name.split("-")[1]))
        )

        return cls(
            joint_specs=joint_specs,
            finger_chains=finger_chains,
            revolute_joint_order=revolute_joint_order,
        )

    def clamp_joint_value(self, joint_name: str, value: float) -> float:
        spec = self.joint_specs[joint_name]
        if spec.joint_type != "revolute":
            return value
        return clamp(value, spec.lower, spec.upper)

    def joint_local_transform(self, spec: JointSpec, joint_angle: float = 0.0) -> np.ndarray:
        base_rotation = rpy_to_rotation(*spec.origin_rpy)
        transform = make_transform(base_rotation, spec.origin_xyz)

        if spec.joint_type == "revolute":
            rotation_about_axis = axis_angle_rotation(spec.axis, joint_angle)
            transform = transform @ make_transform(rotation_about_axis, np.zeros(3, dtype=float))

        return transform

    def forward_tip_position(
        self,
        finger_name: str,
        joint_angles: dict[str, float],
    ) -> np.ndarray:
        chain = self.finger_chains[finger_name]
        transform = np.eye(4, dtype=float)

        for joint_name in chain.revolute_joint_names:
            spec = self.joint_specs[joint_name]
            angle = float(joint_angles.get(joint_name, 0.0))
            transform = transform @ self.joint_local_transform(spec, angle)

        tip_spec = self.joint_specs[chain.tip_joint_name]
        transform = transform @ self.joint_local_transform(tip_spec, 0.0)
        return transform[:3, 3]
