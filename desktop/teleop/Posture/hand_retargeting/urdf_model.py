"""urdf_model.py - MMHand Desktop 模块契约。

职责：解析活动 URDF 并提供轻量前向运动学模型。
线程：接收器可在 SDK/摄像头工作线程执行；纯映射和数学函数不创建线程。
输入：手套、骨架或视觉关键点，以及活动 URDF/标定配置。
输出：统一 J00-J20 目标、接收状态和可诊断质量信息。
状态所有权：接收器拥有最近输入帧；角度映射和模型配置按只读快照使用。
禁止依赖：禁止依赖 Qt/UI 或直接写串口，目标必须经过统一角度适配和后端门禁。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .constants import ROBOT_FINGER_PREFIX
from .math_utils import axis_angle_rotation, clamp, make_transform, parse_xyz, rpy_to_rotation


@dataclass
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
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
    tip_joint_name: str | None
    tip_link_name: str | None
    base_transform: np.ndarray
    base_origin: np.ndarray
    max_reach: float


class URDFRobotHandModel:
    """URDF 机械手模型，提供关节信息与简化正向运动学。"""

    def __init__(
        self,
        joint_specs: dict[str, JointSpec],
        finger_chains: dict[str, FingerChain],
        revolute_joint_order: list[str],
        link_inertial_origins: dict[str, np.ndarray] | None = None,
        visual_contact_tip_offsets: dict[str, np.ndarray] | None = None,
    ) -> None:
        self.joint_specs = joint_specs
        self.finger_chains = finger_chains
        self.revolute_joint_order = revolute_joint_order
        self.link_inertial_origins = dict(link_inertial_origins or {})
        self.visual_contact_tip_offsets = dict(visual_contact_tip_offsets or {})

    @classmethod
    def from_urdf(cls, urdf_path: Path) -> "URDFRobotHandModel":
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        root = ET.parse(urdf_path).getroot()
        joint_specs: dict[str, JointSpec] = {}
        link_inertial_origins: dict[str, np.ndarray] = {}
        visual_contact_tip_offsets = cls._visual_contact_tip_offsets_from_urdf(root, urdf_path)

        for link_elem in root.findall("link"):
            link_name = link_elem.attrib.get("name", "")
            inertial_origin = link_elem.find("inertial/origin")
            if not link_name or inertial_origin is None:
                continue
            link_inertial_origins[link_name] = parse_xyz(inertial_origin.attrib.get("xyz", "0 0 0"))

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

            parent_elem = joint_elem.find("parent")
            child_elem = joint_elem.find("child")
            parent = parent_elem.attrib.get("link", "") if parent_elem is not None else ""
            child = child_elem.attrib.get("link", "") if child_elem is not None else ""

            joint_specs[name] = JointSpec(
                name=name,
                joint_type=joint_type,
                parent=parent,
                child=child,
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
                tip_link_name=joint_specs[tip_joint_name].child,
                base_transform=np.eye(4, dtype=float),
                base_origin=base_origin,
                max_reach=max_reach,
            )
            revolute_joint_order.extend(revolute_joints)

        if not finger_chains:
            finger_chains, revolute_joint_order = cls._build_semantic_mmhand_chains(
                joint_specs,
                link_inertial_origins,
                visual_contact_tip_offsets,
            )
        else:
            revolute_joint_order.sort(
                key=lambda name: (int(name.split("-")[0]), int(name.split("-")[1]))
            )

        return cls(
            joint_specs=joint_specs,
            finger_chains=finger_chains,
            revolute_joint_order=revolute_joint_order,
            link_inertial_origins=link_inertial_origins,
            visual_contact_tip_offsets=visual_contact_tip_offsets,
        )

    @staticmethod
    def _visual_contact_tip_offsets_from_urdf(root: ET.Element, urdf_path: Path) -> dict[str, np.ndarray]:
        out: dict[str, np.ndarray] = {}
        mesh_root = urdf_path.parent
        for link_elem in root.findall("link"):
            link_name = link_elem.attrib.get("name", "")
            if "fingertip" not in link_name:
                continue
            visual = link_elem.find("visual")
            if visual is None:
                continue
            origin_elem = visual.find("origin")
            mesh_elem = visual.find("geometry/mesh")
            if mesh_elem is None:
                continue
            origin = (
                parse_xyz(origin_elem.attrib.get("xyz", "0 0 0"))
                if origin_elem is not None
                else np.zeros(3, dtype=float)
            )
            scale = parse_xyz(mesh_elem.attrib.get("scale", "1 1 1"))
            mesh_file = mesh_elem.attrib.get("filename", "")
            mesh_path = (mesh_root / mesh_file).resolve()
            if not mesh_path.exists():
                continue
            vertices = URDFRobotHandModel._read_binary_stl_vertices(mesh_path)
            if vertices.size == 0:
                continue
            local_vertices = origin + vertices * scale
            axis = 2 if "thumb" in link_name else 0
            index = int(np.argmin(local_vertices[:, axis]) if axis == 2 else np.argmax(local_vertices[:, axis]))
            out[link_name] = local_vertices[index].astype(float)
        return out

    @staticmethod
    def _read_binary_stl_vertices(path: Path) -> np.ndarray:
        data = path.read_bytes()
        if len(data) < 84:
            return np.zeros((0, 3), dtype=float)
        tri_count = struct.unpack("<I", data[80:84])[0]
        expected = 84 + tri_count * 50
        if expected > len(data):
            return np.zeros((0, 3), dtype=float)
        vertices: list[tuple[float, float, float]] = []
        offset = 84
        for _ in range(tri_count):
            values = struct.unpack("<12fH", data[offset : offset + 50])
            vertices.extend((values[3:6], values[6:9], values[9:12]))
            offset += 50
        return np.asarray(vertices, dtype=float)

    @classmethod
    def _build_semantic_mmhand_chains(
        cls,
        joint_specs: dict[str, JointSpec],
        link_inertial_origins: dict[str, np.ndarray],
        visual_contact_tip_offsets: dict[str, np.ndarray],
    ) -> tuple[dict[str, FingerChain], list[str]]:
        # 新版 mmhand URDF 使用语义 joint 名，不再使用旧版 "1-1" 这种命名。
        # 四指链路都先经过 base_link -> finger_base 的 fixed transform，
        # 再依次经过 AA/MCP/PIP/DIP 转动关节；DIP 后就是 fingertip link。
        semantic = {
            "index": (
                "1",
                (
                    ("base_link_1_finger_base_1_fixed_joint",),
                    ("palm_1_fixed_joint", "palm_1_finger_base_1_fixed_joint"),
                    ("palm_fix_joint", "palm_1_finger_base_1_fixed_joint"),
                ),
                [
                    "index_aa_joint",
                    "mmhand_finger_1_proximal_phalanx_1_mcp_fe_joint",
                    "mmhand_finger_1_distal_phalanx_1_pip_joint",
                    "mmhand_finger_1_fingertip_1_dip_joint",
                ],
            ),
            "middle": (
                "2",
                (
                    ("base_link_1_finger_base_2_fixed_joint",),
                    ("palm_1_fixed_joint", "palm_1_finger_base_2_fixed_joint"),
                    ("palm_fix_joint", "palm_1_finger_base_2_fixed_joint"),
                ),
                [
                    "middle_aa_joint",
                    "mmhand_finger_2_proximal_phalanx_1_mcp_fe_joint",
                    "mmhand_finger_2_distal_phalanx_1_pip_joint",
                    "mmhand_finger_2_fingertip_1_dip_joint",
                ],
            ),
            "ring": (
                "3",
                (
                    ("base_link_1_finger_base_3_fixed_joint",),
                    ("palm_1_fixed_joint", "palm_1_finger_base_3_fixed_joint"),
                    ("palm_fix_joint", "palm_1_finger_base_3_fixed_joint"),
                ),
                [
                    "ring_aa_joint",
                    "mmhand_finger_3_proximal_phalanx_1_mcp_fe_joint",
                    "mmhand_finger_3_distal_phalanx_1_pip_joint",
                    "mmhand_finger_3_fingertip_1_dip_joint",
                ],
            ),
            "pinky": (
                "4",
                (
                    # 当前活动 URDF 将小拇指根部编号命名为 5；保留旧候选用于解析
                    # 历史结构模型，避免因固定关节命名变化而丢失整条小拇指链。
                    ("palm_1_fixed_joint", "palm_1_finger_base_5_fixed_joint"),
                    ("base_link_1_finger_base_4_fixed_joint",),
                    ("palm_1_fixed_joint", "palm_1_finger_base_4_fixed_joint"),
                    ("palm_fix_joint", "palm_1_finger_base_4_fixed_joint"),
                ),
                [
                    "little_aa_joint",
                    "mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint",
                    "mmhand_finger_4_distal_phalanx_1_pip_joint",
                    "mmhand_finger_4_fingertip_1_dip_joint",
                ],
            ),
            "thumb": (
                "5",
                (
                    ("palm_1_fixed_joint",),
                    ("palm_fix_joint",),
                    (),
                ),
                [
                    "thumb_cmc1_joint",
                    "thumb_aa_joint",
                    "mmhand_thumb_1_mmhand_finger_5_proximal_phalanx_1_mcp_fe_joint",
                    "mmhand_thumb_1_mmhand_finger_5_distal_phalanx_1_pip_joint",
                    "mmhand_thumb_1_mmhand_finger_5_fingertip_1_dip_joint",
                ],
            ),
        }

        finger_chains: dict[str, FingerChain] = {}
        for finger_name, (prefix, fixed_joint_candidates, revolute_joints) in semantic.items():
            if any(joint_name not in joint_specs for joint_name in revolute_joints):
                continue

            fixed_joint_names: tuple[str, ...] | None = None
            for candidate in fixed_joint_candidates:
                if all(joint_name in joint_specs for joint_name in candidate):
                    fixed_joint_names = candidate
                    break
            if fixed_joint_names is None:
                continue

            base_transform = np.eye(4, dtype=float)
            for fixed_joint_name in fixed_joint_names:
                base_transform = base_transform @ cls._joint_transform_from_specs(joint_specs[fixed_joint_name], 0.0)
            first_transform = base_transform @ cls._joint_transform_from_specs(joint_specs[revolute_joints[0]], 0.0)
            base_origin = first_transform[:3, 3].copy()

            max_reach = 0.0
            for joint_name in revolute_joints[1:]:
                max_reach += float(np.linalg.norm(joint_specs[joint_name].origin_xyz))
            tip_link_name = joint_specs[revolute_joints[-1]].child
            max_reach += float(
                np.linalg.norm(
                    visual_contact_tip_offsets.get(
                        tip_link_name,
                        2.0 * link_inertial_origins.get(tip_link_name, np.zeros(3, dtype=float)),
                    )
                )
            )

            finger_chains[finger_name] = FingerChain(
                name=finger_name,
                prefix=prefix,
                revolute_joint_names=revolute_joints,
                tip_joint_name=None,
                tip_link_name=tip_link_name,
                base_transform=base_transform,
                base_origin=base_origin,
                max_reach=max_reach,
            )

        order: list[str] = []
        for finger_name in ("pinky", "ring", "middle", "index", "thumb"):
            chain = finger_chains.get(finger_name)
            if chain is not None:
                order.extend(chain.revolute_joint_names)
        return finger_chains, order

    @staticmethod
    def _joint_transform_from_specs(spec: JointSpec, joint_angle: float = 0.0) -> np.ndarray:
        base_rotation = rpy_to_rotation(*spec.origin_rpy)
        transform = make_transform(base_rotation, spec.origin_xyz)
        if spec.joint_type == "revolute":
            rotation_about_axis = axis_angle_rotation(spec.axis, joint_angle)
            transform = transform @ make_transform(rotation_about_axis, np.zeros(3, dtype=float))
        return transform

    def _virtual_tip_offset(self, chain: FingerChain) -> np.ndarray:
        if chain.tip_joint_name is not None or not chain.tip_link_name:
            return np.zeros(3, dtype=float)
        contact_offset = self.visual_contact_tip_offsets.get(chain.tip_link_name)
        if contact_offset is not None:
            return contact_offset.copy()
        return 2.0 * self.link_inertial_origins.get(chain.tip_link_name, np.zeros(3, dtype=float))

    def clamp_joint_value(self, joint_name: str, value: float) -> float:
        spec = self.joint_specs[joint_name]
        if spec.joint_type != "revolute":
            return value
        return clamp(value, spec.lower, spec.upper)

    def joint_local_transform(self, spec: JointSpec, joint_angle: float = 0.0) -> np.ndarray:
        return self._joint_transform_from_specs(spec, joint_angle)

    def forward_tip_position(
        self,
        finger_name: str,
        joint_angles: dict[str, float],
    ) -> np.ndarray:
        chain = self.finger_chains[finger_name]
        # 新版 mmhand 的 FK 必须从 fixed finger_base transform 开始。
        # 如果直接从 identity 开始，会漏掉 URDF 里的 base_link -> finger_base 偏移。
        transform = chain.base_transform.copy()

        for joint_name in chain.revolute_joint_names:
            spec = self.joint_specs[joint_name]
            angle = float(joint_angles.get(joint_name, 0.0))
            transform = transform @ self.joint_local_transform(spec, angle)

        if chain.tip_joint_name is not None:
            tip_spec = self.joint_specs[chain.tip_joint_name]
            transform = transform @ self.joint_local_transform(tip_spec, 0.0)
        else:
            tip_offset = self._virtual_tip_offset(chain)
            if float(np.linalg.norm(tip_offset)) > 1e-9:
                transform = transform @ make_transform(np.eye(3, dtype=float), tip_offset)
        return transform[:3, 3]

    def forward_joint_positions(
        self,
        finger_name: str,
        joint_angles: dict[str, float],
    ) -> dict[str, np.ndarray]:
        chain = self.finger_chains[finger_name]
        transform = chain.base_transform.copy()
        positions: dict[str, np.ndarray] = {}

        for joint_name in chain.revolute_joint_names:
            spec = self.joint_specs[joint_name]
            origin_transform = make_transform(rpy_to_rotation(*spec.origin_rpy), spec.origin_xyz)
            transform = transform @ origin_transform
            positions[joint_name] = transform[:3, 3].copy()

            angle = float(joint_angles.get(joint_name, 0.0))
            if spec.joint_type == "revolute":
                rotation_about_axis = axis_angle_rotation(spec.axis, angle)
                transform = transform @ make_transform(rotation_about_axis, np.zeros(3, dtype=float))

        if chain.tip_joint_name is not None:
            tip_spec = self.joint_specs[chain.tip_joint_name]
            transform = transform @ self.joint_local_transform(tip_spec, 0.0)
        else:
            tip_offset = self._virtual_tip_offset(chain)
            if float(np.linalg.norm(tip_offset)) > 1e-9:
                transform = transform @ make_transform(np.eye(3, dtype=float), tip_offset)
        positions["tip"] = transform[:3, 3].copy()
        return positions

    def robot_palm_frame(self) -> tuple[np.ndarray, np.ndarray]:
        # robot 手掌坐标系和 MANUS 手掌坐标系采用同构构造。
        # O_r 是 URDF 的 base_link 坐标系原点。这里数值是 [0, 0, 0]，
        # 因为当前所有 robot 点本来就已经表达在 base_link 坐标系下。
        index = self.finger_chains["index"].base_origin
        middle = self.finger_chains["middle"].base_origin
        ring = self.finger_chains["ring"].base_origin
        pinky = self.finger_chains["pinky"].base_origin

        origin = np.zeros(3, dtype=float)
        center = np.mean(np.stack([index, middle, ring, pinky], axis=0), axis=0)

        # x_r: base_link 原点 -> 四指 root 中心。
        x_axis = center - origin
        x_norm = float(np.linalg.norm(x_axis))
        if x_norm < 1e-8:
            x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            x_axis = x_axis / x_norm

        # y_r: index root -> pinky root，也就是手掌宽度方向。
        y_axis = pinky - index
        y_norm = float(np.linalg.norm(y_axis))
        if y_norm < 1e-8:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            y_axis = y_axis / y_norm

        # z_r: 手掌法向，并按 URDF/base_link 的正法向修正方向。
        z_axis = np.cross(x_axis, y_axis)
        z_norm = float(np.linalg.norm(z_axis))
        if z_norm < 1e-8:
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            z_axis = z_axis / z_norm

        robot_normal = np.array([0.0, 0.0, 1.0], dtype=float)
        if float(np.dot(z_axis, robot_normal)) < 0.0:
            z_axis = -z_axis

        y_axis = np.cross(z_axis, x_axis)
        y_norm = float(np.linalg.norm(y_axis))
        if y_norm < 1e-8:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            y_axis = y_axis / y_norm

        return origin, np.stack([x_axis, y_axis, z_axis], axis=1)

    def base_to_palm_local(self, point_base: np.ndarray) -> np.ndarray:
        origin, rotation = self.robot_palm_frame()
        return (np.asarray(point_base, dtype=float) - origin) @ rotation

    def palm_local_to_base(self, point_local: np.ndarray) -> np.ndarray:
        origin, rotation = self.robot_palm_frame()
        return origin + rotation @ np.asarray(point_local, dtype=float)
