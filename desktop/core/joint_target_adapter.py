"""joint_target_adapter.py - MMHand Desktop 模块契约。

职责：把外部关节目标统一转换到 J00-J20 的活动 URDF 角度空间。
线程：纯映射和数学函数不创建线程，可由 ROS2 executor 或应用线程调用。
输入：外部 ROS2 节点提供的关节角、活动 URDF/标定配置。
输出：统一 J00-J20 目标、映射版本和可诊断质量信息。
状态所有权：只读映射配置按缓存使用，不拥有设备状态或外部输入连接。
禁止依赖：禁止依赖 Qt/UI、任何外部设备 SDK 或直接写串口。
"""

from __future__ import annotations

import json
import math
import re
import zlib
from dataclasses import dataclass
from functools import lru_cache
from typing import Iterable, Mapping

from desktop.core import angle_mapping
from desktop.core.protocol import ENCODER_COUNT


URDF_DEG = "urdf_deg"
URDF_RAD = "urdf_rad"
SUPPORTED_TARGET_SPACES = (URDF_DEG, URDF_RAD)


@dataclass(frozen=True)
class UrdfMappingIdentity:
    mapping_id: str
    mapping_version: int
    urdf_config_crc: int
    joint_order_crc: int


@dataclass(frozen=True)
class JointTargetAdaptation:
    source_space: str
    source_targets: tuple[float, ...]
    urdf_targets_deg: tuple[float, ...]
    mapping: UrdfMappingIdentity


@dataclass(frozen=True)
class NamedJointTargetAdaptation:
    urdf_targets_deg: tuple[float, ...]
    mapped_indices: tuple[int, ...]
    unknown_names: tuple[str, ...]


@lru_cache(maxsize=1)
def active_mapping_identity() -> UrdfMappingIdentity:
    from .robot_model import mapping as _robot_mapping
    data = _robot_mapping()
    version, crc = angle_mapping.urdf_manual_scaled_mapping_metadata()
    entries = list(data.get("joints") or [])
    entries_by_index = {
        int(entry.get("joint_index", -1)): entry
        for entry in entries
    }
    expected_indices = set(range(ENCODER_COUNT))
    if len(entries) != ENCODER_COUNT or set(entries_by_index) != expected_indices:
        raise ValueError(
            "URDF joint order must define each J00-J20 index exactly once"
        )
    order_payload = [
        {
            "joint_index": joint,
            "urdf_joint_name": str(entry.get("urdf_joint_name", "")),
        }
        for joint, entry in sorted(entries_by_index.items())
    ]
    # 只对 J 索引到 URDF 名称的语义映射签名，JSON排列和角度范围变化不会改变该 CRC。
    order_bytes = json.dumps(
        order_payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return UrdfMappingIdentity(
        mapping_id=str(data.get("mapping_id", "active_urdf")),
        mapping_version=int(version),
        urdf_config_crc=int(crc) & 0xFFFFFFFF,
        joint_order_crc=zlib.crc32(order_bytes) & 0xFFFFFFFF,
    )


def normalize_target_space(source_space: str) -> str:
    value = str(source_space or "").strip().lower()
    aliases = {
        "urdf": URDF_DEG,
        "deg": URDF_DEG,
        "degrees": URDF_DEG,
        "rad": URDF_RAD,
        "radians": URDF_RAD,
    }
    value = aliases.get(value, value)
    if value not in SUPPORTED_TARGET_SPACES:
        raise ValueError(
            f"unsupported joint target space {source_space!r}; expected one of "
            f"{', '.join(SUPPORTED_TARGET_SPACES)}"
        )
    return value


def _validated_targets(values: Iterable[float]) -> list[float]:
    try:
        targets = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise ValueError("joint targets must be numeric") from exc
    if len(targets) != ENCODER_COUNT:
        raise ValueError(f"expected {ENCODER_COUNT} joint targets, got {len(targets)}")
    if not all(math.isfinite(value) for value in targets):
        raise ValueError("joint targets must be finite")
    return targets


def adapt_joint_targets(
    values: Iterable[float],
    *,
    source_space: str,
) -> JointTargetAdaptation:
    """返回已校验、统一到活动 URDF 度数空间的一帧关节目标。

    动态 Current Min/URDF Max 约束仍由 HandBackend 负责；此处只处理坐标空间与单位。
    """
    targets = _validated_targets(values)
    space = normalize_target_space(source_space)
    if space == URDF_DEG:
        urdf_targets = list(targets)
    else:
        urdf_targets = [math.degrees(value) for value in targets]
    return JointTargetAdaptation(
        source_space=space,
        source_targets=tuple(targets),
        urdf_targets_deg=tuple(urdf_targets),
        mapping=active_mapping_identity(),
    )


def joint_index_for_urdf_name(joint_name: str) -> int | None:
    name = str(joint_name or "").strip()
    if name in angle_mapping.JOINT_NAME_TO_INDEX:
        return int(angle_mapping.JOINT_NAME_TO_INDEX[name])
    match = re.fullmatch(r"J(\d{2})", name, flags=re.IGNORECASE)
    if match:
        idx = int(match.group(1))
        return idx if 0 <= idx < ENCODER_COUNT else None
    return None


def named_urdf_radians_to_joint_targets(
    joint_angles: Mapping[str, float],
) -> NamedJointTargetAdaptation:
    targets = [0.0] * ENCODER_COUNT
    mapped: set[int] = set()
    unknown: list[str] = []
    for name, angle_rad in joint_angles.items():
        idx = joint_index_for_urdf_name(str(name))
        if idx is None:
            unknown.append(str(name))
            continue
        value = float(angle_rad)
        if not math.isfinite(value):
            raise ValueError(f"non-finite URDF angle for {name}")
        targets[idx] = math.degrees(value)
        mapped.add(idx)
    return NamedJointTargetAdaptation(
        urdf_targets_deg=tuple(targets),
        mapped_indices=tuple(sorted(mapped)),
        unknown_names=tuple(unknown),
    )


__all__ = [
    "SUPPORTED_TARGET_SPACES",
    "URDF_DEG",
    "URDF_RAD",
    "JointTargetAdaptation",
    "NamedJointTargetAdaptation",
    "UrdfMappingIdentity",
    "active_mapping_identity",
    "adapt_joint_targets",
    "joint_index_for_urdf_name",
    "named_urdf_radians_to_joint_targets",
    "normalize_target_space",
]
