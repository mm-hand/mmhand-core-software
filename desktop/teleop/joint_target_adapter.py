"""joint_target_adapter.py - MMHand Desktop 模块契约。

职责：把不同遥操来源统一转换到 J00-J20 URDF 角度空间。
线程：接收器可在 SDK/摄像头工作线程执行；纯映射和数学函数不创建线程。
输入：手套、骨架或视觉关键点，以及活动 URDF/标定配置。
输出：统一 J00-J20 目标、接收状态和可诊断质量信息。
状态所有权：接收器拥有最近输入帧；角度映射和模型配置按只读快照使用。
禁止依赖：禁止依赖 Qt/UI 或直接写串口，目标必须经过统一角度适配和后端门禁。
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
LEGACY_CALIBRATED_DEG = "legacy_calibrated_deg"
SUPPORTED_TARGET_SPACES = (URDF_DEG, URDF_RAD, LEGACY_CALIBRATED_DEG)


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


_LEGACY_JOINT_ALIASES = {
    **{f"4-{offset + 1}": offset for offset in range(4)},
    **{f"3-{offset + 1}": 4 + offset for offset in range(4)},
    **{f"2-{offset + 1}": 8 + offset for offset in range(4)},
    **{f"1-{offset + 1}": 12 + offset for offset in range(4)},
    "5-1": 20,
    "5-2": 16,
    "5-3": 17,
    "5-4": 18,
    "5-5": 19,
}


@lru_cache(maxsize=1)
def active_mapping_identity() -> UrdfMappingIdentity:
    data = json.loads(angle_mapping.URDF_JOINT_MAPPING_PATH.read_text(encoding="utf-8"))
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
        "legacy": LEGACY_CALIBRATED_DEG,
        "calibrated": LEGACY_CALIBRATED_DEG,
        "q_target": LEGACY_CALIBRATED_DEG,
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
    """Return a validated frame in active-URDF degrees.

    Dynamic Current Min/URDF Max limiting intentionally remains in
    HandBackend; this adapter only resolves coordinate systems and units.
    """
    targets = _validated_targets(values)
    space = normalize_target_space(source_space)
    if space == URDF_DEG:
        urdf_targets = list(targets)
    elif space == URDF_RAD:
        urdf_targets = [math.degrees(value) for value in targets]
    else:
        urdf_targets = [
            math.degrees(
                angle_mapping.legacy_calibrated_to_urdf(
                    idx,
                    value,
                    clamp=True,
                )
            )
            for idx, value in enumerate(targets)
        ]
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
    if name in _LEGACY_JOINT_ALIASES:
        return int(_LEGACY_JOINT_ALIASES[name])
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
    "LEGACY_CALIBRATED_DEG",
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
