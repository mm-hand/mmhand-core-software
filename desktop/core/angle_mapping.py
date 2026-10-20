"""angle_mapping.py - MMHand Desktop 模块契约。

职责：统一 URDF、标定坐标和下位机角度空间之间的换算。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""

from __future__ import annotations

import json
import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

from desktop.paths import (
    CALIB_LOAD_SNAPSHOT_PATH as DEFAULT_CALIB_LOAD_SNAPSHOT_PATH,
    URDF_JOINT_MAPPING_PATH as DEFAULT_URDF_JOINT_MAPPING_PATH,
)
from .calibration_snapshot import validate_snapshot

from .protocol import ENCODER_COUNT


CALIB_LOAD_SNAPSHOT_PATH = Path(DEFAULT_CALIB_LOAD_SNAPSHOT_PATH)
URDF_JOINT_MAPPING_PATH = Path(DEFAULT_URDF_JOINT_MAPPING_PATH)


def _urdf_mapping_metadata(
    version_key: str,
    label: str,
    mapping_path: Path | str,
) -> tuple[int, int]:
    data = json.loads(Path(mapping_path).read_text(encoding="utf-8"))
    version = int(data.get(version_key, 0))
    joints = data.get("joints")
    if version <= 0 or not isinstance(joints, list) or len(joints) != ENCODER_COUNT:
        raise AngleMappingError(f"invalid {label} mapping metadata")

    payload = bytearray(struct.pack(">BB", version & 0xFF, len(joints) & 0xFF))
    for expected_index, item in enumerate(joints):
        if not isinstance(item, dict) or int(item.get("joint_index", -1)) != expected_index:
            raise AngleMappingError(f"invalid {label} joint index J{expected_index:02d}")
        limits = item.get("urdf_limit_deg")
        if not isinstance(limits, list) or len(limits) != 2:
            raise AngleMappingError(f"invalid {label} limits J{expected_index:02d}")
        lower, upper = float(limits[0]), float(limits[1])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise AngleMappingError(f"invalid {label} range J{expected_index:02d}")
        payload.extend(struct.pack(">Bff", expected_index, lower, upper))
    return version, zlib.crc32(payload) & 0xFFFFFFFF


def urdf_feedback_mapping_metadata(
    mapping_path: Path | str = URDF_JOINT_MAPPING_PATH,
) -> tuple[int, int]:
    """Return the generated-P4 runtime mapping version and canonical CRC32."""
    return _urdf_mapping_metadata(
        "feedback_mapping_version", "URDF feedback", mapping_path
    )


def urdf_manual_scaled_mapping_metadata(
    mapping_path: Path | str = URDF_JOINT_MAPPING_PATH,
) -> tuple[int, int]:
    """Return the V4 Manual-to-URDF mapping version and canonical CRC32."""
    return _urdf_mapping_metadata(
        "manual_scaled_mapping_version", "manual-scaled", mapping_path
    )


@dataclass(frozen=True)
class JointAngleSpec:
    joint_index: int
    joint_name: str
    urdf_joint_name: str
    urdf_min_rad: float
    urdf_max_rad: float
    default_calib_min_deg: float
    default_calib_max_deg: float
    urdf_at_calib_min_rad: Optional[float] = None
    urdf_at_calib_max_rad: Optional[float] = None
    sign: int = 1
    offset_deg: float = 0.0
    is_coupled: bool = False
    motor_indices: tuple[int, ...] = ()
    mapping_source: str = "current host linear framework"
    mapping_status: str = "legacy"

    @property
    def urdf_min_deg(self) -> float:
        return math.degrees(self.urdf_min_rad)

    @property
    def urdf_max_deg(self) -> float:
        return math.degrees(self.urdf_max_rad)

    @property
    def default_calib_range_deg(self) -> tuple[float, float]:
        return (self.default_calib_min_deg, self.default_calib_max_deg)

    @property
    def urdf_range_rad(self) -> tuple[float, float]:
        return (self.urdf_min_rad, self.urdf_max_rad)

    @property
    def urdf_range_deg(self) -> tuple[float, float]:
        return (self.urdf_min_deg, self.urdf_max_deg)

    @property
    def urdf_at_calib_min_deg(self) -> float:
        value = self.urdf_min_rad if self.urdf_at_calib_min_rad is None else self.urdf_at_calib_min_rad
        return math.degrees(value)

    @property
    def urdf_at_calib_max_deg(self) -> float:
        value = self.urdf_max_rad if self.urdf_at_calib_max_rad is None else self.urdf_at_calib_max_rad
        return math.degrees(value)

    @property
    def calibrated_urdf_endpoints_deg(self) -> tuple[float, float]:
        """URDF degrees corresponding to default calibrated min/max respectively."""
        return (self.urdf_at_calib_min_deg, self.urdf_at_calib_max_deg)


def _load_structure_urdf_joint_specs() -> Dict[int, JointAngleSpec]:
    """从统一生成的映射文件加载 21 关节全部运行时元数据。"""
    try:
        data = json.loads(URDF_JOINT_MAPPING_PATH.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"missing URDF joint mapping: {URDF_JOINT_MAPPING_PATH}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"invalid URDF joint mapping JSON: {URDF_JOINT_MAPPING_PATH}") from exc

    if not isinstance(data, dict) or int(data.get("schema_version", 0)) != 4:
        raise RuntimeError("unsupported URDF joint mapping schema")
    entries = data.get("joints")
    if not isinstance(entries, list):
        raise RuntimeError("URDF joint mapping must contain a joints list")

    by_index: Dict[int, dict] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise RuntimeError("URDF joint mapping entry must be an object")
        try:
            idx = int(entry["joint_index"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("URDF joint mapping entry has invalid joint_index") from exc
        if idx in by_index:
            raise RuntimeError(f"duplicate URDF joint mapping entry J{idx:02d}")
        by_index[idx] = entry

    expected_indices = set(range(ENCODER_COUNT))
    if set(by_index) != expected_indices:
        missing = sorted(expected_indices - set(by_index))
        extra = sorted(set(by_index) - expected_indices)
        raise RuntimeError(f"URDF joint mapping indices mismatch: missing={missing}, extra={extra}")

    specs: Dict[int, JointAngleSpec] = {}
    used_motors: set[int] = set()
    for idx in range(ENCODER_COUNT):
        entry = by_index[idx]
        try:
            joint_name = str(entry["display_name"])
            urdf_joint_name = str(entry["urdf_joint_name"])
            urdf_lo_deg, urdf_hi_deg = [float(v) for v in entry["urdf_limit_deg"]]
            calib_lo_deg, calib_hi_deg = [float(v) for v in entry["calibrated_default_range_deg"]]
            urdf_at_calib_min_deg = float(entry["urdf_at_calibrated_min_deg"])
            urdf_at_calib_max_deg = float(entry["urdf_at_calibrated_max_deg"])
            motor_indices = tuple(int(value) for value in entry["motor_indices"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"URDF joint mapping J{idx:02d} has invalid numeric fields") from exc
        if not joint_name or not urdf_joint_name or len(motor_indices) not in (1, 2):
            raise RuntimeError(f"URDF joint mapping J{idx:02d} has invalid semantic fields")
        if any(motor < 0 or motor >= 27 or motor in used_motors for motor in motor_indices):
            raise RuntimeError(f"URDF joint mapping J{idx:02d} has invalid motor bindings")
        used_motors.update(motor_indices)
        values = (urdf_lo_deg, urdf_hi_deg, calib_lo_deg, calib_hi_deg, urdf_at_calib_min_deg, urdf_at_calib_max_deg)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"URDF joint mapping J{idx:02d} contains a non-finite value")
        if abs(urdf_hi_deg - urdf_lo_deg) < 1e-6 or abs(calib_hi_deg - calib_lo_deg) < 1e-6:
            raise RuntimeError(f"URDF joint mapping J{idx:02d} has a zero-size range")
        if not min(urdf_lo_deg, urdf_hi_deg) <= urdf_at_calib_min_deg <= max(urdf_lo_deg, urdf_hi_deg):
            raise RuntimeError(f"URDF joint mapping J{idx:02d} calibrated-min endpoint is outside URDF limits")
        if not min(urdf_lo_deg, urdf_hi_deg) <= urdf_at_calib_max_deg <= max(urdf_lo_deg, urdf_hi_deg):
            raise RuntimeError(f"URDF joint mapping J{idx:02d} calibrated-max endpoint is outside URDF limits")
        specs[idx] = JointAngleSpec(
            joint_index=idx,
            joint_name=joint_name,
            urdf_joint_name=urdf_joint_name,
            urdf_min_rad=math.radians(urdf_lo_deg),
            urdf_max_rad=math.radians(urdf_hi_deg),
            default_calib_min_deg=calib_lo_deg,
            default_calib_max_deg=calib_hi_deg,
            urdf_at_calib_min_rad=math.radians(urdf_at_calib_min_deg),
            urdf_at_calib_max_rad=math.radians(urdf_at_calib_max_deg),
            is_coupled=bool(entry.get("is_coupled", False)),
            motor_indices=motor_indices,
            mapping_source=str(entry.get("mapping_source", data.get("mapping_id", "structure URDF"))),
            mapping_status=str(entry.get("mapping_status", "unverified")),
        )
    if used_motors != set(range(27)):
        raise RuntimeError("URDF joint mapping does not bind every motor M00-M26")
    return specs


JOINT_SPECS = _load_structure_urdf_joint_specs()

JOINT_NAME_TO_INDEX: Dict[str, int] = {}
for _idx, _spec in JOINT_SPECS.items():
    JOINT_NAME_TO_INDEX[_spec.joint_name] = _idx
    JOINT_NAME_TO_INDEX[_spec.urdf_joint_name] = _idx
    JOINT_NAME_TO_INDEX[f"J{_idx:02d}"] = _idx
    JOINT_NAME_TO_INDEX[str(_idx)] = _idx



class AngleMappingError(ValueError):
    pass


def resolve_joint(joint: int | str) -> int:
    if isinstance(joint, int):
        idx = joint
    else:
        key = str(joint)
        if key not in JOINT_NAME_TO_INDEX:
            raise KeyError(f"unknown joint name/index: {joint!r}")
        idx = JOINT_NAME_TO_INDEX[key]
    if idx not in JOINT_SPECS:
        raise KeyError(f"unknown joint index J{idx:02d}")
    return idx


def get_spec(joint: int | str) -> JointAngleSpec:
    return JOINT_SPECS[resolve_joint(joint)]


def _has_consistent_auto_raw_endpoints(item: dict) -> bool:
    try:
        direction = -1 if int(item.get("encoderDirection", 1)) < 0 else 1
        start_oriented = int(item.get("startEncoder", 0)) % 16384
        tight_oriented = int(item.get("tightEncoder", 0)) % 16384
        auto_min_raw = int(item.get("autoMinRaw", -1))
        auto_max_raw = int(item.get("autoMaxRaw", -1))
    except (TypeError, ValueError):
        return False
    if not 0 <= auto_min_raw < 16384 or not 0 <= auto_max_raw < 16384:
        return False
    expected_min = (-start_oriented) % 16384 if direction < 0 else start_oriented
    expected_max = (-tight_oriented) % 16384 if direction < 0 else tight_oriented
    return auto_min_raw == expected_min and auto_max_raw == expected_max


def load_calibrated_ranges(
    snapshot_path: Path | str | None = None,
) -> Dict[int, tuple[float, float]]:
    # Device ownership belongs to the backend. Pure mapping has no device identity
    # and must never infer calibration from the legacy global file.
    if snapshot_path is None:
        return {}
    path = Path(snapshot_path)

    ranges: Dict[int, tuple[float, float]] = {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        validate_snapshot(data)
        results = data.get("joint_calib_results", [])
        expected_version, expected_crc = urdf_manual_scaled_mapping_metadata()
    except Exception:
        results = []
        expected_version, expected_crc = 0, 0

    if isinstance(results, list):
        for idx, item in enumerate(results[:ENCODER_COUNT]):
            if not isinstance(item, dict) or not item.get("success"):
                continue
            try:
                mapping_version = int(item.get("mappingVersion", 0) or 0)
                mapping_crc = int(item.get("urdfConfigCrc", 0) or 0)
                angle_min = float(item.get("angleMin", 0.0))
                angle_max = float(item.get("angleMax", 0.0))
            except (TypeError, ValueError):
                continue
            urdf_min, urdf_max = JOINT_SPECS[idx].urdf_range_deg
            # 旧快照多余的 currentMinDeg/currentMaxDeg 字段直接忽略：host 只校验
            # 静态 URDF 契约（mapping 版本/CRC 与 angleMin/angleMax）。
            if (
                mapping_version == expected_version
                and mapping_crc == expected_crc
                and math.isfinite(angle_min)
                and math.isfinite(angle_max)
                and abs(angle_min - urdf_min) <= 0.01
                and abs(angle_max - urdf_max) <= 0.01
                and _has_consistent_auto_raw_endpoints(item)
            ):
                # V4 快照只证明该关节已有有效 Auto/LUT；实际命令范围始终取
                # 当前 URDF 静态上下限，不能让历史 Auto Span 再次改变比例。
                ranges[idx] = (urdf_min, urdf_max)

    return ranges


def calibrated_range_deg(joint: int | str) -> Optional[tuple[float, float]]:
    return load_calibrated_ranges().get(resolve_joint(joint))


def default_calibrated_range_deg(joint: int | str) -> tuple[float, float]:
    return get_spec(joint).default_calib_range_deg


def effective_calibrated_range_deg(
    joint: int | str,
    *,
    allow_default: bool = True,
) -> Optional[tuple[float, float]]:
    rng = calibrated_range_deg(joint)
    if rng is not None:
        return rng
    return default_calibrated_range_deg(joint) if allow_default else None


def urdf_to_calibrated(
    joint: int | str,
    q_urdf_rad: float,
    *,
    allow_default: bool = True,
    clamp: bool = True,
) -> float:
    """URDF rad -> calibrated/hardware command degrees.

    This is a temporary linear mapping framework. The measured calibration
    range requires an explicitly selected device snapshot. If no
    calibrated range is available and allow_default is False, this raises.
    """
    idx = resolve_joint(joint)
    spec = JOINT_SPECS[idx]
    q_urdf_clamped = clamp_urdf_for_calibrated_mapping(idx, q_urdf_rad) if clamp else float(q_urdf_rad)
    q_urdf_deg = math.degrees(q_urdf_clamped)
    urdf_at_min, urdf_at_max = spec.calibrated_urdf_endpoints_deg
    default_deg = _linear_map(
        q_urdf_deg,
        urdf_at_min,
        urdf_at_max,
        spec.default_calib_min_deg,
        spec.default_calib_max_deg,
    )

    calib = calibrated_range_deg(idx)
    if calib is None:
        if not allow_default:
            raise AngleMappingError(f"J{idx:02d} has no calibrated range")
        q_calib = default_deg
        calib = spec.default_calib_range_deg
    else:
        q_calib = _linear_map(
            default_deg,
            spec.default_calib_min_deg,
            spec.default_calib_max_deg,
            calib[0],
            calib[1],
        )

    q_calib = spec.sign * q_calib + spec.offset_deg
    if clamp:
        q_calib = clamp_calibrated(idx, q_calib, allow_default=allow_default)
    return q_calib


def calibrated_to_urdf(
    joint: int | str,
    q_calibrated_deg: float,
    *,
    allow_default: bool = True,
    clamp: bool = True,
) -> float:
    """Calibrated/hardware command degrees -> URDF rad."""
    idx = resolve_joint(joint)
    spec = JOINT_SPECS[idx]
    q = (float(q_calibrated_deg) - spec.offset_deg) / spec.sign
    if clamp:
        q = clamp_calibrated(idx, q, allow_default=allow_default)

    calib = calibrated_range_deg(idx)
    if calib is None:
        if not allow_default:
            raise AngleMappingError(f"J{idx:02d} has no calibrated range")
        default_deg = q
    else:
        default_deg = _linear_map(
            q,
            calib[0],
            calib[1],
            spec.default_calib_min_deg,
            spec.default_calib_max_deg,
        )

    urdf_at_min, urdf_at_max = spec.calibrated_urdf_endpoints_deg
    q_urdf_deg = _linear_map(
        default_deg,
        spec.default_calib_min_deg,
        spec.default_calib_max_deg,
        urdf_at_min,
        urdf_at_max,
    )
    q_urdf = math.radians(q_urdf_deg)
    return clamp_urdf(idx, q_urdf) if clamp else q_urdf


def clamp_urdf(joint: int | str, q_urdf_rad: float) -> float:
    spec = get_spec(joint)
    lo, hi = sorted(spec.urdf_range_rad)
    return max(lo, min(hi, float(q_urdf_rad)))


def clamp_urdf_for_calibrated_mapping(joint: int | str, q_urdf_rad: float) -> float:
    """Clamp only to the active URDF hard limits before command conversion.

    Old code special-cased MCP/PIP/DIP as negative-angle flexion and discarded
    their positive URDF range. The structure URDF stores each joint's physical
    command endpoints explicitly, so every joint now follows one rule.
    """
    return clamp_urdf(joint, q_urdf_rad)


def clamp_calibrated(
    joint: int | str,
    q_calibrated_deg: float,
    *,
    allow_default: bool = True,
) -> float:
    rng = effective_calibrated_range_deg(joint, allow_default=allow_default)
    if rng is None:
        raise AngleMappingError(f"J{resolve_joint(joint):02d} has no calibrated range")
    lo, hi = sorted(rng)
    return max(lo, min(hi, float(q_calibrated_deg)))




def _linear_map(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    denom = src_hi - src_lo
    if abs(denom) < 1e-8:
        return dst_lo
    ratio = (float(value) - src_lo) / denom
    return dst_lo + ratio * (dst_hi - dst_lo)


__all__ = [
    "AngleMappingError",
    "JOINT_SPECS",
    "JointAngleSpec",
    "URDF_JOINT_MAPPING_PATH",
    "calibrated_range_deg",
    "calibrated_to_urdf",
    "clamp_calibrated",
    "clamp_urdf",
    "effective_calibrated_range_deg",
    "get_spec",
    "load_calibrated_ranges",
    "resolve_joint",
    "urdf_to_calibrated",
    "urdf_feedback_mapping_metadata",
    "urdf_manual_scaled_mapping_metadata",
]
