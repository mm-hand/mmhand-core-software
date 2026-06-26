"""Central angle mapping helpers.

Canonical rule:
    MANUS / retarget / IK / FK / viewer internals use URDF joint angles.

Units:
    URDF angle: radians in IK/FK code, degrees only in UI text/JSON streams.
    Viewer angle: URDF degrees. The viewer converts degrees back to radians.
    Calibrated angle: degrees sent to firmware CMD_ANGLE_CTRL.

The calibrated mapping below is a temporary linear framework that mirrors the
current host behavior. It must be verified on hardware. Tendon/coupled joints
may need a non-linear or multi-motor model; they are explicitly marked.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

from protocol import ENCODER_COUNT


DEBUG_ANGLE_MAPPING = False

DESKTOP_DIR = Path(__file__).resolve().parent
CALIB_LOAD_SNAPSHOT_PATH = DESKTOP_DIR / "config" / "calib_load_snapshot.json"


@dataclass(frozen=True)
class JointAngleSpec:
    joint_index: int
    joint_name: str
    urdf_joint_name: str
    urdf_min_rad: float
    urdf_max_rad: float
    default_calib_min_deg: float
    default_calib_max_deg: float
    sign: int = 1
    offset_deg: float = 0.0
    is_coupled: bool = False
    motor_indices: tuple[int, ...] = ()
    mapping_source: str = "current host linear framework"

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


@dataclass(frozen=True)
class AngleMappingDebug:
    joint_name: str
    joint_index: int
    q_ik_urdf_rad: float
    q_ik_urdf_deg: float
    q_viewer_deg: float
    q_calibrated_deg: Optional[float]
    urdf_range_deg: tuple[float, float]
    viewer_range_deg: tuple[float, float]
    calibrated_range_deg: Optional[tuple[float, float]]
    sign: int
    offset_deg: float
    is_clamped_urdf: bool
    is_clamped_calibrated: bool
    mapping_source: str


JOINT_SPECS: Dict[int, JointAngleSpec] = {
    0: JointAngleSpec(0, "Pinky root", "little_aa_joint", -0.802851, 0.331613, -45.0, 11.0, is_coupled=True, motor_indices=(0, 1), mapping_source="coupled tendon joint, temporary linear joint-angle mapping"),
    1: JointAngleSpec(1, "Pinky MCP", "mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint", -1.570796, 0.244346, 0.0, math.degrees(1.570796), motor_indices=(2,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    2: JointAngleSpec(2, "Pinky PIP", "mmhand_finger_4_distal_phalanx_1_pip_joint", -1.53589, 0.15708, 0.0, math.degrees(1.53589), motor_indices=(3,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    3: JointAngleSpec(3, "Pinky DIP", "mmhand_finger_4_fingertip_1_dip_joint", -1.570796, 0.20944, 0.0, math.degrees(1.570796), motor_indices=(4,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    4: JointAngleSpec(4, "Ring root", "ring_aa_joint", -0.802851, 0.506145, -51.0, 21.0, is_coupled=True, motor_indices=(5, 6), mapping_source="coupled tendon joint, temporary linear joint-angle mapping"),
    5: JointAngleSpec(5, "Ring MCP", "mmhand_finger_3_proximal_phalanx_1_mcp_fe_joint", -1.570796, 0.244346, 0.0, math.degrees(1.570796), motor_indices=(7,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    6: JointAngleSpec(6, "Ring PIP", "mmhand_finger_3_distal_phalanx_1_pip_joint", -1.53589, 0.15708, 0.0, math.degrees(1.53589), motor_indices=(8,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    7: JointAngleSpec(7, "Ring DIP", "mmhand_finger_3_fingertip_1_dip_joint", -1.570796, 0.20944, 0.0, math.degrees(1.570796), motor_indices=(9,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    8: JointAngleSpec(8, "Middle root", "middle_aa_joint", -0.523599, 0.593412, -24.0, 23.0, is_coupled=True, motor_indices=(10, 11), mapping_source="coupled tendon joint, temporary linear joint-angle mapping"),
    9: JointAngleSpec(9, "Middle MCP", "mmhand_finger_2_proximal_phalanx_1_mcp_fe_joint", -1.570796, 0.244346, 0.0, math.degrees(1.570796), motor_indices=(12,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    10: JointAngleSpec(10, "Middle PIP", "mmhand_finger_2_distal_phalanx_1_pip_joint", -1.53589, 0.15708, 0.0, math.degrees(1.53589), motor_indices=(13,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    11: JointAngleSpec(11, "Middle DIP", "mmhand_finger_2_fingertip_1_dip_joint", -1.570796, 0.20944, 0.0, math.degrees(1.570796), motor_indices=(14,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    12: JointAngleSpec(12, "Index root", "index_aa_joint", -0.506145, 0.610865, -11.0, 30.0, is_coupled=True, motor_indices=(15, 16), mapping_source="coupled tendon joint, temporary linear joint-angle mapping"),
    13: JointAngleSpec(13, "Index MCP", "mmhand_finger_1_proximal_phalanx_1_mcp_fe_joint", -1.570796, 0.244346, 0.0, math.degrees(1.570796), motor_indices=(17,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    14: JointAngleSpec(14, "Index PIP", "mmhand_finger_1_distal_phalanx_1_pip_joint", -1.53589, 0.15708, 0.0, math.degrees(1.53589), motor_indices=(18,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    15: JointAngleSpec(15, "Index DIP", "mmhand_finger_1_fingertip_1_dip_joint", -1.570796, 0.20944, 0.0, math.degrees(1.570796), motor_indices=(19,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    16: JointAngleSpec(16, "Thumb AA", "thumb_aa_joint", -0.506145, 0.523599, -29.0, 30.0, is_coupled=True, motor_indices=(20, 21), mapping_source="coupled tendon joint, temporary linear joint-angle mapping"),
    17: JointAngleSpec(17, "Thumb MCP", "mmhand_thumb_1_mmhand_finger_5_proximal_phalanx_1_mcp_fe_joint", -1.570796, 0.244346, 0.0, math.degrees(1.570796), motor_indices=(22,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    18: JointAngleSpec(18, "Thumb PIP", "mmhand_thumb_1_mmhand_finger_5_distal_phalanx_1_pip_joint", -1.53589, 0.15708, 0.0, math.degrees(1.53589), motor_indices=(23,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    19: JointAngleSpec(19, "Thumb DIP", "mmhand_thumb_1_mmhand_finger_5_fingertip_1_dip_joint", -1.570796, 0.20944, 0.0, math.degrees(1.570796), motor_indices=(24,), mapping_source="flexion maps URDF [lower,0] -> calibrated [max,0]"),
    20: JointAngleSpec(20, "Thumb CMC", "thumb_cmc1_joint", -1.396263, 0.017453, -80.0, 1.0, is_coupled=True, motor_indices=(25, 26), mapping_source="coupled tendon joint, temporary linear joint-angle mapping"),
}

JOINT_NAME_TO_INDEX: Dict[str, int] = {}
for _idx, _spec in JOINT_SPECS.items():
    JOINT_NAME_TO_INDEX[_spec.joint_name] = _idx
    JOINT_NAME_TO_INDEX[_spec.urdf_joint_name] = _idx
    JOINT_NAME_TO_INDEX[f"J{_idx:02d}"] = _idx
    JOINT_NAME_TO_INDEX[str(_idx)] = _idx

FLEXION_JOINT_INDICES = frozenset({1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19})

_CALIB_CACHE: tuple[float, Dict[int, tuple[float, float]]] | None = None


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


def load_calibrated_ranges(snapshot_path: Path | str = CALIB_LOAD_SNAPSHOT_PATH) -> Dict[int, tuple[float, float]]:
    path = Path(snapshot_path)
    try:
        mtime = path.stat().st_mtime
    except OSError:
        return {}

    global _CALIB_CACHE
    if path == CALIB_LOAD_SNAPSHOT_PATH and _CALIB_CACHE is not None and _CALIB_CACHE[0] == mtime:
        return dict(_CALIB_CACHE[1])

    ranges: Dict[int, tuple[float, float]] = {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        results = data.get("joint_calib_results", [])
    except Exception:
        results = []

    if isinstance(results, list):
        for idx, item in enumerate(results[:ENCODER_COUNT]):
            if not isinstance(item, dict) or not item.get("success"):
                continue
            try:
                angle_max = float(item.get("angleMax", 0.0))
            except (TypeError, ValueError):
                continue
            if math.isfinite(angle_max) and angle_max > 0.0:
                ranges[idx] = (0.0, angle_max)

    if path == CALIB_LOAD_SNAPSHOT_PATH:
        _CALIB_CACHE = (mtime, dict(ranges))
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


def urdf_to_viewer(joint: int | str, q_urdf_rad: float) -> float:
    """URDF rad -> viewer degrees.

    The viewer is intentionally an identity view of URDF angle. It should not
    apply hardware/calibration sign or range conversion.
    """
    resolve_joint(joint)
    return math.degrees(float(q_urdf_rad))


def viewer_to_urdf(joint: int | str, q_viewer_deg: float) -> float:
    """Viewer degrees -> URDF rad. Viewer degrees are URDF degrees."""
    resolve_joint(joint)
    return math.radians(float(q_viewer_deg))


def urdf_to_calibrated(
    joint: int | str,
    q_urdf_rad: float,
    *,
    allow_default: bool = True,
    clamp: bool = True,
) -> float:
    """URDF rad -> calibrated/hardware command degrees.

    This is a temporary linear mapping framework. The measured calibration
    range is read from calib_load_snapshot.json as [0, angleMax]. If no
    calibrated range is available and allow_default is False, this raises.
    """
    idx = resolve_joint(joint)
    spec = JOINT_SPECS[idx]
    q_urdf_clamped = clamp_urdf_for_calibrated_mapping(idx, q_urdf_rad) if clamp else float(q_urdf_rad)
    q_urdf_deg = math.degrees(q_urdf_clamped)

    if idx in FLEXION_JOINT_INDICES:
        urdf_lo, urdf_hi = _calibrated_mapping_urdf_range_deg(idx)
        default_deg = _linear_map(
            q_urdf_deg,
            urdf_lo,
            urdf_hi,
            spec.default_calib_max_deg,
            spec.default_calib_min_deg,
        )
    else:
        default_deg = _linear_map(
            q_urdf_deg,
            spec.urdf_min_deg,
            spec.urdf_max_deg,
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

    if idx in FLEXION_JOINT_INDICES:
        urdf_lo, urdf_hi = _calibrated_mapping_urdf_range_deg(idx)
        q_urdf_deg = _linear_map(
            default_deg,
            spec.default_calib_min_deg,
            spec.default_calib_max_deg,
            urdf_hi,
            urdf_lo,
        )
    else:
        q_urdf_deg = _linear_map(
            default_deg,
            spec.default_calib_min_deg,
            spec.default_calib_max_deg,
            spec.urdf_min_deg,
            spec.urdf_max_deg,
        )
    q_urdf = math.radians(q_urdf_deg)
    return clamp_urdf(idx, q_urdf) if clamp else q_urdf


def clamp_urdf(joint: int | str, q_urdf_rad: float) -> float:
    spec = get_spec(joint)
    lo, hi = sorted(spec.urdf_range_rad)
    return max(lo, min(hi, float(q_urdf_rad)))


def clamp_urdf_for_calibrated_mapping(joint: int | str, q_urdf_rad: float) -> float:
    """Clamp URDF angle before converting to calibrated/hardware degrees.

    MCP/PIP/DIP joints have positive extension room in the URDF, but hardware
    flexion commands are defined from open hand at URDF 0 toward negative URDF
    flexion. Therefore calibrated mapping uses [URDF lower, 0] and clamps any
    positive URDF angle to 0 before mapping.
    """
    idx = resolve_joint(joint)
    if idx not in FLEXION_JOINT_INDICES:
        return clamp_urdf(idx, q_urdf_rad)
    lo_deg, hi_deg = _calibrated_mapping_urdf_range_deg(idx)
    lo = math.radians(min(lo_deg, hi_deg))
    hi = math.radians(max(lo_deg, hi_deg))
    return max(lo, min(hi, float(q_urdf_rad)))


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


def batch_urdf_to_calibrated(
    q_urdf_rad: Iterable[float],
    *,
    allow_default: bool = True,
    clamp: bool = True,
) -> list[float]:
    out = [0.0] * ENCODER_COUNT
    for idx, value in enumerate(list(q_urdf_rad)[:ENCODER_COUNT]):
        out[idx] = urdf_to_calibrated(idx, float(value), allow_default=allow_default, clamp=clamp)
    return out


def batch_calibrated_to_urdf(
    q_calibrated_deg: Iterable[float],
    *,
    allow_default: bool = True,
    clamp: bool = True,
) -> list[float]:
    out = [0.0] * ENCODER_COUNT
    for idx, value in enumerate(list(q_calibrated_deg)[:ENCODER_COUNT]):
        out[idx] = calibrated_to_urdf(idx, float(value), allow_default=allow_default, clamp=clamp)
    return out


def mapping_debug(
    joint: int | str,
    q_urdf_rad: float,
    *,
    allow_default: bool = True,
) -> AngleMappingDebug:
    idx = resolve_joint(joint)
    spec = JOINT_SPECS[idx]
    q_urdf = float(q_urdf_rad)
    q_urdf_clamped = clamp_urdf_for_calibrated_mapping(idx, q_urdf)
    q_viewer = urdf_to_viewer(idx, q_urdf_clamped)
    try:
        q_calib_raw = urdf_to_calibrated(idx, q_urdf, allow_default=allow_default, clamp=True)
        q_calib = clamp_calibrated(idx, q_calib_raw, allow_default=allow_default)
        is_clamped_calib = abs(q_calib - q_calib_raw) > 1e-6
    except AngleMappingError:
        q_calib = None
        is_clamped_calib = False

    return AngleMappingDebug(
        joint_name=spec.joint_name,
        joint_index=idx,
        q_ik_urdf_rad=q_urdf,
        q_ik_urdf_deg=math.degrees(q_urdf),
        q_viewer_deg=q_viewer,
        q_calibrated_deg=q_calib,
        urdf_range_deg=spec.urdf_range_deg,
        viewer_range_deg=spec.urdf_range_deg,
        calibrated_range_deg=effective_calibrated_range_deg(idx, allow_default=allow_default),
        sign=spec.sign,
        offset_deg=spec.offset_deg,
        is_clamped_urdf=abs(q_urdf_clamped - q_urdf) > 1e-8,
        is_clamped_calibrated=is_clamped_calib,
        mapping_source=spec.mapping_source,
    )


def debug_print_joint_command(
    joint: int | str,
    q_urdf_rad: float,
    *,
    allow_default: bool = True,
    prefix: str = "[ANGLE_MAP]",
) -> None:
    if not DEBUG_ANGLE_MAPPING:
        return
    d = mapping_debug(joint, q_urdf_rad, allow_default=allow_default)
    calib = "UNKNOWN" if d.q_calibrated_deg is None else f"{d.q_calibrated_deg:+.3f}"
    calib_range = "UNKNOWN" if d.calibrated_range_deg is None else _range_text(d.calibrated_range_deg)
    print(
        f"{prefix} J{d.joint_index:02d} {d.joint_name} "
        f"q_ik_urdf={d.q_ik_urdf_deg:+.3f}deg "
        f"q_viewer={d.q_viewer_deg:+.3f}deg "
        f"q_calibrated={calib}deg "
        f"urdf_range={_range_text(d.urdf_range_deg)} "
        f"viewer_range={_range_text(d.viewer_range_deg)} "
        f"calibrated_range={calib_range} "
        f"sign={d.sign:+d} offset={d.offset_deg:+.3f} "
        f"clamped_urdf={int(d.is_clamped_urdf)} "
        f"clamped_calibrated={int(d.is_clamped_calibrated)} "
        f"source={d.mapping_source}"
    )


def joint_mapping_rows() -> list[dict[str, object]]:
    calib_ranges = load_calibrated_ranges()
    rows: list[dict[str, object]] = []
    for idx in range(ENCODER_COUNT):
        spec = JOINT_SPECS[idx]
        calib = calib_ranges.get(idx)
        rows.append(
            {
                "joint_name": spec.urdf_joint_name,
                "urdf_range": spec.urdf_range_deg,
                "viewer_range": spec.urdf_range_deg,
                "calibrated_range": calib,
                "motor/index": f"J{idx:02d}/M{list(spec.motor_indices) if spec.motor_indices else 'UNKNOWN'}",
                "unit": "URDF rad internal, viewer deg, calibrated deg",
                "sign": spec.sign,
                "offset": spec.offset_deg,
                "current_mapping_source": spec.mapping_source,
            }
        )
    return rows


def _linear_map(value: float, src_lo: float, src_hi: float, dst_lo: float, dst_hi: float) -> float:
    denom = src_hi - src_lo
    if abs(denom) < 1e-8:
        return dst_lo
    ratio = (float(value) - src_lo) / denom
    return dst_lo + ratio * (dst_hi - dst_lo)


def _calibrated_mapping_urdf_range_deg(joint: int | str) -> tuple[float, float]:
    spec = get_spec(joint)
    if spec.joint_index in FLEXION_JOINT_INDICES:
        return (spec.urdf_min_deg, 0.0)
    return spec.urdf_range_deg


def _range_text(rng: tuple[float, float]) -> str:
    return f"[{rng[0]:+.3f},{rng[1]:+.3f}]"


__all__ = [
    "AngleMappingDebug",
    "AngleMappingError",
    "DEBUG_ANGLE_MAPPING",
    "JOINT_SPECS",
    "JointAngleSpec",
    "batch_calibrated_to_urdf",
    "batch_urdf_to_calibrated",
    "calibrated_range_deg",
    "calibrated_to_urdf",
    "clamp_calibrated",
    "clamp_urdf",
    "debug_print_joint_command",
    "effective_calibrated_range_deg",
    "get_spec",
    "joint_mapping_rows",
    "load_calibrated_ranges",
    "mapping_debug",
    "resolve_joint",
    "urdf_to_calibrated",
    "urdf_to_viewer",
    "viewer_to_urdf",
]
