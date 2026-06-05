import json
import re
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from protocol import ENCODER_COUNT

from .manus_receiver import ManusFrame, ManusReceiver


MANUS_CALIBRATION_PATH = Path(__file__).resolve().parents[1] / "config" / "manus_calibration.json"
MANUS_PREVIEW_ONLY = True
ACTUAL_ROBOT_COMMAND_ENABLED = False
DEFAULT_MANUS_SIDE = "left"


FINGER_TO_ROBOT_JOINTS = {
    "Pinky": [0, 1, 2, 3],
    "Ring": [4, 5, 6, 7],
    "Middle": [8, 9, 10, 11],
    "Index": [12, 13, 14, 15],
    "Thumb": [16, 17, 18, 19, 20],
}

MAGNETIC_RANGE100_CONTROL_MAX_DEG = 83.98
DUAL_AUTO_CONTROL_MAX_DEG = 80.0
DUAL_AUTO_JOINT_INDICES = {0, 4, 8, 12, 16, 20}
ROBOT_JOINT_LIMIT_SOURCE = "magnetic calib clamp"
ROBOT_JOINT_LIMITS_DEG = [
    (0.0, DUAL_AUTO_CONTROL_MAX_DEG if idx in DUAL_AUTO_JOINT_INDICES else MAGNETIC_RANGE100_CONTROL_MAX_DEG)
    for idx in range(ENCODER_COUNT)
]
ROBOT_JOINT_ROLE_BY_INDEX = {
    0: "Pinky root",
    1: "Pinky MCP",
    2: "Pinky PIP",
    3: "Pinky DIP",
    4: "Ring root",
    5: "Ring MCP",
    6: "Ring PIP",
    7: "Ring DIP",
    8: "Middle root",
    9: "Middle MCP",
    10: "Middle PIP",
    11: "Middle DIP",
    12: "Index root",
    13: "Index MCP",
    14: "Index PIP",
    15: "Index DIP",
    16: "Thumb root",
    17: "Thumb MCP",
    18: "Thumb PIP",
    19: "Thumb DIP",
    20: "Thumb coupled distal",
}


@dataclass(frozen=True)
class TypedErgonomicValue:
    type_name: str
    side: str
    finger: str
    joint: str
    motion: str
    value_deg: float


@dataclass
class NormalizedFeature:
    key: str
    type_name: str
    human_joint_angle: float
    open_value: float
    close_value: float
    normalized_feature: float
    valid: bool


@dataclass
class RobotPreviewJoint:
    joint_index: int
    joint_name: str
    human_joint_mapping: str
    normalized_feature: float
    robot_q_target_preview: float
    human_joint_angle: float
    joint_limit_deg: str
    joint_limit_source: str
    actual_robot_command: str
    lower_deg: float
    upper_deg: float
    over_limit: bool


def _clip01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _parse_type_name(type_name: str) -> Optional[Tuple[str, str, str, str]]:
    match = re.match(r"^(Left|Right)Finger(Thumb|Index|Middle|Ring|Pinky)(MCP|PIP|DIP)(Spread|Stretch)$", type_name)
    if not match:
        return None
    return match.group(1), match.group(2), match.group(3), match.group(4)


def typed_ergonomics_from_frame(frame: ManusFrame) -> List[TypedErgonomicValue]:
    out: List[TypedErgonomicValue] = []
    for name, value in zip(frame.ergonomics_names, frame.ergonomics_values):
        parsed = _parse_type_name(str(name))
        if parsed is None:
            continue
        side, finger, joint, motion = parsed
        out.append(
            TypedErgonomicValue(
                type_name=str(name),
                side=side.lower(),
                finger=finger,
                joint=joint,
                motion=motion,
                value_deg=float(value),
            )
        )
    return out


def _typed_map(frame: ManusFrame) -> Dict[str, TypedErgonomicValue]:
    return {item.type_name: item for item in typed_ergonomics_from_frame(frame)}


class ManusCalibrationManager:
    def __init__(self, path: Path = MANUS_CALIBRATION_PATH):
        self.path = Path(path)
        self.calibration: Dict[str, Dict[str, float]] = {
            "open": {},
            "close": {},
            "pinch": {},
            "opposition": {},
        }
        self.loaded_at = 0.0
        self.load()

    def load(self) -> bool:
        if not self.path.exists():
            return False
        data = json.loads(self.path.read_text(encoding="utf-8"))
        poses = data.get("poses", {})
        for key in ("open", "close", "pinch", "opposition"):
            values = poses.get(key, {})
            if isinstance(values, dict):
                self.calibration[key] = {str(k): float(v) for k, v in values.items()}
        self.loaded_at = time.time()
        return True

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "saved_at": time.time(),
            "mode": "MANUS_JOINT_SEND_GATED",
            "actual_robot_command_enabled": ACTUAL_ROBOT_COMMAND_ENABLED,
            "poses": self.calibration,
        }
        self.path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    def record_pose(self, pose: str, receiver: ManusReceiver, duration_s: float = 1.0) -> int:
        pose_key = pose.lower()
        if pose_key not in self.calibration:
            raise ValueError(f"unsupported MANUS calibration pose: {pose}")

        samples: Dict[str, List[float]] = {}
        seen_seq = None
        deadline = time.time() + float(duration_s)
        while time.time() < deadline:
            frame = receiver.get_latest_frame()
            if frame is not None and frame.seq != seen_seq:
                seen_seq = frame.seq
                for item in typed_ergonomics_from_frame(frame):
                    samples.setdefault(item.type_name, []).append(item.value_deg)
            time.sleep(0.01)

        medians = {
            name: float(statistics.median(values))
            for name, values in samples.items()
            if values
        }
        self.calibration[pose_key] = medians
        self.save()
        return len(medians)

    def has_open_close(self) -> bool:
        return bool(self.calibration.get("open")) and bool(self.calibration.get("close"))

    def normalized_features(self, frame: ManusFrame) -> List[NormalizedFeature]:
        typed = _typed_map(frame)
        open_values = self.calibration.get("open", {})
        close_values = self.calibration.get("close", {})
        features: List[NormalizedFeature] = []
        for type_name, item in sorted(typed.items()):
            h = item.value_deg
            h_open = float(open_values.get(type_name, h))
            h_close = float(close_values.get(type_name, h_open))
            denom = h_close - h_open
            valid = abs(denom) > 1e-6 and type_name in open_values and type_name in close_values
            s = _clip01((h - h_open) / denom) if valid else 0.0
            features.append(
                NormalizedFeature(
                    key=type_name,
                    type_name=type_name,
                    human_joint_angle=h,
                    open_value=h_open,
                    close_value=h_close,
                    normalized_feature=s,
                    valid=valid,
                )
            )
        return features

    def robot_q_target_preview(self, frame: ManusFrame, state=None, side: str = DEFAULT_MANUS_SIDE) -> List[RobotPreviewJoint]:
        feature_map = {feature.type_name: feature for feature in self.normalized_features(frame)}
        preview = [0.0] * ENCODER_COUNT
        mapping = [""] * ENCODER_COUNT
        normalized = [0.0] * ENCODER_COUNT
        human_angles = [0.0] * ENCODER_COUNT

        for finger in ("Pinky", "Ring", "Middle", "Index"):
            joints = FINGER_TO_ROBOT_JOINTS[finger]
            for motion_joint, robot_idx in (("MCP", joints[1]), ("PIP", joints[2]), ("DIP", joints[3])):
                feature_name = self._feature_name(frame, finger, motion_joint, "Stretch", side=side)
                self._apply_feature(feature_map, feature_name, robot_idx, preview, mapping, normalized, human_angles)
            spread_name = self._feature_name(frame, finger, "MCP", "Spread", side=side)
            self._apply_feature(feature_map, spread_name, joints[0], preview, mapping, normalized, human_angles)

        thumb_joints = FINGER_TO_ROBOT_JOINTS["Thumb"]
        thumb_features = [
            self._feature_name(frame, "Thumb", "MCP", "Spread", side=side),
            self._feature_name(frame, "Thumb", "MCP", "Stretch", side=side),
            self._feature_name(frame, "Thumb", "PIP", "Stretch", side=side),
            self._feature_name(frame, "Thumb", "DIP", "Stretch", side=side),
        ]
        for feature_name, robot_idx in zip(thumb_features, thumb_joints[1:]):
            self._apply_feature(feature_map, feature_name, robot_idx, preview, mapping, normalized, human_angles)
        s_spread = normalized[thumb_joints[1]]
        s_mcp = normalized[thumb_joints[2]]
        s_root = 0.5 * s_spread + 0.5 * s_mcp
        self._apply_scalar(
            s_root,
            thumb_joints[0],
            f"Thumb MCPSpread/MCPStretch coupled -> {self._robot_joint_label(thumb_joints[0])}",
            preview,
            mapping,
            normalized,
            human_angles,
            human_angle_deg=0.0,
        )

        out: List[RobotPreviewJoint] = []
        for idx, value in enumerate(preview):
            lo, hi = ROBOT_JOINT_LIMITS_DEG[idx]
            source_text = (
                f"{ROBOT_JOINT_LIMIT_SOURCE}: dual auto"
                if idx in DUAL_AUTO_JOINT_INDICES
                else f"{ROBOT_JOINT_LIMIT_SOURCE}: RANGE100"
            )
            unclipped = value
            clamped = max(lo, min(hi, unclipped))
            out.append(
                RobotPreviewJoint(
                    joint_index=idx,
                    joint_name=self._robot_joint_label(idx),
                    human_joint_mapping=mapping[idx] or "-",
                    normalized_feature=normalized[idx],
                    robot_q_target_preview=float(clamped),
                    human_joint_angle=human_angles[idx],
                    joint_limit_deg=f"[{lo:.2f}, {hi:.2f}] deg",
                    joint_limit_source=source_text,
                    actual_robot_command="preview only / not sent",
                    lower_deg=lo,
                    upper_deg=hi,
                    over_limit=(unclipped < lo or unclipped > hi),
                )
            )
        return out

    def robot_q_target_angles(self, frame: ManusFrame, side: str = DEFAULT_MANUS_SIDE) -> List[float]:
        preview = self.robot_q_target_preview(frame, side=side)
        angles = [0.0] * ENCODER_COUNT
        for item in preview:
            if 0 <= item.joint_index < ENCODER_COUNT:
                angles[item.joint_index] = float(item.robot_q_target_preview)
        return angles

    @staticmethod
    def _feature_name(frame: ManusFrame, finger: str, joint: str, motion: str, side: str = DEFAULT_MANUS_SIDE) -> str:
        side_prefix = "Right" if str(side or "").lower() == "right" else "Left"
        return f"{side_prefix}Finger{finger}{joint}{motion}"

    @staticmethod
    def _robot_joint_label(joint_idx: int) -> str:
        role = ROBOT_JOINT_ROLE_BY_INDEX.get(joint_idx, "")
        dual = " dual" if joint_idx in DUAL_AUTO_JOINT_INDICES else ""
        return f"J{joint_idx:02d} {role}{dual}".strip()

    def _apply_feature(
        self,
        feature_map: Dict[str, NormalizedFeature],
        feature_name: str,
        robot_idx: int,
        preview: List[float],
        mapping: List[str],
        normalized: List[float],
        human_angles: List[float],
    ) -> None:
        feature = feature_map.get(feature_name)
        s = feature.normalized_feature if feature and feature.valid else 0.0
        human_angle = feature.human_joint_angle if feature is not None else 0.0
        mapping_name = f"{feature_name} -> {self._robot_joint_label(robot_idx)}"
        self._apply_scalar(s, robot_idx, mapping_name, preview, mapping, normalized, human_angles, human_angle)

    @staticmethod
    def _apply_scalar(
        s: float,
        robot_idx: int,
        mapping_name: str,
        preview: List[float],
        mapping: List[str],
        normalized: List[float],
        human_angles: List[float],
        human_angle_deg: float = 0.0,
    ) -> None:
        if not 0 <= robot_idx < ENCODER_COUNT:
            return
        lo, hi = ROBOT_JOINT_LIMITS_DEG[robot_idx]
        s_clipped = _clip01(s)
        preview[robot_idx] = lo + s_clipped * (hi - lo)
        normalized[robot_idx] = s_clipped
        mapping[robot_idx] = mapping_name
        human_angles[robot_idx] = float(human_angle_deg)


__all__ = [
    "ACTUAL_ROBOT_COMMAND_ENABLED",
    "DEFAULT_MANUS_SIDE",
    "MANUS_PREVIEW_ONLY",
    "ManusCalibrationManager",
    "NormalizedFeature",
    "RobotPreviewJoint",
    "TypedErgonomicValue",
    "typed_ergonomics_from_frame",
]
