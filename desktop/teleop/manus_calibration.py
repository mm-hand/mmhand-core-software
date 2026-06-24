import json
import re
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from protocol import ENCODER_COUNT

from .manus_receiver import ManusFrame, ManusReceiver


MANUS_CALIBRATION_PATH = Path(__file__).resolve().parents[1] / "config" / "manus_calibration.json"
MANUS_JOINT_SEND_GATED = True
MANUS_PREVIEW_ONLY = not MANUS_JOINT_SEND_GATED
ACTUAL_ROBOT_COMMAND_ENABLED = False
DEFAULT_MANUS_SIDE = "left"


FINGER_TO_ROBOT_JOINTS = {
    "Pinky": [0, 1, 2, 3],
    "Ring": [4, 5, 6, 7],
    "Middle": [8, 9, 10, 11],
    "Index": [12, 13, 14, 15],
    "Thumb": [20, 16, 17, 18, 19],
}

FOUR_FINGER_NAMES = ("Pinky", "Ring", "Middle", "Index")
FOUR_FINGER_ALIGN_Q_TARGET_DEG = {
    idx: 0.0
    for finger in FOUR_FINGER_NAMES
    for idx in FINGER_TO_ROBOT_JOINTS[finger]
}
FOUR_FINGER_OPEN_AA_Q_TARGET_DEG = {
    0: float(np.degrees(-0.79)),
    4: float(np.degrees(-0.39)),
    8: float(np.degrees(-0.04)),
    12: float(np.degrees(0.52)),
}
MIDDLE_AA_FROM_NEIGHBOR_WEIGHT = 1.0
FOUR_FINGER_FIST_FLEX_Q_TARGET_DEG = {
    1: float(np.degrees(1.57)),
    2: float(np.degrees(1.06)),
    3: float(np.degrees(0.42)),
    5: float(np.degrees(1.57)),
    6: float(np.degrees(1.06)),
    7: float(np.degrees(0.42)),
    9: float(np.degrees(1.57)),
    10: float(np.degrees(1.06)),
    11: float(np.degrees(0.42)),
    13: float(np.degrees(1.57)),
    14: float(np.degrees(1.06)),
    15: float(np.degrees(0.42)),
}
THUMB_PINKY_ROOT_FLEX_Q_TARGET_DEG = {
    17: 73.7,
    18: 47.0,
    19: 24.2,
}
DUAL_AUTO_JOINT_INDICES = {0, 4, 8, 12, 16, 20}
ROBOT_JOINT_LIMIT_SOURCE = "URDF limit"
ROBOT_JOINT_LIMITS_DEG = [
    (-45.0, 11.0),
    (0.0, float(np.degrees(1.570796))),
    (0.0, float(np.degrees(1.53589))),
    (0.0, float(np.degrees(1.570796))),
    (-51.0, 21.0),
    (0.0, float(np.degrees(1.570796))),
    (0.0, float(np.degrees(1.53589))),
    (0.0, float(np.degrees(1.570796))),
    (-24.0, 23.0),
    (0.0, float(np.degrees(1.570796))),
    (0.0, float(np.degrees(1.53589))),
    (0.0, float(np.degrees(1.570796))),
    (-11.0, 30.0),
    (0.0, float(np.degrees(1.570796))),
    (0.0, float(np.degrees(1.53589))),
    (0.0, float(np.degrees(1.570796))),
    (-29.0, 30.0),
    (0.0, float(np.degrees(1.570796))),
    (0.0, float(np.degrees(1.53589))),
    (0.0, float(np.degrees(1.570796))),
    (-80.0, 1.0),
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
    16: "Thumb AA",
    17: "Thumb MCP",
    18: "Thumb PIP",
    19: "Thumb DIP",
    20: "Thumb CMC",
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
class RobotPreviewJoint:
    joint_index: int
    joint_name: str
    human_joint_mapping: str
    gain_deg_per_human_deg: float
    robot_q_target_preview: float
    human_delta_deg: float
    joint_limit_deg: str
    joint_limit_source: str
    actual_robot_command: str
    lower_deg: float
    upper_deg: float
    over_limit: bool


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
        self.calibration: Dict[str, Any] = {
            "align": {},
            "open_aa": {},
            "fist_flex": {},
            "open": {},
            "close": {},
            "pinch": {},
            "opposition": {},
            "thumb_pinky_root": {},
            "raw_open_finger_lengths": {},
            "raw_open_palm_width": {},
            "raw_open_anchor_tip_vectors": {},
            "raw_open_thumb_base_mcp_vector": {},
        }
        self.loaded_at = 0.0
        self.load()

    def load(self) -> bool:
        if not self.path.exists():
            return False
        data = json.loads(self.path.read_text(encoding="utf-8"))
        poses = data.get("poses", {})
        for key in (
            "align",
            "open_aa",
            "fist_flex",
            "open",
            "close",
            "pinch",
            "opposition",
            "thumb_pinky_root",
            "raw_open_finger_lengths",
            "raw_open_palm_width",
        ):
            values = poses.get(key, {})
            if isinstance(values, dict):
                self.calibration[key] = {str(k): float(v) for k, v in values.items()}
        vectors = poses.get("raw_open_anchor_tip_vectors", {})
        if isinstance(vectors, dict):
            parsed_vectors: Dict[str, List[float]] = {}
            for finger_name, values in vectors.items():
                if isinstance(values, list) and len(values) >= 3:
                    try:
                        parsed_vectors[str(finger_name)] = [float(values[0]), float(values[1]), float(values[2])]
                    except (TypeError, ValueError):
                        continue
            self.calibration["raw_open_anchor_tip_vectors"] = parsed_vectors
        thumb_base_mcp = poses.get("raw_open_thumb_base_mcp_vector", {})
        if isinstance(thumb_base_mcp, dict):
            values = thumb_base_mcp.get("value", [])
            if isinstance(values, list) and len(values) >= 3:
                try:
                    self.calibration["raw_open_thumb_base_mcp_vector"] = {
                        "value": [float(values[0]), float(values[1]), float(values[2])]
                    }
                except (TypeError, ValueError):
                    pass
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
        raw_length_samples: Dict[str, List[float]] = {}
        raw_palm_width_samples: List[float] = []
        raw_anchor_vector_samples: Dict[str, List[np.ndarray]] = {}
        raw_thumb_base_mcp_samples: List[np.ndarray] = []
        seen_seq = None
        deadline = time.time() + float(duration_s)
        while time.time() < deadline:
            frame = receiver.get_latest_frame()
            if frame is not None and frame.seq != seen_seq:
                seen_seq = frame.seq
                for item in typed_ergonomics_from_frame(frame):
                    samples.setdefault(item.type_name, []).append(item.value_deg)
                if pose_key in ("align", "open"):
                    # raw-skeleton retarget 的同一次 open 标定同时记录：
                    # 1. 手掌宽度尺度；2. MANUS palm-local 的 anchor->tip 方向。
                    for finger_name, length in self._raw_finger_polyline_lengths(frame).items():
                        raw_length_samples.setdefault(finger_name, []).append(length)
                    raw_open = self._raw_open_palm_local_anchor_vectors(frame)
                    if raw_open is not None:
                        palm_width, vectors, thumb_base_mcp = raw_open
                        raw_palm_width_samples.append(float(palm_width))
                        for finger_name, vector in vectors.items():
                            raw_anchor_vector_samples.setdefault(finger_name, []).append(vector)
                        if thumb_base_mcp is not None:
                            raw_thumb_base_mcp_samples.append(thumb_base_mcp)
            time.sleep(0.01)

        medians = {
            name: float(statistics.median(values))
            for name, values in samples.items()
            if values
        }
        self.calibration[pose_key] = medians
        if pose_key in ("align", "open"):
            self.calibration["raw_open_finger_lengths"] = {
                finger_name: float(statistics.median(values))
                for finger_name, values in raw_length_samples.items()
                if values
            }
            if raw_palm_width_samples:
                self.calibration["raw_open_palm_width"] = {
                    "value": float(statistics.median(raw_palm_width_samples))
                }
            self.calibration["raw_open_anchor_tip_vectors"] = {
                finger_name: [
                    float(statistics.median([float(v[axis]) for v in values]))
                    for axis in range(3)
                ]
                for finger_name, values in raw_anchor_vector_samples.items()
                if values
            }
            if raw_thumb_base_mcp_samples:
                self.calibration["raw_open_thumb_base_mcp_vector"] = {
                    "value": [
                        float(statistics.median([float(v[axis]) for v in raw_thumb_base_mcp_samples]))
                        for axis in range(3)
                    ]
                }
        self.save()
        return len(medians) + (
            (
                len(self.calibration.get("raw_open_finger_lengths", {}))
                + len(self.calibration.get("raw_open_anchor_tip_vectors", {}))
                + (1 if self.calibration.get("raw_open_palm_width") else 0)
                + (1 if self.calibration.get("raw_open_thumb_base_mcp_vector") else 0)
            )
            if pose_key in ("align", "open")
            else 0
        )

    def has_open_close(self) -> bool:
        return bool(self.calibration.get("open")) and bool(self.calibration.get("close"))

    def has_four_finger_reference(self) -> bool:
        return (
            bool(self.calibration.get("align"))
            and bool(self.calibration.get("open_aa"))
            and bool(self.calibration.get("fist_flex"))
        )

    def has_thumb_flex_reference(self) -> bool:
        return (
            bool(self.calibration.get("align"))
            and bool(self.calibration.get("thumb_pinky_root"))
        )

    def has_ergonomics_reference(self) -> bool:
        return self.has_four_finger_reference() or self.has_thumb_flex_reference()

    def raw_open_finger_lengths(self) -> Dict[str, float]:
        # 给 raw-skeleton retarget 使用，用来计算 s_f = L_robot_f / L_MANUS_open_f。
        out: Dict[str, float] = {}
        for name, value in self.calibration.get("raw_open_finger_lengths", {}).items():
            try:
                length = float(value)
            except (TypeError, ValueError):
                continue
            if length > 1e-8:
                out[str(name)] = length
        return out

    def raw_open_palm_width(self) -> Optional[float]:
        values = self.calibration.get("raw_open_palm_width", {})
        if not isinstance(values, dict):
            return None
        try:
            value = float(values.get("value", 0.0))
        except (TypeError, ValueError):
            return None
        return value if value > 1e-8 else None

    def raw_open_anchor_tip_vectors(self) -> Dict[str, np.ndarray]:
        out: Dict[str, np.ndarray] = {}
        values = self.calibration.get("raw_open_anchor_tip_vectors", {})
        if not isinstance(values, dict):
            return out
        for finger_name, vector in values.items():
            if not isinstance(vector, list) or len(vector) < 3:
                continue
            try:
                arr = np.array([float(vector[0]), float(vector[1]), float(vector[2])], dtype=float)
            except (TypeError, ValueError):
                continue
            if float(np.linalg.norm(arr)) > 1e-8:
                out[str(finger_name)] = arr
        return out

    def raw_open_thumb_base_mcp_vector(self) -> Optional[np.ndarray]:
        values = self.calibration.get("raw_open_thumb_base_mcp_vector", {})
        if not isinstance(values, dict):
            return None
        vector = values.get("value", [])
        if not isinstance(vector, list) or len(vector) < 3:
            return None
        try:
            arr = np.array([float(vector[0]), float(vector[1]), float(vector[2])], dtype=float)
        except (TypeError, ValueError):
            return None
        return arr if float(np.linalg.norm(arr)) > 1e-8 else None

    @staticmethod
    def _raw_finger_polyline_lengths(frame: ManusFrame) -> Dict[str, float]:
        # 直接在 MANUS raw skeleton 坐标里测量手指折线长度。
        # 距离不受 palm frame 旋转影响，所以这里不依赖 retarget 代码。
        skeleton = getattr(frame, "skeleton", {}) or {}
        if not isinstance(skeleton, dict):
            return {}

        node_keys = {
            "thumb": ("metacarpal", "proximal", "distal", "tip"),
            "index": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
            "middle": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
            "ring": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
            "pinky": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
        }

        def node_pos(node) -> Optional[np.ndarray]:
            if not isinstance(node, dict):
                return None
            values = node.get("pos")
            if not isinstance(values, list) or len(values) < 3:
                return None
            try:
                return np.array([float(values[0]), float(values[1]), float(values[2])], dtype=float)
            except (TypeError, ValueError):
                return None

        out: Dict[str, float] = {}
        for finger_name, keys in node_keys.items():
            finger = skeleton.get(finger_name)
            if not isinstance(finger, dict):
                continue
            points = [node_pos(finger.get(key)) for key in keys]
            if any(point is None for point in points):
                continue
            length = 0.0
            for a, b in zip(points, points[1:]):
                length += float(np.linalg.norm(b - a))
            if length > 1e-8:
                out[finger_name] = length
        return out

    @staticmethod
    def _raw_open_palm_local_anchor_vectors(
        frame: ManusFrame,
    ) -> Optional[Tuple[float, Dict[str, np.ndarray], Optional[np.ndarray]]]:
        skeleton = getattr(frame, "skeleton", {}) or {}
        if not isinstance(skeleton, dict):
            return None

        def node_pos(node) -> Optional[np.ndarray]:
            if not isinstance(node, dict):
                return None
            values = node.get("pos")
            if not isinstance(values, list) or len(values) < 3:
                return None
            try:
                return np.array([float(values[0]), float(values[1]), float(values[2])], dtype=float)
            except (TypeError, ValueError):
                return None

        def node_rot(node) -> Optional[np.ndarray]:
            if not isinstance(node, dict):
                return None
            values = node.get("rot")
            if not isinstance(values, list) or len(values) < 4:
                return None
            try:
                quat = np.array([float(values[0]), float(values[1]), float(values[2]), float(values[3])], dtype=float)
            except (TypeError, ValueError):
                return None
            norm = float(np.linalg.norm(quat))
            if norm < 1e-8:
                return None
            return quat / norm

        def quat_to_rotation(quat_wxyz: np.ndarray) -> np.ndarray:
            w, x, y, z = [float(v) for v in quat_wxyz]
            return np.array(
                [
                    [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                    [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                    [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
                ],
                dtype=float,
            )

        hand = skeleton.get("hand")
        origin = node_pos(hand)
        if origin is None:
            return None

        metas: Dict[str, np.ndarray] = {}
        for finger_name in ("index", "middle", "ring", "pinky"):
            finger = skeleton.get(finger_name)
            pos = node_pos(finger.get("metacarpal")) if isinstance(finger, dict) else None
            if pos is not None:
                metas[finger_name] = pos
        index_pos = metas.get("index")
        pinky_pos = metas.get("pinky")
        if index_pos is None or pinky_pos is None:
            return None

        center = np.mean(np.stack(list(metas.values()), axis=0), axis=0)

        def normalize(vec: np.ndarray) -> np.ndarray:
            norm = float(np.linalg.norm(vec))
            if norm < 1e-8:
                return np.zeros(3, dtype=float)
            return vec / norm

        x_axis = normalize(center - origin)
        y_axis = normalize(pinky_pos - index_pos)
        z_axis = normalize(np.cross(x_axis, y_axis))
        if float(np.linalg.norm(x_axis)) < 1e-8 or float(np.linalg.norm(y_axis)) < 1e-8 or float(np.linalg.norm(z_axis)) < 1e-8:
            return None
        hand_rot = node_rot(hand)
        if hand_rot is not None:
            hand_normal = quat_to_rotation(hand_rot)[:, 2]
            if float(np.dot(z_axis, hand_normal)) < 0.0:
                z_axis = -z_axis
        y_axis = normalize(np.cross(z_axis, x_axis))
        rotation = np.stack([x_axis, y_axis, z_axis], axis=1)
        palm_width = float(np.linalg.norm((pinky_pos - origin) @ rotation - (index_pos - origin) @ rotation))

        anchor_keys = {
            "thumb": "metacarpal",
            "index": "metacarpal",
            "middle": "metacarpal",
            "ring": "metacarpal",
            "pinky": "metacarpal",
        }
        vectors: Dict[str, np.ndarray] = {}
        for finger_name, anchor_key in anchor_keys.items():
            finger = skeleton.get(finger_name)
            if not isinstance(finger, dict):
                continue
            anchor = node_pos(finger.get(anchor_key))
            tip = node_pos(finger.get("tip"))
            if anchor is None or tip is None:
                continue
            vector = (tip - origin) @ rotation - (anchor - origin) @ rotation
            if float(np.linalg.norm(vector)) > 1e-8:
                vectors[finger_name] = vector
        if not vectors:
            return None
        thumb_base_mcp = None
        thumb = skeleton.get("thumb")
        if isinstance(thumb, dict):
            thumb_base = node_pos(thumb.get("metacarpal"))
            thumb_mcp = node_pos(thumb.get("proximal"))
            if thumb_base is not None and thumb_mcp is not None:
                # thumb CMC/AA 用的是 MANUS metacarpal->proximal 这段相对 align 的变化量。
                thumb_base_mcp = (thumb_mcp - origin) @ rotation - (thumb_base - origin) @ rotation
                if float(np.linalg.norm(thumb_base_mcp)) <= 1e-8:
                    thumb_base_mcp = None
        return palm_width, vectors, thumb_base_mcp

    def robot_q_target_preview(self, frame: ManusFrame, state=None, side: str = DEFAULT_MANUS_SIDE) -> List[RobotPreviewJoint]:
        typed = _typed_map(frame)
        preview = [0.0] * ENCODER_COUNT
        mapping = [""] * ENCODER_COUNT
        gains = [0.0] * ENCODER_COUNT
        human_angles = [0.0] * ENCODER_COUNT

        if self.has_four_finger_reference():
            for finger in FOUR_FINGER_NAMES:
                joints = FINGER_TO_ROBOT_JOINTS[finger]
                for motion_joint, robot_idx in (("MCP", joints[1]), ("PIP", joints[2]), ("DIP", joints[3])):
                    feature_name = self._feature_name(frame, finger, motion_joint, "Stretch", side=side)
                    self._apply_pose_pair_feature(
                        typed,
                        feature_name,
                        robot_idx,
                        "fist_flex",
                        FOUR_FINGER_FIST_FLEX_Q_TARGET_DEG,
                        preview,
                        mapping,
                        gains,
                        human_angles,
                        force_positive_gain=True,
                    )
                spread_name = self._feature_name(frame, finger, "MCP", "Spread", side=side)
                self._apply_pose_pair_feature(
                    typed,
                    spread_name,
                    joints[0],
                    "open_aa",
                    FOUR_FINGER_OPEN_AA_Q_TARGET_DEG,
                    preview,
                    mapping,
                    gains,
                    human_angles,
                )
            middle_aa = FINGER_TO_ROBOT_JOINTS["Middle"][0]
            ring_aa = FINGER_TO_ROBOT_JOINTS["Ring"][0]
            index_aa = FINGER_TO_ROBOT_JOINTS["Index"][0]
            preview[middle_aa] = MIDDLE_AA_FROM_NEIGHBOR_WEIGHT * 0.5 * (preview[ring_aa] + preview[index_aa])
            gains[middle_aa] = 0.0
            human_angles[middle_aa] = 0.0
            mapping[middle_aa] = (
                f"neighbor AA interpolation: 0.5 * ({self._robot_joint_label(ring_aa)} + "
                f"{self._robot_joint_label(index_aa)})"
            )
        else:
            for finger in FOUR_FINGER_NAMES:
                for robot_idx in FINGER_TO_ROBOT_JOINTS[finger]:
                    mapping[robot_idx] = "Record Align + Open AA + Fist Flex first"

        thumb_joints = FINGER_TO_ROBOT_JOINTS["Thumb"]
        for robot_idx in (thumb_joints[0], thumb_joints[1]):
            mapping[robot_idx] = "thumb CMC/AA handled by raw skeleton IK"
        if self.has_thumb_flex_reference():
            for motion_joint, robot_idx in (("MCP", thumb_joints[2]), ("PIP", thumb_joints[3]), ("DIP", thumb_joints[4])):
                feature_name = self._feature_name(frame, "Thumb", motion_joint, "Stretch", side=side)
                self._apply_pose_pair_feature(
                    typed,
                    feature_name,
                    robot_idx,
                    "thumb_pinky_root",
                    THUMB_PINKY_ROOT_FLEX_Q_TARGET_DEG,
                    preview,
                    mapping,
                    gains,
                    human_angles,
                    force_positive_gain=True,
                )
        else:
            for robot_idx in (thumb_joints[2], thumb_joints[3], thumb_joints[4]):
                mapping[robot_idx] = "Record Align + Thumb->PinkyRoot first"

        out: List[RobotPreviewJoint] = []
        for idx, value in enumerate(preview):
            lo, hi = self._robot_joint_limit_deg(idx)
            source_text = (
                f"{ROBOT_JOINT_LIMIT_SOURCE}: signed"
                if idx in DUAL_AUTO_JOINT_INDICES
                else f"{ROBOT_JOINT_LIMIT_SOURCE}: flexion"
            )
            unclipped = value
            clamped = max(lo, min(hi, unclipped))
            out.append(
                RobotPreviewJoint(
                    joint_index=idx,
                    joint_name=self._robot_joint_label(idx),
                    human_joint_mapping=mapping[idx] or "-",
                    gain_deg_per_human_deg=gains[idx],
                    robot_q_target_preview=float(clamped),
                    human_delta_deg=human_angles[idx],
                    joint_limit_deg=f"[{lo:.2f}, {hi:.2f}] deg",
                    joint_limit_source=source_text,
                    actual_robot_command="gated until Enable Joint Send",
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

    @staticmethod
    def _robot_joint_limit_deg(joint_idx: int) -> tuple[float, float]:
        return ROBOT_JOINT_LIMITS_DEG[joint_idx]

    def _apply_pose_pair_feature(
        self,
        typed: Dict[str, TypedErgonomicValue],
        feature_name: str,
        robot_idx: int,
        pose_key: str,
        pose_q_target_deg: Dict[int, float],
        preview: List[float],
        mapping: List[str],
        gains: List[float],
        human_angles: List[float],
        force_positive_gain: bool = False,
    ) -> None:
        if not 0 <= robot_idx < ENCODER_COUNT:
            return

        current = typed.get(feature_name)
        align_values = self.calibration.get("align", {})
        pose_values = self.calibration.get(pose_key, {})
        q_align = FOUR_FINGER_ALIGN_Q_TARGET_DEG.get(robot_idx, 0.0)
        q_pose = pose_q_target_deg.get(robot_idx, q_align)

        mapping_name = f"{feature_name} align->{pose_key} -> {self._robot_joint_label(robot_idx)}"
        if current is None or feature_name not in align_values or feature_name not in pose_values:
            preview[robot_idx] = q_align
            gains[robot_idx] = 0.0
            mapping[robot_idx] = f"missing Record Align/{pose_key}: {mapping_name}"
            human_angles[robot_idx] = 0.0
            return

        h_current = float(current.value_deg)
        h_align = float(align_values[feature_name])
        h_pose = float(pose_values[feature_name])
        denom = h_pose - h_align
        if abs(denom) < 1e-6:
            preview[robot_idx] = q_align
            gains[robot_idx] = 0.0
            mapping[robot_idx] = f"invalid small MANUS delta: {mapping_name}"
            human_angles[robot_idx] = h_current - h_align
            return
        if force_positive_gain:
            direction = 1.0 if denom >= 0.0 else -1.0
            human_delta = direction * (h_current - h_align)
            gain = (q_pose - q_align) / abs(denom)
        else:
            human_delta = h_current - h_align
            gain = (q_pose - q_align) / denom
        preview[robot_idx] = q_align + gain * human_delta
        gains[robot_idx] = gain
        mapping[robot_idx] = mapping_name
        human_angles[robot_idx] = human_delta


__all__ = [
    "ACTUAL_ROBOT_COMMAND_ENABLED",
    "DEFAULT_MANUS_SIDE",
    "MANUS_JOINT_SEND_GATED",
    "MANUS_PREVIEW_ONLY",
    "ManusCalibrationManager",
    "RobotPreviewJoint",
    "TypedErgonomicValue",
    "typed_ergonomics_from_frame",
]
