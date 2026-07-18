"""Four-pose Wuji joint-angle calibration used as vector-IK reference."""

import math
import statistics
import time

from wuji_mmhand_teleop.sources.wuji_fingertip_cp import TIP_SEMANTICS
from wuji_mmhand_teleop.urdf_contract import MODEL_ID
from wuji_mmhand_teleop.retarget.manus_calibration import (
    ManusCalibrationManager,
    RELIABLE_CALIBRATION_POSE_Q_TARGET_DEG,
    ROBOT_JOINT_ROLE_BY_INDEX,
    RobotPreviewJoint,
)


POSES = ("align", "open_aa", "fist_flex", "thumb_pinky_root")

# Wuji fingers are thumb,index,middle,ring,pinky. Slots:
# thumb = CMC rot Z, CMC flex X, CMC abd Y, MCP X, IP X
# fingers = MCP flex X, MCP abd Y, PIP X, DIP X, padding
WUJI_SOURCE_BY_JOINT = {
    0: (4, 1), 1: (4, 0), 2: (4, 2), 3: (4, 3),
    4: (3, 1), 5: (3, 0), 6: (3, 2), 7: (3, 3),
    8: (2, 1), 9: (2, 0), 10: (2, 2), 11: (2, 3),
    12: (1, 1), 13: (1, 0), 14: (1, 2), 15: (1, 3),
    16: (0, 2), 17: (0, 1), 18: (0, 3), 19: (0, 4), 20: (0, 0),
}


def _angle_key(finger: int, slot: int) -> str:
    return f"wuji_f{finger}_q{slot}_rad"


class WujiCalibrationManager(ManusCalibrationManager):
    _TIP_GEOMETRY_KEYS = (
        "align",
        "raw_open_finger_lengths",
        "raw_open_palm_width",
        "raw_open_anchor_tip_vectors",
        "raw_open_thumb_base_mcp_vector",
        "raw_open_thumb_hand_cmc_vector",
        "raw_open_thumb_hand_mcp_vector",
    )

    def __init__(self, *args, **kwargs):
        self.requires_align_recalibration = False
        self.requires_model_recalibration = False
        self._file_metadata = {
            "tip_semantics": TIP_SEMANTICS,
            "hand_model_path": "",
            "robot_model_id": MODEL_ID,
        }
        super().__init__(*args, **kwargs)

    def load(self) -> bool:
        stored_semantics = None
        stored_model_id = None
        if getattr(self, "path", None) is not None and self.path.exists():
            try:
                import json
                stored = json.loads(self.path.read_text(encoding="utf-8"))
                stored_semantics = stored.get("tip_semantics")
                stored_model_id = stored.get("robot_model_id")
            except (OSError, ValueError, TypeError):
                stored_semantics = None
                stored_model_id = None
        loaded = super().load()
        self.requires_align_recalibration = bool(
            loaded and stored_semantics != TIP_SEMANTICS
        )
        if self.requires_align_recalibration:
            for key in self._TIP_GEOMETRY_KEYS:
                self.calibration[key] = {}
        self.requires_model_recalibration = bool(loaded and stored_model_id != MODEL_ID)
        if self.requires_model_recalibration:
            for key in (*POSES, *self._TIP_GEOMETRY_KEYS):
                self.calibration[key] = {}
        return loaded

    def set_hand_model_path(self, path: str) -> None:
        self._file_metadata["hand_model_path"] = str(path)

    def record_pose(self, pose: str, receiver, duration_s: float = 1.0) -> int:
        pose = str(pose).lower()
        if pose not in POSES:
            raise ValueError(f"unsupported Wuji calibration pose: {pose}")

        angle_samples = {key: [] for key in (_angle_key(f, q) for f in range(5) for q in range(5))}
        widths, thumb_hand_cmc_samples, thumb_hand_mcp_samples = [], [], []
        thumb_finger_samples = {name: [] for name in ("index", "middle", "ring", "pinky")}
        lengths_by_finger, vectors_by_finger = {}, {}
        seen_seq = None
        deadline = time.time() + float(duration_s)
        while time.time() < deadline:
            frame = receiver.get_latest_frame()
            if frame is not None and frame.seq != seen_seq and len(frame.joint_angles_rad) == 5:
                seen_seq = frame.seq
                for finger, values in enumerate(frame.joint_angles_rad):
                    for slot, value in enumerate(values[:5]):
                        angle_samples[_angle_key(finger, slot)].append(float(value))
                if pose == "align":
                    raw_open = self._raw_open_palm_local_anchor_vectors(frame)
                    if raw_open is not None:
                        width, vectors, _thumb_vector = raw_open
                        lengths = self._raw_finger_polyline_lengths(frame)
                        if len(vectors) == 5 and len(lengths) == 5:
                            widths.append(float(width))
                            for name, value in lengths.items():
                                lengths_by_finger.setdefault(name, []).append(float(value))
                            for name, value in vectors.items():
                                vectors_by_finger.setdefault(name, []).append(value)
                            # Store Wuji open-hand thumb->finger relations in
                            # palm-local coordinates for morphology-relative
                            # non-pinch stabilization.
                            from wuji_mmhand_teleop.retarget.manus_raw_skeleton import FullRawSkeletonRetargeter
                            localized = FullRawSkeletonRetargeter._raw_skeleton_to_palm_local_nodes(frame)
                            if localized is not None:
                                nodes, _valid = localized
                                hand_node = nodes.get("hand")
                                thumb_nodes = nodes.get("thumb", {})
                                if isinstance(hand_node, dict) and isinstance(thumb_nodes, dict):
                                    cmc_node = thumb_nodes.get("cmc")
                                    mcp_node = thumb_nodes.get("mcp")
                                    if cmc_node is not None and mcp_node is not None:
                                        thumb_hand_cmc_samples.append(cmc_node["pos"] - hand_node["pos"])
                                        thumb_hand_mcp_samples.append(mcp_node["pos"] - hand_node["pos"])
                                thumb_tip = nodes.get("thumb", {}).get("tip")
                                if thumb_tip is not None:
                                    for name in thumb_finger_samples:
                                        finger_tip = nodes.get(name, {}).get("tip")
                                        if finger_tip is not None:
                                            thumb_finger_samples[name].append(
                                                finger_tip["pos"] - thumb_tip["pos"]
                                            )
            time.sleep(0.005)

        accepted = min((len(values) for values in angle_samples.values()), default=0)
        minimum = max(8, int(float(duration_s) * 10.0))
        if accepted < minimum:
            raise RuntimeError(f"Wuji {pose} has only {accepted} angle frames; need {minimum}")
        unstable = {
            key: math.degrees(max(values) - min(values))
            for key, values in angle_samples.items()
            if math.degrees(max(values) - min(values)) > 12.0
        }
        if unstable:
            key, span = max(unstable.items(), key=lambda item: item[1])
            raise RuntimeError(f"Wuji {pose} moved during capture: {key} span={span:.1f} deg")
        if pose == "align" and len(widths) < minimum:
            raise RuntimeError(f"Wuji align has only {len(widths)} valid skeleton frames")
        if pose == "align" and (
            len(thumb_hand_cmc_samples) < minimum or len(thumb_hand_mcp_samples) < minimum
        ):
            raise RuntimeError("Wuji align is missing hand0->CMC/MCP thumb vectors")

        self.calibration[pose] = {
            key: float(statistics.median(values)) for key, values in angle_samples.items()
        }
        if pose == "align":
            for name, values in thumb_finger_samples.items():
                if values:
                    for axis in range(3):
                        self.calibration[pose][f"wuji_thumb_{name}_v{axis}"] = float(
                            statistics.median([float(value[axis]) for value in values])
                        )
            self.calibration["raw_open_finger_lengths"] = {
                name: float(statistics.median(values)) for name, values in lengths_by_finger.items()
            }
            self.calibration["raw_open_palm_width"] = {"value": float(statistics.median(widths))}
            self.calibration["raw_open_anchor_tip_vectors"] = {
                name: [float(statistics.median([float(v[a]) for v in values])) for a in range(3)]
                for name, values in vectors_by_finger.items()
            }
            for key, samples in (("raw_open_thumb_hand_cmc_vector", thumb_hand_cmc_samples),
                                 ("raw_open_thumb_hand_mcp_vector", thumb_hand_mcp_samples)):
                self.calibration[key] = {
                    "value": [float(statistics.median([float(v[a]) for v in samples])) for a in range(3)]
                }
        if pose == "align":
            self.requires_align_recalibration = False
        self.requires_model_recalibration = not all(bool(self.calibration.get(name)) for name in POSES)
        self.save()
        return accepted

    def raw_open_thumb_finger_vectors(self):
        align = self.calibration.get("align", {})
        result = {}
        for name in ("index", "middle", "ring", "pinky"):
            keys = [f"wuji_thumb_{name}_v{axis}" for axis in range(3)]
            if all(key in align for key in keys):
                import numpy as np
                result[name] = np.array([float(align[key]) for key in keys], dtype=float)
        return result

    def has_ergonomics_reference(self) -> bool:
        return all(bool(self.calibration.get(pose)) for pose in POSES)

    def _pose_pair(self, joint_index: int) -> tuple[str, str]:
        if joint_index in {0, 4, 8, 12}:
            return "align", "open_aa"
        if joint_index in set(range(1, 16)) - {4, 8, 12}:
            return "align", "fist_flex"
        return "align", "thumb_pinky_root"

    def robot_q_target_preview(self, frame, state=None, side: str = "left"):
        del state, side
        current = frame.joint_angles_rad
        result = []
        for joint_index in range(21):
            finger, slot = WUJI_SOURCE_BY_JOINT[joint_index]
            lo, hi = self._robot_joint_limits_deg[joint_index]
            pose_a, pose_b = self._pose_pair(joint_index)
            human_a = self.calibration.get(pose_a, {}).get(_angle_key(finger, slot))
            human_b = self.calibration.get(pose_b, {}).get(_angle_key(finger, slot))
            robot_a = RELIABLE_CALIBRATION_POSE_Q_TARGET_DEG[pose_a].get(joint_index, 0.0)
            robot_b = RELIABLE_CALIBRATION_POSE_Q_TARGET_DEG[pose_b].get(joint_index, robot_a)
            gain = 0.0
            target = float(robot_a)
            human_delta_deg = 0.0
            if len(current) == 5 and len(current[finger]) >= 5 and human_a is not None and human_b is not None:
                denominator_deg = math.degrees(float(human_b) - float(human_a))
                human_delta_deg = math.degrees(float(current[finger][slot]) - float(human_a))
                if abs(denominator_deg) >= 2.0 and abs(float(robot_b) - float(robot_a)) > 1e-6:
                    gain = (float(robot_b) - float(robot_a)) / denominator_deg
                    target = float(robot_a) + gain * human_delta_deg
            target = max(lo, min(hi, target))
            result.append(RobotPreviewJoint(
                joint_index=joint_index,
                joint_name=f"J{joint_index:02d} {ROBOT_JOINT_ROLE_BY_INDEX.get(joint_index, '')}",
                human_joint_mapping=f"Wuji finger[{finger}].angles[{slot}] {pose_a}->{pose_b}",
                gain_deg_per_human_deg=gain,
                robot_q_target_preview=target,
                human_delta_deg=human_delta_deg,
                joint_limit_deg=f"[{lo:.2f}, {hi:.2f}] deg",
                joint_limit_source="URDF limit",
                actual_robot_command="reference only",
                lower_deg=lo,
                upper_deg=hi,
                over_limit=False,
            ))
        # The robot's middle-finger AA is mechanically defined by its two
        # neighbours, not by an independent glove angle.
        middle = result[8]
        middle.robot_q_target_preview = 0.5 * (
            result[4].robot_q_target_preview + result[12].robot_q_target_preview
        )
        middle.human_joint_mapping = "neighbor AA interpolation: 0.5 * (J04 ring + J12 index)"
        middle.gain_deg_per_human_deg = 0.0
        middle.human_delta_deg = 0.0
        return result


__all__ = ["POSES", "WUJI_SOURCE_BY_JOINT", "WujiCalibrationManager"]
