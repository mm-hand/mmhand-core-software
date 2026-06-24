import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from protocol import ENCODER_COUNT

from teleop.Posture.hand_retargeting.constants import DEFAULT_URDF_PATH
from teleop.Posture.hand_retargeting.math_utils import axis_angle_rotation
from teleop.Posture.hand_retargeting.retargeting import HandRetargeter
from teleop.Posture.hand_retargeting.urdf_model import URDFRobotHandModel

from .manus_calibration import (
    DEFAULT_MANUS_SIDE,
    FINGER_TO_ROBOT_JOINTS,
    ROBOT_JOINT_LIMITS_DEG,
    ROBOT_JOINT_ROLE_BY_INDEX,
)
from .manus_receiver import ManusFrame

RAW_IK_JOINT_INDICES = list(range(ENCODER_COUNT))
FINGER_ORDER = ("pinky", "ring", "middle", "index", "thumb")
FINGER_NAME_FOR_CALIBRATION = {
    "pinky": "Pinky",
    "ring": "Ring",
    "middle": "Middle",
    "index": "Index",
    "thumb": "Thumb",
}
URDF_PREFIX_TO_JOINT_START = {
    "4": 0,
    "3": 4,
    "2": 8,
    "1": 12,
    "5": 16,
}
SEMANTIC_JOINT_TO_ENCODER_INDEX = {
    # URDF joint 名 -> 逻辑关节编号 Jxx。
    # 注意这不是电机 Mxx 映射；J16/J20 这类双电机关节由下位机再展开到物理电机。
    "little_aa_joint": 0,
    "mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint": 1,
    "mmhand_finger_4_distal_phalanx_1_pip_joint": 2,
    "mmhand_finger_4_fingertip_1_dip_joint": 3,
    "ring_aa_joint": 4,
    "mmhand_finger_3_proximal_phalanx_1_mcp_fe_joint": 5,
    "mmhand_finger_3_distal_phalanx_1_pip_joint": 6,
    "mmhand_finger_3_fingertip_1_dip_joint": 7,
    "middle_aa_joint": 8,
    "mmhand_finger_2_proximal_phalanx_1_mcp_fe_joint": 9,
    "mmhand_finger_2_distal_phalanx_1_pip_joint": 10,
    "mmhand_finger_2_fingertip_1_dip_joint": 11,
    "index_aa_joint": 12,
    "mmhand_finger_1_proximal_phalanx_1_mcp_fe_joint": 13,
    "mmhand_finger_1_distal_phalanx_1_pip_joint": 14,
    "mmhand_finger_1_fingertip_1_dip_joint": 15,
    # 拇指按当前机械手定义：CMC 是 J20，其余沿 URDF 链路顺序对应 J16-J19。
    "thumb_aa_joint": 16,
    "mmhand_thumb_1_mmhand_finger_5_proximal_phalanx_1_mcp_fe_joint": 17,
    "mmhand_thumb_1_mmhand_finger_5_distal_phalanx_1_pip_joint": 18,
    "mmhand_thumb_1_mmhand_finger_5_fingertip_1_dip_joint": 19,
    "thumb_cmc1_joint": 20,
}
FINGER_RAW_NODE_KEYS = {
    "thumb": ("metacarpal", "proximal", "distal", "tip"),
    "index": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
    "middle": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
    "ring": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
    "pinky": ("metacarpal", "proximal", "intermediate", "distal", "tip"),
}
FINGER_TARGET_ANCHOR_NODE = {
    "thumb": "metacarpal",
    "index": "metacarpal",
    "middle": "metacarpal",
    "ring": "metacarpal",
    "pinky": "metacarpal",
}
PALM_FINGER_ORDER = ("index", "middle", "ring", "pinky")
RAW_IK_REGULARIZATION = 5e-3
RAW_IK_FOUR_FINGER_REFERENCE_SCALE = 120.0
RAW_IK_MAX_ITERS = 20
RAW_IK_MAX_STEP_RAD = 0.18
RAW_IK_THUMB_TIP_WEIGHT = 1.5
RAW_IK_THUMB_FINGER_WEIGHTS = {
    "index": 1.0,
    "middle": 0.0,
    "ring": 0.0,
    "pinky": 0.0,
}
RAW_IK_THUMB_INDEX_PINCH_ENTER_M = 0.025
RAW_IK_THUMB_INDEX_PINCH_EXIT_M = 0.045
RAW_IK_THUMB_PINCH_DIR_EPS_M = 0.012
RAW_IK_THUMB_PINCH_CLOSE_DISTANCE_M = 0.006
RAW_IK_THUMB_PINCH_FAR_WEIGHT = 1.0
RAW_IK_THUMB_PINCH_CLOSE_WEIGHT = 200.0
RAW_IK_HAND_TIP_WEIGHT = 2.0
RAW_IK_THUMB_DIP_TIP_WEIGHT = 2.5
RAW_IK_THUMB_WEAK_REFERENCE_SCALE = 10.0
RAW_IK_ERGONOMICS_REFERENCE_WEIGHT = 0.0


@dataclass
class RawSkeletonIkPreviewJoint:
    joint_index: int
    joint_name: str
    skeleton_mapping: str
    target_debug: str
    robot_q_urdf_preview: float
    robot_q_target_preview: float
    joint_limit_deg: str
    valid: bool
    ik_error_mm: float
    target_xyz: str


@dataclass
class RawFingerVectorTarget:
    # raw skeleton 链路的 IK 目标。
    # anchor_tip_base: 机器人手指 root -> fingertip 的目标向量。
    # thumb_tip_base: 已解算拇指 fingertip -> 当前 fingertip 的可选目标向量。
    # display_tip_base: 只用于 UI 显示和 reference 初始化；raw IK 真正优化的是向量。
    anchor_tip_base: np.ndarray
    thumb_tip_base: Optional[np.ndarray]
    display_tip_base: np.ndarray
    hand_tip_base: Optional[np.ndarray] = None
    thumb_base_mcp_dir_base: Optional[np.ndarray] = None
    dip_tip_base: Optional[np.ndarray] = None
    thumb_pinch_active: bool = False
    thumb_tip_debug: Optional[str] = None


def _node_pos(node: Any) -> Optional[np.ndarray]:
    if not isinstance(node, dict):
        return None
    pos = node.get("pos")
    if not isinstance(pos, list) or len(pos) < 3:
        return None
    try:
        return np.array([float(pos[0]), float(pos[1]), float(pos[2])], dtype=float)
    except (TypeError, ValueError):
        return None


def _node_rot(node: Any) -> Optional[np.ndarray]:
    if not isinstance(node, dict):
        return None
    rot = node.get("rot")
    if not isinstance(rot, list) or len(rot) < 4:
        return None
    try:
        quat = np.array([float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])], dtype=float)
    except (TypeError, ValueError):
        return None
    norm = float(np.linalg.norm(quat))
    if norm < 1e-8:
        return None
    return quat / norm


def _quat_to_rotation(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in quat_wxyz]
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-8:
        return np.zeros(3, dtype=float)
    return np.asarray(vec, dtype=float) / norm


def _robot_joint_label(joint_idx: int) -> str:
    role = ROBOT_JOINT_ROLE_BY_INDEX.get(joint_idx, "")
    return f"J{joint_idx:02d} {role}".strip()


def _joint_limit_text(joint_idx: int) -> str:
    lo, hi = _raw_encoder_limit_deg(joint_idx)
    return f"[{lo:.2f}, {hi:.2f}] deg"


def _clamp_joint_deg(joint_idx: int, value: float) -> float:
    lo, hi = _raw_encoder_limit_deg(joint_idx)
    return max(lo, min(hi, float(value)))


def _encoder_index_uses_negative_urdf_flexion(joint_idx: int) -> bool:
    role = ROBOT_JOINT_ROLE_BY_INDEX.get(joint_idx, "")
    return any(token in role for token in ("MCP", "PIP", "DIP"))


def _encoder_index_is_signed_root(joint_idx: int) -> bool:
    role = ROBOT_JOINT_ROLE_BY_INDEX.get(joint_idx, "")
    return any(token in role for token in ("root", "AA", "CMC"))


ROOT_URDF_LIMIT_DEG_BY_INDEX = {
    0: (-45.0, 11.0),
    4: (-51.0, 21.0),
    8: (-24.0, 23.0),
    12: (-11.0, 30.0),
    16: (-29.0, 30.0),
    20: (-80.0, 1.0),
}
ROOT_TARGET_LIMIT_DEG_BY_INDEX = {
    0: (-45.0, 11.0),
    4: (-51.0, 21.0),
    8: (-24.0, 23.0),
    12: (-11.0, 30.0),
    16: (-29.0, 30.0),
    20: (-80.0, 1.0),
}


def _map_between_ranges(value: float, source: tuple[float, float], target: tuple[float, float]) -> float:
    source_lower, source_upper = source
    target_lower, target_upper = target
    denom = source_upper - source_lower
    if abs(denom) < 1e-8:
        return target_lower
    ratio = (float(value) - source_lower) / denom
    return target_lower + ratio * (target_upper - target_lower)


def _root_urdf_deg_to_target_deg(joint_idx: int, urdf_deg: float) -> float:
    source = ROOT_URDF_LIMIT_DEG_BY_INDEX.get(joint_idx)
    target = ROOT_TARGET_LIMIT_DEG_BY_INDEX.get(joint_idx)
    if source is None or target is None:
        return float(urdf_deg)
    return _map_between_ranges(float(urdf_deg), source, target)


def _root_target_deg_to_urdf_deg(joint_idx: int, target_deg: float) -> float:
    source = ROOT_TARGET_LIMIT_DEG_BY_INDEX.get(joint_idx)
    target = ROOT_URDF_LIMIT_DEG_BY_INDEX.get(joint_idx)
    if source is None or target is None:
        return float(target_deg)
    return _map_between_ranges(float(target_deg), source, target)


def _raw_encoder_limit_deg(joint_idx: int) -> tuple[float, float]:
    # MCP/PIP/DIP 是单向屈曲幅值，仍然使用硬件给定的 [0, max]。
    # root/AA/CMC 是双向侧摆/旋转角，raw IK 显示和发送阶段保留正负号；
    # 这里先用 URDF 运动范围作为 signed q_target 范围，避免负角被 clamp 成 0。
    if _encoder_index_is_signed_root(joint_idx):
        target_limits = ROOT_TARGET_LIMIT_DEG_BY_INDEX.get(joint_idx)
        if target_limits is not None:
            return (min(target_limits), max(target_limits))
    return ROBOT_JOINT_LIMITS_DEG[joint_idx]


def _urdf_angle_to_encoder_degrees(joint_idx: int, angle_rad: float) -> float:
    value_deg = math.degrees(float(angle_rad))
    if _encoder_index_is_signed_root(joint_idx):
        value_deg = _root_urdf_deg_to_target_deg(joint_idx, value_deg)
    elif _encoder_index_uses_negative_urdf_flexion(joint_idx):
        # URDF 里这些屈伸关节是负角弯曲，J 编号目标以 0 deg 作为开手。
        # 因此只把弯曲方向翻成正值：URDF 0 -> J 0，URDF -max -> J +max。
        value_deg = -value_deg
    return value_deg


def _encoder_degrees_to_urdf_angle(joint_idx: int, value_deg: float) -> float:
    value_deg = _clamp_joint_deg(joint_idx, float(value_deg))
    if _encoder_index_is_signed_root(joint_idx):
        value_deg = _root_target_deg_to_urdf_deg(joint_idx, value_deg)
    elif _encoder_index_uses_negative_urdf_flexion(joint_idx):
        # J 正角弯曲转回 URDF 负角弯曲。
        value_deg = -value_deg
    return math.radians(value_deg)


def _xyz_text(vec: Optional[np.ndarray]) -> str:
    if vec is None:
        return "-"
    return f"{vec[0]:+.4f},{vec[1]:+.4f},{vec[2]:+.4f}"


def _solution_to_encoder_degrees(joint_angles: dict[str, float]) -> List[float]:
    targets = [0.0] * ENCODER_COUNT
    for joint_name, angle_rad in joint_angles.items():
        idx = _encoder_index_for_joint(joint_name)
        if idx is None:
            continue
        targets[idx] = _urdf_angle_to_encoder_degrees(idx, float(angle_rad))
    return targets


def _encoder_index_for_joint(joint_name: str) -> Optional[int]:
    semantic_idx = SEMANTIC_JOINT_TO_ENCODER_INDEX.get(str(joint_name))
    if semantic_idx is not None:
        return semantic_idx if 0 <= semantic_idx < ENCODER_COUNT else None

    parts = str(joint_name).split("-")
    if len(parts) < 2 or parts[0] not in URDF_PREFIX_TO_JOINT_START or not parts[1].isdigit():
        return None
    idx = URDF_PREFIX_TO_JOINT_START[parts[0]] + int(parts[1]) - 1
    if not 0 <= idx < ENCODER_COUNT:
        return None
    return idx


class FullRawSkeletonRetargeter:
    """Full-hand fingertip IK from MANUS raw skeleton."""

    def __init__(self, urdf_path: Path = DEFAULT_URDF_PATH, calibration_manager: Any = None) -> None:
        self.robot = URDFRobotHandModel.from_urdf(Path(urdf_path))
        self.ik = HandRetargeter(self.robot)
        self.calibration_manager = calibration_manager
        self.robot_palm_origin, self.robot_palm_rotation = self.robot.robot_palm_frame()
        self.robot_palm_width = float(
            np.linalg.norm(
                self._robot_base_to_palm_local(self.robot.finger_chains["pinky"].base_origin)
                - self._robot_base_to_palm_local(self.robot.finger_chains["index"].base_origin)
            )
        )
        self._last_thumb_cmc_aa_mode = "analytic CMC/AA"
        self._thumb_index_pinch_active = False
        self._thumb_index_prev_direction_palm: Optional[np.ndarray] = None

    @staticmethod
    def _raw_skeleton_palm_frame(frame: ManusFrame) -> Optional[tuple[np.ndarray, np.ndarray]]:
        # MANUS 手掌坐标系：
        # O_m = hand node 的位置。
        # x_m = normalize(mean(index/middle/ring/pinky metacarpals) - O_m)。
        # y_raw = normalize(pinky_metacarpal - index_metacarpal)。
        # z_m = normalize(x_m x y_raw)，再用 hand.rot 的局部 z 方向修正正反。
        skeleton = getattr(frame, "skeleton", {}) or {}
        if not isinstance(skeleton, dict):
            return None

        hand = skeleton.get("hand")
        origin = _node_pos(hand)
        if origin is None:
            return None

        metacarpals: Dict[str, np.ndarray] = {}
        for finger_name in PALM_FINGER_ORDER:
            finger = skeleton.get(finger_name) if isinstance(skeleton.get(finger_name), dict) else {}
            pos = _node_pos(finger.get("metacarpal")) if isinstance(finger, dict) else None
            if pos is not None:
                metacarpals[finger_name] = pos

        index_pos = metacarpals.get("index")
        pinky_pos = metacarpals.get("pinky")
        if index_pos is None or pinky_pos is None:
            return None

        center = (
            np.mean(np.stack(list(metacarpals.values()), axis=0), axis=0)
            if metacarpals
            else 0.5 * (index_pos + pinky_pos)
        )

        x_axis = _normalize(center - origin)
        y_axis = _normalize(pinky_pos - index_pos)
        if float(np.linalg.norm(x_axis)) < 1e-8 or float(np.linalg.norm(y_axis)) < 1e-8:
            return None

        z_axis = _normalize(np.cross(x_axis, y_axis))
        if float(np.linalg.norm(z_axis)) < 1e-8:
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)

        hand_rot = _node_rot(hand)
        if hand_rot is not None:
            hand_rotation = _quat_to_rotation(hand_rot)
            hand_normal = hand_rotation[:, 2]
            if float(np.dot(z_axis, hand_normal)) < 0.0:
                z_axis = -z_axis

        y_axis = _normalize(np.cross(z_axis, x_axis))
        if float(np.linalg.norm(y_axis)) < 1e-8:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)

        rotation = np.stack([x_axis, y_axis, z_axis], axis=1)
        return origin, rotation

    @staticmethod
    def _raw_skeleton_to_palm_local_nodes(
        frame: ManusFrame,
    ) -> Optional[tuple[dict[str, Any], set[str]]]:
        # 把每个可用的 MANUS skeleton node 转成 MANUS palm-local：
        # p_i^m = (p_i - O_m)^T R_m。代码里使用 row-vector 写法。
        palm = FullRawSkeletonRetargeter._raw_skeleton_palm_frame(frame)
        if palm is None:
            return None
        palm_origin, palm_rotation = palm

        skeleton = getattr(frame, "skeleton", {}) or {}
        if not isinstance(skeleton, dict):
            return None

        local_nodes: dict[str, Any] = {}
        valid_fingers: set[str] = set()

        def localize_node(node: Any) -> Optional[dict[str, Any]]:
            pos = _node_pos(node)
            if pos is None:
                return None
            return {
                "pos": (pos - palm_origin) @ palm_rotation,
                "rot": _node_rot(node),
                "raw": node,
            }

        hand_local = localize_node(skeleton.get("hand"))
        if hand_local is None:
            return None
        local_nodes["hand"] = hand_local

        for finger_name in FINGER_RAW_NODE_KEYS:
            finger = skeleton.get(finger_name) if isinstance(skeleton.get(finger_name), dict) else {}
            if not isinstance(finger, dict):
                continue
            finger_ok = True
            local_finger: dict[str, Any] = {}
            for node_key in FINGER_RAW_NODE_KEYS[finger_name]:
                local_node = localize_node(finger.get(node_key))
                if local_node is None:
                    finger_ok = False
                    continue
                local_finger[node_key] = local_node
            if finger_ok:
                valid_fingers.add(finger_name)
            if local_finger:
                local_nodes[finger_name] = local_finger

        index = skeleton.get("index") if isinstance(skeleton.get("index"), dict) else {}
        if (
            not isinstance(index, dict)
            or "index" not in local_nodes
            or "metacarpal" not in local_nodes["index"]
        ):
            return None

        return local_nodes, valid_fingers

    def _raw_targets(
        self,
        frame: ManusFrame,
    ) -> tuple[dict[str, RawFingerVectorTarget], Optional[dict[str, Any]], set[str], str]:
        local = self._raw_skeleton_to_palm_local_nodes(frame)
        if local is None:
            return {}, None, set(), "missing required hand/index/pinky metacarpal nodes for palm frame"
        local_nodes, valid_fingers = local
        try:
            targets = self._retarget_available_tip_targets(local_nodes, valid_fingers)
        except Exception as exc:
            return {}, None, valid_fingers, f"full raw skeleton retarget failed: {exc}"
        return targets, local_nodes, valid_fingers, "MANUS palm-local vectors -> robot palm-local vector targets -> IK"

    def _robot_base_to_palm_local(self, point_base: np.ndarray) -> np.ndarray:
        # p_r_local = R_r^T (p_r_base - O_r)。代码里写成 row-vector 乘法。
        return (np.asarray(point_base, dtype=float) - self.robot_palm_origin) @ self.robot_palm_rotation

    def _robot_palm_local_to_base(self, point_local: np.ndarray) -> np.ndarray:
        # p_r_base = O_r + R_r p_r_local。
        return self.robot_palm_origin + self.robot_palm_rotation @ np.asarray(point_local, dtype=float)

    def _robot_thumb_base_mcp_align_vector_palm(self) -> Optional[np.ndarray]:
        chain = self.robot.finger_chains.get("thumb")
        if chain is None:
            return None
        # 用户确认的 thumb align：CMC=0deg, AA=52deg, MCP/PIP/DIP=0deg。
        # 这里取 robot base->MCP 向量，作为 MANUS thumb 第一段相对变化的 robot 起点。
        vector_base = self._thumb_base_to_mcp_vector(0.0, math.radians(52.0))
        if float(np.linalg.norm(vector_base)) < 1e-8:
            return None
        root_palm = self._robot_base_to_palm_local(chain.base_origin)
        mcp_palm = self._robot_base_to_palm_local(chain.base_origin + vector_base)
        vector_palm = mcp_palm - root_palm
        return vector_palm if float(np.linalg.norm(vector_palm)) > 1e-8 else None

    def _retarget_available_tip_targets(
        self,
        local_nodes: dict[str, Any],
        valid_fingers: set[str],
    ) -> dict[str, RawFingerVectorTarget]:
        # 把 MANUS palm-local 向量 retarget 成 robot palm-local 向量。
        # 向量 1，所有手指：anchor -> fingertip。
        # 向量 2，非拇指：thumb fingertip -> 当前 fingertip。
        # 坐标变换由同一次 open 标定得到：
        # v_robot_palm = s_palm * R_m2r * v_manus_palm。
        anchor_vectors_palm: dict[str, np.ndarray] = {}
        human_tips: dict[str, np.ndarray] = {}
        human_palm_width = self._human_palm_width(local_nodes)
        if human_palm_width <= 1e-8:
            return {}
        palm_scale, manus_to_robot_rotation, open_anchor_vectors = self._manus_to_robot_palm_alignment(local_nodes)

        for finger_name, chain in self.robot.finger_chains.items():
            if finger_name not in valid_fingers:
                continue
            local_finger = local_nodes.get(finger_name, {})
            anchor_tip = self._human_anchor_tip_vector(finger_name, local_finger)
            if anchor_tip is None:
                continue
            anchor_pos, tip_pos, anchor_to_tip = anchor_tip
            human_tips[finger_name] = tip_pos

            reference_anchor_tip = self._robot_reference_anchor_tip_vector_palm(finger_name)
            open_anchor_to_tip = open_anchor_vectors.get(finger_name)
            if reference_anchor_tip is not None and open_anchor_to_tip is not None:
                # 标定姿态定义 robot 零位：v_target = v_robot_align + s R (v_current - v_manus_align)。
                # 这样并拢/伸直标定附近会回到 q≈0，弯曲时 IK 追的是 MANUS 相对标定姿态的变化量。
                anchor_tip_palm = reference_anchor_tip + (
                    manus_to_robot_rotation @ (anchor_to_tip - open_anchor_to_tip)
                ) * palm_scale
            else:
                # 没有标定向量时退回绝对向量映射，保证未标定也能预览。
                anchor_tip_palm = (manus_to_robot_rotation @ anchor_to_tip) * palm_scale
            if float(np.linalg.norm(anchor_tip_palm)) < 1e-6:
                continue

            reach = float(np.linalg.norm(anchor_tip_palm))
            if reach > chain.max_reach * 0.995:
                anchor_tip_palm = _normalize(anchor_tip_palm) * chain.max_reach * 0.995
            anchor_vectors_palm[finger_name] = anchor_tip_palm

        targets: dict[str, RawFingerVectorTarget] = {}
        thumb_vector = anchor_vectors_palm.get("thumb")
        thumb_tip = human_tips.get("thumb")
        hand_tip_base_by_finger: dict[str, np.ndarray] = {}
        hand_node = local_nodes.get("hand")
        if isinstance(hand_node, dict):
            for finger_name in self.robot.finger_chains:
                local_finger = local_nodes.get(finger_name)
                if not isinstance(local_finger, dict) or local_finger.get("tip") is None:
                    continue
                hand_to_tip = local_finger["tip"]["pos"] - hand_node["pos"]
                if float(np.linalg.norm(hand_to_tip)) <= 1e-8:
                    continue
                hand_tip_palm = (manus_to_robot_rotation @ hand_to_tip) * palm_scale
                hand_tip_base_by_finger[finger_name] = self.robot_palm_rotation @ hand_tip_palm
        thumb_base_mcp_dir_base = self._human_thumb_base_mcp_direction_base(
            local_nodes,
            manus_to_robot_rotation,
            palm_scale,
        )
        thumb_dip_tip_base = None
        thumb_nodes = local_nodes.get("thumb")
        if isinstance(thumb_nodes, dict):
            thumb_distal = thumb_nodes.get("distal")
            thumb_tip_node = thumb_nodes.get("tip")
            if thumb_distal is not None and thumb_tip_node is not None:
                thumb_dip_tip = thumb_tip_node["pos"] - thumb_distal["pos"]
                if float(np.linalg.norm(thumb_dip_tip)) > 1e-8:
                    thumb_dip_tip_palm = (manus_to_robot_rotation @ thumb_dip_tip) * palm_scale
                    thumb_dip_tip_base = self.robot_palm_rotation @ thumb_dip_tip_palm
        thumb_display_palm = None
        if thumb_vector is not None and "thumb" in self.robot.finger_chains:
            thumb_root_palm = self._robot_base_to_palm_local(self.robot.finger_chains["thumb"].base_origin)
            thumb_display_palm = thumb_root_palm + thumb_vector

        d_human_thumb_index: Optional[float] = None
        if thumb_tip is not None and "index" in human_tips:
            d_human_thumb_index = float(np.linalg.norm(human_tips["index"] - thumb_tip))
            if self._thumb_index_pinch_active:
                if d_human_thumb_index > RAW_IK_THUMB_INDEX_PINCH_EXIT_M:
                    self._thumb_index_pinch_active = False
                    self._thumb_index_prev_direction_palm = None
            elif d_human_thumb_index < RAW_IK_THUMB_INDEX_PINCH_ENTER_M:
                self._thumb_index_pinch_active = True

        for finger_name, anchor_tip_palm in anchor_vectors_palm.items():
            thumb_tip_palm: Optional[np.ndarray] = None
            thumb_tip_debug: Optional[str] = None
            thumb_pinch_active = False
            if (
                finger_name != "thumb"
                and thumb_display_palm is not None
                and thumb_tip is not None
                and finger_name in human_tips
                and human_palm_width > 1e-8
            ):
                thumb_to_tip = human_tips[finger_name] - thumb_tip
                human_distance = float(np.linalg.norm(thumb_to_tip))
                # v_thumb_tip^r,palm 用掌宽缩放，因为它描述的是跨手掌的两指尖关系，
                # 不是沿单根手指方向的长度关系。
                thumb_tip_palm = (manus_to_robot_rotation @ thumb_to_tip) * palm_scale

            if thumb_tip_palm is not None:
                distance = float(np.linalg.norm(thumb_tip_palm))
                current_direction = _normalize(thumb_tip_palm)
                is_index_pair = finger_name == "index"
                thumb_pinch_active = (
                    is_index_pair
                    and self._thumb_index_pinch_active
                    and distance > 1e-8
                    and RAW_IK_THUMB_FINGER_WEIGHTS.get(finger_name, 0.0) > 0.0
                )
                if thumb_pinch_active:
                    if (
                        distance < RAW_IK_THUMB_PINCH_DIR_EPS_M
                        and self._thumb_index_prev_direction_palm is not None
                    ):
                        current_direction = self._thumb_index_prev_direction_palm
                    else:
                        self._thumb_index_prev_direction_palm = current_direction
                    thumb_tip_palm = current_direction * RAW_IK_THUMB_PINCH_CLOSE_DISTANCE_M
                elif is_index_pair:
                    self._thumb_index_prev_direction_palm = None
                active_pairs = "thumb-index" if self._thumb_index_pinch_active else "none"
                if is_index_pair:
                    d_human_text = (
                        f"{d_human_thumb_index * 1000.0:.1f}mm"
                        if d_human_thumb_index is not None
                        else "-"
                    )
                    thumb_tip_debug = (
                        f"pair=thumb-index "
                        f"d_human_ti={d_human_text} "
                        f"is_index_pinch={'yes' if self._thumb_index_pinch_active else 'no'} "
                        f"target_thumb_index_norm={float(np.linalg.norm(thumb_tip_palm)) * 1000.0:.1f}mm "
                        f"shape_weight={RAW_IK_THUMB_TIP_WEIGHT:.2f} "
                        f"skip_thumb_index_collision={'yes' if thumb_pinch_active else 'no'} "
                        f"active_s1_pairs={active_pairs}"
                    )
                else:
                    thumb_tip_debug = (
                        f"pair=thumb-{finger_name} "
                        f"d_human={human_distance * 1000.0:.1f}mm "
                        f"is_index_pinch={'yes' if self._thumb_index_pinch_active else 'no'} "
                        f"target_norm={float(np.linalg.norm(thumb_tip_palm)) * 1000.0:.1f}mm "
                        f"active_s1_pairs={active_pairs}"
                    )

            root_palm = self._robot_base_to_palm_local(self.robot.finger_chains[finger_name].base_origin)
            display_tip_palm = root_palm + anchor_tip_palm
            targets[finger_name] = RawFingerVectorTarget(
                anchor_tip_base=self.robot_palm_rotation @ anchor_tip_palm,
                thumb_tip_base=(
                    self.robot_palm_rotation @ thumb_tip_palm
                    if thumb_tip_palm is not None
                    else None
                ),
                display_tip_base=self._robot_palm_local_to_base(display_tip_palm),
                hand_tip_base=hand_tip_base_by_finger.get(finger_name),
                thumb_base_mcp_dir_base=thumb_base_mcp_dir_base if finger_name == "thumb" else None,
                dip_tip_base=thumb_dip_tip_base if finger_name == "thumb" else None,
                thumb_pinch_active=thumb_pinch_active,
                thumb_tip_debug=thumb_tip_debug,
            )
        return targets

    def _human_thumb_base_mcp_direction_base(
        self,
        local_nodes: dict[str, Any],
        manus_to_robot_rotation: np.ndarray,
        palm_scale: float,
    ) -> Optional[np.ndarray]:
        # MANUS thumb 第一段：metacarpal -> proximal。
        # 不再追绝对方向，而是追相对 align 的变化：
        #   v_target = v_robot_align + s * R_m2r * (v_current - v_manus_align)
        # 然后只取方向，交给 CMC(J20)+AA(J16) 的解析解。
        thumb = local_nodes.get("thumb", {})
        if not isinstance(thumb, dict):
            return None
        metacarpal = thumb.get("metacarpal")
        proximal = thumb.get("proximal")
        if metacarpal is None or proximal is None:
            return None
        vector_manus = proximal["pos"] - metacarpal["pos"]
        if float(np.linalg.norm(vector_manus)) < 1e-8:
            return None

        open_thumb_vector = None
        calibration = self.calibration_manager
        if calibration is not None:
            getter = getattr(calibration, "raw_open_thumb_base_mcp_vector", None)
            if callable(getter):
                open_thumb_vector = getter()
        robot_align_vector = self._robot_thumb_base_mcp_align_vector_palm()
        if (
            open_thumb_vector is not None
            and robot_align_vector is not None
            and float(np.linalg.norm(open_thumb_vector)) > 1e-8
        ):
            direction_palm = robot_align_vector + (
                manus_to_robot_rotation @ (vector_manus - open_thumb_vector)
            ) * float(palm_scale)
        else:
            # 老标定文件没有 thumb align 向量时，临时退回旧逻辑；重新 Record Align 后走相对 align。
            direction_palm = manus_to_robot_rotation @ _normalize(vector_manus)

        if float(np.linalg.norm(direction_palm)) < 1e-8:
            return None
        direction_base = self.robot_palm_rotation @ direction_palm
        if float(np.linalg.norm(direction_base)) < 1e-8:
            return None
        return _normalize(direction_base)

    def _manus_to_robot_palm_alignment(self, local_nodes: dict[str, Any]) -> tuple[float, np.ndarray, dict[str, np.ndarray]]:
        # 标定得到的是 MANUS palm-local 到 robot palm-local 的向量相似变换：
        # v_r = s * R_m2r * v_m。
        # s 使用同一次 Record Open 的掌宽；R_m2r 用 open 下四指 anchor->tip 方向和
        # robot q_target=0 下 root->tip 方向做 Wahba/Kabsch 对齐。
        current_width = self._human_palm_width(local_nodes)
        open_width = None
        open_vectors: dict[str, np.ndarray] = {}
        calibration = self.calibration_manager
        if calibration is not None:
            getter = getattr(calibration, "raw_open_palm_width", None)
            if callable(getter):
                open_width = getter()
            vector_getter = getattr(calibration, "raw_open_anchor_tip_vectors", None)
            if callable(vector_getter):
                open_vectors = vector_getter()

        scale_width = open_width if open_width is not None and open_width > 1e-8 else current_width
        palm_scale = self.robot_palm_width / scale_width if scale_width > 1e-8 else 1.0

        rotation = self._solve_manus_to_robot_rotation(open_vectors)
        return palm_scale, rotation, open_vectors

    def _solve_manus_to_robot_rotation(self, manus_open_vectors: dict[str, np.ndarray]) -> np.ndarray:
        if not manus_open_vectors:
            return np.eye(3, dtype=float)

        source_vectors: list[np.ndarray] = []
        target_vectors: list[np.ndarray] = []
        for finger_name in ("index", "middle", "ring", "pinky"):
            human_vec = manus_open_vectors.get(finger_name)
            robot_vec = self._robot_reference_anchor_tip_vector_palm(finger_name)
            if human_vec is None or robot_vec is None:
                continue
            human_dir = _normalize(human_vec)
            robot_dir = _normalize(robot_vec)
            if float(np.linalg.norm(human_dir)) < 1e-8 or float(np.linalg.norm(robot_dir)) < 1e-8:
                continue
            source_vectors.append(human_dir)
            target_vectors.append(robot_dir)

        if len(source_vectors) < 2:
            return np.eye(3, dtype=float)

        covariance = np.zeros((3, 3), dtype=float)
        for source, target in zip(source_vectors, target_vectors):
            covariance += np.outer(target, source)
        u_mat, _singular, vt_mat = np.linalg.svd(covariance)
        rotation = u_mat @ vt_mat
        if float(np.linalg.det(rotation)) < 0.0:
            u_mat[:, -1] *= -1.0
            rotation = u_mat @ vt_mat
        return rotation

    def _robot_reference_anchor_tip_vector_palm(self, finger_name: str) -> Optional[np.ndarray]:
        chain = self.robot.finger_chains.get(finger_name)
        if chain is None:
            return None
        reference = {}
        for joint_name in chain.revolute_joint_names:
            idx = _encoder_index_for_joint(joint_name)
            reference[joint_name] = _encoder_degrees_to_urdf_angle(idx, 0.0) if idx is not None else 0.0
        tip_base = self.robot.forward_tip_position(finger_name, reference)
        root_base = chain.base_origin
        return self._robot_base_to_palm_local(tip_base) - self._robot_base_to_palm_local(root_base)

    @staticmethod
    def _human_palm_width(local_nodes: dict[str, Any]) -> float:
        index = local_nodes.get("index", {}).get("metacarpal")
        pinky = local_nodes.get("pinky", {}).get("metacarpal")
        if index is None or pinky is None:
            return 0.0
        return float(np.linalg.norm(pinky["pos"] - index["pos"]))

    @staticmethod
    def _human_anchor_tip_vector(
        finger_name: str,
        local_finger: dict[str, Any],
    ) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
        anchor_key = FINGER_TARGET_ANCHOR_NODE[finger_name]
        anchor = local_finger.get(anchor_key)
        tip = local_finger.get("tip")
        if anchor is None or tip is None:
            return None
        anchor_pos = anchor["pos"]
        tip_pos = tip["pos"]
        vector = tip_pos - anchor_pos
        if float(np.linalg.norm(vector)) < 1e-8:
            return None
        return anchor_pos, tip_pos, vector

    def _build_raw_reference_joint_angles(
        self,
        finger_name: str,
        frame: ManusFrame,
    ) -> dict[str, float]:
        chain = self.robot.finger_chains[finger_name]
        ergonomics_reference = self._ergonomics_reference_joint_angles(frame)
        reference: dict[str, float] = {}

        # raw skeleton 的 IK 目标已经由向量约束给出，reference 只负责收束到稳定解：
        # q_ref = 0.7 q_last + 0.3 q_neutral
        # 如果开启 ergonomics 映射，则轻量加入 q_ergo；当前诊断阶段权重设为 0。
        # 这里不再构造 MediaPipe 风格的 21 点数组。
        for joint_name in chain.revolute_joint_names:
            spec = self.robot.joint_specs[joint_name]
            last = float(self.ik.last_joint_angles.get(joint_name, 0.0))
            neutral = max(spec.lower, min(spec.upper, 0.0))
            ergonomics = ergonomics_reference.get(joint_name)

            if ergonomics is None:
                value = 0.7 * last + 0.3 * neutral
            else:
                value = float(ergonomics)
            reference[joint_name] = max(spec.lower, min(spec.upper, value))

        return reference

    def _ergonomics_reference_joint_angles(self, frame: ManusFrame) -> dict[str, float]:
        calibration = self.calibration_manager
        if calibration is None or not getattr(calibration, "has_ergonomics_reference", lambda: False)():
            return {}

        try:
            preview = calibration.robot_q_target_preview(frame, side=DEFAULT_MANUS_SIDE)
        except Exception:
            return {}

        reference: dict[str, float] = {}
        preview_by_index = {
            int(item.joint_index): item
            for item in preview
            if 0 <= int(item.joint_index) < ENCODER_COUNT
        }
        for joint_name in self.robot.revolute_joint_order:
            idx = self._encoder_index_for_urdf_joint(joint_name)
            if idx is None:
                continue
            item = preview_by_index.get(idx)
            if item is None:
                continue
            mapping = str(getattr(item, "human_joint_mapping", ""))
            if float(getattr(item, "gain_deg_per_human_deg", 0.0)) == 0.0 and "neighbor AA interpolation" not in mapping:
                continue
            value_rad = _encoder_degrees_to_urdf_angle(idx, float(item.robot_q_target_preview))
            spec = self.robot.joint_specs[joint_name]
            reference[joint_name] = max(spec.lower, min(spec.upper, value_rad))
        return reference

    @staticmethod
    def _encoder_index_for_urdf_joint(joint_name: str) -> Optional[int]:
        return _encoder_index_for_joint(joint_name)

    def preview(self, frame: ManusFrame) -> List[RawSkeletonIkPreviewJoint]:
        targets, local_nodes, valid_input_fingers, mapping = self._raw_targets(frame)
        if not targets or local_nodes is None:
            return [
                RawSkeletonIkPreviewJoint(
                    joint_index=idx,
                    joint_name=_robot_joint_label(idx),
                    skeleton_mapping=mapping,
                    target_debug="-",
                    robot_q_urdf_preview=0.0,
                    robot_q_target_preview=0.0,
                    joint_limit_deg=_joint_limit_text(idx),
                    valid=False,
                    ik_error_mm=0.0,
                    target_xyz="-",
                )
                for idx in RAW_IK_JOINT_INDICES
            ]

        solved_angles: dict[str, float] = {}
        finger_target_text: dict[str, str] = {}
        finger_target_debug: dict[str, str] = {}
        finger_error_mm: dict[str, float] = {}
        valid_fingers: set[str] = set()

        thumb_tip_base: Optional[np.ndarray] = None
        solved_tip_by_finger: dict[str, np.ndarray] = {}
        solve_order = ("index", "middle", "ring", "pinky", "thumb")
        for finger_name in solve_order:
            # 先解拇指，后面的四指才能使用 thumb->tip 向量目标。
            target = targets.get(finger_name)
            if target is None or finger_name not in self.robot.finger_chains:
                continue
            reference = self._build_raw_reference_joint_angles(finger_name, frame)
            if finger_name == "thumb":
                thumb_finger_constraints = {
                    other_name: (
                        tip_base,
                        other_target.thumb_tip_base,
                        bool(other_target.thumb_pinch_active),
                    )
                    for other_name, tip_base in solved_tip_by_finger.items()
                    for other_target in [targets.get(other_name)]
                    if other_name == "index"
                    if other_target is not None and other_target.thumb_tip_base is not None
                }
                solution = self._solve_thumb_hybrid_ik(
                    target,
                    reference,
                    thumb_finger_constraints=thumb_finger_constraints,
                )
            else:
                solution = self._solve_finger_vector_ik(
                    finger_name,
                    target,
                    reference,
                    thumb_tip_base=thumb_tip_base,
                    max_iters=RAW_IK_MAX_ITERS,
                    regularization_scale=RAW_IK_FOUR_FINGER_REFERENCE_SCALE,
                )
            solved_angles.update(solution)

            solved_tip = self.robot.forward_tip_position(finger_name, solution)
            anchor_error = (solved_tip - self.robot.finger_chains[finger_name].base_origin) - target.anchor_tip_base
            error_terms = [float(np.linalg.norm(anchor_error))]
            if target.hand_tip_base is not None:
                hand_error = (solved_tip - self.robot_palm_origin) - target.hand_tip_base
                error_terms.append(float(np.linalg.norm(hand_error)))
            if finger_name == "thumb" and target.dip_tip_base is not None:
                thumb_chain = self.robot.finger_chains.get("thumb")
                if thumb_chain is not None and thumb_chain.revolute_joint_names:
                    dip_joint = thumb_chain.revolute_joint_names[-1]
                    positions = self.robot.forward_joint_positions("thumb", solution)
                    dip_pos = positions.get(dip_joint)
                    tip_pos = positions.get("tip")
                    if dip_pos is not None and tip_pos is not None:
                        dip_error = (tip_pos - dip_pos) - target.dip_tip_base
                        error_terms.append(float(np.linalg.norm(dip_error)))
            if finger_name == "thumb":
                for other_name, other_tip_base in solved_tip_by_finger.items():
                    other_target = targets.get(other_name)
                    if other_target is None or other_target.thumb_tip_base is None:
                        continue
                    thumb_error = (other_tip_base - solved_tip) - other_target.thumb_tip_base
                    error_terms.append(float(np.linalg.norm(thumb_error)))
            finger_error_mm[finger_name] = float(np.mean(error_terms) * 1000.0)
            if target.hand_tip_base is not None:
                finger_target_text[finger_name] = (
                    f"a2tip={_xyz_text(target.anchor_tip_base)} "
                    f"h2tip={_xyz_text(target.hand_tip_base)}"
                )
            else:
                finger_target_text[finger_name] = f"a2tip={_xyz_text(target.anchor_tip_base)}"
            if finger_name == "thumb" and target.dip_tip_base is not None:
                finger_target_text[finger_name] = (
                    f"{finger_target_text[finger_name]} "
                    f"d2tip={_xyz_text(target.dip_tip_base)}"
                )
            debug_text = self._target_vs_calibration_basis_debug(finger_name, target)
            if target.hand_tip_base is not None:
                debug_text = f"hand-tip weight={RAW_IK_HAND_TIP_WEIGHT:.1f}; {debug_text}"
            if finger_name == "thumb" and target.dip_tip_base is not None:
                debug_text = f"dip-tip weight={RAW_IK_THUMB_DIP_TIP_WEIGHT:.1f}; {debug_text}"
            if finger_name == "thumb":
                pinch_debug_parts: list[str] = []
                current_thumb_constraints = {
                    other_name: (
                        tip_base,
                        other_target.thumb_tip_base,
                        bool(other_target.thumb_pinch_active),
                    )
                    for other_name, tip_base in solved_tip_by_finger.items()
                    for other_target in [targets.get(other_name)]
                    if other_name == "index"
                    if other_target is not None and other_target.thumb_tip_base is not None
                }
                dynamic_pinch_weights = self._thumb_dynamic_pinch_weights(current_thumb_constraints)
                for other_name, pair in current_thumb_constraints.items():
                    other_tip_base, target_thumb_to_finger_base, is_active_pinch = pair
                    other_target = targets.get(other_name)
                    d_robot = float(np.linalg.norm(other_tip_base - solved_tip))
                    target_norm = float(np.linalg.norm(target_thumb_to_finger_base))
                    residual_weight = RAW_IK_THUMB_TIP_WEIGHT * dynamic_pinch_weights.get(other_name, 0.0)
                    pair_debug = other_target.thumb_tip_debug if other_target is not None else None
                    pinch_debug_parts.append(
                        f"{pair_debug or ('pair=thumb-' + other_name)} "
                        f"d_robot_ti={d_robot * 1000.0:.1f}mm "
                        f"pinch_weight={residual_weight:.2f} "
                        f"skip_thumb_index_collision={'yes' if is_active_pinch else 'no'} "
                        f"thumb_{self._finger_limit_debug('thumb', solved_angles)} "
                        f"{other_name}_{self._finger_limit_debug(other_name, solved_angles)} "
                        f"target_check={target_norm * 1000.0:.1f}mm"
                    )
                if pinch_debug_parts:
                    debug_text = f"{'; '.join(pinch_debug_parts)}; {debug_text}"
                chain = self.robot.finger_chains["thumb"]
                if len(chain.revolute_joint_names) >= 2:
                    cmc_joint, aa_joint = chain.revolute_joint_names[:2]
                    if cmc_joint in solution and aa_joint in solution:
                        debug_text = (
                            f"{self._last_thumb_cmc_aa_mode} "
                            f"J20={math.degrees(solution[cmc_joint]):+.1f}deg "
                            f"J16={math.degrees(solution[aa_joint]):+.1f}deg; "
                            f"{debug_text}"
                        )
            finger_target_debug[finger_name] = debug_text
            if finger_name == "thumb":
                thumb_tip_base = solved_tip
            solved_tip_by_finger[finger_name] = solved_tip
            valid_fingers.add(finger_name)

        final_thumb_tip = solved_tip_by_finger.get("thumb")
        if final_thumb_tip is not None:
            best_pair_name: Optional[str] = None
            best_pair_distance = float("inf")
            for candidate_name in ("index",):
                candidate_target = targets.get(candidate_name)
                candidate_tip = solved_tip_by_finger.get(candidate_name)
                if candidate_target is None or candidate_tip is None or candidate_target.thumb_tip_base is None:
                    continue
                target_distance = float(np.linalg.norm(candidate_target.thumb_tip_base))
                if (
                    candidate_target.thumb_pinch_active
                    and target_distance < best_pair_distance
                ):
                    best_pair_name = candidate_name
                    best_pair_distance = target_distance

            if best_pair_name is not None:
                pair_target = targets[best_pair_name]
                reference = self._build_raw_reference_joint_angles(best_pair_name, frame)
                for joint_name in self.robot.finger_chains[best_pair_name].revolute_joint_names:
                    if joint_name in solved_angles:
                        reference[joint_name] = solved_angles[joint_name]
                pair_solution = self._solve_finger_vector_ik(
                    best_pair_name,
                    pair_target,
                    reference,
                    thumb_tip_base=final_thumb_tip,
                    max_iters=RAW_IK_MAX_ITERS * 2,
                    regularization_scale=RAW_IK_FOUR_FINGER_REFERENCE_SCALE * 0.2,
                    anchor_weight=0.25,
                    hand_tip_weight=0.35,
                    thumb_pinch_weight_scale=2.0,
                )
                solved_angles.update(pair_solution)
                pair_tip = self.robot.forward_tip_position(best_pair_name, pair_solution)
                solved_tip_by_finger[best_pair_name] = pair_tip
                pair_error = (pair_tip - final_thumb_tip) - pair_target.thumb_tip_base
                finger_error_mm[best_pair_name] = float(np.linalg.norm(pair_error) * 1000.0)
                post_gap = float(np.linalg.norm(pair_tip - final_thumb_tip))
                finger_target_debug["thumb"] = (
                    f"post {best_pair_name} two-sided d_robot={post_gap * 1000.0:.1f}mm; "
                    f"{finger_target_debug.get('thumb', '-')}"
                )
                finger_target_debug[best_pair_name] = (
                    f"two-sided pinch to thumb d_robot={post_gap * 1000.0:.1f}mm; "
                    f"{finger_target_debug.get(best_pair_name, '-')}"
                )

        self.ik.last_joint_angles.update(solved_angles)
        degrees = _solution_to_encoder_degrees(solved_angles)
        urdf_degrees = [0.0] * ENCODER_COUNT
        for joint_name, angle_rad in solved_angles.items():
            idx = _encoder_index_for_joint(joint_name)
            if idx is not None and 0 <= idx < ENCODER_COUNT:
                urdf_degrees[idx] = math.degrees(float(angle_rad))

        out: List[RawSkeletonIkPreviewJoint] = []
        for finger_name in FINGER_ORDER:
            calibration_finger_name = FINGER_NAME_FOR_CALIBRATION[finger_name]
            for idx in FINGER_TO_ROBOT_JOINTS[calibration_finger_name]:
                is_valid = finger_name in valid_fingers
                out.append(
                    RawSkeletonIkPreviewJoint(
                        joint_index=idx,
                        joint_name=_robot_joint_label(idx),
                        skeleton_mapping=(
                            mapping
                            if is_valid
                            else (
                                f"missing {finger_name} skeleton nodes"
                                if finger_name not in valid_input_fingers
                                else f"{finger_name} IK target unavailable"
                            )
                        ),
                        target_debug=finger_target_debug.get(finger_name, "-"),
                        robot_q_urdf_preview=float(urdf_degrees[idx]) if is_valid else 0.0,
                        robot_q_target_preview=_clamp_joint_deg(idx, float(degrees[idx])) if is_valid else 0.0,
                        joint_limit_deg=_joint_limit_text(idx),
                        valid=is_valid,
                        ik_error_mm=finger_error_mm.get(finger_name, 0.0),
                        target_xyz=finger_target_text.get(finger_name, "-"),
                    )
                )
        return out

    def _target_vs_calibration_basis_debug(self, finger_name: str, target: RawFingerVectorTarget) -> str:
        chain = self.robot.finger_chains[finger_name]
        calibration_vec_palm = self._robot_reference_anchor_tip_vector_palm(finger_name)
        if calibration_vec_palm is None:
            return "target/calib unavailable"
        calibration_vec = self.robot_palm_rotation @ calibration_vec_palm
        target_vec = target.anchor_tip_base
        calibration_norm = float(np.linalg.norm(calibration_vec))
        target_norm = float(np.linalg.norm(target_vec))
        if calibration_norm < 1e-8 or target_norm < 1e-8:
            return "target/calib unavailable"
        cos_angle = float(np.dot(calibration_vec, target_vec) / (calibration_norm * target_norm))
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle_deg = math.degrees(math.acos(cos_angle))
        length_delta_mm = (target_norm - calibration_norm) * 1000.0
        return (
            f"target-vs-calib angle={angle_deg:.1f}deg "
            f"len_delta={length_delta_mm:+.1f}mm "
            f"target={target_norm * 1000.0:.1f}mm calib={calibration_norm * 1000.0:.1f}mm"
        )

    def _solve_finger_vector_ik(
        self,
        finger_name: str,
        target: RawFingerVectorTarget,
        reference: dict[str, float],
        thumb_tip_base: Optional[np.ndarray] = None,
        max_iters: int = 12,
        fixed_joint_angles: Optional[dict[str, float]] = None,
        thumb_finger_constraints: Optional[dict[str, tuple[np.ndarray, np.ndarray, bool]]] = None,
        regularization_scale: float = 1.0,
        anchor_weight: float = 1.0,
        hand_tip_weight: float = RAW_IK_HAND_TIP_WEIGHT,
        dip_tip_weight: float = RAW_IK_THUMB_DIP_TIP_WEIGHT,
        thumb_pinch_weight_scale: float = 1.0,
    ) -> dict[str, float]:
        # raw-skeleton IK 最小化的是向量残差，不是绝对 fingertip 位置：
        #   e1(q) = (FK_tip(q) - robot_root) - v_anchor_tip_target
        #   e2(q) = (FK_tip(q) - FK_thumb_tip) - v_thumb_tip_target  [只用于非拇指]
        # 下面的正规方程里还加入一个小的 reference 正则项。
        chain = self.robot.finger_chains[finger_name]
        fixed_joint_angles = dict(fixed_joint_angles or {})
        joint_names = [name for name in chain.revolute_joint_names if name not in fixed_joint_angles]

        q = np.array([reference[name] for name in joint_names], dtype=float)
        q_ref = np.array([reference[name] for name in joint_names], dtype=float)

        damping = 1e-3
        regularization = RAW_IK_REGULARIZATION * float(regularization_scale)
        eps = 1e-4
        thumb_weight = RAW_IK_THUMB_TIP_WEIGHT * float(thumb_pinch_weight_scale)
        thumb_finger_constraints = dict(thumb_finger_constraints or {})
        def residual(q_values: np.ndarray) -> np.ndarray:
            joint_map = dict(fixed_joint_angles)
            joint_map.update({name: float(value) for name, value in zip(joint_names, q_values)})
            tip = self.robot.forward_tip_position(finger_name, joint_map)
            terms = [
                ((tip - chain.base_origin) - target.anchor_tip_base) * float(anchor_weight),
            ]
            if target.hand_tip_base is not None and hand_tip_weight > 0.0:
                terms.append(
                    ((tip - self.robot_palm_origin) - target.hand_tip_base)
                    * float(hand_tip_weight)
                )
            if finger_name == "thumb" and target.dip_tip_base is not None and dip_tip_weight > 0.0:
                thumb_chain = self.robot.finger_chains.get("thumb")
                if thumb_chain is not None and thumb_chain.revolute_joint_names:
                    dip_joint = thumb_chain.revolute_joint_names[-1]
                    positions = self.robot.forward_joint_positions("thumb", joint_map)
                    dip_pos = positions.get(dip_joint)
                    tip_pos = positions.get("tip")
                    if dip_pos is not None and tip_pos is not None:
                        terms.append(
                            ((tip_pos - dip_pos) - target.dip_tip_base)
                            * float(dip_tip_weight)
                        )
            if finger_name == "thumb" and thumb_weight > 0.0:
                dynamic_pinch_weights = self._thumb_dynamic_pinch_weights(thumb_finger_constraints)
                for other_name, pair in thumb_finger_constraints.items():
                    other_tip_base, target_thumb_to_finger_base, _is_active_pinch = pair
                    finger_weight = dynamic_pinch_weights.get(other_name, 0.0)
                    if finger_weight <= 0.0:
                        continue
                    terms.append(
                        ((other_tip_base - tip) - target_thumb_to_finger_base)
                        * thumb_weight
                        * finger_weight
                    )
            if (
                finger_name != "thumb"
                and thumb_tip_base is not None
                and target.thumb_tip_base is not None
                and thumb_weight > 0.0
            ):
                distance_m = float(np.linalg.norm(target.thumb_tip_base))
                objective_weight = (
                    RAW_IK_THUMB_PINCH_CLOSE_WEIGHT
                    if distance_m <= RAW_IK_THUMB_PINCH_CLOSE_DISTANCE_M * 1.5
                    else RAW_IK_THUMB_PINCH_FAR_WEIGHT
                )
                terms.append(
                    ((tip - thumb_tip_base) - target.thumb_tip_base)
                    * thumb_weight
                    * math.sqrt(float(objective_weight))
                )
            return np.concatenate(terms)

        for _ in range(max_iters):
            error = residual(q)
            if float(np.linalg.norm(error)) < 1e-4:
                break

            jacobian = np.zeros((len(error), len(joint_names)), dtype=float)
            for i, _joint_name in enumerate(joint_names):
                q_delta = q.copy()
                q_delta[i] += eps
                jacobian[:, i] = (residual(q_delta) - error) / eps

            lhs = jacobian.T @ jacobian + (damping + regularization) * np.eye(len(joint_names))
            rhs = jacobian.T @ error + regularization * (q - q_ref)
            delta_q = -np.linalg.solve(lhs, rhs)
            q = q + np.clip(delta_q, -RAW_IK_MAX_STEP_RAD, RAW_IK_MAX_STEP_RAD)

            for i, joint_name in enumerate(joint_names):
                spec = self.robot.joint_specs[joint_name]
                q[i] = max(spec.lower, min(spec.upper, float(q[i])))

        out = dict(fixed_joint_angles)
        out.update({name: float(value) for name, value in zip(joint_names, q)})
        return out

    @staticmethod
    def _thumb_dynamic_pinch_weights(
        thumb_finger_constraints: dict[str, tuple[np.ndarray, np.ndarray, bool]],
    ) -> dict[str, float]:
        weights: dict[str, float] = {}
        for finger_name, pair in thumb_finger_constraints.items():
            base_weight = RAW_IK_THUMB_FINGER_WEIGHTS.get(finger_name, 0.0)
            if base_weight <= 0.0:
                continue
            _other_tip_base, _target_thumb_to_finger_base, is_active_pinch = pair
            objective_weight = (
                RAW_IK_THUMB_PINCH_CLOSE_WEIGHT
                if is_active_pinch
                else RAW_IK_THUMB_PINCH_FAR_WEIGHT
            )
            weights[finger_name] = float(base_weight) * math.sqrt(float(objective_weight))
        return weights

    @staticmethod
    def _thumb_close_pinch_active(
        thumb_finger_constraints: dict[str, tuple[np.ndarray, np.ndarray, bool]],
    ) -> bool:
        for _finger_name, pair in thumb_finger_constraints.items():
            _other_tip_base, _target_thumb_to_finger_base, is_active_pinch = pair
            if is_active_pinch:
                return True
        return False

    def _thumb_pinch_error(
        self,
        solution: dict[str, float],
        thumb_finger_constraints: dict[str, tuple[np.ndarray, np.ndarray, bool]],
    ) -> float:
        if not thumb_finger_constraints:
            return 0.0
        thumb_tip = self.robot.forward_tip_position("thumb", solution)
        errors = []
        for _finger_name, pair in thumb_finger_constraints.items():
            other_tip_base, target_thumb_to_finger_base, _is_active_pinch = pair
            errors.append(float(np.linalg.norm((other_tip_base - thumb_tip) - target_thumb_to_finger_base)))
        if not errors:
            return 0.0
        return float(min(errors))

    def _finger_limit_debug(self, finger_name: str, solution: dict[str, float]) -> str:
        chain = self.robot.finger_chains.get(finger_name)
        if chain is None:
            return "limit=unknown"
        near: list[str] = []
        margin = math.radians(2.0)
        for joint_name in chain.revolute_joint_names:
            if joint_name not in solution:
                continue
            spec = self.robot.joint_specs[joint_name]
            value = float(solution[joint_name])
            if min(abs(value - spec.lower), abs(spec.upper - value)) <= margin:
                idx = _encoder_index_for_joint(joint_name)
                near.append(_robot_joint_label(idx) if idx is not None else joint_name)
        return "limit=" + ("yes:" + ",".join(near) if near else "no")

    def _thumb_pinch_dominant_target(
        self,
        target: RawFingerVectorTarget,
        thumb_finger_constraints: dict[str, tuple[np.ndarray, np.ndarray, bool]],
    ) -> Optional[RawFingerVectorTarget]:
        best_pair: Optional[tuple[np.ndarray, np.ndarray, bool]] = None
        best_distance = float("inf")
        for _finger_name, pair in thumb_finger_constraints.items():
            _other_tip_base, target_thumb_to_finger_base, is_active_pinch = pair
            distance = float(np.linalg.norm(target_thumb_to_finger_base))
            if is_active_pinch and distance < best_distance:
                best_pair = pair
                best_distance = distance
        if best_pair is None:
            return None

        other_tip_base, target_thumb_to_finger_base, _is_active_pinch = best_pair
        desired_thumb_tip_base = other_tip_base - target_thumb_to_finger_base
        return RawFingerVectorTarget(
            anchor_tip_base=desired_thumb_tip_base - self.robot.finger_chains["thumb"].base_origin,
            thumb_tip_base=target.thumb_tip_base,
            display_tip_base=desired_thumb_tip_base,
            hand_tip_base=desired_thumb_tip_base - self.robot_palm_origin,
            thumb_base_mcp_dir_base=target.thumb_base_mcp_dir_base,
            dip_tip_base=target.dip_tip_base,
        )

    def _solve_thumb_hybrid_ik(
        self,
        target: RawFingerVectorTarget,
        reference: dict[str, float],
        thumb_finger_constraints: Optional[dict[str, tuple[np.ndarray, np.ndarray, bool]]] = None,
    ) -> dict[str, float]:
        chain = self.robot.finger_chains["thumb"]
        if len(chain.revolute_joint_names) < 5 or target.thumb_base_mcp_dir_base is None:
            return self._solve_finger_vector_ik(
                "thumb",
                target,
                reference,
                max_iters=RAW_IK_MAX_ITERS,
                regularization_scale=RAW_IK_THUMB_WEAK_REFERENCE_SCALE,
            )

        cmc_joint = chain.revolute_joint_names[0]
        aa_joint = chain.revolute_joint_names[1]
        cmc_aa = self._solve_thumb_cmc_aa_analytic(
            target.thumb_base_mcp_dir_base,
        )
        if cmc_aa is None:
            return self._solve_finger_vector_ik(
                "thumb",
                target,
                reference,
                max_iters=RAW_IK_MAX_ITERS,
                regularization_scale=RAW_IK_THUMB_WEAK_REFERENCE_SCALE,
            )

        thumb_finger_constraints = dict(thumb_finger_constraints or {})
        cmc_aa_reference = dict(reference)
        cmc_aa_reference[cmc_joint] = cmc_aa[0]
        cmc_aa_reference[aa_joint] = cmc_aa[1]
        fixed_solution = self._solve_finger_vector_ik(
            "thumb",
            target,
            cmc_aa_reference,
            max_iters=RAW_IK_MAX_ITERS,
            thumb_finger_constraints=thumb_finger_constraints,
            regularization_scale=RAW_IK_THUMB_WEAK_REFERENCE_SCALE,
        )
        self._last_thumb_cmc_aa_mode = "CMC/AA IK-init"
        if self._thumb_close_pinch_active(thumb_finger_constraints):
            pinch_reference = dict(reference)
            pinch_reference[cmc_joint] = cmc_aa[0]
            pinch_reference[aa_joint] = cmc_aa[1]
            free_solution = self._solve_finger_vector_ik(
                "thumb",
                target,
                pinch_reference,
                max_iters=RAW_IK_MAX_ITERS,
                thumb_finger_constraints=thumb_finger_constraints,
                regularization_scale=RAW_IK_THUMB_WEAK_REFERENCE_SCALE,
            )
            fixed_error = self._thumb_pinch_error(fixed_solution, thumb_finger_constraints)
            free_error = self._thumb_pinch_error(free_solution, thumb_finger_constraints)
            best_solution = fixed_solution
            best_error = fixed_error
            best_mode = f"pinch CMC/AA fixed fallback err={fixed_error * 1000.0:.1f}mm"
            if free_error < best_error:
                best_solution = free_solution
                best_error = free_error
                best_mode = (
                    f"pinch CMC/AA free err={free_error * 1000.0:.1f}<{fixed_error * 1000.0:.1f}mm"
                )

            dominant_target = self._thumb_pinch_dominant_target(target, thumb_finger_constraints)
            if dominant_target is not None:
                dominant_solution = self._solve_finger_vector_ik(
                    "thumb",
                    dominant_target,
                    pinch_reference,
                    max_iters=RAW_IK_MAX_ITERS * 2,
                    thumb_finger_constraints=thumb_finger_constraints,
                    regularization_scale=RAW_IK_THUMB_WEAK_REFERENCE_SCALE * 0.2,
                    anchor_weight=0.15,
                    hand_tip_weight=0.25,
                    dip_tip_weight=0.35,
                    thumb_pinch_weight_scale=2.0,
                )
                dominant_error = self._thumb_pinch_error(dominant_solution, thumb_finger_constraints)
                if dominant_error < best_error:
                    best_solution = dominant_solution
                    best_error = dominant_error
                    best_mode = (
                        f"pinch dominant err={dominant_error * 1000.0:.1f}"
                        f"<{fixed_error * 1000.0:.1f}mm"
                    )

            self._last_thumb_cmc_aa_mode = best_mode
            if best_solution is free_solution:
                self._last_thumb_cmc_aa_mode = (
                    f"pinch CMC/AA free err={free_error * 1000.0:.1f}<{fixed_error * 1000.0:.1f}mm"
                )
            return best_solution
        return fixed_solution

    def _solve_thumb_cmc_aa_analytic(
        self,
        target_direction_base: np.ndarray,
    ) -> Optional[tuple[float, float]]:
        # 解析 CMC(J20)+AA(J16)：让 robot base->MCP 方向对齐 MANUS thumb 第一段方向。
        # v(q20,q16) = R_cmc(q20) * (t_aa + R_aa(q16) * t_mcp)
        # q16 用半角替换得到解析候选；q20 用三维绕 CMC 轴的闭式对齐角求解。
        chain = self.robot.finger_chains.get("thumb")
        if chain is None or len(chain.revolute_joint_names) < 3:
            return None
        cmc_joint, aa_joint, mcp_joint = chain.revolute_joint_names[:3]
        cmc_spec = self.robot.joint_specs[cmc_joint]
        aa_spec = self.robot.joint_specs[aa_joint]
        mcp_spec = self.robot.joint_specs[mcp_joint]

        d_target = _normalize(target_direction_base)
        if float(np.linalg.norm(d_target)) < 1e-8:
            return None
        cmc_axis = _normalize(cmc_spec.axis)
        aa_axis = _normalize(aa_spec.axis)
        if abs(abs(float(cmc_axis[2])) - 1.0) > 1e-5:
            return None

        t_aa = np.asarray(aa_spec.origin_xyz, dtype=float)
        t_mcp = np.asarray(mcp_spec.origin_xyz, dtype=float)
        axis_dot = float(np.dot(aa_axis, t_mcp))
        axis_part = aa_axis * axis_dot
        a_vec = t_aa + axis_part
        b_vec = t_mcp - axis_part
        c_vec = np.cross(aa_axis, t_mcp)

        q16_candidates = self._thumb_aa_candidates_from_direction(
            a_vec,
            b_vec,
            c_vec,
            float(d_target[2]),
            aa_spec.lower,
            aa_spec.upper,
        )
        if not q16_candidates:
            return None

        best: Optional[tuple[float, float, float]] = None

        for q16 in q16_candidates:
            # 先看 q20=0 时，这个 AA 候选给出的 base->MCP 方向。
            # CMC 只负责把这个方向绕 cmc_axis 旋到目标方向。
            zero_cmc_vec = self._thumb_base_to_mcp_vector(0.0, q16)
            zero_cmc_dir = _normalize(zero_cmc_vec)
            if float(np.linalg.norm(zero_cmc_dir)) < 1e-8:
                continue

            source_parallel = cmc_axis * float(np.dot(cmc_axis, zero_cmc_dir))
            target_parallel = cmc_axis * float(np.dot(cmc_axis, d_target))
            source_perp = zero_cmc_dir - source_parallel
            target_perp = d_target - target_parallel
            if float(np.linalg.norm(source_perp)) < 1e-8 or float(np.linalg.norm(target_perp)) < 1e-8:
                q20_candidates = [cmc_spec.lower, cmc_spec.upper]
            else:
                source_perp = _normalize(source_perp)
                target_perp = _normalize(target_perp)
                q20_base = math.atan2(
                    float(np.dot(cmc_axis, np.cross(source_perp, target_perp))),
                    float(np.dot(source_perp, target_perp)),
                )
                q20_candidates = self._angle_equivalents_in_limits(q20_base, cmc_spec.lower, cmc_spec.upper)

            for q20 in q20_candidates:
                vec = self._thumb_base_to_mcp_vector(q20, q16)
                direction = _normalize(vec)
                if float(np.linalg.norm(direction)) < 1e-8:
                    continue
                direction_error = float(np.linalg.norm(direction - d_target) ** 2)
                score = direction_error
                if best is None or score < best[0]:
                    best = (score, q20, q16)

        if best is None:
            return None
        return best[1], best[2]

    def _thumb_aa_candidates_from_direction(
        self,
        a_vec: np.ndarray,
        b_vec: np.ndarray,
        c_vec: np.ndarray,
        target_z: float,
        lower: float,
        upper: float,
    ) -> list[float]:
        # u(q) = A + B cos(q) + C sin(q)
        # CMC 绕 z 轴，保留 z/||u||，所以解：
        #   u_z(q)^2 - target_z^2 * ||u(q)||^2 = 0
        # 用 x=tan(q/2) 化成四次多项式，得到 q 的解析候选。
        p0 = np.asarray(a_vec + b_vec, dtype=float)
        p1 = np.asarray(2.0 * c_vec, dtype=float)
        p2 = np.asarray(a_vec - b_vec, dtype=float)

        def square_poly(component: int) -> np.ndarray:
            poly = np.array([p0[component], p1[component], p2[component]], dtype=float)
            return np.convolve(poly, poly)

        coeff = square_poly(2) - float(target_z * target_z) * (
            square_poly(0) + square_poly(1) + square_poly(2)
        )

        candidates: list[float] = []
        trimmed = np.trim_zeros(coeff[::-1], trim="f")
        if trimmed.size > 0:
            roots = np.roots(trimmed)
            for root in roots:
                if abs(float(np.imag(root))) > 1e-7:
                    continue
                x = float(np.real(root))
                q = 2.0 * math.atan(x)
                candidates.extend(self._angle_equivalents_in_limits(q, lower, upper))

        # 目标方向如果超出 CMC/AA 可达方向，解析方程可能没有合法根。
        # thumb 没有可靠 reference，这里只加入边界候选，再回代原始方向误差选最接近的一支。
        candidates.extend([lower, upper])

        unique: list[float] = []
        for value in candidates:
            if lower - 1e-8 <= value <= upper + 1e-8 and not any(abs(value - old) < 1e-6 for old in unique):
                unique.append(max(lower, min(upper, float(value))))
        return unique

    def _thumb_local_base_to_mcp_vector(self, q16: float) -> np.ndarray:
        chain = self.robot.finger_chains["thumb"]
        aa_joint = chain.revolute_joint_names[1]
        mcp_joint = chain.revolute_joint_names[2]
        aa_spec = self.robot.joint_specs[aa_joint]
        mcp_spec = self.robot.joint_specs[mcp_joint]
        rotation = axis_angle_rotation(aa_spec.axis, q16)
        return np.asarray(aa_spec.origin_xyz, dtype=float) + rotation @ np.asarray(mcp_spec.origin_xyz, dtype=float)

    def _thumb_base_to_mcp_vector(self, q20: float, q16: float) -> np.ndarray:
        chain = self.robot.finger_chains["thumb"]
        cmc_joint, aa_joint, mcp_joint = chain.revolute_joint_names[:3]
        transform = chain.base_transform.copy()
        transform = transform @ self.robot.joint_local_transform(self.robot.joint_specs[cmc_joint], q20)
        transform = transform @ self.robot.joint_local_transform(self.robot.joint_specs[aa_joint], q16)
        transform = transform @ self.robot.joint_local_transform(self.robot.joint_specs[mcp_joint], 0.0)
        return transform[:3, 3] - chain.base_origin

    @staticmethod
    def _angle_equivalents_in_limits(angle: float, lower: float, upper: float) -> list[float]:
        out: list[float] = []
        two_pi = 2.0 * math.pi
        for k in range(-2, 3):
            candidate = float(angle) + k * two_pi
            if lower - 1e-8 <= candidate <= upper + 1e-8:
                out.append(max(lower, min(upper, candidate)))
        return out

    def target_angles(self, preview: List[RawSkeletonIkPreviewJoint]) -> List[float]:
        angles = [0.0] * ENCODER_COUNT
        for item in preview:
            if item.valid and item.joint_index in RAW_IK_JOINT_INDICES:
                angles[item.joint_index] = _clamp_joint_deg(item.joint_index, item.robot_q_target_preview)
        return angles


PinkyRawSkeletonRetargeter = FullRawSkeletonRetargeter

__all__ = ["FullRawSkeletonRetargeter", "PinkyRawSkeletonRetargeter", "RawSkeletonIkPreviewJoint"]
