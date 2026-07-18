"""URDF-derived Wuji fingertip contact-point transforms."""

from __future__ import annotations

import copy
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


TIP_SEMANTICS = "wuji_fingertip_cp_v1"

_FINGER_TIP_LINKS = {
    "thumb": ("thumb_tip", "thumb_fingertip_cp"),
    "index": ("index_finger_tip", "index_fingertip_cp"),
    "middle": ("middle_finger_tip", "middle_fingertip_cp"),
    "ring": ("ring_finger_tip", "ring_fingertip_cp"),
    "pinky": ("pinky_tip", "pinky_fingertip_cp"),
}


@dataclass(frozen=True)
class FingertipContactTransform:
    translation: np.ndarray
    rotation: np.ndarray


def _parse_vector(text: str, label: str) -> np.ndarray:
    try:
        value = np.asarray([float(item) for item in str(text).split()], dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid {label}: {text!r}") from exc
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"invalid {label}: {text!r}")
    return value


def _rpy_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def load_fingertip_contact_transforms(
    urdf_path: Path,
) -> dict[str, FingertipContactTransform]:
    path = Path(urdf_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"Wuji active hand URDF not found: {path}")
    root = ET.parse(path).getroot()
    joints = {joint.attrib.get("name", ""): joint for joint in root.findall("joint")}
    result: dict[str, FingertipContactTransform] = {}
    for finger_name, (tip_link, contact_link) in _FINGER_TIP_LINKS.items():
        joint_name = f"{contact_link}_fixed"
        joint = joints.get(joint_name)
        if joint is None:
            raise ValueError(f"Wuji URDF missing fixed joint {joint_name!r}")
        if joint.attrib.get("type") != "fixed":
            raise ValueError(f"Wuji URDF joint {joint_name!r} must be fixed")
        parent = joint.find("parent")
        child = joint.find("child")
        parent_link = parent.attrib.get("link", "") if parent is not None else ""
        child_link = child.attrib.get("link", "") if child is not None else ""
        if parent_link != tip_link or child_link != contact_link:
            raise ValueError(
                f"Wuji URDF joint {joint_name!r} must connect "
                f"{tip_link!r} to {contact_link!r}"
            )
        origin = joint.find("origin")
        xyz = _parse_vector(
            origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            f"{joint_name} xyz",
        )
        rpy = _parse_vector(
            origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0",
            f"{joint_name} rpy",
        )
        result[finger_name] = FingertipContactTransform(
            translation=xyz,
            rotation=_rpy_rotation(rpy),
        )
    return result


def active_fingertip_contact_transforms(
    glove: Any,
) -> tuple[str, Path, dict[str, FingertipContactTransform]]:
    profile = str(glove.get("calibration.hand_profile") or "").strip()
    if not profile:
        raise ValueError("Wuji calibration.hand_profile is empty")
    model_value = glove.get(f"calibration.hand_model_paths.{profile}")
    model_path = Path(str(model_value or "")).expanduser()
    if not str(model_value or "").strip():
        raise ValueError(f"Wuji active hand model path is empty for profile {profile!r}")
    return profile, model_path, load_fingertip_contact_transforms(model_path)


def _quat_wxyz_to_rotation(quaternion: Any) -> np.ndarray:
    value = np.asarray(quaternion, dtype=float).reshape(4)
    if not np.all(np.isfinite(value)):
        raise ValueError("non-finite Wuji tip quaternion")
    norm = float(np.linalg.norm(value))
    if norm < 1e-9:
        raise ValueError("zero-length Wuji tip quaternion")
    w, x, y, z = value / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def _rotation_to_quat_wxyz(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        value = [0.25 * scale,
                 (matrix[2, 1] - matrix[1, 2]) / scale,
                 (matrix[0, 2] - matrix[2, 0]) / scale,
                 (matrix[1, 0] - matrix[0, 1]) / scale]
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            value = [(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                     (matrix[0, 1] + matrix[1, 0]) / scale,
                     (matrix[0, 2] + matrix[2, 0]) / scale]
        elif axis == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            value = [(matrix[0, 2] - matrix[2, 0]) / scale,
                     (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                     (matrix[1, 2] + matrix[2, 1]) / scale]
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            value = [(matrix[1, 0] - matrix[0, 1]) / scale,
                     (matrix[0, 2] + matrix[2, 0]) / scale,
                     (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale]
    quaternion = np.asarray(value, dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[0] < 0.0:
        quaternion = -quaternion
    return quaternion.tolist()


def apply_fingertip_contact_points(
    skeleton: dict[str, Any],
    transforms: dict[str, FingertipContactTransform],
) -> None:
    for finger_name in _FINGER_TIP_LINKS:
        finger = skeleton.get(finger_name)
        if not isinstance(finger, dict):
            continue
        tip = finger.get("tip")
        if not isinstance(tip, dict):
            continue
        contact = transforms.get(finger_name)
        if contact is None:
            raise ValueError(f"missing fingertip_cp transform for {finger_name}")
        raw_tip = copy.deepcopy(tip)
        position = np.asarray(raw_tip.get("pos"), dtype=float).reshape(3)
        tip_rotation = _quat_wxyz_to_rotation(raw_tip.get("rot"))
        contact_rotation = tip_rotation @ contact.rotation
        corrected = copy.deepcopy(raw_tip)
        corrected["pos"] = (position + tip_rotation @ contact.translation).tolist()
        corrected["rot"] = _rotation_to_quat_wxyz(contact_rotation)
        finger["raw_tip"] = raw_tip
        finger["tip"] = corrected


__all__ = [
    "FingertipContactTransform",
    "TIP_SEMANTICS",
    "active_fingertip_contact_transforms",
    "apply_fingertip_contact_points",
    "load_fingertip_contact_transforms",
]
