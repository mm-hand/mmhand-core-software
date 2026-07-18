"""Runtime contract shared by the Wuji publisher, UI, and bundled URDF."""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Sequence


PACKAGE_ROOT = Path(__file__).resolve().parent
ASSET_ROOT = PACKAGE_ROOT / "assets" / "hku_hand_v2_urdf"
URDF_PATH = ASSET_ROOT / "urdf" / "hand.urdf"
CONTRACT_PATH = ASSET_ROOT / "urdf_contract.json"


def _load_contract() -> dict[str, Any]:
    data = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    joints = sorted(data.get("joints", []), key=lambda item: int(item["joint_index"]))
    if [int(item["joint_index"]) for item in joints] != list(range(21)):
        raise RuntimeError("URDF contract must contain ordered J00-J20")
    data["joints"] = joints
    return data


CONTRACT = _load_contract()
JOINT_COUNT = len(CONTRACT["joints"])
JOINT_NAMES = tuple(str(item["name"]) for item in CONTRACT["joints"])
JOINT_LIMITS_DEG = tuple(
    (float(item["limit_deg"][0]), float(item["limit_deg"][1]))
    for item in CONTRACT["joints"]
)
TOPIC_NAME = str(CONTRACT["topic"])
INPUT_SPACE = str(CONTRACT["input_space"])
MODEL_ID = (
    f"{CONTRACT['mapping_id']}:v{CONTRACT['mapping_version']}:"
    f"{CONTRACT['urdf_config_crc']}"
)
MESSAGE_LAYOUT_LABEL = f"mmhand:J00-J20:{INPUT_SPACE}:{MODEL_ID}"


def validate_bundled_urdf() -> None:
    digest = hashlib.sha256(URDF_PATH.read_bytes()).hexdigest().upper()
    if digest != str(CONTRACT["urdf_sha256"]).upper():
        raise RuntimeError(f"bundled URDF hash mismatch: {digest}")

    root = ET.parse(URDF_PATH).getroot()
    urdf_joints = {joint.attrib.get("name", ""): joint for joint in root.findall("joint")}
    for index, (name, expected) in enumerate(zip(JOINT_NAMES, JOINT_LIMITS_DEG)):
        joint = urdf_joints.get(name)
        if joint is None:
            raise RuntimeError(f"bundled URDF missing J{index:02d} {name}")
        limit = joint.find("limit")
        if limit is None:
            raise RuntimeError(f"bundled URDF missing limit for J{index:02d} {name}")
        actual = (
            math.degrees(float(limit.attrib["lower"])),
            math.degrees(float(limit.attrib["upper"])),
        )
        if any(abs(a - b) > 0.02 for a, b in zip(actual, expected)):
            raise RuntimeError(
                f"URDF limit mismatch J{index:02d}: actual={actual}, contract={expected}"
            )


def validate_target(values: Sequence[float], layout_label: str = "") -> str | None:
    if len(values) != JOINT_COUNT:
        return f"expected {JOINT_COUNT}, got {len(values)}"
    if not all(math.isfinite(float(value)) for value in values):
        return "contains non-finite value"
    if layout_label and layout_label != MESSAGE_LAYOUT_LABEL:
        return f"layout contract mismatch: {layout_label}"
    return None


__all__ = [
    "CONTRACT",
    "INPUT_SPACE",
    "JOINT_COUNT",
    "JOINT_LIMITS_DEG",
    "JOINT_NAMES",
    "MESSAGE_LAYOUT_LABEL",
    "MODEL_ID",
    "TOPIC_NAME",
    "URDF_PATH",
    "validate_bundled_urdf",
    "validate_target",
]
