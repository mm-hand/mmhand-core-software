#!/usr/bin/env python3
"""Validate an external mmhand URDF resource root before linking it to desktop."""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = PROJECT_ROOT / "desktop/resources/config/robot_model_manifest.json"


def _required_joint_names(manifest_path: Path) -> set[str]:
    """校验器只读取语义清单，不导入 Desktop 运行时代码。"""
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    joints = data.get("joints")
    if not isinstance(joints, list) or len(joints) != 21:
        raise ValueError("robot model manifest must contain 21 joints")
    names = {str(item.get("urdf_joint_name", "")) for item in joints}
    if "" in names or len(names) != 21:
        raise ValueError("robot model manifest contains invalid joint names")
    return names




def validate(resource_root: Path, manifest_path: Path = DEFAULT_MANIFEST) -> list[str]:
    errors: list[str] = []
    resource_root = resource_root.resolve()
    urdf_path = resource_root / "urdf" / "hand.urdf"
    if not urdf_path.is_file():
        return [f"missing required URDF: {urdf_path}"]

    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError as exc:
        return [f"invalid URDF XML: {exc}"]

    if root.tag != "robot":
        errors.append(f"URDF root must be <robot>, got <{root.tag}>")

    joints = {joint.get("name", ""): joint for joint in root.findall("joint")}
    try:
        required_joint_names = _required_joint_names(manifest_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return [f"invalid robot model manifest: {exc}"]
    missing_joints = sorted(required_joint_names - set(joints))
    if missing_joints:
        errors.append("missing required 21-joint mapping names: " + ", ".join(missing_joints))

    for name in sorted(required_joint_names & set(joints)):
        joint = joints[name]
        joint_type = joint.get("type", "")
        limit = joint.find("limit")
        if joint_type == "revolute":
            if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                errors.append(f"revolute joint has no lower/upper limit: {name}")
                continue
            try:
                lower = float(limit.get("lower", ""))
                upper = float(limit.get("upper", ""))
            except ValueError:
                errors.append(f"joint limit is not numeric: {name}")
                continue
            if lower >= upper:
                errors.append(f"joint limit lower >= upper: {name} ({lower} >= {upper})")
                continue

    print(f"URDF: {urdf_path}")
    print(f"Robot: {root.get('name', '<unnamed>')}")
    print(f"Joints: {len(joints)}; required mapping joints: {len(required_joint_names)}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate an external URDF root before mmhand desktop uses it."
    )
    parser.add_argument(
        "--urdf-root",
        required=True,
        type=Path,
        help="Resource root containing urdf/hand.urdf with the control joint limits.",
    )
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args = parser.parse_args()
    errors = validate(args.urdf_root, args.manifest)
    if errors:
        print("URDF validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("URDF validation passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
