#!/usr/bin/env python3
"""Validate an external mmhand URDF resource root before linking it to desktop."""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from desktop.core.angle_mapping import JOINT_SPECS


def _resolve_mesh_path(resource_root: Path, urdf_path: Path, filename: str) -> Path | None:
    if filename.startswith("file://"):
        return Path(filename.removeprefix("file://"))
    if filename.startswith("package://"):
        return None
    return (urdf_path.parent / filename).resolve()


def validate(resource_root: Path) -> list[str]:
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

    mesh_count = 0
    for mesh in root.findall(".//mesh"):
        mesh_count += 1
        filename = (mesh.get("filename") or mesh.get("file") or "").strip()
        if not filename:
            errors.append("mesh entry has no filename")
            continue
        resolved = _resolve_mesh_path(resource_root, urdf_path, filename)
        if resolved is None:
            errors.append(f"unsupported package:// mesh URI: {filename}")
        elif not resolved.is_file():
            errors.append(f"missing mesh: {filename} -> {resolved}")

    joints = {joint.get("name", ""): joint for joint in root.findall("joint")}
    required_specs = {spec.urdf_joint_name: spec for spec in JOINT_SPECS.values()}
    required_joint_names = set(required_specs)
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
            expected = required_specs[name]
            if abs(lower - expected.urdf_min_rad) > 1e-6 or abs(upper - expected.urdf_max_rad) > 1e-6:
                errors.append(
                    "joint limit differs from urdf_joint_mapping_v2.json: "
                    f"{name} actual=({lower:.8f},{upper:.8f}) "
                    f"mapped=({expected.urdf_min_rad:.8f},{expected.urdf_max_rad:.8f})"
                )

    print(f"URDF: {urdf_path}")
    print(f"Robot: {root.get('name', '<unnamed>')}")
    print(f"Meshes: {mesh_count}; joints: {len(joints)}; required mapping joints: {len(required_joint_names)}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate an external URDF root before mmhand desktop uses it."
    )
    parser.add_argument(
        "--urdf-root",
        required=True,
        type=Path,
        help="Resource root containing urdf/hand.urdf and its mesh tree.",
    )
    args = parser.parse_args()
    errors = validate(args.urdf_root)
    if errors:
        print("URDF validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("URDF validation passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
