#!/usr/bin/env python3
"""Build a desktop-compatible URDF adapter from the structure repository."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from desktop.core.angle_mapping import JOINT_SPECS


# The structure repository uses CAD-oriented names. The desktop pipeline uses
# these legacy semantic names in angle mapping, retargeting, and calibration.
SOURCE_TO_DESKTOP_JOINT = {
    "Little_MCP_AA": "little_aa_joint",
    "Little_MCP_FE": "mmhand_finger_4_proximal_phalanx_1_mcp_fe_joint",
    "finger_4_distal_phalanx_1_pip_joint": "mmhand_finger_4_distal_phalanx_1_pip_joint",
    "finger_4_fingertip_1_dip_joint": "mmhand_finger_4_fingertip_1_dip_joint",
    "Ring_MCP_AA": "ring_aa_joint",
    "Ring_MCP_FE": "mmhand_finger_3_proximal_phalanx_1_mcp_fe_joint",
    "finger_3_distal_phalanx_1_pip_joint": "mmhand_finger_3_distal_phalanx_1_pip_joint",
    "finger_3_fingertip_1_dip_joint": "mmhand_finger_3_fingertip_1_dip_joint",
    "Middle_MCP_AA": "middle_aa_joint",
    "Middle_MCP_FE": "mmhand_finger_2_proximal_phalanx_1_mcp_fe_joint",
    "finger_2_distal_phalanx_1_pip_joint": "mmhand_finger_2_distal_phalanx_1_pip_joint",
    "finger_2_fingertip_1_dip_joint": "mmhand_finger_2_fingertip_1_dip_joint",
    "Index_MCP_AA": "index_aa_joint",
    "Index_MCP_FE": "mmhand_finger_1_proximal_phalanx_1_mcp_fe_joint",
    "finger_1_distal_phalanx_1_pip_joint": "mmhand_finger_1_distal_phalanx_1_pip_joint",
    "finger_1_fingertip_1_dip_joint": "mmhand_finger_1_fingertip_1_dip_joint",
    "Thumb_MCP_AA": "thumb_aa_joint",
    "Thumb_MCP_FE": "mmhand_thumb_1_mmhand_finger_5_proximal_phalanx_1_mcp_fe_joint",
    "mmhand_thumb_1_finger_7_distal_phalanx_1_pip_joint": "mmhand_thumb_1_mmhand_finger_5_distal_phalanx_1_pip_joint",
    "mmhand_thumb_1_finger_7_fingertip_1_dip_joint": "mmhand_thumb_1_mmhand_finger_5_fingertip_1_dip_joint",
    "Thumb_CMC": "thumb_cmc1_joint",
}


def _repo_commit(source_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(source_root.parent), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def _ensure_mesh_junction(adapter_root: Path, source_meshes: Path) -> None:
    target = adapter_root / "meshes"
    if target.exists() or target.is_symlink():
        if target.resolve() == source_meshes.resolve():
            return
        raise RuntimeError(f"adapter mesh path already exists with another target: {target}")

    target_text = str(target).replace("'", "''")
    source_text = str(source_meshes).replace("'", "''")
    command = (
        "$ErrorActionPreference='Stop'; "
        f"New-Item -ItemType Junction -Path '{target_text}' -Target '{source_text}' | Out-Null"
    )
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"cannot create mesh Junction: {result.stderr or result.stdout}")


def build_adapter(source_root: Path, adapter_root: Path) -> Path:
    source_root = source_root.resolve()
    source_urdf = source_root / "urdf" / "mmhand.urdf"
    source_meshes = source_root / "meshes"
    if not source_urdf.is_file() or not source_meshes.is_dir():
        raise FileNotFoundError(
            "source root must contain urdf/mmhand.urdf and meshes/: " + str(source_root)
        )

    tree = ET.parse(source_urdf)
    root = tree.getroot()
    source_names = set()
    output_names = set()
    for joint in root.findall("joint"):
        source_name = joint.get("name", "")
        source_names.add(source_name)
        desktop_name = SOURCE_TO_DESKTOP_JOINT.get(source_name)
        if desktop_name:
            if desktop_name in output_names:
                raise ValueError(f"duplicate desktop joint alias: {desktop_name}")
            joint.set("name", desktop_name)
            output_names.add(desktop_name)

    missing_source = sorted(set(SOURCE_TO_DESKTOP_JOINT) - source_names)
    if missing_source:
        raise ValueError("source URDF is missing expected joints: " + ", ".join(missing_source))

    expected_desktop = {spec.urdf_joint_name for spec in JOINT_SPECS.values()}
    missing_desktop = sorted(expected_desktop - output_names)
    if missing_desktop:
        raise ValueError("adapter did not create required desktop joints: " + ", ".join(missing_desktop))

    adapter_root.mkdir(parents=True, exist_ok=True)
    _ensure_mesh_junction(adapter_root, source_meshes)
    output_urdf = adapter_root / "urdf" / "hand.urdf"
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)

    manifest = {
        "source_root": str(source_root),
        "source_urdf": str(source_urdf),
        "source_commit": _repo_commit(source_root),
        "adapter_urdf": str(output_urdf),
        "joint_aliases": SOURCE_TO_DESKTOP_JOINT,
    }
    (adapter_root / "structure_urdf_adapter.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return output_urdf


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a desktop-compatible URDF adapter from mm-hand/structure."
    )
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--adapter-root", required=True, type=Path)
    args = parser.parse_args()
    output_urdf = build_adapter(args.source_root, args.adapter_root)
    print(f"Adapter URDF created: {output_urdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
