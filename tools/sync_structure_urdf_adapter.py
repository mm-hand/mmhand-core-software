#!/usr/bin/env python3
"""Build a desktop-compatible URDF adapter from the structure repository."""

from __future__ import annotations

import argparse
import json
import subprocess
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = PROJECT_ROOT / "desktop/resources/config/robot_model_manifest.json"


def _load_joint_aliases(manifest_path: Path) -> dict[str, str]:
    """从唯一语义清单建立 CAD 名称到运行时名称的转换表。"""
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    joints = data.get("joints")
    if not isinstance(joints, list) or len(joints) != 21:
        raise ValueError("robot model manifest must contain 21 joints")
    aliases: dict[str, str] = {}
    for expected, item in enumerate(joints):
        if not isinstance(item, dict) or int(item.get("joint_index", -1)) != expected:
            raise ValueError(f"invalid manifest joint order at J{expected:02d}")
        source = str(item.get("source_urdf_joint_name", ""))
        target = str(item.get("urdf_joint_name", ""))
        if not source or not target or source in aliases:
            raise ValueError(f"invalid manifest alias at J{expected:02d}")
        aliases[source] = target
    return aliases


def _atomic_write_text(path: Path, content: str) -> None:
    """文件级原子替换，避免更新中断时破坏当前可用 URDF。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
        Path(temp_name).replace(path)
    except Exception:
        Path(temp_name).unlink(missing_ok=True)
        raise


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


def _resolve_source_urdf(source_root: Path) -> Path:
    """选择 structure 导出的 URDF；默认文件缺失时仅接受唯一候选。"""
    urdf_root = source_root / "urdf"
    preferred = urdf_root / "mmhand.urdf"
    if preferred.is_file():
        return preferred

    candidates = sorted(path for path in urdf_root.glob("*.urdf") if path.is_file())
    if len(candidates) != 1:
        names = ", ".join(path.name for path in candidates) or "none"
        raise FileNotFoundError(
            "source URDF mmhand.urdf is missing and the fallback is ambiguous: " + names
        )

    fallback = candidates[0]
    print(f"Warning: mmhand.urdf is missing; using the only URDF candidate: {fallback}")
    return fallback




def build_adapter(source_root: Path, adapter_root: Path,
                  manifest_path: Path = DEFAULT_MANIFEST) -> Path:
    source_root = source_root.resolve()
    source_urdf = _resolve_source_urdf(source_root)
    joint_aliases = _load_joint_aliases(manifest_path)
    # CAD 导出器可能只改变 PIP/DIP 名称大小写；语义匹配不应因此失效。
    folded_aliases = {name.casefold(): target for name, target in joint_aliases.items()}
    if len(folded_aliases) != len(joint_aliases):
        raise ValueError("manifest contains case-insensitive duplicate source joint names")
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    # CAD 临时文件名不能成为运行时模型标识，适配器始终输出稳定名称。
    root.set("name", "mmhand")
    source_names = set()
    output_names = set()
    for joint in root.findall("joint"):
        source_name = joint.get("name", "")
        source_names.add(source_name)
        desktop_name = folded_aliases.get(source_name.casefold())
        if desktop_name:
            if desktop_name in output_names:
                raise ValueError(f"duplicate desktop joint alias: {desktop_name}")
            joint.set("name", desktop_name)
            output_names.add(desktop_name)

    source_names_folded = {name.casefold() for name in source_names}
    missing_source = sorted(
        name for name in joint_aliases if name.casefold() not in source_names_folded
    )
    if missing_source:
        raise ValueError("source URDF is missing expected joints: " + ", ".join(missing_source))

    expected_desktop = set(joint_aliases.values())
    missing_desktop = sorted(expected_desktop - output_names)
    if missing_desktop:
        raise ValueError("adapter did not create required desktop joints: " + ", ".join(missing_desktop))

    adapter_root.mkdir(parents=True, exist_ok=True)
    output_urdf = adapter_root / "urdf" / "hand.urdf"
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    xml_text = ET.tostring(root, encoding="unicode", xml_declaration=True)
    _atomic_write_text(output_urdf, xml_text)

    manifest = {
        "source_root": str(source_root),
        "source_urdf": str(source_urdf),
        "source_commit": _repo_commit(source_root),
        "adapter_urdf": str(output_urdf),
        "joint_aliases": joint_aliases,
    }
    _atomic_write_text(
        adapter_root / "structure_urdf_adapter.json",
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
    )
    return output_urdf


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a desktop-compatible URDF adapter from mm-hand/structure."
    )
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--adapter-root", required=True, type=Path)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args = parser.parse_args()
    output_urdf = build_adapter(args.source_root, args.adapter_root, args.manifest)
    print(f"Adapter URDF created: {output_urdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
