"""验证结构 Git 来源和原始 URDF 树；不修改结构仓库或输入 XML。"""
from __future__ import annotations

import hashlib
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from desktop.core.model_resources import mesh_paths


def _git(directory: Path, *args: str) -> str:
    result = subprocess.run(["git", "-C", str(directory), *args],
                            capture_output=True, encoding="utf-8", check=False)
    if result.returncode:
        raise ValueError(f"cannot verify structure Git source {directory}: {result.stderr.strip()}")
    return result.stdout.strip()


def inspect_source(urdf: Path) -> tuple[dict, Path, dict[str, Path]]:
    urdf = urdf.resolve()
    if not urdf.is_file():
        raise ValueError(f"missing original URDF: {urdf}")
    repository = Path(_git(urdf.parent, "rev-parse", "--show-toplevel")).resolve()
    relative = urdf.relative_to(repository).as_posix()
    _git(repository, "ls-files", "--error-unmatch", "--", relative)
    if _git(repository, "status", "--porcelain", "--untracked-files=all"):
        raise ValueError(f"structure repository has uncommitted changes: {repository}")
    commit = _git(repository, "rev-parse", "HEAD")
    resources = mesh_paths(urdf)
    files = {urdf, *resources.values()}
    package_files = list(urdf.parent.parent.glob("package.xml"))
    files.update(package_files)
    hashes = {}
    for path in files:
        if not path.is_relative_to(repository):
            raise ValueError(f"resource escapes structure repository: {path}")
        name = path.relative_to(repository).as_posix()
        _git(repository, "ls-files", "--error-unmatch", "--", name)
        # git show returns bytes: compare Git's canonical blob, accounting for checkout CRLF.
        blob = _git(repository, "hash-object", "--path", name, str(path))
        if blob != _git(repository, "rev-parse", f"HEAD:{name}"):
            raise ValueError(f"resource differs from recorded commit: {path}")
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"source_commit": commit, "source_urdf_path": relative,
            "resource_sha256": dict(sorted(hashes.items()))}, repository, resources


def validate_tree(urdf: Path, names: list[str]) -> None:
    root = ET.parse(urdf).getroot()
    if root.tag != "robot":
        raise ValueError(f"URDF root must be robot: {urdf}")
    links = [link.get("name", "") for link in root.findall("link")]
    joints = root.findall("joint")
    joint_names = [joint.get("name", "") for joint in joints]
    for label, values in (("link", links), ("joint", joint_names)):
        if not values or any(not value for value in values) or len(values) != len(set(values)):
            raise ValueError(f"missing or duplicate {label} names: {urdf}")
    missing = set(names) - set(joint_names)
    if missing:
        raise ValueError(f"URDF missing declared control joints: {sorted(missing)}")
    edges = {}
    children = set()
    for joint in joints:
        name = joint.get("name")
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"missing parent/child: {name}")
        a, b = parent.get("link"), child.get("link")
        if a not in links or b not in links or a == b or b in children:
            raise ValueError(f"invalid parent/child tree: {name} {a} -> {b}")
        children.add(b)
        edges.setdefault(a, []).append(b)
        if name in names:
            if joint.get("type") != "revolute" or joint.find("mimic") is not None:
                raise ValueError(f"control joint must be independent revolute: {name}")
            axis = joint.find("axis")
            values = [float(value) for value in axis.get("xyz", "").split()] if axis is not None else []
            if len(values) != 3 or not all(math.isfinite(value) for value in values) or sum(v*v for v in values) == 0:
                raise ValueError(f"invalid control joint axis: {name}")
        for tag, defaults in (("origin", {"xyz": "0 0 0", "rpy": "0 0 0"}),):
            element = joint.find(tag)
            if element is not None:
                for key, default in defaults.items():
                    values = [float(v) for v in element.get(key, default).split()]
                    if len(values) != 3 or not all(math.isfinite(v) for v in values):
                        raise ValueError(f"invalid {key}: {name}")
    roots = set(links) - children
    if len(roots) != 1:
        raise ValueError(f"URDF must have one root: {sorted(roots)}")
    visited = set()
    pending = list(roots)
    while pending:
        node = pending.pop()
        if node in visited:
            raise ValueError(f"cycle in URDF: {node}")
        visited.add(node)
        pending.extend(edges.get(node, []))
    if visited != set(links):
        raise ValueError(f"disconnected or cyclic URDF links: {sorted(set(links)-visited)}")
