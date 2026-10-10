"""原始模型来源、资源与拓扑语义回归测试。"""
import copy
import hashlib
import json
import struct
import subprocess
import zlib

import pytest

from tools.generate_robot_model import (
    DEFAULT_MANIFEST, DEFAULT_SOURCE, DEFAULT_URDF, ModelGenerationError,
    load_manifest, load_urdf_limits, topology_crc32, publish_outputs,
)
from tools.robot_model_source import inspect_source, validate_tree


def test_packaged_urdf_is_original_bytes_with_committed_source_identity():
    source = json.loads(DEFAULT_SOURCE.read_text(encoding="utf-8"))
    assert len(source["source_commit"]) == 40
    assert source["source_urdf_path"].endswith("mmhand.urdf")
    assert hashlib.sha256(DEFAULT_URDF.read_bytes()).hexdigest() == source["resource_sha256"][source["source_urdf_path"]]
    manifest = load_manifest(DEFAULT_MANIFEST)
    validate_tree(DEFAULT_URDF, [item["urdf_joint_name"] for item in manifest["joints"]])
    assert all("source_urdf_joint_name" not in joint for joint in manifest["joints"])
    assert all("calibrated_default_range_deg" not in joint for joint in manifest["joints"])


def test_topology_uses_fixed_binary_encoding_and_ignores_names_and_joint_binding():
    manifest = load_manifest(DEFAULT_MANIFEST)
    motors = manifest["motors"]
    expected_bytes = b"\x02\x1b" + b"".join(struct.pack(">BBHb", i, i, 4096, 1) for i in range(27))
    assert topology_crc32(motors) == zlib.crc32(expected_bytes) & 0xFFFFFFFF
    changed = copy.deepcopy(manifest)
    changed["joints"][0]["urdf_joint_name"] = "renamed"
    changed["joints"][0]["motor_indices"].reverse()
    assert topology_crc32(changed["motors"]) == topology_crc32(motors)
    changed["motors"][0]["feedback_abs_sign"] = -1
    assert topology_crc32(changed["motors"]) != topology_crc32(motors)
    assert topology_crc32(list(reversed(motors))) == topology_crc32(motors)


@pytest.mark.parametrize("replacement", [
    '<joint name="j" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 0"/></joint>',
    '<joint name="j" type="revolute"><parent link="base"/><child link="missing"/><axis xyz="0 0 1"/></joint>',
    '<joint name="j" type="revolute"><parent link="tip"/><child link="tip"/><axis xyz="0 0 1"/></joint>',
])
def test_invalid_axis_or_tree_is_rejected(tmp_path, replacement):
    urdf = tmp_path / "bad.urdf"
    urdf.write_text('<robot name="hand"><link name="base"/><link name="tip"/>' + replacement + '</robot>', encoding="utf-8")
    with pytest.raises(ValueError):
        validate_tree(urdf, ["j"])


def test_git_source_rejects_dirty_and_untracked_inputs(tmp_path):
    repository = tmp_path / "structure"
    repository.mkdir()
    def git(*args):
        return subprocess.run(["git", "-C", str(repository), *args], check=True,
                              capture_output=True, text=True)
    git("init")
    git("config", "user.name", "Model test")
    git("config", "user.email", "model-test@example.invalid")
    urdf = repository / "hand.urdf"
    urdf.write_text('<robot name="hand"><link name="base"/></robot>', encoding="utf-8")
    git("add", "hand.urdf")
    git("commit", "-m", "test fixture")
    source, root, resources = inspect_source(urdf)
    assert root == repository.resolve() and resources == {}
    assert source["source_commit"] == git("rev-parse", "HEAD").stdout.strip()
    urdf.write_text('<robot name="changed"><link name="base"/></robot>', encoding="utf-8")
    with pytest.raises(ValueError, match="uncommitted"):
        inspect_source(urdf)


def test_failed_publication_restores_previous_output_set(tmp_path, monkeypatch):
    import tools.generate_robot_model as generator
    first, second = tmp_path / "first", tmp_path / "second"
    first.write_bytes(b"old first")
    second.write_bytes(b"old second")
    replace = generator.os.replace
    def fail_second(source, target):
        if target == second:
            raise OSError("simulated disk failure")
        return replace(source, target)
    monkeypatch.setattr(generator.os, "replace", fail_second)
    with pytest.raises(OSError, match="disk failure"):
        publish_outputs({first: b"new first", second: b"new second"})
    assert first.read_bytes() == b"old first"
    assert second.read_bytes() == b"old second"
    assert not list(tmp_path.glob("*.tmp"))
