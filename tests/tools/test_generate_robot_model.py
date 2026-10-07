"""统一机器人模型生成器的回归测试。"""

from __future__ import annotations

import argparse
import copy
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from tools import generate_robot_model


def _args(tmp_path: Path, *, urdf: Path | None = None) -> argparse.Namespace:
    """把所有输出隔离到临时目录，测试不会覆盖仓库生成物。"""
    return argparse.Namespace(
        manifest=generate_robot_model.DEFAULT_MANIFEST,
        urdf=urdf or generate_robot_model.DEFAULT_URDF,
        mapping_output=tmp_path / "mapping.json",
        fingerprint_output=tmp_path / "fingerprint.json",
        p4_limits_output=tmp_path / "limits.h",
        p4_fingerprint_output=tmp_path / "fingerprint.h",
        p4_bindings_output=tmp_path / "bindings.h",
        check=False,
    )


def test_generator_outputs_desktop_and_p4_contracts(tmp_path: Path) -> None:
    outputs = generate_robot_model.build_outputs(_args(tmp_path))
    assert len(outputs) == 5
    mapping = json.loads(outputs[tmp_path / "mapping.json"])
    fingerprint = json.loads(outputs[tmp_path / "fingerprint.json"])

    assert mapping["schema_version"] == 4
    assert [item["joint_index"] for item in mapping["joints"]] == list(range(21))
    assert len(fingerprint["control_model_crc"]) == 8
    assert len(fingerprint["firmware_source_crc"]) == 8


def test_urdf_limit_change_updates_every_control_contract(tmp_path: Path) -> None:
    tree = ET.parse(generate_robot_model.DEFAULT_URDF)
    root = tree.getroot()
    target = next(joint for joint in root.findall("joint") if joint.get("name") == "Little_MCP_AA")
    target.find("limit").set("upper", "1.000000")
    changed_urdf = tmp_path / "hand.urdf"
    tree.write(changed_urdf, encoding="utf-8", xml_declaration=True)

    baseline = generate_robot_model.build_outputs(_args(tmp_path / "base"))
    changed = generate_robot_model.build_outputs(_args(tmp_path / "changed", urdf=changed_urdf))
    baseline_mapping = json.loads(baseline[tmp_path / "base/mapping.json"])
    changed_mapping = json.loads(changed[tmp_path / "changed/mapping.json"])
    baseline_fingerprint = json.loads(baseline[tmp_path / "base/fingerprint.json"])
    changed_fingerprint = json.loads(changed[tmp_path / "changed/fingerprint.json"])

    assert baseline_mapping["joints"][0]["urdf_limit_deg"] != changed_mapping["joints"][0]["urdf_limit_deg"]
    assert baseline_fingerprint["control_model_crc"] != changed_fingerprint["control_model_crc"]
    assert baseline_fingerprint["visual_model_crc"] != changed_fingerprint["visual_model_crc"]


def test_manifest_rejects_duplicate_motor_binding(tmp_path: Path) -> None:
    manifest = json.loads(generate_robot_model.DEFAULT_MANIFEST.read_text(encoding="utf-8"))
    invalid = copy.deepcopy(manifest)
    invalid["joints"][1]["motor_indices"] = [0]
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(invalid), encoding="utf-8")

    try:
        generate_robot_model.load_manifest(path)
    except generate_robot_model.ModelGenerationError as exc:
        assert "duplicate" in str(exc)
    else:
        raise AssertionError("duplicate motor binding was accepted")
