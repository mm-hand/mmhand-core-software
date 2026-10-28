"""统一机器人模型生成器的回归测试。"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from tools import generate_robot_model


def test_joint_rebinding_changes_control_contract_but_not_feedback_topology():
    manifest = generate_robot_model.load_manifest(generate_robot_model.DEFAULT_MANIFEST)
    limits = generate_robot_model.load_urdf_limits(generate_robot_model.DEFAULT_URDF, manifest)
    original = generate_robot_model.build_fingerprint(manifest, limits, generate_robot_model.DEFAULT_URDF, "commit")
    rebound = copy.deepcopy(manifest)
    rebound["joints"][2]["motor_indices"], rebound["joints"][3]["motor_indices"] = (
        rebound["joints"][3]["motor_indices"], rebound["joints"][2]["motor_indices"])
    changed = generate_robot_model.build_fingerprint(rebound, limits, generate_robot_model.DEFAULT_URDF, "commit")
    assert changed["control_model_crc"] != original["control_model_crc"]
    assert changed["hardware_map_crc"] == original["hardware_map_crc"]


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
    model = json.loads(outputs[tmp_path / "mapping.json"])
    mapping = model["mapping"]
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
    baseline_mapping = json.loads(baseline[tmp_path / "base/mapping.json"])["mapping"]
    changed_mapping = json.loads(changed[tmp_path / "changed/mapping.json"])["mapping"]
    baseline_fingerprint = json.loads(baseline[tmp_path / "base/fingerprint.json"])
    changed_fingerprint = json.loads(changed[tmp_path / "changed/fingerprint.json"])

    assert baseline_mapping["joints"][0]["urdf_limit_deg"] != changed_mapping["joints"][0]["urdf_limit_deg"]
    assert baseline_fingerprint["control_model_crc"] != changed_fingerprint["control_model_crc"]
    assert baseline_fingerprint["visual_model_crc"] != changed_fingerprint["visual_model_crc"]


def test_manifest_rejects_duplicate_motor_binding(tmp_path: Path) -> None:
    manifest = json.loads(generate_robot_model.DEFAULT_MANIFEST.read_text(encoding="utf-8"))["manifest"]
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


def test_firmware_only_outputs_preserve_desktop_and_check_current_crc(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    args = _args(tmp_path)
    args.firmware_only = True
    outputs = generate_robot_model.build_outputs(args)
    assert set(outputs) == {args.p4_limits_output, args.p4_fingerprint_output}
    args.mapping_output.write_text("desktop mapping retained", encoding="utf-8")
    args.fingerprint_output.write_text("desktop fingerprint retained", encoding="utf-8")
    argv = [
        "generate_robot_model.py", "--firmware-only",
        "--mapping-output", str(args.mapping_output),
        "--fingerprint-output", str(args.fingerprint_output),
        "--p4-limits-output", str(args.p4_limits_output),
        "--p4-fingerprint-output", str(args.p4_fingerprint_output),
    ]
    monkeypatch.setattr(sys, "argv", argv)
    assert generate_robot_model.main() == 0
    assert args.mapping_output.read_text(encoding="utf-8") == "desktop mapping retained"
    assert args.fingerprint_output.read_text(encoding="utf-8") == "desktop fingerprint retained"
    monkeypatch.setattr(sys, "argv", argv + ["--check"])
    assert generate_robot_model.main() == 0
    args.p4_fingerprint_output.write_text("stale source CRC", encoding="utf-8")
    assert generate_robot_model.main() == 1
    # Default checks continue to reject unsynchronized Desktop outputs.
    args.p4_fingerprint_output.write_text(outputs[args.p4_fingerprint_output], encoding="utf-8")
    monkeypatch.setattr(sys, "argv", [item for item in argv if item != "--firmware-only"] + ["--check"])
    assert generate_robot_model.main() == 1


def test_firmware_only_rejects_redirecting_p4_output_into_desktop(tmp_path: Path) -> None:
    args = _args(tmp_path)
    args.firmware_only = True
    args.p4_fingerprint_output = generate_robot_model.DEFAULT_FINGERPRINT
    with pytest.raises(generate_robot_model.ModelGenerationError, match="cannot be inside desktop"):
        generate_robot_model.build_outputs(args)


def test_safety_catalog_changes_update_firmware_fingerprint(tmp_path, monkeypatch):
    # The .def contract is compiled through generated headers, which are excluded
    # from the source hash. Its source must still be included in the fingerprint.
    manifest = generate_robot_model.load_manifest(generate_robot_model.DEFAULT_MANIFEST)
    p4 = tmp_path / 'ESP32-P4'
    catalog = p4 / 'src/config/safety/SafetyRuleCatalog.def'
    catalog.parent.mkdir(parents=True)
    (p4 / 'ESP32-P4.ino').write_text('// fixture')
    config = tmp_path / 'desktop/resources/config'
    config.mkdir(parents=True)
    (config / 'upper_protocol_schema.json').write_text('{}')
    catalog.write_text('// one safety contract')
    monkeypatch.setattr(generate_robot_model, 'ROOT', tmp_path)
    first = generate_robot_model.build_fingerprint(manifest, [], generate_robot_model.DEFAULT_URDF, 'fixture')
    catalog.write_text('// changed safety contract')
    second = generate_robot_model.build_fingerprint(manifest, [], generate_robot_model.DEFAULT_URDF, 'fixture')
    assert first['firmware_source_crc'] != second['firmware_source_crc']
    assert first['control_model_crc'] == second['control_model_crc']
