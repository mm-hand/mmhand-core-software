import json
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from desktop.core import angle_mapping
from desktop.core.calibration_snapshot import seal_snapshot
from desktop.core.protocol import ENCODER_COUNT, MOTOR_COUNT
from desktop.paths import DEFAULT_URDF_PATH, URDF_JOINT_MAPPING_PATH
from desktop.teleop import manus_calibration
from desktop.teleop import manus_raw_skeleton_retarget


def _active_urdf_limits_deg() -> dict[str, tuple[float, float]]:
    root = ET.parse(DEFAULT_URDF_PATH).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        limits[str(joint.get("name"))] = (
            math.degrees(float(limit.attrib["lower"])),
            math.degrees(float(limit.attrib["upper"])),
        )
    return limits


def test_all_21_mapping_limits_match_active_urdf():
    urdf_limits = _active_urdf_limits_deg()
    assert len(angle_mapping.JOINT_SPECS) == 21
    for idx, spec in angle_mapping.JOINT_SPECS.items():
        assert spec.mapping_status == "unverified"
        assert spec.urdf_joint_name in urdf_limits
        actual = urdf_limits[spec.urdf_joint_name]
        assert spec.urdf_range_deg == pytest.approx(actual, abs=1e-4)


def test_feedback_mapping_declares_runtime_v2_and_staged_v3():
    data = json.loads(URDF_JOINT_MAPPING_PATH.read_text(encoding="utf-8"))
    assert data["schema_version"] == 4
    assert data["feedback_mapping_version"] == 2
    assert data["manual_scaled_mapping_version"] == 3
    assert data["p4_sync_required"] is False
    assert data["p4_generated_header_required"] is True

    entries = data["joints"]
    assert len(entries) == 21
    for entry in entries:
        lower, upper = entry["urdf_limit_deg"]
        assert math.isfinite(float(lower))
        assert math.isfinite(float(upper))
        assert float(lower) < float(upper)


def test_generated_p4_urdf_limits_are_current_and_crc_matches():
    root = Path(__file__).resolve().parents[3]
    subprocess.run(
        [sys.executable, str(root / "tools" / "generate_robot_model.py"), "--check"],
        cwd=root,
        check=True,
    )
    version, crc = angle_mapping.urdf_feedback_mapping_metadata()
    header = (
        root
        / "ESP32-P4"
        / "ServoBoardMain"
        / "src"
        / "generated"
        / "UrdfJointLimits.generated.h"
    ).read_text(encoding="utf-8")
    assert f"kUrdfFeedbackMappingVersion = {version}" in header
    match = re.search(r"kUrdfJointLimitConfigCrc = 0x([0-9A-F]{8})UL", header)
    assert match is not None
    assert int(match.group(1), 16) == crc

    manual_version, manual_crc = angle_mapping.urdf_manual_scaled_mapping_metadata()
    assert f"kManualScaledUrdfFeedbackMappingVersion = {manual_version}" in header
    manual_match = re.search(r"kManualScaledUrdfConfigCrc = 0x([0-9A-F]{8})UL", header)
    assert manual_match is not None
    assert int(manual_match.group(1), 16) == manual_crc


def test_endpoint_mapping_round_trip_without_calib_snapshot(monkeypatch, tmp_path):
    monkeypatch.setattr(angle_mapping, "CALIB_LOAD_SNAPSHOT_PATH", tmp_path / "missing_snapshot.json")
    monkeypatch.setattr(angle_mapping, "_CALIB_CACHE", None)

    for idx, spec in angle_mapping.JOINT_SPECS.items():
        calib_min, calib_max = spec.default_calib_range_deg
        urdf_min, urdf_max = spec.calibrated_urdf_endpoints_deg

        assert angle_mapping.urdf_to_calibrated(idx, math.radians(urdf_min)) == pytest.approx(calib_min)
        assert angle_mapping.urdf_to_calibrated(idx, math.radians(urdf_max)) == pytest.approx(calib_max)
        assert math.degrees(angle_mapping.calibrated_to_urdf(idx, calib_min)) == pytest.approx(urdf_min)
        assert math.degrees(angle_mapping.calibrated_to_urdf(idx, calib_max)) == pytest.approx(urdf_max)


def test_bad_direction_negative_v3_endpoints_are_not_loaded(tmp_path):
    version, crc = angle_mapping.urdf_manual_scaled_mapping_metadata()
    results = [{} for _ in range(21)]
    results[14] = {
        "success": True,
        "mappingVersion": version,
        "urdfConfigCrc": crc,
        "encoderDirection": -1,
        "startEncoder": 6398,
        "tightEncoder": 10598,
        "autoMinRaw": 6398,
        "autoMaxRaw": 10598,
        "angleMin": -192.258,
        "angleMax": 88.0,
    }
    snapshot = tmp_path / "bad_snapshot.json"
    def write_current_snapshot():
        payload = seal_snapshot({
            "format": "mmhand_calibration_bundle",
            "schema_version": 5,
            "active_joint_mask": 1 << 14,
            "active_motor_mask": 1 << 18,
            "motor_abs_limit_valid_bitmap": 1 << 18,
            "motor_abs_limit_locked_bitmap": 0,
            "motor_abs_limit_min": [0] * MOTOR_COUNT,
            "motor_abs_limit_max": [0] * MOTOR_COUNT,
            "manual_reference_valid_bitmap": (1 << ENCODER_COUNT) - 1,
            "manual_reference_min_raw": [0] * ENCODER_COUNT,
            "manual_reference_max_raw": [1] * ENCODER_COUNT,
            "joint_calib_results": results,
        })
        snapshot.write_text(json.dumps(payload), encoding="utf-8")

    write_current_snapshot()

    assert 14 not in angle_mapping.load_calibrated_ranges(snapshot)

    results[14]["autoMinRaw"] = 9986
    results[14]["autoMaxRaw"] = 5786
    results[14]["angleMin"] = -8.609
    write_current_snapshot()
    assert angle_mapping.load_calibrated_ranges(snapshot)[14] == pytest.approx(
        (-8.609, 88.0)
    )


def test_manus_modules_use_central_command_ranges():
    for idx in range(21):
        expected = angle_mapping.default_calibrated_range_deg(idx)
        assert manus_calibration.ROBOT_JOINT_LIMITS_DEG[idx] == expected
        assert manus_raw_skeleton_retarget._raw_encoder_limit_deg(idx) == (
            min(angle_mapping.effective_calibrated_range_deg(idx)),
            max(angle_mapping.effective_calibrated_range_deg(idx)),
        )
