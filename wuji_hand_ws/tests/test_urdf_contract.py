import math
import json
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = ROOT / "src" / "wuji_mmhand_teleop"
sys.path.insert(0, str(PACKAGE_SOURCE))

from wuji_mmhand_teleop.retarget.hand_retargeting.urdf_model import URDFRobotHandModel
from wuji_mmhand_teleop.retarget.manus_raw_skeleton import (
    _encoder_degrees_to_urdf_angle,
    _urdf_angle_to_encoder_degrees,
)
from wuji_mmhand_teleop.retarget.robot_joint_map import build_encoder_target_limits_deg
from wuji_mmhand_teleop.retarget.wuji_calibration import POSES, WujiCalibrationManager
from wuji_mmhand_teleop.urdf_contract import (
    JOINT_COUNT,
    JOINT_LIMITS_DEG,
    JOINT_NAMES,
    MESSAGE_LAYOUT_LABEL,
    URDF_PATH,
    validate_bundled_urdf,
    validate_target,
)


def test_bundled_urdf_matches_contract():
    validate_bundled_urdf()
    assert JOINT_COUNT == 21
    assert len(set(JOINT_NAMES)) == 21


def test_urdf_mesh_references_exist():
    text = URDF_PATH.read_text(encoding="utf-8")
    import re

    mesh_paths = re.findall(r'<mesh\s+filename="([^"]+)"', text)
    assert mesh_paths
    for mesh_path in mesh_paths:
        if mesh_path.startswith("package://hku_hand_v2_urdf/"):
            candidate = URDF_PATH.parent.parent / mesh_path.replace(
                "package://hku_hand_v2_urdf/", ""
            )
        else:
            candidate = URDF_PATH.parent / mesh_path
        assert candidate.resolve().is_file(), mesh_path


def test_direct_urdf_limits_and_round_trip():
    robot = URDFRobotHandModel.from_urdf(URDF_PATH)
    limits = build_encoder_target_limits_deg(robot)
    for index, expected in enumerate(JOINT_LIMITS_DEG):
        assert all(math.isclose(a, b, abs_tol=0.02) for a, b in zip(limits[index], expected))
        for value in (expected[0], 0.5 * (expected[0] + expected[1]), expected[1]):
            radians = _encoder_degrees_to_urdf_angle(index, value, limits)
            restored = _urdf_angle_to_encoder_degrees(index, radians, limits)
            assert math.isclose(restored, value, abs_tol=0.02)


def test_target_contract_validation():
    assert validate_target([0.0] * JOINT_COUNT, MESSAGE_LAYOUT_LABEL) is None
    assert "expected 21" in validate_target([0.0] * 20, MESSAGE_LAYOUT_LABEL)
    assert "non-finite" in validate_target([float("nan")] * JOINT_COUNT, MESSAGE_LAYOUT_LABEL)
    assert "mismatch" in validate_target([0.0] * JOINT_COUNT, "old-model")


def test_saved_calibration_is_bound_to_current_robot_model():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "wuji_calibration.json"
        manager = WujiCalibrationManager(path=path)
        for pose in POSES:
            manager.calibration[pose] = {"sample": 1.0}
        manager.save()
        payload = json.loads(path.read_text(encoding="utf-8"))
        assert payload["robot_model_id"] == "structure_urdf_v2:v3:73FD45FA"

        restored = WujiCalibrationManager(path=path)
        assert not restored.requires_model_recalibration
        assert all(restored.calibration[pose] for pose in POSES)
