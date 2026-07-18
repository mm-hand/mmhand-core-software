import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from wuji_mmhand_teleop.retarget.wuji_calibration import WujiCalibrationManager
from wuji_mmhand_teleop.sources.wuji_fingertip_cp import (
    TIP_SEMANTICS,
    active_fingertip_contact_transforms,
    apply_fingertip_contact_points,
    load_fingertip_contact_transforms,
)


FINGER_LINKS = {
    "thumb": ("thumb_tip", "thumb_fingertip_cp"),
    "index": ("index_finger_tip", "index_fingertip_cp"),
    "middle": ("middle_finger_tip", "middle_fingertip_cp"),
    "ring": ("ring_finger_tip", "ring_fingertip_cp"),
    "pinky": ("pinky_tip", "pinky_fingertip_cp"),
}


def write_test_urdf(path: Path, *, missing: str = "") -> None:
    joints = []
    for finger, (tip_link, contact_link) in FINGER_LINKS.items():
        if finger == missing:
            continue
        joints.append(
            f'<joint name="{contact_link}_fixed" type="fixed">'
            f'<parent link="{tip_link}"/><child link="{contact_link}"/>'
            f'<origin xyz="0 -0.002 -0.002" rpy="0 0 0"/></joint>'
        )
    path.write_text("<robot name=\"test\">" + "".join(joints) + "</robot>")


def make_skeleton(rotation):
    node = {
        "pos": [1.0, 2.0, 3.0],
        "rot": list(rotation),
        "confidence": 0.75,
    }
    return {finger: {"tip": dict(node)} for finger in FINGER_LINKS}


class FakeGlove:
    def __init__(self, path):
        self.path = str(path)
        self.requests = []

    def get(self, path):
        self.requests.append(path)
        if path == "calibration.hand_profile":
            return "wujihand"
        if path == "calibration.hand_model_paths.wujihand":
            return self.path
        raise KeyError(path)


class WujiFingertipContactPointTest(unittest.TestCase):
    def test_loads_all_five_fixed_transforms(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand.urdf"
            write_test_urdf(path)
            transforms = load_fingertip_contact_transforms(path)
        self.assertEqual(set(transforms), set(FINGER_LINKS))
        for transform in transforms.values():
            np.testing.assert_allclose(
                transform.translation, [0.0, -0.002, -0.002])
            np.testing.assert_allclose(transform.rotation, np.eye(3))

    def test_active_profile_selects_active_model(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand.urdf"
            write_test_urdf(path)
            glove = FakeGlove(path)
            profile, selected, transforms = active_fingertip_contact_transforms(
                glove)
        self.assertEqual(profile, "wujihand")
        self.assertEqual(selected, path)
        self.assertEqual(len(transforms), 5)
        self.assertEqual(
            glove.requests,
            ["calibration.hand_profile",
             "calibration.hand_model_paths.wujihand"],
        )

    def test_preserves_raw_tip_and_corrects_current_tip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand.urdf"
            write_test_urdf(path)
            transforms = load_fingertip_contact_transforms(path)
        skeleton = make_skeleton([1.0, 0.0, 0.0, 0.0])
        apply_fingertip_contact_points(skeleton, transforms)
        for finger in FINGER_LINKS:
            self.assertEqual(
                skeleton[finger]["raw_tip"]["pos"], [1.0, 2.0, 3.0])
            np.testing.assert_allclose(
                skeleton[finger]["tip"]["pos"],
                [1.0, 1.998, 2.998],
            )
            self.assertEqual(skeleton[finger]["tip"]["confidence"], 0.75)

    def test_offset_rotates_in_tip_local_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand.urdf"
            write_test_urdf(path)
            transforms = load_fingertip_contact_transforms(path)
        half = math.sqrt(0.5)
        skeleton = make_skeleton([half, 0.0, 0.0, half])
        apply_fingertip_contact_points(skeleton, transforms)
        np.testing.assert_allclose(
            skeleton["index"]["tip"]["pos"],
            [1.002, 2.0, 2.998],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            skeleton["index"]["tip"]["rot"],
            [half, 0.0, 0.0, half],
            atol=1e-12,
        )

    def test_missing_contact_joint_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand.urdf"
            write_test_urdf(path, missing="ring")
            with self.assertRaisesRegex(ValueError, "ring_fingertip_cp_fixed"):
                load_fingertip_contact_transforms(path)


class WujiCalibrationSemanticsTest(unittest.TestCase):
    def test_legacy_file_invalidates_tip_geometry_and_robot_pose_references(self):
        payload = {
            "version": 1,
            "poses": {
                "align": {"wuji_f0_q0_rad": 0.1},
                "open_aa": {"wuji_f0_q0_rad": 0.2},
                "fist_flex": {"wuji_f0_q0_rad": 0.3},
                "thumb_pinky_root": {"wuji_f0_q0_rad": 0.4},
                "raw_open_palm_width": {"value": 0.08},
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            path.write_text(json.dumps(payload))
            manager = WujiCalibrationManager(path=path)
        self.assertTrue(manager.requires_align_recalibration)
        self.assertTrue(manager.requires_model_recalibration)
        self.assertEqual(manager.calibration["align"], {})
        self.assertEqual(manager.calibration["open_aa"], {})
        self.assertEqual(manager.calibration["fist_flex"], {})
        self.assertEqual(manager.calibration["thumb_pinky_root"], {})
        self.assertEqual(manager.calibration["raw_open_palm_width"], {})

    def test_saved_file_records_semantics_and_model(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            manager = WujiCalibrationManager(path=path)
            manager.set_hand_model_path("/tmp/current-hand.urdf")
            manager.save()
            saved = json.loads(path.read_text())
        self.assertEqual(saved["tip_semantics"], TIP_SEMANTICS)
        self.assertEqual(saved["hand_model_path"], "/tmp/current-hand.urdf")


if __name__ == "__main__":
    unittest.main()
