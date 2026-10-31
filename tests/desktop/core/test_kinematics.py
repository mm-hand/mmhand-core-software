"""V5 接触标定 FK 的最小回归：解析、批量一致性、几何修正合成。"""

import math

import numpy as np

from desktop.core.kinematics import (
    Kinematics,
    SENSOR_LINKS,
    decompose_origin,
)
from desktop.paths import DEFAULT_URDF_PATH


def _model():
    return Kinematics(DEFAULT_URDF_PATH)


def test_urdf_tree_has_21_joints_and_15_sensor_links():
    model = _model()
    assert len(model.names) == 21
    assert len(SENSOR_LINKS) == 15
    assert set(SENSOR_LINKS.values()) <= set(model.link_names)


def test_batch_fk_matches_single_frame_and_is_finite():
    model = _model()
    q0 = np.zeros(21)
    q1 = np.linspace(-5.0, 20.0, 21)
    batch = model.sensor_transforms(np.stack([q0, q1]))
    assert batch.shape == (2, 15, 4, 4)
    assert np.isfinite(batch).all()
    single0 = model.sensor_transforms(q0)[0]
    single1 = model.sensor_transforms(q1)[0]
    assert np.allclose(batch[0], single0)
    assert np.allclose(batch[1], single1)


def test_composed_origin_decomposes_back_to_urdf_xyz_rpy():
    model = _model()
    joint = model.by_name["index_aa_joint"]
    xyz, rpy = decompose_origin(joint["origin"])
    assert all(math.isfinite(v) for v in xyz + rpy)
    assert any(abs(v) > 1e-9 for v in xyz + rpy)


def test_geometric_correction_changes_pose_and_round_trips():
    model = _model()
    q = np.zeros(21)
    base = model.sensor_transforms(q)[0]
    corrected = model.correction_origin(
        "index_aa_joint", (0.001, -0.0005, 0.0002), 0.004, -0.003)
    posed = model.sensor_transforms(q, {"index_aa_joint": corrected})[0]
    assert not np.allclose(base, posed)
    xyz, rpy = decompose_origin(corrected)
    assert all(math.isfinite(v) for v in xyz + rpy)
