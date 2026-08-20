"""活动 URDF 与 Hand Pose 映射的无 Qt 回归测试。"""

import math

import pytest

from desktop.core.angle_mapping import (
    AngleMappingError,
    JOINT_SPECS,
    calibrated_angles_to_urdf_joint_positions,
)
from desktop.core.protocol import ENCODER_COUNT
from desktop.paths import DEFAULT_URDF_PATH
from desktop.visualize.urdf_kinematics import UrdfKinematics


def test_pose_mapping_uses_all_active_urdf_joint_names() -> None:
    positions = calibrated_angles_to_urdf_joint_positions([0.0] * ENCODER_COUNT)
    kin = UrdfKinematics(str(DEFAULT_URDF_PATH))

    assert len(positions) == ENCODER_COUNT
    assert set(positions) == {
        JOINT_SPECS[index].urdf_joint_name for index in range(ENCODER_COUNT)
    }
    assert set(positions).issubset(kin.joint_by_name())
    assert all(math.isfinite(value) for value in positions.values())


def test_pose_mapping_rejects_incomplete_joint_vector() -> None:
    with pytest.raises(AngleMappingError, match="exactly"):
        calibrated_angles_to_urdf_joint_positions([0.0] * (ENCODER_COUNT - 1))
