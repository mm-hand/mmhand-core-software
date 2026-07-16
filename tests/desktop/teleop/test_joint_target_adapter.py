import math

import pytest

from desktop.core import angle_mapping
from desktop.core.protocol import ENCODER_COUNT
from desktop.teleop.joint_target_adapter import (
    LEGACY_CALIBRATED_DEG,
    URDF_DEG,
    URDF_RAD,
    adapt_joint_targets,
    joint_index_for_urdf_name,
    named_urdf_radians_to_joint_targets,
)


def test_active_urdf_degrees_are_identity() -> None:
    values = [float(idx) for idx in range(ENCODER_COUNT)]
    result = adapt_joint_targets(values, source_space=URDF_DEG)
    assert list(result.urdf_targets_deg) == values


def test_active_urdf_radians_convert_to_degrees() -> None:
    values = [math.radians(float(idx)) for idx in range(ENCODER_COUNT)]
    result = adapt_joint_targets(values, source_space=URDF_RAD)
    assert list(result.urdf_targets_deg) == pytest.approx(
        [float(idx) for idx in range(ENCODER_COUNT)]
    )


def test_legacy_calibrated_endpoints_map_to_active_urdf_endpoints() -> None:
    legacy_min = []
    legacy_max = []
    expected_min = []
    expected_max = []
    for idx in range(ENCODER_COUNT):
        old_low, old_high = angle_mapping.default_calibrated_range_deg(idx)
        spec = angle_mapping.get_spec(idx)
        new_low, new_high = spec.calibrated_urdf_endpoints_deg
        legacy_min.append(old_low)
        legacy_max.append(old_high)
        expected_min.append(new_low)
        expected_max.append(new_high)

    mapped_min = adapt_joint_targets(legacy_min, source_space=LEGACY_CALIBRATED_DEG)
    mapped_max = adapt_joint_targets(legacy_max, source_space=LEGACY_CALIBRATED_DEG)
    assert list(mapped_min.urdf_targets_deg) == pytest.approx(expected_min)
    assert list(mapped_max.urdf_targets_deg) == pytest.approx(expected_max)


def test_semantic_and_legacy_thumb_names_use_canonical_j16_to_j20_order() -> None:
    assert joint_index_for_urdf_name("thumb_aa_joint") == 16
    assert joint_index_for_urdf_name("thumb_cmc1_joint") == 20
    assert joint_index_for_urdf_name("5-1") == 20
    assert joint_index_for_urdf_name("5-2") == 16
    assert joint_index_for_urdf_name("5-5") == 19


def test_named_urdf_radians_map_all_21_semantic_names() -> None:
    source = {
        spec.urdf_joint_name: math.radians(float(idx))
        for idx, spec in angle_mapping.JOINT_SPECS.items()
    }
    result = named_urdf_radians_to_joint_targets(source)
    assert result.mapped_indices == tuple(range(ENCODER_COUNT))
    assert result.unknown_names == ()
    assert list(result.urdf_targets_deg) == pytest.approx(
        [float(idx) for idx in range(ENCODER_COUNT)]
    )


@pytest.mark.parametrize(
    "values",
    ([0.0] * (ENCODER_COUNT - 1), [math.nan] * ENCODER_COUNT),
)
def test_adapter_rejects_invalid_frames(values) -> None:
    with pytest.raises(ValueError):
        adapt_joint_targets(values, source_space=URDF_DEG)
