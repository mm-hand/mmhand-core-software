import math

import pytest

from desktop.core import angle_mapping
from desktop.core.joint_target_adapter import (
    URDF_DEG,
    URDF_RAD,
    adapt_joint_targets,
    joint_index_for_urdf_name,
    named_urdf_radians_to_joint_targets,
)
from desktop.core.protocol import ENCODER_COUNT


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


def test_legacy_target_space_is_rejected() -> None:
    with pytest.raises(ValueError, match="unsupported joint target space"):
        adapt_joint_targets(
            [0.0] * ENCODER_COUNT,
            source_space="legacy_calibrated_deg",
        )


def test_semantic_thumb_names_use_canonical_j16_to_j20_order() -> None:
    assert joint_index_for_urdf_name("Thumb_MCP_AA") == 16
    assert joint_index_for_urdf_name("Thumb_CMC") == 20
    assert joint_index_for_urdf_name("thumb_aa_joint") is None
    assert joint_index_for_urdf_name("5-1") is None


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
