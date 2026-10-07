"""上位机多圈运行锚点的原子存储、身份和环形 raw 校验测试。"""

from __future__ import annotations

import json

import pytest

from desktop.core.device_state_store import (
    DeviceStateStoreError,
    MultiTurnRuntimeStore,
    circular_raw_distance,
    nearest_logical_abs,
    multiturn_identity_compatible,
)
from desktop.core.protocol import MOTOR_COUNT


def _identity() -> dict:
    return {
        "device_id": "0000123456789ABC",
        "urdf_repo_path": "models/hand.urdf",
        "product_fingerprint": {"schema_version": 2, "hardware_map_crc": 7},
        "calibration_snapshot_sha256": "abc",
    }


def test_runtime_store_round_trip_and_backup(tmp_path) -> None:
    path = tmp_path / "multiturn_runtime.json"
    store = MultiTurnRuntimeStore(path)
    first = store.save(
        identity=_identity(),
        valid_motor_mask=0x5,
        logical_abs=range(MOTOR_COUNT),
        raw=range(MOTOR_COUNT),
        p4_time_ms=123,
    )
    assert store.load(identity=_identity())["logical_abs"] == list(range(MOTOR_COUNT))
    assert first["valid_motor_mask"] == "0x00000005"

    store.save(
        identity=_identity(),
        valid_motor_mask=0x1,
        logical_abs=[9] * MOTOR_COUNT,
        raw=[4097] * MOTOR_COUNT,
        p4_time_ms=456,
    )
    assert path.with_suffix(".json.bak").exists()
    assert store.load(identity=_identity())["raw"] == [1] * MOTOR_COUNT


def test_runtime_store_rejects_identity_or_integrity_mismatch(tmp_path) -> None:
    path = tmp_path / "multiturn_runtime.json"
    store = MultiTurnRuntimeStore(path)
    store.save(
        identity=_identity(),
        valid_motor_mask=1,
        logical_abs=[0] * MOTOR_COUNT,
        raw=[0] * MOTOR_COUNT,
        p4_time_ms=0,
    )
    with pytest.raises(DeviceStateStoreError):
        store.load(identity={"product_fingerprint": {}, "calibration_snapshot_sha256": "x"})

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["logical_abs"][0] = 99
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(DeviceStateStoreError):
        store.load(identity=_identity())


def test_circular_raw_distance_handles_wrap() -> None:
    assert circular_raw_distance(4090, 5) == 11
    assert circular_raw_distance(100, 500) == 400
    assert circular_raw_distance(100, 501) == 401


def test_nearest_logical_abs_supports_both_wrap_directions() -> None:
    assert nearest_logical_abs(4090, 4090, 5) == 4101
    assert nearest_logical_abs(4101, 5, 4090) == 4090
    assert nearest_logical_abs(3 * 4096 + 100, 100, 700) == 3 * 4096 + 700


def test_nearest_logical_abs_preserves_non_integer_turn_offset() -> None:
    """logical offset 不为整圈时，只能使用成对的 SavedRaw 计算增量。"""
    assert nearest_logical_abs(5000, 1000, 1100) == 5100


def test_anchor_identity_rejects_unknown_or_other_device_even_with_same_crc():
    current = _identity()
    assert multiturn_identity_compatible(current, current)
    for saved in (
        {**current, "device_id": ""},
        {**current, "device_id": "0000123456789ABD"},
        {**current, "urdf_repo_path": "other/hand.urdf"},
        {**current, "product_fingerprint": {"schema_version": 1, "hardware_map_crc": 7}},
    ):
        assert not multiturn_identity_compatible(saved, current)
