"""验证多圈 JSON 在固件更新后仍可恢复，同时拒绝硬件拓扑不匹配。"""

import pytest

from desktop.core.device_state_store import DeviceStateStoreError, MultiTurnRuntimeStore


def _identity(*, hardware_map_crc: int, firmware_crc: int, snapshot: str) -> dict:
    return {
        "device_id": "0000123456789ABC",
        "urdf_repo_path": "model/hand.urdf",
        "product_fingerprint": {
            "schema_version": 2,
            "feedback_mapping_version": 3,
            "manual_scaled_mapping_version": 4,
            "control_model_crc": 11,
            "visual_model_crc": 12,
            "hardware_map_crc": hardware_map_crc,
            "parameter_crc": 13,
            "firmware_source_crc": firmware_crc,
            "structure_commit_crc": 14,
        },
        "calibration_snapshot_sha256": snapshot,
    }


def test_multiturn_anchor_survives_firmware_and_calibration_update(tmp_path):
    store = MultiTurnRuntimeStore(tmp_path / "multiturn_runtime.json")
    saved_identity = _identity(
        hardware_map_crc=0x12345678,
        firmware_crc=0x11111111,
        snapshot="old-calibration",
    )
    store.save(
        identity=saved_identity,
        valid_motor_mask=1,
        logical_abs=[4097] + [0] * 26,
        raw=[1] + [0] * 26,
        p4_time_ms=100,
    )

    # 普通重烧录或局部重新标定不改变舵机 raw/logical ABS 坐标关系。
    current_identity = _identity(
        hardware_map_crc=0x12345678,
        firmware_crc=0x22222222,
        snapshot="new-calibration",
    )
    assert store.load(identity=current_identity)["logical_abs"][0] == 4097


def test_multiturn_anchor_rejects_different_hardware_map(tmp_path):
    store = MultiTurnRuntimeStore(tmp_path / "multiturn_runtime.json")
    store.save(
        identity=_identity(
            hardware_map_crc=0x12345678,
            firmware_crc=1,
            snapshot="same",
        ),
        valid_motor_mask=1,
        logical_abs=[4097] + [0] * 26,
        raw=[1] + [0] * 26,
        p4_time_ms=100,
    )

    with pytest.raises(DeviceStateStoreError, match="another product/calibration"):
        store.load(
            identity=_identity(
                hardware_map_crc=0x87654321,
                firmware_crc=1,
                snapshot="same",
            )
        )
