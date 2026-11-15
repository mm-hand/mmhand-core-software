"""新建标定包时许可名单登记全部电机，避免“建包当刻不在就永远写不进”。

2026-11-15 的现场：建包时 M26 还在归零/暂不可信，名单冻结成 M00–M25；
随后补写 M26 的锚点被范围门禁永久拒绝。名单只是写锚点的许可，真实锚点
仍由可信反馈和 multiturn_anchor_valid_bitmap 决定。
"""

import pytest

from desktop.core.calibration_snapshot import parse_mask, read_snapshot
from desktop.core.device_state_store import BundleMultiTurnStore
from desktop.core.protocol import VALID_MOTOR_MASK

IDENTITY = {
    "device_id": "0000123456789ABC",
    "urdf_repo_path": "tests://repo/model.urdf",
    "product_fingerprint": {
        "schema_version": 2,
        "feedback_mapping_version": 3,
        "hardware_map_crc": 0x1234,
    },
}
FIRST_SAVE_MASK = 0x03FFFFFF  # 建包瞬间可信的 M00–M25，M26 尚未可信


def _abs_values():
    return [100 + 37 * motor for motor in range(27)]


def test_new_bundle_permits_an_anchor_motor_that_was_not_trusted_yet(tmp_path):
    path = tmp_path / "bundle.mmcal.json"
    store = BundleMultiTurnStore(path)
    store.save(identity=IDENTITY, valid_motor_mask=FIRST_SAVE_MASK, logical_abs=_abs_values())

    payload = read_snapshot(path)
    assert parse_mask(payload["scoped_native_abs_motor_mask"]) == VALID_MOTOR_MASK
    # 许可写全，但真实锚点仍然只有建包时真正写入的电机。
    assert parse_mask(payload["multiturn_anchor_valid_bitmap"]) == FIRST_SAVE_MASK

    # M26 之后变得可信：补写不再被“不在名单里”拒绝。
    store.save(identity=IDENTITY, valid_motor_mask=VALID_MOTOR_MASK, logical_abs=_abs_values())

    payload = read_snapshot(path)
    assert parse_mask(payload["multiturn_anchor_valid_bitmap"]) == VALID_MOTOR_MASK
    assert parse_mask(payload["multiturn_anchor_updated_motor_mask"]) == VALID_MOTOR_MASK
    assert payload["multiturn_anchor_logical_abs"] == _abs_values()


def test_new_bundle_scope_does_not_create_missing_anchors(tmp_path):
    path = tmp_path / "bundle.mmcal.json"
    store = BundleMultiTurnStore(path)
    store.save(identity=IDENTITY, valid_motor_mask=1 << 3, logical_abs=_abs_values())

    payload = read_snapshot(path)
    # 只有真实写入的电机进入“有效锚点”位图；未接入/不可信的电机不会被当成已有锚点。
    assert parse_mask(payload["multiturn_anchor_valid_bitmap"]) == 1 << 3
