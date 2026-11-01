"""验证静态状态按需同步与任务诊断订阅命令的唯一协议契约。"""

import pytest

from desktop.core.generated_upper_protocol import (
    CMD_DIAGNOSTICS_SUBSCRIBE,
    CMD_STATE_SYNC_REQUEST,
    STATE_SYNC_ALL,
    STATE_SYNC_JOINT_CALIBRATION,
    STATE_SYNC_PRODUCT_FINGERPRINT,
)
from desktop.core.protocol import (
    build_diagnostics_subscribe_cmd,
    build_state_sync_request_cmd,
)


def test_state_sync_request_encodes_big_endian_mask() -> None:
    mask = STATE_SYNC_JOINT_CALIBRATION | STATE_SYNC_PRODUCT_FINGERPRINT
    frame = build_state_sync_request_cmd(mask)

    assert frame == bytes([0xFE, 0x06, CMD_STATE_SYNC_REQUEST, 0x00, 0x00, 0x00, mask, 0xFF])


def test_state_sync_request_rejects_unknown_bits() -> None:
    with pytest.raises(ValueError):
        build_state_sync_request_cmd(STATE_SYNC_ALL | 0x100)


def test_task_diag_subscription_has_explicit_on_off_payload() -> None:
    assert build_diagnostics_subscribe_cmd(1) == bytes(
        [0xFE, 0x04, CMD_DIAGNOSTICS_SUBSCRIBE, 0x01, 0x05, 0xFF]
    )
    assert build_diagnostics_subscribe_cmd(0) == bytes(
        [0xFE, 0x04, CMD_DIAGNOSTICS_SUBSCRIBE, 0x00, 0x05, 0xFF]
    )
