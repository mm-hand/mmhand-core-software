"""验证静态状态按需同步与任务诊断订阅命令的唯一协议契约。"""

import pytest

from desktop.core.generated_upper_protocol import (
    CMD_STATE_SYNC_REQUEST,
    CMD_TASK_DIAG_SUBSCRIBE,
    COMMAND_PAYLOAD_LENGTHS,
    STATE_SYNC_ALL,
    STATE_SYNC_JOINT_CALIBRATION,
    STATE_SYNC_PRODUCT_FINGERPRINT,
)
from desktop.core.protocol import (
    build_state_sync_request_cmd,
    build_task_diag_subscribe_cmd,
)


def test_state_sync_request_encodes_big_endian_mask() -> None:
    mask = STATE_SYNC_JOINT_CALIBRATION | STATE_SYNC_PRODUCT_FINGERPRINT
    frame = build_state_sync_request_cmd(mask)

    assert COMMAND_PAYLOAD_LENGTHS[CMD_STATE_SYNC_REQUEST] == 4
    assert frame == bytes([0xFE, 0x06, CMD_STATE_SYNC_REQUEST, 0x00, 0x00, 0x00, mask, 0xFF])


def test_state_sync_request_rejects_unknown_bits() -> None:
    with pytest.raises(ValueError):
        build_state_sync_request_cmd(STATE_SYNC_ALL | 0x80)


def test_task_diag_subscription_has_explicit_on_off_payload() -> None:
    assert COMMAND_PAYLOAD_LENGTHS[CMD_TASK_DIAG_SUBSCRIBE] == 1
    assert build_task_diag_subscribe_cmd(True) == bytes(
        [0xFE, 0x03, CMD_TASK_DIAG_SUBSCRIBE, 0x01, 0xFF]
    )
    assert build_task_diag_subscribe_cmd(False) == bytes(
        [0xFE, 0x03, CMD_TASK_DIAG_SUBSCRIBE, 0x00, 0xFF]
    )
