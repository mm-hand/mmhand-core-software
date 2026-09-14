"""验证旧版上位机 Rope Init 命令仍保持协议兼容。"""

import pytest

from desktop.core.generated_upper_protocol import (
    CMD_ROPE_INIT,
    COMMAND_PAYLOAD_LENGTHS,
)
from desktop.core.protocol import build_rope_init_cmd


def test_legacy_rope_init_command_remains_available() -> None:
    """新版 UI 不显示按钮，但旧版客户端仍可发送同一四字节关节掩码。"""
    mask = (1 << 0) | (1 << 14)
    assert COMMAND_PAYLOAD_LENGTHS[CMD_ROPE_INIT] == 4
    assert build_rope_init_cmd(mask) == bytes(
        [0xFE, 0x06, CMD_ROPE_INIT, 0x00, 0x00, 0x40, 0x01, 0xFF]
    )


def test_legacy_rope_init_rejects_empty_scope() -> None:
    with pytest.raises(ValueError):
        build_rope_init_cmd(0)
