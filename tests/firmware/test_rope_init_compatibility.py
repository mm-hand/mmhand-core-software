"""验证旧版上位机 Pull Joint into Workspace 命令仍保持协议兼容。"""

import pytest

from desktop.core.generated_upper_protocol import (
    CMD_PULL_JOINT_INTO_WORKSPACE,
)
from desktop.core.protocol import build_pull_joint_into_workspace_cmd


def test_legacy_pull_joint_into_workspace_command_remains_available() -> None:
    """新版 UI 不显示按钮，但旧版客户端仍可发送同一四字节关节掩码。"""
    mask = (1 << 0) | (1 << 14)
    assert build_pull_joint_into_workspace_cmd(mask) == bytes(
        [0xFE, 0x06, CMD_PULL_JOINT_INTO_WORKSPACE, 0x00, 0x00, 0x40, 0x01, 0xFF]
    )


def test_legacy_pull_joint_into_workspace_rejects_empty_scope() -> None:
    with pytest.raises(ValueError):
        build_pull_joint_into_workspace_cmd(0)
