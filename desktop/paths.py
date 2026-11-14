"""paths.py - MMHand Desktop 模块契约。

职责：集中解析仓库、资源、配置和运行数据目录。
线程：纯模块初始化或工具调用线程执行，不创建后台线程。
输入：包配置、路径参数或文本流。
输出：稳定导入边界、路径或过滤后的数据。
状态所有权：只拥有模块级不可变配置，不拥有设备运行状态。
禁止依赖：禁止导入 NiceGUI 控件或直接操作下位机通信。
"""

import json
import os
import tempfile
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
RESOURCE_DIR = PACKAGE_DIR / "resources"
CONFIG_DIR = RESOURCE_DIR / "config"
# Default runtime root is the config directory itself. Tests and tools may
# override it with MMHAND_DATA_DIR without touching tracked configuration.
DATA_DIR = Path(os.environ.get("MMHAND_DATA_DIR") or CONFIG_DIR).expanduser().resolve()
URDF_DIR = RESOURCE_DIR / "mmhand_urdf"
# Logs and recordings share the repository-level logs tree. The app writes
# recordings under logs/; the whole tree is gitignored.
LOG_DIR = Path(os.environ.get("MMHAND_LOG_DIR") or (PROJECT_ROOT / "logs")).resolve()
RUN_DATA_DIR = LOG_DIR


def relative_display(path) -> str:
    """Return a repository-relative POSIX path for UI text; never an absolute path."""
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(PROJECT_ROOT).as_posix()
    except ValueError:
        return Path(os.path.relpath(resolved, PROJECT_ROOT)).as_posix()


def atomic_write_json(path, payload, *, indent=2) -> None:
    """同目录临时文件 + fsync + 原子替换，避免中断留下半份 JSON。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    handle, name = tempfile.mkstemp(prefix=f'.{target.name}.', suffix='.tmp', dir=target.parent)
    try:
        with os.fdopen(handle, 'w', encoding='utf-8', newline='\n') as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=indent)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, target)
    finally:
        if os.path.exists(name):
            os.unlink(name)


# Active calibration snapshot. There is no separate checked-in template: the
# first calibration creates this file, later calibrations replace it atomically.
CALIB_ACTIVE_SNAPSHOT_PATH = DATA_DIR / "calibration_bundle.mmcal.json"
# One merged robot-model contract: hand-authored source/manifest plus generated mapping.
ROBOT_MODEL_PATH = CONFIG_DIR / "robot_model.json"
PRODUCT_FINGERPRINT_PATH = CONFIG_DIR / "product_fingerprint.json"
DEFAULT_URDF_PATH = URDF_DIR / "urdf" / "mmhand.urdf"
