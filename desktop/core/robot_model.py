"""robot_model.py - single merged robot-model contract.

职责：读取并校验 robot_model.json 的 source、manifest 和 mapping。
线程：纯模块缓存，首次读取后只读。
输入：desktop/resources/config/robot_model.json。
输出：只读模型字典及分节访问器。
状态所有权：只拥有 lru_cache 的只读解析结果。
禁止依赖：禁止导入 UI、通信或直接操作硬件。
"""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

from desktop.paths import ROBOT_MODEL_PATH

MODEL_SCHEMA_VERSION = 5
MAPPING_SCHEMA_VERSION = 4


class RobotModelError(ValueError):
    pass


@lru_cache(maxsize=1)
def load_model(path: str | None = None) -> dict:
    source = Path(path) if path else ROBOT_MODEL_PATH
    try:
        data = json.loads(source.read_text(encoding='utf-8'))
    except OSError as exc:
        raise RobotModelError(f'missing robot model: {source}') from exc
    except json.JSONDecodeError as exc:
        raise RobotModelError(f'invalid robot model JSON: {source}') from exc
    if not isinstance(data, dict) or int(data.get('schema_version', 0)) != MODEL_SCHEMA_VERSION:
        raise RobotModelError('unsupported robot model schema')
    for key in ('source', 'manifest', 'mapping'):
        if not isinstance(data.get(key), dict):
            raise RobotModelError(f'robot model is missing {key}')
    mapping = data['mapping']
    if int(mapping.get('schema_version', 0)) != MAPPING_SCHEMA_VERSION:
        raise RobotModelError('unsupported robot mapping schema')
    if not isinstance(mapping.get('joints'), list) or not isinstance(mapping.get('motors'), list):
        raise RobotModelError('robot mapping must contain joints and motors')
    return data


def mapping() -> dict:
    return load_model()['mapping']


def manifest() -> dict:
    return load_model()['manifest']


def source() -> dict:
    return load_model()['source']
