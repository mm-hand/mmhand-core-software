"""Versioned desktop control settings; RAM application on P4 is a separate transaction.

职责：校验、持久化并编码整组控制参数。
线程：设置操作线程；文件写入由互斥锁串行化。
输入：当前/默认JSON和schema字段。
输出：经校验配置、修订、CRC及下发载荷。
状态所有权：拥有本机配置，不拥有P4已应用状态。
禁止依赖：禁止串口、UI或运动授权。
"""
from __future__ import annotations

import copy
import json
import math
import threading
import struct
import zlib
from pathlib import Path

from .generated_control_parameters import BOUNDS, RAMP_BOUNDS
from desktop.paths import CONFIG_DIR, atomic_write_json


DEFAULT_PATH = CONFIG_DIR / 'control_parameters.default.json'
CURRENT_PATH = CONFIG_DIR / 'control_parameters.json'

TYPES = ('aa', 'mcp', 'pip', 'dip', 'cmc')

RAMP_TYPES = TYPES
RAMP_VERSION = 1


def validate(data: dict) -> dict:
    if not isinstance(data, dict) or set(data) != {'schema_version', 'revision', 'ramp_revision', *BOUNDS, 'reversal_ramp'}:
        raise ValueError('Control parameter groups do not match the current schema')
    if data['schema_version'] != 7 or not isinstance(data['revision'], int) or data['revision'] < 1 \
            or not isinstance(data['ramp_revision'], int) or data['ramp_revision'] < 1:
        raise ValueError('Control parameter schema or revision is invalid')
    for group, fields in BOUNDS.items():
        values = data[group]
        if not isinstance(values, dict) or set(values) != set(fields):
            raise ValueError(f'{group}: missing or unexpected field')
        for name, (lower, upper) in fields.items():
            value = values[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f'{group}.{name}: expected {lower}–{upper}')
    ramp = data['reversal_ramp']
    if not isinstance(ramp, dict) or set(ramp) != set(RAMP_BOUNDS):
        raise ValueError('reversal_ramp: missing or unexpected field')
    for name, (lower, upper) in RAMP_BOUNDS.items():
        value = ramp[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f'reversal_ramp.{name}: expected {lower}–{upper}')
    for name in RAMP_TYPES:
        if ramp[f'b0_{name}'] > data['reversal'][f'takeup_max_{name}']:
            raise ValueError(f'reversal_ramp.b0_{name} must not exceed reversal.takeup_max_{name}')
        if data['reversal'][f'takeup_max_{name}'] > 0 and ramp[f'b0_{name}'] <= 0 and ramp[f'vb_{name}'] <= 0:
            raise ValueError(f'reversal_ramp: {name} takeup_max>0 needs b0>0 or vb>0')
    if data['feedback']['fast_window_ms'] >= data['feedback']['slow_window_ms']:
        raise ValueError('Fast feedback window must be shorter than slow window')
    if data['ros2_input']['enter_deg'] >= data['ros2_input']['exit_deg']:
        raise ValueError('Stationary exit threshold must exceed enter threshold')
    return copy.deepcopy(data)


def configuration_crc(data: dict) -> int:
    return zlib.crc32(configuration_values(data)) & 0xFFFFFFFF


def configuration_values(data: dict) -> bytes:
    return struct.pack('>' + 'f' * sum(map(len, BOUNDS.values())),
                       *(data[group][field] for group, fields in BOUNDS.items() for field in fields))


def configuration_payload(data: dict) -> bytes:
    return struct.pack('>BI', 7, data['revision']) + configuration_values(data) + struct.pack('>I', configuration_crc(data))


def configuration_frame(data: dict) -> bytes:
    from .generated_upper_protocol import CMD_CONTROL_PARAMETERS_SET
    from .protocol import _build_command_frame
    return _build_command_frame(CMD_CONTROL_PARAMETERS_SET, configuration_payload(data))


def ramp_configuration_values(data: dict) -> bytes:
    ramp = data['reversal_ramp']
    return struct.pack('>' + 'f' * len(RAMP_BOUNDS),
                       *(ramp[name] for name in RAMP_BOUNDS))


def ramp_configuration_crc(data: dict) -> int:
    return zlib.crc32(ramp_configuration_values(data)) & 0xFFFFFFFF


def ramp_configuration_payload(data: dict) -> bytes:
    return struct.pack('>BI', RAMP_VERSION, data['ramp_revision']) + \
        ramp_configuration_values(data) + struct.pack('>I', ramp_configuration_crc(data))


def ramp_configuration_frame(data: dict) -> bytes:
    from .generated_upper_protocol import CMD_CONTROL_RAMP_PARAMETERS_SET
    from .protocol import _build_command_frame
    return _build_command_frame(CMD_CONTROL_RAMP_PARAMETERS_SET, ramp_configuration_payload(data))


class ControlParameterStore:
    def __init__(self, path: Path = CURRENT_PATH, defaults: Path = DEFAULT_PATH):
        self.path, self.defaults = Path(path), Path(defaults)
        self.lock = threading.RLock()
        self._default = validate(json.loads(self.defaults.read_text(encoding='utf-8')))
        self.error = ''
        if self.path.exists():
            try:
                self._data = validate(json.loads(self.path.read_text(encoding='utf-8')))
            except (OSError, ValueError, TypeError) as exc:
                self.error = f'Current control parameters invalid: {exc}. Restore Defaults before enabling control.'
                self._data = copy.deepcopy(self._default)
        else:
            self._data = copy.deepcopy(self._default)
            self._write(self._data)

    def snapshot(self) -> dict:
        with self.lock:
            result = copy.deepcopy(self._data)
            result['crc32'] = configuration_crc(result)
            result['ramp_crc32'] = ramp_configuration_crc(result)
            return result

    def save(self, proposed: dict) -> dict:
        with self.lock:
            data = validate(proposed)
            data['revision'] = self._data['revision'] + 1
            data['ramp_revision'] = self._data['ramp_revision'] + 1
            self._write(data)
            self._data = data
            self.error = ''
            return self.snapshot()

    def restore_defaults(self) -> dict:
        with self.lock:
            data = copy.deepcopy(self._default)
            data['revision'] = self._data['revision'] + 1
            data['ramp_revision'] = self._data['ramp_revision'] + 1
            self._write(data)
            self._data = data
            self.error = ''
            return self.snapshot()

    def _write(self, data: dict):
        atomic_write_json(self.path, data)
