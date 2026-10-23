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
import os
import tempfile
import threading
import struct
import zlib
from pathlib import Path


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'resources' / 'config'
DEFAULT_PATH = CONFIG_DIR / 'control_parameters.default.json'
CURRENT_PATH = CONFIG_DIR / 'control_parameters.json'

TYPES = ('aa', 'mcp', 'pip', 'dip', 'cmc')
BOUNDS = {
    'joint_kp': {name: (0.0, 2.0) for name in TYPES},
    'joint_ki': {
        **{name: (0.0, 1.0) for name in TYPES},
        **{f'integral_limit_counts_{name}': (0, 500) for name in TYPES},
        'integral_noise_deg': (0.0, 2.0),
        'integral_confirm_ms': (0, 1000),
    },
    'joint_kd': {name: (0.0, 0.5) for name in TYPES},
    'dual_tendon': {'pair_recovery_gain': (0.0, 0.5)},
    'feedback': {'fast_window_ms': (10, 100), 'slow_window_ms': (100, 1000)},
    'reversal': {
        **{f'confirm_deg_{name}': (0.1, 10.0) for name in TYPES},
        'confirm_ms': (0, 500),
        **{f'takeup_max_{name}': (0, 2000) for name in TYPES},
        **{f'exit_travel_deg_{name}': (0.05, 5.0) for name in TYPES},
    },
    'collision': {'arm_velocity_flex_deg_s': (0.1, 100.0), 'arm_velocity_extend_deg_s': (0.1, 100.0), 'stall_velocity_flex_deg_s': (0, 20.0), 'stall_velocity_extend_deg_s': (0, 20.0), 'min_error_deg': (0.5, 30.0), 'target_reset_deg': (0.1, 10.0), 'retreat_deg': (0.1, 10.0)},
    'output': {'lut_release_margin_counts': (0, 500), 'lut_tight_margin_counts': (0, 1000)},
    'ros2_input': {'min_cutoff_hz': (0.01, 10.0), 'beta': (0, 1.0), 'derivative_cutoff_hz': (0.01, 10.0), 'enter_deg': (0.01, 1.0), 'enter_dwell_ms': (0, 1000), 'exit_deg': (0.02, 2.0), 'exit_confirm_ms': (0, 500), 'max_gap_ms': (100, 5000)},
    'adaptive_kp': {'enabled': (0, 1), 'lag_limit_s': (0.02, 1.0), 'overshoot_deg': (0.5, 10.0)},
}

RAMP_TYPES = TYPES
RAMP_BOUNDS = {
    **{f'b0_{name}': (0, 500) for name in RAMP_TYPES},
    **{f'vb_{name}': (0, 5000) for name in RAMP_TYPES},
    **{f'load_limit_{name}': (100, 1023) for name in RAMP_TYPES},
    'takeup_timeout_ms': (0, 2000),
}
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
    payload = configuration_payload(data)
    return bytes((254, len(payload) + 2, CMD_CONTROL_PARAMETERS_SET)) + payload + bytes((255,))


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
    payload = ramp_configuration_payload(data)
    return bytes((254, len(payload) + 2, CMD_CONTROL_RAMP_PARAMETERS_SET)) + payload + bytes((255,))


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
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle, name = tempfile.mkstemp(prefix='.control-parameters-', suffix='.json', dir=self.path.parent)
        try:
            with os.fdopen(handle, 'w', encoding='utf-8') as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self.path)
        finally:
            if os.path.exists(name): os.unlink(name)
