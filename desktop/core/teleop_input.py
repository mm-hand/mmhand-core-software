"""One Euro and fixed-anchor hysteresis for the sole ROS2 degree input.

Each joint owns its own filter state. The filter never authorizes motion;
HandBackend applies the same scope and safety gates after this transform.


职责：对ROS2度数输入做One Euro及固定锚点滞回。
线程：ROS2输入线程，在后端授权锁内调用。
输入：21维目标角及单调时间戳。
输出：过滤后的21维角度。
状态所有权：拥有滤波历史；重连/授权变更时重置。
禁止依赖：禁止设备IO或隐式运动授权。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from desktop.paths import CONFIG_DIR


@dataclass(frozen=True)
class InputFilterConfig:
    min_cutoff_hz: float = 1.0
    beta: float = 0.02
    derivative_cutoff_hz: float = 1.0
    enter_deg: float = 0.08
    enter_dwell_ms: int = 200
    exit_deg: float = 0.25
    exit_confirm_ms: int = 20
    max_gap_ms: int = 500

    @classmethod
    def from_file(cls, path: Path | None = None) -> 'InputFilterConfig':
        source = path or CONFIG_DIR / 'control_parameters.default.json'
        fields = json.loads(source.read_text(encoding='utf-8'))['ros2_input']
        value = cls(**fields)
        if not (0 < value.min_cutoff_hz <= 50 and 0 <= value.beta <= 10
                and 0 < value.derivative_cutoff_hz <= 50
                and 0 < value.enter_deg < value.exit_deg <= 10
                and 0 < value.enter_dwell_ms <= 5000
                and 0 < value.exit_confirm_ms <= 1000
                and 100 <= value.max_gap_ms <= 5000):
            raise ValueError('invalid ROS2 input filter configuration')
        return value


def _alpha(cutoff_hz: float, dt: float) -> float:
    return 1.0 / (1.0 + 1.0 / (2.0 * math.pi * cutoff_hz * dt))


class StationaryAwareOneEuro:
    """Input, output and stationary thresholds use degrees, matching mmhand_teleop."""

    def __init__(self, config: InputFilterConfig):
        self.config = config
        self.reset()

    def reset(self):
        self.previous_raw = None
        self.previous_time = None
        self.filtered = None
        self.derivative = 0.0
        self.output = None
        self.state = 'MOVING'
        self.candidate = None
        self.candidate_since = None
        self.anchor = None
        self.exit_since = None

    def update(self, raw_deg: float, now_s: float) -> tuple[float | None, bool]:
        if not math.isfinite(raw_deg) or not math.isfinite(now_s):
            return self.output, False
        if self.previous_time is None:
            self.previous_raw = raw_deg
            self.previous_time = now_s
            self.filtered = self.output = raw_deg
            self.candidate = raw_deg
            self.candidate_since = now_s
            return raw_deg, True
        dt = now_s - self.previous_time
        if dt <= 0:
            return self.output, False
        if dt * 1000 > self.config.max_gap_ms:
            self.reset()
            return self.update(raw_deg, now_s)
        derivative = (raw_deg - self.filtered) / dt
        self.derivative += _alpha(self.config.derivative_cutoff_hz, dt) * (derivative - self.derivative)
        cutoff = self.config.min_cutoff_hz + self.config.beta * abs(self.derivative)
        self.filtered += _alpha(cutoff, dt) * (raw_deg - self.filtered)
        self.previous_raw = raw_deg
        self.previous_time = now_s
        enter = self.config.enter_deg
        leave = self.config.exit_deg
        if self.state == 'MOVING':
            if self.candidate is None or abs(self.filtered - self.candidate) >= enter:
                self.candidate = self.filtered
                self.candidate_since = now_s
            elif now_s - self.candidate_since >= self.config.enter_dwell_ms / 1000:
                self.state = 'STATIONARY'
                self.anchor = self.filtered
                self.exit_since = None
            self.output = self.filtered
        else:
            if abs(self.filtered - self.anchor) > leave:
                self.exit_since = self.exit_since or now_s
                if now_s - self.exit_since >= self.config.exit_confirm_ms / 1000:
                    self.state = 'MOVING'
                    self.candidate = self.filtered
                    self.candidate_since = now_s
                    self.exit_since = None
                    # Start following from the held target to avoid one-frame
                    # jumps after a long stationary interval.
                    self.filtered = self.anchor
                    self.derivative = 0.0
            else:
                self.exit_since = None
            self.output = self.anchor if self.state == 'STATIONARY' else self.filtered
        return self.output, True


class TeleopInputFilter:
    def __init__(self, count: int = 21, config: InputFilterConfig | None = None):
        self.config = config or InputFilterConfig.from_file()
        self.filters = [StationaryAwareOneEuro(self.config) for _ in range(count)]

    def reset(self):
        for item in self.filters:
            item.reset()

    def update_degrees(self, values: Iterable[float], now_s: float) -> list[float] | None:
        sample = [float(value) for value in values]
        if len(sample) != len(self.filters) or not all(math.isfinite(v) for v in sample):
            return None
        if any(item.previous_time is not None and now_s <= item.previous_time for item in self.filters):
            return None
        result = [item.update(value, now_s) for item, value in zip(self.filters, sample)]
        return [value for value, valid in result] if all(valid for _, valid in result) else None
