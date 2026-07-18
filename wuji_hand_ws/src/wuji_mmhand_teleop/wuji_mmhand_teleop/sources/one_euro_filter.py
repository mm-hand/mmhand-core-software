"""Small vector One Euro filter used by live skeleton sources."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np


class OneEuroFilter3D:
    def __init__(
        self,
        min_cutoff: float = 0.5,
        beta: float = 5.0,
        d_cutoff: float = 1.0,
        reset_gap_s: float = 0.25,
    ) -> None:
        if min_cutoff <= 0.0 or d_cutoff <= 0.0 or beta < 0.0 or reset_gap_s <= 0.0:
            raise ValueError("invalid One Euro filter parameters")
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.reset_gap_s = float(reset_gap_s)
        self.reset()

    def reset(self) -> None:
        self._time_s: Optional[float] = None
        self._raw: Optional[np.ndarray] = None
        self._value: Optional[np.ndarray] = None
        self._velocity = np.zeros(3, dtype=float)

    @staticmethod
    def alpha(cutoff_hz: float, dt_s: float) -> float:
        tau = 1.0 / (2.0 * math.pi * float(cutoff_hz))
        return float(dt_s) / (tau + float(dt_s))

    def update(self, value, time_s: float) -> np.ndarray:
        current = np.asarray(value, dtype=float).reshape(3)
        timestamp = float(time_s)
        if not np.all(np.isfinite(current)) or not math.isfinite(timestamp):
            self.reset()
            return current.copy()
        if self._time_s is None or self._raw is None or self._value is None:
            self._time_s = timestamp
            self._raw = current.copy()
            self._value = current.copy()
            return current.copy()

        dt_s = timestamp - self._time_s
        if dt_s <= 0.0 or dt_s > self.reset_gap_s:
            self.reset()
            return self.update(current, timestamp)

        raw_velocity = (current - self._raw) / dt_s
        velocity_alpha = self.alpha(self.d_cutoff, dt_s)
        self._velocity = velocity_alpha * raw_velocity + (1.0 - velocity_alpha) * self._velocity
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self._velocity))
        value_alpha = self.alpha(cutoff, dt_s)
        self._value = value_alpha * current + (1.0 - value_alpha) * self._value
        self._raw = current.copy()
        self._time_s = timestamp
        return self._value.copy()


__all__ = ["OneEuroFilter3D"]
