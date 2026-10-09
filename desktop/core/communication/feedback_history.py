"""Bounded immutable P4 feedback history for time-aligned calibration samples.

The RX thread records packets using their P4 clock, not their host arrival time.
A consumer marks a phase and reads its complete history; reset or eviction raises
an error instead of silently fitting an incomplete movement.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from threading import Lock
from typing import Iterable


class FeedbackHistoryError(RuntimeError):
    """The requested calibration feedback history is no longer available."""


@dataclass(frozen=True, slots=True)
class FeedbackFrame:
    kind: str
    p4_time_ms: int
    values: tuple[int, ...]
    valid_mask: int
    received_at: float
    sequence: int


@dataclass(frozen=True, slots=True)
class FeedbackCursor:
    generation: int
    sequence: int
    seeds: tuple[FeedbackFrame, ...]


class TimestampedFeedbackHistory:
    """Thread-safe packet history, scoped by a mark and a coordinate epoch."""

    def __init__(self, max_frames: int = 32768):
        if isinstance(max_frames, bool) or int(max_frames) != max_frames or max_frames < 1:
            raise ValueError("max_frames must be a positive integer")
        self._max_frames = int(max_frames)
        self._lock = Lock()
        self._frames: deque[FeedbackFrame] = deque()
        self._latest: dict[str, FeedbackFrame] = {}
        self._generation = 0
        self._sequence = 0
        self._discarded_through = 0

    def record(self, kind: str, p4_time_ms: int, values: Iterable[int],
               valid_mask: int, received_at: float) -> FeedbackFrame:
        if kind not in ("encoder_raw", "servo_abs"):
            raise ValueError(f"unsupported feedback kind: {kind}")
        timestamp = int(p4_time_ms)
        if not 0 <= timestamp <= 0xFFFFFFFF:
            raise ValueError("P4 feedback time must be uint32 milliseconds")
        positions = tuple(int(value) for value in values)
        mask = int(valid_mask)
        if mask < 0:
            raise ValueError("feedback valid_mask must not be negative")
        mask &= (1 << len(positions)) - 1
        arrival = float(received_at)
        if not math.isfinite(arrival):
            raise ValueError("feedback arrival time must be finite")
        with self._lock:
            previous = self._latest.get(kind)
            # An invalid packet, or corrected values at the same P4 timestamp,
            # is a distinct event. Only an identical re-transmission is skipped.
            if (previous is not None and previous.p4_time_ms == timestamp
                    and previous.values == positions and previous.valid_mask == mask):
                return previous
            self._sequence += 1
            frame = FeedbackFrame(kind, timestamp, positions, mask, arrival, self._sequence)
            if len(self._frames) == self._max_frames:
                self._discarded_through = self._frames.popleft().sequence
            self._frames.append(frame)
            self._latest[kind] = frame
            return frame

    def mark(self) -> FeedbackCursor:
        with self._lock:
            seeds = tuple(sorted(self._latest.values(), key=lambda frame: frame.sequence))
            return FeedbackCursor(self._generation, self._sequence, seeds)

    def read(self, cursor: FeedbackCursor) -> tuple[FeedbackFrame, ...]:
        with self._lock:
            if cursor.generation != self._generation:
                raise FeedbackHistoryError(
                    "feedback history reset: calibration coordinates or P4 session changed")
            if cursor.sequence > self._sequence:
                raise FeedbackHistoryError("feedback cursor belongs to a different history")
            if cursor.sequence < self._discarded_through:
                raise FeedbackHistoryError(
                    "feedback history overflow: complete calibration phase was evicted")
            return cursor.seeds + tuple(
                frame for frame in self._frames if frame.sequence > cursor.sequence)

    def reset(self) -> None:
        """Invalidate every cursor after F0 rebase, reconnect, or P4 restart."""
        with self._lock:
            self._generation += 1
            self._frames.clear()
            self._latest.clear()
            self._discarded_through = self._sequence
