"""Give synthetic telemetry both clocks, just as the real RX reducer does."""
import contextlib
import threading
import time


def stamp(state, field):
    """Record a wall-clock field on the monotonic clock.

    The conversion preserves whatever offset the caller set, so a deliberately
    old timestamp still reads as old. Reading both clocks once, before the
    arithmetic, keeps the pairing consistent.
    """
    wall = getattr(state, field)
    if not wall:
        state.monotonic_received[field] = 0.0
        return
    now_monotonic, now_wall = time.monotonic(), time.time()
    state.monotonic_received[field] = now_monotonic + (wall - now_wall)


@contextlib.contextmanager
def live_status_feedback(state, field="system_status_last_rx_ts", interval=0.01,
                         max_seconds=60.0):
    """Refresh a synthetic status timestamp for the whole test.

    Real hardware streams status frames, so ``now - received_at(field)`` stays
    small. A synchronous mock only writes on command and goes stale during a
    multi-second wait loop. ``max_seconds`` bounds a feed that is never closed
    explicitly; a daemon thread would otherwise outlive its test.
    """
    stop = threading.Event()
    deadline = time.monotonic() + max_seconds

    def _loop():
        while not stop.wait(interval):
            if time.monotonic() >= deadline:
                return
            if getattr(state, "has_system_status_data", False):
                setattr(state, field, time.time())
                stamp(state, field)

    thread = threading.Thread(target=_loop, daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join(timeout=1.0)


@contextlib.contextmanager
def live_servo_feedback(state, field="servo_abs_last_rx_ts", interval=0.01):
    """Keep a synthetic servo-ABS timestamp fresh while a wait loop runs."""
    stop = threading.Event()

    def _loop():
        while not stop.wait(interval):
            setattr(state, field, time.time())
            stamp(state, field)

    thread = threading.Thread(target=_loop, daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join(timeout=1.0)
