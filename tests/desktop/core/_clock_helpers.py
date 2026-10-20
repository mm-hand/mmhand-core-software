"""Give synthetic telemetry both clocks, just as the real RX reducer does."""
import contextlib
import threading
import time


def stamp(state, field):
    wall = getattr(state, field)
    state.monotonic_received[field] = time.monotonic() + (wall - time.time()) if wall else 0.0


@contextlib.contextmanager
def live_servo_feedback(state, field='servo_abs_last_rx_ts', interval=0.01):
    """Keep a synthetic servo-ABS timestamp fresh while a wait loop runs.

    Release/maintenance wait loops require live feedback; unit tests that fake a
    device must refresh the receive timestamp just like the RX thread would.
    """
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
