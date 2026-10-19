"""Give synthetic telemetry both clocks, just as the real RX reducer does."""
import time


def stamp(state, field):
    wall = getattr(state, field)
    state.monotonic_received[field] = time.monotonic() + (wall - time.time()) if wall else 0.0
