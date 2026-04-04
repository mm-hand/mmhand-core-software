# modes.py - Mode entrypoints (GUI-only flow)

import threading
from enum import Enum
from typing import Optional

from core_logic import HandController


class Mode(Enum):
    MONITOR = "monitor"
    TELEOP = "teleop"
    ALGORITHM = "algorithm"


def _run_monitor_gui(controller: HandController, port: Optional[str] = None) -> None:
    controller.reset()
    from ui_main import HandGUI
    gui_win = HandGUI(
        controller,
        title_suffix="Monitor",
        current_port=port,
        current_mode="Monitor",
        action_callback=lambda msg: print(f"UI action: {msg}", flush=True),
    )
    gui_win.run()


def _run_teleop_gui(controller: HandController, port: Optional[str] = None) -> None:
    controller.reset()
    try:
        from example_teleop import run as teleop_run
    except ImportError:
        print("example_teleop module not found. Please implement run(controller, port=None).")
        return
    if not callable(teleop_run):
        print("example_teleop.run must be callable: run(controller, port).")
        return
    teleop_run(controller, port)


def _run_algorithm_gui(controller: HandController, port: Optional[str] = None) -> None:
    controller.reset()
    try:
        from example_algorithm import run as algorithm_run
    except ImportError:
        print("example_algorithm module not found. Please implement run(controller, stop_event=None).")
        return
    if not callable(algorithm_run):
        print("example_algorithm.run must be callable: run(controller, stop_event=None).")
        return

    stop_event = threading.Event()

    def run_alg():
        algorithm_run(controller, stop_event=stop_event)

    alg_thread = threading.Thread(target=run_alg, daemon=False)
    alg_thread.start()
    from ui_main import HandGUI
    gui_win = HandGUI(
        controller,
        title_suffix="Algorithm",
        current_port=port,
        current_mode="Algorithm",
        on_close_extra=lambda: stop_event.set(),
    )
    gui_win.run()
    stop_event.set()
    alg_thread.join(timeout=3.0)


def run(
    mode: Mode,
    controller: HandController,
    *,
    port: Optional[str] = None,
    gui: bool = True,
) -> None:
    """
    Unified GUI entrypoint for all modes.
    The `gui` argument is kept only for backward compatibility.
    """
    if mode == Mode.MONITOR:
        _run_monitor_gui(controller, port)
        return

    if mode == Mode.TELEOP:
        _run_teleop_gui(controller, port)
        return

    if mode == Mode.ALGORITHM:
        _run_algorithm_gui(controller, port)
        return

    raise ValueError(f"Unknown mode: {mode}")
