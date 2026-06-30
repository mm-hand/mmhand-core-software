"""UI mode runner with optional ROS2 joint-target input."""

from __future__ import annotations

import threading
from enum import Enum
from typing import Optional

from desktop.core.hand_backend import HandBackend
from desktop.ros2.joint_targets import DEFAULT_JOINT_TARGET_TOPIC, Ros2JointTargetBridge

POSE_DISPLAY_FROM_TEXT = False


class Mode(Enum):
    MONITOR = "monitor"
    TELEOP = "teleop"
    ALGORITHM = "algorithm"


def _run_monitor_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
) -> None:
    backend.reset()
    from desktop.ui.ui_app import HandGUI

    gui_win = HandGUI(
        backend,
        title_suffix="Monitor",
        current_port=port,
        current_mode="Monitor",
        action_callback=None,
        pose_generate=POSE_DISPLAY_FROM_TEXT,
        tactile_port=tactile_port,
    )
    gui_win.run()


def _run_teleop_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
) -> None:
    backend.reset()
    try:
        from desktop.teleop.example_teleop import run as teleop_run
    except ImportError:
        print("desktop.teleop.example_teleop module not found. Please implement run(backend, port=None).")
        return
    if not callable(teleop_run):
        print("example_teleop.run must be callable: run(backend, port).")
        return
    try:
        teleop_run(backend, port, tactile_port=tactile_port)
    except TypeError as exc:
        if "tactile_port" not in str(exc):
            raise
        teleop_run(backend, port)


def _run_algorithm_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
) -> None:
    backend.reset()
    try:
        from desktop.algorithm.example_algorithm import run as algorithm_run
    except ImportError:
        print("desktop.algorithm.example_algorithm module not found. Please implement run(backend, stop_event=None).")
        return
    if not callable(algorithm_run):
        print("example_algorithm.run must be callable: run(backend, stop_event=None).")
        return

    stop_event = threading.Event()

    def run_alg() -> None:
        algorithm_run(backend, stop_event=stop_event)

    alg_thread = threading.Thread(target=run_alg, daemon=False)
    alg_thread.start()
    from desktop.ui.ui_app import HandGUI

    gui_win = HandGUI(
        backend,
        title_suffix="Algorithm",
        current_port=port,
        current_mode="Algorithm",
        on_close_extra=lambda: stop_event.set(),
        pose_generate=POSE_DISPLAY_FROM_TEXT,
        tactile_port=tactile_port,
    )
    gui_win.run()
    stop_event.set()
    alg_thread.join(timeout=3.0)


def run(
    mode: Mode,
    backend: HandBackend,
    *,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    gui: bool = True,
) -> None:
    _ = gui
    if not isinstance(backend, HandBackend):
        backend = HandBackend(backend)

    if mode == Mode.MONITOR:
        _run_monitor_gui(backend, port, tactile_port)
        return
    if mode == Mode.TELEOP:
        _run_teleop_gui(backend, port, tactile_port)
        return
    if mode == Mode.ALGORITHM:
        _run_algorithm_gui(backend, port, tactile_port)
        return
    raise ValueError(f"Unknown mode: {mode}")


def run_ui(
    *,
    mode: str = "monitor",
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    ros2_enabled: bool = False,
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
    auto_start: bool = False,
    backend: Optional[HandBackend] = None,
) -> int:
    backend = backend or HandBackend()
    if port:
        if backend.connect(port):
            print(f"Connected {port}")
        else:
            print("Connection failed; launching UI without an active device")
    if auto_start and not ros2_enabled:
        backend.start()

    bridge = None
    if ros2_enabled:
        bridge = Ros2JointTargetBridge(backend, topic=ros_topic, auto_start=auto_start)
        try:
            bridge.start()
            print(f"ROS2 joint target input enabled on {ros_topic}")
        except Exception as exc:
            print(f"ROS2 disabled: {exc}")
            bridge = None

    try:
        run(Mode(mode), backend, port=port, tactile_port=tactile_port, gui=True)
    finally:
        if bridge is not None:
            bridge.stop()
    return 0
