"""UI mode runner with optional ROS2 joint-target input."""

from __future__ import annotations

import threading
from enum import Enum
from typing import Optional

from desktop.core.hand_backend import HandBackend
from desktop.ros2.joint_targets import DEFAULT_JOINT_TARGET_TOPIC, Ros2JointTargetBridge
from desktop.ros2.raw_ik_target import RawIkTargetBridge

POSE_DISPLAY_FROM_TEXT = False


class Mode(Enum):
    MONITOR = "monitor"
    TELEOP = "teleop"
    REPLAY = "replay"


def _run_monitor_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    *,
    ros2_enabled: bool = False,
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
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
        ros2_enabled=ros2_enabled,
        ros_topic=ros_topic,
    )
    gui_win.run()


def _run_teleop_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    ros2_enabled: bool = False,
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
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
        teleop_run(
            backend,
            port,
            tactile_port=tactile_port,
            ros2_enabled=ros2_enabled,
            ros_topic=ros_topic,
        )
    except TypeError as exc:
        if not any(name in str(exc) for name in ("tactile_port", "ros2_enabled", "ros_topic")):
            raise
        teleop_run(backend, port)


def _run_replay_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    *,
    ros2_enabled: bool = False,
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
) -> None:
    from desktop.algorithm.replay_service import ReplayService
    replay_service = ReplayService(backend)

    from desktop.ui.ui_app import HandGUI

    gui_win = HandGUI(
        backend,
        title_suffix="Replay",
        current_port=port,
        current_mode="Replay",
        replay_service=replay_service,
        pose_generate=POSE_DISPLAY_FROM_TEXT,
        tactile_port=tactile_port,
        ros2_enabled=ros2_enabled,
        ros_topic=ros_topic,
    )
    gui_win.run()


def _make_ros2_bridge(
    backend: HandBackend,
    *,
    ros2_input: str,
    ros_topic: str,
    auto_start: bool,
    raw_ik_space: str = "legacy_calibrated_deg",
):
    if ros2_input == "raw_ik":
        return RawIkTargetBridge(
            backend,
            topic=ros_topic,
            auto_start=auto_start,
            input_space=raw_ik_space,
        )
    return Ros2JointTargetBridge(backend, topic=ros_topic, auto_start=auto_start)


def run(
    mode: Mode,
    backend: HandBackend,
    *,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    gui: bool = True,
    ros2_enabled: bool = False,
    ros2_input: str = "joint_target",
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
) -> None:
    _ = gui
    if mode == Mode.MONITOR:
        _run_monitor_gui(
            backend,
            port,
            tactile_port,
            ros2_enabled=ros2_enabled,
            ros_topic=ros_topic,
        )
        return
    if mode == Mode.TELEOP:
        _run_teleop_gui(
            backend,
            port,
            tactile_port,
            ros2_enabled=ros2_enabled,
            ros_topic=ros_topic,
        )
        return
    if mode == Mode.REPLAY:
        _run_replay_gui(
            backend,
            port,
            tactile_port,
            ros2_enabled=ros2_enabled,
            ros_topic=ros_topic,
        )
        return
    raise ValueError(f"Unknown mode: {mode}")


def run_ui(
    *,
    mode: str = "monitor",
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    ros2_enabled: bool = False,
    ros2_input: str = "joint_target",
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
    auto_start: bool = False,
    ros2_raw_ik_space: str = "legacy_calibrated_deg",
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
        bridge = _make_ros2_bridge(
            backend,
            ros2_input=ros2_input,
            ros_topic=ros_topic,
            auto_start=auto_start,
            raw_ik_space=ros2_raw_ik_space,
        )
        try:
            bridge.start()
            print(f"ROS2 {ros2_input} input enabled on {ros_topic}")
            if auto_start:
                # --auto-start 只启动普通控制器，不授予遥操输出权限。
                # ROS2 可继续接收和刷新最新帧，用户必须显式点击 RAW IK ENABLE。
                backend.start()
                print("Controller auto-started; RAW IK output remains disabled")
        except Exception as exc:
            print(f"ROS2 disabled: {exc}")
            bridge = None

    try:
        run(
            Mode(mode),
            backend,
            port=port,
            tactile_port=tactile_port,
            gui=True,
            ros2_enabled=bridge is not None,
            ros2_input=ros2_input,
            ros_topic=ros_topic,
        )
    finally:
        if bridge is not None:
            bridge.stop()
    return 0
