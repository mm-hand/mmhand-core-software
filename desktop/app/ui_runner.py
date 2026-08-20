"""ui_runner.py - MMHand Desktop 模块契约。

职责：装配 HandBackend、Qt 主界面和可选 ROS2 输入桥。
线程：在进程主线程执行；仅负责创建和关闭 CLI、Qt 及可选 ROS2 生命周期。
输入：命令行参数、环境变量和启动配置。
输出：已装配的应用服务、进程退出码和启动诊断。
状态所有权：只拥有启动期配置与生命周期句柄，不拥有设备运行状态。
禁止依赖：禁止实现协议解析、控制算法和设备安全判断，统一委托 core/服务层。
"""

from __future__ import annotations

import faulthandler
import sys
import threading
import traceback
from enum import Enum
from typing import Optional

from desktop.core.hand_backend import HandBackend
from desktop.core.session_log import log_session
from desktop.ros2.joint_targets import DEFAULT_JOINT_TARGET_TOPIC, Ros2JointTargetBridge
from desktop.ros2.raw_ik_target import RawIkTargetBridge

POSE_DISPLAY_FROM_TEXT = False


def _configure_gui_diagnostics() -> None:
    """在 Qt 应用创建前固定渲染后端，并保留原生崩溃的控制台诊断。"""
    from desktop.ui.ui_qt_compat import QT_LIB, configure_opengl_before_app

    mode = configure_opengl_before_app()
    try:
        faulthandler.enable()
    except (OSError, RuntimeError):
        pass
    previous_hook = sys.excepthook

    def _log_uncaught_exception(exc_type, exc_value, exc_traceback) -> None:
        detail = "".join(
            traceback.format_exception(exc_type, exc_value, exc_traceback)
        ).strip()
        log_session("GUI", f"uncaught exception:\n{detail}")
        previous_hook(exc_type, exc_value, exc_traceback)

    sys.excepthook = _log_uncaught_exception
    log_session("GUI", f"Qt={QT_LIB} requested_opengl={mode}")


class Mode(Enum):
    MONITOR = "monitor"
    REPLAY = "replay"


def _run_monitor_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    *,
    ros2_enabled: bool = True,
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


def _run_replay_gui(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    *,
    ros2_enabled: bool = False,
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
) -> None:
    from desktop.replay.replay_service import ReplayService
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
    ros2_enabled: bool = True,
    ros2_input: str = "joint_target",
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
    auto_start: bool = False,
    ros2_raw_ik_space: str = "legacy_calibrated_deg",
    backend: Optional[HandBackend] = None,
) -> int:
    _configure_gui_diagnostics()
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
                # --auto-start 只启动普通控制器，不授予外部 ROS2 输出权限。
                # ROS2 可继续接收和刷新最新帧，用户必须显式点击 ROS2 INPUT。
                backend.start()
                print("Controller auto-started; ROS2 input remains unauthorized")
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
