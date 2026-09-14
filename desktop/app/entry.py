"""entry.py - MMHand Desktop 模块契约。

职责：解析上位机命令行参数并选择 CLI 或 Qt 运行模式。
线程：在进程主线程执行；仅负责创建和关闭 CLI、Qt 及可选 ROS2 生命周期。
输入：命令行参数、环境变量和启动配置。
输出：已装配的应用服务、进程退出码和启动诊断。
状态所有权：只拥有启动期配置与生命周期句柄，不拥有设备运行状态。
禁止依赖：禁止实现协议解析、控制算法和设备安全判断，统一委托 core/服务层。
"""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

ROS2_INPUTS = ["off", "joint_target", "raw_ik"]
ROS2_OUTPUTS = ["off", "joint_states", "joint_cmd", "all"]


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", "-p", default=None, help="Main P4 serial port.")
    parser.add_argument(
        "--tactile-port",
        default=None,
        help="Optional S3 tactile serial port. Other data/control stay on --port.",
    )
    ros2_group = parser.add_mutually_exclusive_group()
    ros2_group.add_argument(
        "--ros2",
        dest="ros2",
        action="store_true",
        help="Enable the selected ROS2 input/output bridge (default).",
    )
    ros2_group.add_argument(
        "--no-ros2",
        dest="ros2",
        action="store_false",
        help="Launch without any ROS2 subscription or publisher.",
    )
    # 上位机默认装配 ROS2 输入桥；无 ROS2 环境可显式使用 --no-ros2。
    parser.set_defaults(ros2=True)
    parser.add_argument("--ros2-feedback-mode", choices=["always", "armed"], default="armed")
    parser.add_argument(
        "--ros2-input",
        choices=ROS2_INPUTS,
        default="joint_target",
        help="Select the fixed ROS2 input, or disable subscription (default: joint_target).",
    )
    parser.add_argument(
        "--ros2-output",
        choices=ROS2_OUTPUTS,
        default="all",
        help="Publish fixed joint_states, joint_cmd, both (all), or neither (off).",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Automatically start the controller when enabled by the selected entrypoint.",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="mmhand desktop upper-computer application")
    subparsers = parser.add_subparsers(dest="command")

    ui = subparsers.add_parser("ui", help="Launch the Qt UI")
    _add_common_args(ui)
    ui.add_argument(
        "--mode",
        "-m",
        choices=["monitor", "replay"],
        default="monitor",
        help="UI mode to launch.",
    )

    cli = subparsers.add_parser("cli", help="Launch the terminal monitor/control CLI")
    _add_common_args(cli)
    cli.add_argument("--no-plot", action="store_true", help="Disable the debug plot window.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 0
    if args.command == "ui":
        from desktop.app.ui_runner import run_ui

        return run_ui(
            mode=args.mode,
            port=args.port,
            tactile_port=args.tactile_port,
            ros2_enabled=bool(args.ros2),
            ros2_input=args.ros2_input,
            ros2_output=args.ros2_output,
            ros2_feedback_mode=args.ros2_feedback_mode,
            auto_start=bool(args.auto_start),
        )
    if args.command == "cli":
        from desktop.app.cli_monitor import run_cli_monitor

        return run_cli_monitor(
            port=args.port,
            tactile_port=args.tactile_port,
            no_plot=bool(args.no_plot),
            ros2_enabled=bool(args.ros2),
            ros2_input=args.ros2_input,
            ros2_output=args.ros2_output,
            ros2_feedback_mode=args.ros2_feedback_mode,
            auto_start=bool(args.auto_start),
        )
    parser.error(f"unknown command: {args.command}")
    return 2
