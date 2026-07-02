"""Command-line entrypoint for the desktop package."""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

DEFAULT_JOINT_TARGET_TOPIC = "/mmhand/joint_targets"
DEFAULT_RAW_IK_TARGET_TOPIC = "/raw_ik_target"


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", "-p", default=None, help="Main P4 serial port.")
    parser.add_argument(
        "--tactile-port",
        default=None,
        help="Optional S3 tactile serial port. Other data/control stay on --port.",
    )
    parser.add_argument("--ros2", action="store_true", help="Enable ROS2 21-angle subscriber.")
    parser.add_argument(
        "--ros2-input",
        choices=["joint_target", "raw_ik"],
        default="joint_target",
        help="Select ROS2 input path: final joint targets or external MANUS/IK raw targets.",
    )
    parser.add_argument(
        "--ros2-joint-target-topic",
        default=DEFAULT_JOINT_TARGET_TOPIC,
        help=f"ROS2 Float32MultiArray topic for final 21 joint targets (default: {DEFAULT_JOINT_TARGET_TOPIC}).",
    )
    parser.add_argument(
        "--ros2-raw-ik-topic",
        default=DEFAULT_RAW_IK_TARGET_TOPIC,
        help=f"ROS2 Float32MultiArray topic for external MANUS/IK 21-angle output (default: {DEFAULT_RAW_IK_TARGET_TOPIC}).",
    )
    parser.add_argument(
        "--ros-topic",
        default=None,
        help="Legacy ROS2 topic override for the selected --ros2-input.",
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
        choices=["monitor", "teleop", "algorithm"],
        default="monitor",
        help="UI mode to launch.",
    )

    cli = subparsers.add_parser("cli", help="Launch the terminal monitor/control CLI")
    _add_common_args(cli)
    cli.add_argument("--no-plot", action="store_true", help="Disable the debug plot window.")
    return parser


def _resolve_ros_topic(args: argparse.Namespace) -> str:
    if args.ros_topic:
        return str(args.ros_topic)
    if args.ros2_input == "raw_ik":
        return str(args.ros2_raw_ik_topic)
    return str(args.ros2_joint_target_topic)


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
            ros_topic=_resolve_ros_topic(args),
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
            ros_topic=_resolve_ros_topic(args),
            auto_start=bool(args.auto_start),
        )
    parser.error(f"unknown command: {args.command}")
    return 2
