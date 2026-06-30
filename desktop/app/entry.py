"""Command-line entrypoint for the desktop package."""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

DEFAULT_JOINT_TARGET_TOPIC = "/mmhand/joint_targets"


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", "-p", default=None, help="Main P4 serial port.")
    parser.add_argument(
        "--tactile-port",
        default=None,
        help="Optional S3 tactile serial port. Other data/control stay on --port.",
    )
    parser.add_argument("--ros2", action="store_true", help="Enable ROS2 joint target subscriber.")
    parser.add_argument(
        "--ros-topic",
        default=DEFAULT_JOINT_TARGET_TOPIC,
        help=f"ROS2 Float32MultiArray topic for 21 joint targets (default: {DEFAULT_JOINT_TARGET_TOPIC}).",
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
            ros_topic=args.ros_topic,
            auto_start=bool(args.auto_start),
        )
    if args.command == "cli":
        from desktop.app.cli_monitor import run_cli_monitor

        return run_cli_monitor(
            port=args.port,
            tactile_port=args.tactile_port,
            no_plot=bool(args.no_plot),
            ros2_enabled=bool(args.ros2),
            ros_topic=args.ros_topic,
            auto_start=bool(args.auto_start),
        )
    parser.error(f"unknown command: {args.command}")
    return 2
