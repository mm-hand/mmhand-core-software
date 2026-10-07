"""entry.py - MMHand Desktop 模块契约。

职责：解析精简启动参数并打开 Qt 界面。
线程：在进程主线程执行；仅负责创建和关闭 CLI、Qt 及可选 ROS2 生命周期。
输入：命令行参数、环境变量和启动配置。
输出：已装配的应用服务、进程退出码和启动诊断。
状态所有权：只拥有启动期配置与生命周期句柄，不拥有设备运行状态。
禁止依赖：禁止实现协议解析、控制算法和设备安全判断，统一委托 core/服务层。
"""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MM-Hand Qt desktop application")
    parser.add_argument(
        "--no-ros2",
        dest="ros2",
        action="store_false",
        help="Launch without the ROS2 21-angle input bridge.",
    )
    parser.set_defaults(ros2=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    from desktop.app.ui_runner import run_ui

    return run_ui(ros2_enabled=bool(args.ros2))
