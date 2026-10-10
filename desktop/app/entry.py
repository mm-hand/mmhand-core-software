"""桌面启动契约。

职责：解析唯一网页入口的启动参数。
线程：主线程。
输入：命令行。
输出：应用退出码。
状态所有权：启动配置。
禁止依赖：不得实现协议和控制算法。
"""
import argparse


def build_parser():
    parser = argparse.ArgumentParser(description='MM-Hand 本地控制台')
    parser.add_argument('--no-ros2', action='store_true', help='不启动 ROS2 输入桥')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    from desktop.app.ui_runner import run_ui
    return run_ui(ros2_enabled=not args.no_ros2)
