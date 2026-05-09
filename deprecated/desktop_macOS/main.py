# main.py - Desktop entrypoint (control/development)

import argparse

from core_logic import HandController
from modes import Mode, run


def main():
    parser = argparse.ArgumentParser(description="Dexterous Hand Software")
    parser.add_argument(
        "--app-mode",
        choices=["control", "development"],
        default="control",
        help="应用模式: control(默认, GUI控制) / development(终端开发调试)",
    )
    parser.add_argument(
        "--mode", "-m",
        choices=["monitor", "teleop", "algorithm"],
        default="monitor",
        help="运行模式参数（control 模式下作为 GUI 类型输入：monitor/teleop/algorithm）",
    )
    parser.add_argument(
        "--port", "-p",
        default=None,
        help="串口路径。control 模式可选（不指定则在 GUI 内选择）",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="development 模式下关闭调试曲线窗口",
    )
    args = parser.parse_args()

    # Development mode: run desktop-native console monitor (no client.py import)
    if args.app_mode == "development":
        from development_mode import run_development_mode
        run_development_mode(no_plot=bool(args.no_plot))
        return
    elif args.app_mode == "control":
        # control 模式：保留 --mode，并用其决定启动的 GUI 类型
        show_gui = True
        run_mode = Mode(args.mode)
        port = args.port

        controller = HandController()
        if port:
            if controller.initialize(port):
                print(f"已连接 {port}")
            else:
                print("连接失败，将以未连接状态启动 GUI")

        # 根据启动模式配置 PID 初值；后续可在 GUI 中继续切换模式
        if run_mode == Mode.MONITOR:
            controller.set_pid_control(False)
        else:
            controller.set_pid_control(True)
        run(run_mode, controller, port=port, gui=show_gui)

    else:
        print("无效的应用模式，退出")
        return


if __name__ == "__main__":
    main()
