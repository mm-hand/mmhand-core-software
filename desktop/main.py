"""main.py - MMHand Desktop 模块契约。

职责：保留旧启动方式到正式应用入口的兼容转发。
线程：在进程主线程执行；仅负责创建和关闭 CLI、Qt 及可选 ROS2 生命周期。
输入：命令行参数、环境变量和启动配置。
输出：已装配的应用服务、进程退出码和启动诊断。
状态所有权：只拥有启动期配置与生命周期句柄，不拥有设备运行状态。
禁止依赖：禁止实现协议解析、控制算法和设备安全判断，统一委托 core/服务层。
"""

import os
import sys

if __package__ in (None, ""):
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

from desktop.app.entry import main


if __name__ == "__main__":
    raise SystemExit(main())
