# example_teleop.py - 遥操作模式接口
#
# 选择「遥操作」模式时，程序会调用本文件中的 run(controller, port)。
# 默认启动带控制面板的 GUI，便于手动操作。
# 可在此改为你的遥操逻辑：例如从数据手套/从手读取输入，映射为 21 个目标角度后
# 调用 controller.set_target_angles(angles)。

from typing import Optional
from core_logic import HandController


def run(controller: HandController, port: Optional[str] = None) -> None:
    """遥操作入口：接收 controller 与 port，在此填写你的遥操逻辑。"""
    from ui_main import HandGUI
    controller.set_pid_control(True)
    gui = HandGUI(controller, title_suffix="Teleoperation", current_port=port, current_mode="Teleoperation")
    gui.run()
