# example_algorithm.py - 算法模式接口
#
# 选择「算法」模式时，程序会调用本文件中的 run(controller, stop_event=...)。
# 请在此实现你的控制逻辑：读取 controller.current_state，调用 controller.set_target_angles 等。
# 若传入 stop_event，循环中应检查 stop_event.is_set() 并在置位时退出（如关闭监控窗口时）。

import time
from typing import Optional
import threading

from core_logic import HandController


def run(
    controller: HandController,
    stop_event: Optional[threading.Event] = None,
) -> None:
    """算法入口：接收 controller 与可选的 stop_event（置位时退出循环）。"""
    print("算法模式已启动，请在 example_algorithm.py 的 run() 中填写控制逻辑。")
    try:
        while True:
            if stop_event and stop_event.is_set():
                break
            with controller.state_lock:
                angles = list(controller.current_state.angles)
            controller.set_target_angles(angles)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    if stop_event is None or not stop_event.is_set():
        controller.stop()
        controller.shutdown()
    print("算法退出")


