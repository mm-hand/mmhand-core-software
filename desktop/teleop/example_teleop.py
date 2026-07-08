# example_teleop.py - 遥操作模式接口
#
# 选择「遥操作」模式时，程序会调用本文件中的 run(backend, port)。
# 默认启动带控制面板的 GUI，便于手动操作。
# 可在此改为你的遥操逻辑：例如从数据手套/从手读取输入，映射为 21 个目标角度后
# 调用 backend.send_joint_targets(angles, source="...", live=True)。

from typing import Optional
from desktop.core.hand_backend import HandBackend
from desktop.ros2.joint_targets import DEFAULT_JOINT_TARGET_TOPIC


def run(
    backend: HandBackend,
    port: Optional[str] = None,
    tactile_port: Optional[str] = None,
    ros2_enabled: bool = False,
    ros_topic: str = DEFAULT_JOINT_TARGET_TOPIC,
) -> None:
    """遥操作入口：接收 backend 与 port，在此填写你的遥操逻辑。"""
    from desktop.ui.ui_app import HandGUI
    if not isinstance(backend, HandBackend):
        backend = HandBackend(backend)
    backend.set_pid_control(True)
    gui = HandGUI(
        backend,
        title_suffix="Teleoperation",
        current_port=port,
        current_mode="Teleoperation",
        tactile_port=tactile_port,
        ros2_enabled=ros2_enabled,
        ros_topic=ros_topic,
    )
    gui.run()
