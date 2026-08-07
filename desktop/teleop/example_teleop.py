"""example_teleop.py - MMHand Desktop 模块契约。

职责：提供遥操链路的人工启动和联调示例。
线程：接收器可在 SDK/摄像头工作线程执行；纯映射和数学函数不创建线程。
输入：手套、骨架或视觉关键点，以及活动 URDF/标定配置。
输出：统一 J00-J20 目标、接收状态和可诊断质量信息。
状态所有权：接收器拥有最近输入帧；角度映射和模型配置按只读快照使用。
禁止依赖：禁止承载正式控制规则；该文件只装配示例 UI、ROS2 和遥操组件。
"""
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
