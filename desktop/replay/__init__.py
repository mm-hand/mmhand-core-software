"""__init__.py - MMHand Desktop 模块契约。

职责：声明 CSV 动作回放用例包。
线程：解析在调用线程执行，播放调度在 Replay 工作线程执行。
输入：运行 CSV、模型契约、活动关节范围和播放控制事件。
输出：带时间轴的稀疏关节帧、进度、诊断和停止结果。
状态所有权：轨迹对象由 ReplayService 拥有；设备运行状态只读且归 HandBackend。
禁止依赖：禁止依赖 Qt、串口和 ROS2，禁止直接修改 HandModel 或下位机状态。
"""

from .replay_controller import ReplayController, ReplayStatus
from .replay_csv_loader import load_replay_csv
from .replay_models import ReplayFrame, ReplayPreparation, ReplayTrace
from .replay_service import ReplayService

__all__ = [
    "ReplayController",
    "ReplayFrame",
    "ReplayPreparation",
    "ReplayService",
    "ReplayStatus",
    "ReplayTrace",
    "load_replay_csv",
]
