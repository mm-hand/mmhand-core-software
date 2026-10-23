"""communication - P4 通信基础组件包。

职责：集中导出串口传输、帧解码、数据包路由、状态归并和 ACK 跟踪组件。
线程：组件本身不创建线程；由 ``LowerComputerComm`` 的 RX/TX 线程调用。
输入：串口字节流、P4 协议包和 ACK 查询条件。
输出：完整协议帧、路由结果、不可变时间语义和最新 ``HandModel`` 快照。
状态所有权：各组件只持有其职责内状态，禁止重新形成跨职责的通信大对象。
禁止依赖：禁止依赖 NiceGUI/UI、app、ROS2、Replay 和具体业务页面。
"""

from .ack_tracker import AckTracker
from .event_channel import EventChannel
from .frame_decoder import FrameDecoder
from .packet_router import P4PacketRouter, RoutedPacket
from .serial_transport import SerialTransport
from .state_reducer import StateReducer

__all__ = [
    "AckTracker",
    "EventChannel",
    "FrameDecoder",
    "P4PacketRouter",
    "RoutedPacket",
    "SerialTransport",
    "StateReducer",
]
