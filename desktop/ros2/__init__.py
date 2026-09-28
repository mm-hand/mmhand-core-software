"""__init__.py - MMHand Desktop 模块契约。

职责：声明可选 ROS2 输入集成包。
线程：ROS2 executor 线程执行；与 Qt/串口线程通过 HandBackend 边界协作。
输入：ROS2 Topic 消息、节点参数和活动模型契约。
输出：校验后的 21 关节目标或明确拒绝诊断。
状态所有权：仅拥有 ROS2 节点、订阅器和最近消息元数据，不拥有设备状态。
禁止依赖：禁止依赖 Qt/UI，禁止直接写串口或绕过 HandBackend 的控制门禁。
"""

