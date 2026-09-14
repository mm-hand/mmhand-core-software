"""__init__.py - MMHand Desktop 模块契约。

职责：声明 CSV 运行数据记录子包。
线程：采样入口可被通信线程调用，磁盘写入由记录器串行化；不得阻塞设备控制。
输入：带双时间戳的类型化 Topic 样本、事件和会话配置。
输出：CSV 文件以及结构化写入状态。
状态所有权：记录会话拥有文件句柄、采样序号和降采样缓存，不拥有设备真值。
禁止依赖：禁止依赖 UI 或改变控制状态，禁止在 writer 内重新解释协议业务语义。
"""

from .recorder import DataSessionRecorder

__all__ = ["DataSessionRecorder"]
