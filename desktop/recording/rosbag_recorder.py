"""rosbag_recorder.py - MMHand Desktop 模块契约。

职责：把统一 Topic 样本适配为可选的 ROS2 bag 记录。
线程：由记录会话工作线程调用；外部运行时初始化不得阻塞 Qt 主线程。
输入：统一 Topic 样本、会话元数据和记录配置。
输出：外部记录文件、写入状态和可诊断错误。
状态所有权：仅拥有记录后端句柄和会话资源，不拥有控制状态。
禁止依赖：禁止依赖 UI 或参与控制决策，禁止反向修改采集快照。
"""
from __future__ import annotations

import json
import os
import time
from datetime import datetime
from typing import Dict, Iterable, Optional

from desktop.core.data_recording.topic_schema import ROSBAG_TOPIC_NAMES


try:
    import rosbag2_py  # type: ignore
    from rclpy.serialization import serialize_message  # type: ignore
    from std_msgs.msg import String  # type: ignore
except Exception:  # pragma: no cover - ROS2 is optional and usually Linux-only
    rosbag2_py = None
    serialize_message = None
    String = None


class RosbagSessionRecorder:
    """Optional ROS2 bag writer.

    Rows are written as JSON payloads in std_msgs/msg/String. This keeps the
    first implementation independent from custom ROS message packages while
    preserving topic names and timestamps for replay/analysis.
    """

    def __init__(self):
        self.path = ""
        self.enabled = False
        self.error = ""
        self._writer = None
        self._topic_names: Dict[str, str] = {}

    @property
    def available(self) -> bool:
        return rosbag2_py is not None and serialize_message is not None and String is not None

    def start(
        self,
        path: str,
        topic_fieldnames: Dict[str, Iterable[str]],
        meta: Optional[Dict[str, object]] = None,
    ) -> None:
        self.stop()
        self.path = path
        self.enabled = self.available
        self.error = "" if self.enabled else "ROS2 rosbag2_py/std_msgs is not available"
        if not self.enabled:
            return
        try:
            os.makedirs(path, exist_ok=True)
            self._writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            )
            self._writer.open(storage_options, converter_options)
            self._topic_names = {
                topic: ROSBAG_TOPIC_NAMES.get(topic, f"/mmhand/{topic}")
                for topic in topic_fieldnames.keys()
            }
            for topic_name in self._topic_names.values():
                self._create_topic(topic_name)
            host_time = time.time()
            self.write_row(
                "event_log",
                {
                    "host_time": datetime.fromtimestamp(host_time).strftime(
                        "%Y-%m-%d %H:%M:%S.%f"
                    )[:-3],
                    "p4_time_ms": "",
                    "_host_time_ns": int(host_time * 1_000_000_000),
                    "event_type": "rosbag_start",
                    "source": "recorder",
                    "detail": "ROS2 bag recording started",
                },
            )
        except Exception as exc:
            self.error = str(exc)
            self.enabled = False
            self._writer = None

    def stop(self) -> None:
        self._writer = None
        self.enabled = False
        self.path = ""
        self._topic_names = {}

    def write_row(self, topic: str, row: Dict[str, object]) -> None:
        if not self.enabled or self._writer is None:
            return
        topic_name = self._topic_names.get(topic)
        if not topic_name:
            topic_name = ROSBAG_TOPIC_NAMES.get(topic, f"/mmhand/{topic}")
            self._topic_names[topic] = topic_name
            self._create_topic(topic_name)
        msg = String()
        # 下划线开头的是记录器内部索引，不进入 ROS2 消息正文。
        public_row = {key: value for key, value in row.items() if not str(key).startswith("_")}
        msg.data = json.dumps(public_row, ensure_ascii=False, default=str)
        self._writer.write(topic_name, serialize_message(msg), _row_timestamp_ns(row))

    def _create_topic(self, topic_name: str) -> None:
        if self._writer is None or rosbag2_py is None:
            return
        try:
            metadata = rosbag2_py.TopicMetadata(
                name=topic_name,
                type="std_msgs/msg/String",
                serialization_format="cdr",
                offered_qos_profiles="",
            )
        except TypeError:
            metadata = rosbag2_py.TopicMetadata(
                id=0,
                name=topic_name,
                type="std_msgs/msg/String",
                serialization_format="cdr",
                offered_qos_profiles="",
            )
        self._writer.create_topic(metadata)


def _row_timestamp_ns(row: Dict[str, object]) -> int:
    try:
        return int(row.get("_host_time_ns", 0) or 0)
    except Exception:
        return 0
