"""topic_schema.py - MMHand Desktop 模块契约。

职责：定义各运行数据 Topic 的字段、类型和 CSV 文件契约。
线程：采样入口可被通信线程调用，磁盘写入由记录器串行化；不得阻塞设备控制。
输入：带双时间戳的类型化 Topic 样本、事件和会话配置。
输出：CSV 文件以及结构化写入状态。
状态所有权：记录会话拥有文件句柄、采样序号和降采样缓存，不拥有设备真值。
禁止依赖：禁止依赖 UI 或改变控制状态，禁止在 writer 内重新解释协议业务语义。
"""
from __future__ import annotations

from typing import Dict, List


# CSV 统一只暴露两条时间轴：
# host_time 为带毫秒的本地日期时间，p4_time_ms 为固件开机后的 millis()。
TIME_FIELDS = ["sample_index", "host_time", "p4_time_ms"]


TOPIC_FIELDNAMES: Dict[str, List[str]] = {
    "core_state": [
        *TIME_FIELDS,
        "mode",
        "controller_started",
        "calib_status",
        "system_state",
        "system_fault_bitmap",
        "feedback_collision_armed_mask",
        "feedback_collision_hold_mask",
        "feedback_collision_transition_count",
        "active_scope_label",
        "active_motor_mask",
        "active_joint_mask",
        "active_require_can",
        "multi_turn_unknown_mask",
        "manual_reference_valid_mask",
        "manual_reference_crc",
        "manual_reference_revision",
        "manual_reference_committed",
        "manual_reference_commit_status",
        "auto_calibration_valid_mask",
        "has_sensor_data",
        "has_servo_angle_data",
        "has_servo_raw_data",
        "has_system_status_data",
        "has_fault_status_data",
    ],
    "tactile_full": [
        *TIME_FIELDS,
        "finger_index",
        "finger_name",
        "sensor_index",
        "row",
        "col",
        "fx",
        "fy",
        "fz",
        "force_mag",
    ],
    "event_log": [
        *TIME_FIELDS,
        "event_type",
        "source",
        "detail",
        "active_scope_label",
        "active_motor_mask",
        "active_joint_mask",
    ],
    "teleop_input": [
        *TIME_FIELDS,
        "source",
        "source_kind",
        "frame_kind",
        "replayable",
        "stream_id",
        "command_sequence",
        "frame_id",
        "valid",
        "latency_ms",
        "active_joint_mask",
        "command_joint_mask",
        "live_enabled",
        "blocked",
        "block_reason",
        "sent_to_backend",
        "input_space",
        "output_space",
        "mapping_id",
        "mapping_version",
        "urdf_config_crc",
        "joint_order_crc",
        "clamped_joints_json",
        "raw_joint_targets_json",
        "input_joint_targets_json",
        "output_joint_targets_json",
    ],
}


def _wide_joint_fields() -> List[str]:
    fields = [
        *TIME_FIELDS,
        "control_trace_sequence",
        "target_token",
        "command_age_ms",
        "feedback_age_ms",
        "trace_sequence_gap",
    ]
    for idx in range(21):
        label = f"J{idx:02d}"
        fields.extend([
            f"{label}_current_deg",
            f"{label}_current_valid",
            f"{label}_target_deg",
            f"{label}_encoder_raw",
            f"{label}_active",
        ])
    return fields


def _wide_servo_fields() -> List[str]:
    fields = [
        *TIME_FIELDS,
    ]
    for idx in range(27):
        label = f"M{idx:02d}"
        fields.extend([
            f"{label}_joint_index",
            f"{label}_target_abs",
            f"{label}_raw",
            f"{label}_abs",
            f"{label}_saved_raw",
            f"{label}_saved_raw_diff",
            f"{label}_load",
            f"{label}_load_mag",
            f"{label}_load_dir",
            f"{label}_speed",
            f"{label}_current_raw",
            f"{label}_voltage",
            f"{label}_temperature",
            f"{label}_online",
            f"{label}_raw_online",
            f"{label}_telem_online",
            f"{label}_active",
        ])
    return fields


_FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
_FINGER_CN_NAMES = ["拇指", "食指", "中指", "无名指", "小指"]


def _wide_tactile_summary_fields() -> List[str]:
    fields = [
        *TIME_FIELDS,
        "tactile_online",
        "tactile_summary_seq",
        "tactile_full_seq",
        "tactile_full_crc",
        "tactile_full_crc_errors",
        "tactile_full_seq_mismatches",
        "tactile_full_dropped",
    ]
    for name in _FINGER_NAMES:
        fields.extend([
            f"{name}_sensor_count",
            f"{name}_max_force",
            f"{name}_mean_force",
            f"{name}_contact_count",
        ])
    return fields


TOPIC_FIELDNAMES.update(
    {
        "joint_wide": _wide_joint_fields(),
        "servo_wide": _wide_servo_fields(),
        "tactile_summary_wide": _wide_tactile_summary_fields(),
    }
)


CSV_TOPIC_FILENAMES: Dict[str, str] = {
    "core_state": "01_基础状态.csv",
    "joint_wide": "02_关节总表.csv",
    "servo_wide": "03_舵机总表.csv",
    "tactile_summary_wide": "04_触觉总表.csv",
    "tactile_full": "05_触觉全量.csv",
    "event_log": "06_操作事件总表.csv",
    "teleop_input": "07_遥操作总表.csv",
}


DEFAULT_TOPICS = [
    "core_state",
    "joint_wide",
    "servo_wide",
    "tactile_summary_wide",
    "teleop_input",
    "event_log",
]

# Save Data 仅维护这些 CSV 总表。
DEFAULT_CSV_TOPICS = [
    "core_state",
    "joint_wide",
    "servo_wide",
    "tactile_summary_wide",
    "event_log",
    "teleop_input",
]

# 勾选触觉全量时，CSV 仅增加一张长表。
TACTILE_FULL_TOPICS = [
    "tactile_full",
]
