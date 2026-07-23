from __future__ import annotations

from typing import Dict, List


# CSV/HDF5/ROS bag 统一只暴露两条时间轴：
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
        "manual_reference_nvs_sequence",
        "manual_reference_nvs_valid",
        "manual_reference_save_status",
        "auto_calibration_valid_mask",
        "has_sensor_data",
        "has_servo_angle_data",
        "has_servo_raw_data",
        "has_system_status_data",
        "has_fault_status_data",
    ],
    "joint_state": [
        *TIME_FIELDS,
        "joint_index",
        "is_active_joint",
        "joint_deg",
        "joint_target_deg",
        "joint_encoder_raw",
        "joint_debug_valid",
        "joint_debug_actual_deg",
        "joint_debug_target_deg",
        "joint_debug_cmd_target_pos",
    ],
    "servo_state": [
        *TIME_FIELDS,
        "motor_index",
        "joint_index",
        "is_active_motor",
        "is_active_joint",
        "motor_target_abs",
        "servo_raw",
        "servo_abs",
        "servo_hardware_pos",
        "online",
        "raw_online",
        "telem_online",
        "speed",
        "load",
        "load_mag",
        "load_dir",
        "current_raw",
        "voltage",
        "temperature",
        "overload_fault",
    ],
    "tactile_summary": [
        *TIME_FIELDS,
        "finger_index",
        "finger_name",
        "sensor_count",
        "max_force",
        "mean_force",
        "contact_count",
        "tactile_online",
        "tactile_summary_seq",
        "tactile_full_seq",
        "tactile_full_crc",
        "tactile_full_crc_errors",
        "tactile_full_seq_mismatches",
        "tactile_full_dropped",
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
    "joint_debug_packets": [
        *TIME_FIELDS,
        "joint_index",
        "valid",
        "target_deg",
        "actual_deg",
        "loop1_output",
        "cmd_valid",
        "cmd_target_pos",
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
        "clamped_joints_json",
        "raw_joint_targets_json",
        "input_joint_targets_json",
        "output_joint_targets_json",
    ],
}


def _wide_joint_fields() -> List[str]:
    fields = [
        *TIME_FIELDS,
    ]
    for idx in range(21):
        label = f"J{idx:02d}"
        fields.extend([
            f"{label}_current_deg",
            f"{label}_current_valid",
            f"{label}_target_deg",
            f"{label}_encoder_raw",
            f"{label}_active",
            f"{label}_debug_valid",
            f"{label}_debug_actual_deg",
            f"{label}_debug_target_deg",
            f"{label}_debug_pid_output",
            f"{label}_debug_cmd_valid",
            f"{label}_debug_cmd_target_pos",
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
            f"{label}_hardware_pos",
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


def _wide_tactile_full_fields() -> List[str]:
    fields = [
        *TIME_FIELDS,
        "finger_index",
        "finger_name",
    ]
    for sensor_idx in range(3):
        for row_idx in range(4):
            for col_idx in range(4):
                prefix = f"S{sensor_idx}_R{row_idx}_C{col_idx}"
                fields.extend([f"{prefix}_fx", f"{prefix}_fy", f"{prefix}_fz", f"{prefix}_mag"])
    return fields


TOPIC_FIELDNAMES.update(
    {
        "joint_wide": _wide_joint_fields(),
        "servo_wide": _wide_servo_fields(),
        "tactile_summary_wide": _wide_tactile_summary_fields(),
        "tactile_full_thumb_wide": _wide_tactile_full_fields(),
        "tactile_full_index_wide": _wide_tactile_full_fields(),
        "tactile_full_middle_wide": _wide_tactile_full_fields(),
        "tactile_full_ring_wide": _wide_tactile_full_fields(),
        "tactile_full_pinky_wide": _wide_tactile_full_fields(),
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
    "joint_state",
    "joint_wide",
    "servo_state",
    "servo_wide",
    "tactile_summary",
    "tactile_summary_wide",
    "teleop_input",
    "joint_debug_packets",
    "event_log",
]

# CSV面向人工查看，只输出合并后的总表；长表和原始debug仍保留给HDF5/ROS bag。
DEFAULT_CSV_TOPICS = [
    "core_state",
    "joint_wide",
    "servo_wide",
    "tactile_summary_wide",
    "event_log",
    "teleop_input",
]

# 勾选触觉全量时，CSV只增加一张长表；HDF5同时保留五指宽表便于算法读取。
TACTILE_FULL_TOPICS = [
    "tactile_full",
    "tactile_full_thumb_wide",
    "tactile_full_index_wide",
    "tactile_full_middle_wide",
    "tactile_full_ring_wide",
    "tactile_full_pinky_wide",
]


ROSBAG_TOPIC_NAMES = {
    "core_state": "/mmhand/core_state",
    "joint_state": "/mmhand/joint_state",
    "servo_state": "/mmhand/motor_state",
    "tactile_summary": "/mmhand/tactile_summary",
    "teleop_input": "/mmhand/teleop",
    "joint_debug_packets": "/mmhand/joint_debug_packets",
    "event_log": "/mmhand/events",
}
