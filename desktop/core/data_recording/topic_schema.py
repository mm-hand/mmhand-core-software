from __future__ import annotations

from typing import Dict, List


TOPIC_FIELDNAMES: Dict[str, List[str]] = {
    "core_state": [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
        "mode",
        "controller_started",
        "calib_status",
        "system_state",
        "system_fault_bitmap",
        "active_scope_label",
        "active_motor_mask",
        "active_joint_mask",
        "active_require_can",
        "multi_turn_unknown_mask",
        "has_sensor_data",
        "has_servo_angle_data",
        "has_servo_raw_data",
        "has_system_status_data",
        "has_fault_status_data",
    ],
    "joint_state": [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
        "joint_index",
        "is_active_joint",
        "joint_deg",
        "joint_target_deg",
        "joint_encoder_raw",
        "joint_reverse_release_fault",
        "joint_debug_valid",
        "joint_debug_actual_deg",
        "joint_debug_target_deg",
        "joint_debug_cmd_target_pos",
        "joint_debug_timestamp_ms",
    ],
    "servo_state": [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
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
        "voltage",
        "temperature",
        "overload_fault",
    ],
    "tactile_summary": [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
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
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
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
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
        "device_ts_ms",
        "joint_index",
        "valid",
        "target_deg",
        "actual_deg",
        "loop1_output",
        "loop2_actual",
        "loop2_output",
        "cmd_valid",
        "cmd_target_pos",
    ],
    "event_log": [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
        "event_type",
        "source",
        "detail",
        "active_scope_label",
        "active_motor_mask",
        "active_joint_mask",
    ],
    "teleop_input": [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
        "source",
        "frame_id",
        "valid",
        "latency_ms",
        "joint_targets_json",
    ],
    "teleop_output": [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
        "source",
        "live_enabled",
        "blocked",
        "block_reason",
        "sent_to_backend",
        "joint_targets_json",
    ],
}


def _wide_joint_fields() -> List[str]:
    fields = [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
    ]
    for idx in range(21):
        label = f"J{idx:02d}"
        fields.extend([
            f"{label}_deg",
            f"{label}_target_deg",
            f"{label}_encoder_raw",
            f"{label}_active",
            f"{label}_debug_actual_deg",
            f"{label}_debug_target_deg",
            f"{label}_debug_cmd_target_pos",
        ])
    return fields


def _wide_servo_fields() -> List[str]:
    fields = [
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
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
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
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
        "sample_index",
        "host_time_iso",
        "host_time_unix_s",
        "host_time",
        "elapsed_s",
        "device_timestamp",
        "device_elapsed_s",
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
    "core_state": "01_基础状态_core_state.csv",
    "joint_state": "02_关节长表_joint_state.csv",
    "joint_wide": "03_关节宽表_21关节一行.csv",
    "servo_state": "04_舵机长表_servo_state.csv",
    "servo_wide": "05_舵机宽表_27舵机一行.csv",
    "tactile_summary": "06_触觉汇总长表_tactile_summary.csv",
    "tactile_summary_wide": "07_触觉汇总宽表_五指一行.csv",
    "tactile_full": "08_触觉全量长表_tactile_full.csv",
    "tactile_full_thumb_wide": "09_触觉全量_拇指_wide.csv",
    "tactile_full_index_wide": "10_触觉全量_食指_wide.csv",
    "tactile_full_middle_wide": "11_触觉全量_中指_wide.csv",
    "tactile_full_ring_wide": "12_触觉全量_无名指_wide.csv",
    "tactile_full_pinky_wide": "13_触觉全量_小指_wide.csv",
    "joint_debug_packets": "14_joint_debug原始包.csv",
    "event_log": "15_操作事件_event_log.csv",
    "teleop_input": "16_遥操作输入_teleop_input.csv",
    "teleop_output": "17_遥操作输出_teleop_output.csv",
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
    "teleop_output",
    "joint_debug_packets",
    "event_log",
]


ROSBAG_TOPIC_NAMES = {
    "core_state": "/mmhand/core_state",
    "joint_state": "/mmhand/joint_state",
    "servo_state": "/mmhand/motor_state",
    "tactile_summary": "/mmhand/tactile_summary",
    "teleop_input": "/mmhand/teleop_input",
    "teleop_output": "/mmhand/teleop_output",
    "joint_debug_packets": "/mmhand/joint_debug_packets",
    "event_log": "/mmhand/events",
}
