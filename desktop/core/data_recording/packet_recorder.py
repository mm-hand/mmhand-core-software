"""逐帧记录 P4/S3 原始载荷，并生成控制总表。

职责：解析待记录报文并生成记录行。
线程：记录 writer 线程执行。
输入：P4/S3 报文及主机接收时间。
输出：原始报文和控制跟踪记录。
状态所有权：拥有记录序列缓存，不拥有设备状态。
禁止依赖：禁止依赖 Qt、串口对象或上层应用。
"""
from __future__ import annotations

import struct
from ..protocol import (PACKET_TYPE_CONTROL_TRACE, PACKET_TYPE_CONTROL_COMPONENTS,
                        split_p4_timestamp, parse_control_trace_packet, parse_control_components_packet)


class PacketRecorder:
    def __init__(self, write):
        self.write = write
        self.details = {}
        self.components = {}
        self.previous = None

    def consume(self, source, packets, received_at):
        for packet_type, raw in packets:
            payload, stamp = split_p4_timestamp(packet_type, raw) if source == "p4" else (raw, None)
            self.write("packet_frames", {
                "host_time": received_at, "p4_time_ms": stamp,
                "source": source, "packet_type": packet_type, "payload_hex": raw.hex(),
            })
            if source == 'p4' and packet_type == PACKET_TYPE_CONTROL_COMPONENTS:
                part = parse_control_components_packet(payload)
                if part is not None:
                    self.components[(stamp, part['sequence'])] = part
                    while len(self.components) > 32:
                        self.components.pop(next(iter(self.components)))
                continue
            if source != "p4" or packet_type != PACKET_TYPE_CONTROL_TRACE:
                continue
            if len(payload) == 240:
                header = struct.unpack_from(">6I", payload)
                detail = dict(zip(("sequence", "servo_timestamp", "control_dt_us", "servo_valid_mask",
                                   "recovery_applied_mask", "p4_trace_dropped"), header))
                detail["actual_abs"] = struct.unpack_from(">27i", payload, 24)
                detail["pid_counts"] = struct.unpack_from(">27h", payload, 132)
                detail["recovery_counts"] = struct.unpack_from(">27h", payload, 186)
                self.details[(stamp, header[0])] = detail
                while len(self.details) > 32:
                    self.details.pop(next(iter(self.details)))
                continue
            trace = parse_control_trace_packet(payload)
            if trace is None or stamp is None:
                continue
            key = (stamp, trace["sequence"])
            detail = self.details.pop(key, None)
            components = self.components.pop(key, None)
            if key == self.previous:
                continue
            delta = (trace["sequence"] - self.previous[1]) & 0xFFFFFFFF if self.previous else 0
            row = {
                "host_time": received_at, "p4_time_ms": stamp,
                "control_trace_sequence": trace["sequence"], "target_token": trace["target_token"],
                "command_age_ms": (stamp - trace["target_accepted_ms"]) & 0xFFFFFFFF,
                "feedback_age_ms": (stamp - trace["encoder_timestamp_ms"]) & 0xFFFFFFFF,
                "trace_period_ms": (stamp - self.previous[0]) & 0xFFFFFFFF if self.previous else "",
                "trace_sequence_gap": max(0, delta - components['divisor']) if components and delta < 0x80000000 else '',
                "trace_sample_divisor": components['divisor'] if components else '',
                "trace_components_valid": int(components is not None),
                "motor_command_mask": f'0x{trace["motor_command_mask"]:08X}',
                "trace_detail_valid": int(detail is not None),
            }
            if detail:
                row.update({k: detail[k] for k in ("control_dt_us", "servo_valid_mask", "recovery_applied_mask", "p4_trace_dropped")})
                row["servo_feedback_age_ms"] = (stamp - detail["servo_timestamp"]) & 0xFFFFFFFF
            for i in range(21):
                row[f"J{i:02d}_slow_deg"] = components['slow_deg'][i] if components else ''
                valid = bool(trace["valid_joint_mask"] & (1 << i))
                row.update({f"J{i:02d}_target_deg": trace["target_deg"][i],
                            f"J{i:02d}_current_deg": trace["actual_deg"][i] if valid else "",
                            f"J{i:02d}_current_valid": int(valid),
                            f"J{i:02d}_active": int(bool(trace["active_joint_mask"] & (1 << i)))})
            for i in range(27):
                for field in ('proportional_counts', 'integral_counts', 'reversal_counts'):
                    row[f"M{i:02d}_{field}"] = components[field][i] if components else ''
                row[f"M{i:02d}_cmd_abs"] = trace["motor_command_abs"][i] if trace["motor_command_mask"] & (1 << i) else ""
                if detail:
                    for field in ("actual_abs", "pid_counts", "recovery_counts"):
                        row[f"M{i:02d}_{field}"] = detail[field][i] if detail["servo_valid_mask"] & (1 << i) else ""
            self.write("joint_wide", row)
            self.previous = key
