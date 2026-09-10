"""replay_csv_loader.py - MMHand Desktop 模块契约。

职责：识别遥操或关节总表 CSV 并恢复带时间戳的稀疏轨迹。
线程：解析在调用线程执行，播放调度在 Replay 工作线程执行。
输入：运行 CSV、模型契约、活动关节范围和播放控制事件。
输出：带时间轴的稀疏关节帧、进度、诊断和停止结果。
状态所有权：轨迹对象由 ReplayService 拥有；设备运行状态只读且归 HandBackend。
禁止依赖：禁止依赖 Qt、串口和 ROS2，禁止直接修改 HandModel 或下位机状态。
"""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Dict, Optional, Sequence

from desktop.core.data_recording.motion_sampler import classify_motion_source
from desktop.core.protocol import ENCODER_COUNT

from .replay_models import ALL_JOINTS_MASK, ReplayFrame, ReplayTrace


REPLAY_LARGE_GAP_SECONDS = 1.0
_COMMAND_TARGET_FIELDS = (
    "output_joint_targets_json",
    "joint_targets_json",
    "input_joint_targets_json",
)


def _is_true(value: object) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "y", "on"}


def _parse_int(value: object, default: int = 0) -> int:
    text = str(value or "").strip()
    if not text:
        return int(default)
    return int(text, 0)


def _parse_host_time(row: Dict[str, str]) -> tuple[float, str]:
    """读取新格式毫秒时间，同时兼容旧ISO和Unix秒字段。"""
    candidates = [
        row.get("host_time", ""),
        row.get("host_time_iso", ""),
        row.get("host_time_unix_s", ""),
    ]
    for candidate in candidates:
        text = str(candidate or "").strip()
        if not text:
            continue
        try:
            epoch = float(text)
            if math.isfinite(epoch):
                return epoch, text
        except ValueError:
            pass
        try:
            parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
            return parsed.timestamp(), text
        except ValueError:
            continue
    raise ValueError("Missing a parseable host_time")


def _parse_joint_targets(
    row: Dict[str, str],
    previous_values: Optional[list[float]],
    previous_valid_mask: int,
) -> tuple[list[float], int]:
    """读取21维JSON或具名稀疏目标列；空单元格沿用该关节上一帧目标。"""
    for field_name in _COMMAND_TARGET_FIELDS:
        text = str(row.get(field_name, "") or "").strip()
        if not text:
            continue
        try:
            values = [float(value) for value in json.loads(text)]
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"{field_name} is not a valid JSON array") from exc
        if len(values) != ENCODER_COUNT:
            raise ValueError(f"{field_name} does not contain {ENCODER_COUNT} joint angles")
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{field_name} contains NaN or an infinite value")
        return values, ALL_JOINTS_MASK

    values = list(previous_values) if previous_values is not None else [0.0] * ENCODER_COUNT
    value_mask = int(previous_valid_mask) & ALL_JOINTS_MASK
    for joint in range(ENCODER_COUNT):
        text = str(row.get(f"J{joint:02d}_target_deg", "") or "").strip()
        if not text:
            continue
        value = float(text)
        if not math.isfinite(value):
            raise ValueError(f"J{joint:02d}_target_deg contains NaN or an infinite value")
        values[joint] = value
        value_mask |= 1 << joint
    if value_mask == 0:
        raise ValueError("Missing a joint target array or named Jxx_target_deg columns")
    return values, value_mask


def _same_optional_metadata(current: object, candidate: object) -> bool:
    if current in ("", 0, None) or candidate in ("", 0, None):
        return True
    return current == candidate


def _detect_csv_kind(fieldnames: Sequence[str]) -> str:
    names = set(fieldnames)
    if any(field in names for field in _COMMAND_TARGET_FIELDS):
        return "command"
    if any(f"J{joint:02d}_current_deg" in names for joint in range(ENCODER_COUNT)):
        return "joint_actual"
    if any(f"J{joint:02d}_target_deg" in names for joint in range(ENCODER_COUNT)):
        return "command"
    raise ValueError(
        "CSV is neither a motion command table nor a table with Jxx_target_deg/Jxx_current_deg columns"
    )


def _parse_actual_joint_row(
    row: Dict[str, str],
    previous_values: Optional[list[float]],
    previous_valid_mask: int,
) -> tuple[list[float], int, int]:
    """从关节宽表提取实际Current；非活动关节沿用上一帧或填0。"""
    values = list(previous_values) if previous_values is not None else [0.0] * ENCODER_COUNT
    active_mask = 0
    valid_mask = int(previous_valid_mask) & ALL_JOINTS_MASK
    has_active_columns = False
    for joint in range(ENCODER_COUNT):
        label = f"J{joint:02d}"
        active_text = str(row.get(f"{label}_active", "") or "").strip()
        if active_text:
            has_active_columns = True
            if _is_true(active_text):
                active_mask |= 1 << joint
        valid_field = f"{label}_current_valid"
        valid = _is_true(row.get(valid_field)) if valid_field in row else True
        text = str(row.get(f"{label}_current_deg", "") or "").strip()
        if not valid or not text:
            continue
        value = float(text)
        if not math.isfinite(value):
            raise ValueError(f"{label}_current_deg contains NaN or an infinite value")
        values[joint] = value
        valid_mask |= 1 << joint
    if not has_active_columns:
        active_mask = valid_mask
    if active_mask == 0:
        raise ValueError("This row has no active joints")
    return values, active_mask & ALL_JOINTS_MASK, valid_mask


def _append_timed_frame(
    frames: list[ReplayFrame],
    frame: ReplayFrame,
    *,
    first_epoch: float,
    epoch: float,
) -> int:
    """处理毫秒时间戳并列：连续帧保留最新值，离散命令保持原顺序。"""
    relative = max(0.0, epoch - first_epoch)
    normalized = ReplayFrame(
        relative,
        frame.target_urdf_deg,
        frame.frame_kind,
        frame.source_kind,
        frame.stream_id,
        frame.command_joint_mask,
        frame.joint_value_mask,
    )
    if not frames or relative > frames[-1].time_s:
        frames.append(normalized)
        return 0
    if (
        normalized.frame_kind == "continuous"
        and frames[-1].frame_kind == "continuous"
        and normalized.stream_id == frames[-1].stream_id
    ):
        frames[-1] = ReplayFrame(
            frames[-1].time_s,
            normalized.target_urdf_deg,
            normalized.frame_kind,
            normalized.source_kind,
            normalized.stream_id,
            normalized.command_joint_mask,
            normalized.joint_value_mask,
        )
        return 1
    # host_time只精确到毫秒；给同毫秒离散事件增加1 us顺序偏移，禁止吞掉命令。
    frames.append(
        ReplayFrame(
            frames[-1].time_s + 0.000001,
            normalized.target_urdf_deg,
            normalized.frame_kind,
            normalized.source_kind,
            normalized.stream_id,
            normalized.command_joint_mask,
            normalized.joint_value_mask,
        )
    )
    return 0


def load_replay_csv(path: Path | str) -> ReplayTrace:
    """载入动作指令总表或关节实际角总表，生成可安全回放的完整轨迹。"""
    csv_path = Path(path).expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV does not exist: {csv_path}")

    frames: list[ReplayFrame] = []
    warnings: list[str] = []
    source_names: set[str] = set()
    mapping_id = ""
    mapping_version = 0
    urdf_crc = 0
    joint_order_crc = 0
    active_joint_mask = 0
    raw_rows = 0
    dropped_rows = 0
    first_epoch: Optional[float] = None
    first_host_text = ""
    previous_epoch: Optional[float] = None
    max_gap_s = 0.0
    legacy_sent_unknown = False
    legacy_scope_unknown = False
    previous_actual_values: Optional[list[float]] = None
    previous_actual_valid_mask = 0
    previous_command_values: Optional[list[float]] = None
    previous_command_valid_mask = 0
    missing_requested_value_mask = 0

    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("CSV has no header")
        data_kind = _detect_csv_kind(reader.fieldnames)

        for row_number, row in enumerate(reader, start=2):
            raw_rows += 1
            try:
                epoch, host_text = _parse_host_time(row)
            except ValueError as exc:
                raise ValueError(f"Invalid CSV row {row_number}: {exc}") from exc
            if previous_epoch is not None and epoch < previous_epoch:
                raise ValueError(f"host_time moves backward at CSV row {row_number}")

            if data_kind == "command":
                row_source = str(row.get("source", "") or "").strip()
                live = _is_true(row.get("live_enabled"))
                inferred_source_kind, inferred_frame_kind, inferred_replayable = (
                    classify_motion_source(row_source, live)
                )
                source_kind = str(row.get("source_kind", "") or inferred_source_kind)
                frame_kind = str(row.get("frame_kind", "") or inferred_frame_kind)
                replayable_text = str(row.get("replayable", "") or "").strip()
                # 新schema列可能存在但旧写入方未填值；空值必须继续按来源推断，
                # 不能把整份历史CSV误判为“禁止回放”。
                replayable = (
                    _is_true(replayable_text)
                    if replayable_text
                    else inferred_replayable
                )
                if "valid" in row and not _is_true(row.get("valid")):
                    replayable = False
                if "blocked" in row and _is_true(row.get("blocked")):
                    replayable = False
                if "sent_to_backend" in row:
                    if not _is_true(row.get("sent_to_backend")):
                        replayable = False
                else:
                    legacy_sent_unknown = True
                if not replayable or frame_kind not in ("continuous", "discrete"):
                    dropped_rows += 1
                    previous_epoch = epoch
                    continue
                try:
                    targets, value_mask = _parse_joint_targets(
                        row,
                        previous_command_values,
                        previous_command_valid_mask,
                    )
                    previous_command_values = list(targets)
                    previous_command_valid_mask = int(value_mask)
                    row_mask = _parse_int(
                        row.get("command_joint_mask")
                        or row.get("active_joint_mask"),
                        0,
                    ) & ALL_JOINTS_MASK
                    if row_mask == 0:
                        row_mask = value_mask
                    row_mapping_id = str(row.get("mapping_id", "") or "")
                    row_mapping_version = _parse_int(row.get("mapping_version"), 0)
                    row_crc = _parse_int(row.get("urdf_config_crc"), 0)
                    row_order_crc = _parse_int(row.get("joint_order_crc"), 0)
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"Invalid CSV row {row_number}: {exc}") from exc
                if not _same_optional_metadata(mapping_id, row_mapping_id):
                    raise ValueError(f"mapping_id changes at CSV row {row_number}")
                if not _same_optional_metadata(mapping_version, row_mapping_version):
                    raise ValueError(f"mapping_version changes at CSV row {row_number}")
                if not _same_optional_metadata(urdf_crc, row_crc):
                    raise ValueError(f"URDF CRC changes at CSV row {row_number}")
                if not _same_optional_metadata(joint_order_crc, row_order_crc):
                    raise ValueError(f"joint_order_crc changes at CSV row {row_number}")
                mapping_id = mapping_id or row_mapping_id
                mapping_version = mapping_version or row_mapping_version
                urdf_crc = urdf_crc or row_crc
                joint_order_crc = joint_order_crc or row_order_crc
                stream_id = str(row.get("stream_id", "") or "")
                if not stream_id:
                    stream_id = (
                        f"legacy-{source_kind}-{row_mask:08X}"
                        if frame_kind == "continuous"
                        else f"legacy-event-{row_number}"
                    )
                source_names.add(row_source or source_kind)
            else:
                try:
                    targets, row_mask, value_mask = _parse_actual_joint_row(
                        row,
                        previous_actual_values,
                        previous_actual_valid_mask,
                    )
                except (TypeError, ValueError) as exc:
                    dropped_rows += 1
                    previous_epoch = epoch
                    continue
                previous_actual_values = list(targets)
                previous_actual_valid_mask = int(value_mask)
                source_kind = "joint_feedback"
                frame_kind = "continuous"
                stream_id = f"joint-actual-{row_mask:08X}"
                source_names.add("Joint Current")

            if row_mask == 0:
                legacy_scope_unknown = True
            missing_requested_value_mask |= row_mask & ~value_mask
            active_joint_mask |= row_mask
            if first_epoch is None:
                first_epoch = epoch
                first_host_text = host_text
            if previous_epoch is not None:
                max_gap_s = max(max_gap_s, epoch - previous_epoch)
            frame = ReplayFrame(
                0.0,
                targets,
                frame_kind,
                source_kind,
                stream_id,
                row_mask,
                value_mask,
            )
            dropped_rows += _append_timed_frame(
                frames,
                frame,
                first_epoch=first_epoch,
                epoch=epoch,
            )
            previous_epoch = epoch

    if legacy_sent_unknown:
        warnings.append("Legacy CSV has no sent_to_backend field; original queue acceptance cannot be verified")
    if legacy_scope_unknown or active_joint_mask == 0:
        warnings.append("Some CSV frames have no active scope; Replay will use the current P4 active joint mask")
    if data_kind == "joint_actual":
        warnings.append("Joint-angle CSV has no model CRC; Replay can only validate against the current calibrated workspace")
    if missing_requested_value_mask:
        missing_text = ", ".join(
            f"J{joint:02d}"
            for joint in range(ENCODER_COUNT)
            if missing_requested_value_mask & (1 << joint)
        )
        warnings.append(f"Some active joints have no valid values and will be skipped: {missing_text}")
    if max_gap_s > REPLAY_LARGE_GAP_SECONDS:
        warnings.append(f"A {max_gap_s:.3f}s target gap exists; Replay will preserve this pause")

    trace = ReplayTrace(
        path=csv_path,
        frames=frames,
        source=", ".join(sorted(source_names)),
        start_host_time=first_host_text,
        active_joint_mask=active_joint_mask,
        mapping_id=mapping_id,
        mapping_version=mapping_version,
        urdf_crc=urdf_crc,
        joint_order_crc=joint_order_crc,
        raw_row_count=raw_rows,
        dropped_row_count=dropped_rows,
        max_gap_s=max_gap_s,
        data_kind=data_kind,
        warnings=warnings,
    )
    trace.validate()
    return trace
