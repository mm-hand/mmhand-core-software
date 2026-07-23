"""读取 Save Data 生成的遥操作CSV，并恢复原始上位机时间轴。"""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
from typing import Dict, Optional

from desktop.core.protocol import ENCODER_COUNT

from .replay_models import ReplayFrame, ReplayTrace


# 超过该间隔仍保留原始停顿，但在UI中提示用户检查记录是否中断。
REPLAY_LARGE_GAP_SECONDS = 1.0
_TELEOP_SOURCE_MARKERS = ("ROS2", "RAW_IK", "MANUS", "WUJI", "TELEOP")


def _is_true(value: object) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "y", "on"}


def _parse_int(value: object, default: int = 0) -> int:
    text = str(value or "").strip()
    if not text:
        return int(default)
    return int(text, 0)


def _parse_host_time(row: Dict[str, str]) -> tuple[float, str]:
    """兼容新格式和历史Run Data，但不会把旧字段重新写回新CSV。"""
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
    raise ValueError("缺少可解析的host_time")


def _parse_joint_targets(row: Dict[str, str]) -> list[float]:
    # 新轨迹表优先播放真正通过保护后送入backend的目标。
    field_candidates = (
        "output_joint_targets_json",
        "joint_targets_json",
        "input_joint_targets_json",
    )
    for field_name in field_candidates:
        text = str(row.get(field_name, "") or "").strip()
        if not text:
            continue
        try:
            values = [float(value) for value in json.loads(text)]
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"{field_name}不是有效JSON数组") from exc
        if len(values) != ENCODER_COUNT:
            raise ValueError(f"{field_name}不是{ENCODER_COUNT}维关节角")
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{field_name}包含NaN或无穷值")
        return values
    raise ValueError("缺少关节目标数组")


def _is_teleop_source(source: str) -> bool:
    upper = str(source or "").upper()
    return any(marker in upper for marker in _TELEOP_SOURCE_MARKERS)


def _same_optional_metadata(current: object, candidate: object) -> bool:
    if current in ("", 0, None) or candidate in ("", 0, None):
        return True
    return current == candidate


def load_replay_csv(path: Path | str) -> ReplayTrace:
    """载入整份遥操作轨迹；无效、被拦截或未实际发送的行不会参与回放。"""
    csv_path = Path(path).expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV不存在：{csv_path}")

    frames: list[ReplayFrame] = []
    warnings: list[str] = []
    source = ""
    mapping_id = ""
    mapping_version = 0
    urdf_crc = 0
    active_joint_mask = 0
    raw_rows = 0
    dropped_rows = 0
    first_epoch: Optional[float] = None
    first_host_text = ""
    previous_epoch: Optional[float] = None
    max_gap_s = 0.0
    legacy_sent_unknown = False
    legacy_scope_unknown = False
    fixed_nonzero_mask = 0

    with csv_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("CSV没有表头")
        for row_number, row in enumerate(reader, start=2):
            raw_rows += 1
            row_source = str(row.get("source", "") or "").strip()
            if not _is_teleop_source(row_source):
                dropped_rows += 1
                continue
            if "valid" in row and not _is_true(row.get("valid")):
                dropped_rows += 1
                continue
            if "blocked" in row and _is_true(row.get("blocked")):
                dropped_rows += 1
                continue
            if "sent_to_backend" in row:
                if not _is_true(row.get("sent_to_backend")):
                    dropped_rows += 1
                    continue
            else:
                legacy_sent_unknown = True

            try:
                epoch, host_text = _parse_host_time(row)
                targets = _parse_joint_targets(row)
                row_mask = _parse_int(
                    row.get("command_joint_mask") or row.get("active_joint_mask"),
                    0,
                )
                row_mapping_id = str(row.get("mapping_id", "") or "")
                row_mapping_version = _parse_int(row.get("mapping_version"), 0)
                row_crc = _parse_int(row.get("urdf_config_crc"), 0)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"CSV第{row_number}行无效：{exc}") from exc

            if not _same_optional_metadata(mapping_id, row_mapping_id):
                raise ValueError(f"CSV第{row_number}行mapping_id发生变化")
            if not _same_optional_metadata(mapping_version, row_mapping_version):
                raise ValueError(f"CSV第{row_number}行mapping_version发生变化")
            if not _same_optional_metadata(urdf_crc, row_crc):
                raise ValueError(f"CSV第{row_number}行URDF CRC发生变化")
            mapping_id = mapping_id or row_mapping_id
            mapping_version = mapping_version or row_mapping_version
            urdf_crc = urdf_crc or row_crc
            source = source or row_source
            active_joint_mask |= row_mask
            if row_mask == 0:
                legacy_scope_unknown = True
            elif fixed_nonzero_mask == 0:
                fixed_nonzero_mask = row_mask
            elif row_mask != fixed_nonzero_mask:
                raise ValueError(
                    f"CSV第{row_number}行active scope发生变化，不能作为一条连续轨迹播放"
                )

            if first_epoch is None:
                first_epoch = epoch
                first_host_text = host_text
            if previous_epoch is not None:
                if epoch < previous_epoch:
                    raise ValueError(f"CSV第{row_number}行host_time发生倒退")
                gap = epoch - previous_epoch
                max_gap_s = max(max_gap_s, gap)
            relative = max(0.0, epoch - first_epoch)
            if frames and relative <= frames[-1].time_s:
                # 同一时间戳只保留最后到达的目标，避免回放时集中突发发送。
                frames[-1] = ReplayFrame(frames[-1].time_s, targets)
                dropped_rows += 1
            else:
                frames.append(ReplayFrame(relative, targets))
            previous_epoch = epoch

    if legacy_sent_unknown:
        warnings.append("历史CSV没有sent_to_backend，无法确认每帧当时是否真正入队")
    if legacy_scope_unknown or active_joint_mask == 0:
        warnings.append("CSV没有active scope，播放时将使用P4当前active joint mask")
    if max_gap_s > REPLAY_LARGE_GAP_SECONDS:
        warnings.append(
            f"存在{max_gap_s:.3f}s无目标间隔，Replay会原样保留这段停顿"
        )

    trace = ReplayTrace(
        path=csv_path,
        frames=frames,
        source=source,
        start_host_time=first_host_text,
        active_joint_mask=active_joint_mask,
        mapping_id=mapping_id,
        mapping_version=mapping_version,
        urdf_crc=urdf_crc,
        raw_row_count=raw_rows,
        dropped_row_count=dropped_rows,
        max_gap_s=max_gap_s,
        warnings=warnings,
    )
    trace.validate()
    return trace
