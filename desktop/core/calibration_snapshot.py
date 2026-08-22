"""calibration_snapshot.py - MMHand Desktop 模块契约。

职责：校验、合并并原子保存当前版本的标定快照。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import time
import uuid
from pathlib import Path

from .motor_mapping import MOTOR_TO_JOINT_INDEX
from .protocol import ENCODER_COUNT, MOTOR_COUNT, VALID_MOTOR_MASK


CALIBRATION_SNAPSHOT_FORMAT = "mmhand_calibration_bundle"
CALIBRATION_SNAPSHOT_SCHEMA_VERSION = 6
_JOINT_MASK = (1 << ENCODER_COUNT) - 1


class CalibrationSnapshotError(ValueError):
    """快照格式、完整性或版本不满足当前程序要求。"""


def parse_mask(value) -> int:
    """读取十六进制字符串或整数掩码。"""
    if isinstance(value, str):
        return int(value, 0)
    return int(value or 0)


def format_mask(value: int) -> str:
    """统一用八位十六进制保存掩码，便于人工查看。"""
    return f"0x{int(value) & 0xFFFFFFFF:08X}"


def motor_mask_for_joint_mask(joint_mask: int) -> int:
    """根据正式电机映射计算关节直接使用的舵机，不包含工作空间辅助电机。"""
    joint_mask = int(joint_mask) & _JOINT_MASK
    motor_mask = 0
    for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
        if 0 <= int(joint_idx) < ENCODER_COUNT and (
            joint_mask & (1 << int(joint_idx))
        ):
            motor_mask |= 1 << motor_idx
    return motor_mask & VALID_MOTOR_MASK


def joint_mask_for_motor_mask(motor_mask: int) -> int:
    """根据正式电机映射返回受舵机坐标变化影响的全部关节。"""
    motor_mask = int(motor_mask) & VALID_MOTOR_MASK
    joint_mask = 0
    for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
        if (motor_mask & (1 << motor_idx)) and 0 <= int(joint_idx) < ENCODER_COUNT:
            joint_mask |= 1 << int(joint_idx)
    return joint_mask & _JOINT_MASK


def _canonical_bytes(payload: dict) -> bytes:
    canonical = dict(payload)
    canonical.pop("integrity", None)
    return json.dumps(
        canonical,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")


def snapshot_sha256(payload: dict) -> str:
    """计算不含 integrity 字段的规范 JSON 摘要。"""
    return hashlib.sha256(_canonical_bytes(payload)).hexdigest()


def seal_snapshot(payload: dict) -> dict:
    """写入固定格式标识、版本和完整性摘要。"""
    sealed = dict(payload)
    sealed["format"] = CALIBRATION_SNAPSHOT_FORMAT
    sealed["schema_version"] = CALIBRATION_SNAPSHOT_SCHEMA_VERSION
    sealed["integrity"] = {
        "algorithm": "sha256",
        "digest": snapshot_sha256(sealed),
    }
    return sealed


def validate_snapshot(payload: dict, *, verify_integrity: bool = True) -> dict:
    """严格验证唯一受支持的 V6/V4 快照，不再兼容旧映射语义。"""
    if not isinstance(payload, dict):
        raise CalibrationSnapshotError("snapshot root must be an object")
    if payload.get("format") != CALIBRATION_SNAPSHOT_FORMAT:
        raise CalibrationSnapshotError("unsupported calibration snapshot format")
    version = int(payload.get("schema_version", 0) or 0)
    if version != CALIBRATION_SNAPSHOT_SCHEMA_VERSION:
        raise CalibrationSnapshotError(
            f"unsupported calibration snapshot schema v{version}; "
            f"only v{CALIBRATION_SNAPSHOT_SCHEMA_VERSION} is accepted"
        )

    results = payload.get("joint_calib_results")
    mins = payload.get("motor_abs_limit_min")
    maxs = payload.get("motor_abs_limit_max")
    manual_min = payload.get("manual_reference_min_raw")
    manual_max = payload.get("manual_reference_max_raw")
    if not isinstance(results, list) or len(results) != ENCODER_COUNT:
        raise CalibrationSnapshotError("joint_calib_results must contain 21 entries")
    if not isinstance(mins, list) or len(mins) != MOTOR_COUNT:
        raise CalibrationSnapshotError("motor_abs_limit_min must contain 27 entries")
    if not isinstance(maxs, list) or len(maxs) != MOTOR_COUNT:
        raise CalibrationSnapshotError("motor_abs_limit_max must contain 27 entries")
    if not isinstance(manual_min, list) or len(manual_min) != ENCODER_COUNT:
        raise CalibrationSnapshotError("manual_reference_min_raw must contain 21 entries")
    if not isinstance(manual_max, list) or len(manual_max) != ENCODER_COUNT:
        raise CalibrationSnapshotError("manual_reference_max_raw must contain 21 entries")

    if verify_integrity:
        integrity = payload.get("integrity")
        if not isinstance(integrity, dict) or integrity.get("algorithm") != "sha256":
            raise CalibrationSnapshotError("snapshot SHA-256 metadata is missing")
        expected = str(integrity.get("digest", "")).lower()
        actual = snapshot_sha256(payload)
        if expected != actual:
            raise CalibrationSnapshotError("snapshot SHA-256 mismatch")
    return payload


def read_snapshot(path: Path | str) -> dict:
    """读取并严格校验 V6 快照。"""
    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationSnapshotError(f"cannot read snapshot: {exc}") from exc
    return validate_snapshot(payload)


def atomic_write_snapshot(path: Path | str, payload: dict, *, backup: bool = True) -> dict:
    """先写临时文件并复读校验，再原子替换正式快照。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    sealed = seal_snapshot(payload)
    temp_path = target.with_name(f".{target.name}.{uuid.uuid4().hex}.tmp")
    backup_path = target.with_suffix(target.suffix + ".bak")
    try:
        with temp_path.open("w", encoding="utf-8", newline="\n") as handle:
            json.dump(sealed, handle, indent=2, ensure_ascii=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        read_snapshot(temp_path)
        if backup and target.exists():
            shutil.copy2(target, backup_path)
        os.replace(temp_path, target)
    finally:
        if temp_path.exists():
            temp_path.unlink()
    return sealed


def _normalized_list(values, count: int, default):
    normalized = list(values or [])[:count]
    while len(normalized) < count:
        normalized.append(default)
    return normalized


def _empty_v6_snapshot() -> dict:
    """建立新的 V6 空快照；旧 V5 只归档，不把旧映射语义带入当前控制。"""
    return {
        "joint_calib_results": [None] * ENCODER_COUNT,
        "motor_abs_limit_min": [0] * MOTOR_COUNT,
        "motor_abs_limit_max": [0] * MOTOR_COUNT,
        "motor_abs_limit_valid_bitmap": format_mask(0),
        "manual_reference_min_raw": [0] * ENCODER_COUNT,
        "manual_reference_max_raw": [0] * ENCODER_COUNT,
        "manual_reference_valid_bitmap": format_mask(0),
        "active_joint_mask": format_mask(0),
        "active_motor_mask": format_mask(0),
    }


def _read_v6_or_archive_v5(target: Path) -> dict:
    """读取 V6；遇到 V5 时先留档，再允许本次 Manual 写入创建干净 V6。"""
    try:
        return read_snapshot(target)
    except CalibrationSnapshotError:
        try:
            legacy = json.loads(target.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            raise
        if (
            legacy.get("format") != CALIBRATION_SNAPSHOT_FORMAT
            or int(legacy.get("schema_version", 0) or 0) != 5
        ):
            raise
        archive = target.with_name(
            f"{target.stem}.legacy_v5_{time.strftime('%Y%m%d_%H%M%S')}"
            f"{target.suffix}"
        )
        shutil.copy2(target, archive)
        return _empty_v6_snapshot()


def update_manual_snapshot(
    path: Path | str,
    *,
    valid_mask: int,
    min_raw: list[int],
    max_raw: list[int],
) -> dict:
    """更新 V6 Manual 数据，并只失效端点真正变化关节的 Auto/LUT 与限位。"""
    target = Path(path)
    if target.exists():
        payload = _read_v6_or_archive_v5(target)
    else:
        payload = _empty_v6_snapshot()
    if len(min_raw) != ENCODER_COUNT or len(max_raw) != ENCODER_COUNT:
        raise CalibrationSnapshotError("Manual endpoints must contain 21 entries")
    valid_mask = int(valid_mask) & _JOINT_MASK
    old_mask = parse_mask(payload.get("manual_reference_valid_bitmap"))
    old_min = _normalized_list(
        payload.get("manual_reference_min_raw"), ENCODER_COUNT, 0
    )
    old_max = _normalized_list(
        payload.get("manual_reference_max_raw"), ENCODER_COUNT, 0
    )
    changed_mask = 0
    for joint_idx in range(ENCODER_COUNT):
        bit = 1 << joint_idx
        old_valid = bool(old_mask & bit)
        new_valid = bool(valid_mask & bit)
        if (
            old_valid != new_valid
            or (new_valid and int(old_min[joint_idx]) != int(min_raw[joint_idx]))
            or (new_valid and int(old_max[joint_idx]) != int(max_raw[joint_idx]))
        ):
            changed_mask |= bit

    payload["manual_reference_min_raw"] = [int(value) for value in min_raw]
    payload["manual_reference_max_raw"] = [int(value) for value in max_raw]
    payload["manual_reference_valid_bitmap"] = format_mask(valid_mask)

    results = _normalized_list(
        payload.get("joint_calib_results"), ENCODER_COUNT, None
    )
    limit_valid = parse_mask(payload.get("motor_abs_limit_valid_bitmap"))
    invalid_motor_mask = motor_mask_for_joint_mask(changed_mask)
    for joint_idx in range(ENCODER_COUNT):
        if changed_mask & (1 << joint_idx):
            results[joint_idx] = None
    payload["joint_calib_results"] = results
    payload["motor_abs_limit_valid_bitmap"] = format_mask(
        limit_valid & ~invalid_motor_mask
    )

    active_joint_mask = 0
    for joint_idx, result in enumerate(results):
        if isinstance(result, dict) and bool(result.get("success", False)):
            motors = motor_mask_for_joint_mask(1 << joint_idx)
            if motors and ((limit_valid & ~invalid_motor_mask) & motors) == motors:
                active_joint_mask |= 1 << joint_idx
    payload["active_joint_mask"] = format_mask(active_joint_mask)
    payload["active_motor_mask"] = format_mask(
        motor_mask_for_joint_mask(active_joint_mask)
    )
    payload["updated_joint_mask"] = format_mask(changed_mask)
    payload["updated_motor_mask"] = format_mask(invalid_motor_mask)
    payload["saved_at_host_time"] = time.time()
    return atomic_write_snapshot(target, payload)


def invalidate_calibration_snapshot_scope(
    path: Path | str,
    *,
    motor_mask: int,
    joint_mask: int,
    reason: str,
) -> str:
    """原子失效指定 Auto/LUT/ABS Limit，同时保留独立的 Manual 端点。

    Reset Zero 会改变舵机 logical ABS 坐标系，旧 Auto/LUT 不能继续加载；
    Manual Min/Max 属于磁编码器原始端点，不依赖舵机圈数，必须继续保留。
    """
    target = Path(path)
    if not target.exists():
        return "no Calib Load snapshot"

    motor_mask = int(motor_mask) & VALID_MOTOR_MASK
    joint_mask = int(joint_mask) & _JOINT_MASK
    reason_tag = str(reason).strip().lower() or "coordinate_reset"
    stamp = time.strftime("%Y%m%d_%H%M%S")
    archive = target.with_name(
        f"{target.stem}.before_scoped_{reason_tag}_{stamp}{target.suffix}"
    )

    shutil.copy2(target, archive)
    payload = read_snapshot(target)
    results = _normalized_list(
        payload.get("joint_calib_results"), ENCODER_COUNT, None
    )
    mins = _normalized_list(payload.get("motor_abs_limit_min"), MOTOR_COUNT, 0)
    maxs = _normalized_list(payload.get("motor_abs_limit_max"), MOTOR_COUNT, 0)
    valid = parse_mask(payload.get("motor_abs_limit_valid_bitmap"))

    for joint_idx in range(ENCODER_COUNT):
        if joint_mask & (1 << joint_idx):
            results[joint_idx] = None
    for motor_idx in range(MOTOR_COUNT):
        if motor_mask & (1 << motor_idx):
            mins[motor_idx] = 0
            maxs[motor_idx] = 0

    payload["joint_calib_results"] = results
    payload["motor_abs_limit_min"] = mins
    payload["motor_abs_limit_max"] = maxs
    payload["motor_abs_limit_valid_bitmap"] = format_mask(valid & ~motor_mask)

    remaining_joint_mask = 0
    remaining_limit_mask = valid & ~motor_mask
    for joint_idx, result in enumerate(results):
        joint_motors = motor_mask_for_joint_mask(1 << joint_idx)
        if (
            isinstance(result, dict)
            and bool(result.get("success", False))
            and joint_motors
            and (remaining_limit_mask & joint_motors) == joint_motors
        ):
            remaining_joint_mask |= 1 << joint_idx
    payload["active_joint_mask"] = format_mask(remaining_joint_mask)
    payload["active_motor_mask"] = format_mask(
        motor_mask_for_joint_mask(remaining_joint_mask)
    )
    payload[f"scoped_{reason_tag}_invalidated_at"] = time.time()
    payload[f"scoped_{reason_tag}_motor_mask"] = format_mask(motor_mask)
    payload[f"scoped_{reason_tag}_joint_mask"] = format_mask(joint_mask)
    atomic_write_snapshot(target, payload, backup=False)
    return (
        f"snapshot backed up as {archive.name}; Manual endpoints preserved; "
        f"remaining Auto joints={remaining_joint_mask.bit_count()}"
    )


def merge_snapshot(
    previous: dict | None,
    current: dict,
    *,
    updated_joint_mask: int,
) -> dict:
    """把本轮成功关节事务合并到旧快照，保留其余关节和舵机数据。"""
    updated_joint_mask = int(updated_joint_mask) & _JOINT_MASK
    if updated_joint_mask == 0:
        raise CalibrationSnapshotError("updated_joint_mask is empty")
    updated_motor_mask = motor_mask_for_joint_mask(updated_joint_mask)

    if previous is not None:
        validate_snapshot(previous)
        merged = json.loads(json.dumps(previous, ensure_ascii=False))
    else:
        merged = _empty_v6_snapshot()

    current_results = _normalized_list(
        current.get("joint_calib_results"), ENCODER_COUNT, None
    )
    merged_results = _normalized_list(
        merged.get("joint_calib_results"), ENCODER_COUNT, None
    )
    for joint_idx in range(ENCODER_COUNT):
        if (updated_joint_mask & (1 << joint_idx)) == 0:
            continue
        result = current_results[joint_idx]
        if not isinstance(result, dict) or not bool(result.get("success", False)):
            raise CalibrationSnapshotError(
                f"J{joint_idx:02d} has no successful fresh calibration result"
            )
        merged_results[joint_idx] = result
    merged["joint_calib_results"] = merged_results

    current_mins = _normalized_list(current.get("motor_abs_limit_min"), MOTOR_COUNT, 0)
    current_maxs = _normalized_list(current.get("motor_abs_limit_max"), MOTOR_COUNT, 0)
    merged_mins = _normalized_list(merged.get("motor_abs_limit_min"), MOTOR_COUNT, 0)
    merged_maxs = _normalized_list(merged.get("motor_abs_limit_max"), MOTOR_COUNT, 0)
    current_valid = parse_mask(current.get("motor_abs_limit_valid_bitmap"))
    merged_valid = parse_mask(merged.get("motor_abs_limit_valid_bitmap"))
    for motor_idx in range(MOTOR_COUNT):
        bit = 1 << motor_idx
        if (updated_motor_mask & bit) == 0:
            continue
        if (current_valid & bit) == 0:
            raise CalibrationSnapshotError(
                f"M{motor_idx:02d} ABS limit is not valid"
            )
        merged_mins[motor_idx] = int(current_mins[motor_idx])
        merged_maxs[motor_idx] = int(current_maxs[motor_idx])
        merged_valid |= bit
    merged["motor_abs_limit_min"] = merged_mins
    merged["motor_abs_limit_max"] = merged_maxs
    merged["motor_abs_limit_valid_bitmap"] = format_mask(merged_valid)
    # 旧 V6 文件中的 motor_abs_limit_locked_bitmap 作为多余字段被忽略；
    # 新快照不再写出该字段，运行边界统一由 LUT 和 logical range 决定。
    merged.pop("motor_abs_limit_locked_bitmap", None)

    # Manual 端点来自 P4 RAM 的实时读回。只覆盖 P4 明确报告有效的条目，
    # 避免未连接关节的默认 0 污染已有便携快照。
    current_manual_mask = parse_mask(current.get("manual_reference_valid_bitmap"))
    merged_manual_mask = parse_mask(merged.get("manual_reference_valid_bitmap"))
    current_manual_min = _normalized_list(
        current.get("manual_reference_min_raw"), ENCODER_COUNT, 0
    )
    current_manual_max = _normalized_list(
        current.get("manual_reference_max_raw"), ENCODER_COUNT, 0
    )
    merged_manual_min = _normalized_list(
        merged.get("manual_reference_min_raw"), ENCODER_COUNT, 0
    )
    merged_manual_max = _normalized_list(
        merged.get("manual_reference_max_raw"), ENCODER_COUNT, 0
    )
    for joint_idx in range(ENCODER_COUNT):
        bit = 1 << joint_idx
        if current_manual_mask & bit:
            merged_manual_min[joint_idx] = int(current_manual_min[joint_idx])
            merged_manual_max[joint_idx] = int(current_manual_max[joint_idx])
            merged_manual_mask |= bit
    merged["manual_reference_min_raw"] = merged_manual_min
    merged["manual_reference_max_raw"] = merged_manual_max
    merged["manual_reference_valid_bitmap"] = format_mask(merged_manual_mask)

    # 有效掩码必须由真实条目重新计算，不能继续 OR 已失效的历史掩码。
    valid_joint_mask = 0
    for joint_idx, result in enumerate(merged_results):
        if isinstance(result, dict) and bool(result.get("success", False)):
            joint_motors = motor_mask_for_joint_mask(1 << joint_idx)
            if joint_motors and (merged_valid & joint_motors) == joint_motors:
                valid_joint_mask |= 1 << joint_idx
    valid_motor_scope = motor_mask_for_joint_mask(valid_joint_mask)
    merged["active_joint_mask"] = format_mask(valid_joint_mask)
    merged["active_motor_mask"] = format_mask(valid_motor_scope)
    merged["saved_at_host_time"] = time.time()
    merged["updated_joint_mask"] = format_mask(updated_joint_mask)
    merged["updated_motor_mask"] = format_mask(updated_motor_mask)
    merged["format"] = CALIBRATION_SNAPSHOT_FORMAT
    merged["schema_version"] = CALIBRATION_SNAPSHOT_SCHEMA_VERSION

    for key in (
        "hand_profile",
        "feedback_mapping_version",
        "manual_scaled_mapping_version",
        "urdf_config_crc",
        "manual_reference_crc",
        "source",
    ):
        if key in current:
            merged[key] = current[key]
    return seal_snapshot(merged)
