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
import math
import os
import shutil
import threading
import time
import uuid
from pathlib import Path

from .motor_mapping import MOTOR_TO_JOINT_INDEX
from .protocol import (
    DUAL_TENDON_JOINT_INDICES,
    ENCODER_COUNT,
    HAND_PROFILE,
    MOTOR_COUNT,
    TENDON_LUT_MAX_SAMPLES,
    VALID_MOTOR_MASK,
    tendon_lut_field,
)


CALIBRATION_SNAPSHOT_FORMAT = "mmhand_calibration_bundle"
CALIBRATION_SNAPSHOT_SCHEMA_VERSION = 6
_JOINT_MASK = (1 << ENCODER_COUNT) - 1
_SNAPSHOT_WRITE_LOCK = threading.RLock()


class CalibrationSnapshotError(ValueError):
    """快照格式、完整性或版本不满足当前程序要求。"""


class CalibrationSnapshotStore:
    """标定快照的唯一运行时读写门面。

    业务层通过该对象完成完整的 read-modify-write 事务，避免 UI、启动恢复
    和多圈锚点路径各自操作同一文件。
    """

    def __init__(self, path: Path | str):
        self.path = Path(path)

    def exists(self) -> bool:
        return self.path.exists()

    def load(self) -> dict:
        return read_snapshot(self.path)

    def inspect(self, path: Path | str) -> dict:
        return read_snapshot(path)

    def activate(self, payload: dict) -> dict:
        with snapshot_write_transaction():
            return atomic_write_snapshot(self.path, payload)

    def merge_calibration(self, payload: dict, joint_mask: int) -> dict:
        with snapshot_write_transaction():
            previous = self.load() if self.exists() else None
            merged = merge_snapshot(
                previous,
                payload,
                updated_joint_mask=joint_mask,
            )
            return atomic_write_snapshot(self.path, merged)

    def invalidate_scope(
        self,
        *,
        motor_mask: int,
        joint_mask: int,
        reason: str,
    ) -> str:
        return invalidate_calibration_snapshot_scope(
            self.path,
            motor_mask=motor_mask,
            joint_mask=joint_mask,
            reason=reason,
        )


def parse_mask(value) -> int:
    """读取十六进制字符串或整数掩码。"""
    if isinstance(value, str):
        return int(value, 0)
    return int(value or 0)


def joint_calib_result_has_valid_tendon_lut(item, joint_idx: int) -> bool:
    """按当前电机映射校验单关节标定结果中的腱绳 LUT。"""
    if not isinstance(item, dict) or not bool(item.get("success", False)):
        return False
    if not bool(tendon_lut_field(item, "tendonLutValid", False)):
        return False
    try:
        count = int(tendon_lut_field(item, "tendonLutCount", 0))
    except (TypeError, ValueError):
        return False
    if count < 3 or count > TENDON_LUT_MAX_SAMPLES:
        return False
    expected_count = 2 if int(joint_idx) in DUAL_TENDON_JOINT_INDICES else 1
    expected_motors = [
        motor_idx
        for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX)
        if int(mapped_joint) == int(joint_idx)
    ]
    motors = tendon_lut_field(item, "tendonLutMotor", [])
    if len(expected_motors) != expected_count or not isinstance(motors, list):
        return False
    if len(motors) < 2:
        return False
    try:
        if any(int(motors[i]) != expected_motors[i] for i in range(expected_count)):
            return False
        if expected_count == 1 and int(motors[1]) != 0xFF:
            return False
    except (TypeError, ValueError):
        return False
    motor_abs = tendon_lut_field(item, "tendonLutMotorAbs", [])
    return bool(
        isinstance(motor_abs, list)
        and len(motor_abs) >= 2
        and all(
            isinstance(row, list) and len(row) >= count
            for row in motor_abs[:expected_count]
        )
    )


def joint_calib_result_tendon_lut_abs_ranges(
    item, joint_idx: int
) -> list[tuple[int, int, int]]:
    """返回有效 LUT 各执行器的 ``(motor, min_abs, max_abs)``。"""
    if not joint_calib_result_has_valid_tendon_lut(item, joint_idx):
        return []
    count = int(tendon_lut_field(item, "tendonLutCount", 0))
    motors = tendon_lut_field(item, "tendonLutMotor", [])
    motor_abs = tendon_lut_field(item, "tendonLutMotorAbs", [])
    actuator_count = 2 if int(joint_idx) in DUAL_TENDON_JOINT_INDICES else 1
    ranges: list[tuple[int, int, int]] = []
    try:
        for actuator in range(actuator_count):
            values = [int(value) for value in motor_abs[actuator][:count]]
            ranges.append((int(motors[actuator]), min(values), max(values)))
    except (TypeError, ValueError):
        return []
    return ranges


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
    multiturn_anchor_fields(payload)

    if verify_integrity:
        integrity = payload.get("integrity")
        if not isinstance(integrity, dict) or integrity.get("algorithm") != "sha256":
            raise CalibrationSnapshotError("snapshot SHA-256 metadata is missing")
        expected = str(integrity.get("digest", "")).lower()
        actual = snapshot_sha256(payload)
        if expected != actual:
            raise CalibrationSnapshotError("snapshot SHA-256 mismatch")
    return payload


def validate_snapshot_for_load(
    payload: dict, *, mapping_metadata: tuple[int, int] | None = None
) -> str | None:
    """校验加载前的静态标定契约；设备回读仍由控制器负责。"""
    try:
        validate_snapshot(payload)
        joint_mask = parse_mask(payload.get("active_joint_mask")) & _JOINT_MASK
        motor_mask = parse_mask(payload.get("active_motor_mask")) & VALID_MOTOR_MASK
        valid_motor_mask = (
            parse_mask(payload.get("motor_abs_limit_valid_bitmap"))
            & VALID_MOTOR_MASK
        )
    except (CalibrationSnapshotError, TypeError, ValueError) as exc:
        return str(exc)

    results = payload.get("joint_calib_results", [])
    if len(results) != ENCODER_COUNT:
        return f"invalid joint calibration count: {len(results)}"
    if joint_mask == 0:
        for joint_idx, item in enumerate(results):
            if isinstance(item, dict) and bool(item.get("success", False)):
                joint_mask |= 1 << joint_idx
    manual_mask = parse_mask(payload.get("manual_reference_valid_bitmap")) & _JOINT_MASK
    # Manual-only 与部分标定快照允许加载：Manual 端点逐关节恢复，缺失的
    # Auto/LUT 关节直接跳过，不会用默认值污染设备。
    if joint_mask == 0 and manual_mask == 0:
        return "snapshot contains no calibration result or Manual endpoints"
    if joint_mask != 0 and motor_mask == 0:
        return "snapshot contains no automatic calibration motor scope"

    missing = [
        joint_idx
        for joint_idx in range(ENCODER_COUNT)
        if (joint_mask & (1 << joint_idx))
        and (
            not isinstance(results[joint_idx], dict)
            or not bool(results[joint_idx].get("success", False))
        )
    ]
    if missing:
        return "snapshot missing active joint calibration: " + ",".join(
            f"J{int(idx):02d}" for idx in missing
        )

    try:
        if mapping_metadata is None:
            from .angle_mapping import urdf_manual_scaled_mapping_metadata

            mapping_metadata = urdf_manual_scaled_mapping_metadata()
        expected_version, expected_crc = mapping_metadata
    except (OSError, TypeError, ValueError) as exc:
        return f"current manual-scaled URDF mapping is invalid: {exc}"

    incompatible = [
        joint_idx
        for joint_idx in range(ENCODER_COUNT)
        if (joint_mask & (1 << joint_idx))
        and (
            int(results[joint_idx].get("mappingVersion", 0) or 0)
            != expected_version
            or int(results[joint_idx].get("urdfConfigCrc", 0) or 0)
            != expected_crc
        )
    ]
    if incompatible:
        return (
            "snapshot mapping/URDF contract mismatch; recalibration required: "
            + ",".join(f"J{int(idx):02d}" for idx in incompatible)
        )

    try:
        from .angle_mapping import get_spec
    except (OSError, TypeError, ValueError) as exc:
        return f"current URDF joint limits are invalid: {exc}"
    incompatible_ranges = []
    for joint_idx in range(ENCODER_COUNT):
        if (joint_mask & (1 << joint_idx)) == 0:
            continue
        try:
            result = results[joint_idx]
            urdf_min, urdf_max = get_spec(joint_idx).urdf_range_deg
            # V4 契约只要求 LUT 的 angleMin/angleMax 等于静态 URDF Min/Max。
            # 旧 JSON 里已删除的 currentMinDeg/currentMaxDeg 是多余字段，直接忽略。
            values = (
                float(result.get("angleMin")),
                float(result.get("angleMax")),
            )
            if (
                not all(math.isfinite(value) for value in values)
                or abs(values[0] - urdf_min) > 0.01
                or abs(values[1] - urdf_max) > 0.01
            ):
                incompatible_ranges.append(joint_idx)
        except (OSError, TypeError, ValueError):
            incompatible_ranges.append(joint_idx)
    if incompatible_ranges:
        return (
            "snapshot V4 range mismatch: "
            + ",".join(f"J{int(idx):02d}" for idx in incompatible_ranges)
            + "; re-run automatic calibration with the current firmware"
        )

    missing_lut = [
        joint_idx
        for joint_idx in range(ENCODER_COUNT)
        if (joint_mask & (1 << joint_idx))
        and not joint_calib_result_has_valid_tendon_lut(results[joint_idx], joint_idx)
    ]
    if missing_lut:
        return (
            "snapshot missing tendon LUT; recalibrate with the current firmware: "
            + ",".join(f"J{int(idx):02d}" for idx in missing_lut)
        )

    missing_motors = [
        motor_idx
        for motor_idx in range(MOTOR_COUNT)
        if (motor_mask & (1 << motor_idx))
        and not (valid_motor_mask & (1 << motor_idx))
    ]
    if missing_motors:
        return "snapshot missing active motor ABS limits: " + ",".join(
            f"M{int(idx):02d}" for idx in missing_motors
        )
    return None


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
    backup_path = target.parent / "backups" / (target.name + ".bak")
    backup_path.parent.mkdir(parents=True, exist_ok=True)
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


def snapshot_write_transaction():
    """返回覆盖快照 read-modify-write 全过程的同进程可重入锁。"""
    return _SNAPSHOT_WRITE_LOCK


def _normalized_list(values, count: int, default):
    normalized = list(values or [])[:count]
    while len(normalized) < count:
        normalized.append(default)
    return normalized


def multiturn_anchor_fields(payload: dict) -> tuple[int, list[int], list[int]]:
    """读取快照内的多圈后备锚点；旧 V6 快照没有该字段时视为全空。"""
    has_any = any(
        key in payload
        for key in (
            "multiturn_anchor_valid_bitmap",
            "multiturn_anchor_logical_abs",
            "multiturn_anchor_raw",
        )
    )
    if not has_any:
        return 0, [0] * MOTOR_COUNT, [0] * MOTOR_COUNT
    try:
        valid_mask = parse_mask(payload.get("multiturn_anchor_valid_bitmap", 0))
    except (TypeError, ValueError) as exc:
        raise CalibrationSnapshotError("invalid multiturn anchor valid bitmap") from exc
    logical_abs = payload.get("multiturn_anchor_logical_abs")
    raw = payload.get("multiturn_anchor_raw")
    if not isinstance(logical_abs, list) or len(logical_abs) != MOTOR_COUNT:
        raise CalibrationSnapshotError(
            "multiturn_anchor_logical_abs must contain 27 entries"
        )
    if not isinstance(raw, list) or len(raw) != MOTOR_COUNT:
        raise CalibrationSnapshotError("multiturn_anchor_raw must contain 27 entries")
    try:
        return (
            valid_mask & VALID_MOTOR_MASK,
            [int(value) for value in logical_abs],
            [int(value) % 4096 for value in raw],
        )
    except (TypeError, ValueError) as exc:
        raise CalibrationSnapshotError("invalid multiturn anchor values") from exc


def _empty_v6_snapshot() -> dict:
    """建立新的 V6 空快照。"""
    return {
        # 导入契约要求快照自带 hand_profile；缺失的文件无法通过 Inspect。
        "hand_profile": HAND_PROFILE,
        "joint_calib_results": [None] * ENCODER_COUNT,
        "motor_abs_limit_min": [0] * MOTOR_COUNT,
        "motor_abs_limit_max": [0] * MOTOR_COUNT,
        "motor_abs_limit_valid_bitmap": format_mask(0),
        "manual_reference_min_raw": [0] * ENCODER_COUNT,
        "manual_reference_max_raw": [0] * ENCODER_COUNT,
        "manual_reference_valid_bitmap": format_mask(0),
        "active_joint_mask": format_mask(0),
        "active_motor_mask": format_mask(0),
        "multiturn_anchor_valid_bitmap": format_mask(0),
        "multiturn_anchor_logical_abs": [0] * MOTOR_COUNT,
        "multiturn_anchor_raw": [0] * MOTOR_COUNT,
    }


def update_snapshot_multiturn_anchors(
    path: Path | str,
    *,
    motor_mask: int,
    logical_abs: list[int],
    raw: list[int],
) -> dict:
    """将已确认写入运行锚点的 Mxx ABS/raw 同步写回标定快照。"""
    if len(logical_abs) != MOTOR_COUNT or len(raw) != MOTOR_COUNT:
        raise CalibrationSnapshotError("multiturn anchors must contain 27 entries")
    with snapshot_write_transaction():
        payload = read_snapshot(path)
        active_mask = parse_mask(payload.get("active_motor_mask")) & VALID_MOTOR_MASK
        requested_mask = int(motor_mask) & VALID_MOTOR_MASK
        # A coordinate rebase invalidates the joint's Auto/LUT and removes it
        # from the active scope before the new turn-zero anchor is captured.
        # The invalidated scope is recorded in the snapshot; its motors still
        # have a valid new coordinate and must be allowed to persist that
        # anchor.  Otherwise every calibration rebase deadlocks in
        # "anchor motors are not active in calibration snapshot".
        invalidated_scope_mask = 0
        for key, value in payload.items():
            if key.startswith("scoped_") and key.endswith("_motor_mask"):
                invalidated_scope_mask |= parse_mask(value)
        anchor_scope_mask = active_mask | (invalidated_scope_mask & VALID_MOTOR_MASK)
        if requested_mask == 0 or (requested_mask & anchor_scope_mask) != requested_mask:
            raise CalibrationSnapshotError(
                "multiturn anchor motors are not active in calibration snapshot"
            )
        valid_mask, saved_abs, saved_raw = multiturn_anchor_fields(payload)
        for motor_idx in range(MOTOR_COUNT):
            bit = 1 << motor_idx
            if requested_mask & bit:
                saved_abs[motor_idx] = int(logical_abs[motor_idx])
                saved_raw[motor_idx] = int(raw[motor_idx]) % 4096
        payload["multiturn_anchor_valid_bitmap"] = format_mask(
            (valid_mask | requested_mask) & anchor_scope_mask
        )
        payload["multiturn_anchor_logical_abs"] = saved_abs
        payload["multiturn_anchor_raw"] = saved_raw
        payload["multiturn_anchor_updated_motor_mask"] = format_mask(requested_mask)
        payload["multiturn_anchor_saved_at_host_time"] = time.time()
        return atomic_write_snapshot(path, payload, backup=False)


def invalidate_snapshot_multiturn_anchors(
    path: Path | str,
    *,
    motor_mask: int,
    reason: str,
) -> dict | None:
    """Clear selected runtime anchors inside the active calibration bundle."""
    mask = int(motor_mask) & VALID_MOTOR_MASK
    if mask == 0:
        return None
    target = Path(path)
    if not target.exists():
        # 未标定设备没有快照可作废；与 invalidate_calibration_snapshot_scope
        # 的缺失容忍一致，归零/Release 事务不得因无文件而崩溃。
        return None
    with snapshot_write_transaction():
        payload = read_snapshot(target)
        valid_mask, logical_abs, raw = multiturn_anchor_fields(payload)
        for motor_idx in range(MOTOR_COUNT):
            if mask & (1 << motor_idx):
                logical_abs[motor_idx] = 0
                raw[motor_idx] = 0
        payload["multiturn_anchor_valid_bitmap"] = format_mask(valid_mask & ~mask)
        payload["multiturn_anchor_logical_abs"] = logical_abs
        payload["multiturn_anchor_raw"] = raw
        payload["multiturn_anchor_last_invalidation"] = {
            "motor_mask": format_mask(mask),
            "reason": str(reason),
            "host_time": time.time(),
        }
        return atomic_write_snapshot(target, payload, backup=False)


def update_manual_snapshot(
    path: Path | str,
    *,
    valid_mask: int,
    min_raw: list[int],
    max_raw: list[int],
    identity: dict | None = None,
    updated_joint_mask: int | None = None,
) -> dict:
    """只更新指定关节；None 表示显式整表替换，RAM 缺项不影响范围外 JSON。"""
    with snapshot_write_transaction():
        return _update_manual_snapshot_locked(
            path,
            valid_mask=valid_mask,
            min_raw=min_raw,
            max_raw=max_raw,
            identity=identity,
            updated_joint_mask=updated_joint_mask,
        )


def _update_manual_snapshot_locked(
    path: Path | str,
    *,
    valid_mask: int,
    min_raw: list[int],
    max_raw: list[int],
    identity: dict | None = None,
    updated_joint_mask: int | None = None,
) -> dict:
    """在快照写事务内更新 Manual 数据。"""
    target = Path(path)
    if target.exists():
        payload = read_snapshot(target)
        if identity is not None and any(payload.get(key) != identity.get(key)
                for key in ("device_id", "urdf_repo_path")):
            raise CalibrationSnapshotError("existing Manual snapshot identity mismatch")
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
    scope = _JOINT_MASK if updated_joint_mask is None else updated_joint_mask
    if type(scope) is not int or scope <= 0 or scope & ~_JOINT_MASK:
        raise CalibrationSnapshotError("Manual update joint scope is empty or invalid")
    valid_mask = (old_mask & ~scope) | (valid_mask & scope)
    min_raw = [int(min_raw[j]) if scope & (1 << j) else old_min[j]
               for j in range(ENCODER_COUNT)]
    max_raw = [int(max_raw[j]) if scope & (1 << j) else old_max[j]
               for j in range(ENCODER_COUNT)]
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
    anchor_valid, anchor_abs, anchor_raw = multiturn_anchor_fields(payload)
    for joint_idx in range(ENCODER_COUNT):
        if changed_mask & (1 << joint_idx):
            results[joint_idx] = None
    for motor_idx in range(MOTOR_COUNT):
        if invalid_motor_mask & (1 << motor_idx):
            anchor_abs[motor_idx] = 0
            anchor_raw[motor_idx] = 0
    payload["joint_calib_results"] = results
    payload["motor_abs_limit_valid_bitmap"] = format_mask(
        limit_valid & ~invalid_motor_mask
    )
    payload["multiturn_anchor_valid_bitmap"] = format_mask(
        anchor_valid & ~invalid_motor_mask
    )
    payload["multiturn_anchor_logical_abs"] = anchor_abs
    payload["multiturn_anchor_raw"] = anchor_raw

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
    if identity is not None:
        payload.update(identity)
    # 修复历史 bundle 缺少 hand_profile 的存量文件：下次 Manual 保存即自愈。
    payload.setdefault("hand_profile", HAND_PROFILE)
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
    if not Path(path).exists():
        return f"no calibration snapshot at {Path(path).name}"
    with snapshot_write_transaction():
        return _invalidate_calibration_snapshot_scope_locked(
            path,
            motor_mask=motor_mask,
            joint_mask=joint_mask,
            reason=reason,
        )


def _invalidate_calibration_snapshot_scope_locked(
    path: Path | str,
    *,
    motor_mask: int,
    joint_mask: int,
    reason: str,
) -> str:
    """在快照写事务内失效指定范围。"""
    target = Path(path)
    if not target.exists():
        return "no Calib Load snapshot"

    motor_mask = int(motor_mask) & VALID_MOTOR_MASK
    joint_mask = int(joint_mask) & _JOINT_MASK
    payload = read_snapshot(target)
    results = _normalized_list(
        payload.get("joint_calib_results"), ENCODER_COUNT, None
    )
    valid = parse_mask(payload.get("motor_abs_limit_valid_bitmap"))
    anchor_valid, _anchor_abs, _anchor_raw = multiturn_anchor_fields(payload)
    # 重复 Release / 重复失效同一范围时不要反复重写并产生空备份：范围已经
    # 没有可失效的数据就原样返回，避免把“已失效”状态再覆盖一次。
    scope_has_data = (
        bool(valid & motor_mask)
        or bool(anchor_valid & motor_mask)
        or any(
            isinstance(results[joint_idx], dict)
            and bool(results[joint_idx].get("success", False))
            for joint_idx in range(ENCODER_COUNT)
            if joint_mask & (1 << joint_idx)
        )
    )
    if not scope_has_data:
        return "scope already invalid; snapshot not rewritten"
    reason_tag = str(reason).strip().lower() or "coordinate_reset"
    stamp = time.strftime("%Y%m%d_%H%M%S")
    archive = target.parent / "backups" / f"{target.stem}.before_scoped_{reason_tag}_{stamp}{target.suffix}"
    archive.parent.mkdir(parents=True, exist_ok=True)

    shutil.copy2(target, archive)
    mins = _normalized_list(payload.get("motor_abs_limit_min"), MOTOR_COUNT, 0)
    maxs = _normalized_list(payload.get("motor_abs_limit_max"), MOTOR_COUNT, 0)
    anchor_valid, anchor_abs, anchor_raw = multiturn_anchor_fields(payload)

    for joint_idx in range(ENCODER_COUNT):
        if joint_mask & (1 << joint_idx):
            results[joint_idx] = None
    for motor_idx in range(MOTOR_COUNT):
        if motor_mask & (1 << motor_idx):
            mins[motor_idx] = 0
            maxs[motor_idx] = 0
            anchor_abs[motor_idx] = 0
            anchor_raw[motor_idx] = 0

    payload["joint_calib_results"] = results
    payload["motor_abs_limit_min"] = mins
    payload["motor_abs_limit_max"] = maxs
    payload["motor_abs_limit_valid_bitmap"] = format_mask(valid & ~motor_mask)
    payload["multiturn_anchor_valid_bitmap"] = format_mask(anchor_valid & ~motor_mask)
    payload["multiturn_anchor_logical_abs"] = anchor_abs
    payload["multiturn_anchor_raw"] = anchor_raw

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
        if "device_id" in current and any(previous.get(key) != current.get(key)
                for key in ("device_id", "urdf_repo_path")):
            raise CalibrationSnapshotError("existing Auto snapshot identity mismatch")
        merged = json.loads(json.dumps(previous, ensure_ascii=False))
    else:
        merged = _empty_v6_snapshot()

    anchor_valid, anchor_abs, anchor_raw = multiturn_anchor_fields(merged)
    merged["multiturn_anchor_valid_bitmap"] = format_mask(anchor_valid)
    merged["multiturn_anchor_logical_abs"] = anchor_abs
    merged["multiturn_anchor_raw"] = anchor_raw

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
        "device_id",
        "urdf_repo_path",
        "product_fingerprint",
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
