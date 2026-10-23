"""device_state_store.py - 上位机运行期设备状态持久化。

职责：原子保存 27 台舵机的 logical ABS 与同帧单圈 raw 锚点，并在恢复前
校验反馈坐标契约、硬件拓扑和文件完整性；其余指纹只保留作诊断。
线程：由 HandController 更新线程调用；模块内锁保证强制保存与退出保存不竞争。
输入：P4 已确认可信的在线舵机反馈、产品指纹、当前 V6 标定快照摘要。
输出：可跨 P4/舵机掉电使用的运行锚点 JSON。
状态所有权：文件锁和原子替换由 MultiTurnRuntimeStore 独占管理。
禁止依赖：禁止依赖 GUI、串口对象或控制状态机；本模块只负责持久化和校验。
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import threading
import time
import uuid
from pathlib import Path
from typing import Iterable

from .protocol import MOTOR_COUNT, VALID_MOTOR_MASK


MULTITURN_RUNTIME_FORMAT = "mmhand_multiturn_runtime"
MULTITURN_RUNTIME_SCHEMA_VERSION = 1


class DeviceStateStoreError(ValueError):
    """运行锚点不存在、损坏或与当前产品身份不匹配。"""


def circular_raw_distance(a: int, b: int, period: int = 4096) -> int:
    """计算单圈编码值的最短环形距离，正确处理 4095 -> 0 跨圈。"""
    delta = abs((int(a) % period) - (int(b) % period))
    return min(delta, period - delta)


def nearest_logical_abs(
    saved_logical_abs: int,
    saved_raw: int,
    current_raw: int,
    period: int = 4096,
) -> int:
    """把当前 raw 展开到距离最近一次 logical ABS 最近的相邻圈。

    SavedRaw 只能表达单圈相位，必须与同帧 saved logical ABS 配对使用。
    差值先折叠到 [-period/2, period/2)，因此 4095 -> 0 能按连续跨圈恢复。
    """
    half = int(period) // 2
    delta = ((int(current_raw) - int(saved_raw) + half) % int(period)) - half
    return int(saved_logical_abs) + delta


def _fixed_int_list(values: Iterable[int], count: int) -> list[int]:
    result = [int(value) for value in list(values)[:count]]
    result.extend([0] * (count - len(result)))
    return result


def canonical_product_fingerprint(value: dict) -> dict:
    """只保留协议定义的稳定字段，避免字典顺序和临时字段改变设备身份。"""
    source = dict(value or {})
    keys = (
        "schema_version",
        "feedback_mapping_version",
        "manual_scaled_mapping_version",
        "control_model_crc",
        "visual_model_crc",
        "hardware_map_crc",
        "parameter_crc",
        "firmware_source_crc",
        "structure_commit_crc",
    )
    return {key: int(source.get(key, 0) or 0) for key in keys}


def multiturn_identity_compatible(saved: dict, current: dict) -> bool:
    """判断两份身份是否可共用多圈锚点。

    logical ABS 的展开只依赖舵机通道拓扑和反馈坐标语义。固件源码、控制参数、
    URDF 或整份标定快照发生变化，不会改变同一 Mxx 的 raw/logical ABS 关系，
    因此不能因为这些诊断指纹变化而丢弃全部 27 路运行锚点。
    """
    saved, current = dict(saved or {}), dict(current or {})
    device_id = str(current.get("device_id", ""))
    if (not re.fullmatch(r"[0-9A-F]{16}", device_id)
            or int(device_id, 16) == 0
            or saved.get("device_id") != device_id
            or not current.get("urdf_repo_path")
            or saved.get("urdf_repo_path") != current.get("urdf_repo_path")):
        return False
    saved_fp = canonical_product_fingerprint(
        dict(saved or {}).get("product_fingerprint", {})
    )
    current_fp = canonical_product_fingerprint(
        dict(current or {}).get("product_fingerprint", {})
    )
    compatibility_keys = (
        "schema_version",
        "feedback_mapping_version",
        "hardware_map_crc",
    )
    return (current_fp["schema_version"] == 2
            and all(saved_fp[key] == current_fp[key] for key in compatibility_keys))


def _canonical_bytes(payload: dict) -> bytes:
    body = dict(payload)
    body.pop("integrity", None)
    return json.dumps(
        body, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")


def _digest(payload: dict) -> str:
    return hashlib.sha256(_canonical_bytes(payload)).hexdigest()


class MultiTurnRuntimeStore:
    """管理单个设备的多圈运行锚点，写入采用临时文件 + fsync + 原子替换。"""

    def __init__(self, path: Path | str):
        self.path = Path(path)
        self._lock = threading.Lock()

    def load(self, *, identity: dict | None = None) -> dict:
        """读取并验证锚点；提供 identity 时要求多圈坐标契约兼容。"""
        with self._lock:
            try:
                payload = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise DeviceStateStoreError(f"cannot read runtime anchors: {exc}") from exc
        if payload.get("format") != MULTITURN_RUNTIME_FORMAT:
            raise DeviceStateStoreError("unsupported runtime anchor format")
        if int(payload.get("schema_version", 0) or 0) != MULTITURN_RUNTIME_SCHEMA_VERSION:
            raise DeviceStateStoreError("unsupported runtime anchor schema")
        integrity = payload.get("integrity")
        if not isinstance(integrity, dict) or integrity.get("algorithm") != "sha256":
            raise DeviceStateStoreError("runtime anchor SHA-256 metadata is missing")
        if str(integrity.get("digest", "")).lower() != _digest(payload):
            raise DeviceStateStoreError("runtime anchor SHA-256 mismatch")
        logical_abs = payload.get("logical_abs")
        raw = payload.get("raw")
        if not isinstance(logical_abs, list) or len(logical_abs) != MOTOR_COUNT:
            raise DeviceStateStoreError("logical_abs must contain 27 entries")
        if not isinstance(raw, list) or len(raw) != MOTOR_COUNT:
            raise DeviceStateStoreError("raw must contain 27 entries")
        if identity is not None and not multiturn_identity_compatible(
            payload.get("identity", {}), identity
        ):
            raise DeviceStateStoreError("runtime anchors belong to another product/calibration")
        return payload

    def save(
        self,
        *,
        identity: dict,
        valid_motor_mask: int,
        logical_abs: Iterable[int],
        raw: Iterable[int],
        p4_time_ms: int,
    ) -> dict:
        """保存完整 27 路数组；valid_motor_mask 决定哪些条目可参与恢复。"""
        payload = {
            "format": MULTITURN_RUNTIME_FORMAT,
            "schema_version": MULTITURN_RUNTIME_SCHEMA_VERSION,
            "identity": dict(identity),
            "saved_at_host_time": time.time(),
            "p4_time_ms": int(p4_time_ms) & 0xFFFFFFFF,
            "valid_motor_mask": f"0x{int(valid_motor_mask) & VALID_MOTOR_MASK:08X}",
            "logical_abs": _fixed_int_list(logical_abs, MOTOR_COUNT),
            "raw": [value % 4096 for value in _fixed_int_list(raw, MOTOR_COUNT)],
        }
        payload["integrity"] = {"algorithm": "sha256", "digest": _digest(payload)}
        target = self.path
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_name(f".{target.name}.{uuid.uuid4().hex}.tmp")
        backup = target.with_suffix(target.suffix + ".bak")
        with self._lock:
            try:
                with temp.open("w", encoding="utf-8", newline="\n") as handle:
                    json.dump(payload, handle, indent=2, ensure_ascii=False)
                    handle.write("\n")
                    handle.flush()
                    os.fsync(handle.fileno())
                # 临时文件也必须能通过完整校验，之后才替换正式文件。
                check = json.loads(temp.read_text(encoding="utf-8"))
                if str(check["integrity"]["digest"]).lower() != _digest(check):
                    raise DeviceStateStoreError("runtime anchor write verification failed")
                if target.exists():
                    shutil.copy2(target, backup)
                os.replace(temp, target)
            finally:
                if temp.exists():
                    temp.unlink()
        return payload

    def invalidate_motor_mask(self, motor_mask: int, *, reason: str) -> dict | None:
        """原子清除指定舵机锚点；标定硬件归零前调用，禁止旧圈数回灌。"""
        mask = int(motor_mask) & VALID_MOTOR_MASK
        if mask == 0 or not self.path.exists():
            return None
        payload = self.load()
        valid_mask = int(str(payload.get("valid_motor_mask", "0")), 0)
        logical_abs = list(payload.get("logical_abs", []))
        raw = list(payload.get("raw", []))
        for motor_idx in range(MOTOR_COUNT):
            if mask & (1 << motor_idx):
                logical_abs[motor_idx] = 0
                raw[motor_idx] = 0
        payload["saved_at_host_time"] = time.time()
        payload["valid_motor_mask"] = f"0x{(valid_mask & ~mask):08X}"
        payload["logical_abs"] = _fixed_int_list(logical_abs, MOTOR_COUNT)
        payload["raw"] = [value % 4096 for value in _fixed_int_list(raw, MOTOR_COUNT)]
        payload["last_invalidation"] = {
            "motor_mask": f"0x{mask:08X}",
            "reason": str(reason),
            "host_time": time.time(),
        }
        payload["integrity"] = {"algorithm": "sha256", "digest": _digest(payload)}
        # 圈数归零与失效原因必须作为同一个事务落盘，避免中途留下无说明的半状态。
        return self._write_payload(payload)

    def _write_payload(self, payload: dict) -> dict:
        """写入已经组装好的 payload，供附加事务元数据时复用。"""
        target = self.path
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_name(f".{target.name}.{uuid.uuid4().hex}.tmp")
        backup = target.with_suffix(target.suffix + ".bak")
        with self._lock:
            try:
                with temp.open("w", encoding="utf-8", newline="\n") as handle:
                    json.dump(payload, handle, indent=2, ensure_ascii=False)
                    handle.write("\n")
                    handle.flush()
                    os.fsync(handle.fileno())
                if target.exists():
                    shutil.copy2(target, backup)
                os.replace(temp, target)
            finally:
                if temp.exists():
                    temp.unlink()
        return payload

class BundleMultiTurnStore:
    """Runtime anchors stored inside the active calibration bundle.

    The bundle already owns the calibration identity and the anchor arrays.
    This adapter keeps the old MultiTurnRuntimeStore call surface while
    eliminating the separate multiturn_runtime.json file.
    """

    def __init__(self, path: Path | str):
        self.path = Path(path)

    def _bundle(self) -> dict:
        from .calibration_snapshot import read_snapshot
        return read_snapshot(self.path)

    def load(self, *, identity: dict | None = None) -> dict:
        from .calibration_snapshot import format_mask, multiturn_anchor_fields
        payload = self._bundle()
        valid, logical_abs, raw = multiturn_anchor_fields(payload)
        saved_identity = {
            "device_id": payload.get("device_id", ""),
            "urdf_repo_path": payload.get("urdf_repo_path", ""),
            "product_fingerprint": payload.get("product_fingerprint", {}),
        }
        if identity is not None and not multiturn_identity_compatible(saved_identity, identity):
            raise DeviceStateStoreError("runtime anchors belong to another product/calibration")
        if valid == 0:
            raise DeviceStateStoreError("runtime anchors are empty")
        return {
            "format": MULTITURN_RUNTIME_FORMAT,
            "schema_version": MULTITURN_RUNTIME_SCHEMA_VERSION,
            "identity": saved_identity,
            "saved_at_host_time": payload.get("multiturn_anchor_saved_at_host_time", 0.0),
            "valid_motor_mask": format_mask(valid),
            "logical_abs": logical_abs,
            "raw": raw,
        }

    def save(self, *, identity: dict, valid_motor_mask: int,
             logical_abs: Iterable[int], raw: Iterable[int], p4_time_ms: int) -> dict:
        from .calibration_snapshot import update_snapshot_multiturn_anchors
        payload = self._bundle()
        saved_identity = {
            "device_id": payload.get("device_id", ""),
            "urdf_repo_path": payload.get("urdf_repo_path", ""),
            "product_fingerprint": payload.get("product_fingerprint", {}),
        }
        if not multiturn_identity_compatible(saved_identity, identity):
            raise DeviceStateStoreError("runtime anchors belong to another product/calibration")
        update_snapshot_multiturn_anchors(
            self.path,
            motor_mask=int(valid_motor_mask) & VALID_MOTOR_MASK,
            logical_abs=_fixed_int_list(logical_abs, MOTOR_COUNT),
            raw=[value % 4096 for value in _fixed_int_list(raw, MOTOR_COUNT)],
        )
        return self.load()

    def invalidate_motor_mask(self, motor_mask: int, *, reason: str) -> dict | None:
        from .calibration_snapshot import invalidate_snapshot_multiturn_anchors
        return invalidate_snapshot_multiturn_anchors(self.path, motor_mask=motor_mask, reason=reason)
