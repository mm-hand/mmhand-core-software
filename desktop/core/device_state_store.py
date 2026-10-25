"""device_state_store.py - 上位机运行期设备状态持久化。

职责：保存27台舵机的原生ABS恢复记录，并在恢复前
校验反馈坐标契约、硬件拓扑和文件完整性；其余指纹只保留作诊断。
线程：由独立持久化线程和显式坐标事务调用；标定包统一序列化写入。
输入：P4已确认的原生ABS、产品身份和电机有效范围。
输出：可跨 P4/舵机掉电使用的运行锚点 JSON。
状态所有权：只通过 CalibrationSnapshotStore 的原子事务读写唯一标定包。
禁止依赖：禁止依赖 GUI、串口对象或控制状态机；本模块只负责持久化和校验。
"""

from __future__ import annotations

import re
import time
from pathlib import Path
from typing import Iterable

from .protocol import MOTOR_COUNT, VALID_MOTOR_MASK


class DeviceStateStoreError(ValueError):
    """运行锚点不存在、损坏或与当前产品身份不匹配。"""


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

    原生ABS恢复只依赖舵机通道拓扑和坐标语义。固件源码、控制参数、
    URDF 或整份标定快照发生变化，不会改变同一 Mxx 的 坐标基准，
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


class BundleMultiTurnStore:
    """The active calibration bundle is the sole native-ABS recovery snapshot."""

    def __init__(self, path: Path | str):
        self.path = Path(path)

    def _bundle(self) -> dict:
        from .calibration_snapshot import read_snapshot
        return read_snapshot(self.path)

    def load(self, *, identity: dict | None = None) -> dict:
        from .calibration_snapshot import format_mask, multiturn_anchor_fields
        payload = self._bundle()
        valid, logical_abs, _raw = multiturn_anchor_fields(payload)
        saved_identity = {
            "device_id": payload.get("device_id", ""),
            "urdf_repo_path": payload.get("urdf_repo_path", ""),
            "product_fingerprint": payload.get("product_fingerprint", {}),
        }
        if identity is not None and not multiturn_identity_compatible(saved_identity, identity):
            raise DeviceStateStoreError("runtime anchors belong to another product/calibration")
        return {
            "identity": saved_identity,
            "saved_at_host_time": payload.get("multiturn_anchor_saved_at_host_time", 0.0),
            "valid_motor_mask": format_mask(valid),
            "logical_abs": logical_abs,
            "raw": [value % 4096 for value in logical_abs],
        }

    def save(self, *, identity: dict, valid_motor_mask: int,
             logical_abs: Iterable[int]) -> dict:
        from .calibration_snapshot import update_snapshot_multiturn_anchors
        if not multiturn_identity_compatible(identity, identity):
            raise DeviceStateStoreError("native ABS requires a valid product identity")
        valid_motor_mask = int(valid_motor_mask) & VALID_MOTOR_MASK
        if not valid_motor_mask:
            raise DeviceStateStoreError("native ABS snapshot requires trusted motors")
        absolute = _fixed_int_list(logical_abs, MOTOR_COUNT)
        from .calibration_snapshot import (_empty_v7_snapshot, atomic_write_snapshot,
                                           format_mask, snapshot_write_transaction)
        # Direct control on a never-calibrated device still needs one native-ABS bundle.
        with snapshot_write_transaction():
            if not self.path.exists():
                payload = _empty_v7_snapshot()
                payload.update(identity)
                payload.update(
                    scoped_native_abs_motor_mask=format_mask(valid_motor_mask),
                    multiturn_anchor_valid_bitmap=format_mask(valid_motor_mask),
                    multiturn_anchor_logical_abs=absolute,
                    multiturn_anchor_saved_at_host_time=time.time(),
                )
                atomic_write_snapshot(self.path, payload)
                return self.load()
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
            logical_abs=absolute,
            raw=[value % 4096 for value in absolute],
        )
        return self.load()

    def invalidate_motor_mask(self, motor_mask: int, *, reason: str) -> dict | None:
        from .calibration_snapshot import invalidate_snapshot_multiturn_anchors
        return invalidate_snapshot_multiturn_anchors(self.path, motor_mask=motor_mask, reason=reason)
