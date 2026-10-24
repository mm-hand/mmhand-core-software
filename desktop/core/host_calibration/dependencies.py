"""dependencies.py - host 标定的 AA 依赖解析。

职责：解析单腱关节所需的 AA 结果，并判断 host 快照结果是否仍可用。
线程：标定工作线程与快照读取路径调用；不创建线程。
输入：关节掩码、host 标定快照、当前 HandModel 与 URDF 配置 CRC。
输出：缺失 AA 掩码及校验后的完整 AA 标定结果。
状态所有权：只读取输入快照与设备状态；不写入快照、硬件或控制器。
禁止依赖：禁止串口收发、运动命令、UI 控件或 P4 执行细节。

A single-tendon joint (J09..J11, J13..J15) is calibrated relative to its
finger's AA joint (J08 or J12).  The AA result needed here is the host-side
mapping metadata produced by an earlier calibration.  P4 only stores the
execution subset (LUT/limits/auto raw); the host snapshot keeps the complete
result including the host-only ``manualSpanCounts`` field.
"""
from __future__ import annotations

from copy import deepcopy

from ..calibration_snapshot import joint_calib_result_has_valid_tendon_lut, parse_mask
from ..protocol import ENCODER_COUNT
from .config import finger_aa_joint, joint_motors
from .math import forward_encoder_delta, manual_entry_crc


def _joint_mask_bits(joint_mask: int):
    mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
    return [joint for joint in range(ENCODER_COUNT) if mask & (1 << joint)]


def required_aa_joint_mask(joint_mask: int) -> int:
    """AA joints needed by selected single-tendon joints but not selected themselves."""
    selected = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
    required = 0
    for joint in _joint_mask_bits(selected):
        aa = finger_aa_joint(joint)
        if aa is not None and not selected & (1 << aa):
            required |= 1 << aa
    return required


def usable_snapshot_aa_result(
    snapshot, joint: int, state, urdf_config_crc: int,
) -> dict | None:
    """Return a complete, still-compatible AA result from the host snapshot.

    The result is rejected unless it belongs to the current hand/model, was not
    invalidated afterwards, and its manual-reference span/CRC still matches the
    endpoints currently loaded on the device.
    """
    if not isinstance(snapshot, dict):
        return None
    joint = int(joint)
    results = snapshot.get("joint_calib_results") or []
    result = results[joint] if 0 <= joint < len(results) else None
    if not isinstance(result, dict) or not bool(result.get("success", False)):
        return None
    if int(result.get("mappingVersion", 0) or 0) != 4:
        return None
    if int(result.get("urdfConfigCrc", -1) or -1) != int(urdf_config_crc):
        return None
    if not joint_calib_result_has_valid_tendon_lut(result, joint):
        return None

    # The snapshot must still be the active one for this joint and its motors.
    active_joint_mask = parse_mask(snapshot.get("active_joint_mask"))
    motor_mask = sum(1 << motor for motor in joint_motors(joint))
    valid_limit_mask = parse_mask(snapshot.get("motor_abs_limit_valid_bitmap"))
    if not active_joint_mask & (1 << joint) or (valid_limit_mask & motor_mask) != motor_mask:
        return None

    # Identity guard: never reuse a result from another hand/model generation.
    state_device = ""
    if bool(state.has_product_fingerprint):
        state_device = str((getattr(state, "product_fingerprint", {}) or {}).get("device_id", ""))
    snapshot_device = str(snapshot.get("device_id", "") or "")
    if state_device and snapshot_device and state_device != snapshot_device:
        return None

    # Manual endpoints must still describe the same reference.
    if not int(state.manual_reference_valid_bitmap) & (1 << joint):
        return None
    try:
        low = int(state.manual_reference_min_raw[joint])
        high = int(state.manual_reference_max_raw[joint])
    except (AttributeError, IndexError, TypeError, ValueError):
        return None
    expected_crc = manual_entry_crc(joint, {"minRaw": low, "maxRaw": high})
    if int(result.get("manualReferenceCrc", 0) or 0) != expected_crc:
        return None
    expected_span = forward_encoder_delta(low, high)
    if not 0 < expected_span < 16384:
        return None
    if int(result.get("manualSpanCounts", 0) or 0) != expected_span:
        return None
    return deepcopy(result)


def missing_aa_joint_mask(
    joint_mask: int, snapshot, state, urdf_config_crc: int,
) -> int:
    """AA dependencies without a usable full host result (need re-calibration)."""
    missing = 0
    for aa in _joint_mask_bits(required_aa_joint_mask(joint_mask)):
        if usable_snapshot_aa_result(snapshot, aa, state, urdf_config_crc) is None:
            missing |= 1 << aa
    return missing
