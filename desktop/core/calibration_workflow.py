"""calibration_workflow.py - MMHand 标定应用事务服务。

职责：串行编排标定准备、启动恢复、快照持久化和多圈锚点登记。
线程：所有阻塞事务在单工作线程执行；状态观察入口必须保持非阻塞。
输入：HandModel 快照、标定范围和经用户确认的标定文件。
输出：与 Qt 无关的工作流事件，以及写入本地的 V6 标定快照。
状态所有权：本服务唯一拥有标定事务 epoch、待保存范围和结果基线。
禁止依赖：禁止依赖 Qt/UI、页面控件或直接操作串口对象。
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import json
import threading
import time
from typing import Callable, Optional

from desktop.paths import CALIB_LOAD_SNAPSHOT_PATH

from .angle_mapping import urdf_manual_scaled_mapping_metadata
from .calibration_snapshot import (
    CalibrationSnapshotError,
    CalibrationSnapshotStore,
    joint_calib_result_has_valid_tendon_lut,
    motor_mask_for_joint_mask,
    parse_mask,
    validate_snapshot_for_load as validate_calibration_snapshot_for_load,
)
from .calibration_diagnostics import format_calibration_warning
from .data_models import HandModel
from .motor_mapping import MOTOR_TO_JOINT_INDEX
from .protocol import (
    DUAL_TENDON_JOINT_INDICES,
    ENCODER_COUNT,
    HAND_PROFILE,
    MOTOR_COUNT,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK,
    VALID_MOTOR_MASK,
    ControlMode,
    manual_encoder_reference_entry_crc,
)
from .session_log import log_session


_JOINT_MASK = (1 << ENCODER_COUNT) - 1
_DUAL_JOINT_MASK = sum(1 << joint for joint in DUAL_TENDON_JOINT_INDICES)
_CALIB_STATUSES = {"PENDING", "RUNNING", "SUCCESS", "WARN", "FAILED"}


@dataclass(frozen=True)
class CalibrationWorkflowEvent:
    epoch: int
    phase: str
    ok: bool
    message: str
    joint_mask: int = 0
    motor_mask: int = 0
    failed_motor_mask: int = 0
    details: dict = field(default_factory=dict)


def joint_calib_result_signature(item: Optional[dict]) -> Optional[str]:
    if not isinstance(item, dict) or not bool(item.get("success", False)):
        return None
    return json.dumps(item, sort_keys=True, ensure_ascii=False)


def _format_joints(indices) -> str:
    return ",".join(f"J{int(idx):02d}" for idx in indices)


def _format_motors(indices) -> str:
    return ",".join(f"M{int(idx):02d}" for idx in indices)


class CalibrationWorkflowService:
    """标定事务唯一编排者；所有文件写入和阻塞设备调用均串行执行。"""

    def __init__(self, controller, snapshot_path=CALIB_LOAD_SNAPSHOT_PATH):
        self._controller = controller
        self._store = CalibrationSnapshotStore(snapshot_path)
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="mmhand-calibration",
        )
        self._lock = threading.RLock()
        self._callbacks: list[Callable[[CalibrationWorkflowEvent], None]] = []
        self._epoch = 0
        self._operation = ""
        self._pending_joint_mask = 0
        self._pending_motor_mask = 0
        self._calibration_phase_masks: list[int] = []
        self._calibration_phase_index = 0
        self._calibration_motor_mask = 0
        self._calibration_phase_records: list[dict] = []
        self._phase_waiting_for_start = False
        self._result_baselines: dict[int, Optional[str]] = {}
        self._last_snapshot_signature = ""
        self._last_observation_signature = ""
        self._last_calib_status = "IDLE"
        self._last_notice = ""

    @property
    def snapshot_path(self):
        return self._store.path

    def register_callback(
        self, callback: Callable[[CalibrationWorkflowEvent], None]
    ) -> None:
        with self._lock:
            if callback not in self._callbacks:
                self._callbacks.append(callback)

    def unregister_callback(
        self, callback: Callable[[CalibrationWorkflowEvent], None]
    ) -> None:
        with self._lock:
            self._callbacks = [item for item in self._callbacks if item != callback]

    def cancel(self) -> None:
        with self._lock:
            self._epoch += 1
            self._operation = ""
            self._pending_joint_mask = 0
            self._pending_motor_mask = 0
            self._calibration_phase_masks = []
            self._calibration_phase_index = 0
            self._calibration_motor_mask = 0
            self._calibration_phase_records = []
            self._phase_waiting_for_start = False
            self._result_baselines = {}
            self._last_snapshot_signature = ""
            self._last_observation_signature = ""

    def _next_epoch(self, operation: str) -> Optional[int]:
        with self._lock:
            if self._operation:
                return None
            self._epoch += 1
            self._operation = operation
            self._pending_joint_mask = 0
            self._pending_motor_mask = 0
            self._calibration_phase_masks = []
            self._calibration_phase_index = 0
            self._calibration_motor_mask = 0
            self._calibration_phase_records = []
            self._phase_waiting_for_start = False
            self._result_baselines = {}
            self._last_snapshot_signature = ""
            self._last_observation_signature = ""
            return self._epoch

    def _finish_operation(self, epoch: int) -> None:
        with self._lock:
            if epoch == self._epoch:
                self._operation = ""

    def _clear_calibration_tracking(self, epoch: int) -> None:
        with self._lock:
            if epoch != self._epoch:
                return
            self._pending_joint_mask = 0
            self._pending_motor_mask = 0
            self._calibration_phase_masks = []
            self._calibration_phase_index = 0
            self._calibration_motor_mask = 0
            self._calibration_phase_records = []
            self._phase_waiting_for_start = False
            self._result_baselines = {}
            self._operation = ""

    def _is_current(self, epoch: int) -> bool:
        with self._lock:
            return epoch == self._epoch

    def _emit(self, event: CalibrationWorkflowEvent) -> None:
        with self._lock:
            if event.epoch != self._epoch:
                return
            callbacks = list(self._callbacks)
        log_session(
            "CALIB_WORKFLOW",
            f"epoch={event.epoch} phase={event.phase} ok={int(event.ok)} {event.message}",
        )
        for callback in callbacks:
            try:
                callback(event)
            except Exception:
                pass

    def _start_calibration_phase(
        self,
        epoch: int,
        baseline_results: list,
        *,
        reset_mask: int = 0,
    ) -> bool:
        with self._lock:
            if epoch != self._epoch:
                return False
            joint_mask = self._calibration_phase_masks[
                self._calibration_phase_index
            ]
            phase_count = len(self._calibration_phase_masks)
            phase_index = self._calibration_phase_index
            motor_mask = (
                motor_mask_for_joint_mask(joint_mask)
                & self._calibration_motor_mask
                & VALID_MOTOR_MASK
            )
        self._controller.set_control_mode(ControlMode.JOINT_ANGLE)
        if not motor_mask or not self._controller.set_active_scope(
            motor_mask, joint_mask, True
        ):
            self._emit(
                CalibrationWorkflowEvent(
                    epoch,
                    "prepare",
                    False,
                    "active scope apply failed for calibration phase",
                    joint_mask,
                    reset_mask,
                    motor_mask,
                )
            )
            return False
        with self._lock:
            if epoch != self._epoch:
                return False
            self._pending_joint_mask = joint_mask
            self._pending_motor_mask = motor_mask
            self._result_baselines = {
                joint_idx: joint_calib_result_signature(
                    baseline_results[joint_idx]
                    if joint_idx < len(baseline_results)
                    else None
                )
                for joint_idx in range(ENCODER_COUNT)
                if joint_mask & (1 << joint_idx)
            }
            self._last_snapshot_signature = ""
            self._last_observation_signature = ""
        if not self._controller.request_firmware_calibration():
            self._emit(
                CalibrationWorkflowEvent(
                    epoch,
                    "prepare",
                    False,
                    "firmware calibration command was not queued",
                    joint_mask,
                    reset_mask,
                    motor_mask,
                )
            )
            return False
        self._emit(
            CalibrationWorkflowEvent(
                epoch,
                "prepare",
                True,
                (
                    "turn counters reset; " if reset_mask else ""
                ) + f"calibration phase {phase_index + 1}/{phase_count} started",
                joint_mask,
                reset_mask,
                details={"phase_index": phase_index, "phase_count": phase_count},
            )
        )
        return True

    def _advance_calibration_phase(
        self, epoch: int, state: HandModel
    ) -> bool:
        with self._lock:
            if (
                epoch != self._epoch
                or self._calibration_phase_index + 1
                >= len(self._calibration_phase_masks)
            ):
                return False
            self._calibration_phase_index += 1
            self._phase_waiting_for_start = True
        results = list(getattr(state, "joint_calib_results", []) or [])
        if self._start_calibration_phase(epoch, results):
            return True
        self._clear_calibration_tracking(epoch)
        return False

    def _complete_calibration_phase(
        self, epoch: int, state: HandModel, status: str
    ) -> None:
        """Record one terminal phase, continue immediately, then emit one batch result."""
        with self._lock:
            if epoch != self._epoch:
                return
            phase_index = self._calibration_phase_index
            phase_count = len(self._calibration_phase_masks)
            joint_mask = self._pending_joint_mask
            motor_mask = self._pending_motor_mask
        warning_mask = int(
            getattr(state, "calibration_warning_bitmap", 0) or 0
        ) & joint_mask
        warning_summary = (
            format_calibration_warning(state, warning_mask) if warning_mask else None
        )
        record = {
            "phase_index": phase_index,
            "phase_count": phase_count,
            "status": status,
            "joint_mask": joint_mask,
            "motor_mask": motor_mask,
            "warning_mask": warning_mask,
            "warning_summary": (
                warning_summary if warning_summary and warning_summary.get("ready") else None
            ),
            "rope_init_status": int(getattr(state, "rope_init_status", 0) or 0),
            "rope_init_reason": int(getattr(state, "rope_init_reason", 0) or 0),
            "completed_joint_mask": int(
                getattr(state, "calib_diag_completed_joint_mask", 0) or 0
            ) & joint_mask,
            "failed_joint_mask": int(
                getattr(state, "calib_diag_failed_joint_mask", 0) or 0
            ) & joint_mask,
        }
        with self._lock:
            if epoch != self._epoch:
                return
            self._calibration_phase_records.append(record)
            records = list(self._calibration_phase_records)
            has_next = phase_index + 1 < phase_count
        self._emit(
            CalibrationWorkflowEvent(
                epoch,
                "phase_complete",
                status != "FAILED",
                f"calibration phase {phase_index + 1}/{phase_count} ended with {status}",
                joint_mask,
                motor_mask,
                details={**record, "has_next_phase": has_next},
            )
        )
        if status != "FAILED" and has_next and self._advance_calibration_phase(
            epoch, state
        ):
            return
        ok = status != "FAILED" and not has_next
        self._emit(
            CalibrationWorkflowEvent(
                epoch,
                "calibration_complete",
                ok,
                (
                    "all calibration phases completed"
                    if ok
                    else "calibration stopped before all phases completed"
                ),
                joint_mask,
                motor_mask,
                details={"records": records, "terminal_status": status},
            )
        )
        self._clear_calibration_tracking(epoch)

    def begin_calibration(
        self,
        *,
        joint_mask: int,
        motor_mask: int,
        reset_motor_mask: int,
        baseline_state: HandModel,
    ) -> bool:
        epoch = self._next_epoch("calibration")
        if epoch is None:
            return False
        joint_mask = int(joint_mask) & _JOINT_MASK
        motor_mask = int(motor_mask) & VALID_MOTOR_MASK
        reset_motor_mask = int(reset_motor_mask) & VALID_MOTOR_MASK
        baseline_results = list(
            getattr(baseline_state, "joint_calib_results", []) or []
        )
        dual_mask = joint_mask & _DUAL_JOINT_MASK
        single_mask = joint_mask & ~_DUAL_JOINT_MASK
        phase_masks = [mask for mask in (dual_mask, single_mask) if mask]
        with self._lock:
            self._calibration_phase_masks = phase_masks
            self._calibration_phase_index = 0
            self._calibration_motor_mask = motor_mask

        def worker() -> None:
            reset_mask = 0
            failed_mask = reset_motor_mask
            started = False
            try:
                ok, message, reset_mask, failed_mask = (
                    self._controller.prepare_calibration_turn_zero(reset_motor_mask)
                )
                reset_mask &= VALID_MOTOR_MASK
                if reset_mask:
                    invalidated_joints = 0
                    for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX):
                        if reset_mask & (1 << motor_idx):
                            invalidated_joints |= 1 << int(mapped_joint)
                    self._store.invalidate_scope(
                        motor_mask=reset_mask,
                        joint_mask=invalidated_joints,
                        reason="calibration_turn_zero",
                    )
                if not self._is_current(epoch):
                    return
                if not ok:
                    self._emit(
                        CalibrationWorkflowEvent(
                            epoch, "prepare", False, str(message), joint_mask,
                            reset_mask, int(failed_mask) & VALID_MOTOR_MASK,
                        )
                    )
                    return
                started = self._start_calibration_phase(
                    epoch, baseline_results, reset_mask=reset_mask
                )
            except Exception as exc:
                self._emit(
                    CalibrationWorkflowEvent(
                        epoch,
                        "prepare",
                        False,
                        f"{type(exc).__name__}: {exc}",
                        joint_mask,
                        reset_mask,
                        int(failed_mask) & VALID_MOTOR_MASK,
                    )
                )
            finally:
                if not started:
                    self._finish_operation(epoch)

        self._executor.submit(worker)
        return True

    def observe_state(self, state: HandModel) -> None:
        with self._lock:
            pending_mask = self._pending_joint_mask
            epoch = self._epoch
        if pending_mask == 0:
            return
        status = str(getattr(state, "calib_status", "IDLE") or "IDLE")
        results = list(getattr(state, "joint_calib_results", []) or [])
        observation = json.dumps(
            {
                "status": status,
                "valid": int(getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0),
                "completed": int(getattr(state, "calib_diag_completed_joint_mask", 0) or 0),
                "failed": int(getattr(state, "calib_diag_failed_joint_mask", 0) or 0),
                "results": [
                    joint_calib_result_signature(results[idx])
                    if idx < len(results) and pending_mask & (1 << idx)
                    else None
                    for idx in range(ENCODER_COUNT)
                ],
            },
            sort_keys=True,
        )
        with self._lock:
            if epoch != self._epoch or observation == self._last_observation_signature:
                return
            self._last_observation_signature = observation
        self._executor.submit(self._save_observed_state, epoch, state)

    def _save_observed_state(self, epoch: int, state: HandModel) -> None:
        with self._lock:
            if epoch != self._epoch:
                return
            pending_mask = self._pending_joint_mask
            baselines = dict(self._result_baselines)
        status = str(getattr(state, "calib_status", "IDLE") or "IDLE")
        with self._lock:
            waiting_for_start = self._phase_waiting_for_start
        if waiting_for_start:
            if status not in {"PENDING", "RUNNING"}:
                return
            with self._lock:
                if epoch == self._epoch:
                    self._phase_waiting_for_start = False
        if status not in _CALIB_STATUSES:
            if status == "IDLE" and self._last_calib_status in _CALIB_STATUSES:
                with self._lock:
                    self._pending_joint_mask = 0
                    self._pending_motor_mask = 0
                    self._result_baselines = {}
                self._finish_operation(epoch)
            self._last_calib_status = status
            return
        self._last_calib_status = status
        results = list(getattr(state, "joint_calib_results", []) or [])
        incremental_mask = 0
        for joint_idx in range(ENCODER_COUNT):
            if not (pending_mask & (1 << joint_idx)):
                continue
            item = results[joint_idx] if joint_idx < len(results) else None
            signature = joint_calib_result_signature(item)
            if signature is not None and signature != baselines.get(joint_idx):
                incremental_mask |= 1 << joint_idx
        completed_mask = int(
            getattr(state, "calib_diag_completed_joint_mask", 0) or 0
        ) & _JOINT_MASK
        failed_mask = int(
            getattr(state, "calib_diag_failed_joint_mask", 0) or 0
        ) & _JOINT_MASK
        if status in {"PENDING", "RUNNING"}:
            save_mask = incremental_mask
        elif status == "FAILED":
            save_mask = pending_mask & ((completed_mask & ~failed_mask) | incremental_mask)
        else:
            save_mask = pending_mask
        if save_mask == 0:
            if status == "FAILED":
                self._complete_calibration_phase(epoch, state, status)
            return
        try:
            payload, active_motor_mask, signature = self._snapshot_payload(
                state, save_mask
            )
            if signature == self._last_snapshot_signature:
                if status in {"SUCCESS", "WARN", "FAILED"}:
                    self._complete_calibration_phase(epoch, state, status)
                return
            saved = self._store.merge_calibration(payload, save_mask)
            if not self._controller.request_multiturn_anchor_capture(
                active_motor_mask
            ):
                raise CalibrationSnapshotError(
                    "calibration saved, but multi-turn anchor capture was rejected"
                )
            anchors_saved, anchor_message = (
                self._controller.wait_for_multiturn_anchor_capture(
                    active_motor_mask
                )
            )
            if not anchors_saved:
                raise CalibrationSnapshotError(
                    "calibration saved, but multi-turn anchors are incomplete: "
                    + str(anchor_message)
                )
            self._last_snapshot_signature = signature
            with self._lock:
                for joint_idx in range(ENCODER_COUNT):
                    if save_mask & (1 << joint_idx):
                        item = results[joint_idx] if joint_idx < len(results) else None
                        self._result_baselines[joint_idx] = joint_calib_result_signature(item)
            self._emit(
                CalibrationWorkflowEvent(
                    epoch,
                    "snapshot_saved",
                    True,
                    f"saved {self._store.path.name}; {anchor_message}",
                    save_mask,
                    active_motor_mask,
                    details={"digest": saved.get("integrity", {}).get("digest", "")},
                )
            )
            if status in {"SUCCESS", "WARN", "FAILED"}:
                self._complete_calibration_phase(epoch, state, status)
        except (CalibrationSnapshotError, OSError, TypeError, ValueError) as exc:
            self._notice_once(
                CalibrationWorkflowEvent(
                    epoch,
                    "snapshot_error",
                    False,
                    str(exc),
                    save_mask,
                )
            )
            if status == "FAILED":
                self._complete_calibration_phase(epoch, state, status)
            elif status in {"SUCCESS", "WARN"}:
                # 终态保存异常不能永久占用工作流；保留 pending 数据，后续
                # 状态若补齐仍可重试，同时允许用户主动重新标定。
                self._emit(
                    CalibrationWorkflowEvent(
                        epoch,
                        "calibration_complete",
                        False,
                        f"calibration result could not be persisted: {exc}",
                        save_mask,
                        details={"records": list(self._calibration_phase_records)},
                    )
                )
                self._clear_calibration_tracking(epoch)

    def _snapshot_payload(self, state: HandModel, joint_mask: int):
        valid_bitmap = int(
            getattr(state, "motor_abs_limit_valid_bitmap", 0) or 0
        ) & VALID_MOTOR_MASK
        motor_mask = motor_mask_for_joint_mask(joint_mask) & VALID_MOTOR_MASK
        if motor_mask and (valid_bitmap & motor_mask) != motor_mask:
            missing = [
                i
                for i in range(MOTOR_COUNT)
                if motor_mask & (1 << i) and not valid_bitmap & (1 << i)
            ]
            raise CalibrationSnapshotError("waiting motor limits: " + _format_motors(missing))
        source_results = list(getattr(state, "joint_calib_results", []) or [])
        results: list[Optional[dict]] = [None] * ENCODER_COUNT
        for joint_idx in range(ENCODER_COUNT):
            if joint_mask & (1 << joint_idx) and joint_idx < len(source_results):
                item = source_results[joint_idx]
                if isinstance(item, dict) and bool(item.get("success", False)):
                    results[joint_idx] = dict(item)
        missing = [i for i in range(ENCODER_COUNT) if joint_mask & (1 << i) and results[i] is None]
        if missing:
            raise CalibrationSnapshotError("waiting joint results: " + _format_joints(missing))
        missing_lut = [
            i
            for i in range(ENCODER_COUNT)
            if joint_mask & (1 << i)
            and not joint_calib_result_has_valid_tendon_lut(results[i], i)
        ]
        if missing_lut:
            raise CalibrationSnapshotError("waiting tendon LUT: " + _format_joints(missing_lut))
        manual_mask = int(
            getattr(state, "manual_reference_valid_bitmap", 0) or 0
        ) & _JOINT_MASK
        manual_min = list(getattr(state, "manual_reference_min_raw", []) or [])
        manual_max = list(getattr(state, "manual_reference_max_raw", []) or [])
        if (manual_mask & joint_mask) != joint_mask:
            raise CalibrationSnapshotError(
                "Manual missing: " + _format_joints(
                    i
                    for i in range(ENCODER_COUNT)
                    if joint_mask & (1 << i) and not manual_mask & (1 << i)
                )
            )
        if len(manual_min) < ENCODER_COUNT or len(manual_max) < ENCODER_COUNT:
            raise CalibrationSnapshotError("Manual endpoint readback incomplete")
        mapping_version, urdf_crc = urdf_manual_scaled_mapping_metadata()
        incompatible = [
            i
            for i in range(ENCODER_COUNT)
            if joint_mask & (1 << i)
            and (
                int(results[i].get("mappingVersion", 0) or 0)
                != mapping_version
                or int(results[i].get("urdfConfigCrc", 0) or 0)
                != urdf_crc
                or int(results[i].get("manualReferenceCrc", 0) or 0)
                != manual_encoder_reference_entry_crc(i, manual_min[i], manual_max[i])
            )
        ]
        if incompatible:
            raise CalibrationSnapshotError(
                "v4/Manual CRC mismatch: " + _format_joints(incompatible)
            )
        mins = [
            int(value)
            for value in list(
                getattr(state, "motor_abs_limit_min", []) or []
            )[:MOTOR_COUNT]
        ]
        maxs = [
            int(value)
            for value in list(
                getattr(state, "motor_abs_limit_max", []) or []
            )[:MOTOR_COUNT]
        ]
        if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
            raise CalibrationSnapshotError("motor ABS limit readback incomplete")
        payload = {
            "saved_at_host_time": time.time(),
            "hand_profile": HAND_PROFILE,
            "feedback_mapping_version": mapping_version,
            "manual_scaled_mapping_version": mapping_version,
            "urdf_config_crc": f"0x{urdf_crc:08X}",
            "manual_reference_crc": f"0x{int(getattr(state, 'manual_reference_crc', 0) or 0):08X}",
            "active_joint_mask": f"0x{joint_mask:08X}",
            "active_motor_mask": f"0x{motor_mask:08X}",
            "motor_abs_limit_valid_bitmap": f"0x{valid_bitmap:08X}",
            "motor_abs_limit_min": mins,
            "motor_abs_limit_max": maxs,
            "manual_reference_valid_bitmap": f"0x{manual_mask:08X}",
            "manual_reference_min_raw": [int(value) for value in manual_min[:ENCODER_COUNT]],
            "manual_reference_max_raw": [int(value) for value in manual_max[:ENCODER_COUNT]],
            "source": {
                "transport": "P4",
                "identity_contract": "hand_profile+urdf_crc+manual_entry_crc",
            },
            "joint_calib_results": results,
        }
        signature = json.dumps(
            {
                "joint_mask": joint_mask,
                "motor_mask": motor_mask,
                "valid": valid_bitmap,
                "mins": mins,
                "maxs": maxs,
                "results": results,
            },
            sort_keys=True,
            ensure_ascii=False,
        )
        return payload, motor_mask, signature

    def _notice_once(self, event: CalibrationWorkflowEvent) -> None:
        key = f"{event.phase}:{event.message}"
        if key == self._last_notice:
            return
        self._last_notice = key
        self._emit(event)

    def inspect_snapshot(self, path) -> dict:
        snapshot = self._store.inspect(path)
        if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
            raise CalibrationSnapshotError(
                f"hand_profile mismatch: {snapshot.get('hand_profile')} != {HAND_PROFILE}"
            )
        error = validate_calibration_snapshot_for_load(snapshot)
        if error:
            raise CalibrationSnapshotError(error)
        return snapshot

    def begin_import(self, snapshot: dict, source: str = "") -> bool:
        epoch = self._next_epoch("import")
        if epoch is None:
            return False

        def worker() -> None:
            try:
                if not self._is_current(epoch):
                    return
                error = validate_calibration_snapshot_for_load(snapshot)
                if error:
                    raise CalibrationSnapshotError(error)
                if not self._is_current(epoch):
                    return
                self._store.activate(snapshot)
                if not self._is_current(epoch):
                    return
                ok, message = self._controller.load_calibration_snapshot(snapshot)
                self._emit(
                    CalibrationWorkflowEvent(
                        epoch,
                        "import",
                        bool(ok),
                        str(message),
                        parse_mask(snapshot.get("active_joint_mask")) & _JOINT_MASK,
                        parse_mask(snapshot.get("active_motor_mask")) & VALID_MOTOR_MASK,
                        details={"source": str(source)},
                    )
                )
            except Exception as exc:
                self._emit(
                    CalibrationWorkflowEvent(
                        epoch, "import", False, f"{type(exc).__name__}: {exc}"
                    )
                )
            finally:
                self._finish_operation(epoch)

        self._executor.submit(worker)
        return True

    def begin_startup_restore(self, active_joint_mask: int) -> bool:
        epoch = self._next_epoch("startup")
        if epoch is None:
            return False
        active_joint_mask = int(active_joint_mask) & _JOINT_MASK

        def worker() -> None:
            try:
                if not self._is_current(epoch):
                    return
                if not self._controller.query_manual_encoder_references():
                    self._emit(
                        CalibrationWorkflowEvent(
                            epoch,
                            "startup",
                            False,
                            "Manual query ACK/readback timeout",
                        )
                    )
                    return
                if not self._is_current(epoch):
                    return
                state_lock = getattr(self._controller, "state_lock", None)
                if state_lock is None:
                    state = self._controller.current_state
                    manual_mask = int(
                        getattr(state, "manual_reference_valid_bitmap", 0) or 0
                    )
                    committed = bool(
                        getattr(state, "manual_reference_committed", False)
                    )
                    system_state = getattr(state, "system_state", None)
                else:
                    with state_lock:
                        state = self._controller.current_state
                        manual_mask = int(
                            getattr(state, "manual_reference_valid_bitmap", 0)
                            or 0
                        )
                        committed = bool(
                            getattr(state, "manual_reference_committed", False)
                        )
                        system_state = getattr(state, "system_state", None)
                if not self._store.exists():
                    missing = (
                        active_joint_mask
                        if not committed
                        else active_joint_mask & ~manual_mask
                    )
                    self._emit(
                        CalibrationWorkflowEvent(
                            epoch,
                            "startup_no_snapshot",
                            True,
                            "no calibration snapshot; automatic calibration required",
                            missing,
                        )
                    )
                    return
                snapshot = self._store.load()
                if str(snapshot.get("hand_profile", "")) != HAND_PROFILE:
                    raise CalibrationSnapshotError("hand profile mismatch")
                if system_state in (
                    SYSTEM_STATE_RUN_DIRECT,
                    SYSTEM_STATE_RUN_FEEDBACK,
                    SYSTEM_STATE_CALIBRATION_RUNNING,
                ):
                    raise CalibrationSnapshotError("lower controller is moving")
                manual_ok = False
                manual_message = "Manual restore was not attempted"
                for attempt in range(2):
                    if not self._is_current(epoch):
                        return
                    manual_ok, manual_message = (
                        self._controller.restore_manual_references_from_snapshot(
                            snapshot
                        )
                    )
                    if manual_ok:
                        break
                    if attempt == 0:
                        time.sleep(0.20)
                if not manual_ok:
                    raise CalibrationSnapshotError(str(manual_message))
                if not self._is_current(epoch):
                    return
                motor_mask = parse_mask(snapshot.get("active_motor_mask")) & VALID_MOTOR_MASK
                recovery_ok, recovery_message = (
                    self._controller.wait_for_multiturn_recovery(motor_mask)
                )
                if not recovery_ok:
                    raise CalibrationSnapshotError(str(recovery_message))
                if not self._is_current(epoch):
                    return
                error = validate_calibration_snapshot_for_load(snapshot)
                if error:
                    self._emit(
                        CalibrationWorkflowEvent(
                            epoch,
                            "startup_manual_only",
                            True,
                            f"{manual_message}; Auto/LUT skipped: {error}",
                        )
                    )
                    return
                if not self._is_current(epoch):
                    return
                loaded, message = self._controller.load_calibration_snapshot(snapshot)
                self._emit(
                    CalibrationWorkflowEvent(
                        epoch,
                        "startup",
                        bool(loaded),
                        str(message),
                        parse_mask(snapshot.get("active_joint_mask")) & _JOINT_MASK,
                        motor_mask,
                    )
                )
            except Exception as exc:
                self._emit(
                    CalibrationWorkflowEvent(
                        epoch, "startup", False, f"{type(exc).__name__}: {exc}"
                    )
                )
            finally:
                self._finish_operation(epoch)

        self._executor.submit(worker)
        return True
