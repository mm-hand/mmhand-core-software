"""Calibration transaction service: current host flow, cancellation and file restore.

职责：串行执行最新 Host Calibration 与经确认的联合快照恢复。
线程：单工作执行器；取消只递增 epoch，机械收尾结束前保持互斥。
输入：标定范围、设备状态和已校验联合快照。
输出：逐关节事件、读回结果与持久化快照。
状态所有权：本服务唯一拥有执行器、事务 epoch、取消和事件回调。
禁止依赖：禁止依赖 UI 或直接操作串口。
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import threading
from typing import Callable, Optional
from desktop.paths import CALIB_ACTIVE_SNAPSHOT_PATH
from .angle_mapping import urdf_manual_scaled_mapping_metadata
from .calibration_snapshot import CalibrationSnapshotError, CalibrationSnapshotStore, parse_mask, validate_snapshot_for_load as validate_calibration_snapshot_for_load
from .calibration_diagnostics import format_calibration_warning
from .data_models import HandModel
from .protocol import ENCODER_COUNT, HAND_PROFILE, VALID_MOTOR_MASK
from .session_log import log_session
from .host_calibration.config import finger_aa_joint, joint_motors
from .host_calibration.dependencies import missing_aa_joint_mask
from .host_calibration.flow import run as run_host_calibration

_JOINT_MASK = (1 << ENCODER_COUNT) - 1

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



class CalibrationWorkflowService:
    """One executor owns calibration and restore transactions."""

    def __init__(self, controller, snapshot_path=CALIB_ACTIVE_SNAPSHOT_PATH):
        self._controller = controller
        self._store_override = snapshot_path != CALIB_ACTIVE_SNAPSHOT_PATH
        self._snapshot_store = CalibrationSnapshotStore(snapshot_path)
        self._executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="mmhand-calibration",
        )
        self._lock = threading.RLock()
        self._callbacks: list[Callable[[CalibrationWorkflowEvent], None]] = []
        self._epoch = 0
        self._operation = ""
        self._import_worker_active = False
        self._host_worker_active = False

    @property
    def _store(self):
        if not self._store_override:
            path = self._controller.calibration_snapshot_path
            if self._snapshot_store.path != path:
                self._snapshot_store = CalibrationSnapshotStore(path)
        return self._snapshot_store

    def _validate_device_identity(self, snapshot):
        if self._store_override:
            return
        current = self._controller.calibration_data_identity
        if any(snapshot.get(key) != current[key]
               for key in ("device_id", "urdf_repo_path")):
            raise CalibrationSnapshotError("calibration device or URDF identity mismatch")
        stored = snapshot.get("product_fingerprint", {})
        expected = current["product_fingerprint"]
        if any(stored.get(key) != expected[key] for key in (
                "schema_version", "manual_scaled_mapping_version",
                "control_model_crc", "hardware_map_crc")):
            raise CalibrationSnapshotError("calibration model or feedback contract mismatch")

    @property
    def snapshot_path(self):
        return self._store.path

    def is_host_calibration_active(self) -> bool:
        with self._lock:
            return self._host_worker_active

    def is_import_active(self) -> bool:
        with self._lock:
            return self._import_worker_active

    def last_diagnostic(self):
        """本轮标定最近一次失败记录；没有则 None。"""
        io = getattr(self._controller, "_host_calibration_io", None)
        return getattr(io, "_last_diag", None)

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

    def _next_epoch(self, operation: str) -> Optional[int]:
        with self._lock:
            if self._operation or self._import_worker_active or self._host_worker_active:
                return None
            self._epoch += 1
            self._operation = operation
            self._host_worker_active = operation == "host_calibration"
            return self._epoch

    def _finish_operation(self, epoch: int) -> None:
        with self._lock:
            if epoch == self._epoch:
                self._operation = ""

    def _clear_calibration_tracking(self, epoch: int) -> None:
        with self._lock:
            if epoch != self._epoch:
                return
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

    def inspect_snapshot(self, path) -> dict:
        snapshot = self._store.inspect(path)
        self._validate_device_identity(snapshot)
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
        with self._lock:
            self._import_worker_active = True

        def worker() -> None:
            activated = False
            stage = 'Validate import'
            try:
                if not self._is_current(epoch):
                    return
                self._validate_device_identity(snapshot)
                error = validate_calibration_snapshot_for_load(snapshot)
                if error:
                    raise CalibrationSnapshotError(error)
                if not self._is_current(epoch):
                    return
                stage = 'Restore P4 and read back'
                ok, message = self._controller.load_calibration_snapshot(snapshot)
                if not self._is_current(epoch):
                    self._emit(CalibrationWorkflowEvent(epoch, 'import', False,
                        'Import cancelled after device write; local snapshot unchanged. Check device readback.'))
                    return
                if ok:
                    stage = 'P4 readback succeeded; activate local snapshot'
                    self._store.activate(snapshot)
                    activated = True
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
                        epoch, "import", False, f"{stage}: {type(exc).__name__}: {exc}"
                    )
                )
            finally:
                self._finish_operation(epoch)
                with self._lock:
                    self._import_worker_active = False
                    cancelled = epoch != self._epoch
                    current_epoch = self._epoch
                if cancelled:
                    self._emit(CalibrationWorkflowEvent(current_epoch, 'import', False,
                        ('Import cancelled after local activation; check device readback.' if activated else
                         'Import cancelled; local snapshot unchanged. Check device readback.')))

        self._executor.submit(worker)
        return True

    def begin_calibration(
        self,
        *,
        joint_mask: int,
        motor_mask: int,
        reset_motor_mask: int,
        baseline_state: HandModel,
    ) -> bool:
        """Run the sole supported host calibration workflow."""
        epoch = self._next_epoch("host_calibration")
        if epoch is None:
            return False
        joint_mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
        motor_mask = int(motor_mask) & VALID_MOTOR_MASK
        reset_motor_mask = int(reset_motor_mask) & VALID_MOTOR_MASK

        def worker() -> None:
            records: list[dict] = []
            try:
                if not self._is_current(epoch):
                    return
                identity = {} if self._store_override else self._controller.calibration_data_identity
                # Single-tendon joints need their finger AA as a reference. If the
                # host snapshot has no usable full AA result, add that AA to this
                # same transaction so the flow calibrates it first.
                auto_joint_mask = 0
                selected_manual_mask = int(
                    getattr(baseline_state, "manual_reference_valid_bitmap", 0) or 0
                )
                if (bool(getattr(baseline_state, "has_manual_reference_encoder_directions", False))
                        and selected_manual_mask & joint_mask):
                    snapshot = None
                    try:
                        if self._store.exists():
                            snapshot = self._store.load()
                    except (OSError, ValueError, KeyError, TypeError):
                        snapshot = None
                    auto_joint_mask = missing_aa_joint_mask(
                        joint_mask,
                        snapshot,
                        baseline_state,
                        int(urdf_manual_scaled_mapping_metadata()[1]),
                    )
                auto_motor_mask = sum(
                    1 << motor
                    for joint in range(ENCODER_COUNT)
                    if auto_joint_mask & (1 << joint)
                    for motor in joint_motors(joint)
                )
                effective_joint_mask = joint_mask | auto_joint_mask
                effective_motor_mask = motor_mask | auto_motor_mask
                effective_reset_mask = reset_motor_mask | auto_motor_mask
                io = self._controller.create_host_calibration_io(
                    lambda: not self._is_current(epoch),
                    effective_joint_mask,
                    effective_motor_mask,
                    reset_motor_mask=effective_reset_mask,
                    snapshot_store=self._store,
                )
                io.snapshot_identity = identity
                auto_text = ''
                if auto_joint_mask:
                    auto_joints = ", ".join(
                        f"J{joint:02d}"
                        for joint in range(ENCODER_COUNT)
                        if auto_joint_mask & (1 << joint)
                    )
                    auto_text = (
                        f"; auto-included {auto_joints} AA because no usable host "
                        "calibration result remains"
                    )
                    log_session("CALIB_WORKFLOW", f"auto-included AA joints {auto_joints}")
                self._emit(CalibrationWorkflowEvent(
                    epoch, "prepare", True,
                    "host calibration started" + auto_text
                    + "; each joint is rebased before calibration",
                    effective_joint_mask, effective_reset_mask,
                    details={"auto_aa_joint_mask": auto_joint_mask},
                ))

                def on_joint(joint: int, outcome: dict) -> None:
                    if not self._is_current(epoch):
                        return
                    ok = bool(outcome.get("success"))
                    result = outcome.get("result") or {}
                    warning = bool(outcome.get("warning")) or bool(
                        result.get("assemblyWarningReason")
                    )
                    status = "WARN" if ok and warning else "SUCCESS" if ok else "FAILED"
                    joint_motors = int(outcome.get("motor_mask", 0))
                    record = {
                        "phase_index": int(outcome.get("sequence_index", 0)),
                        "phase_count": int(outcome.get("sequence_count", 1)),
                        "status": status,
                        "joint_mask": 1 << joint,
                        "motor_mask": joint_motors,
                        "warning_mask": (1 << joint) if warning else 0,
                        "warning_summary": None,
                        "completed_joint_mask": int(outcome.get("completed_joint_mask", 0)),
                        "failed_joint_mask": int(outcome.get("failed_joint_mask", 0)),
                    }
                    if warning:
                        state_lock = getattr(self._controller, "state_lock", None)
                        if state_lock is not None:
                            with state_lock:
                                summary = format_calibration_warning(
                                    self._controller.current_state, 1 << joint
                                )
                        else:
                            summary = format_calibration_warning(
                                self._controller.current_state, 1 << joint
                            )
                        if summary.get("ready"):
                            record["warning_summary"] = summary
                    records.append(record)
                    if ok:
                        self._emit(CalibrationWorkflowEvent(
                            epoch, "snapshot_saved", True,
                            str(outcome.get("commit_message") or
                                f"J{joint:02d} Manual/LUT/limits and anchors saved"),
                            1 << joint, joint_motors,
                        ))
                    self._emit(CalibrationWorkflowEvent(
                        epoch, "phase_complete", ok,
                        f"J{joint:02d} calibration ended with {status}",
                        1 << joint, joint_motors,
                        details={**record,
                                 "has_next_phase": bool(outcome.get("has_next_joint"))},
                    ))

                def on_started(joint, motors):
                    if self._is_current(epoch):
                        self._emit(CalibrationWorkflowEvent(
                            epoch, 'joint_started', True, f'J{joint:02d} calibrating',
                            1 << joint, motors))

                session = run_host_calibration(io, effective_joint_mask, effective_motor_mask, on_joint,
                                               on_joint_started=on_started)
                if self._is_current(epoch):
                    self._emit(CalibrationWorkflowEvent(
                        epoch, "calibration_complete", session.success,
                        session.message, effective_joint_mask, effective_motor_mask,
                        details={
                            "records": records,
                            "terminal_status": ("HOST_FAILED" if
                                session.terminal_status == "FAILED" else session.terminal_status),
                            "completed_joint_mask": session.completed_joint_mask,
                            "failed_joint_mask": session.failed_joint_mask,
                            "cancelled": session.cancelled,
                        },
                    ))
            except Exception as exc:
                self._emit(CalibrationWorkflowEvent(
                    epoch, "calibration_complete", False,
                    f"{type(exc).__name__}: {exc}", effective_joint_mask, effective_motor_mask,
                    details={"records": records, "terminal_status": "HOST_ERROR"},
                ))
            finally:
                self._clear_calibration_tracking(epoch)
                with self._lock:
                    self._host_worker_active = False

        try:
            self._executor.submit(worker)
        except RuntimeError:
            self._clear_calibration_tracking(epoch)
            with self._lock:
                self._host_worker_active = False
            return False
        return True
