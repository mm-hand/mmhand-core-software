"""workflow.py - 继承原标定事务服务的上位机校准实现。

职责：覆写校准入口与状态观察，复用原线程、事件、取消和快照事务。
线程：复用基类的单工作执行器，不新建线程或互斥锁。
输入：HandModel、关节/电机范围和原 P4 校准参数。
输出：逐关节 Manual/LUT 写回及兼容原 UI 的工作流事件。
状态所有权：基类拥有通用事务，子类仅增加宿主工作线程活动标志。
禁止依赖：禁止依赖 Qt/UI、页面控件或直接操作串口对象。
"""

from desktop.paths import CALIB_LOAD_SNAPSHOT_PATH

from ..calibration_diagnostics import format_calibration_warning
from ..calibration_workflow import CalibrationWorkflowEvent, CalibrationWorkflowService
from ..data_models import HandModel
from ..protocol import ENCODER_COUNT, VALID_MOTOR_MASK
from .flow import run as run_host_calibration


class HostCalibrationWorkflowService(CalibrationWorkflowService):
    """原工作流的宿主实现；UI 继续调用相同的 begin_calibration 接口。"""

    def __init__(self, controller, snapshot_path=CALIB_LOAD_SNAPSHOT_PATH):
        super().__init__(controller, snapshot_path)
        self._host_worker_active = False

    def is_host_calibration_active(self) -> bool:
        with self._lock:
            return self._host_worker_active

    def _next_epoch(self, operation):
        with self._lock:
            if self._host_worker_active:
                return None
            epoch = super()._next_epoch(operation)
            if epoch is not None and operation == "host_calibration":
                self._host_worker_active = True
            return epoch

    def observe_state(self, state: HandModel) -> None:
        # 取消已清 operation，但旧 worker 的机械收口仍须独占控制。
        with self._lock:
            if self._host_worker_active:
                return
            super().observe_state(state)

    def begin_firmware_calibration(self, **kwargs) -> bool:
        """保留显式请求原 P4 工作流的入口。"""
        return super().begin_calibration(**kwargs)

    def begin_calibration(
        self,
        *,
        joint_mask: int,
        motor_mask: int,
        reset_motor_mask: int,
        baseline_state: HandModel,
    ) -> bool:
        """UI 的原入口；默认在上位机运行原 P4 标定状态机。"""
        epoch = self._next_epoch("host_calibration")
        if epoch is None:
            return False
        joint_mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
        motor_mask = int(motor_mask) & VALID_MOTOR_MASK
        reset_motor_mask = int(reset_motor_mask) & VALID_MOTOR_MASK
        with self._lock:
            self._pending_joint_mask = joint_mask
            self._pending_motor_mask = motor_mask

        def worker() -> None:
            records: list[dict] = []
            try:
                if not self._is_current(epoch):
                    return
                io = self._controller.create_host_calibration_io(
                    lambda: not self._is_current(epoch),
                    joint_mask,
                    motor_mask,
                    reset_motor_mask=reset_motor_mask,
                    snapshot_store=self._store,
                )
                self._emit(CalibrationWorkflowEvent(
                    epoch, "prepare", True,
                    "host calibration started; each joint is rebased before calibration",
                    joint_mask, reset_motor_mask,
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

                session = run_host_calibration(io, joint_mask, motor_mask, on_joint)
                if self._is_current(epoch):
                    self._emit(CalibrationWorkflowEvent(
                        epoch, "calibration_complete", session.success,
                        session.message, joint_mask, motor_mask,
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
                    f"{type(exc).__name__}: {exc}", joint_mask, motor_mask,
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
