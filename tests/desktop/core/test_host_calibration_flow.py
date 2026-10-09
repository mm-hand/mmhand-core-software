"""原 P4 串行调度和 UI 默认入口的宿主迁移回归。"""

import queue
import threading
from unittest.mock import MagicMock

import pytest

from desktop.core.host_calibration import workflow as calibration_workflow
from desktop.core.calibration_workflow import CalibrationWorkflowService
from desktop.core.data_models import HandModel
from desktop.core.host_calibration import flow
from desktop.core.host_calibration.config import AutoCalibrationParams


class _IO:
    def __init__(self):
        self.calls = []
        self.running = True
        self.prepare_ok = True
        self.commit_ok = True
        self.finish_ok = True
        self.feedback_ok = True

    def set_workspace_read_mask(self, mask):
        self.calls.append(("workspace", mask))

    def session_state(self, status, **fields):
        self.calls.append(("state", status, fields))

    def wait_servo_recovery(self, mask):
        self.calls.append(("servo", mask))
        return self.feedback_ok

    def wait_encoder_recovery(self, mask):
        self.calls.append(("encoder", mask))
        return self.feedback_ok

    def owns_control(self):
        return self.running

    def prepare_joint(self, joint, mask):
        self.calls.append(("prepare", joint, mask))
        return self.prepare_ok, "zero prepared"

    def commit_joint(self, joint, result, limits):
        self.calls.append(("commit", joint, result, limits))
        return self.commit_ok, "saved"

    def finish_session(self, success, completed, failed):
        self.calls.append(("finish", success, completed, failed))
        return self.finish_ok, "rope ready" if self.finish_ok else "rope init failed"

    def diag(self, *args):
        self.calls.append(("diag", args))

    def diag_if_empty(self, *args):
        self.calls.append(("diag_if_empty", args))


def _mask(joints):
    return sum(flow.calibration_joint_motor_mask(joint) for joint in joints)


def _success():
    return {"success": True, "aborted": False, "result": {"success": True},
            "limits": {0: (-1000, 1000)}}


@pytest.fixture
def machines(monkeypatch):
    calls = []
    outcomes = {}
    monkeypatch.setattr(flow, "reset_dual_workspace_cache",
                        lambda io: calls.append(("reset_cache",)))
    monkeypatch.setattr(flow, "invalidate_dual_workspace_cache_joint",
                        lambda io, joint: calls.append(("invalidate", joint)))
    monkeypatch.setattr(flow, "build_dynamic_workspace_motor_mask",
                        lambda io, joints: 0)

    def single(io, joint, params, ordered, failed, index, selected):
        calls.append(("single", joint, tuple(ordered), failed, index, selected, params))
        return outcomes.get(joint, _success())

    def dual(io, joint, index, count, selected, params):
        calls.append(("dual", joint, index, count, selected, params))
        return outcomes.get(joint, _success())

    monkeypatch.setattr(flow, "run_single", single)
    monkeypatch.setattr(flow, "run_dual", dual)
    return calls, outcomes


def test_original_mechanical_order_and_one_commit_per_completed_joint(machines):
    io = _IO()
    calls, _ = machines
    joints = (0, 1, 14, 16, 17, 20)
    results = []
    session = flow.run(io, sum(1 << j for j in joints), _mask(joints),
                       lambda j, out: results.append((j, out)))

    expected = (0, 1, 14, 20, 16, 17)
    assert session.success and session.ordered_joints == expected
    assert [item[1] for item in calls if item[0] in ("single", "dual")] == list(expected)
    assert [item[1] for item in io.calls if item[0] == "prepare"] == list(expected)
    assert [item[1] for item in io.calls if item[0] == "commit"] == list(expected)
    assert io.calls.index(("servo", _mask(joints))) < next(
        i for i, item in enumerate(io.calls) if item[0] == "prepare"
    )
    assert session.terminal_status == "SUCCESS"
    assert io.calls[-2] == ("finish", True, sum(1 << j for j in joints), 0)


def test_motor_only_selection_infers_the_complete_joint_group(machines):
    io = _IO()
    session = flow.run(io, 0, 1, lambda *_: None)
    assert session.ordered_joints == (0,)
    assert ("prepare", 0, 3) in io.calls


def test_side_sway_joints_use_slower_targets_and_matching_motion_threshold(machines):
    io = _IO()
    calls, _ = machines
    joints = (0, 4, 8, 12, 16, 20, 1)
    session = flow.run(io, sum(1 << j for j in joints), _mask(joints), lambda *_: None)
    assert session.success
    for call in calls:
        if call[0] not in ("single", "dual"):
            continue
        joint, params = call[1], call[-1]
        expected = (400, 400, 50) if joint in (1, 20) else (200, 200, 50)
        assert (params.tightenSpeedCountsPerSec, params.servoSpeed,
                params.windowServoDeltaCounts) == expected
        assert params.windowMs == (200 if joint in (1, 20) else 400)
        assert params.servoAcc == 20


def test_explicit_joint_machine_parameters_are_preserved(machines):
    io = _IO()
    calls, _ = machines
    params = AutoCalibrationParams(tightenSpeedCountsPerSec=150, servoSpeed=150,
                                   windowServoDeltaCounts=18)
    session = flow.run(io, 1, _mask((0,)), lambda *_: None, params=params)
    assert session.success
    assert next(call[-1] for call in calls if call[0] == "dual") is params


def test_single_failure_continues_and_preserves_successful_results(machines):
    io = _IO()
    calls, outcomes = machines
    outcomes[1] = {"success": False, "aborted": False,
                   "reason": "endpoint failed", "result": None, "limits": {}}
    session = flow.run(io, (1 << 1) | (1 << 2), _mask((1, 2)), lambda *_: None)

    assert not session.success
    assert session.failed_joint_mask == 1 << 1
    assert session.completed_joint_mask == 1 << 2
    assert [item[1] for item in io.calls if item[0] == "commit"] == [2]
    assert [item for item in calls if item[0] == "single"][1][3] == 1 << 1
    assert session.records[0]["has_next_joint"] is True


@pytest.mark.parametrize("joint,aborted", [(0, False), (1, True)])
def test_dual_failure_and_single_abort_stop_without_committing_failure(machines, joint, aborted):
    io = _IO()
    calls, outcomes = machines
    outcomes[joint] = {"success": False, "aborted": aborted,
                       "reason": "mechanical fault", "result": None, "limits": {}}
    session = flow.run(io, (1 << joint) | (1 << 14), _mask((joint, 14)), lambda *_: None)
    assert not session.success and session.failed_joint_mask == 1 << joint
    assert [item[1] for item in calls if item[0] in ("single", "dual")] == [joint]
    assert not any(item[0] == "commit" for item in io.calls)
    assert session.records[0]["has_next_joint"] is False


@pytest.mark.parametrize("step", ["prepare", "commit"])
def test_zero_or_writeback_failure_stops_the_session(machines, step):
    io = _IO()
    setattr(io, step + "_ok", False)
    session = flow.run(io, (1 << 1) | (1 << 2), _mask((1, 2)), lambda *_: None)
    assert not session.success and session.failed_joint_mask == 1 << 1
    assert [item[1] for item in io.calls if item[0] == "prepare"] == [1]
    assert session.records[0]["aborted"] is True


def test_preflight_failure_does_not_move_or_zero_a_joint(machines):
    io = _IO()
    io.feedback_ok = False
    session = flow.run(io, 1 << 1, _mask((1,)), lambda *_: None)
    assert not session.success
    assert not any(item[0] in ("prepare", "commit") for item in io.calls)
    assert ("finish", False, 0, 0) in io.calls


def test_preflight_failure_exposes_actual_recovery_cause(machines):
    io = _IO()
    io.feedback_ok = False
    io.feedback_recovery_error = 'maintenance RESET timeout/reject'
    session = flow.run(io, 1 << 1, _mask((1,)), lambda *_: None)
    assert session.message == 'selected servo feedback recovery failed: maintenance RESET timeout/reject'


def test_cancel_between_joints_stops_and_keeps_the_committed_joint(machines):
    io = _IO()
    session = flow.run(io, (1 << 1) | (1 << 2), _mask((1, 2)),
                       lambda *_: setattr(io, "running", False))
    assert session.cancelled and not session.success
    assert session.completed_joint_mask == 1 << 1
    assert [item[1] for item in io.calls if item[0] == "prepare"] == [1]


@pytest.mark.parametrize("step", ["prepare", "commit", "motion"])
def test_cancel_during_joint_keeps_prior_results_without_new_failure(machines, monkeypatch, step):
    io = _IO()
    io.cancelled = False
    io.is_cancelled = lambda: io.cancelled
    _, outcomes = machines
    # 原策略允许单舵机失败后继续；取消也不能抹掉此前的真实失败记录。
    outcomes[1] = {"success": False, "aborted": False,
                   "reason": "earlier mechanical fault", "result": None, "limits": {}}
    if step == "motion":
        original_single = flow.run_single

        def single(*args):
            if args[1] == 3:
                io.cancelled = True
                return {"success": False, "aborted": True,
                        "reason": "calibration cancelled", "result": None, "limits": {}}
            return original_single(*args)

        monkeypatch.setattr(flow, "run_single", single)
    else:
        original = getattr(io, step + "_joint")

        def cancel_on_third(joint, *args):
            if joint == 3:
                io.calls.append((step, joint, *args))
                io.cancelled = True
                return False, "logical ABS zero applied; feedback wait cancelled"
            return original(joint, *args)

        setattr(io, step + "_joint", cancel_on_third)
    session = flow.run(io, sum(1 << j for j in (1, 2, 3, 4)), _mask((1, 2, 3, 4)),
                       lambda *_: None)
    assert session.cancelled and not session.success
    assert session.completed_joint_mask == 1 << 2
    assert session.failed_joint_mask == 1 << 1
    assert session.records[0]["reason"] == "earlier mechanical fault"
    assert session.records[-1]["joint"] == 3
    assert session.records[-1]["cancelled"]
    assert not session.records[-1]["has_next_joint"]
    assert not any(call[0] == "diag_if_empty" for call in io.calls)
    assert not any(call[0] == "diag" for call in io.calls)
    assert [call[1] for call in io.calls if call[0] == "prepare"] == [1, 2, 3]


def test_cancel_during_preflight_is_reported_as_cancelled_without_joint_failure(machines):
    io = _IO()
    io.feedback_ok = False
    io.is_cancelled = lambda: True
    session = flow.run(io, 1 << 1, _mask((1,)), lambda *_: None)
    assert session.cancelled and not session.success
    assert session.failed_joint_mask == session.completed_joint_mask == 0
    assert "cancelled" in session.message
    assert not any(call[0] in ("prepare", "commit", "diag_if_empty") for call in io.calls)


def test_rope_init_failure_is_warn_with_valid_calibration_kept(machines):
    io = _IO()
    io.finish_ok = False
    session = flow.run(io, 1 << 1, _mask((1,)), lambda *_: None)
    assert session.success and session.terminal_status == "WARN"
    assert "rope init failed" in session.message
    assert session.completed_joint_mask == 1 << 1


def test_exception_always_releases_workspace_and_finishes_session(machines, monkeypatch):
    io = _IO()

    def crash(*_):
        raise RuntimeError("unexpected feedback failure")

    monkeypatch.setattr(flow, "run_single", crash)
    with pytest.raises(RuntimeError, match="unexpected feedback"):
        flow.run(io, 1 << 1, _mask((1,)), lambda *_: None)
    assert ("finish", False, 0, 0) in io.calls
    assert io.calls[-3] == ("workspace", 0)


def test_ui_default_workflow_uses_host_io_and_ignores_p4_idle(tmp_path, monkeypatch):
    entered = threading.Event()
    release = threading.Event()
    io = _IO()

    class Controller:
        current_state = HandModel()
        state_lock = threading.RLock()

        def create_host_calibration_io(self, cancel_check, joint_mask, motor_mask, **kwargs):
            self.factory = (cancel_check, joint_mask, motor_mask, kwargs)
            return io

        def request_firmware_calibration(self):
            raise AssertionError("default UI path must use the host machines")

    def host_run(calib_io, joint_mask, motor_mask, on_joint):
        assert calib_io is io
        entered.set()
        assert release.wait(2)
        on_joint(1, {**_success(), "motor_mask": _mask((1,)),
                     "sequence_index": 0, "sequence_count": 1,
                     "completed_joint_mask": 1 << 1, "failed_joint_mask": 0,
                     "has_next_joint": False})
        return flow.CalibrationSessionResult(True, 1 << 1, 0, (1,),
                                              message="complete", terminal_status="SUCCESS")

    monkeypatch.setattr(calibration_workflow, "run_host_calibration", host_run)
    controller = Controller()
    service = calibration_workflow.HostCalibrationWorkflowService(controller, tmp_path / "calib.json")
    events = queue.Queue()
    service.register_callback(events.put)
    assert service.begin_calibration(joint_mask=1 << 1, motor_mask=_mask((1,)),
                                     reset_motor_mask=_mask((1,)), baseline_state=HandModel())
    assert entered.wait(2)
    service.observe_state(HandModel())
    assert service._operation == "host_calibration"
    release.set()
    completed = None
    while completed is None:
        event = events.get(timeout=2)
        if event.phase == "calibration_complete":
            completed = event
    assert completed.ok and completed.details["completed_joint_mask"] == 1 << 1
    assert controller.factory[3]["snapshot_store"] is service._store
    assert controller.factory[3]["reset_motor_mask"] == _mask((1,))
    service._executor.shutdown(wait=True)


def test_cancel_keeps_external_output_blocked_until_host_worker_finishes(tmp_path, monkeypatch):
    entered = threading.Event()
    release = threading.Event()

    class Controller:
        def create_host_calibration_io(self, *_args, **_kwargs):
            return _IO()

    def host_run(*_args):
        entered.set()
        assert release.wait(2)
        return flow.CalibrationSessionResult(False, cancelled=True)

    monkeypatch.setattr(calibration_workflow, 'run_host_calibration', host_run)
    service = calibration_workflow.HostCalibrationWorkflowService(Controller(), tmp_path / 'calib.json')
    args = dict(joint_mask=2, motor_mask=4, reset_motor_mask=4, baseline_state=HandModel())
    assert service.begin_calibration(**args)
    assert entered.wait(2)
    service.cancel()
    assert service.is_host_calibration_active()
    assert not service.begin_calibration(**args)
    release.set()
    service._executor.shutdown(wait=True)
    assert not service.is_host_calibration_active()


def test_host_service_inherits_original_transaction_facilities(tmp_path):
    service = calibration_workflow.HostCalibrationWorkflowService(MagicMock(), tmp_path / 'calib.json')
    assert isinstance(service, CalibrationWorkflowService)
    assert service.cancel.__func__ is CalibrationWorkflowService.cancel
    assert service.begin_import.__func__ is CalibrationWorkflowService.begin_import
    assert service.begin_startup_restore.__func__ is CalibrationWorkflowService.begin_startup_restore
    assert not service.is_host_calibration_active()
    assert not CalibrationWorkflowService.is_host_calibration_active(service)
    service._executor.shutdown(wait=True)


def test_explicit_firmware_entry_uses_original_base_flow_in_host_subclass(tmp_path):
    controller = MagicMock()
    controller.prepare_calibration_turn_zero.return_value = (True, 'prepared', 4, 0)
    controller.set_active_scope.return_value = True
    controller.request_firmware_calibration.return_value = True
    service = calibration_workflow.HostCalibrationWorkflowService(controller, tmp_path / 'calib.json')
    prepared = threading.Event()
    service.register_callback(lambda event: prepared.set() if event.phase == 'prepare' else None)
    assert service.begin_firmware_calibration(
        joint_mask=2, motor_mask=4, reset_motor_mask=4, baseline_state=HandModel())
    assert prepared.wait(2)
    controller.request_firmware_calibration.assert_called_once()
    controller.create_host_calibration_io.assert_not_called()
    assert not service.is_host_calibration_active()
    service.cancel()
    service._executor.shutdown(wait=True)


def test_executor_submit_failure_rolls_back_host_flags(tmp_path):
    service = calibration_workflow.HostCalibrationWorkflowService(MagicMock(), tmp_path / 'calib.json')
    service._executor.shutdown(wait=True)
    assert not service.begin_calibration(
        joint_mask=2, motor_mask=4, reset_motor_mask=4, baseline_state=HandModel())
    assert not service.is_host_calibration_active()
    assert service._operation == ''
