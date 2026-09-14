import queue
import threading

from desktop.core.angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from desktop.core.calibration_snapshot import read_snapshot
from desktop.core.calibration_workflow import CalibrationWorkflowService
from desktop.core.data_models import HandModel
from desktop.core.motor_mapping import MOTOR_TO_JOINT_INDEX
from desktop.core.protocol import (
    CALIB_WARN_AUTO_MIN_OFFSET,
    ENCODER_COUNT,
    MOTOR_COUNT,
    SYSTEM_STATE_IDLE,
    manual_encoder_reference_crc,
    manual_encoder_reference_entry_crc,
)


class _Controller:
    def __init__(self):
        self.calls = []
        self.anchor_capture_result = (True, "anchors saved")
        self.state_lock = threading.RLock()
        self.current_state = HandModel()
        self.current_state.system_state = SYSTEM_STATE_IDLE

    def prepare_calibration_turn_zero(self, motor_mask):
        self.calls.append(("reset_zero", int(motor_mask)))
        return True, "reset", int(motor_mask), 0

    def set_control_mode(self, mode):
        self.calls.append(("mode", mode))

    def set_active_scope(self, motor_mask, joint_mask, require_can):
        self.calls.append(
            ("scope", int(motor_mask), int(joint_mask), bool(require_can))
        )
        return True

    def request_firmware_calibration(self):
        self.calls.append(("calibrate",))
        return True

    def request_multiturn_anchor_capture(self, motor_mask):
        self.calls.append(("anchor", int(motor_mask)))
        return True

    def wait_for_multiturn_anchor_capture(self, motor_mask):
        self.calls.append(("anchor_wait", int(motor_mask)))
        return self.anchor_capture_result

    def query_manual_encoder_references(self):
        self.calls.append(("query_manual",))
        return True

    def restore_manual_references_from_snapshot(self, snapshot):
        self.calls.append(("restore_manual", snapshot["active_joint_mask"]))
        return True, "manual restored"

    def wait_for_multiturn_recovery(self, motor_mask):
        self.calls.append(("recover_turns", int(motor_mask)))
        return True, "turns restored"

    def load_calibration_snapshot(self, snapshot):
        self.calls.append(("load_auto", snapshot["active_joint_mask"]))
        return True, "loaded"


def _motor_for_joint(joint_idx: int) -> int:
    return next(
        motor_idx
        for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX)
        if int(mapped_joint) == int(joint_idx)
    )


def _motors_for_joint(joint_idx: int) -> list[int]:
    return [
        motor_idx
        for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX)
        if int(mapped_joint) == int(joint_idx)
    ]


def _successful_state(joint_idx: int) -> HandModel:
    motors = _motors_for_joint(joint_idx)
    motor_idx = motors[0]
    mapping_version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    urdf_min, urdf_max = get_spec(joint_idx).urdf_range_deg
    manual_min = [100 + idx for idx in range(ENCODER_COUNT)]
    manual_max = [4000 + idx for idx in range(ENCODER_COUNT)]
    state = HandModel()
    state.calib_status = "SUCCESS"
    state.calib_diag_completed_joint_mask = 1 << joint_idx
    state.calib_diag_failed_joint_mask = 0
    state.motor_abs_limit_valid_bitmap = sum(1 << motor for motor in motors)
    state.motor_abs_limit_min = [0] * MOTOR_COUNT
    state.motor_abs_limit_max = [0] * MOTOR_COUNT
    for motor in motors:
        state.motor_abs_limit_min[motor] = 1000
        state.motor_abs_limit_max[motor] = 5000
    state.manual_reference_valid_bitmap = (1 << ENCODER_COUNT) - 1
    state.manual_reference_min_raw = manual_min
    state.manual_reference_max_raw = manual_max
    state.manual_reference_crc = manual_encoder_reference_crc(
        manual_min, manual_max
    )
    state.joint_calib_results = [None] * ENCODER_COUNT
    state.joint_calib_results[joint_idx] = {
        "joint": joint_idx,
        "success": True,
        "mappingVersion": mapping_version,
        "urdfConfigCrc": urdf_crc,
        "angleMin": urdf_min,
        "currentMinDeg": urdf_min,
        "angleMax": urdf_max,
        "currentMaxDeg": urdf_max,
        "manualReferenceCrc": manual_encoder_reference_entry_crc(
            joint_idx, manual_min[joint_idx], manual_max[joint_idx]
        ),
        "tendonLutValid": True,
        "tendonLutCount": 3,
        "tendonLutMotor": [motor_idx, motors[1] if len(motors) > 1 else 0xFF],
        "tendonLutMotorAbs": [
            [1000, 3000, 5000],
            [5000, 3000, 1000] if len(motors) > 1 else [0, 0, 0],
        ],
    }
    return state


def _next_phase(events: queue.Queue, phase: str, timeout: float = 3.0):
    while True:
        event = events.get(timeout=timeout)
        if event.phase == phase:
            return event


def test_two_calibration_rounds_merge_results_and_capture_each_anchor(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)

    for joint_idx in (14, 15):
        motor_idx = _motor_for_joint(joint_idx)
        baseline = HandModel()
        assert service.begin_calibration(
            joint_mask=1 << joint_idx,
            motor_mask=1 << motor_idx,
            reset_motor_mask=1 << motor_idx,
            baseline_state=baseline,
        )
        assert _next_phase(events, "prepare").ok
        service.observe_state(_successful_state(joint_idx))
        saved = _next_phase(events, "snapshot_saved")
        assert saved.joint_mask == 1 << joint_idx
        assert saved.motor_mask == 1 << motor_idx

    snapshot = read_snapshot(service.snapshot_path)
    assert snapshot["joint_calib_results"][14]["success"] is True
    assert snapshot["joint_calib_results"][15]["success"] is True
    assert int(snapshot["active_joint_mask"], 0) & ((1 << 14) | (1 << 15)) == (
        (1 << 14) | (1 << 15)
    )
    assert ("anchor", 1 << _motor_for_joint(14)) in controller.calls
    assert ("anchor", 1 << _motor_for_joint(15)) in controller.calls
    assert ("anchor_wait", 1 << _motor_for_joint(14)) in controller.calls
    assert ("anchor_wait", 1 << _motor_for_joint(15)) in controller.calls


def test_calibration_does_not_report_saved_before_anchor_commit(tmp_path):
    controller = _Controller()
    controller.anchor_capture_result = (False, "M18 missing")
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    joint_idx = 14
    motor_idx = _motor_for_joint(joint_idx)

    assert service.begin_calibration(
        joint_mask=1 << joint_idx,
        motor_mask=1 << motor_idx,
        reset_motor_mask=1 << motor_idx,
        baseline_state=HandModel(),
    )
    assert _next_phase(events, "prepare").ok
    service.observe_state(_successful_state(joint_idx))
    error = _next_phase(events, "snapshot_error")

    assert error.ok is False
    assert "multi-turn anchors are incomplete" in error.message
    assert "M18 missing" in error.message


def test_startup_restore_orders_manual_turn_recovery_before_auto_load(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    joint_idx = 14
    motor_idx = _motor_for_joint(joint_idx)

    assert service.begin_calibration(
        joint_mask=1 << joint_idx,
        motor_mask=1 << motor_idx,
        reset_motor_mask=1 << motor_idx,
        baseline_state=HandModel(),
    )
    assert _next_phase(events, "prepare").ok
    service.observe_state(_successful_state(joint_idx))
    assert _next_phase(events, "snapshot_saved").ok

    controller.calls.clear()
    controller.current_state.system_state = SYSTEM_STATE_IDLE
    assert service.begin_startup_restore(1 << joint_idx)
    assert _next_phase(events, "startup").ok

    names = [item[0] for item in controller.calls]
    assert names == [
        "query_manual",
        "restore_manual",
        "recover_turns",
        "load_auto",
    ]


def test_cancelled_startup_restore_stops_after_current_blocking_step(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    joint_idx = 14
    motor_idx = _motor_for_joint(joint_idx)
    assert service.begin_calibration(
        joint_mask=1 << joint_idx,
        motor_mask=1 << motor_idx,
        reset_motor_mask=1 << motor_idx,
        baseline_state=HandModel(),
    )
    assert _next_phase(events, "prepare").ok
    service.observe_state(_successful_state(joint_idx))
    assert _next_phase(events, "snapshot_saved").ok

    query_started = threading.Event()
    query_release = threading.Event()

    def blocking_query():
        controller.calls.append(("query_manual",))
        query_started.set()
        query_release.wait(2.0)
        return True

    controller.calls.clear()
    controller.query_manual_encoder_references = blocking_query
    assert service.begin_startup_restore(1 << joint_idx)
    assert query_started.wait(2.0)
    service.cancel()
    query_release.set()
    drained = threading.Event()
    service._executor.submit(drained.set)
    assert drained.wait(2.0)

    assert [item[0] for item in controller.calls] == ["query_manual"]


def test_success_after_identical_running_save_releases_transaction(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    joint_idx = 14
    motor_idx = _motor_for_joint(joint_idx)
    assert service.begin_calibration(
        joint_mask=1 << joint_idx,
        motor_mask=1 << motor_idx,
        reset_motor_mask=1 << motor_idx,
        baseline_state=HandModel(),
    )
    assert _next_phase(events, "prepare").ok

    running = _successful_state(joint_idx)
    running.calib_status = "RUNNING"
    service.observe_state(running)
    assert _next_phase(events, "snapshot_saved").ok
    running.calib_status = "SUCCESS"
    service.observe_state(running)
    drained = threading.Event()
    service._executor.submit(drained.set)
    assert drained.wait(2.0)

    next_joint = 15
    next_motor = _motor_for_joint(next_joint)
    assert service.begin_calibration(
        joint_mask=1 << next_joint,
        motor_mask=1 << next_motor,
        reset_motor_mask=1 << next_motor,
        baseline_state=HandModel(),
    )
    assert _next_phase(events, "prepare").ok


def test_identical_recalibration_is_saved_after_scope_invalidation(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    joint_idx = 14
    motor_idx = _motor_for_joint(joint_idx)

    for _round in range(2):
        assert service.begin_calibration(
            joint_mask=1 << joint_idx,
            motor_mask=1 << motor_idx,
            reset_motor_mask=1 << motor_idx,
            baseline_state=HandModel(),
        )
        assert _next_phase(events, "prepare").ok
        service.observe_state(_successful_state(joint_idx))
        assert _next_phase(events, "snapshot_saved").ok

    snapshot = read_snapshot(service.snapshot_path)
    assert snapshot["joint_calib_results"][joint_idx]["success"] is True
    assert int(snapshot["active_joint_mask"], 0) & (1 << joint_idx)


def test_mixed_calibration_runs_dual_then_single_with_one_reset(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    dual_joint = 0
    single_joint = 14
    joint_mask = (1 << dual_joint) | (1 << single_joint)
    motor_mask = sum(
        1 << motor
        for motor, joint in enumerate(MOTOR_TO_JOINT_INDEX)
        if joint_mask & (1 << int(joint))
    )

    assert service.begin_calibration(
        joint_mask=joint_mask,
        motor_mask=motor_mask,
        reset_motor_mask=motor_mask,
        baseline_state=HandModel(),
    )
    first = _next_phase(events, "prepare")
    assert first.ok and first.joint_mask == 1 << dual_joint

    service.observe_state(_successful_state(dual_joint))
    assert _next_phase(events, "snapshot_saved").joint_mask == 1 << dual_joint
    second = _next_phase(events, "prepare")
    assert second.ok and second.joint_mask == 1 << single_joint

    running = HandModel()
    running.calib_status = "RUNNING"
    service.observe_state(running)
    service.observe_state(_successful_state(single_joint))
    assert _next_phase(events, "snapshot_saved").joint_mask == 1 << single_joint

    assert [call[0] for call in controller.calls].count("reset_zero") == 1
    scopes = [call for call in controller.calls if call[0] == "scope"]
    assert [call[2] for call in scopes] == [1 << dual_joint, 1 << single_joint]
    assert [call[0] for call in controller.calls].count("calibrate") == 2


def test_mixed_calibration_warn_continues_and_emits_one_final_batch(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    dual_joint, single_joint = 0, 14
    joint_mask = (1 << dual_joint) | (1 << single_joint)
    motor_mask = sum(
        1 << motor
        for motor, joint in enumerate(MOTOR_TO_JOINT_INDEX)
        if joint_mask & (1 << int(joint))
    )
    assert service.begin_calibration(
        joint_mask=joint_mask,
        motor_mask=motor_mask,
        reset_motor_mask=motor_mask,
        baseline_state=HandModel(),
    )
    assert _next_phase(events, "prepare").joint_mask == 1 << dual_joint

    warned = _successful_state(dual_joint)
    warned.calib_status = "WARN"
    warned.has_calibration_warning_status = True
    warned.calibration_warning_bitmap = 1 << dual_joint
    warned.joint_calib_results[dual_joint].update(
        {
            "hasAssemblyWarningFields": True,
            "assemblyWarningReason": CALIB_WARN_AUTO_MIN_OFFSET,
            "autoMinOffsetCounts": 400,
        }
    )
    service.observe_state(warned)
    assert _next_phase(events, "snapshot_saved").joint_mask == 1 << dual_joint
    phase_done = _next_phase(events, "phase_complete")
    assert phase_done.details["status"] == "WARN"
    assert phase_done.details["has_next_phase"] is True
    assert _next_phase(events, "prepare").joint_mask == 1 << single_joint

    running = HandModel()
    running.calib_status = "RUNNING"
    service.observe_state(running)
    service.observe_state(_successful_state(single_joint))
    assert _next_phase(events, "snapshot_saved").joint_mask == 1 << single_joint
    assert _next_phase(events, "phase_complete").details["status"] == "SUCCESS"
    completed = _next_phase(events, "calibration_complete")
    assert completed.ok is True
    assert [item["status"] for item in completed.details["records"]] == [
        "WARN",
        "SUCCESS",
    ]


def test_mixed_calibration_stops_when_dual_phase_fails(tmp_path):
    controller = _Controller()
    service = CalibrationWorkflowService(
        controller, tmp_path / "calibration_bundle.mmcal.json"
    )
    events = queue.Queue()
    service.register_callback(events.put)
    joint_mask = (1 << 0) | (1 << 14)
    motor_mask = sum(
        1 << motor
        for motor, joint in enumerate(MOTOR_TO_JOINT_INDEX)
        if joint_mask & (1 << int(joint))
    )
    assert service.begin_calibration(
        joint_mask=joint_mask,
        motor_mask=motor_mask,
        reset_motor_mask=motor_mask,
        baseline_state=HandModel(),
    )
    assert _next_phase(events, "prepare").joint_mask == 1 << 0

    failed = HandModel()
    failed.calib_status = "FAILED"
    failed.calib_diag_failed_joint_mask = 1 << 0
    service.observe_state(failed)
    drained = threading.Event()
    service._executor.submit(drained.set)
    assert drained.wait(2.0)

    assert [call[0] for call in controller.calls].count("calibrate") == 1
    assert [call for call in controller.calls if call[0] == "scope"][-1][2] == 1 << 0
