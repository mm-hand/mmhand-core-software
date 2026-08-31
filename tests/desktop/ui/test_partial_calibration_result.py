from types import SimpleNamespace
from unittest.mock import MagicMock

from desktop.core.protocol import (
    CALIB_DIAG_PHASE_SEQUENCE,
    CALIB_DIAG_REASON_JOINT_STEP_FAILED,
    SYSTEM_STATE_READY,
)
from desktop.ui.ui_control_calibration import CalibrationWorkflowController
from desktop.ui.ui_control_common import (
    DUAL_TENDON_JOINT_INDICES,
    MOTOR_TO_JOINT_INDEX,
)
from desktop.ui.ui_control_state import ControlPanelStateMixin


class _FirmwareEpochHarness(ControlPanelStateMixin):
    def __init__(self, *, transition_count: int, p4_time_ms: int):
        self.backend = SimpleNamespace(request_state_sync=MagicMock())
        self.context = SimpleNamespace(emit_action=MagicMock())
        self._last_state_transition_count = transition_count
        self._last_multi_turn_unknown_bitmap = 0
        self._last_p4_time_ms = p4_time_ms
        self._startup_device_check_epoch = 4
        self._startup_device_check_running = False
        self.cache_reset_reasons = []
        self.startup_checks = 0

    def _reset_motor_target_cache_for_device_epoch(self, reason):
        self.cache_reset_reasons.append(reason)

    def begin_startup_device_check(self):
        self.startup_checks += 1
        self._startup_device_check_running = True


def _valid_tendon_lut_result(joint_idx: int) -> dict:
    motors = [
        motor_idx
        for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX)
        if mapped_joint == joint_idx
    ]
    return {
        "success": True,
        "tendonLutValid": True,
        "tendonLutCount": 3,
        "tendonLutMotor": (motors + [0xFF, 0xFF])[:2],
        "tendonLutMotorAbs": [[100, 200, 300], [400, 500, 600]],
    }


class _ReadyStateHarness(ControlPanelStateMixin):
    def __init__(self):
        self.backend = SimpleNamespace(ros2_output_enabled=lambda: False)
        self.context = SimpleNamespace(emit_action=lambda _message: None)
        self._ros2_target_values = [None]
        self._last_calib_status = "RUNNING"
        self._last_joint_calib_valid_mask = 0
        self._joint_target_initialized = True
        self._joint_target_user_set = [False] * 21
        self._pending_calib_joint_mask = 0
        self._pending_calib_motor_mask = 0
        self._pending_calib_result_baselines = {}
        self._app_mode = "monitor"
        self._body_widget = SimpleNamespace(update_state=self._record_body_update)
        self.body_updates = 0
        self.statuses = []

    def _record_body_update(self, _state):
        self.body_updates += 1

    def _set_status(self, message):
        self.statuses.append(str(message))

    def clear_ros2_joint_targets(self):
        self._ros2_target_values = [None]

    def _update_encoder_common_bus_guard(self, _state):
        pass

    def reset_turn_unknown_popup_episode(self):
        pass

    def reset_servo_turn_overflow_popup_episode(self):
        pass

    def _handle_firmware_epoch_change(self, _state):
        pass

    def _handle_servo_route_status(self, _state):
        pass

    def _handle_connectivity_transitions(self, _state):
        pass

    def _handle_direct_command_result(self, _state):
        pass

    def _joint_calib_valid_mask(self, _state):
        return 0

    def _maybe_show_calibration_assembly_warning(self, _state):
        pass

    def _maybe_save_calibration_snapshot(self, _state):
        raise RuntimeError("simulated snapshot write failure")


def test_partial_calibration_summary_returns_only_completed_joints():
    state = SimpleNamespace(
        calib_diag_valid=True,
        calib_diag_completed_joint_mask=(1 << 3) | (1 << 4),
        calib_diag_failed_joint_mask=(1 << 5),
    )

    mask = CalibrationWorkflowController._partial_calibration_completed_joint_mask(
        SimpleNamespace(), state
    )

    assert mask == (1 << 3) | (1 << 4)


def test_process_failure_diag_is_not_treated_as_partial_summary():
    state = SimpleNamespace(
        calib_diag_valid=True,
        calib_diag_reason=CALIB_DIAG_REASON_JOINT_STEP_FAILED,
        calib_diag_phase=CALIB_DIAG_PHASE_SEQUENCE,
        calib_diag_motor=3,
        calib_diag_value_a=9,
        calib_diag_value_b=0,
        calib_diag_bitmap=(1 << 3),
        calib_diag_completed_joint_mask=0,
        calib_diag_failed_joint_mask=0,
    )

    mask = CalibrationWorkflowController._partial_calibration_completed_joint_mask(
        SimpleNamespace(), state
    )

    assert mask == 0


def test_incremental_result_mask_only_contains_changed_successful_joints():
    old_j03 = {"success": True, "joint": 3, "zeroEncoder": 100}
    new_j03 = {"success": True, "joint": 3, "zeroEncoder": 120}
    unchanged_j04 = {"success": True, "joint": 4, "zeroEncoder": 200}
    results = [None] * 21
    results[3] = new_j03
    results[4] = unchanged_j04
    panel = SimpleNamespace(
        _pending_calib_joint_mask=(1 << 3) | (1 << 4) | (1 << 5),
        _pending_calib_result_baselines={
            3: CalibrationWorkflowController._joint_calib_result_signature(old_j03),
            4: CalibrationWorkflowController._joint_calib_result_signature(unchanged_j04),
            5: None,
        },
    )

    mask = CalibrationWorkflowController._incremental_calibration_completed_joint_mask(
        panel,
        SimpleNamespace(joint_calib_results=results),
    )

    assert mask == (1 << 3)


def test_incremental_result_mask_ignores_failed_or_unselected_results():
    results = [None] * 21
    results[2] = {"success": False, "joint": 2}
    results[6] = {"success": True, "joint": 6, "zeroEncoder": 600}
    panel = SimpleNamespace(
        _pending_calib_joint_mask=(1 << 2),
        _pending_calib_result_baselines={2: None},
    )

    mask = CalibrationWorkflowController._incremental_calibration_completed_joint_mask(
        panel,
        SimpleNamespace(joint_calib_results=results),
    )

    assert mask == 0


def test_tendon_lut_validation_accepts_j14_single_and_a_dual_motor_joint():
    single_joint = 14
    dual_joint = min(DUAL_TENDON_JOINT_INDICES)

    assert CalibrationWorkflowController._joint_calib_result_has_valid_tendon_lut(
        _valid_tendon_lut_result(single_joint), single_joint
    )
    assert CalibrationWorkflowController._joint_calib_result_has_valid_tendon_lut(
        _valid_tendon_lut_result(dual_joint), dual_joint
    )


def test_success_ready_update_continues_when_snapshot_save_fails():
    panel = _ReadyStateHarness()
    state = SimpleNamespace(
        calib_status="SUCCESS",
        system_state=SYSTEM_STATE_READY,
        multi_turn_unknown_bitmap=0,
        servo_turn_overflow_bitmap=0,
    )

    panel.update_state(state)
    panel.update_state(state)

    assert panel.body_updates == 2
    assert panel.statuses == [
        "Calib snapshot save failed: RuntimeError: simulated snapshot write failure"
    ]


def test_p4_timestamp_rollback_restarts_startup_restore_workflow():
    panel = _FirmwareEpochHarness(transition_count=10, p4_time_ms=5000)
    state = SimpleNamespace(
        has_system_status_data=True,
        state_transition_count=10,
        multi_turn_unknown_bitmap=0,
        has_p4_time=True,
        p4_time_ms=20,
    )

    panel._handle_firmware_epoch_change(state)

    panel.backend.request_state_sync.assert_called_once_with(127)
    assert panel.startup_checks == 1
    assert panel._startup_device_check_running
    assert panel.cache_reset_reasons == [
        "Firmware reboot detected: restoring P4 calibration state"
    ]


def test_p4_timestamp_wrap_does_not_restart_startup_restore_workflow():
    panel = _FirmwareEpochHarness(transition_count=10, p4_time_ms=0xFFFFFFF0)
    state = SimpleNamespace(
        has_system_status_data=True,
        state_transition_count=11,
        multi_turn_unknown_bitmap=0,
        has_p4_time=True,
        p4_time_ms=20,
    )

    panel._handle_firmware_epoch_change(state)

    panel.backend.request_state_sync.assert_not_called()
    assert panel.startup_checks == 0
