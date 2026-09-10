from types import SimpleNamespace
from unittest.mock import MagicMock

from desktop.core.calibration_snapshot import (
    joint_calib_result_has_valid_tendon_lut,
    joint_calib_result_tendon_lut_abs_ranges,
)
from desktop.core.calibration_workflow import CalibrationWorkflowEvent
from desktop.core.protocol import SYSTEM_STATE_READY
from desktop.ui.ui_control_calibration import CalibrationWorkflowController
from desktop.ui.ui_control_common import (
    DUAL_TENDON_JOINT_INDICES,
    MOTOR_TO_JOINT_INDEX,
)
from desktop.ui.ui_control_state import ControlPanelStateMixin


class _FirmwareEpochHarness(ControlPanelStateMixin):
    def __init__(self, *, transition_count: int, p4_time_ms: int):
        self.backend = SimpleNamespace(
            request_state_sync=MagicMock(),
            cancel_calibration_workflow=MagicMock(),
        )
        self.context = SimpleNamespace(emit_action=MagicMock())
        self._last_multi_turn_unknown_bitmap = 0
        self._last_system_status_p4_time_ms = p4_time_ms
        self._last_p4_reboot_host_ts = 0.0
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


class _ReadyStateHarness(ControlPanelStateMixin, CalibrationWorkflowController):
    def __init__(self):
        self.backend = SimpleNamespace(ros2_output_enabled=lambda: False)
        self.context = SimpleNamespace(emit_action=lambda _message: None)
        self._ros2_target_values = [None]
        self._last_calib_status = "RUNNING"
        self._last_joint_calib_valid_mask = 0
        self._joint_target_initialized = True
        self._joint_target_user_set = [False] * 21
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


class _TextItem:
    def __init__(self):
        self.text = ""
        self.tooltip = ""

    def setText(self, value):
        self.text = str(value)

    def setToolTip(self, value):
        self.tooltip = str(value)


class _AbsLimitDisplayHarness(ControlPanelStateMixin):
    def __init__(self):
        self.joint_abs_limit_items = [None] * 21
        self.joint_abs_limit_items[12] = _TextItem()
        self.joint_table = None


def test_tendon_lut_validation_accepts_j14_single_and_a_dual_motor_joint():
    single_joint = 14
    dual_joint = min(DUAL_TENDON_JOINT_INDICES)

    assert joint_calib_result_has_valid_tendon_lut(
        _valid_tendon_lut_result(single_joint), single_joint
    )
    assert joint_calib_result_has_valid_tendon_lut(
        _valid_tendon_lut_result(dual_joint), dual_joint
    )


def test_tendon_lut_abs_ranges_use_only_valid_samples_for_dual_joint():
    joint_idx = 12
    result = _valid_tendon_lut_result(joint_idx)
    result["tendonLutMotorAbs"] = [
        [351, 663, 1557, -99999],
        [1901, 1595, 711, 99999],
    ]

    assert joint_calib_result_tendon_lut_abs_ranges(result, joint_idx) == [
        (15, 351, 1557),
        (16, 711, 1901),
    ]


def test_joint_abs_limit_cell_displays_lut_ranges_with_runtime_margins():
    panel = _AbsLimitDisplayHarness()
    result = _valid_tendon_lut_result(12)
    result["tendonLutMotorAbs"] = [
        [351, 663, 1557],
        [1901, 1595, 711],
    ]
    state = SimpleNamespace(
        joint_calib_results=[None] * 12 + [result] + [None] * 8,
        motor_abs_limit_valid_bitmap=(1 << 15) | (1 << 16),
        motor_abs_limit_min=[-99999] * 27,
        motor_abs_limit_max=[99999] * 27,
    )

    panel._refresh_joint_abs_limit_item(12, state)

    item = panel.joint_abs_limit_items[12]
    assert item.text == "M15 [-149,2557]\nM16 [211,2901]"
    assert "LUT [351, 1557]" in item.tooltip
    assert "release margin=500" in item.tooltip
    assert "tight margin=1000" in item.tooltip
    assert "99999" not in item.text


def test_success_ready_refresh_is_independent_from_snapshot_error_event():
    panel = _ReadyStateHarness()
    state = SimpleNamespace(
        calib_status="SUCCESS",
        system_state=SYSTEM_STATE_READY,
        multi_turn_unknown_bitmap=0,
        servo_turn_overflow_bitmap=0,
    )

    panel.update_state(state)
    panel._on_calibration_workflow_event(
        CalibrationWorkflowEvent(1, "snapshot_error", False, "write failure")
    )
    panel.update_state(state)

    assert panel.body_updates == 2
    assert panel.statuses == ["Calib snapshot save blocked: write failure"]


def test_system_status_uptime_restart_restarts_startup_restore_workflow():
    panel = _FirmwareEpochHarness(transition_count=10, p4_time_ms=5000)
    state = SimpleNamespace(
        has_system_status_data=True,
        state_transition_count=10,
        multi_turn_unknown_bitmap=0,
        system_status_p4_time_ms=20,
    )

    panel._handle_firmware_epoch_change(state)

    panel.backend.cancel_calibration_workflow.assert_called_once_with()
    panel.backend.request_state_sync.assert_called_once_with(127)
    assert panel.startup_checks == 1
    assert panel._startup_device_check_running
    assert panel.cache_reset_reasons == [
        "Firmware reboot detected: restoring P4 calibration state"
    ]


def test_system_status_uptime_wrap_does_not_restart_startup_restore_workflow():
    panel = _FirmwareEpochHarness(transition_count=10, p4_time_ms=0xFFFFFFF0)
    state = SimpleNamespace(
        has_system_status_data=True,
        state_transition_count=11,
        multi_turn_unknown_bitmap=0,
        system_status_p4_time_ms=20,
    )

    panel._handle_firmware_epoch_change(state)

    panel.backend.cancel_calibration_workflow.assert_not_called()
    panel.backend.request_state_sync.assert_not_called()
    assert panel.startup_checks == 0


def test_global_packet_timestamp_rollback_does_not_fake_firmware_reboot():
    panel = _FirmwareEpochHarness(transition_count=10, p4_time_ms=5000)
    state = SimpleNamespace(
        has_system_status_data=True,
        state_transition_count=10,
        multi_turn_unknown_bitmap=0,
        has_p4_time=True,
        p4_time_ms=20,
        system_status_p4_time_ms=5100,
    )

    panel._handle_firmware_epoch_change(state)

    panel.backend.cancel_calibration_workflow.assert_not_called()
    panel.backend.request_state_sync.assert_not_called()
    assert panel.startup_checks == 0


def test_transition_counter_reset_without_uptime_restart_is_not_reboot():
    panel = _FirmwareEpochHarness(transition_count=80, p4_time_ms=30000)
    state = SimpleNamespace(
        has_system_status_data=True,
        state_transition_count=0,
        multi_turn_unknown_bitmap=0,
        system_status_p4_time_ms=30100,
    )

    panel._handle_firmware_epoch_change(state)

    panel.backend.cancel_calibration_workflow.assert_not_called()
    panel.backend.request_state_sync.assert_not_called()
    assert panel.startup_checks == 0
