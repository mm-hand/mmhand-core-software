"""Tests for core_logic.py - HandController.set_motor_positions_absolute"""

import pytest
from unittest.mock import MagicMock, call

import desktop.core.core_logic as core_logic
from desktop.core.calibration_snapshot import seal_snapshot
from desktop.core.core_logic import HandController
from desktop.core.protocol import (
    CMD_CLEAR_CALIBRATION_SCOPE,
    CMD_CALIB_LOAD_JOINT,
    CMD_CALIB_LOAD_LIMITS,
    CMD_RESTORE_MULTITURN,
    ControlMode,
    ENCODER_COUNT,
    MOTOR_COUNT,
    TENDON_LUT_MAX_SAMPLES,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RUN_FEEDBACK,
)


class TestSetMotorPositionsAbsolute:
    """Test suite for HandController.set_motor_positions_absolute method."""

    @pytest.fixture
    def controller(self):
        """Create a HandController instance with mocked dependencies."""
        return HandController()

    @pytest.fixture
    def mock_comm(self):
        """Create a mock communication object."""
        comm = MagicMock()
        comm.send_command = MagicMock()
        return comm

    # --- Normal Case Tests ---

    def test_normal_case_sends_command(self, controller, mock_comm):
        """Verify command is sent with valid input in DIRECT_MOTOR mode."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        motor_abs = [1000] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        mock_comm.send_command.assert_called_once()
        call_args = mock_comm.send_command.call_args[0][0]
        assert call_args[0] == "motor_pos_abs"
        assert len(call_args[1]) == MOTOR_COUNT

    def test_normal_case_values_preserved(self, controller, mock_comm):
        """Verify valid values within range are sent unchanged."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        test_values = [0, 1000, -5000, 15000] * ((MOTOR_COUNT + 3) // 4)
        motor_abs = test_values[:MOTOR_COUNT]
        controller.set_motor_positions_absolute(motor_abs)

        call_args = mock_comm.send_command.call_args[0][0]
        raw = call_args[1]
        for i, expected in enumerate(motor_abs):
            assert raw[i] == expected

    def test_each_motor_uses_its_own_current_session_range(
        self, controller, mock_comm
    ):
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR
        controller.current_state.servo_logical_range_valid_bitmap = (
            (1 << 0) | (1 << 1)
        )
        controller.current_state.servo_logical_range_min[0] = -100
        controller.current_state.servo_logical_range_max[0] = 200
        controller.current_state.servo_logical_range_min[1] = 1000
        controller.current_state.servo_logical_range_max[1] = 2000

        targets = [0] * MOTOR_COUNT
        targets[0] = 500
        targets[1] = 500
        controller.set_motor_positions_absolute(targets)

        sent = mock_comm.send_command.call_args[0][0][1]
        assert sent[0] == 200
        assert sent[1] == 1000
        assert sent[2] == 0

    # --- Paused State Tests ---

    def test_paused_returns_early(self, controller, mock_comm):
        """Verify method returns early when paused."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR
        controller._paused = True

        motor_abs = [1000] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        mock_comm.send_command.assert_not_called()

    # --- Invalid Length Tests ---

    def test_wrong_length_raises_valueerror(self, controller):
        """Verify ValueError is raised when motor_abs length is not MOTOR_COUNT."""
        controller._paused = False
        motor_abs = [1000] * (MOTOR_COUNT - 1)  # Wrong length

        with pytest.raises(ValueError) as excinfo:
            controller.set_motor_positions_absolute(motor_abs)
        assert str(MOTOR_COUNT) in str(excinfo.value)

    def test_empty_list_raises_valueerror(self, controller):
        """Verify ValueError is raised for empty list."""
        controller._paused = False

        with pytest.raises(ValueError):
            controller.set_motor_positions_absolute([])

    def test_too_many_values_raises_valueerror(self, controller):
        """Verify ValueError is raised when list is longer than MOTOR_COUNT."""
        controller._paused = False
        motor_abs = [1000] * (MOTOR_COUNT + 1)

        with pytest.raises(ValueError):
            controller.set_motor_positions_absolute(motor_abs)

    # --- Boundary Value Tests ---

    def test_min_limit_value_preserved(self, controller, mock_comm):
        """Verify exact MIN limit value is preserved."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        motor_abs = [MOTOR_ABS_CMD_MIN] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        call_args = mock_comm.send_command.call_args[0][0]
        raw = call_args[1]
        assert all(v == MOTOR_ABS_CMD_MIN for v in raw)

    def test_max_limit_value_preserved(self, controller, mock_comm):
        """Verify exact MAX limit value is preserved."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        motor_abs = [MOTOR_ABS_CMD_MAX] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        call_args = mock_comm.send_command.call_args[0][0]
        raw = call_args[1]
        assert all(v == MOTOR_ABS_CMD_MAX for v in raw)

    # --- Value Clamping Tests ---

    def test_value_below_min_clamped(self, controller, mock_comm):
        """Verify values below MIN are clamped to MIN."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        below_min = MOTOR_ABS_CMD_MIN - 1000
        motor_abs = [below_min] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        call_args = mock_comm.send_command.call_args[0][0]
        raw = call_args[1]
        assert all(v == MOTOR_ABS_CMD_MIN for v in raw)

    def test_value_above_max_clamped(self, controller, mock_comm):
        """Verify values above MAX are clamped to MAX."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        above_max = MOTOR_ABS_CMD_MAX + 1000
        motor_abs = [above_max] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        call_args = mock_comm.send_command.call_args[0][0]
        raw = call_args[1]
        assert all(v == MOTOR_ABS_CMD_MAX for v in raw)

    def test_mixed_values_clamped_correctly(self, controller, mock_comm):
        """Verify mixed values above/below/within range are clamped correctly."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        motor_abs = [
            MOTOR_ABS_CMD_MIN - 500,  # Below min -> clamped
            MOTOR_ABS_CMD_MAX + 500,  # Above max -> clamped
            0,                         # Valid
            15000,                     # Valid
            MOTOR_ABS_CMD_MIN,         # Exact min
            MOTOR_ABS_CMD_MAX,         # Exact max
        ] * ((MOTOR_COUNT + 5) // 6)
        motor_abs = motor_abs[:MOTOR_COUNT]

        controller.set_motor_positions_absolute(motor_abs)

        call_args = mock_comm.send_command.call_args[0][0]
        raw = call_args[1]
        assert raw[0] == MOTOR_ABS_CMD_MIN
        assert raw[1] == MOTOR_ABS_CMD_MAX

    # --- Control Mode Tests ---

    def test_joint_angle_mode_no_command(self, controller, mock_comm):
        """Verify no command is sent when mode is JOINT_ANGLE."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.JOINT_ANGLE

        motor_abs = [1000] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        mock_comm.send_command.assert_not_called()

    # --- No Communication Tests ---

    def test_no_comm_no_command(self, controller):
        """Verify no command is sent when comm is None."""
        controller.comm = None
        controller._control_mode = ControlMode.DIRECT_MOTOR

        motor_abs = [1000] * MOTOR_COUNT
        # Should not raise, just silently return
        controller.set_motor_positions_absolute(motor_abs)

    # --- Type Conversion Tests ---

    def test_float_values_converted(self, controller, mock_comm):
        """Verify float values are converted to int."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.DIRECT_MOTOR

        motor_abs = [1000.7, -500.3, 0.0] * (MOTOR_COUNT // 3)
        motor_abs = motor_abs[:MOTOR_COUNT]
        controller.set_motor_positions_absolute(motor_abs)

        call_args = mock_comm.send_command.call_args[0][0]
        raw = call_args[1]
        # Values should be converted to int
        assert all(isinstance(v, int) for v in raw)

    # --- Combination Tests ---

    def test_paused_with_joint_angle_mode(self, controller, mock_comm):
        """Verify early return when both paused and wrong mode."""
        controller.comm = mock_comm
        controller._control_mode = ControlMode.JOINT_ANGLE
        controller._paused = True

        motor_abs = [1000] * MOTOR_COUNT
        controller.set_motor_positions_absolute(motor_abs)

        mock_comm.send_command.assert_not_called()

    def test_comm_none_with_direct_motor_mode(self, controller):
        """Verify silent return when comm is None but mode is correct."""
        controller.comm = None
        controller._control_mode = ControlMode.DIRECT_MOTOR
        controller._paused = False

        motor_abs = [1000] * MOTOR_COUNT
        # Should not raise
        controller.set_motor_positions_absolute(motor_abs)


class TestRequestRunFeedback:
    """Test suite for HandController.request_run_feedback."""

    @pytest.fixture
    def controller(self):
        return HandController()

    def test_sends_when_cached_state_is_idle_after_scope_change(self, controller):
        controller.comm = MagicMock()
        controller._paused = False
        controller.current_state.system_state = SYSTEM_STATE_IDLE

        assert controller.request_run_feedback() is True
        controller.comm.send_command.assert_called_once_with("run_feedback")

    def test_requeues_when_feedback_is_already_running(self, controller):
        controller.comm = MagicMock()
        controller._paused = False
        controller.current_state.system_state = SYSTEM_STATE_RUN_FEEDBACK

        assert controller.request_run_feedback() is True
        controller.comm.send_command.assert_called_once_with("run_feedback")


class TestResetMultiturnBaselineForNewDevice:
    def test_rejects_when_all_motor_turns_are_trusted_before_any_command(self):
        controller = HandController()
        controller.comm = MagicMock()
        controller.current_state.multi_turn_unknown_bitmap = 0
        controller.current_state.has_servo_raw_data = True
        controller.current_state.servo_raw_positions = [1000] * MOTOR_COUNT
        controller.current_state.servo_raw_online = [True] * MOTOR_COUNT

        ok, message, motor_mask, joint_mask = (
            controller.reset_multiturn_baseline_for_new_device()
        )

        assert ok is False
        assert "already trusted" in message
        assert motor_mask == 0
        assert joint_mask == 0
        controller.comm.send_command.assert_not_called()

    def test_repairs_only_unknown_motor_and_returns_its_joint(self, monkeypatch):
        controller = HandController()
        controller.comm = MagicMock()
        state = controller.current_state
        state.multi_turn_unknown_bitmap = 1 << 3
        state.has_servo_raw_data = True
        state.servo_raw_positions = [1000] * MOTOR_COUNT
        state.servo_raw_positions[3] = 1234
        state.servo_raw_online = [True] * MOTOR_COUNT
        state.multiturn_nvs_enabled = False
        state.has_servo_angle_data = False
        state.servo_angles = [0] * MOTOR_COUNT
        state.servo_online = [True] * MOTOR_COUNT

        monkeypatch.setattr(core_logic.time, "sleep", lambda _seconds: None)

        def wait_for_ack(cmd, _timeout, received_after=None):
            del received_after
            if cmd == CMD_RESTORE_MULTITURN:
                state.multi_turn_unknown_bitmap = 0
                state.has_servo_angle_data = True
                state.servo_angles[3] = 1234
                return (cmd, 1, 0, 1.0)
            if cmd == CMD_CLEAR_CALIBRATION_SCOPE:
                return (cmd, 1, 0, 2.0)
            return None

        controller.comm.wait_for_ack.side_effect = wait_for_ack

        ok, message, motor_mask, joint_mask = (
            controller.reset_multiturn_baseline_for_new_device()
        )

        assert ok is True
        assert motor_mask == (1 << 3)
        assert joint_mask == (1 << 2)
        assert "affected joint" in message
        assert call(
            ("restore_multiturn", 1 << 3, [0, 0, 0, 1234] + [0] * (MOTOR_COUNT - 4))
        ) in controller.comm.send_command.call_args_list
        assert call(
            ("clear_calibration_scope", 1 << 3)
        ) in controller.comm.send_command.call_args_list
        assert not any(
            args and args[0] == "clear_calibration"
            for args, _kwargs in controller.comm.send_command.call_args_list
        )


class TestLoadCalibrationSnapshot:
    def test_each_joint_waits_for_an_ack_newer_than_its_own_send(self, monkeypatch):
        controller = HandController()
        controller.comm = MagicMock()
        controller.query_manual_encoder_references = MagicMock(return_value=True)
        controller.current_state.manual_reference_nvs_valid = True
        controller.current_state.manual_reference_valid_bitmap = (1 << 9) | (1 << 10)
        controller.current_state.manual_reference_min_raw = [1000] * ENCODER_COUNT
        controller.current_state.manual_reference_max_raw = [5000] * ENCODER_COUNT

        mapping_version = 3
        urdf_crc = 0x12345678
        manual_crc = 0x00ABCDEF
        monkeypatch.setattr(
            core_logic,
            "urdf_manual_scaled_mapping_metadata",
            lambda: (mapping_version, urdf_crc),
        )
        monkeypatch.setattr(
            core_logic,
            "manual_encoder_reference_entry_crc",
            lambda *_args: manual_crc,
        )
        monkeypatch.setattr(
            core_logic.time,
            "time",
            MagicMock(side_effect=[100.0, 101.0, 102.0]),
        )
        controller.comm.wait_for_ack.side_effect = [
            (CMD_CALIB_LOAD_LIMITS, 1, 0, 100.1),
            (CMD_CALIB_LOAD_JOINT, 1, 0, 101.1),
            (CMD_CALIB_LOAD_JOINT, 1, 0, 102.1),
        ]

        joint_results = [None] * ENCODER_COUNT
        for joint, motor in ((9, 12), (10, 13)):
            joint_results[joint] = {
                "success": True,
                "mappingVersion": mapping_version,
                "urdfConfigCrc": urdf_crc,
                "manualReferenceCrc": manual_crc,
                "tendonLutValid": True,
                "tendonLutCount": 3,
                "tendonLutMotor": [motor, 0xFF],
                "tendonLutAngleStartDeg": 0.0,
                "tendonLutAngleStepDeg": 5.0,
                "tendonLutMotorAbs": [
                    list(range(TENDON_LUT_MAX_SAMPLES)),
                    [0] * TENDON_LUT_MAX_SAMPLES,
                ],
            }
        snapshot = seal_snapshot({
            "format": "mmhand_calibration_bundle",
            "schema_version": 5,
            "hand_profile": "v2",
            "active_joint_mask": (1 << 9) | (1 << 10),
            "active_motor_mask": (1 << 12) | (1 << 13),
            "motor_abs_limit_valid_bitmap": (1 << 12) | (1 << 13),
            "motor_abs_limit_locked_bitmap": 0,
            "motor_abs_limit_min": [0] * MOTOR_COUNT,
            "motor_abs_limit_max": [0] * MOTOR_COUNT,
            "manual_reference_valid_bitmap": (1 << 9) | (1 << 10),
            "manual_reference_min_raw": [1000] * ENCODER_COUNT,
            "manual_reference_max_raw": [5000] * ENCODER_COUNT,
            "joint_calib_results": joint_results,
        })

        ok, message = controller.load_calibration_snapshot(snapshot)

        assert ok is True
        assert message == "loaded 2 joints"
        assert controller.comm.wait_for_ack.call_args_list == [
            call(
                CMD_CALIB_LOAD_LIMITS,
                core_logic.CALIB_LOAD_ACK_TIMEOUT_S,
                received_after=100.0,
            ),
            call(
                CMD_CALIB_LOAD_JOINT,
                core_logic.CALIB_LOAD_ACK_TIMEOUT_S,
                received_after=101.0,
            ),
            call(
                CMD_CALIB_LOAD_JOINT,
                core_logic.CALIB_LOAD_ACK_TIMEOUT_S,
                received_after=102.0,
            ),
        ]
