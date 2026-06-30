"""Tests for core_logic.py - HandController.set_motor_positions_absolute"""

import pytest
from unittest.mock import MagicMock

from desktop.core.core_logic import HandController
from desktop.core.protocol import ControlMode, MOTOR_COUNT, MOTOR_ABS_CMD_MIN, MOTOR_ABS_CMD_MAX


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
