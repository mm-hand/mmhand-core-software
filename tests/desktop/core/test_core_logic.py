"""Tests for core_logic.py - HandController.set_motor_positions_absolute"""

import pytest
from unittest.mock import MagicMock, call

import desktop.core.core_logic as core_logic
from desktop.core.angle_mapping import get_spec
from desktop.core.calibration_snapshot import (
    atomic_write_snapshot,
    merge_snapshot,
    multiturn_anchor_fields,
    seal_snapshot,
)
from desktop.core.core_logic import HandController
from desktop.core.data_models import HandModel
from desktop.core.device_state_store import MultiTurnRuntimeStore
from desktop.core.protocol import (
    CMD_CALIB_LOAD_JOINT,
    CMD_CALIB_LOAD_LIMITS,
    CMD_MANUAL_ENCODER_REFERENCE,
    CMD_RESET,
    CMD_SERVO_RESET_ZERO,
    ControlMode,
    ENCODER_COUNT,
    MOTOR_COUNT,
    TENDON_LUT_MAX_SAMPLES,
    MOTOR_ABS_CMD_MIN,
    MOTOR_ABS_CMD_MAX,
    PROTO_ACK_STATUS_QUEUED,
    PROTO_ACK_STATUS_OK,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RUN_FEEDBACK,
)


def _trusted_multiturn_state(motor_idx: int) -> HandModel:
    state = HandModel()
    bit = 1 << motor_idx
    state.system_state = SYSTEM_STATE_IDLE
    state.servo_online_bitmap = bit
    state.has_servo_route_status = True
    state.servo_route_controllable_bitmap = bit
    state.multi_turn_unknown_bitmap = 0
    state.has_servo_angle_data = True
    state.has_servo_raw_data = True
    state.servo_angles = [0] * MOTOR_COUNT
    state.servo_raw_positions = [0] * MOTOR_COUNT
    state.servo_raw_online = [False] * MOTOR_COUNT
    state.servo_angles[motor_idx] = 8192
    state.servo_raw_positions[motor_idx] = 0
    state.servo_raw_online[motor_idx] = True
    return state


def test_calibrated_anchor_is_saved_only_after_explicit_capture_request(
    tmp_path, monkeypatch
):
    controller = HandController()
    controller._multiturn_store = MultiTurnRuntimeStore(
        tmp_path / "multiturn_runtime.json"
    )
    identity = {
        "product_fingerprint": {
            "schema_version": 1,
            "feedback_mapping_version": 3,
            "hardware_map_crc": 0x1234,
        }
    }
    controller._multiturn_identity = lambda _state: identity
    motor_idx = 18
    bit = 1 << motor_idx
    snapshot_path = tmp_path / "calibration_bundle.mmcal.json"
    results = [None] * ENCODER_COUNT
    results[14] = {
        "joint": 14,
        "success": True,
        "tendonLutValid": True,
        "tendonLutCount": 3,
        "tendonLutMotor": [motor_idx, 0xFF],
        "tendonLutMotorAbs": [[100, 200, 300], [0, 0, 0]],
    }
    atomic_write_snapshot(
        snapshot_path,
        merge_snapshot(
            None,
            {
                "active_joint_mask": 1 << 14,
                "active_motor_mask": bit,
                "motor_abs_limit_valid_bitmap": bit,
                "motor_abs_limit_min": [0] * MOTOR_COUNT,
                "motor_abs_limit_max": [0] * MOTOR_COUNT,
                "manual_reference_min_raw": [0] * ENCODER_COUNT,
                "manual_reference_max_raw": [0] * ENCODER_COUNT,
                "manual_reference_valid_bitmap": 0,
                "joint_calib_results": results,
            },
            updated_joint_mask=1 << 14,
        ),
    )
    monkeypatch.setattr(core_logic, "CALIB_LOAD_SNAPSHOT_PATH", snapshot_path)
    state = _trusted_multiturn_state(motor_idx)

    controller._block_multiturn_anchor_capture(bit)
    controller._persist_multiturn_state(state, force=True)
    payload = controller._multiturn_store.load(identity=identity)
    assert int(payload["valid_motor_mask"], 0) & bit == 0

    assert controller.request_multiturn_anchor_capture(bit)
    controller._service_multiturn_anchor_capture(state)

    payload = controller._multiturn_store.load(identity=identity)
    assert int(payload["valid_motor_mask"], 0) & bit
    assert controller._multiturn_anchor_capture_pending_mask == 0
    snapshot_mask, snapshot_abs, snapshot_raw = multiturn_anchor_fields(
        core_logic.read_snapshot(snapshot_path)
    )
    assert snapshot_mask & bit
    assert snapshot_abs[motor_idx] == 8192
    assert snapshot_raw[motor_idx] == 0


def test_calibrated_anchors_are_saved_per_eligible_motor(tmp_path, monkeypatch):
    controller = HandController()
    controller._multiturn_store = MultiTurnRuntimeStore(
        tmp_path / "multiturn_runtime.json"
    )
    identity = {
        "product_fingerprint": {
            "schema_version": 1,
            "feedback_mapping_version": 3,
            "hardware_map_crc": 0x1234,
        }
    }
    controller._multiturn_identity = lambda _state: identity
    motor_mask = (1 << 18) | (1 << 19)
    snapshot_path = tmp_path / "calibration_bundle.mmcal.json"
    results = [None] * ENCODER_COUNT
    for joint_idx, motor_idx in ((14, 18), (15, 19)):
        results[joint_idx] = {
            "joint": joint_idx,
            "success": True,
            "tendonLutValid": True,
            "tendonLutCount": 3,
            "tendonLutMotor": [motor_idx, 0xFF],
            "tendonLutMotorAbs": [[100, 200, 300], [0, 0, 0]],
        }
    atomic_write_snapshot(
        snapshot_path,
        merge_snapshot(
            None,
            {
                "active_joint_mask": (1 << 14) | (1 << 15),
                "active_motor_mask": motor_mask,
                "motor_abs_limit_valid_bitmap": motor_mask,
                "motor_abs_limit_min": [0] * MOTOR_COUNT,
                "motor_abs_limit_max": [0] * MOTOR_COUNT,
                "manual_reference_min_raw": [0] * ENCODER_COUNT,
                "manual_reference_max_raw": [0] * ENCODER_COUNT,
                "manual_reference_valid_bitmap": 0,
                "joint_calib_results": results,
            },
            updated_joint_mask=(1 << 14) | (1 << 15),
        ),
    )
    monkeypatch.setattr(core_logic, "CALIB_LOAD_SNAPSHOT_PATH", snapshot_path)
    state = _trusted_multiturn_state(18)

    assert controller.request_multiturn_anchor_capture(motor_mask)
    controller._service_multiturn_anchor_capture(state)
    assert controller._multiturn_anchor_capture_pending_mask == 1 << 19

    state.servo_online_bitmap = motor_mask
    state.servo_route_controllable_bitmap = motor_mask
    state.servo_angles[19] = 12288
    state.servo_raw_positions[19] = 0
    state.servo_raw_online[19] = True
    controller._service_multiturn_anchor_capture(state)

    assert controller._multiturn_anchor_capture_pending_mask == 0
    snapshot_mask, _snapshot_abs, _snapshot_raw = multiturn_anchor_fields(
        core_logic.read_snapshot(snapshot_path)
    )
    assert snapshot_mask & motor_mask == motor_mask


def test_startup_multiturn_recovery_waits_for_first_complete_route_scan():
    controller = HandController()

    ok, message = controller.wait_for_multiturn_recovery(1 << 18, timeout_s=0.01)

    assert ok is False
    assert "complete P4 servo feedback and route scan" in message


def test_startup_multiturn_recovery_defers_motor_offline_after_route_scan():
    controller = HandController()
    state = HandModel()
    state.has_servo_angle_data = True
    state.has_servo_raw_data = True
    state.has_servo_route_status = True
    state.servo_route_scan_state = 0
    state.servo_route_scan_progress = 100
    controller.current_state.copy_state_from(state)

    ok, message = controller.wait_for_multiturn_recovery(1 << 18, timeout_s=0.1)

    assert ok is True
    assert "offline" in message


def test_startup_multiturn_recovery_repairs_known_anchor_before_success():
    controller = HandController()
    motor_idx = 18
    bit = 1 << motor_idx
    state = _trusted_multiturn_state(motor_idx)
    state.servo_route_scan_state = 0
    state.servo_route_scan_progress = 100
    controller.current_state.copy_state_from(state)
    controller._multiturn_recovery_anchors = MagicMock(
        return_value=(0, [0] * MOTOR_COUNT, [0] * MOTOR_COUNT, True)
    )
    controller.multiturn_anchor_capture_status = MagicMock(
        side_effect=[(False, "anchor missing"), (True, "anchors saved")]
    )
    controller.request_multiturn_anchor_capture = MagicMock(return_value=True)

    ok, message = controller.wait_for_multiturn_recovery(bit, timeout_s=0.2)

    assert ok is True
    assert message == "multi-turn recovery confirmed"
    controller.request_multiturn_anchor_capture.assert_called_once_with(bit)


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


class TestAsyncPersistenceAck:
    """异步 Flash 命令必须在 QUEUED 后等待最终状态，不能误报完成。"""

    def test_manual_reference_accepts_queued_then_verifies_readback(
        self, monkeypatch
    ):
        controller = HandController()
        controller.comm = MagicMock()
        controller.comm.wait_for_ack.return_value = (
            CMD_MANUAL_ENCODER_REFERENCE,
            0,
            PROTO_ACK_STATUS_QUEUED,
            1.0,
        )
        controller._wait_for_manual_reference_commit_result = MagicMock(
            return_value=True
        )
        controller._verify_manual_encoder_readback = MagicMock(return_value=True)
        save_json = MagicMock()
        monkeypatch.setattr(
            "desktop.core.core_logic.update_manual_snapshot", save_json
        )

        mins = [100] * ENCODER_COUNT
        maxs = [200] * ENCODER_COUNT
        assert controller.set_manual_encoder_references(mins, maxs) is True
        controller._wait_for_manual_reference_commit_result.assert_called_once()
        controller._verify_manual_encoder_readback.assert_called_once()
        save_json.assert_called_once()

def test_host_multiturn_restore_uses_nearest_circle_for_each_motor(tmp_path):
    """固定 400-count 门禁已删除，正反向跨圈都应按最近圈独立恢复。"""
    controller = HandController()
    controller.comm = MagicMock()
    identity = {
        "product_fingerprint": {"schema_version": 1},
        "calibration_snapshot_sha256": "abc",
    }
    controller._multiturn_identity = MagicMock(return_value=identity)
    controller._multiturn_store = MultiTurnRuntimeStore(
        tmp_path / "multiturn_runtime.json"
    )
    controller._multiturn_store.save(
        identity=identity,
        valid_motor_mask=(1 << 0) | (1 << 1),
        logical_abs=[8182, 4106] + [0] * (MOTOR_COUNT - 2),
        raw=[4090, 10] + [0] * (MOTOR_COUNT - 2),
        p4_time_ms=10,
    )

    state = controller.current_state
    state.multi_turn_unknown_bitmap = (1 << 0) | (1 << 1)
    state.servo_online_bitmap = (1 << 0) | (1 << 1)
    state.has_servo_route_status = True
    state.servo_route_controllable_bitmap = (1 << 0) | (1 << 1)
    state.has_servo_raw_data = True
    state.servo_raw_online[0] = True
    state.servo_raw_online[1] = True
    state.servo_raw_positions[0] = 6
    state.servo_raw_positions[1] = 4090

    controller._service_multiturn_restore(state)

    command = controller.comm.send_command.call_args.args[0]
    assert command[0] == "restore_multiturn"
    assert command[1] == (1 << 0) | (1 << 1)
    assert command[2][0] == 8194
    assert command[2][1] == 4090
    assert controller._multiturn_pending_restore_mask == command[1]


class TestPrepareCalibrationTurnZero:
    def test_invalidates_only_successful_selected_motor_anchors(self, tmp_path, monkeypatch):
        """Calibrate 私有归零事务只清理成功电机，不能破坏其余 JSON 锚点。"""
        controller = HandController()
        controller.comm = MagicMock()
        controller._multiturn_store = MultiTurnRuntimeStore(
            tmp_path / "multiturn_runtime.json"
        )
        identity = {
            "product_fingerprint": {"schema_version": 1},
            "calibration_snapshot_sha256": "abc",
        }
        controller._multiturn_store_payload = controller._multiturn_store.save(
            identity=identity,
            valid_motor_mask=(1 << 2) | (1 << 3),
            logical_abs=[0, 0, 5000, 6000] + [0] * (MOTOR_COUNT - 4),
            raw=[0, 0, 904, 1904] + [0] * (MOTOR_COUNT - 4),
            p4_time_ms=10,
        )
        state = controller.current_state
        state.has_servo_reset_zero_status = True
        state.servo_online_bitmap = (1 << 2) | (1 << 3)
        state.servo_reset_zero_status = 0
        state.servo_reset_zero_result_token = 5
        monkeypatch.setattr(core_logic.time, "sleep", lambda _seconds: None)

        def wait_for_ack(cmd, _timeout, received_after=None):
            del received_after
            if cmd == CMD_RESET:
                return (cmd, 0, PROTO_ACK_STATUS_QUEUED, 0.5)
            if cmd == CMD_SERVO_RESET_ZERO:
                state.servo_reset_zero_result_token = 6
                state.servo_reset_zero_status = 2
                state.servo_reset_zero_success_mask = 1 << 2
                state.servo_reset_zero_failed_mask = 0
                return (cmd, 1, PROTO_ACK_STATUS_OK, 1.0)
            return None

        controller.comm.wait_for_ack.side_effect = wait_for_ack
        ok, _message, success_mask, failed_mask = (
            controller.prepare_calibration_turn_zero(1 << 2)
        )

        assert ok is True
        assert success_mask == 1 << 2
        assert failed_mask == 0
        payload = controller._multiturn_store.load(identity=identity)
        assert int(payload["valid_motor_mask"], 0) == 1 << 3
        assert call("reset") in controller.comm.send_command.call_args_list
        assert call(("servo_reset_zero", 1 << 2)) in controller.comm.send_command.call_args_list

    def test_manual_prepare_freezes_only_online_motors(self):
        """离线通道不加入 Release Rope 的冻结集合。"""
        controller = HandController()
        controller.comm = MagicMock()
        controller.prepare_calibration_turn_zero = MagicMock(return_value=(False, "reset rejected", 0, 0))
        state = controller.current_state
        state.system_state = 0
        state.has_servo_angle_data = True
        state.has_servo_route_status = True
        all_motors = (1 << MOTOR_COUNT) - 1
        state.servo_abs_last_rx_ts = core_logic.time.time()
        state.servo_online_bitmap = all_motors & ~(1 << 24)
        state.servo_route_controllable_bitmap = all_motors

        result = controller.manual_prepare_all_motors()

        assert result.ok is False
        assert "reset rejected" in result.message
        controller.prepare_calibration_turn_zero.assert_called_once_with(
            all_motors & ~(1 << 24), invalidation_reason="manual_prepare"
        )





class TestLoadCalibrationSnapshot:
    def test_manual_restore_entry_is_independent_from_auto_load(self, monkeypatch):
        """P4 重启后应能先恢复 Manual，不能被 Auto/LUT 的后续校验阻断。"""
        controller = HandController()
        snapshot = {"format": "mmhand_calibration_bundle"}
        validate = MagicMock()
        restore = MagicMock(return_value=(True, "restored 4 Manual joint(s)"))
        monkeypatch.setattr(core_logic, "validate_snapshot", validate)
        monkeypatch.setattr(controller, "_restore_snapshot_manual_references", restore)

        result = controller.restore_manual_references_from_snapshot(snapshot)

        assert result == (True, "restored 4 Manual joint(s)")
        validate.assert_called_once_with(snapshot)
        restore.assert_called_once_with(snapshot)

    def test_each_joint_waits_for_an_ack_newer_than_its_own_send(self, monkeypatch):
        controller = HandController()
        controller.comm = MagicMock()
        controller.query_manual_encoder_references = MagicMock(return_value=True)
        controller.current_state.manual_reference_committed = True
        controller.current_state.manual_reference_valid_bitmap = (1 << 9) | (1 << 10)
        controller.current_state.manual_reference_min_raw = [1000] * ENCODER_COUNT
        controller.current_state.manual_reference_max_raw = [5000] * ENCODER_COUNT

        mapping_version = 4
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
            urdf_min, urdf_max = get_spec(joint).urdf_range_deg
            joint_results[joint] = {
                "success": True,
                "mappingVersion": mapping_version,
                "urdfConfigCrc": urdf_crc,
                "manualReferenceCrc": manual_crc,
                "angleMin": urdf_min,
                "angleMax": urdf_max,
                "currentMinDeg": urdf_min,
                "currentMaxDeg": urdf_max,
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
            "schema_version": 6,
            "hand_profile": "v2",
            "active_joint_mask": (1 << 9) | (1 << 10),
            "active_motor_mask": (1 << 12) | (1 << 13),
            "motor_abs_limit_valid_bitmap": (1 << 12) | (1 << 13),
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


class TestShutdownSemantics:
    """普通退出依赖 P4 租约 Hold，显式急停才允许发送锁存式 STOP。"""

    @staticmethod
    def _controller_with_comm():
        controller = HandController()
        controller.comm = MagicMock()
        controller._update_thread = None
        return controller

    def test_default_shutdown_does_not_latch_lower_estop(self, monkeypatch):
        controller = self._controller_with_comm()
        monkeypatch.setattr(core_logic.time, "sleep", lambda _seconds: None)

        controller.shutdown()

        controller.comm.send_command.assert_not_called()
        controller.comm.disconnect.assert_called_once_with()

    def test_explicit_stop_lower_still_sends_estop(self, monkeypatch):
        controller = self._controller_with_comm()
        monkeypatch.setattr(core_logic.time, "sleep", lambda _seconds: None)

        controller.shutdown(stop_lower=True)

        controller.comm.send_command.assert_called_once_with("stop")
        controller.comm.disconnect.assert_called_once_with()
