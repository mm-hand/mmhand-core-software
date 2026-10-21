"""Regression for the 2026-10-22 root cause: an uncalibrated device has no
calibration bundle, and anchor invalidation must tolerate the missing file.

Before the fix, the EE turn-zero result made _update_loop crash inside
_service_servo_turn_zero_result before current_state.copy_state_from ran, so
current_state froze, the Release wait never saw the result and timed out, and
the unresolved mask stayed latched until an app restart."""

import queue
import threading
import time
from unittest.mock import Mock

from _clock_helpers import stamp

from desktop.core import core_logic
from desktop.core.calibration_snapshot import invalidate_snapshot_multiturn_anchors
from desktop.core.core_logic import HandController
from desktop.core.protocol import (
    CMD_RESET,
    CMD_SERVO_RESET_ZERO,
    HOST_PROTOCOL_VERSION,
    PROTO_ACK_STATUS_QUEUED,
    SYSTEM_STATE_IDLE,
)

MASK = 0b111


def make_controller(monkeypatch, tmp_path):
    controller = HandController()
    controller.comm = Mock()
    controller.comm.send_command.return_value = True
    controller.comm.drain_ack_events.return_value = []
    controller.comm.drain_safety_events.return_value = []
    controller.comm.rx_queue = queue.Queue()
    state = controller.current_state
    state.has_product_fingerprint = True
    state.product_fingerprint = {
        "device_id": "0000123456789ABC",
        "protocol_version": HOST_PROTOCOL_VERSION,
    }
    state.has_system_status_data = state.has_servo_route_status = True
    state.system_state = SYSTEM_STATE_IDLE
    state.system_status_last_rx_ts = time.time()
    stamp(state, "system_status_last_rx_ts")
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = MASK
    state.has_servo_rebase_zero_result = state.has_servo_reset_zero_status = True
    state.servo_rebase_zero_result_token = 7
    state.servo_reset_zero_request_token = state.servo_reset_zero_result_token = 7
    state.has_servo_angle_data = state.has_servo_raw_data = True
    monkeypatch.setattr(core_logic, "_active_urdf_repo_path", lambda: "tests://repo/model.urdf")
    monkeypatch.setattr(core_logic, "CALIB_ACTIVE_SNAPSHOT_PATH", tmp_path / "missing.mmcal.json")
    monkeypatch.setattr(core_logic, "CALIBRATION_TURN_RESET_SETTLE_S", 0)
    monkeypatch.setattr(core_logic, "SERVO_RESET_ZERO_ACK_TIMEOUT_S", 1.0)
    monkeypatch.setattr(core_logic, "SERVO_RESET_ZERO_RESULT_TIMEOUT_S", 1.0)
    return controller


def publish_full_success(controller, mask):
    state = controller.current_state
    token = state.servo_reset_zero_result_token + 1
    state.servo_reset_zero_request_token = state.servo_reset_zero_result_token = token
    state.servo_reset_zero_success_mask = mask
    state.servo_reset_zero_failed_mask = 0
    state.servo_reset_zero_status = 2
    state.system_status_last_rx_ts = time.time()
    stamp(state, "system_status_last_rx_ts")


def test_multiturn_anchor_invalidation_tolerates_missing_snapshot(tmp_path):
    missing = tmp_path / "missing.mmcal.json"
    result = invalidate_snapshot_multiturn_anchors(
        missing, motor_mask=MASK, reason="manual_prepare",
    )
    assert result is None


def test_turn_zero_succeeds_without_calibration_bundle(monkeypatch, tmp_path):
    controller = make_controller(monkeypatch, tmp_path)
    assert not controller.calibration_snapshot_path.exists()

    def ack(command, _timeout, received_after_monotonic):
        if command == CMD_RESET:
            return command, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic
        publish_full_success(controller, MASK)
        return command, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_QUEUED, received_after_monotonic

    controller.comm.wait_for_ack.side_effect = ack

    ok, message, success, failed = controller.prepare_calibration_turn_zero(MASK)

    assert ok is True, message
    assert success == MASK
    assert failed == 0
    assert controller._servo_turn_zero_unresolved_mask == 0


def test_update_loop_publishes_state_when_bookkeeping_raises(monkeypatch, tmp_path):
    controller = make_controller(monkeypatch, tmp_path)
    controller.running = True
    controller._service_servo_turn_zero_result = Mock(
        side_effect=RuntimeError("bookkeeping boom")
    )
    controller._queue_multiturn_persistence = Mock()

    model = controller._coordinate_state()
    model.system_state = 99
    controller.comm.rx_queue.put(model)

    thread = threading.Thread(target=controller._update_loop, daemon=True)
    thread.start()
    deadline = time.time() + 2.0
    while time.time() < deadline and controller.current_state.system_state != 99:
        time.sleep(0.01)
    controller.running = False
    thread.join(timeout=2.0)

    assert controller.current_state.system_state == 99
