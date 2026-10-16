"""Connection identity changes must preserve Connect and explicit STOP semantics."""

import threading
import time
from unittest.mock import Mock

import pytest

from desktop.core.data_models import HandModel
from desktop.core.protocol import HOST_PROTOCOL_VERSION, SYSTEM_STATE_READY
from test_application_actions import hand, wait_operation


DEVICE_A = "000080F1B2D068EB"
DEVICE_B = "000080F1B2D068EC"


def publish_state(hand, device_id=None):
    """Publish through the real backend callback without opening a serial port."""
    state = HandModel()
    state.has_system_status_data = True
    state.system_status_last_rx_ts = time.time()
    state.system_state = SYSTEM_STATE_READY
    state.protocol_compatible = True
    if device_id is not None:
        state.has_product_fingerprint = True
        state.product_fingerprint = {
            "protocol_version": HOST_PROTOCOL_VERSION,
            "device_id": device_id,
        }
    hand._controller.current_state = state
    for callback in tuple(hand._controller.update_callbacks):
        callback(state)


@pytest.mark.parametrize(
    "connections",
    [
        [("COM13", DEVICE_A)],
        [("COM13", DEVICE_A), ("COM13", DEVICE_A)],
        [("COM13", DEVICE_A), ("COM14", DEVICE_B)],
    ],
    ids=["first-connect", "same-device-reconnect", "different-device-port"],
)
def test_connect_accepts_identity_received_during_initialization(
    hand, monkeypatch, connections
):
    identities = iter(device_id for _port, device_id in connections)

    def initialize(port):
        publish_state(hand)
        publish_state(hand, next(identities))
        return True

    initializer = Mock(side_effect=initialize)
    disconnect = Mock()
    monkeypatch.setattr(hand._controller, "initialize", initializer)
    monkeypatch.setattr(hand, "disconnect", disconnect)

    for port, device_id in connections:
        operation = wait_operation(hand, hand.request_connect(port))
        assert operation["status"] == "SUCCESS", operation["message"]
        assert hand.get_state_snapshot().product_fingerprint["device_id"] == device_id
    assert [call.args[0] for call in initializer.call_args_list] == [
        port for port, _device_id in connections
    ]
    disconnect.assert_not_called()


@pytest.mark.parametrize("stop_after_identity", [False, True], ids=["before-id", "after-id"])
def test_stop_during_connect_still_cancels_and_disconnects(
    hand, monkeypatch, stop_after_identity
):
    entered, release = threading.Event(), threading.Event()

    def initialize(port):
        publish_state(hand)
        if stop_after_identity:
            publish_state(hand, DEVICE_A)
        entered.set()
        assert release.wait(2), "test did not release connection initialization"
        if not stop_after_identity:
            publish_state(hand, DEVICE_A)
        return True

    disconnect = Mock()
    monkeypatch.setattr(hand._controller, "initialize", initialize)
    monkeypatch.setattr(hand, "disconnect", disconnect)
    result = hand.request_connect("COM13")
    try:
        assert result.accepted, result.message
        assert entered.wait(1), "connection worker did not start"
        hand.stop_motion()
        hand.stop.assert_called_once()
        disconnect.assert_not_called()
    finally:
        release.set()

    operation = wait_operation(hand, result)
    assert operation["status"] == "FAILED"
    assert "cancelled" in operation["message"].lower()
    disconnect.assert_called_once_with(stop_lower=True)


def test_device_change_outside_connect_cancels_operation_and_clears_drafts(
    hand, monkeypatch
):
    def initialize(port):
        publish_state(hand)
        publish_state(hand, DEVICE_A)
        return True

    monkeypatch.setattr(hand._controller, "initialize", initialize)
    monkeypatch.setattr(hand, "disconnect", Mock())
    connection = wait_operation(hand, hand.request_connect("COM13"))
    assert connection["status"] == "SUCCESS", connection["message"]

    entered, release = threading.Event(), threading.Event()

    def observe(epoch):
        entered.set()
        assert release.wait(2), "test did not release observation"
        return True, "observation complete"

    result = hand.submit_operation("Read device state", observe, requires_device=False)
    try:
        assert result.accepted, result.message
        assert entered.wait(1), "observation worker did not start"
        hand._manual_drafts[0] = {"min": 12000}
        hand._manual_saved_pairs[0] = (12000, 11000)
        hand._import_previews["preview"] = {"name": "device-a.mmcal.json"}
        publish_state(hand, DEVICE_B)
        assert hand._manual_drafts == {}
        assert hand._manual_saved_pairs == {}
        assert hand._import_previews == {}
    finally:
        release.set()

    operation = wait_operation(hand, result)
    assert operation["status"] == "FAILED"
    assert "cancelled" in operation["message"].lower()


@pytest.mark.parametrize("known_device", [False, True], ids=["new-id", "same-as-history"])
@pytest.mark.parametrize("replacement", [DEVICE_B, None], ids=["other-device", "identity-lost"])
def test_identity_change_after_confirmation_cancels_same_connect(
    hand, monkeypatch, known_device, replacement
):
    disconnect = Mock()
    monkeypatch.setattr(hand, "disconnect", disconnect)

    if known_device:
        def initial_connection(port):
            publish_state(hand)
            publish_state(hand, DEVICE_A)
            return True

        monkeypatch.setattr(hand._controller, "initialize", initial_connection)
        operation = wait_operation(hand, hand.request_connect("COM13"))
        assert operation["status"] == "SUCCESS", operation["message"]

    def initialize(port):
        if not known_device:
            publish_state(hand)
        publish_state(hand, DEVICE_A)
        publish_state(hand, replacement)
        return True

    monkeypatch.setattr(hand._controller, "initialize", initialize)
    operation = wait_operation(hand, hand.request_connect("COM13"))
    assert operation["status"] == "FAILED"
    assert "cancelled" in operation["message"].lower()
    disconnect.assert_called_once_with(stop_lower=True)


def test_stop_during_connect_cancels_and_disconnects(hand, monkeypatch):
    entered, release = threading.Event(), threading.Event()

    def initialize(port):
        publish_state(hand)
        publish_state(hand, DEVICE_A)
        entered.set()
        assert release.wait(2), "test did not release connection initialization"
        return True

    disconnect = Mock()
    monkeypatch.setattr(hand._controller, "initialize", initialize)
    monkeypatch.setattr(hand, "disconnect", disconnect)
    result = hand.request_connect("COM13")
    try:
        assert result.accepted, result.message
        assert entered.wait(1), "connection worker did not start"
        assert hand.stop_motion().accepted
    finally:
        release.set()

    operation = wait_operation(hand, result)
    assert operation["status"] == "FAILED"
    assert "cancelled" in operation["message"].lower()
    disconnect.assert_called_once_with(stop_lower=True)


def test_identity_arriving_after_connect_cancels_next_device_operation_only(
    hand, monkeypatch
):
    def initialize(port):
        publish_state(hand)
        return True

    disconnect = Mock()
    monkeypatch.setattr(hand._controller, "initialize", initialize)
    monkeypatch.setattr(hand, "disconnect", disconnect)
    connection = wait_operation(hand, hand.request_connect("COM13"))
    assert connection["status"] == "SUCCESS", connection["message"]

    entered, release = threading.Event(), threading.Event()

    def observe(epoch):
        entered.set()
        assert release.wait(2), "test did not release observation"
        return True, "observation complete"

    result = hand.submit_operation("Read device state", observe)
    try:
        assert result.accepted, result.message
        assert entered.wait(1), "observation worker did not start"
        publish_state(hand, DEVICE_A)
    finally:
        release.set()

    operation = wait_operation(hand, result)
    assert operation["status"] == "FAILED"
    assert "cancelled" in operation["message"].lower()
    assert hand.get_state_snapshot().product_fingerprint["device_id"] == DEVICE_A
    disconnect.assert_not_called()


def test_connect_exception_does_not_mask_next_connection_identity(hand, monkeypatch):
    def failed_initialize(port):
        publish_state(hand)
        publish_state(hand, DEVICE_A)
        raise RuntimeError("simulated connection initialization failure")

    disconnect = Mock()
    monkeypatch.setattr(hand, "disconnect", disconnect)
    monkeypatch.setattr(hand._controller, "initialize", failed_initialize)
    failed = wait_operation(hand, hand.request_connect("COM13"))
    assert failed["status"] == "FAILED"
    assert "simulated connection initialization failure" in failed["message"]

    def initialize(port):
        publish_state(hand)
        publish_state(hand, DEVICE_B)
        return True

    monkeypatch.setattr(hand._controller, "initialize", initialize)
    connected = wait_operation(hand, hand.request_connect("COM14"))
    assert connected["status"] == "SUCCESS", connected["message"]
    assert hand.get_state_snapshot().product_fingerprint["device_id"] == DEVICE_B
    disconnect.assert_not_called()


def test_shutdown_allows_controller_callback_to_finish(hand, monkeypatch):
    hand.register_update_callback(lambda state: None)
    publish_state(hand, DEVICE_A)
    callback_threads = []

    def shutdown(stop_lower=False):
        worker = threading.Thread(target=publish_state, args=(hand, DEVICE_B))
        callback_threads.append(worker)
        worker.start()
        worker.join(0.5)
        assert not worker.is_alive(), "shutdown held the output lock while joining its callback"

    monkeypatch.setattr(hand._controller, "shutdown", shutdown)
    try:
        hand.shutdown(stop_lower=True)
    finally:
        for worker in callback_threads:
            worker.join(1)


@pytest.mark.parametrize("cancellation", ["stop", "device-change"])
def test_connect_cancelled_during_snapshot_load_disconnects_once(
    hand, monkeypatch, cancellation
):
    def initialize(port):
        publish_state(hand)
        publish_state(hand, DEVICE_A)
        return True

    entered, release = threading.Event(), threading.Event()

    def load():
        entered.set()
        assert release.wait(2), "test did not release snapshot load"
        return {}

    disconnect = Mock()
    monkeypatch.setattr(hand._controller, "initialize", initialize)
    monkeypatch.setattr(hand._calibration_workflow._store, "load", load)
    monkeypatch.setattr(hand, "disconnect", disconnect)
    result = hand.request_connect("COM13")
    try:
        assert result.accepted, result.message
        assert entered.wait(1), "connection did not reach snapshot load"
        if cancellation == "stop":
            hand.stop_motion()
        else:
            publish_state(hand, DEVICE_B)
        disconnect.assert_not_called()
    finally:
        release.set()

    operation = wait_operation(hand, result)
    assert operation["status"] == "FAILED"
    assert "cancelled" in operation["message"].lower()
    disconnect.assert_called_once_with(stop_lower=True)
