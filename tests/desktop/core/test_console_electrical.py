"""Snapshot attribution: a dead CAN link must not be reported as a broken encoder."""
import time


def _connected(hand):
    hand._connected_at_monotonic = time.monotonic() - 10  # past the 3 s connect grace period
    for encoder in hand._controller.current_state.encoders:
        encoder.raw_error = False
        encoder.raw_fault = ''
        encoder.raw_sensor = 12000


def test_fresh_raw_frames_mean_can_ok_and_encoders_ok(hand):
    _connected(hand)
    snapshot = hand.application_snapshot(include_curves=False)
    assert snapshot['can']['label'] == 'CAN: OK'
    assert all(row['encoder_state'] == 'ok' for row in snapshot['joints'])


def test_missing_raw_frames_mean_every_joint_is_can_error(hand):
    _connected(hand)
    hand._controller.current_state.encoder_raw_last_rx_ts = time.time() - 5
    snapshot = hand.application_snapshot(include_curves=False)
    assert snapshot['can']['label'] == 'CAN: ERROR'
    assert all(row['encoder_state'] == 'can_error' for row in snapshot['joints'])
    assert any(issue['code'] == 'HOST_ELECTRICS_CAN' for issue in snapshot['user_issues'])


def test_invalid_encoder_is_per_joint(hand):
    _connected(hand)
    hand._controller.current_state.encoders[5].raw_error = True
    snapshot = hand.application_snapshot(include_curves=False)
    assert snapshot['can']['label'] == 'CAN: OK'
    assert snapshot['joints'][5]['encoder_state'] == 'error'
    assert snapshot['joints'][4]['encoder_state'] == 'ok'


def test_open_and_short_come_from_the_can_sentinel(hand):
    _connected(hand)
    state = hand._controller.current_state
    state.encoders[0].raw_fault = 'open'
    state.encoders[4].raw_fault = 'short'
    snapshot = hand.application_snapshot(include_curves=False)
    assert snapshot['joints'][0]['encoder_state'] == 'open'
    assert snapshot['joints'][4]['encoder_state'] == 'short'
    assert any(issue['code'] == 'HOST_ENCODER_WIRING' for issue in snapshot['user_issues'])
