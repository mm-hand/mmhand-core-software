"""Snapshot completeness, local target ownership and nested mutable isolation."""
from dataclasses import fields
import pickle
import numpy as np
from desktop.core.data_models import HandModel, FingerTactile, TactileSensor, clone_hand_model


def test_every_model_field_copies_while_local_targets_and_zeros_remain():
    source, target = HandModel(), HandModel()
    for item in fields(source):
        if item.name in ('encoders', 'motors'):
            continue
        value = getattr(source, item.name)
        if isinstance(value, bool): value = not value
        elif isinstance(value, int): value += 7
        elif isinstance(value, float): value += 1.25
        elif isinstance(value, str): value += '-test'
        elif isinstance(value, dict): value = {7: {'sample': [1, 2]}}
        elif isinstance(value, list): value = value + [None]
        setattr(source, item.name, value)
    source.encoders[0].raw = 42
    source.encoders[0].target_angle_deg = 15
    source.encoders[0].zero_raw = 13
    target.encoders[0].target_angle_deg = 25
    target.encoders[0].zero_raw = 23
    encoder = target.encoders[0]
    target.copy_state_from(source)
    for item in fields(source):
        if item.name not in ('encoders', 'motors'):
            assert pickle.dumps(getattr(target, item.name)) == pickle.dumps(getattr(source, item.name)), item.name
    assert target.encoders[0] is encoder
    assert (encoder.raw, encoder.target_angle_deg, encoder.zero_raw) == (42, 25, 23)
    assert clone_hand_model(source).target_angles == source.target_angles
    assert clone_hand_model(source).encoders[0].zero_raw == 13


def test_nested_calibration_history_and_tactile_arrays_are_isolated():
    source = HandModel()
    source.joint_calib_results[0] = {'lut': {'points': [1, 2]}}
    source.task_runtime = {0: {'nested': [1]}}
    source.proto_ack_history = [{'nested': [3]}]
    source.tactile_data = [FingerTactile([TactileSensor(np.ones((5, 5, 3)), np.ones(3))])]
    target = clone_hand_model(source)
    target.joint_calib_results[0]['lut']['points'][0] = 99
    target.task_runtime[0]['nested'][0] = 99
    target.proto_ack_history[0]['nested'][0] = 99
    target.tactile_data[0].sensors[0].contact_forces[0, 0, 0] = 99
    target.tactile_data[0].sensors[0].resultant[0] = 99
    assert source.joint_calib_results[0]['lut']['points'][0] == 1
    assert source.task_runtime[0]['nested'][0] == 1
    assert source.proto_ack_history[0]['nested'][0] == 3
    assert source.tactile_data[0].sensors[0].contact_forces[0, 0, 0] == 1
    assert source.tactile_data[0].sensors[0].resultant[0] == 1


def test_copy_state_from_subset_only_copies_requested_fields():
    source, target = HandModel(), HandModel()
    source.p4_time_ms = 7
    source.calib_status = 'SUCCESS'
    source.servo_angles[0] = 5
    source.encoders[0].raw = 42
    source.encoders[0].target_angle_deg = 15
    source.encoders[0].zero_raw = 13
    target.encoders[0].target_angle_deg = 25
    target.encoders[0].zero_raw = 23
    target.motors[0].motor_id = 0

    target.copy_state_from(source, field_names=('p4_time_ms',))
    assert target.p4_time_ms == 7
    assert target.calib_status == 'IDLE'
    assert target.servo_angles[0] == 0
    # 未显式请求 encoders/motors 时保持本地值不变。
    assert target.encoders[0].raw == 0
    assert target.encoders[0].target_angle_deg == 25

    target.copy_state_from(source, field_names=('servo_angles', 'encoders'))
    assert target.servo_angles == source.servo_angles
    assert target.servo_angles is not source.servo_angles
    assert target.encoders[0].raw == 42
    # 本地目标角与零点仍由本对象拥有。
    assert target.encoders[0].target_angle_deg == 25
    assert target.encoders[0].zero_raw == 23

    # 默认路径（field_names=None）保持全量拷贝语义。
    target.copy_state_from(source)
    assert target.calib_status == 'SUCCESS'
    assert target.encoders[0].raw == 42
    assert target.encoders[0].target_angle_deg == 25


def test_clone_hand_model_snapshot_does_not_alias_working_containers():
    working = HandModel()
    working.servo_angles[0] = 11
    working.encoders[0].raw = 42
    snapshot = clone_hand_model(working)
    assert snapshot.servo_angles == working.servo_angles
    assert snapshot.servo_angles is not working.servo_angles
    assert snapshot.encoders[0] is not working.encoders[0]

    working.servo_angles[0] = 22
    working.encoders[0].raw = 99
    working.monotonic_received['encoder_raw_last_rx_ts'] = 1.0
    assert snapshot.servo_angles[0] == 11
    assert snapshot.encoders[0].raw == 42
    assert snapshot.monotonic_received == {}
