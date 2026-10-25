"""Calibration completion retains final failure and warning details for the UI."""
import pytest

from test_application_actions import hand
from desktop.core.calibration_workflow import CalibrationWorkflowEvent


def begin_operation(hand, operation_id='calibration-1'):
    hand._calibration_operation_id = hand._application_busy = operation_id
    item = dict(operation_id=operation_id, name='Selected-joint Calibration', time='12:00:00',
                scope='Joints 0x00000F', status='RUNNING', message='Calibrating', phase='prepare',
                origin='Motor Calibration', finished_at=None, alert_acknowledged=False)
    hand._application_operations.append(item)
    return item


def completion(hand, *, ok=True, message='all calibration joints completed', joint_mask=15, **details):
    hand._application_calibration_event(CalibrationWorkflowEvent(
        1, 'calibration_complete', ok, message, joint_mask=joint_mask, details=details))


@pytest.mark.parametrize('terminal', ['FAILED', 'HOST_FAILED', 'HOST_ERROR'])
def test_failed_calibration_keeps_actual_root_cause_and_failed_joints(hand, terminal):
    item = begin_operation(hand)
    reason = 'J03: servo target was rejected because feedback was lost'
    completion(hand, ok=False, message=reason, terminal_status=terminal,
               completed_joint_mask=7, failed_joint_mask=8)

    assert item['status'] == 'FAILED'
    assert item['message'] == reason
    alert = hand.application_snapshot()['operations'][-1]['calibration_alert']
    assert alert['title'] == 'Selected-joint Calibration failed'
    assert alert['severity'] == 'negative'
    assert reason in alert['message']
    assert 'Failed joints: J03' in alert['message']
    assert 'J00' not in alert['message']
    assert hand._calibration_operation_id is None
    assert hand._application_busy is None


def test_incomplete_selected_joints_are_not_reported_as_failed(hand):
    """准备/中断阶段没开始的关节不是“失败”，也不再列成长名单。"""
    item = begin_operation(hand)
    completion(hand, ok=False, message='calibration interrupted', terminal_status='HOST_FAILED',
               joint_mask=(1 << 1) | (1 << 4), completed_joint_mask=1 << 1)
    message = item['calibration_alert']['message']
    assert 'calibration interrupted' in message
    assert 'Failed joints' not in message
    assert 'Please try again.' in message


def test_failure_includes_record_joint_and_retains_successful_joint_warning(hand):
    item = begin_operation(hand)
    records = [
        dict(status='FAILED', joint_mask=8),
        dict(status='WARN', joint_mask=1, phase_index=0, phase_count=2,
             warning_summary=dict(ready=True, message='J00: excessive tendon friction')),
    ]
    completion(hand, ok=False, message='J03: no magnetic encoder feedback',
               terminal_status='HOST_FAILED', completed_joint_mask=7, records=records)
    message = item['calibration_alert']['message']
    assert 'J03: no magnetic encoder feedback' in message
    assert 'Failed joints: J03' in message
    assert 'J00: excessive tendon friction' in message
    assert 'Phase 1/2' in message


def test_warning_completion_aggregates_each_record_summary(hand):
    item = begin_operation(hand)
    records = [dict(status='WARN', joint_mask=1 << j, phase_index=i, phase_count=2,
                    warning_summary=dict(ready=True, message=f'J{j:02d}: excessive tendon friction'))
               for i, j in enumerate((0, 20))]
    completion(hand, terminal_status='WARN', records=records)
    alert = item['calibration_alert']
    assert item['status'] == 'SUCCESS'
    assert alert['title'] == 'Selected-joint Calibration completed with warnings'
    assert alert['severity'] == 'warning'
    assert 'all calibration joints completed' in alert['message']
    for text in ('Phase 1/2', 'Phase 2/2', 'J00: excessive tendon friction', 'J20: excessive tendon friction'):
        assert text in alert['message']


@pytest.mark.parametrize('terminal,records', [
    ('WARN', []),
    ('WARN', [dict(status='WARN', warning_summary=None)]),
    ('SUCCESS', [dict(status='WARN', warning_summary=dict(ready=False))]),
])
def test_warning_still_alerts_when_summary_is_unavailable(hand, terminal, records):
    item = begin_operation(hand)
    completion(hand, terminal_status=terminal, records=records)
    alert = item['calibration_alert']
    assert alert['severity'] == 'warning'
    assert 'Calibration ended with warnings' in alert['message']


def test_warning_summary_is_not_suppressed_by_success_terminal(hand):
    item = begin_operation(hand)
    completion(hand, terminal_status='SUCCESS', records=[dict(
        status='SUCCESS', warning_summary=dict(ready=True, message='J04: high friction'))])
    assert item['calibration_alert']['severity'] == 'warning'
    assert 'J04: high friction' in item['calibration_alert']['message']


def test_alert_acknowledgement_is_idempotent_and_stored(hand):
    item = begin_operation(hand)
    completion(hand, ok=False, terminal_status='HOST_FAILED',
               failed_joint_mask=8, message='J03 failed')
    assert item['alert_acknowledged'] is False
    assert item['finished_at'] is not None
    first = hand.acknowledge_calibration_alert(item['operation_id'])
    second = hand.acknowledge_calibration_alert(item['operation_id'])
    assert first.accepted and second.accepted
    assert item['alert_acknowledged'] is True
    snapshot = hand.application_snapshot()['operations'][-1]
    assert snapshot['alert_acknowledged'] is True
    assert snapshot['origin'] == 'Motor Calibration'


def test_pure_successful_calibration_has_no_alert(hand):
    item = begin_operation(hand)
    completion(hand, terminal_status='SUCCESS', records=[dict(status='SUCCESS', joint_mask=15)])
    assert item['status'] == 'SUCCESS'
    assert 'calibration_alert' not in item


def test_phase_warning_waits_for_completion_and_duplicate_terminal_is_ignored(hand):
    item = begin_operation(hand)
    hand._application_calibration_event(CalibrationWorkflowEvent(
        1, 'phase_complete', True, 'J00 calibration ended with WARN', joint_mask=1,
        details=dict(status='WARN')))
    assert 'calibration_alert' not in item
    assert item['status'] == 'RUNNING'
    completion(hand, terminal_status='WARN')
    alert = dict(item['calibration_alert'])
    completion(hand, ok=False, terminal_status='HOST_ERROR', message='late duplicate event')
    assert item['calibration_alert'] == alert
    assert item['status'] == 'SUCCESS'


def test_alert_snapshot_cannot_mutate_stored_completion(hand):
    item = begin_operation(hand)
    completion(hand, terminal_status='WARN')
    hand.application_snapshot()['operations'][-1]['calibration_alert']['message'] = 'UI draft'
    assert item['calibration_alert']['message'] != 'UI draft'
