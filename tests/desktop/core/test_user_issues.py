"""Guidance preserves evidence and never mistakes event history for live truth."""
import json
from pathlib import Path

import pytest

from desktop.core.data_models import HandModel
from desktop.core.safety_rules import safety_rule_table
from desktop.core.user_issues import (
    IssueMessage, guidance, host_issue, operation_issue, rule_issue, snapshot_issues,
)


def test_every_active_rule_has_plain_english_actionable_guidance():
    table = safety_rule_table()
    assert len(table) == 73
    assert set(guidance()['rules']) == {str(i) for i in table}
    for rule_id in table:
        issue = rule_issue(rule_id)
        assert issue.title and issue.impact and 1 <= len(issue.steps) <= 3
        assert all(step.strip() for step in issue.steps)
        assert issue.code == table[rule_id]['code']
        assert issue.details['rule_id'] == rule_id
        assert issue.status == 'History'


def test_all_host_calibration_diagnostics_have_guidance():
    from desktop.core.calibration_diagnostics import CALIBRATION_DIAGNOSTICS
    assert {f'HOST_CALIB_{r}' for r in CALIBRATION_DIAGNOSTICS} <= set(guidance()['host'])
    for reason in CALIBRATION_DIAGNOSTICS:
        issue = operation_issue('raw calibration evidence', diagnostic_reason=reason)
        assert issue.code == f'HOST_CALIB_{reason}'
        assert 1 <= len(issue.steps) <= 3
    assert 'two turns' in operation_issue('', diagnostic_reason=27).title
    assert 'exchange' in operation_issue('', diagnostic_reason=34).title


def test_unknown_failure_does_not_claim_a_physical_cause_or_encourage_blind_retry():
    message = 'servo rebase rejected: status=142'
    issue = operation_issue(message)
    assert issue.code == 'HOST_ACTION_FAILED'
    assert issue.details['original_message'] == message
    assert 'Please try again' not in issue.text()
    # A suggestion to check a latch is not confirmation of a latch.
    assert operation_issue('Check the P4 rejection reason; clear a latched STOP explicitly with Clear Faults.').code == 'HOST_ACTION_FAILED'


def test_explicit_stop_rejection_uses_the_rule_guidance():
    issue = operation_issue(IssueMessage('F1 rejected status=9', 'ACT-1005'))
    assert issue.code == 'ACT-1005'
    assert 'clear of obstacles' in issue.text() and 'Retry the action' in issue.text()
    assert issue.details['original_message'] == 'F1 rejected status=9'
    assert operation_issue(IssueMessage('reason=3', 'ROP-3403')).code == 'ROP-3403'


@pytest.mark.parametrize('message,code', [
    ('servo ABS rebase failed: M00', 'HOST_ABS_REBASE_FAILED'),
    ('selected servo feedback recovery failed: offline=0x20', 'HOST_FEEDBACK_RECOVERY_FAILED'),
    ('servo ABS rebase result timed out; original transaction remains unresolved', 'HOST_RESULT_UNKNOWN'),
    ('servo ABS rebase partly completed', 'HOST_RESULT_PARTIAL'),
])
def test_known_operation_outcomes_remain_distinct(message, code):
    assert operation_issue(message).code == code


@pytest.mark.parametrize('message', [
    'No online, routable motors to initialize. Check the motor buses.',
    'Motors are online but their configuration check has not finished. Check the motor bus and retry.',
    'selected motor offline, unroutable or Turn Unknown',
    'selected motor firmware/configuration check has not passed',
    'waiting for logical ABS and routes',
    'Motor scope setup failed',
])
def test_motor_readiness_rejections_keep_a_specific_explanation(message):
    issue = operation_issue(message)
    assert issue.code == 'HOST_MOTORS_NOT_READY'
    assert issue.severity == 'Blocking' and 1 <= len(issue.steps) <= 3


def test_snapshots_coalesce_related_motors_and_keep_history_separate():
    state = HandModel(active_motor_mask=3, servo_online_bitmap=0)
    state.safety_event_history = [dict(rule_id=1008, motor_channel=0, sequence=12)]
    issues = snapshot_issues(state, {}, fresh=True)
    assert len(issues) == 2
    current, historical = issues
    assert current['scope'] == 'M00, M01' and current['count'] == 2
    assert len(current['details']['evidence']) == 2
    assert historical['status'] == 'History'
    state.servo_online_bitmap = 3
    issues = snapshot_issues(state, {}, fresh=True)
    assert len(issues) == 1 and issues[0]['status'] == 'History'
    # Stale data is not proof of recovery; event evidence remains available.
    assert snapshot_issues(state, {}, fresh=False)[0]['status'] == 'History'


def test_selected_motors_are_checked_even_outside_active_motion_scope():
    issues = snapshot_issues(HandModel(), {'selected_motors': [26]}, fresh=True)
    assert issues[0]['scope'] == 'M26'


def test_coalescing_keeps_the_most_severe_rule_and_all_evidence():
    state = HandModel()
    state.safety_event_history = [dict(rule_id=1001, motor_channel=0),
                                  dict(rule_id=2002, motor_channel=1)]
    issue = snapshot_issues(state, {}, fresh=True)[0]
    assert issue['severity'] == 'Emergency' and issue['code'] == 'CTL-2002'
    assert issue['count'] == 2 and len(issue['details']['evidence']) == 2


def test_assembly_current_issue_requires_a_valid_result_and_p4_warning_bits():
    state = HandModel()
    state.joint_calib_results[1] = dict(success=True, assemblyWarningReason=1)
    assert not snapshot_issues(state, {}, fresh=True)
    state.auto_calibration_valid_bitmap = 2
    issues = snapshot_issues(state, {}, fresh=True)
    assert issues[0]['code'] == 'CAW-4101'
    state.joint_calib_results[1] = dict(success=True, assemblyWarningReason=0,
        autoMinOffsetCounts=5000, autoMaxShortfallCounts=5000)
    assert not snapshot_issues(state, {}, fresh=True)


def test_safety_catalog_reserves_history_ids_without_runtime_definitions():
    path = Path(__file__).resolve().parents[3] / 'desktop/resources/config/safety_rules.archive.json'
    archive = json.loads(path.read_text())['rules']
    assert len(archive) == 30 and len({r['rule_id'] for r in archive}) == 30
    assert not ({r['rule_id'] for r in archive} & set(safety_rule_table()))
    for item in archive:
        assert rule_issue(item['rule_id'], status='Current issue').status == 'History'


def test_user_issue_does_not_mutate_event_evidence():
    event = {'rule_id': 1008, 'nested': {'value': 4}}
    issue = rule_issue(1008, details=event).to_dict()
    issue['details']['nested']['value'] = 100
    assert event['nested']['value'] == 4
