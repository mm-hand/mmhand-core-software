"""User-facing safety and operation explanations without control authority.

职责：把 P4 规则、操作结果和当前状态转换为统一的通俗英文问题。
线程：只读快照，可在后端或 UI 调用。
输入：规则编号、结构化诊断、操作结果和设备快照。
输出：UserIssue；保留原始证据和历史状态。
状态所有权：只拥有解释元数据，不拥有设备真值。
禁止依赖：不得导入 UI、串口或发送恢复和运动命令。
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field
from functools import lru_cache
import json
from pathlib import Path

_CONFIG = Path(__file__).resolve().parents[1] / 'resources/config'


@lru_cache(maxsize=1)
def guidance():
    return json.loads((_CONFIG / 'safety_guidance.json').read_text(encoding='utf-8'))


@dataclass(frozen=True)
class UserIssue:
    code: str
    title: str
    impact: str
    steps: tuple[str, ...]
    severity: str = 'Blocking'
    category: str = 'action'
    scope: str = 'System'
    status: str = 'Last action failed'
    details: dict = field(default_factory=dict)
    count: int = 1

    def to_dict(self):
        return asdict(self)

    def text(self):
        return '\n'.join((self.title, self.impact, *(f'{i}. {step}' for i, step in enumerate(self.steps, 1))))


class IssueMessage(str):
    """Preserve an explicit cause through existing string-based result APIs."""
    def __new__(cls, text, issue_code):
        value = super().__new__(cls, text)
        value.issue_code = issue_code
        return value


def host_issue(code, *, scope='System', status='Last action failed', details=None):
    definition = guidance()['host'].get(code, guidance()['host']['HOST_ACTION_FAILED'])
    return UserIssue(code=code, **{k: definition[k] for k in ('title', 'impact', 'category')},
                     steps=tuple(definition['steps']), scope=scope, status=status,
                     severity='Info' if definition['category'] == 'cancelled' else 'Blocking',
                     details=deepcopy(details or {}))


def rule_issue(rule_id, *, scope='System', status='History', details=None):
    from .safety_rules import safety_rule_table, safety_rule_archive
    rule_id = int(rule_id)
    table = safety_rule_table()
    row = table.get(rule_id)
    archived = None
    if row is None:
        archived = safety_rule_archive().get(rule_id)
        if archived and archived.get('replacement'):
            row = table.get(archived['replacement'])
    definition = guidance()['rules'].get(str(row['rule_id'])) if row else None
    evidence = deepcopy(details or {})
    evidence['rule_id'] = rule_id
    if archived:
        evidence['historical_rule'] = deepcopy(archived)
        status = 'History'
    if not definition:
        issue = host_issue('HOST_ACTION_FAILED', scope=scope, status=status, details=evidence)
        return UserIssue(**{**asdict(issue), 'code': f'P4-{rule_id}',
                            'title': 'An older or unrecognized device event was received'})
    return UserIssue(code=row['code'], title=definition['title'], impact=definition['impact'],
                     steps=tuple(definition['steps']), severity=row['severity'], category=definition['category'],
                     scope=scope, status=status, details=evidence)


def operation_issue(message, *, scope='System', diagnostic_reason=None, details=None):
    evidence = {**(details or {}), 'original_message': str(message)}
    explicit = getattr(message, 'issue_code', None)
    if explicit and '-' in explicit and explicit.rsplit('-', 1)[-1].isdigit():
        return rule_issue(int(explicit.rsplit('-', 1)[-1]), scope=scope, status='Last action failed', details=evidence)
    if explicit:
        return host_issue(explicit, scope=scope, details=evidence)
    if diagnostic_reason is not None:
        return host_issue(f'HOST_CALIB_{int(diagnostic_reason)}', scope=scope, details=evidence)
    # Adapt known legacy result phrases only. Unknown results retain a neutral
    # explanation; they never acquire a guessed physical cause or an automatic retry.
    text = str(message).lower()
    patterns = (
        (('cancelled',), 'HOST_CANCELLED'),
        (('result timed out', 'result timeout', 'completion is unknown', 'unconfirmed'), 'HOST_RESULT_UNKNOWN'),
        (('partly completed', 'partially completed'), 'HOST_RESULT_PARTIAL'),
        (('servo abs rebase failed',), 'HOST_ABS_REBASE_FAILED'),
        (('servo feedback recovery failed',), 'HOST_FEEDBACK_RECOVERY_FAILED'),
        (('parameters are not applied', 'parameter synchronization', 'parameters have not'), 'HOST_PARAMETERS_PENDING'),
        (('parameters invalid', 'parameter schema', 'expected 0', 'expected 100'), 'HOST_PARAMETERS_INVALID'),
        (('disconnected', 'serial not connected', 'serial open failed', 'status data is stale'), 'HOST_CONNECTION_LOST'),
        (('protocol contract', 'firmware version', 'handshake'), 'HOST_FIRMWARE_MISMATCH'),
        (('still finishing', 'operation is busy'), 'HOST_OPERATION_BUSY'),
        (('select joints', 'select motors', 'select at least', 'scope is empty'), 'HOST_SELECTION_EMPTY'),
        (('manual endpoints', 'manual raw value'), 'HOST_MANUAL_INVALID'),
        (('missing manual min', 'manual calibration missing', 'manual reference missing'), 'HOST_MANUAL_MISSING'),
        (('motor calibration missing', 'auto calibration missing', 'v4 calibration fields missing'), 'HOST_CALIBRATION_MISSING'),
        (('outside the allowed', 'outside [urdf', 'outside the safe', 'would require clamping'), 'HOST_TARGET_INVALID'),
    )
    for markers, code in patterns:
        if any(marker in text for marker in markers):
            return host_issue(code, scope=scope, details=evidence)
    # An explicit app-side latch message is evidence; a generic reset failure isn't.
    if text.startswith(('device is stopped (e-stop latched)', 'clear faults explicitly before enabling after stop')):
        return rule_issue(1005, scope=scope, status='Last action failed', details=evidence)
    return host_issue('HOST_ACTION_FAILED', scope=scope, details=evidence)


def issue_text(issue):
    if not issue:
        return ''
    return '\n'.join((issue['title'], issue['impact'],
                      *(f'{i}. {step}' for i, step in enumerate(issue['steps'], 1))))


def _scope(motor=255, joint=255):
    return f'J{joint:02d}' if 0 <= joint < 21 else f'M{motor:02d}' if 0 <= motor < 27 else 'System'


def snapshot_issues(state, snapshot, *, fresh):
    """Current conditions come from fresh state; events alone are always history."""
    current = []
    if not fresh:
        if snapshot.get('connections', {}).get('p4', {}).get('port'):
            current.append(host_issue('HOST_CONNECTION_LOST', status='Current issue').to_dict())
    else:
        motor_mask = int(state.active_motor_mask)
        for motor in snapshot.get('selected_motors', []):
            motor_mask |= 1 << int(motor)
        offline = motor_mask & ~int(state.servo_online_bitmap)
        for motor in range(27):
            bit = 1 << motor
            rule = (1008 if offline & bit else 1003 if state.servo_route_conflict_bitmap & motor_mask & bit
                    else 1013 if state.servo_turn_overflow_bitmap & motor_mask & bit
                    else 1009 if state.multi_turn_unknown_bitmap & motor_mask & bit else None)
            if rule:
                current.append(rule_issue(rule, scope=f'M{motor:02d}', status='Current issue',
                    details={'online_mask': state.servo_online_bitmap,
                             'controllable_mask': state.servo_route_controllable_bitmap,
                             'unknown_position_mask': state.multi_turn_unknown_bitmap,
                             'overflow_mask': state.servo_turn_overflow_bitmap}).to_dict())
        for bit, rule in ((1, 3501), (4, 3502), (16, 3503), (64, 3505)):
            if state.system_fault_bitmap & bit:
                current.append(rule_issue(rule, status='Current issue',
                    details={'fault_bitmap': state.system_fault_bitmap}).to_dict())
        for row in snapshot.get('joints', []):
            if row.get('encoder') == 'FAULT':
                current.append(rule_issue(2003, scope=f"J{int(row['joint']):02d}", status='Current issue').to_dict())
        # Assembly warnings are authoritative P4 bits; no local 350/300 recomputation.
        for joint, result in enumerate(state.joint_calib_results):
            if not state.auto_calibration_valid_bitmap & (1 << joint) or not isinstance(result, dict):
                continue
            reason = int(result.get('assemblyWarningReason', 0))
            for bit, rule in ((1, 4101), (2, 4102), (4, 4103)):
                if reason & bit:
                    current.append(rule_issue(rule, scope=f'J{joint:02d}', status='Current issue',
                                              details={'calibration_result': result}).to_dict())
    if snapshot.get('stop_latched'):
        current.insert(0, rule_issue(1005, status='Current issue', details={'source': 'app_stop_request'}).to_dict())
    # Keep separate failures when the controller does not report enough state to
    # tell whether an old event has recovered. Silence is never treated as recovery.
    for operation in snapshot.get('operations', [])[-10:]:
        if operation.get('status') == 'FAILED':
            issue = operation.get('user_issue') or operation_issue(operation.get('message', ''),
                scope=operation.get('scope', 'System')).to_dict()
            current.append(deepcopy(issue))
    for event in state.safety_event_history[-20:]:
        current.append(rule_issue(event['rule_id'],
            scope=_scope(event.get('motor_channel', 255), event.get('joint_index', 255)),
            details=event).to_dict())
    groups = {}
    for issue in current:
        key = (issue['category'], issue['status'])
        if key not in groups:
            groups[key] = {**issue, 'scopes': [], 'count': 0, 'details': {'evidence': []}}
        group = groups[key]
        severity_order = {'Emergency': 0, 'Blocking': 1, 'Warning': 2, 'Info': 3}
        if severity_order.get(issue['severity'], 3) < severity_order.get(group['severity'], 3):
            for field in ('code', 'title', 'impact', 'steps', 'severity'):
                group[field] = issue[field]
        group['count'] += issue.get('count', 1)
        if issue['scope'] not in group['scopes']: group['scopes'].append(issue['scope'])
        group['scope'] = ', '.join(group['scopes'])
        group['details']['evidence'].append({'code': issue['code'], 'scope': issue['scope'], 'title': issue['title'], **issue['details']})
    priority = {'Emergency': 0, 'Blocking': 1, 'Warning': 2, 'Info': 3}
    return sorted(groups.values(), key=lambda i: (i['status'] != 'Current issue', priority.get(i['severity'], 3)))
