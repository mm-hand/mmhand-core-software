"""Read-only console interpretation. No device commands or control decisions.

Quality thresholds mirror the existing calibration warning contract. Missing
measurements remain unknown; presentation labels never authorize motion.


职责：解释只读状态与质量标签。
线程：读取后端同步快照，不创建线程。
输入：后端快照及用户交互。
输出：只读显示字段。
状态所有权：不拥有设备状态，不授权控制。
禁止依赖：禁止依赖 UI 或改变硬件判据。
"""
from .host_calibration.math import forward_encoder_delta, deg_to_encoder_count, f32
from .host_calibration.config import joint_motors
from .motor_mapping import MOTOR_TO_JOINT_INDEX
from .calibration_snapshot import parse_mask
from .protocol import (
    DUAL_TENDON_JOINT_INDICES, ControlMode, SERVO_TARGET_OWNER_CONTROL,
    SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_RUN_DIRECT,
)


def joint_quality(valid, span, urdf_span, draft_status='', fresh=True):
    if draft_status and draft_status not in ('SAVED', 'DRAFT'):
        return ['FAILED']
    if draft_status == 'DRAFT':
        return ['PENDING']
    if not fresh:
        return ['UNKNOWN']
    if not valid:
        return ['PENDING']
    if span is None or urdf_span is None:
        return ['UNKNOWN']
    return ['OFFSET' if abs(span - urdf_span) > 360 else 'READY']


def motor_quality(valid, result, fresh=True, *, joint=None):
    if result and result.get('success') is False:
        return ['FAILED']
    if not fresh:
        return ['UNKNOWN']
    if not valid:
        return ['PENDING']
    if not result or result.get('success') is not True or not result.get('hasAssemblyWarningFields'):
        return ['UNKNOWN']
    if 'autoMinOffsetCounts' not in result or 'autoMaxShortfallCounts' not in result:
        return ['UNKNOWN']
    warnings = []
    reason = int(result.get('assemblyWarningReason', 0))
    if reason & 1 or int(result['autoMinOffsetCounts']) >= 350:
        warnings.append('HIGH FRICTION' if joint in DUAL_TENDON_JOINT_INDICES else 'WEAK RETURN')
    if reason & 2 or int(result['autoMaxShortfallCounts']) >= 300:
        warnings.append('HIGH FRICTION')
    return list(dict.fromkeys(warnings)) or ['READY']


def calibration_status_badge(state, fresh=True):
    """Stable calibration completeness over the installed/manual joint scope.

    The denominator must not follow encoder_online_bitmap: CAN telemetry can
    briefly report a different set of encoders, which made the badge jump
    between 0/8 and 0/13. Manual endpoints and the loaded active scope define
    the joints that actually belong to this hand/calibration.
    """
    if not fresh:
        return 'UNKNOWN'
    if getattr(state, 'calib_status', '') in ('PENDING', 'RUNNING'):
        return 'CALIBRATING'
    count = len(getattr(state, 'encoders', []) or [])
    mask = (1 << count) - 1 if count else 0
    known = (
        int(getattr(state, 'manual_reference_valid_bitmap', 0) or 0)
        | int(getattr(state, 'active_joint_mask', 0) or 0)
    ) & mask
    if not known:
        return 'NEED CALIBRATION'
    auto = int(getattr(state, 'auto_calibration_valid_bitmap', 0) or 0) & known
    auto_count = auto.bit_count()
    known_count = known.bit_count()
    if auto_count == known_count:
        return f'READY {auto_count}/{known_count}'
    return f'NEED CALIBRATION {auto_count}/{known_count}'


def direct_zero_required(state, fresh=False):
    """Direct 页没有可用电机坐标：没有一台在线可路由电机知道自己的 ABS。

    用「全部在线电机 Turn Unknown」而不是「没有 motor calibration」当判据：
    归零本身会清掉 motor calibration，用后者会无限重弹。
    """
    if not fresh:
        return False
    online = int(getattr(state, 'servo_online_bitmap', 0) or 0) & int(
        getattr(state, 'servo_route_controllable_bitmap', 0) or 0)
    unknown = int(getattr(state, 'multi_turn_unknown_bitmap', 0) or 0)
    return bool(online and not (online & ~unknown))


def enrich_snapshot(hand, state, snapshot, now, fresh):
    control_known = fresh and state.has_active_scope_status
    control_active = state.control_enabled and state.servo_target_owner == SERVO_TARGET_OWNER_CONTROL
    feedback_active = (control_active and state.control_mode == ControlMode.JOINT_ANGLE.value
                       and state.system_state == SYSTEM_STATE_RUN_FEEDBACK)
    direct_active = (control_active and state.control_mode == ControlMode.DIRECT_MOTOR.value
                     and state.system_state == SYSTEM_STATE_RUN_DIRECT)
    snapshot['control_mode'] = ('UNKNOWN' if not control_known else
                                'FEEDBACK' if feedback_active else
                                'DIRECT' if direct_active else 'IDLE')
    for row in snapshot['joints']:
        j, draft = row['joint'], row['draft']
        row['feedback_control_status'] = ('UNKNOWN' if not control_known else
            'ENABLED' if feedback_active and state.active_joint_mask & (1 << j) else 'DISABLED')
        row['collision_detected'] = bool(fresh and state.feedback_collision_hold_bitmap & (1 << j))
        result = state.joint_calib_results[j] or {}
        minimum, maximum = draft.get('min', row['manual_min']), draft.get('max', row['manual_max'])
        span = forward_encoder_delta(minimum, maximum) if row['manual_valid'] or {'min', 'max'} <= draft.keys() else None
        urdf_span = deg_to_encoder_count(f32(f32(row['max']) - f32(row['min'])))
        quality = joint_quality(row['manual_valid'] or draft.get('status') == 'SAVED', span, urdf_span, hand._joint_transactions.get(j, draft.get('status', '')), fresh)
        saved_pair = hand._manual_saved_pairs.get(j)
        persisted = (draft.get('status') == 'SAVED' or state.manual_reference_committed and
                     hand._manual_saved_pairs.get(j) == (state.manual_reference_min_raw[j], state.manual_reference_max_raw[j]))
        persistence = (draft.get('status') or ('Saved' if persisted and row['manual_valid'] else
                       'Saved in JSON; not loaded in P4' if saved_pair and not row['manual_valid'] else
                       'RAM differs from saved JSON' if saved_pair else 'RAM / JSON unknown'))
        if row['manual_valid'] and quality[0] in ('READY', 'OFFSET') and not persisted:
            quality = ['UNKNOWN']
        motor_status = motor_quality(row['auto_valid'], result, fresh, joint=j)
        if draft and draft.get('status') != 'SAVED' or draft.get('status') == 'SAVED' and not row['auto_valid']:
            motor_status = ['PENDING']
        transaction = hand._motor_transactions.get(j)
        if transaction in ('FAILED', 'PENDING', 'CALIBRATING'):
            motor_status = [transaction]
        row.update(manual_min=minimum if 'min' in draft or row['manual_valid'] else None,
                   capture_status=hand._manual_capture_status.get(j, ''),
                   manual_max=maximum if 'max' in draft or row['manual_valid'] else None,
                   span=span, urdf_span=urdf_span, difference=abs(span - urdf_span) if span is not None else None,
                   joint_quality=quality, motor_quality=motor_status, motors=list(joint_motors(j)),
                   auto_min=result.get('autoMinRaw'), auto_max=result.get('autoMaxRaw'),
                   auto_span=(result.get('autoSpanCounts')
                              or result.get('assemblyAutoSpanCounts')),
                   result=result,
                   json_status='Saved' if persisted and row['manual_valid'] else 'Unknown',
                   persistence=persistence,
                   encoder='FAULT' if fresh and state.encoder_fault_bitmap & (1 << j) else
                           'ONLINE' if fresh and state.has_encoder_health_status and state.encoder_online_bitmap & (1 << j) else 'UNKNOWN',
                   ros_target=getattr(hand, '_console_ros_targets', {}).get(j))
    for row in snapshot['motors']:
        m = row['motor']
        row['direct_control_status'] = ('UNKNOWN' if not control_known else
            'ENABLED' if direct_active and state.active_motor_mask & (1 << m) else 'DISABLED')
        limits = None
        if fresh and state.servo_logical_range_valid_bitmap & (1 << m):
            lo, hi = state.servo_logical_range_min[m], state.servo_logical_range_max[m]
            if state.motor_abs_limit_valid_bitmap & (1 << m):
                lo, hi = max(lo, state.motor_abs_limit_min[m]), min(hi, state.motor_abs_limit_max[m])
            if lo <= hi:
                limits = [lo, hi]
        row.update(joint=MOTOR_TO_JOINT_INDEX[m], limits=limits,
                   bus=state.servo_route_bus[m] if fresh and state.has_servo_route_status else None,
                   route=state.servo_route_state[m] if fresh and state.has_servo_route_status else None,
                   raw=state.servo_raw_positions[m] if fresh and state.has_servo_raw_data and row['abs'] is not None and state.servo_raw_p4_time_ms == state.servo_feedback_p4_time_ms else None,
                   saved_raw=state.host_multiturn_saved_raw_positions[m] if state.host_multiturn_valid_mask & (1 << m) else None,
                   turn=(state.servo_angles[m] // 4096) if row['abs'] is not None and state.has_connectivity_status and not state.multi_turn_unknown_bitmap & (1 << m) else None)
    snapshot['selected_motors'] = list(hand._selected_motors)
    trace = hand._replay_service.trace() if hand._replay_service else None
    snapshot['replay']['file'] = dict(name=trace.name, frames=trace.frame_count, duration_s=trace.duration_s,
        rate_hz=trace.average_hz, warnings=list(trace.warnings)) if trace else {}
    current_op = next((op for op in snapshot['operations'] if op['operation_id'] == hand._application_busy), {})
    snapshot['connections'] = {}
    comm = hand.tactile_comm
    link = comm.connection_diagnostics() if comm and callable(getattr(comm, 'connection_diagnostics', None)) else {}
    if not isinstance(link, dict):
        link = {}
    s3_received_at = link.get('last_valid_rx', getattr(hand._tactile_state, 'tactile_last_rx_ts', 0.0))
    # RX can publish after this snapshot captured `now`; that is fresh data.
    s3_age = max(0, int((now - s3_received_at) * 1000)) if s3_received_at else None
    for name, connected, port, age in (
        ('p4', hand.is_comm_connected(), getattr(hand.comm, 'port', ''), snapshot['diagnostics']['receive_age_ms']),
        ('s3', hand.is_tactile_serial_connected(), hand.tactile_port, s3_age)):
        pending = current_op.get('name', '').replace(' ', '').lower()
        transition = 'Connecting…' if pending == 'connect' + name else 'Disconnecting…' if pending == 'disconnect' + name else ''
        snapshot['connections'][name] = dict(connected=connected, port=port, age_ms=age if connected else None,
            label=transition or ('Disconnect' if connected else 'Connect'), pending=bool(transition))
        snapshot['connections'][name]['data_status'] = ('Disconnected' if not connected else
            'Waiting for data' if age is None else 'Stale' if age > 600 else 'Online')
        if name == 's3' and connected and age is None and link.get('opened_at') and now - link['opened_at'] > 10:
            snapshot['connections'][name]['data_status'] = 'No valid frames' if link.get('bytes', 0) else 'No bytes received'
    snapshot['recording'].update(enabled=hand._save_data, full_tactile=hand._record_full_tactile,
        duration_s=max(0, now - hand._recording_started_at) if hand._recorder.is_running() else 0,
        error=hand._recording_error, finalizing=bool(hand._recording_thread and hand._recording_thread.is_alive()))
    fault_fresh = fresh and state.has_fault_status_data and 0 <= now - state.fault_status_last_rx_ts <= 0.6
    calibration_badge = calibration_status_badge(state, fresh)
    snapshot['badges'] = {
        'System': snapshot['device'], 'Calibration': calibration_badge,
        'Overload': 'ACTIVE' if fault_fresh and any(state.servo_overload_fault) or fresh and state.has_servo_route_status and state.servo_bus_overload_mask else 'CLEAR' if fault_fresh and state.has_servo_route_status else 'UNKNOWN',
        'Collision': 'HOLD' if fresh and state.feedback_collision_hold_bitmap else 'ARMED' if fresh and state.feedback_collision_armed_bitmap else 'OFF' if fresh and state.has_feedback_collision_status else 'UNKNOWN',
        'Multi-turn': 'UNKNOWN' if not fresh or not state.has_connectivity_status or not state.servo_online_bitmap else 'MISSING' if state.multi_turn_unknown_bitmap & state.servo_online_bitmap else 'KNOWN'}
    snapshot['direct_zero_required'] = direct_zero_required(state, fresh)
    snapshot['import_preview'] = None
    for token, preview in hand._import_previews.items():
        payload = preview['payload']
        snapshot['import_preview'] = dict(token=token, name=preview['name'], saved_at=payload.get('saved_at_host_time', 'Unknown'),
            profile=payload.get('hand_profile', 'Unknown'), joints=parse_mask(payload.get('active_joint_mask')),
            manual_joints=parse_mask(payload.get('manual_reference_valid_bitmap')),
            motors=parse_mask(payload.get('active_motor_mask')))
    tactile = snapshot['tactile']
    tactile.update(level=state.tactile_data_level, modules=[])
    for f, finger in enumerate(state.tactile_data):
        for m, sensor in enumerate(finger.sensors):
            index = f * 3 + m
            age = now - state.tactile_module_last_rx_ts[index]
            valid = bool(state.tactile_module_valid_mask & (1 << index)) and 0 <= age <= 0.6
            tactile['modules'].append(dict(finger=f, module=m, valid=valid, age_ms=int(age * 1000) if state.tactile_module_last_rx_ts[index] else None,
                force=sensor.resultant.tolist() if valid else None,
                taxels=sensor.contact_forces.tolist() if valid and state.tactile_data_level == 'FULL' else None))
    snapshot['diagnostics']['routes'] = [dict(motor=m['motor'], bus=m['bus'], route=m['route'], online=m['online']) for m in snapshot['motors']]
    snapshot['diagnostics']['raw_warning_bitmap'] = state.calibration_warning_bitmap
    snapshot['diagnostics'].update(direct_reject_reason=state.direct_reject_reason,
        feedback_reject_reason=state.feedback_reject_reason, encoder_fault_bitmap=state.encoder_fault_bitmap,
        multi_turn_unknown_bitmap=state.multi_turn_unknown_bitmap,
        active_motor_mask=state.active_motor_mask, active_joint_mask=state.active_joint_mask,
        calibration_results=state.joint_calib_results, manual_reference_crc=state.manual_reference_crc)
    issues = [dict(id=f'p{i}', severity='Warning', channel='System', problem=text, details=text) for i, text in enumerate(snapshot['problems'])]
    if fault_fresh:
        for m, fault in enumerate(state.servo_overload_fault):
            if fault:
                issues.append(dict(id=f'overload-{m}', severity='Fault', channel=f'M{m:02d}', problem='OVERLOAD',
                    details=dict(trigger_load=state.servo_overload_trigger_load[m], guidance='Resolve the mechanical load before clearing faults.')))
    for row in snapshot['joints']:
        if row['encoder'] == 'FAULT':
            issues.append(dict(id=f"enc-{row['joint']}", severity='Fault', channel=f"J{row['joint']:02d}", problem='ENCODER FAULT', details='Check encoder wiring and communication.'))
        for status in dict.fromkeys(row['joint_quality'] + row['motor_quality']):
            if status in ('OFFSET', 'WEAK RETURN', 'HIGH FRICTION', 'FAILED'):
                issues.append(dict(id=f"{row['joint']}-{status}", severity='Fault' if status == 'FAILED' else 'Warning',
                    channel=f"J{row['joint']:02d}", problem=status, details=dict(manual_span=row['span'], urdf_span=row['urdf_span'],
                    difference=row['difference'], raw_result=row['result'], thresholds={'OFFSET': '>360', 'WEAK RETURN': '≥350', 'HIGH FRICTION': '≥300'},
                    guidance='Check installation, spring return, tendon routing and preload. Labels describe symptoms, not measured force.')))
    snapshot['issues'] = issues
    snapshot['stop_latched'] = bool(getattr(hand, '_estop_requested_at', 0.0))
    return snapshot
