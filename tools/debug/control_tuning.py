"""Scoped non-contact tuning through normal desktop authorization and protocol.

audit is offline; health never enables motors. run/sweep require an explicit
joint and motion flag. Exceptions STOP and terminate, never retry or clear
faults automatically. Full recording uses the normal recorder; trial samples
also retain P4 timestamps and same-cycle control components.

职责：编排离线审计、无运动通信检查与显式指定关节的自由运动试验。
线程：主试验线程；串口与记录线程归属 HandBackend。
输入：本机标定、参数和正常协议反馈；输出：独立试验目录与原始记录。
状态所有权：临时拥有试验控制范围，异常时终止运动，不自动解除故障。
禁止依赖：不得绕过正常使能门禁、猜测多圈坐标或覆盖标定。
"""
from __future__ import annotations
import argparse
import copy
from dataclasses import asdict
import json
import math
from pathlib import Path
import shutil
import sys
import time
import queue
import threading
import subprocess

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from desktop.core.control_tuning import Trial, calibration_audit, case_matrix, metrics, sine_metrics, reversal_metrics, pair_metrics, noise_metrics, square_metrics
from desktop.core.control_parameters import CURRENT_PATH


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['audit', 'health', 'inspect', 'profiles', 'clear', 'calibrate', 'run', 'sweep', 'ros2-offline'])
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--joint', type=int)
    parser.add_argument('--additional-joints',default='',help='explicit additional joints for Case I only')
    parser.add_argument('--phase-deg',type=int,choices=(0,90),default=0)
    parser.add_argument('--routing-label',help='recorded physical sheath placement; required for Case I')
    parser.add_argument('--motion', action='store_true')
    parser.add_argument('--clear-before-test', action='store_true',
                        help='explicit one-time Clear Faults before this test; never retry after failure')
    parser.add_argument('--case', choices=list('ABCDEFGHIJKP'), default='A')
    parser.add_argument('--duration', type=float, default=300, help='disabled health duration in seconds')
    parser.add_argument('--parameter', choices=['kp','kd','pair','takeup','confirm','ki','ilimit','fast'], default='kp')
    parser.add_argument('--values', help='comma separated sweep values')
    parser.add_argument('--base-kp',type=float,help='fixed Kp for the tested joint type in non-Kp sweeps')
    parser.add_argument('--base-parameters',type=Path,help='explicit fixed configuration for this experiment')
    parser.add_argument('--trial', help='comma-separated exact matrix trial names, optional')
    parser.add_argument('--amplitude-deg',type=float,help='Case P override: signed amplitude (half swing) for all three waveforms')
    parser.add_argument('--envelope-margin-deg',type=float,default=2.0,help='measured-angle excursion tolerated past the audited internal workspace before a trial aborts')
    parser.add_argument('--overshoot-stop-deg',type=float,default=0,help='square-edge overshoot abort threshold; 0 = auto max(3deg, 50pct of amplitude)')
    parser.add_argument('--frequency-hz',type=float,help='Case P override: waveform frequency for all three waveforms')
    args = parser.parse_args()
    test_joints=sorted(set(([args.joint] if args.joint is not None else [])+
                           [int(x) for x in args.additional_joints.split(',') if x.strip()]))
    if any(not 0<=j<21 for j in test_joints): parser.error('joint indices must be0..20')
    if args.additional_joints and (args.case!='I' or args.mode!='run'):
        parser.error('additional joints require explicit Case I run')
    if args.case=='I' and not args.routing_label:
        parser.error('Case I requires --routing-label describing the actual sheath arrangement')
    if args.mode in ('calibrate','run','sweep') and (not args.motion or args.joint is None):
        parser.error('run/sweep require --motion and an explicit --joint')
    args.output.mkdir(parents=True, exist_ok=False)
    reproduction=args.output/'reproduction'; reproduction.mkdir()
    for source in (Path(__file__),ROOT/'desktop/core/control_tuning.py',ROOT/'tools/debug/export_control_tuning.py',
                   ROOT/'desktop/core/core_logic.py',ROOT/'desktop/core/application_actions.py',
                   ROOT/'desktop/core/host_calibration/runtime.py',ROOT/'desktop/core/data_recording/packet_recorder.py'):
        shutil.copy2(source,reproduction/(source.parent.name+'-'+source.name))
    (args.output/'invocation.json').write_text(json.dumps(dict(argv=sys.argv,python=sys.version,
        executable=sys.executable,wall_time=time.time()),indent=2),encoding='utf-8')
    bundle = json.loads(args.calibration.read_text(encoding='utf-8'))
    audit = calibration_audit(bundle)
    (args.output/'audit.json').write_text(json.dumps(audit,indent=2),encoding='utf-8')
    shutil.copy2(args.calibration,args.output/'calibration.mmcal.json')
    shutil.copy2(CURRENT_PATH,args.output/'parameters.before.json')
    for row in audit:
        if row['eligible']:
            (args.output/f'J{row["joint"]:02d}-matrix.json').write_text(json.dumps(case_matrix(
                row['internal_low_deg'],row['internal_high_deg']),indent=2),encoding='utf-8')
    if args.mode == 'audit':
        print(json.dumps(audit),flush=True); return 0
    if args.mode == 'ros2-offline':
        from desktop.core.teleop_input import InputFilterConfig, StationaryAwareOneEuro
        filt = StationaryAwareOneEuro(InputFilterConfig())
        samples = []
        for i in range(6000):
            t=i*.01
            raw=.05*math.sin(2*math.pi*7*t) if t<30 else 1+.05*math.sin(2*math.pi*7*t)
            out, valid=filt.update(raw,t)
            samples.append(dict(t=t,raw=raw,filtered=out,valid=valid,state=filt.state))
        (args.output/'input-filter.json').write_text(json.dumps(samples),encoding='utf-8')
        return 0
    from desktop.core.hand_backend import HandBackend
    from desktop.core import application_actions
    application_actions.RUN_DATA_DIR=args.output/'recordings'
    from desktop.core.host_calibration.config import joint_motors
    from desktop.core.protocol import CMD_CONTROL_RELEASE
    from desktop.core.session_log import session_log_since
    hand=HandBackend(); hand._save_data=False; hand.open_application()
    packet_queue=queue.Queue(maxsize=50000)
    archive_errors=[]
    def archive_packets():
        try:
            with (args.output/'packets.jsonl').open('w',encoding='utf-8',buffering=65536) as stream:
                while True:
                    item=packet_queue.get()
                    if item is None: break
                    stamp,source,packets=item
                    for kind,payload in packets:
                        stream.write(json.dumps(dict(host_time=stamp,source=source,
                            packet_type=kind,payload_hex=payload.hex()))+'\n')
        except Exception as exc: archive_errors.append(str(exc))
    packet_thread=threading.Thread(target=archive_packets,name='tuning-packet-archive',daemon=True)
    packet_thread.start()
    events=(args.output/'events.jsonl').open('w',encoding='utf-8',buffering=1)
    failure_log_cursor=session_log_since()['sequence']
    def record(kind, **data):
        line=json.dumps(dict(time=time.time(),kind=kind,**data),ensure_ascii=False)
        print(line,flush=True); events.write(line+'\n')
    def wait(predicate, timeout=15):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate(): return
            time.sleep(.02)
        s=hand.get_state_snapshot()
        raise RuntimeError(f'timed out waiting for verified device state; '
                           f'online=0x{s.servo_online_bitmap:08X} '
                           f'unknown=0x{s.multi_turn_unknown_bitmap:08X} '
                           f'faults=0x{s.system_fault_bitmap:08X}')
    def operation(request, timeout=25):
        if not request.accepted: raise RuntimeError(request.message)
        if request.operation_id is None: return
        def done():
            if (args.output/'STOP').exists(): raise RuntimeError('explicit test STOP requested')
            if archive_errors or hand._recording_error:
                raise RuntimeError(f'data retention failed: {archive_errors} {hand._recording_error}')
            row=next(r for r in hand._application_operations if r['operation_id']==request.operation_id)
            if row['status']=='FAILED': raise RuntimeError(row['message'])
            return row['status']=='SUCCESS'
        wait(done,timeout)
        wait(lambda: not hand._application_busy,3)
    def stopped():
        s=hand.get_state_snapshot()
        return hand._fresh_application_state() and not s.control_enabled and not s.servo_target_owner
    def release():
        if not hand.is_comm_connected(): return
        stamp=time.time()
        if not hand.comm.send_command('control_release'): raise RuntimeError('release not queued')
        ack=hand.comm.wait_for_ack(CMD_CONTROL_RELEASE,2,received_after=stamp)
        if not ack or ack[2] not in (0,2): raise RuntimeError('release not acknowledged')
        wait(stopped,3)
        hand.select_joints([]); hand.select_motors([])
    def healthy(joint=None, enabled=False, require_coordinates=True):
        nonlocal failure_log_cursor
        recent=session_log_since(failure_log_cursor); failure_log_cursor=recent['sequence']
        if (args.output/'STOP').exists(): raise RuntimeError('explicit test STOP requested')
        if archive_errors or hand._recording_error:
            raise RuntimeError(f'data retention failed: {archive_errors} {hand._recording_error}')
        s=hand.get_state_snapshot()
        if not hand._fresh_application_state(): raise RuntimeError('system feedback stale')
        if enabled and (not s.control_enabled or s.feedback_collision_hold_bitmap):
            raise RuntimeError('control disabled or collision hold during free motion')
        if joint is not None:
            for checked in (test_joints if joint==args.joint else [joint]):
                motors=joint_motors(checked); mask=sum(1<<m for m in motors)
                if not s.encoder_online_bitmap & (1<<checked) or s.encoders[checked].angle_error:
                    raise RuntimeError(f'J{checked:02d} selected encoder invalid')
                if (require_coordinates and s.multi_turn_unknown_bitmap & mask) or s.servo_online_bitmap & mask != mask:
                    raise RuntimeError(f'J{checked:02d} selected servo offline/unknown')
            if time.time()-s.encoder_mapped_last_rx_ts>.15 or time.time()-s.servo_abs_last_rx_ts>.15:
                raise RuntimeError(f'selected feedback older than150ms: encoder_age_ms={(time.time()-s.encoder_mapped_last_rx_ts)*1000:.1f}, servo_age_ms={(time.time()-s.servo_abs_last_rx_ts)*1000:.1f}, trace_age_ms={(time.time()-s.control_trace_last_rx_ts)*1000:.1f}')
        return s
    def set_parameters(proposed):
        release()
        operation(hand.request_save_control_parameters(proposed))
        wait(lambda: hand._control_parameter_sync.startswith('APPLIED'),15)
        wait(lambda: not hand._application_busy,3)
        record('PARAMETERS',configuration=hand.control_parameter_snapshot())
    results=[]
    motion_started=False
    ros_bridge=ros_node=ros_publisher=ros_type=None
    raw=None
    inputs=None
    try:
        operation(hand.request_connect(args.port)); wait(stopped)
        hand.request_recording(True)
        wait(lambda: hand._recorder.is_running())
        record('RECORDING',path=hand._recording_path)
        normal_sink=hand.comm.recording_packet_sink
        def packet_sink(source,packets):
            normal_sink(source,packets)
            try: packet_queue.put_nowait((time.time(),source,list(packets)))
            except queue.Full: archive_errors.append('packet archive queue full')
        hand.set_recording_packet_sink(packet_sink)
        inspected=hand._calibration_workflow.inspect_snapshot(args.calibration)
        ok,message=hand._controller.load_calibration_snapshot(inspected)
        if not ok: raise RuntimeError(message)
        wait(lambda: hand._control_parameter_sync.startswith('APPLIED'))
        if args.clear_before_test:
            operation(hand.request_clear_faults())
            record('EXPLICIT_CLEAR_FAULTS',reason='operator-selected preparation; no automatic retry')
        if args.mode == 'profiles':
            # Read only: never APPLY_PROFILE, RENAME, or clear a fault here.
            from desktop.core.id_protocol import BEGIN, END, READ_PROFILE
            s=hand.get_state_snapshot()
            if s.control_enabled or s.servo_target_owner:
                raise RuntimeError('profile read requires disabled control')
            session=int(time.time_ns() & 0xffffffff) or 1
            def checked(reply):
                if reply['status'] != 0: raise RuntimeError(f'profile read rejected: {reply}')
                return reply
            checked(hand.comm.maintenance_request(session,BEGIN))
            profiles=[]
            try:
                for motor,bus in enumerate(s.servo_route_bus):
                    if not (s.servo_route_controllable_bitmap & (1<<motor)): continue
                    reply=checked(hand.comm.maintenance_request(session,READ_PROFILE,bus,motor))
                    profiles.append(dict(motor=motor,bus=bus,registers_84_85_86=reply['profile']))
                (args.output/'motor-profiles.json').write_text(json.dumps(profiles,indent=2),encoding='utf-8')
                record('MOTOR_PROFILES_READ_ONLY',profiles=profiles)
            finally:
                checked(hand.comm.maintenance_request(session,END))
            return 0
        if args.mode in ('inspect','clear'):
            if args.mode=='clear': operation(hand.request_clear_faults())
            time.sleep(1)
            s=hand.get_state_snapshot()
            record('DEVICE_STATE',state=s.system_state,mode=s.control_mode,
                   enabled=s.control_enabled,owner=s.servo_target_owner,
                   faults=s.system_fault_bitmap,unknown=s.multi_turn_unknown_bitmap,
                   active_motors=s.active_motor_mask,active_joints=s.active_joint_mask,
                   angles={j:s.encoders[j].current_angle_deg for j in (12,13)},
                   parameters=hand.control_parameter_snapshot())
            return 0
        if args.mode=='calibrate':
            hand.select_joints([args.joint]); motion_started=True
            operation(hand.request_calibration(),timeout=1200)
            record('CALIBRATION_COMPLETE',joint=args.joint,
                   result=hand.get_state_snapshot().joint_calib_results[args.joint],
                   storage=hand.calibration_storage_paths())
            for path in hand.calibration_storage_paths().values():
                if isinstance(path,str) and Path(path).is_file():
                    shutil.copy2(path,args.output/('after-'+Path(path).name))
            release()
            return 0
        # Multi-turn restoration is asynchronous after a flash/reconnect.
        # Wait for the selected scope; never require unrelated fingers.
        def scope_ready():
            try:
                for j in ((12,13) if args.mode=='health' else (args.joint,)):
                    healthy(j, require_coordinates=args.mode!='health')
                return True
            except RuntimeError:
                return False
        wait(scope_ready,25)
        record('CONNECTED',fingerprint=hand.get_state_snapshot().product_fingerprint,
               parameter=hand.control_parameter_snapshot())
        if args.mode=='health':
            noise=(args.output/'disabled-feedback.jsonl').open('w',encoding='utf-8',buffering=1)
            end=time.monotonic()+args.duration; last=0
            while time.monotonic()<end:
                s=healthy()
                if not stopped(): raise RuntimeError('unexpected motion ownership during health check')
                for joint in (12,13): healthy(joint, require_coordinates=False)
                noise.write(json.dumps(dict(p4_ms=s.encoder_mapped_p4_time_ms,
                            q=[s.encoders[j].current_angle_deg for j in (12,13)],
                            motors=[s.servo_angles[m] for m in (15,16,17)],
                            multi_turn_unknown_bitmap=s.multi_turn_unknown_bitmap))+'\n')
                if time.monotonic()-last>=10:
                    record('HEALTH',remaining_s=max(0,end-time.monotonic()),p4_ms=s.p4_time_ms,
                           manual=s.manual_reference_valid_bitmap,auto=s.auto_calibration_valid_bitmap,
                           connection=hand.comm.connection_diagnostics())
                    last=time.monotonic()
                time.sleep(.05)
            noise.close()
            rows=[json.loads(line) for line in (args.output/'disabled-feedback.jsonl').read_text(encoding='utf-8').splitlines()]
            summary=noise_metrics(rows)
            (args.output/'disabled-noise.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
            record('PASSED',mode='disabled communication health',seconds=args.duration,
                   motion_readiness_verified=False,noise=summary)
            return 0
        row=next((r for r in audit if r['joint']==args.joint),None)
        if not row or not row['eligible']: raise RuntimeError(f'joint failed calibration preflight: {row}')
        low,high=row['internal_low_deg'],row['internal_high_deg']
        scoped_rows=[next((r for r in audit if r['joint']==j),None) for j in test_joints]
        if any(r is None or not r['eligible'] for r in scoped_rows): raise RuntimeError('additional scope failed calibration audit')
        command_joint_mask=sum(1<<j for j in test_joints)
        command_motor_mask=sum(1<<m for r in scoped_rows for m in r['motors'])
        scoped_motors=sorted({m for r in scoped_rows for m in r['motors']})
        centers={r['joint']:(r['internal_low_deg']+r['internal_high_deg'])/2 for r in scoped_rows}
        candidates=[Trial(**x) for x in case_matrix(low,high) if x['case']==args.case]
        if args.case=='P' and (args.amplitude_deg is not None or args.frequency_hz is not None):
            if args.amplitude_deg is None or args.frequency_hz is None or args.amplitude_deg<=0 or args.frequency_hz<=0:
                raise RuntimeError('--amplitude-deg and --frequency-hz must be positive and set together')
            amp,freq=float(args.amplitude_deg),float(args.frequency_hz)
            period=1.0/freq
            center=(low+high)/2
            margin=float(args.envelope_margin_deg)
            if center-amp<low-margin or center+amp>high+margin: raise RuntimeError('amplitude exceeds the audited internal workspace with guard')
            candidates=[Trial('P','screen-square','square',center,amp,freq,5/freq,repetitions=1),
                        Trial('P','screen-triangle','triangle',center,amp,freq,6*period,period,repetitions=1,speed=4*amp*freq),
                        Trial('P','screen-sine','sine',center,amp,freq,7/freq,2/freq,repetitions=1)]
        if args.case=='H':
            candidates=[Trial('H',f'integral-{f}','hold',low+f*(high-low),repetitions=1) for f in (.2,.5,.8)]
        if args.case=='I':
            if len(test_joints)<2: raise RuntimeError('Case I requires explicit additional joints')
            center=(low+high)/2
            candidates=[Trial('I','multi-sine','sine',center,2,.1,30,repetitions=1),
                        Trial('I','multi-square','square',center,2,.1,30,repetitions=1),
                        Trial('I','multi-triangle','triangle',center,2,.625,9.6,repetitions=1,speed=5)]
        if args.case=='J':
            from desktop.ros2.joint_targets import Ros2JointTargetBridge,DEFAULT_JOINT_TARGET_TOPIC
            import rclpy
            from std_msgs.msg import Float32MultiArray
            ros_bridge=Ros2JointTargetBridge(hand); ros_bridge.start()
            hand.set_ros_availability(True)
            ros_node=rclpy.create_node(f'mmhand_tuning_publisher_{time.time_ns()}')
            ros_type=Float32MultiArray
            ros_publisher=ros_node.create_publisher(ros_type,DEFAULT_JOINT_TARGET_TOPIC,1)
            wait(lambda:ros_publisher.get_subscription_count()>0,10)
            center=(low+high)/2
            candidates=[Trial('J','ros-jitter','jitter',center,duration=30,repetitions=1)]
            candidates += [Trial('J',f'ros-step-{a}','step',center,a,duration=6,repetitions=3) for a in (-2,-.5,.5,2)]
            candidates += [Trial(**{**x,'case':'J','name':'ros-'+x['name']}) for x in case_matrix(low,high) if x['case']=='P']
        if args.trial: candidates=[t for t in candidates if t.name in args.trial.split(',')]
        if not candidates: raise RuntimeError('no matching trials')
        initial=hand._control_parameters.snapshot(); initial.pop('crc32',None)
        tendon_group='dual_tendon' if len(row['motors'])==2 else 'single_tendon'
        # J00..J19: four joints per finger in AA/MCP/PIP/DIP order; J20: thumb CMC.
        kp_fields=('aa','mcp','pip','dip','cmc')
        kp_field=kp_fields[-1] if args.joint==20 else kp_fields[args.joint%4]
        points=[None]
        if args.mode=='sweep':
            points=[float(v) for v in args.values.split(',')] if args.values else {
                'kp':[.2,.4,.6,.8,1.],
                'kd':[0,.02,.05,.1,.2],
                'pair':[0,.01,.03,.05,.08],
                'takeup':[40,80,140],
                'confirm':[.1,.25,.5,1.],
                'ki':[0,.02,.05,.1], 'ilimit':[20,50,100], 'fast':[10,20,30]}[args.parameter]
        fixed=json.loads(args.base_parameters.read_text(encoding='utf-8')) if args.base_parameters else initial
        fixed.pop('crc32',None)
        (args.output/'parameters.fixed.json').write_text(json.dumps(fixed,indent=2),encoding='utf-8')
        for point in points:
            proposal=copy.deepcopy(fixed)
            if args.base_kp is not None: proposal['joint_kp'][kp_field]=args.base_kp
            if point is not None:
                groups={'kp':('joint_kp',kp_field),
                        'kd':('joint_kd',kp_field),
                        'pair':('dual_tendon','pair_recovery_gain'),'takeup':('reversal','takeup_counts'),
                        'confirm':('reversal','confirm_deg'),
                        'ki':('joint_ki',kp_field),'ilimit':(tendon_group,'integral_limit_counts'),
                        'fast':('feedback','fast_window_ms')}
                g,k=groups[args.parameter]; proposal[g][k]=point
            if args.mode=='sweep' and args.parameter=='kp':
                proposal['joint_ki'][kp_field]=0
                proposal['joint_kd'][kp_field]=0
                proposal['dual_tendon']['pair_recovery_gain']=0
                proposal['reversal']['takeup_counts']=0
            set_parameters(proposal)
            for trial in candidates:
                for repeat in range(trial.repetitions):
                    release(); healthy(args.joint)
                    hand.select_joints(test_joints); motion_started=True
                    operation(hand.request_enable())
                    hand.request_recording(True); wait(lambda: hand._recorder.is_running())
                    s=healthy(args.joint,True)
                    start_q=s.encoders[args.joint].current_angle_deg
                    # Approach a trial's start at5deg/s before its measurement clock.
                    start_angles={j:s.encoders[j].current_angle_deg for j in test_joints}
                    centers[args.joint]=trial.center
                    approach=max(abs(start_angles[j]-centers[j])/5 for j in test_joints)
                    clock=time.monotonic()
                    while time.monotonic()-clock<approach:
                        elapsed=time.monotonic()-clock
                        target=start_q+(trial.center-start_q)*min(1,elapsed/max(.01,approach))
                        values=list(hand.get_state_snapshot().target_angles); values[args.joint]=target
                        for j in test_joints: values[j]=start_angles[j]+(centers[j]-start_angles[j])*min(1,elapsed/max(.01,approach))
                        result=hand.send_joint_targets(values,source='Tuning approach',live=True,command_joint_mask=command_joint_mask)
                        if not result.sent or result.clamped: raise RuntimeError(result.message)
                        healthy(args.joint,True); time.sleep(.02)
                    # Remove the approach transient before the measured pre-step
                    # baseline; do not claim arrival merely from target ACK.
                    hold_until=time.monotonic()+2
                    while time.monotonic()<hold_until:
                        values=list(hand.get_state_snapshot().target_angles); values[args.joint]=trial.center
                        for j in test_joints: values[j]=centers[j]
                        result=hand.send_joint_targets(values,source='Tuning settle',live=True,command_joint_mask=command_joint_mask)
                        if not result.sent or result.clamped: raise RuntimeError(result.message)
                        healthy(args.joint,True); time.sleep(.02)
                    if ros_publisher is not None: operation(hand.request_ros_input(True))
                    clock=time.monotonic(); last_trace=None; samples=[]; last_send=0
                    previous_ref=trial.center; edge_direction=0
                    log_cursor=session_log_since()['sequence']; log_check=0
                    start_ms=hand.get_state_snapshot().p4_time_ms
                    name=f'{len(results):04d}-{trial.name}'
                    raw=(args.output/(name+'.jsonl')).open('w',encoding='utf-8',buffering=1)
                    inputs=(args.output/(name+'-inputs.jsonl')).open('w',encoding='utf-8',buffering=1)
                    published_input=trial.center
                    (args.output/(name+'.metadata.json')).write_text(json.dumps(dict(
                        joint=args.joint,joints=test_joints,phase_deg=args.phase_deg,routing=args.routing_label,
                        trial=asdict(trial),point=point,parameter=args.parameter,
                        configuration=hand.control_parameter_snapshot(),fingerprint=s.product_fingerprint,
                        p4_start_ms=start_ms),indent=2),encoding='utf-8')
                    record('TRIAL_START',point=point,trial=asdict(trial),repeat=repeat,recording=hand._recording_path)
                    while time.monotonic()-clock<trial.duration:
                        elapsed=time.monotonic()-clock
                        target=trial.target(elapsed)
                        if not low-1e-6<=target<=high+1e-6: raise RuntimeError('trajectory exceeds internal workspace')
                        if elapsed-last_send>=.019:
                            values=list(hand.get_state_snapshot().target_angles); values[args.joint]=target
                            for j in test_joints:
                                if j!=args.joint:
                                    phase=args.phase_deg/360/trial.frequency if trial.frequency else 0
                                    values[j]=centers[j]+trial.target(elapsed+phase)-trial.center
                            if ros_publisher is not None:
                                msg=ros_type(); msg.data=[float(v) for v in values]; ros_publisher.publish(msg)
                            else:
                                result=hand.send_joint_targets(values,source='Control tuning',live=True,command_joint_mask=command_joint_mask)
                                if not result.sent or result.clamped: raise RuntimeError(result.message)
                            published_input=target
                            inputs.write(json.dumps(dict(host_time=time.time(),elapsed=elapsed,target_deg=target,
                                joint_targets_deg=values,transport='ROS2' if ros_publisher is not None else 'direct backend'))+'\n')
                            last_send=elapsed
                        s=healthy(args.joint,True)
                        if s.control_trace_sequence!=last_trace and s.has_control_trace:
                            if s.control_trace_valid_joint_mask & command_joint_mask != command_joint_mask:
                                raise RuntimeError('selected control trace feedback invalid')
                            if time.time()-s.control_trace_last_rx_ts>.15:
                                raise RuntimeError('control trace older than150ms')
                            last_trace=s.control_trace_sequence
                            q=s.control_trace_actual_deg[args.joint]
                            applied_ref=s.control_trace_target_deg[args.joint]
                            if trial.kind=='square':
                                if abs(applied_ref-previous_ref)>.1:
                                    edge_direction=1 if applied_ref>previous_ref else -1
                                overshoot_stop = float(args.overshoot_stop_deg) if args.overshoot_stop_deg else max(3,.5*abs(trial.amplitude))
                                if edge_direction*(q-applied_ref)>overshoot_stop:
                                    raise RuntimeError('repeated step overshoot safety stop')
                            previous_ref=applied_ref
                            for checked in scoped_rows:
                                angle=s.control_trace_actual_deg[checked['joint']]
                                if not checked['internal_low_deg']-float(args.envelope_margin_deg)<=angle<=checked['internal_high_deg']+float(args.envelope_margin_deg):
                                    raise RuntimeError(f"J{checked['joint']:02d} outside internal test envelope")
                            if not low-float(args.envelope_margin_deg)<=q<=high+float(args.envelope_margin_deg): raise RuntimeError('measured angle outside test envelope')
                            if trial.kind=='step' and elapsed>=1 and (q-trial.center-trial.amplitude)*(1 if trial.amplitude>0 else -1)>max(3,.25*abs(trial.amplitude)):
                                raise RuntimeError('step overshoot safety stop')
                            if s.control_trace_motor_command_mask & ~command_motor_mask:
                                raise RuntimeError('command emitted for unselected motor')
                            p4_elapsed=((s.control_trace_p4_time_ms-start_ms)&0xFFFFFFFF)*.001
                            if p4_elapsed>trial.duration+1: continue
                            sample=dict(t=p4_elapsed,host_elapsed=elapsed,p4_ms=s.control_trace_p4_time_ms,sequence=last_trace,
                                        accepted_ms=s.control_trace_target_accepted_ms,
                                        q=q,target=s.control_trace_target_deg[args.joint],
                                        joint_actual_deg=s.control_trace_actual_deg,joint_target_deg=s.control_trace_target_deg,
                                        valid_joint_mask=s.control_trace_valid_joint_mask,
                                        generated_input_deg=published_input,input_published_elapsed=last_send,
                                        components=s.control_trace_components,detail=s.control_trace_detail,
                                        commands=s.control_trace_motor_command_abs,
                                        collision_armed=bool(s.feedback_collision_armed_bitmap&(1<<args.joint)),
                                        telemetry_host_rx=s.servo_telem_last_rx_ts,
                                        telemetry_online={m:s.servo_telem_online[m] for m in scoped_motors},
                                        encoder_host_rx=s.encoder_mapped_last_rx_ts,servo_host_rx=s.servo_abs_last_rx_ts,
                                        load={m:s.servo_load[m] for m in scoped_motors},
                                        temperature_c={m:s.servo_temperature[m] for m in scoped_motors},
                                        collision=bool(s.feedback_collision_hold_bitmap&(1<<args.joint)))
                            samples.append(sample)
                            raw.write(json.dumps(sample)+'\n')
                        time.sleep(.005)
                    if ros_publisher is not None: hand.request_ros_input(False)
                    release()
                    raw.close()
                    inputs.close()
                    (args.output/(name+'.json')).write_text(json.dumps(samples),encoding='utf-8')
                    event=next((((s['accepted_ms']-start_ms)&0xFFFFFFFF)*.001 for s in samples
                                if abs(s['target']-trial.center-trial.amplitude)<.02),1) if trial.kind=='step' else None
                    measurement=metrics(samples,event_s=event,
                                        baseline=trial.center,target=trial.center+trial.amplitude,
                                        discard_s=trial.discard)
                    if trial.kind=='sine':
                        measurement['sine']=sine_metrics(samples,trial.frequency,trial.discard)
                        measurement['reversals']=reversal_metrics(samples,trial.frequency)
                    if trial.kind=='square': measurement['edges']=square_metrics(samples)
                    measurement['pair']=pair_metrics(samples,bundle['joint_calib_results'][args.joint],
                        hand.control_parameter_snapshot()['dual_tendon']['pair_recovery_gain'])
                    if not measurement['valid']: raise RuntimeError(str(measurement))
                    if trial.kind=='step' and measurement.get('overshoot_deg',0)>max(3,.25*abs(trial.amplitude)):
                        raise RuntimeError(f'unsafe step overshoot: {measurement}')
                    results.append(dict(trial=asdict(trial),point=point,repeat=repeat,metrics=measurement))
                    record('TRIAL_RESULT',**results[-1])
                    (args.output/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
        # Sweeps are experiments, not automatic endorsement of the last point.
        set_parameters(initial)
        record('COMPLETED',trials=len(results),parameters_restored=True)
        return 0
    except BaseException as exc:
        if motion_started and hand.is_comm_connected(): hand.stop_motion(source='control tuning aborted')
        record('STOPPED',error=f'{type(exc).__name__}: {exc}',session=session_log_since()['lines'][-200:])
        return 1
    finally:
        if raw is not None and not raw.closed: raw.close()
        if inputs is not None and not inputs.closed: inputs.close()
        if ros_node is not None: ros_node.destroy_node()
        if ros_bridge is not None: ros_bridge.stop()
        hand.request_recording(False)
        if hand._recording_thread: hand._recording_thread.join(timeout=10)
        record('FINALIZED',recording=hand._recording_path,session=session_log_since()['lines'])
        hand.disconnect(stop_lower=False); hand.close_application(); events.close()
        packet_queue.put(None,timeout=10); packet_thread.join(timeout=15)
        # Export in a fresh interpreter: a long experiment may have imported an
        # earlier analysis module before offline tooling was updated.
        exported=subprocess.run([sys.executable,str(ROOT/'tools/debug/export_control_tuning.py'),str(args.output)])
        (args.output/'export-status.json').write_text(json.dumps(dict(returncode=exported.returncode)),encoding='utf-8')


if __name__=='__main__':
    raise SystemExit(main())
