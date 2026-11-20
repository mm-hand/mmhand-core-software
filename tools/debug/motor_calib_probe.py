#!/usr/bin/env python3
"""小拇指 Motor Calibration + 60 s 腱漂移实测（headless）。

职责：连接 P4、加载标定包、重做指关节 Motor Calibration，replay 一段三角波，
      把运行时的 关节角-电机 ABS 曲线与标定 LUT 对比，输出 CSV/图/摘要。
线程：单进程串行；不启动 NiceGUI。
输入：串口、关节列表、三角波参数。
输出：logs/motor_calib_probe_<时间戳>/ 与终端摘要。
状态所有权：只拥有本次探测进程；设备真值归 HandBackend/P4。
禁止依赖：禁止 UI、直接协议或跳过 HandBackend 安全门禁。
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta
import json
import math
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

PROBE_SPAN_HEAD_S = 1.0
PROBE_TAIL_S = 0.5
RAMP_SPEED_DEG_S = 10.0
SAMPLE_PERIOD_S = 0.05
MAX_FOLLOW_ERROR_DEG = 25.0
MAX_FOLLOW_ERROR_S = 2.0
LIMIT_GUARD_COUNTS = 25


def wait_result(hand, result, label, timeout=20.0):
    if not result.accepted:
        raise RuntimeError(f'{label}: {result.message}')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        item = next((row for row in hand._application_operations
                     if row['operation_id'] == result.operation_id), None)
        if item is not None and item['status'] in ('SUCCESS', 'FAILED'):
            if item['status'] != 'SUCCESS':
                raise RuntimeError(f'{label}: {item.get("message") or item["status"]}')
            return item
        time.sleep(0.02)
    raise TimeoutError(f'{label} did not finish in {timeout:.0f}s')


def wait_until(predicate, label, timeout, interval=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
    raise TimeoutError(f'{label} did not become ready in {timeout:.0f}s')


def lut_from_result(result, motor):
    motors = [int(m) for m in (result.get('tendonLutMotor') or [])]
    if motor not in motors:
        raise RuntimeError(f'M{motor:02d} is not in tendonLutMotor {motors}')
    column = motors.index(motor)
    count = int(result.get('tendonLutCount') or 0)
    if count < 2:
        raise RuntimeError(f'J{int(result.get("joint", -1)):02d} has no usable tendon LUT')
    values = [int(v) for v in result['tendonLutMotorAbs'][column][:count]]
    start = float(result['tendonLutAngleStartDeg'])
    step = float(result['tendonLutAngleStepDeg'])
    angles = [start + step * i for i in range(count)]
    angle_max = float(result.get('angleMax', angles[-1]))
    if angles[-1] > angle_max:
        angles[-1] = angle_max
    return {'motor': motor, 'angles': angles, 'values': values}


def lut_model(lut, deg):
    angles, values = lut['angles'], lut['values']
    if deg <= angles[0]:
        return float(values[0])
    if deg >= angles[-1]:
        return float(values[-1])
    index = 0
    for i in range(len(angles) - 1):
        if angles[i] <= deg <= angles[i + 1]:
            index = i
            break
    span = angles[index + 1] - angles[index]
    return values[index] + (values[index + 1] - values[index]) * (deg - angles[index]) / span


def build_probe_csv(path, joint, start_deg, lo, hi, speed, duration):
    from desktop.core.joint_target_adapter import active_mapping_identity
    mapping = active_mapping_identity()
    ramp = abs(lo - start_deg) / RAMP_SPEED_DEG_S
    total = PROBE_SPAN_HEAD_S + ramp + duration + PROBE_TAIL_S
    leg = (hi - lo) / speed
    n = int(round(total / SAMPLE_PERIOD_S))
    fields = ['host_time', 'source', 'source_kind', 'frame_kind', 'replayable', 'stream_id',
              'command_joint_mask', 'valid', 'blocked', 'sent_to_backend', 'input_space',
              'output_space', 'mapping_id', 'mapping_version', 'urdf_config_crc',
              'joint_order_crc', 'output_joint_targets_json']
    base = datetime(2026, 11, 19, 0, 0, 0)
    previous = start_deg
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for i in range(n + 1):
            t = min(i * SAMPLE_PERIOD_S, total)
            if t <= PROBE_SPAN_HEAD_S:
                value = start_deg
            elif t <= PROBE_SPAN_HEAD_S + ramp:
                value = start_deg + math.copysign(RAMP_SPEED_DEG_S, lo - start_deg) * (t - PROBE_SPAN_HEAD_S)
            elif t <= PROBE_SPAN_HEAD_S + ramp + duration:
                body = t - PROBE_SPAN_HEAD_S - ramp
                phase = body % (2 * leg)
                value = lo + (speed * phase if phase < leg else speed * (2 * leg - phase))
            else:
                value = previous
            previous = value
            targets = [0.0] * 21
            targets[joint] = round(value, 3)
            writer.writerow([(base + timedelta(seconds=t)).isoformat(timespec='microseconds'),
                             'generated:calib-probe', 'generated', 'continuous', 1,
                             f'calib-probe-j{joint:02d}', 1 << joint, 1, 0, 1, 'urdf_deg', 'urdf_deg',
                             mapping.mapping_id, mapping.mapping_version,
                             mapping.urdf_config_crc, mapping.joint_order_crc, json.dumps(targets)])
    return total


def read_lut_from_bundle(joint, motor):
    from desktop.paths import CALIB_ACTIVE_SNAPSHOT_PATH
    payload = json.loads(CALIB_ACTIVE_SNAPSHOT_PATH.read_text(encoding='utf-8'))
    result = payload['joint_calib_results'][joint]
    return lut_from_result(result, motor) if isinstance(result, dict) and result.get('success') else None


def current_lut(hand, joint, motor):
    """标定刚结束时以 P4 回读的快照为准，保存包可能还没落盘。"""
    state = hand.get_state_snapshot()
    results = getattr(state, 'joint_calib_results', None) or []
    result = results[joint] if joint < len(results) else None
    if isinstance(result, dict) and result.get('success'):
        return lut_from_result(result, motor)
    lut = read_lut_from_bundle(joint, motor)
    if lut is None:
        raise RuntimeError(f'J{joint:02d} has no valid calibration result after calibration')
    return lut


def run_calibration(hand, joints, timeout):
    hand.select_joints(list(joints))
    result = hand.request_calibration(origin='Motor Calibration Probe')
    return wait_result(hand, result, 'Motor calibration', timeout=timeout)


def wait_replay_with_guards(hand, joint, motor, total_s, max_error_deg=MAX_FOLLOW_ERROR_DEG,
                            max_error_s=MAX_FOLLOW_ERROR_S, limit_guard=True):
    from desktop.core.protocol import SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK
    deadline = time.monotonic() + total_s + 30.0
    slow_since = None
    while time.monotonic() < deadline:
        snapshot = hand.application_snapshot(include_curves=False)
        replay = snapshot['replay'].get('state')
        if replay in ('completed', 'aborted', 'failed'):
            if replay != 'completed':
                raise RuntimeError(f'Replay ended: {replay}')
            return
        state = hand.get_state_snapshot()
        if int(state.system_state) not in (SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK):
            raise RuntimeError(f'P4 left feedback control (state {state.system_state})')
        if int(state.system_fault_bitmap):
            raise RuntimeError(f'P4 fault 0x{state.system_fault_bitmap:08X}')
        target = state.target_angles[joint]
        angle = state.encoders[joint].current_angle_deg
        if angle is not None and abs(target - angle) > max_error_deg:
            slow_since = slow_since or time.monotonic()
            if time.monotonic() - slow_since > max_error_s:
                raise RuntimeError(f'J{joint:02d} follow error {target - angle:.1f}°; aborting')
        else:
            slow_since = None
        if limit_guard and state.motor_abs_limit_valid_bitmap & (1 << motor):
            lo = state.motor_abs_limit_min[motor] + LIMIT_GUARD_COUNTS
            hi = state.motor_abs_limit_max[motor] - LIMIT_GUARD_COUNTS
            abs_value = state.servo_angles[motor]
            if lo <= hi and abs_value is not None and not (lo <= abs_value <= hi):
                raise RuntimeError(f'M{motor:02d} reached its tendon limit ({abs_value}); aborting')
        time.sleep(0.1)
    raise TimeoutError('Replay did not finish')


def collect_probe(trace_dir, joint, motor):
    trace_path = Path(trace_dir) / '08_raw_frames.csv'
    traces, details = {}, {}
    with trace_path.open(newline='', encoding='utf-8-sig') as handle:
        for row in csv.DictReader(handle):
            if row.get('packet_type') != '24':
                continue
            payload = bytes.fromhex(row['payload_hex'])[:-4]
            if len(payload) == 240:
                sequence = struct.unpack_from('>I', payload, 0)[0]
                details[sequence] = struct.unpack_from('>27i', payload, 24)[motor]
            elif len(payload) == 220:
                sequence = struct.unpack_from('>7I', payload, 0)[0]
                traces[sequence] = struct.unpack_from('>21h', payload, 28 + 42)[joint] / 100.0
    return [(traces[s], details[s]) for s in traces if s in details]


def summarize(points, lut, joint, motor, lut_before, out_dir, duration_s=None):
    residuals = [motor_abs - lut_model(lut, angle) for angle, motor_abs in points]
    ordered = sorted(points)
    window = max(1, len(points) // 10)
    head = [motor_abs - lut_model(lut, angle) for angle, motor_abs in ordered[:window]]
    tail = [motor_abs - lut_model(lut, angle) for angle, motor_abs in ordered[-window:]]
    angles = [angle for angle, _ in points]
    motor_values = [value for _, value in points]
    slope = 0.0
    if len(points) > 2 and max(angles) - min(angles) > 10:
        n = len(points)
        mean_a = sum(angles) / n
        mean_m = sum(motor_values) / n
        cov = sum((a - mean_a) * (m - mean_m) for a, m in points)
        var = sum((a - mean_a) ** 2 for a in angles)
        slope = cov / var
    lut_slope = ((lut['values'][-1] - lut['values'][0]) / (lut['angles'][-1] - lut['angles'][0]))
    before = None if lut_before is None else lut_before
    report = {
        'joint': joint,
        'motor': motor,
        'samples': len(points),
        'angle_min': min(angles),
        'angle_max': max(angles),
        'residual_mean': sum(residuals) / len(residuals),
        'residual_p95': sorted(abs(r) for r in residuals)[int(0.95 * len(residuals))],
        'residual_max_abs': max(abs(r) for r in residuals),
        'residual_head_mean': sum(head) / len(head),
        'residual_tail_mean': sum(tail) / len(tail),
        'residual_drift': sum(tail) / len(tail) - sum(head) / len(head),
        'runtime_slope_counts_per_deg': slope,
        'lut_slope_counts_per_deg': lut_slope,
        'lut_after': lut,
        'lut_before': before,
        'duration_s': duration_s,
    }
    (out_dir / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding='utf-8')
    plot_probe(points, lut, lut_before, report, out_dir)
    return report


def plot_probe(points, lut, lut_before, report, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    for name in ('Noto Sans CJK SC', 'Noto Sans CJK TC'):
        if any(f.name == name for f in fm.fontManager.ttflist):
            plt.rcParams['font.sans-serif'] = [name]
            break
    plt.rcParams['axes.unicode_minus'] = False
    fig, (left, right) = plt.subplots(1, 2, figsize=(12.5, 4.8))
    grid = [lut['angles'][0] + i * 0.5 for i in range(int((lut['angles'][-1] - lut['angles'][0]) / 0.5) + 1)]
    left.plot(grid, [lut_model(lut, deg) for deg in grid], ls='--', lw=1.5,
              color='#1f77b4', label='新标定 LUT')
    if lut_before:
        left.plot(grid, [lut_model(lut_before, deg) for deg in grid], ls=':', lw=1.2,
                  color='#999999', label='旧标定 LUT')
    angles = [p[0] for p in points]
    motors = [p[1] for p in points]
    scatter = left.scatter(angles, motors, c=range(len(points)), cmap='viridis', s=6)
    left.set_xlabel('关节角 (°)')
    left.set_ylabel('电机 ABS (counts)')
    left.grid(alpha=0.3)
    left.legend(loc='lower right', fontsize=8)
    fig.colorbar(scatter, ax=left, label='采样顺序（早→晚）')
    times = [i * SAMPLE_PERIOD_S for i in range(len(points))]
    residuals = [motor - lut_model(lut, angle) for angle, motor in points]
    right.plot(times, residuals, lw=1.0, color='#d62728')
    right.axhline(report['residual_head_mean'], ls='--', lw=1.0, color='#2e7d32',
                  label=f"前段均值 {report['residual_head_mean']:+.0f}")
    right.axhline(report['residual_tail_mean'], ls='--', lw=1.0, color='#c62828',
                  label=f"后段均值 {report['residual_tail_mean']:+.0f}")
    right.set_xlabel('探测时间 (s)')
    right.set_ylabel('实测 − 新 LUT (counts)')
    right.grid(alpha=0.3)
    right.legend(loc='best', fontsize=8)
    span = report.get('duration_s')
    span_text = f'{span:.0f} s' if span else '探测'
    fig.suptitle(f"J{report['joint']:02d}/M{report['motor']:02d} Motor Calibration 后 {span_text} 漂移", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_dir / 'drift.png', dpi=120)


def main(argv=None):
    parser = argparse.ArgumentParser(description='小拇指 Motor Calibration + 60 s 漂移探测')
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--calibrate-joints', default='0,1,2,3')
    parser.add_argument('--probe-joint', type=int, default=1)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--lo', type=float, default=10.0)
    parser.add_argument('--hi', type=float, default=60.0)
    parser.add_argument('--speed', type=float, default=20.0)
    parser.add_argument('--out', default='')
    parser.add_argument('--max-follow-error-deg', type=float, default=MAX_FOLLOW_ERROR_DEG,
                        help='跟随误差超过该值并持续 --max-follow-error-s 就中止（有障碍物时放宽）')
    parser.add_argument('--max-follow-error-s', type=float, default=MAX_FOLLOW_ERROR_S)
    parser.add_argument('--no-limit-guard', action='store_true',
                        help='不因电机顶到腱窗口而中止（长时顶障碍物实验用）')
    parser.add_argument('--skip-calibration', action='store_true')
    parser.add_argument('--calibrate-only', action='store_true',
                        help='只做标定并打印结果，不发探测轨迹（放障碍物前用）')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    joints = [int(v) for v in str(args.calibrate_joints).split(',') if str(v).strip()]
    joint = int(args.probe_joint)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = Path(args.out) if args.out else ROOT / 'logs' / f'motor_calib_probe_{stamp}'
    out_dir.mkdir(parents=True, exist_ok=True)

    from desktop.core.hand_backend import HandBackend
    from desktop.replay.replay_service import ReplayService
    from desktop.paths import CALIB_ACTIVE_SNAPSHOT_PATH

    hand = HandBackend()
    hand.open_application(replay_service=ReplayService(hand))
    try:
        hand.request_recording(False)
        print(f'[probe] connect {args.port}')
        try:
            wait_result(hand, hand.request_connect(args.port), 'Connect P4', timeout=20.0)
        except Exception as exc:
            raise SystemExit(f'连接失败（上位机可能正占用串口，先关闭上位机）：{exc}')
        # 复位后 P4 会处于 UNCONTROLLABLE（多圈坐标未知），标定流程自己会做坐标准备，
        # 这里只等协议/参数就绪，不强制 READY。
        wait_until(lambda: str(hand.application_snapshot(include_curves=False)
                               .get('communication', '')) == 'Online',
                   'P4 status online', 15.0)
        wait_until(lambda: str(hand.application_snapshot(include_curves=False)
                               .get('control_parameter_sync', '')).startswith('APPLIED'),
                   'Control parameters applied', 20.0)
        snapshot = hand.application_snapshot(include_curves=False)
        motor = int(snapshot['joints'][joint]['motors'][0])
        bundle_before = json.loads(Path(CALIB_ACTIVE_SNAPSHOT_PATH).read_text(encoding='utf-8'))
        (out_dir / 'calib_before.mmcal.json').write_text(
            json.dumps(bundle_before, ensure_ascii=False, indent=1), encoding='utf-8')
        lut_before = None
        if isinstance(bundle_before['joint_calib_results'][joint], dict) and \
                bundle_before['joint_calib_results'][joint].get('success'):
            lut_before = lut_from_result(bundle_before['joint_calib_results'][joint], motor)
        print(f"[probe] bundle saved_at={bundle_before.get('saved_at_host_time')} motor=M{motor:02d} "
              f"before_lut={'yes' if lut_before else 'invalidated/none'}")
        print(f"[probe] J{joint:02d} angle={snapshot['joints'][joint]['angle']} "
              f"range=[{snapshot['joints'][joint]['min']}, {snapshot['joints'][joint]['max']}]")

        if args.dry_run:
            print('[probe] dry run: no calibration, no motion')
            return 0

        # P4 复位后 Manual RAM 与标定 RAM 都是空的，先把磁盘标定包重新装进去
        #（等同界面上的 Load Joint & Motor Calibration），否则标定会报缺 Manual Min/Max。
        restored, restore_message = hand._controller.load_calibration_snapshot(bundle_before)
        if not restored:
            raise RuntimeError(f'Load saved calibration failed: {restore_message}')
        print(f'[probe] saved calibration loaded: {restore_message}')
        wait_until(lambda: hand.get_state_snapshot().manual_reference_committed,
                   'Manual reference committed', 10.0)

        if not args.skip_calibration:
            print(f'[probe] calibrating joints {joints} (each ~20 s)')
            run_calibration(hand, joints, timeout=300.0)
            print('[probe] calibration finished')
            for j in joints:
                state = hand.get_state_snapshot()
                result = state.joint_calib_results[j] if j < len(state.joint_calib_results) else None
                if isinstance(result, dict) and result.get('success'):
                    print(f"[probe] J{j:02d} angleMin={result.get('angleMin')} angleMax={result.get('angleMax')} "
                          f"lutN={result.get('tendonLutCount')} "
                          f"lut=[{result['tendonLutMotorAbs'][0][0]} .. {result['tendonLutMotorAbs'][0][result['tendonLutCount']-1]}]")
                else:
                    print(f'[probe] J{j:02d} calibration result missing')
            if args.calibrate_only:
                print('[probe] calibrate-only: stop before the probe run')
                return 0

        lut_after = current_lut(hand, joint, motor)
        if lut_before is not None:
            (out_dir / 'lut_before.json').write_text(json.dumps(lut_before, indent=1), encoding='utf-8')
        (out_dir / 'lut_after.json').write_text(json.dumps(lut_after, indent=1), encoding='utf-8')

        # 复位后多圈坐标未知时，先按保存的锚点恢复，否则没有有效关节角可用。
        state = hand.get_state_snapshot()
        if int(getattr(state, 'multi_turn_unknown_bitmap', 0) or 0) & (1 << motor):
            recovered, message = hand._controller.wait_for_multiturn_recovery(1 << motor, timeout_s=15.0)
            if not recovered:
                raise RuntimeError(f'Multi-turn recovery failed: {message}')
            print(f'[probe] multi-turn recovery: {message}')

        angle = hand.get_state_snapshot().encoders[joint].current_angle_deg
        if angle is None:
            raise RuntimeError(f'J{joint:02d} has no valid angle')
        if args.hi <= args.lo:
            raise SystemExit('--hi must be greater than --lo')
        joint_row = hand.application_snapshot(include_curves=False)['joints'][joint]
        safe_lo, safe_hi = joint_row['min'] + 3.0, joint_row['max'] - 3.0
        start = min(max(angle, safe_lo), safe_hi)
        print(f'[probe] start angle {angle:.1f}° -> probe {start:.1f}° ... lo={args.lo} hi={args.hi}')
        csv_path = out_dir / f'probe_j{joint:02d}.csv'
        total = build_probe_csv(csv_path, joint, start, args.lo, args.hi, args.speed, args.duration)

        from desktop.replay.replay_csv_loader import load_replay_csv
        trace = load_replay_csv(csv_path)
        if trace.warnings:
            print(f'[probe] replay warnings: {trace.warnings}')

        hand.select_joints([joint])
        wait_result(hand, hand.request_replay_upload(csv_path.name, csv_path.read_bytes()),
                    'Import probe CSV')
        # request_recording 直接返回、用后台线程真正开录，没有 operation id 可等。
        hand.request_recording(True, ['packet_frames', 'joint_wide', 'event_log'])
        wait_until(lambda: hand.application_snapshot(include_curves=False)['recording']['running'],
                   'Recording running', 15.0)
        wait_result(hand, hand.request_replay_start(), 'Replay start', timeout=15.0)
        wait_replay_with_guards(hand, joint, motor, total,
                                args.max_follow_error_deg, args.max_follow_error_s,
                                not args.no_limit_guard)
        hand.request_recording(False)
        wait_until(lambda: not hand.application_snapshot(include_curves=False)['recording']['running'],
                   'Recording stop', 10.0)
        wait_until(lambda: not (hand._recording_thread and hand._recording_thread.is_alive()),
                   'Recording writer', 15.0)
        recording = hand.application_snapshot(include_curves=False)['recording'].get('path')
        if not recording:
            raise RuntimeError('recording did not produce a directory')
        print(f'[probe] recording {recording}')

        points = collect_probe(recording, joint, motor)
        if len(points) < 100:
            raise RuntimeError(f'only {len(points)} motor/angle pairs collected')
        report = summarize(points, lut_after, joint, motor, lut_before, out_dir, args.duration)
        print(f"[probe] samples={report['samples']} angle {report['angle_min']:.1f}..{report['angle_max']:.1f}°")
        print(f"[probe] residual mean={report['residual_mean']:+.0f} p95={report['residual_p95']:.0f} "
              f"max={report['residual_max_abs']:.0f} counts")
        print(f"[probe] drift head={report['residual_head_mean']:+.0f} tail={report['residual_tail_mean']:+.0f} "
              f"delta={report['residual_drift']:+.0f} counts")
        print(f"[probe] runtime slope={report['runtime_slope_counts_per_deg']:.1f} "
              f"vs LUT {report['lut_slope_counts_per_deg']:.1f} counts/deg")
        print(f'[probe] artifacts in {out_dir}')
        return 0
    finally:
        try:
            hand.stop_motion(emergency=False, source='calibration probe cleanup')
        except Exception:
            pass
        hand.close_application()


if __name__ == '__main__':
    raise SystemExit(main())
