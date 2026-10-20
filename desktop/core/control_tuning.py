"""Pure preflight, trajectories and measurements for repeatable control tuning.

职责：检查现有标定、生成内部工作区轨迹和计算磁编坐标中的控制指标。
线程：由离线分析或试验线程调用；无共享可变状态。
输入：标定JSON、试验定义和P4时间轴样本。
输出：可审计的有效范围、测试矩阵和数值指标。
状态所有权：不拥有设备、参数存储或运动授权。
禁止依赖：禁止串口、UI、运动命令及隐式参数修改。
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass
import numpy as np


def bitmap(value):
    return int(value, 0) if isinstance(value, str) else int(value)


def calibration_audit(bundle):
    """A success flag alone is insufficient: every required slope must work."""
    results = []
    for joint, lut in enumerate(bundle['joint_calib_results']):
        if not lut or not lut.get('success'):
            continue
        reasons, flat, changes = [], [], []
        lo, hi = float(lut['currentMinDeg']), float(lut['currentMaxDeg'])
        margin = max(5.0, (hi - lo) * .1)
        motors = [int(m) for m in lut['tendonLutMotor'] if int(m) != 255]
        n, step, start = int(lut['tendonLutCount']), float(lut['tendonLutAngleStepDeg']), float(lut['tendonLutAngleStartDeg'])
        if (n < 2 or not math.isfinite(step) or step <= 0 or
                not all(math.isfinite(v) for v in (lo, hi, start)) or
                lo >= hi or not lut.get('tendonLutValid')):
            reasons.append('invalid LUT shape')
        if not motors or len(set(motors)) != len(motors):
            reasons.append('missing/duplicate motor binding')
        if not bitmap(bundle['manual_reference_valid_bitmap']) & (1 << joint):
            reasons.append('joint calibration absent')
        for axis, motor in enumerate(motors):
            limits = bitmap(bundle['motor_abs_limit_valid_bitmap'])
            if not limits & (1 << motor) or bundle['motor_abs_limit_min'][motor] >= bundle['motor_abs_limit_max'][motor]:
                reasons.append(f'M{motor:02d} ABS limits invalid')
            values = lut['tendonLutMotorAbs'][axis][:n]
            if len(values) != n:
                reasons.append(f'M{motor:02d} truncated LUT')
            if step <= 0 or not math.isfinite(step):
                continue
            if start > lo or start + (n-1)*step < hi:
                reasons.append(f'M{motor:02d} LUT does not cover calibrated range')
            slopes = []
            for i in range(1, len(values)):
                left, right = max(lo, start + (i - 1) * step), min(hi, start + i * step)
                if right <= left:
                    continue
                slope = (values[i] - values[i - 1]) / step
                slopes.append(slope)
                if not math.isfinite(slope) or abs(slope) <= .001:
                    flat.append(dict(motor=motor, low_deg=left, high_deg=right))
            signs = {int(math.copysign(1, s)) for s in slopes if abs(s) > .001}
            if len(signs) > 1:
                changes.append(motor)
        if flat:
            reasons.append('zero/nonfinite local slope; do not synthesize a replacement')
        if changes:
            reasons.append('local slope changes sign')
        if lo + margin >= hi - margin:
            reasons.append('no internal workspace')
        results.append(dict(joint=joint, motors=motors, low_deg=lo, high_deg=hi,
                            internal_low_deg=lo + margin, internal_high_deg=hi - margin,
                            eligible=not reasons, reasons=reasons, flat_segments=flat,
                            sign_change_motors=changes))
    return results


@dataclass(frozen=True)
class Trial:
    case: str
    name: str
    kind: str
    center: float
    amplitude: float = 0
    frequency: float = 0
    duration: float = 20
    discard: float = 0
    repetitions: int = 3
    speed: float = 0

    def target(self, elapsed):
        if self.kind == 'hold':
            return self.center
        if self.kind == 'jitter':
            return self.center + .05*math.sin(2*math.pi*7*elapsed)
        if self.kind == 'step':
            return self.center + (self.amplitude if elapsed >= 1 else 0)
        if self.kind == 'sine':
            return self.center + self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed)
        if self.kind == 'square':
            return self.center + self.amplitude * (1 if int(elapsed*2*self.frequency)%2 == 0 else -1)
        if self.kind == 'triangle':
            # Start at the center, continuous turns, exactly bounded amplitude.
            return self.center + 2 * self.amplitude / math.pi * math.asin(math.sin(
                self.speed / self.amplitude * math.pi / 2 * elapsed))
        raise ValueError(self.kind)


def case_matrix(low, high):
    center, width = (low + high) / 2, high - low
    if width < 20:
        raise ValueError('less than20 degrees internal workspace; reduced protocol requires review')
    trials = []
    for fraction in (.2, .5, .8):
        trials.append(Trial('A', f'hold-{fraction}', 'hold', low + fraction * width,repetitions=1))
    # Cases B/C use positions with room for both signed target offsets.
    for fraction in (.2, .5, .8):
        q = low + fraction * width
        for amplitude in (.5, 2, 5, 10):
            if amplitude<=2 and fraction!=.5: continue
            for sign in (-1, 1):
                if low <= q + sign * amplitude <= high:
                    trials.append(Trial('B' if amplitude <= 2 else 'C',
                                        f'step-{fraction}-{sign*amplitude}', 'step', q,
                                        sign * amplitude, duration=6))
    a = min(10, width / 2)
    for amplitude in (2,5,10):
        for frequency in (.05,.1,.2):
            trials.append(Trial('C',f'square-{amplitude}-{frequency}','square',center,amplitude,frequency,
                                5/frequency,repetitions=1))
        for frequency in (.05, .1, .2, .3, .5):
            trials.append(Trial('E', f'sine-{amplitude}-{frequency}', 'sine', center, amplitude, frequency,
                                7/frequency, 2/frequency,repetitions=1))
    for amplitude in (5,10):
        for speed in (5, 10, 20):
            period=4*amplitude/speed
            trials.append(Trial('D',f'triangle-{amplitude}-{speed}','triangle',center,amplitude,
                                1/period,6*period,period,repetitions=1,speed=speed))
    # All three waveforms participate in each P/rate candidate screen.
    trials.extend([
        Trial('P','screen-square','square',center,5,.1,50,repetitions=1),
        Trial('P','screen-triangle','triangle',center,5,.5,12,2,repetitions=1,speed=10),
        Trial('P','screen-triangle-slow','triangle',center,5,.25,24,4,repetitions=1,speed=5),
        Trial('P','screen-sine','sine',center,5,.2,35,10,repetitions=1),
    ])
    trials.extend([
        Trial('F', 'reversal', 'sine', center, a, .2, 50),
        Trial('F', 'reversal-triangle', 'triangle', center, a, 10/(4*a), 4*a/10*10, speed=10),
        Trial('F', 'reversal-square', 'square', center, 5, .1, 100),
        Trial('G', 'pair-screen', 'sine', center, 5, .2, 25,repetitions=1),
        Trial('G', 'pair-screen-triangle', 'triangle', center, 5, .5, 10,repetitions=1,speed=10),
        Trial('G', 'pair-screen-square', 'square', center, 5, .1, 50,repetitions=1),
        Trial('G', 'pair-confirm', 'sine', center, 5, .2, 150,repetitions=1),
        Trial('G', 'pair-confirm-triangle', 'triangle', center, 5, .5, 60,repetitions=1,speed=10),
        Trial('G', 'pair-confirm-square', 'square', center, 5, .1, 300,repetitions=1),
        Trial('K', 'repeat-sine', 'sine', center, 5, .2, 200, repetitions=1),
        Trial('K', 'repeat-triangle', 'triangle', center, 5, .5, 200,repetitions=1,speed=10),
        Trial('K', 'repeat-square', 'square', center, 5, .1, 200,repetitions=1),
    ])
    return [asdict(t) for t in trials]


def metrics(samples, *, event_s=None, baseline=None, target=None, discard_s=0):
    """No fabricated metrics for missing data, tiny steps or failed trials."""
    if not samples:
        return dict(valid=False, reason='no samples')
    t = np.asarray([r['t'] for r in samples], dtype=float)
    q = np.asarray([r['q'] for r in samples], dtype=float)
    ref = np.asarray([r['target'] for r in samples], dtype=float)
    if not np.all(np.isfinite(np.concatenate((t, q, ref)))) or np.any(np.diff(t) <= 0):
        return dict(valid=False, reason='invalid/nonmonotonic samples')
    use = t >= discard_s
    if np.count_nonzero(use) < 10:
        return dict(valid=False, reason='insufficient samples')
    e = ref[use] - q[use]
    dt = np.diff(t)
    result = dict(valid=True, samples=int(np.count_nonzero(use)),
                  duration_s=float(t[-1]-t[0]), rmse_deg=float(np.sqrt(np.mean(e*e))),
                  p95_error_deg=float(np.percentile(np.abs(e),95)),
                  mae_deg=float(np.mean(np.abs(e))), max_error_deg=float(np.max(np.abs(e))),
                  max_sample_gap_s=float(np.max(dt)) if len(dt) else None,
                  jitter_deg=float(np.std(q[t >= max(discard_s, t[-1]-2)])),
                  collision_events=sum(bool(r.get('collision')) and not bool(samples[i-1].get('collision'))
                                       for i,r in enumerate(samples) if i))
    if event_s is not None and baseline is not None and target is not None:
        size = abs(target - baseline)
        tolerance = min(1., max(.15, .1*size))
        post = np.flatnonzero(t >= event_s)
        band = np.abs(q - target) <= tolerance
        settled = None
        for i in post:
            end = np.searchsorted(t, t[i]+.3)
            if end < len(t) and band[i:end+1].all():
                settled = float(t[i]-event_s); break
        tail = t >= max(event_s,t[-1]-1)
        direction = 1 if target > baseline else -1
        pre=(t<event_s)&(t>=event_s-.3)
        baseline_observed=np.count_nonzero(pre)>=3
        measured_baseline=float(np.mean(q[pre])) if baseline_observed else None
        response_fraction=((float(np.mean(q[tail]))-measured_baseline)/(target-baseline)
                           if size and baseline_observed else None)
        result.update(settling_s=settled, tolerance_deg=tolerance,
                      baseline_observed=bool(baseline_observed),baseline_actual_deg=measured_baseline,
                      steady_mae_deg=float(np.mean(np.abs(q[tail]-target))),
                      response_fraction=response_fraction,
                      overshoot_deg=float(max(0,np.max(direction*(q[post]-target)))) if len(post) else None)
        crossings = []
        for fraction in (.1, .9):
            reached = [i for i in post if measured_baseline is not None and direction*(q[i]-measured_baseline) >= fraction*size]
            crossings.append(float(t[reached[0]]) if reached and size else None)
        result['rise_10_90_s'] = (crossings[1]-crossings[0]
                                  if all(v is not None for v in crossings) else None)
        result['goal_met'] = bool(settled is not None and response_fraction is not None and response_fraction>=.5 and
                                  result['steady_mae_deg'] <= tolerance and
                                  (size < 5 or (settled <= 1.5 and result['overshoot_deg'] <= 1)))
    return result


def noise_metrics(rows):
    """Encoder-coordinate noise only; no physical accuracy or motion readiness claim."""
    if len(rows) < 10:
        return dict(valid=False, reason='insufficient disabled samples')
    result = []
    for axis in range(len(rows[0]['q'])):
        values = np.asarray([r['q'][axis] for r in rows], dtype=float)
        if not np.all(np.isfinite(values)):
            return dict(valid=False, reason='invalid disabled encoder values')
        result.append(dict(mean_deg=float(np.mean(values)), std_deg=float(np.std(values)),
                           peak_to_peak_deg=float(np.ptp(values)),
                           p95_deviation_deg=float(np.percentile(np.abs(values-np.median(values)),95))))
    return dict(valid=True, samples=len(rows), encoder_noise=result,
                coordinates_known=all(not r.get('multi_turn_unknown_bitmap',0) &
                                      ((1<<15)|(1<<16)|(1<<17)) for r in rows))


def sine_metrics(samples, frequency, discard_s=0):
    rows=[s for s in samples if s['t']>=discard_s]
    if len(rows)<10 or frequency<=0:
        return dict(valid=False,reason='insufficient sine data')
    t=np.array([r['t'] for r in rows]); w=2*math.pi*frequency
    design=np.column_stack((np.sin(w*t),np.cos(w*t),np.ones(len(t))))
    q=np.array([r['q'] for r in rows]); target=np.array([r['target'] for r in rows])
    actual=np.linalg.lstsq(design,q,rcond=None)[0]
    reference=np.linalg.lstsq(design,target,rcond=None)[0]
    amp=math.hypot(*reference[:2])
    if amp<.01: return dict(valid=False,reason='no reference excitation')
    phase=(math.atan2(reference[1],reference[0])-math.atan2(actual[1],actual[0])+math.pi)%(2*math.pi)-math.pi
    actual_amp=math.hypot(*actual[:2])
    residual=float(np.sqrt(np.mean((q-design@actual)**2)))
    identifiable=actual_amp>=.15 and actual_amp>=2*residual
    return dict(valid=True,amplitude_ratio=math.hypot(*actual[:2])/amp,
                phase_identifiable=identifiable,
                phase_lag_deg=math.degrees(phase) if identifiable else None,
                phase_delay_s=phase/w if identifiable else None,
                harmonic_residual_rmse_deg=residual)


def square_metrics(samples):
    """Measure each actually accepted edge separately, including unsettled halves."""
    edges=[i for i in range(1,len(samples)) if abs(samples[i]['target']-samples[i-1]['target'])>.1]
    output=[]
    for n,i in enumerate(edges):
        end=edges[n+1] if n+1<len(edges) else len(samples)
        rows=samples[max(0,i-5):end]
        if len(rows)<10: continue
        measured=metrics(rows,event_s=samples[i]['t'],baseline=samples[i-1]['target'],target=samples[i]['target'])
        output.append(dict(t=samples[i]['t'],direction='flexion' if samples[i]['target']>samples[i-1]['target'] else 'extension',**measured))
    return dict(valid=bool(output),edges=output,unsettled=sum(x.get('settling_s') is None for x in output))


def reversal_metrics(samples, frequency, threshold=.2):
    if not samples or frequency<=0: return dict(valid=False,events=[])
    t=np.array([r['t'] for r in samples]); q=np.array([r['q'] for r in samples])
    refs=np.array([r['target'] for r in samples]); turns=[]; previous=0
    for i,delta in enumerate(np.diff(refs),1):
        direction=1 if delta>.001 else -1 if delta<-.001 else 0
        if direction and previous and direction!=previous: turns.append((i,direction))
        if direction: previous=direction
    events=[]
    for index,(start,direction) in enumerate(turns):
        end=turns[index+1][0] if index+1<len(turns) else len(t)
        if t[end-1]-t[start]<.1: continue
        if index+1==len(turns) and t[end-1]-t[start]<.95/(2*frequency): continue
        peak=float(t[start]); half=t[end-1]-peak
        response=None
        anchor=q[start]
        for i in range(start,end):
            anchor=min(anchor,q[i]) if direction>0 else max(anchor,q[i])
            sustained=int(np.searchsorted(t,t[i]+.03))
            if sustained<end and np.all(direction*(q[i:sustained+1]-anchor)>=threshold):
                response=float(t[i]); break
        compensation=next((r['t'] for r in samples[start:] if r['t']<peak+half
                           and any(abs(v)>0 for v in r.get('components',{}).get('reversal_counts',[]))),None)
        events.append(dict(direction=direction,target_reversal_s=peak,response_s=response,
                           delay_s=response-peak if response is not None else None,
                           compensation_start_s=compensation,
                           compensation_delay_s=response-compensation if response is not None and compensation is not None else None))
    delays=[e['delay_s'] for e in events if e['delay_s'] is not None]
    by_direction={}
    for direction in (-1,1):
        selected=[e for e in events if e['direction']==direction]
        valid=[e['delay_s'] for e in selected if e['delay_s'] is not None]
        by_direction[str(direction)]=dict(events=len(selected),missed=sum(e['delay_s'] is None for e in selected),
            median_s=float(np.median(valid)) if valid else None,p95_s=float(np.percentile(valid,95)) if valid else None)
    return dict(valid=bool(delays),events=events,missed=sum(e['delay_s'] is None for e in events),
                by_direction=by_direction,
                median_s=float(np.median(delays)) if delays else None,
                p95_s=float(np.percentile(delays,95)) if delays else None)


def pair_metrics(samples, lut, gain=None):
    motors=[m for m in lut['tendonLutMotor'] if m!=255]
    if len(motors)!=2: return dict(applicable=False)
    n=lut['tendonLutCount']; step=lut['tendonLutAngleStepDeg']
    axis=lut['tendonLutAngleStartDeg']+step*np.arange(n)
    output=[]; expected=cancelled=0
    for row in samples:
        detail=row.get('detail',{})
        if not all(detail.get('servo_valid_mask',0)&(1<<m) for m in motors): continue
        q=row['q']; actual=np.array([detail['actual_abs'][m] for m in motors])
        model=np.array([np.interp(q,axis,lut['tendonLutMotorAbs'][i][:n]) for i in range(2)])
        slope=np.array([(np.interp(q+.01,axis,lut['tendonLutMotorAbs'][i][:n])-
                         np.interp(q-.01,axis,lut['tendonLutMotorAbs'][i][:n]))/.02 for i in range(2)])
        if np.dot(slope,slope)<1e-6: continue
        z=model-actual; perpendicular=z-slope*(np.dot(slope,z)/np.dot(slope,slope))
        if gain is not None and gain>0 and np.max(np.abs(perpendicular))*gain>1e-6:
            expected+=1
            mask=sum(1<<int(m) for m in motors)
            if detail.get('recovery_applied_mask',0)&mask!=mask: cancelled+=1
        output.append((row['t'],float(np.max(np.abs(perpendicular)))))
    if len(output)<10: return dict(applicable=True,valid=False,reason='missing same-cycle motor detail')
    values=np.array(output)
    return dict(applicable=True,valid=True,samples=len(output),
                median_counts=float(np.median(values[:,1])),p95_counts=float(np.percentile(values[:,1],95)),
                initial_median_counts=float(np.median(values[values[:,0]<=values[0,0]+.5,1])),
                final_median_counts=float(np.median(values[values[:,0]>=values[-1,0]-.5,1])),
                recovery_cancelled_fraction=cancelled/expected if expected else None,
                drift_counts_per_s=float(np.polyfit(values[:,0]-values[0,0],values[:,1],1)[0]))
