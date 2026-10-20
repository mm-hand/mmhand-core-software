"""Exercise tuning measurements and reject unusable calibration, without IO."""
import math
import numpy as np
from desktop.core.control_tuning import Trial, case_matrix, metrics, sine_metrics, reversal_metrics, calibration_audit, pair_metrics, noise_metrics


def test_small_unmoved_step_is_not_a_successful_settle():
    samples=[dict(t=i*.01,q=0,target=.5 if i>=100 else 0) for i in range(600)]
    result=metrics(samples,event_s=1,baseline=0,target=.5)
    assert result['settling_s'] is None
    assert result['response_fraction']==0
    assert result['tolerance_deg']==.15
    assert result['rise_10_90_s'] is None
    assert not result['goal_met']


def test_static_offset_inside_final_band_is_not_counted_as_a_small_step_response():
    rows=[dict(t=i*.01,q=.45,target=.5 if i>=100 else 0) for i in range(600)]
    result=metrics(rows,event_s=1,baseline=0,target=.5)
    assert result['settling_s']==0
    assert abs(result['response_fraction'])<1e-10
    assert not result['goal_met']


def test_sine_fit_separates_phase_and_amplitude():
    rows=[dict(t=i*.01,q=4*math.sin(2*math.pi*.2*(i*.01-.3)),
               target=5*math.sin(2*math.pi*.2*i*.01)) for i in range(4000)]
    result=sine_metrics(rows,.2,10)
    assert abs(result['amplitude_ratio']-.8)<1e-8
    assert abs(result['phase_delay_s']-.3)<1e-8
    assert result['harmonic_residual_rmse_deg']<1e-8


def test_nearly_motionless_sine_does_not_report_a_meaningful_phase_delay():
    rows=[dict(t=i*.01,q=.02*math.sin(2*math.pi*.2*i*.01),
               target=5*math.sin(2*math.pi*.2*i*.01)) for i in range(1000)]
    result=sine_metrics(rows,.2)
    assert result['valid'] and not result['phase_identifiable']
    assert result['phase_delay_s'] is None and result['amplitude_ratio']<.01


def test_matrix_never_exceeds_internal_workspace_and_triangle_starts_at_center():
    for description in case_matrix(-21.6,29.6):
        trial=Trial(**description)
        for t in np.linspace(0,trial.duration,501):
            assert -21.60001<=trial.target(t)<=29.60001
        if trial.kind=='triangle': assert trial.target(0)==trial.center


def test_calibration_success_with_flat_lut_is_rejected():
    lut=dict(success=True,tendonLutValid=True,currentMinDeg=0,currentMaxDeg=30,
             tendonLutCount=4,tendonLutAngleStepDeg=10,tendonLutAngleStartDeg=0,
             tendonLutMotor=[0,255],tendonLutMotorAbs=[[0,100,100,100]])
    bundle=dict(joint_calib_results=[lut],manual_reference_valid_bitmap='0x1',
                motor_abs_limit_valid_bitmap=1,motor_abs_limit_min=[-10],motor_abs_limit_max=[200])
    audit=calibration_audit(bundle)[0]
    assert not audit['eligible']
    assert audit['flat_segments'][0]['low_deg']==10
    assert lut['tendonLutMotorAbs'][0]==[0,100,100,100]


def test_missing_trace_does_not_produce_success_or_pair_measurement():
    assert not metrics([])['valid']
    assert not metrics([dict(t=1,q=0,target=0)]*10)['valid']
    lut=dict(tendonLutMotor=[0,1],tendonLutCount=2,tendonLutAngleStepDeg=20,
             tendonLutAngleStartDeg=0,tendonLutMotorAbs=[[0,100],[100,0]])
    assert not pair_metrics([dict(t=0,q=0)],lut)['valid']


def test_reversal_count_and_latency_do_not_invent_missing_responses():
    rows=[dict(t=i*.01,q=0,target=5*math.sin(2*math.pi*.2*i*.01)) for i in range(2000)]
    result=reversal_metrics(rows,.2)
    assert not result['valid'] and result['missed']>0
    assert result['median_s'] is None


def test_step_rise_time_and_steady_error_have_distinct_meanings():
    rows=[]
    for i in range(600):
        t=i*.01
        rows.append(dict(t=t,q=0 if t<1 else min(10,(t-1)*20),target=0 if t<1 else 10))
    result=metrics(rows,event_s=1,baseline=0,target=10)
    assert abs(result['rise_10_90_s']-.4)<.011
    assert result['goal_met']
    assert result['overshoot_deg']==0


def test_disabled_noise_is_not_motion_readiness():
    rows=[dict(q=[1,2],multi_turn_unknown_bitmap=1<<17) for _ in range(20)]
    result=noise_metrics(rows)
    assert result['valid'] and not result['coordinates_known']
    assert result['encoder_noise'][0]['std_deg']==0


def test_candidate_screen_requires_all_three_waveforms():
    trials=[Trial(**r) for r in case_matrix(-21.6,29.6) if r['case']=='P']
    assert {r.kind for r in trials}=={'sine','square','triangle'}
    square=next(t for t in trials if t.kind=='square')
    assert square.target(0)==square.center+5
    assert square.target(5)==square.center-5
    assert square.target(10)==square.center+5


def test_recorded_feedback_age_preserves_snapshot_order_and_wrap():
    from desktop.core.data_recording.packet_recorder import signed_age_ms
    assert signed_age_ms(1000,1001)==-1
    assert signed_age_ms(1000,849)==151
    assert signed_age_ms(5,0xfffffff0)==21


def test_export_preserves_interrupted_samples_and_missing_values(tmp_path):
    import json,csv
    from tools.debug.export_control_tuning import export
    source=tmp_path/'interrupted.jsonl'
    original='\n'.join(json.dumps(dict(t=i*.1,q=1,target=2,missing=None)) for i in range(20))+'\n{'
    source.write_text(original)
    index=export(tmp_path)
    assert source.read_text()==original
    assert index[0]['malformed_lines']==1 and index[0]['rows']==20
    with (tmp_path/'derived/interrupted.csv').open(encoding='utf-8-sig') as f:
        rows=list(csv.DictReader(f))
    assert rows[0]['missing']==''
    assert (tmp_path/'derived/interrupted.svg').exists()
