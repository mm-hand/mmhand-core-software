"""Execute production PI/filter code against deterministic synthetic LUTs, without hardware."""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

from desktop.core.control_parameters import configuration_payload

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / 'ESP32-P4/src'


def test_measured_base_signed_slopes_pair_recovery_windows_and_anchored_reversal(tmp_path):
    compiler = shutil.which('cl') or shutil.which('g++') or shutil.which('clang++')
    assert compiler, 'Run under a native compiler environment (MSVC vcvars64 or GCC/Clang)'
    def stripped(path):
        return re.sub(r'^#(?:include|pragma).*\n', '', path.read_text(encoding='utf-8'), flags=re.M)
    generated = (SRC / 'generated/ControlParameters.generated.h').read_text(encoding='utf-8')
    fixture = '''#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#define MAX_JOINT_ACTUATORS 2
#define TENDON_LUT_ACTUATOR_COUNT 2
#define ENCODER_TOTAL_NUM 21
#define SERVO_TOTAL_NUM 27
#define COLLISION_WINDOW_MS 500
namespace mmhand::config::control { inline constexpr float kFeedbackDerivativeFilterMs = 25.0f; inline constexpr float kFeedbackDerivativeLimitDegPerS = 400.0f; }
struct SharedSafetyGuard {};
struct ServoAngleData_t { int32_t servoAngles[27] = {}; uint8_t onlineStatus[27] = {}; };
struct ServoTelemetryData_t { int16_t load[27] = {}; };
struct JointCalibrationResult { uint8_t tendonLutMotor[2] = {0,1}; };
struct SystemRuntimeContext { uint32_t motor_abs_limit_valid_bitmap = 0x7ffffff; int32_t motor_abs_limit_min[27]; int32_t motor_abs_limit_max[27]; };
struct JointActuatorItem { uint8_t motorChannel; };
inline uint8_t getJointActuatorCount(uint8_t joint) { return joint == 1 || joint == 20 ? 2 : 1; }
inline bool getJointActuator(uint8_t, uint8_t axis, JointActuatorItem* out) { out->motorChannel = axis; return true; }
inline bool interpolateTendonCalibrationMotorAbs(const JointCalibrationResult&, uint8_t axis, float q, float* out) { *out = 100 + (axis ? -20 : 10)*q; return true; }
inline bool tendonCalibrationMiddleSlope(const JointCalibrationResult&, uint8_t axis, float* out) { *out = axis ? -20.0f : 10.0f; return true; }
inline bool singleTendonCalibrationMotorAbsRange(const JointCalibrationResult&,uint8_t,uint8_t,int32_t,int32_t,int32_t* lo,int32_t* hi) { *lo=-10000; *hi=10000; return true; }
inline bool tendonCalibrationMotorAbsRangeWithMargins(const JointCalibrationResult& c,uint8_t a,uint8_t m,int32_t r,int32_t t,int32_t* lo,int32_t* hi) { return singleTendonCalibrationMotorAbsRange(c,a,m,r,t,lo,hi); }
'''
    fixture += generated + '\n'
    fixture += (SRC / 'generated/RampParameters.generated.h').read_text(encoding='utf-8') + '\n'
    fixture += stripped(SRC / 'system/SnapshotTimestamp.h') + '\n'
    fixture += stripped(SRC / 'hal/servo/ServoValue.h') + '\n'
    for name in ('ControlParameters', 'RampParameters', 'FeedbackFilter', 'JointFeedbackController'):
        fixture += stripped(SRC / f'control/{name}.h') + '\n'
        source = re.sub(r'^#include ".*"\n', '', (SRC / f'control/{name}.cpp').read_text(encoding='utf-8'), flags=re.M)
        (tmp_path / f'{name}.cpp').write_text('#include "fixture.h"\n' + source, encoding='utf-8')
    fixture += stripped(SRC / 'safety/policies/CollisionSafetyPolicy.h')
    collision = re.sub(r'^#include ".*"\n', '', (SRC / 'safety/policies/CollisionSafetyPolicy.cpp').read_text(encoding='utf-8'), flags=re.M)
    (tmp_path / 'CollisionSafetyPolicy.cpp').write_text('#include "fixture.h"\n'+collision,encoding='utf-8')
    (tmp_path / 'fixture.h').write_text(fixture, encoding='utf-8')
    config = json.loads((ROOT / 'desktop/resources/config/control_parameters.default.json').read_text())
    payload = ','.join(str(value) for value in configuration_payload(config))
    harness = r'''
#include "fixture.h"
#include <cassert>
using namespace mmhand::control;
int main() {
    assert(p4::system::snapshotFreshAtCycle(1000,1001,150,10));
    assert(p4::system::snapshotFreshAtCycle(1000,1010,150,10));
    assert(!p4::system::snapshotFreshAtCycle(1000,1011,150,10));
    assert(p4::system::snapshotFreshAtCycle(1000,850,150,10));
    assert(!p4::system::snapshotFreshAtCycle(1000,849,150,10));
    assert(p4::system::snapshotFreshAtCycle(5,0xfffffff0U,150,10));
    assert(!p4::system::snapshotFreshAtCycle(1000,0,150,10));
    uint8_t payload[] = {PAYLOAD};
    assert(!parametersConfigured());
    assert(applyParameterPayload(payload,sizeof(payload)));
    Parameters p = readParameters(); assert(p.configured && p.revision == 1);
    // Keep this native test independent of the product default tuning values.
    p.values[JointKpAa]=1.0f; p.values[JointKpMcp]=1.0f;
    p.values[DualTendonPairRecoveryGain]=0.05f; p.values[ReversalTakeupMaxAa]=80;
    payload[7] ^= 1; assert(!applyParameterPayload(payload,sizeof(payload)));
    assert(readParameters().crc == p.crc);
    FeedbackFilter filter; FeedbackEstimate e;
    for (uint32_t t=10; t<=510; t+=10) e=filter.update(t,t/10.0f,true,p);
    assert(e.ready && e.fresh && fabsf(e.fast-50.5f)<0.001f && fabsf(e.slow-26.5f)<0.001f);
    assert(!filter.update(510,123,true,p).fresh);
    assert(!filter.update(520,123,false,p).ready);
    ServoAngleData_t servo; ServoTelemetryData_t telemetry; servo.onlineStatus[0]=servo.onlineStatus[1]=1;
    servo.servoAngles[0]=servo.servoAngles[1]=100;
    JointCalibrationResult lut;
    FeedbackEstimate fb; fb.fast=fb.slow=0; fb.ready=fb.fresh=true; fb.sampleDt=0.01f;
    JointFeedbackController controller; RampParameters ramp;
    auto one=controller.compute(0,1,fb,1000,servo,telemetry,lut,3,p,ramp);
    assert(one.count==1 && one.targets[0]==110); // measured 100 + 1.0 * signed middle slope 10 * 1
    servo.servoAngles[0]=110; servo.servoAngles[1]=105;
    auto two=controller.compute(1,1,fb,1000,servo,telemetry,lut,3,p,ramp);
    assert(two.count==2 && fabsf(two.proportional[0]-10)<0.01f && fabsf(two.proportional[1]+20)<0.01f);
    assert(fabsf(two.recovery[0]+0.5f)<0.01f && fabsf(two.recovery[1]+0.25f)<0.01f);
    assert(controller.compute(1,1,fb,1010,servo,telemetry,lut,1,p,ramp).count==0); // incomplete pair rejected
    p.values[JointKpAa]=2.0f; p.values[JointKpMcp]=1.2f; p.values[JointKpPip]=0.5f;
    p.values[JointKpDip]=1.5f; p.values[JointKpCmc]=0.9f;
    servo.servoAngles[0]=servo.servoAngles[1]=100;
    JointFeedbackController grouped;
    assert(fabsf(grouped.compute(0,1,fb,2000,servo,telemetry,lut,3,p,ramp).proportional[0]-20)<0.01f); // J00 AA
    assert(fabsf(grouped.compute(2,1,fb,2000,servo,telemetry,lut,3,p,ramp).proportional[0]-5)<0.01f);  // J02 PIP
    assert(fabsf(grouped.compute(3,1,fb,2000,servo,telemetry,lut,3,p,ramp).proportional[0]-15)<0.01f); // J03 DIP
    auto mcp=grouped.compute(1,1,fb,2000,servo,telemetry,lut,3,p,ramp);                                // J01 MCP
    assert(fabsf(mcp.proportional[0]-12)<0.01f && fabsf(mcp.proportional[1]+24)<0.01f);
    auto cmc=grouped.compute(20,1,fb,2000,servo,telemetry,lut,3,p,ramp);                               // J20 CMC
    assert(fabsf(cmc.proportional[0]-9)<0.01f && fabsf(cmc.proportional[1]+18)<0.01f);
    controller.resetAll(); servo.servoAngles[0]=100;
    p.values[JointKdAa]=0.1f; p.values[JointKiAa]=0.0f;
    fb.fast=fb.slow=0; fb.fresh=true; fb.sampleDt=0.01f;
    controller.compute(0,0,fb,3000,servo,telemetry,lut,3,p,ramp);
    fb.fast=1.0f;
    auto damped=controller.compute(0,0,fb,3010,servo,telemetry,lut,3,p,ramp);
    assert(damped.derivative[0] < -20.0f && damped.targets[0] < 90); // measured velocity opposes D
    controller.resetAll(); fb.fast=fb.slow=0;
    controller.compute(0,0,fb,3100,servo,telemetry,lut,3,p,ramp);
    auto targetStep=controller.compute(0,1,fb,3110,servo,telemetry,lut,3,p,ramp);
    assert(targetStep.derivative[0] > 20.0f); // target velocity contributes positively
    controller.resetAll(); p.values[JointKdMcp]=0.1f;
    controller.compute(1,0,fb,3200,servo,telemetry,lut,3,p,ramp);
    auto pairStep=controller.compute(1,1,fb,3210,servo,telemetry,lut,3,p,ramp);
    assert(pairStep.derivative[0] > 20.0f && pairStep.derivative[1] < -40.0f);
    fb.fresh=false;
    auto staleD=controller.compute(1,2,fb,3220,servo,telemetry,lut,3,p,ramp);
    assert(staleD.derivative[0]==0 && staleD.derivative[1]==0);
    fb.fresh=true;
    p.values[JointKdAa]=0.0f; controller.resetAll(); fb.fast=fb.slow=0;
    p.values[JointKiAa]=0.2f; p.values[JointKiIntegralConfirmMs]=20;
    controller.compute(0,0,fb,990,servo,telemetry,lut,3,p,ramp);
    JointControlResult integrated;
    for(uint32_t t=1000;t<=1100;t+=10) integrated=controller.compute(0,1,fb,t,servo,telemetry,lut,3,p,ramp);
    assert(integrated.integral[0]>0);
    fb.fresh=false;
    auto unchanged=controller.compute(0,1,fb,1110,servo,telemetry,lut,3,p,ramp);
    assert(fabsf(unchanged.integral[0]-integrated.integral[0])<1e-5f); // no duplicate integration or leakage
    fb.fresh=true;
    auto pending=controller.compute(0,-1,fb,1120,servo,telemetry,lut,3,p,ramp);
    assert(pending.integral[0]>0); // an unconfirmed reversal does not clear I
    auto confirmed=controller.compute(0,-1,fb,1140,servo,telemetry,lut,3,p,ramp);
    assert(fabsf(confirmed.integral[0])<1e-5f); // confirmed reversal clears old loading bias
    controller.resetAll();
    p.values[JointKiMcp]=0.2f;
    controller.compute(1,0,fb,1190,servo,telemetry,lut,3,p,ramp);
    for(uint32_t t=1200;t<=1300;t+=10) integrated=controller.compute(1,1,fb,t,servo,telemetry,lut,3,p,ramp);
    assert(integrated.integral[0]>0 && integrated.integral[1]<0);
    controller.compute(1,-1,fb,1310,servo,telemetry,lut,3,p,ramp);
    confirmed=controller.compute(1,-1,fb,1330,servo,telemetry,lut,3,p,ramp);
    assert(fabsf(confirmed.integral[0])<1e-5f && fabsf(confirmed.integral[1])<1e-5f);
    controller.resetAll(); fb.fresh=true; p.values[JointKiAa]=0; p.values[ReversalTakeupMaxAa]=80; ramp.values[ReversalRampB0Aa]=40; ramp.values[ReversalRampVbAa]=0; ramp.values[ReversalRampLoadLimitAa]=900;
    controller.compute(0,0,fb,2000,servo,telemetry,lut,3,p,ramp);
    controller.compute(0,2,fb,2010,servo,telemetry,lut,3,p,ramp);
    controller.compute(0,0,fb,2020,servo,telemetry,lut,3,p,ramp);
    auto reverse=controller.compute(0,0,fb,2040,servo,telemetry,lut,3,p,ramp);
    assert(reverse.reversal[0]<0);
    for(uint32_t t=2050;t<=2300;t+=10) {
        servo.servoAngles[0]=reverse.targets[0];
        reverse=controller.compute(0,0,fb,t,servo,telemetry,lut,3,p,ramp);
    }
    assert(fabsf(reverse.reversal[0]) > 1.0f); // PDF additive take-up bias, not a floor
    FeedbackCollisionGuard guard;
    FeedbackCollisionInput input = {};
    input.contextActive=input.feedbackValid=input.pidPushActive=true;
    input.targetDeg=30;
    FeedbackCollisionResult collision;
    for(uint32_t t=1000;t<=1500;t+=10) {
        input.timestampMs=t; input.feedbackDeg=(t-1000)*0.02f;
        collision=guard.update(0,input);
        assert(!collision.holdActive);
        if(t<1500) assert(!collision.armed);
    }
    assert(collision.armed);
    for(uint32_t t=1510;t<=2000;t+=10) {
        input.timestampMs=t; input.feedbackDeg=10;
        collision=guard.update(0,input);
        if(t<2000) assert(!collision.holdActive);
    }
    assert(collision.holdEntered && collision.holdActive);
    input.timestampMs=2010; input.targetDeg=29;
    collision=guard.update(0,input);
    assert(collision.holdReleased && !collision.armed);
    guard.resetAll(); input.targetDeg=30; input.feedbackDeg=0;
    for(uint32_t t=3000;t<=5000;t+=10) {
        input.timestampMs=t; collision=guard.update(0,input);
        assert(!collision.armed && !collision.holdActive); // a stationary joint never arms itself
    }

    // Adaptive Kp: fresh, sustained lag raises only the moving joint.
    p.values[AdaptiveKpEnabled] = 1.0f;
    p.values[AdaptiveKpLagLimitS] = 0.12f;
    p.values[AdaptiveKpOvershootDeg] = 2.0f;
    FeedbackEstimate afb; afb.ready = afb.fresh = true; afb.sampleDt = 0.01f;
    ServoAngleData_t aservo; aservo.onlineStatus[0] = 1; aservo.onlineStatus[1] = 1;
    ServoTelemetryData_t atel;
    auto sample = [&](JointFeedbackController& c, uint32_t t, float target, float error) {
        afb.fast = target - error;
        afb.slow = afb.fast;
        return c.compute(0, target, afb, t, aservo, atel, lut, 3, p, ramp);
    };
    const float errors[] = {-3.0f, 1.0f, 2.0f};
    for (float error : errors) {
        JointFeedbackController c;
        uint32_t changed = 0;
        float previous = 1.0f;
        for (uint32_t t = 1000; t <= 18000; t += 10) {
            sample(c, t, fminf((t - 1000) * 0.01f, 140.0f), error);
            const float scale = c.kpScaleOf(0);
            if (t < 1500) assert(scale == 1.0f); // needs 500 ms of evidence
            if (scale != previous) {
                assert(!changed || t - changed >= 2000);
                changed = t;
                previous = scale;
            }
            if (t > 15500) assert(changed <= 15000); // no growth at rest
        }
        assert(fabsf(c.kpScaleOf(0) - (error == 2.0f ? 1.3f : 1.0f)) < 1e-6f);
        assert(c.kpScaleOf(1) == 1.0f);
    }

    // Small noise, slow crossings and a one-way overshoot must not lower Kp.
    for (int mode = 0; mode < 3; ++mode) {
        JointFeedbackController c;
        for (uint32_t t = 1000; t <= 6000; t += 10) {
            const float error = mode == 0 ? (t / 250 % 2 ? 1.9f : -1.9f) :
                mode == 1 ? ((t - 1000) / 1200 % 2 ? 3.0f : -3.0f) : (t < 1500 ? 3.0f : -3.0f);
            sample(c, t, 10.0f, error);
        }
        assert(c.kpScaleOf(0) == 1.0f);
    }

    // Static-target oscillation still lowers Kp at a boundary, including clock wrap.
    JointFeedbackController adaptive;
    aservo.servoAngles[0] = 20000;
    uint32_t decreased = 0;
    for (uint32_t elapsed = 0; elapsed <= 20000; elapsed += 10) {
        const uint32_t t = 0xfffff000U + elapsed;
        const float previous = adaptive.kpScaleOf(0);
        assert(sample(adaptive, t, 10, elapsed / 250 % 2 ? -3 : 3).boundaryClamped);
        if (adaptive.kpScaleOf(0) != previous) {
            assert(!decreased || elapsed - decreased >= 1000);
            decreased = elapsed;
        }
    }
    assert(fabsf(adaptive.kpScaleOf(0) - 0.3f) < 1e-6f);
    aservo.servoAngles[0] = 0;

    // Gaps, suspension, invalid data and disabling discard unfinished cycles.
    for (int mode = 0; mode < 5; ++mode) {
        JointFeedbackController c;
        for (uint32_t t = 1000; t <= 1200; t += 10) sample(c, t, 10, t < 1100 ? 3 : -3);
        uint32_t resume = 1210;
        if (mode == 0) c.suspend(0, 1200);
        if (mode == 1) {
            afb.fresh = false;
            for (uint32_t t = 1210; t <= 1400; t += 10) sample(c, t, 10, -3);
            afb.fresh = true;
            resume = 1410;
        }
        if (mode == 2) { afb.ready = false; sample(c, 1210, 10, -3); afb.ready = true; resume = 1220; }
        if (mode == 3) {
            p.values[AdaptiveKpEnabled] = 0;
            sample(c, 1210, 10, -3);
            p.values[AdaptiveKpEnabled] = 1;
            resume = 1220;
        }
        if (mode == 4) resume = 1410;
        for (uint32_t t = resume; t < resume + 100; t += 10) sample(c, t, 10, 3);
        assert(c.kpScaleOf(0) == 1.0f);
        for (uint32_t t = resume + 100; t < resume + 400; t += 10) sample(c, t, 10, t < resume + 300 ? -3 : 3);
        assert(fabsf(c.kpScaleOf(0) - 0.9f) < 1e-6f);
        for (uint32_t t = resume + 400; t < resume + 2300; t += 10) sample(c, t, 10 + (t - resume - 400) * 0.01f, 1.5f);
        assert(fabsf(c.kpScaleOf(0) - 0.9f) < 1e-6f); // 2 s cooldown after lowering
        for (uint32_t t = resume + 2300; t <= resume + 2400; t += 10) sample(c, t, 10 + (t - resume - 400) * 0.01f, 1.5f);
        assert(c.kpScaleOf(0) > 0.9f);
    }

    // A revision resets the multiplier; disabling uses the configured base Kp.
    p.revision++;
    sample(adaptive, 30000, 10, 0);
    assert(fabsf(adaptive.kpScaleOf(0) - 1.0f) < 1e-6f);
    p.values[AdaptiveKpEnabled] = 0.0f;
    assert(fabsf(adaptive.effectiveKpOf(0, p) - p[JointKpAa]) < 1e-6f);
    return 0;
}
'''.replace('PAYLOAD', payload)
    (tmp_path / 'main.cpp').write_text(harness, encoding='utf-8')
    sources = [str(tmp_path / f'{name}.cpp') for name in ('main','ControlParameters','RampParameters','FeedbackFilter','JointFeedbackController','CollisionSafetyPolicy')]
    exe = tmp_path / ('control_test.exe' if os.name == 'nt' else 'control_test')
    command = [compiler, '/nologo', '/std:c++17', '/EHsc', *sources, f'/Fe:{exe}'] if Path(compiler).name.lower()=='cl.exe' else [compiler,'-std=c++17',*sources,'-o',str(exe)]
    subprocess.run(command,cwd=tmp_path,check=True,capture_output=True,text=True,encoding='utf-8',errors='replace')
    subprocess.run([str(exe)],check=True,capture_output=True,text=True,encoding='utf-8',errors='replace')


def test_full_table_middle_slope_and_envelope_match_the_host_rule(tmp_path):
    compiler = shutil.which('cl') or shutil.which('g++') or shutil.which('clang++')
    assert compiler, 'Run under a native compiler environment (MSVC vcvars64 or GCC/Clang)'
    header = (SRC / 'system/calibration/CalibrationRuntimeStore.h').read_text(encoding='utf-8')
    match = re.search(r'struct JointCalibrationResult \{.*?\n\};', header, re.S)
    assert match, 'JointCalibrationResult struct moved; update the fixture'
    source = re.sub(r'^#include ".*"\n', '',
                    (SRC / 'system/calibration/TendonCalibrationLut.cpp').read_text(encoding='utf-8'),
                    flags=re.M)
    fixture = f'''
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#define TENDON_LUT_ACTUATOR_COUNT 2
#define TENDON_LUT_MAX_SAMPLES 24
static const uint8_t kManualScaledUrdfFeedbackMappingVersion = 4;
{match.group(0)}
inline int8_t getMotorTendonShortenAbsSign(uint8_t) {{ return 1; }}
inline int32_t motorApplyTendonDelta(uint8_t, int32_t abs, int32_t delta) {{ return abs + delta; }}
bool tendonCalibrationMiddleSlope(const JointCalibrationResult&, uint8_t, float*);
bool tendonCalibrationContainsAngle(const JointCalibrationResult&, float);
bool tendonCalibrationMotorAbsRangeWithMargins(const JointCalibrationResult&, uint8_t, uint8_t,
                                               int32_t, int32_t, int32_t*, int32_t*);
bool interpolateTendonCalibrationMotorAbs(const JointCalibrationResult&, uint8_t, float, float*);
'''
    harness = r'''
#include "fixture.h"
#include <cassert>
#include <cstdio>
static void fill(JointCalibrationResult& c, const int32_t* values, uint8_t count,
                 float start, float step, float angleMax) {
    c = JointCalibrationResult{};
    c.success = true;
    c.scaled = true;
    c.tendonLutValid = 1;
    c.mappingVersion = 4;
    c.tendonLutCount = count;
    c.tendonLutAngleStartDeg = start;
    c.tendonLutAngleStepDeg = step;
    c.angleMin = start;
    c.angleMax = angleMax;
    for (uint8_t i = 0; i < count; i++) {
        c.tendonLutMotorAbs[0][i] = values[i];
        c.tendonLutMotorAbs[1][i] = values[i];
    }
}
int main() {
    JointCalibrationResult c;
    float slope = 0.0f;
    int32_t lo = 0, hi = 0;
    // 两端陡、中间平：索引 1..5（5..25 度）严格是 abs = 3000 + 10*angle。
    const int32_t shaped[7] = {0, 3000, 3050, 3100, 3150, 3200, 6000};
    fill(c, shaped, 7, 0.0f, 5.0f, 30.0f);
    assert(tendonCalibrationMiddleSlope(c, 0, &slope));
    assert(fabsf(slope - 10.0f) < 0.01f);          // 中段斜率，不是端点斜率 200
    assert(tendonCalibrationMotorAbsRangeWithMargins(c, 0, 0, 500, 1000, &lo, &hi));
    assert(lo == -500 && hi == 7000);              // min 0-500, max 6000+1000
    // 旧 2 点表：中段斜率退化为唯一线段斜率，旧 bundle 不需要重标。
    const int32_t two[2] = {1000, 4000};
    fill(c, two, 2, -15.0f, 105.0f, 90.0f);
    assert(tendonCalibrationMiddleSlope(c, 0, &slope));
    assert(fabsf(slope - 3000.0f / 105.0f) < 0.001f);
    assert(tendonCalibrationContainsAngle(c, 90.0f));
    assert(!tendonCalibrationContainsAngle(c, 90.5f));
    // 全平表：没有可用斜率，必须拒绝而不是给出 0 增益。
    const int32_t flat[5] = {1000, 1000, 1000, 1000, 1000};
    fill(c, flat, 5, 0.0f, 5.0f, 20.0f);
    assert(!tendonCalibrationMiddleSlope(c, 0, &slope));
    printf("ok\n");
    return 0;
}
'''
    (tmp_path / 'fixture.h').write_text(fixture, encoding='utf-8')
    (tmp_path / 'TendonCalibrationLut.cpp').write_text('#include "fixture.h"\n' + source,
                                                       encoding='utf-8')
    (tmp_path / 'main.cpp').write_text(harness, encoding='utf-8')
    exe = tmp_path / ('lut_test.exe' if os.name == 'nt' else 'lut_test')
    sources = [str(tmp_path / name) for name in ('main.cpp', 'TendonCalibrationLut.cpp')]
    command = ([compiler, '/nologo', '/std:c++17', '/EHsc', *sources, f'/Fe:{exe}']
               if Path(compiler).name.lower() == 'cl.exe'
               else [compiler, '-std=c++17', *sources, '-o', str(exe)])
    subprocess.run(command, cwd=tmp_path, check=True, capture_output=True, text=True,
                   encoding='utf-8', errors='replace')
    subprocess.run([str(exe)], check=True, capture_output=True, text=True,
                   encoding='utf-8', errors='replace')
