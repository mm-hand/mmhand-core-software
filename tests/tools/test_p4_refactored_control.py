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
    assert(one.count==1 && one.targets[0]==110); // measured 100 + 1.0 * signed local slope 10 * 1
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
    assert(damped.derivative[0] < -20.0f && damped.targets[0] < 90); // measured damping, no target kick
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

    // Adaptive Kp: a persistent lag plus a clean hold raises only that joint.
    p.values[AdaptiveKpEnabled] = 1.0f;
    p.values[AdaptiveKpLagLimitS] = 0.12f;
    p.values[AdaptiveKpOvershootDeg] = 2.0f;
    JointFeedbackController adaptive;
    FeedbackEstimate afb; afb.ready = afb.fresh = true; afb.sampleDt = 0.01f;
    ServoAngleData_t aservo; aservo.onlineStatus[0] = 1; aservo.onlineStatus[1] = 1;
    ServoTelemetryData_t atel;
    for (uint32_t t = 1000; t <= 61000; t += 10) {
        const float atarget = t < 1500 ? (t - 1000) * 0.02f : 10.0f;
        afb.fast = atarget - 3.0f;
        afb.slow = afb.fast;
        aservo.servoAngles[0] = 1000 + (int32_t)(afb.fast * 10.0f);
        adaptive.compute(0, atarget, afb, t, aservo, atel, lut, 3, p, ramp);
    }
    assert(adaptive.kpScaleOf(0) > 1.25f);
    assert(fabsf(adaptive.kpScaleOf(1) - 1.0f) < 1e-6f); // no cross-joint coupling

    // A completed move/hold with 3 deg overshoot lowers that joint once.
    JointFeedbackController overshoot;
    FeedbackEstimate ofb; ofb.ready = ofb.fresh = true; ofb.sampleDt = 0.01f;
    for (uint32_t t = 1000; t <= 3200; t += 10) {
        const float target = t < 1500 ? (t - 1000) * 0.02f : 10.0f;
        ofb.fast = t < 1650 ? target : 13.0f;
        ofb.slow = ofb.fast;
        aservo.servoAngles[0] = 1000 + (int32_t)(ofb.fast * 10.0f);
        overshoot.compute(0, target, ofb, t, aservo, atel, lut, 3, p, ramp);
    }
    assert(fabsf(overshoot.kpScaleOf(0) - 0.9f) < 0.01f);

    // Revision changes reset the learned scale; disabling falls back to base Kp.
    p.revision++;
    adaptive.compute(0, 10.0f, afb, 61010, aservo, atel, lut, 3, p, ramp);
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
