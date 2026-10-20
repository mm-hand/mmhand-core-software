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
namespace mmhand::config::control {}
struct SharedSafetyGuard {};
struct ServoAngleData_t { int32_t servoAngles[27] = {}; uint8_t onlineStatus[27] = {}; };
struct JointCalibrationResult { uint8_t tendonLutMotor[2] = {0,1}; };
struct SystemRuntimeContext { uint32_t motor_abs_limit_valid_bitmap = 0x7ffffff; int32_t motor_abs_limit_min[27]; int32_t motor_abs_limit_max[27]; };
struct JointActuatorItem { uint8_t motorChannel; };
inline uint8_t getJointActuatorCount(uint8_t joint) { return joint == 1 ? 2 : 1; }
inline bool getJointActuator(uint8_t, uint8_t axis, JointActuatorItem* out) { out->motorChannel = axis; return true; }
inline bool interpolateTendonCalibrationMotorAbs(const JointCalibrationResult&, uint8_t axis, float q, float* out) { *out = 100 + (axis ? -20 : 10)*q; return true; }
inline bool singleTendonCalibrationMotorAbsRange(const JointCalibrationResult&,uint8_t,uint8_t,int32_t,int32_t,int32_t* lo,int32_t* hi) { *lo=-10000; *hi=10000; return true; }
inline bool tendonCalibrationMotorAbsRangeWithMargins(const JointCalibrationResult& c,uint8_t a,uint8_t m,int32_t r,int32_t t,int32_t* lo,int32_t* hi) { return singleTendonCalibrationMotorAbsRange(c,a,m,r,t,lo,hi); }
'''
    fixture += generated + '\n'
    fixture += stripped(SRC / 'system/SnapshotTimestamp.h') + '\n'
    for name in ('ControlParameters', 'FeedbackFilter', 'JointFeedbackController'):
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
    payload[7] ^= 1; assert(!applyParameterPayload(payload,sizeof(payload)));
    assert(readParameters().crc == p.crc);
    FeedbackFilter filter; FeedbackEstimate e;
    for (uint32_t t=10; t<=510; t+=10) e=filter.update(t,t/10.0f,true,p);
    assert(e.ready && e.fresh && fabsf(e.fast-50.5f)<0.001f && fabsf(e.slow-26.5f)<0.001f);
    assert(!filter.update(510,123,true,p).fresh);
    assert(!filter.update(520,123,false,p).ready);
    ServoAngleData_t servo; servo.onlineStatus[0]=servo.onlineStatus[1]=1;
    servo.servoAngles[0]=servo.servoAngles[1]=100;
    JointCalibrationResult lut;
    FeedbackEstimate fb; fb.fast=fb.slow=0; fb.ready=fb.fresh=true; fb.sampleDt=0.01f;
    JointFeedbackController controller;
    auto one=controller.compute(0,1,fb,1000,servo,lut,3,p);
    assert(one.count==1 && one.targets[0]==108); // measured 100 + .8 * signed local slope 10 * 1
    servo.servoAngles[0]=110; servo.servoAngles[1]=105;
    auto two=controller.compute(1,1,fb,1000,servo,lut,3,p);
    assert(two.count==2 && fabsf(two.proportional[0]-8)<0.01f && fabsf(two.proportional[1]+16)<0.01f);
    assert(fabsf(two.recovery[0]+0.5f)<0.01f && fabsf(two.recovery[1]+0.25f)<0.01f);
    assert(controller.compute(1,1,fb,1010,servo,lut,1,p).count==0); // incomplete pair rejected
    controller.resetAll(); servo.servoAngles[0]=100;
    p.values[SingleTendonKiPerS]=0.2f; p.values[FeedbackIntegralConfirmMs]=20;
    controller.compute(0,0,fb,990,servo,lut,3,p);
    JointControlResult integrated;
    for(uint32_t t=1000;t<=1100;t+=10) integrated=controller.compute(0,1,fb,t,servo,lut,3,p);
    assert(integrated.integral[0]>0);
    fb.fresh=false;
    auto unchanged=controller.compute(0,1,fb,1110,servo,lut,3,p);
    assert(fabsf(unchanged.integral[0]-integrated.integral[0])<1e-5f); // no duplicate integration or leakage
    fb.fresh=true;
    auto pending=controller.compute(0,-1,fb,1120,servo,lut,3,p);
    assert(pending.integral[0]>0); // an unconfirmed reversal does not clear I
    auto confirmed=controller.compute(0,-1,fb,1140,servo,lut,3,p);
    assert(fabsf(confirmed.integral[0])<1e-5f); // confirmed reversal clears old loading bias
    controller.resetAll();
    p.values[DualTendonKiPerS]=0.2f;
    controller.compute(1,0,fb,1190,servo,lut,3,p);
    for(uint32_t t=1200;t<=1300;t+=10) integrated=controller.compute(1,1,fb,t,servo,lut,3,p);
    assert(integrated.integral[0]>0 && integrated.integral[1]<0);
    controller.compute(1,-1,fb,1310,servo,lut,3,p);
    confirmed=controller.compute(1,-1,fb,1330,servo,lut,3,p);
    assert(fabsf(confirmed.integral[0])<1e-5f && fabsf(confirmed.integral[1])<1e-5f);
    controller.resetAll(); fb.fresh=true; p.values[SingleTendonKiPerS]=0;
    controller.compute(0,0,fb,2000,servo,lut,3,p);
    controller.compute(0,2,fb,2010,servo,lut,3,p);
    controller.compute(0,0,fb,2020,servo,lut,3,p);
    auto reverse=controller.compute(0,0,fb,2040,servo,lut,3,p);
    assert(reverse.reversal[0]<0);
    for(uint32_t t=2050;t<=2300;t+=10) {
        servo.servoAngles[0]=reverse.targets[0];
        reverse=controller.compute(0,0,fb,t,servo,lut,3,p);
    }
    assert(reverse.targets[0]==20); // anchor 100 minus the fixed take-up floor 80
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
    return 0;
}
'''.replace('PAYLOAD', payload)
    (tmp_path / 'main.cpp').write_text(harness, encoding='utf-8')
    sources = [str(tmp_path / f'{name}.cpp') for name in ('main','ControlParameters','FeedbackFilter','JointFeedbackController','CollisionSafetyPolicy')]
    exe = tmp_path / ('control_test.exe' if os.name == 'nt' else 'control_test')
    command = [compiler, '/nologo', '/std:c++17', '/EHsc', *sources, f'/Fe:{exe}'] if Path(compiler).name.lower()=='cl.exe' else [compiler,'-std=c++17',*sources,'-o',str(exe)]
    subprocess.run(command,cwd=tmp_path,check=True,capture_output=True,text=True,encoding='utf-8',errors='replace')
    subprocess.run([str(exe)],check=True,capture_output=True,text=True,encoding='utf-8',errors='replace')
