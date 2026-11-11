"""Execute production snapshot aggregation and LUT-bound reconstruction without hardware."""
import subprocess
from pathlib import Path

from tools.precommit_checks import compiler_environment
import shutil

SRC = Path(__file__).resolve().parents[2] / 'ESP32-P4/src'


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth, closing = 1, opening + 1
    while depth:
        depth += (source[closing] == '{') - (source[closing] == '}')
        closing += 1
    return source[start:closing]


def run_native(tmp_path, name, source):
    env = compiler_environment()
    compiler = next(path for n in ('g++', 'clang++', 'cl')
                    if (path := shutil.which(n, path=env.get('PATH'))))
    cpp, executable = tmp_path / f'{name}.cpp', tmp_path / f'{name}.exe'
    cpp.write_text(source, encoding='utf-8')
    if Path(compiler).name.lower() in ('cl', 'cl.exe'):
        args = [compiler, '/std:c++17', '/EHsc', '/utf-8', f'/I{SRC}', str(cpp),
                f'/Fo{tmp_path.resolve()}\\', f'/Fe{executable}']
    else:
        args = [compiler, '-std=c++17', f'-I{SRC}', str(cpp), '-o', str(executable)]
    subprocess.run(args, check=True, env=env, cwd=tmp_path, timeout=120)
    subprocess.run([str(executable)], check=True, env=env, cwd=tmp_path, timeout=10)


def test_direct_admission_preserves_priority_and_samples_unknown_once(tmp_path):
    implementation = (SRC / 'safety/policies/SystemReadinessPolicy.cpp').read_text(encoding='utf-8')
    header = (SRC / 'safety/policies/SystemReadinessPolicy.h').read_text(encoding='utf-8')
    source = r'''
#include <cassert>
#include <cstdint>
#include "config/SafetyConfig.h"
#include "data/SafetyTypes.h"
#include "data/SystemTypes.h"
using namespace mmhand::config::safety;
constexpr uint32_t validMotorChannelMask() { return 3; }
struct ServoAngleData_t { uint32_t timestamp = 1000; uint8_t onlineStatus[SERVO_TOTAL_NUM] = {1, 1}; };
struct SystemRuntimeContext {
    uint32_t multi_turn_unknown_bitmap = 0, servo_turn_overflow_bitmap = 0;
    uint32_t system_fault_bitmap = 0, system_fault_latched_bitmap = 0;
    bool routes = true;
    struct Feedback {
        bool available = true;
        bool peek(ServoAngleData_t* out) { *out = {}; return available; }
    } servoFeedbackQueue;
};
static unsigned unknownReads;
static SystemRuntimeContext runtime;
uint32_t sharedAtomicLoad(const uint32_t* value) {
    if (value == &runtime.multi_turn_unknown_bitmap) ++unknownReads;
    return *value;
}
bool routesControllable(SystemRuntimeContext* context, uint32_t) { return context->routes; }
namespace p4::platform::time { uint32_t monotonicMillis() { return 1000; } }
'''
    source += function(header, 'enum class DirectMotorAdmission') + ';\n'
    source += function(implementation, 'DirectMotorAdmission evaluateDirectMotorAdmission(')
    source += r'''
int main() {
    using Decision = DirectMotorAdmission;
    assert(evaluateDirectMotorAdmission(&runtime, 1) == Decision::Ready && unknownReads == 1);
    runtime.multi_turn_unknown_bitmap = 1;
    runtime.servoFeedbackQueue.available = false;
    unknownReads = 0;
    assert(evaluateDirectMotorAdmission(&runtime, 1) == Decision::MultiTurnUnknown && unknownReads == 1);
    assert(evaluateDirectMotorAdmission(&runtime, 2) == Decision::HardwareNotReady);
    runtime.servoFeedbackQueue.available = true;
    runtime.servo_turn_overflow_bitmap = 2;
    assert(evaluateDirectMotorAdmission(&runtime, 2) == Decision::HardwareNotReady);
    runtime.servo_turn_overflow_bitmap = 0;
    runtime.routes = false;
    assert(evaluateDirectMotorAdmission(&runtime, 2) == Decision::HardwareNotReady);
}
'''
    run_native(tmp_path, 'direct_admission', source)


def test_feedback_admission_reuses_readiness_when_workspace_becomes_ready(tmp_path):
    dispatcher = (SRC / 'tasks/system_state/SystemEventDispatcher.cpp').read_text(encoding='utf-8')
    start = dispatcher.index('case SYSTEM_EVENT_RUN_FEEDBACK: {')
    end = dispatcher.index('case SYSTEM_EVENT_MOTOR_DIRECT_RELEASE:', start)
    source = r'''
#include <cassert>
#include "data/ActuatorTypes.h"
#include "data/SystemTypes.h"
#define FW_LOG_INFO_PRINTF(...) ((void)0)
#define FW_LOG_INFO_PRINTLN(...) ((void)0)
struct SystemRuntimeContext {
    uint8_t system_state = SYSTEM_STATE_READY;
    uint8_t feedback_recovery_pending = 0, status = 0, reason = 0;
    uint32_t feedback_recovery_request_sequence = 0;
    bool manual = true, ready = true, enteredReady = false;
    unsigned manualReads = 0, workspaceReads = 0, grants = 0;
};
template<class T> T sharedAtomicLoad(T* value) { return *value; }
template<class T> void sharedAtomicStore(T* field, T value) { *field = value; }
namespace p4::state {
bool prepareHostControlCommandAdmission(SystemRuntimeContext*) { return true; }
}
namespace p4::system {
bool canAcceptRunFeedback(uint8_t state) { return state == SYSTEM_STATE_READY; }
}
namespace p4::recovery {
uint8_t pullJointIntoWorkspaceActiveScopeRejectReason(SystemRuntimeContext*) {
    return PULL_JOINT_INTO_WORKSPACE_REASON_NONE;
}
}
bool rejectMotionWhenEmergencyStopLatched(SystemRuntimeContext*, const char*) { return false; }
bool manualReferenceGateReady(SystemRuntimeContext* r) { ++r->manualReads; return r->manual; }
void logManualReferenceReject(SystemRuntimeContext*, const char*) {}
bool feedbackTargetsPointIntoWorkspace(SystemRuntimeContext*) { return true; }
bool activeMotorsMultiTurnKnown(SystemRuntimeContext*) { return true; }
bool beginPullJointIntoWorkspaceMotion(SystemRuntimeContext*, bool, uint32_t, uint32_t) { return true; }
struct Readiness { bool ready; uint32_t recoverableBelowMask; };
Readiness evaluateWorkspace(SystemRuntimeContext* r, bool manual) {
    assert(manual); ++r->workspaceReads; return {r->ready, 0};
}
void setFeedbackAdmissionResult(SystemRuntimeContext* r, uint32_t, uint8_t status, uint8_t reason) {
    r->status = status; r->reason = reason;
}
void setSystemState(SystemRuntimeContext* r, uint8_t state) {
    r->system_state = state;
    r->enteredReady |= state == SYSTEM_STATE_READY;
}
bool setCommandModeJoint(SystemRuntimeContext*) { return true; }
void resetServoTargetQueue(SystemRuntimeContext*) {}
bool grantMotionOwnerUnlessServoRebaseZeroPending(SystemRuntimeContext* r, uint8_t, uint8_t, uint8_t, uint32_t) {
    ++r->grants; return true;
}
void runFeedback(SystemRuntimeContext* sharedData) {
    SystemEvent_t event{};
    event.event = SYSTEM_EVENT_RUN_FEEDBACK;
    switch (event.event) {
'''
    source += dispatcher[start:end] + r'''
    }
}
int main() {
    SystemRuntimeContext r;
    r.system_state = SYSTEM_STATE_JOINT_BEYOND_WORKSPACE;
    runFeedback(&r);
    assert(r.enteredReady && r.system_state == SYSTEM_STATE_RUN_FEEDBACK);
    assert(r.manualReads == 1 && r.workspaceReads == 1 && r.grants == 1);
    assert(r.status == FEEDBACK_ADMISSION_ACCEPTED);
    r = {};
    r.ready = false;
    runFeedback(&r);
    assert(r.reason == FEEDBACK_REJECT_WORKSPACE && r.grants == 0 && r.workspaceReads == 1);
    r = {};
    r.system_state = SYSTEM_STATE_RUN_FEEDBACK;
    r.manual = false;
    runFeedback(&r);
    assert(r.reason == FEEDBACK_REJECT_MANUAL_REFERENCE && r.workspaceReads == 0 && r.grants == 0);
}
'''
    run_native(tmp_path, 'feedback_admission', source)


def test_health_snapshot_reads_each_mailbox_once_and_preserves_safety_thresholds(tmp_path):
    implementation = (SRC / 'safety/policies/SystemFaultPolicy.cpp').read_text(encoding='utf-8')
    header = (SRC / 'safety/policies/SystemFaultPolicy.h').read_text(encoding='utf-8')
    snapshot_type = function(header, 'struct SystemHealthSnapshot') + ';'
    source = r'''
#include <cassert>
#include <cstdint>
#include "config/SafetyConfig.h"
using namespace mmhand::config::safety;
constexpr int ENCODER_TOTAL_NUM=21;
constexpr uint32_t SYSTEM_FAULT_CAN_OFFLINE=1, SYSTEM_FAULT_OVERLOAD=4,
    SYSTEM_FAULT_SERVO_OFFLINE=16, SYSTEM_FAULT_MULTITURN_UNKNOWN=64;
constexpr int SYSTEM_STATE_RUN_DIRECT=3, SYSTEM_STATE_RUN_FEEDBACK=5,
    SYSTEM_STATE_PULL_JOINT_INTO_WORKSPACE_RUNNING=9;
struct RemoteSensorData_t { bool isValid=true; uint32_t timestamp=1000; };
struct ServoAngleData_t { uint32_t timestamp=1000; uint32_t onlineMask=3; };
template<class T> struct Mailbox {
    T data{}; unsigned reads=0; bool available=true;
    bool peek(T* out) { ++reads; *out=data; return available; }
};
struct SystemRuntimeContext {
    uint32_t active_joint_mask=3, encoder_online_bitmap=3, encoder_fault_bitmap=0;
    uint8_t can_link_online=1, system_state=5;
    uint32_t overload_fault_bitmap=0, multi_turn_unknown_bitmap=0, system_fault_latched_bitmap=0;
    Mailbox<RemoteSensorData_t> canRxQueue;
    Mailbox<ServoAngleData_t> servoFeedbackQueue;
};
template<class T> T sharedAtomicLoad(T* value) { return *value; }
namespace p4::state {
bool requiresCan(SystemRuntimeContext* r) { return r->system_state==5; }
uint32_t requiredMotorMask(SystemRuntimeContext*) { return 3; }
bool requiredServosOnline(SystemRuntimeContext*, const ServoAngleData_t& s) { return (s.onlineMask & 3)==3; }
}
'''
    source += snapshot_type + '\n' + function(implementation, 'SystemHealthSnapshot readSystemHealth(')
    source += r'''
int main() {
    SystemRuntimeContext r;
    auto h=readSystemHealth(&r,1500);
    assert(h.faults==0 && h.sensorFresh && h.encoderOnlineMask==3);
    assert(r.canRxQueue.reads==1 && r.servoFeedbackQueue.reads==1);
    h=readSystemHealth(&r,1501); assert(h.faults==SYSTEM_FAULT_SERVO_OFFLINE);
    h=readSystemHealth(&r,2000); assert(h.sensorFresh);
    h=readSystemHealth(&r,2001); assert(!h.sensorFresh && (h.faults & SYSTEM_FAULT_CAN_OFFLINE));
    r.canRxQueue.data.timestamp=r.servoFeedbackQueue.data.timestamp=3000;
    r.encoder_fault_bitmap=2;
    r.overload_fault_bitmap=1; r.multi_turn_unknown_bitmap=2;
    // The servo owns its own overload protection, so overload_fault_bitmap is a
    // display-only bit and no longer contributes SYSTEM_FAULT_OVERLOAD.
    h=readSystemHealth(&r,3000);
    assert(h.faults==(1|64) && h.encoderFaultMask==2);
    r.system_state=3; h=readSystemHealth(&r,3000); assert(h.faults==(64));
    r.overload_fault_bitmap=0; r.multi_turn_unknown_bitmap=0;
    r.system_fault_latched_bitmap=4; h=readSystemHealth(&r,3000); assert(h.faults==4);
    assert(readSystemHealth(nullptr,3000).faults==0);
}
'''
    run_native(tmp_path, 'health', source)


def test_loaded_single_and_dual_lut_bounds_follow_applied_parameters_and_rebuild_old_limits(tmp_path):
    dispatcher = (SRC / 'tasks/host_comm/HostCommandDispatcher.cpp').read_text(encoding='utf-8')
    lut = (SRC / 'system/calibration/TendonCalibrationLut.cpp').read_text(encoding='utf-8')
    source = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "generated/ControlParameters.generated.h"
using std::isfinite;
constexpr int ENCODER_TOTAL_NUM=2, TENDON_LUT_ACTUATOR_COUNT=2, TENDON_LUT_MAX_SAMPLES=3;
struct SharedSafetyGuard {};
struct JointCalibrationResult {
    bool success=true,scaled=true; uint8_t tendonLutValid=1,tendonLutCount=2;
    uint8_t tendonLutMotor[2]={0,1}; float tendonLutAngleStartDeg=0,tendonLutAngleStepDeg=5;
    int32_t tendonLutMotorAbs[2][3]={{100,200,0},{300,500,0}};
};
struct JointCalibrationRuntimeSnapshot { JointCalibrationResult result; };
JointCalibrationRuntimeSnapshot loaded[2];
bool readJointCalibrationRuntimeSnapshot(uint8_t j,JointCalibrationRuntimeSnapshot* s) { *s=loaded[j];return true; }
struct SystemRuntimeContext { int32_t motor_abs_limit_min[3]={},motor_abs_limit_max[3]={}; uint32_t motor_abs_limit_valid_bitmap=0; };
struct JointActuatorItem { uint8_t motorChannel; };
uint8_t getJointActuatorCount(uint8_t j) { return j==0?2:1; }
bool getJointActuator(uint8_t j,uint8_t axis,JointActuatorItem* a) { a->motorChannel=j==0?axis:2;return true; }
int8_t getMotorTendonShortenAbsSign(uint8_t m) { return m==1?-1:1; }
int32_t motorApplyTendonDelta(uint8_t m,int32_t position,int32_t delta) { return position+getMotorTendonShortenAbsSign(m)*delta; }
uint32_t sharedAtomicFetchOr(uint32_t* p,uint32_t v) { return *p|=v; }
uint32_t sharedAtomicFetchAnd(uint32_t* p,uint32_t v) { return *p&=v; }
namespace mmhand::control {
struct Parameters { bool configured=true; float values[ParameterCount]={}; float operator[](size_t i) const { return values[i]; } } applied;
Parameters readParameters() { return applied; }
}
'''
    for signature in ('bool calibrationHasUsableTendonLut(', 'bool tendonCalibrationMotorAbsRange(',
                      'bool tendonCalibrationMotorAbsRangeWithMargins('):
        source += '\n' + function(lut, signature)
    source += '\n' + function(dispatcher, 'bool refreshLoadedJointAbsLimits(')
    source += '\n' + function(dispatcher, 'void refreshLoadedLutLimits(')
    source += r'''
int main() {
    using namespace mmhand::control;
    loaded[1].result.tendonLutMotor[0]=2;
    applied.values[OutputLutReleaseMarginCounts]=500;
    applied.values[OutputLutTightMarginCounts]=1000;
    SystemRuntimeContext r;
    refreshLoadedLutLimits(&r,7);
    assert(r.motor_abs_limit_valid_bitmap==7);
    assert(r.motor_abs_limit_min[0]==-400 && r.motor_abs_limit_max[0]==1200);
    assert(r.motor_abs_limit_min[1]==-700 && r.motor_abs_limit_max[1]==1000);
    assert(r.motor_abs_limit_min[2]==-400 && r.motor_abs_limit_max[2]==1200);
    // An old snapshot's 900-count tight bound cannot survive reconstruction.
    r.motor_abs_limit_max[2]=1100; refreshLoadedLutLimits(&r,4);
    assert(r.motor_abs_limit_max[2]==1200);
    applied.values[OutputLutReleaseMarginCounts]=350;
    applied.values[OutputLutTightMarginCounts]=750;
    refreshLoadedLutLimits(&r,7);
    assert(r.motor_abs_limit_min[0]==-250 && r.motor_abs_limit_max[0]==950);
    assert(r.motor_abs_limit_min[1]==-450 && r.motor_abs_limit_max[1]==850);
    // A corrupt motor mapping must not partially publish a dual-joint limit.
    auto old=r.motor_abs_limit_max[0]; loaded[0].result.tendonLutMotor[1]=26;
    assert(!refreshLoadedJointAbsLimits(&r,0,loaded[0].result));
    assert(r.motor_abs_limit_max[0]==old);
    refreshLoadedLutLimits(&r,3); assert((r.motor_abs_limit_valid_bitmap & 3)==0);
    applied.configured=false;
    assert(!refreshLoadedJointAbsLimits(&r,1,loaded[1].result));
}
'''
    run_native(tmp_path, 'lut_bounds', source)
