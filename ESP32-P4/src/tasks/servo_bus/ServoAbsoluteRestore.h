/**
 * @file ServoAbsoluteRestore.h
 * @brief 仅用于完整ABS归零：0x0B校准后0x0A清圈数，新反馈确认。
 */
#ifndef MMHAND_SERVO_ABSOLUTE_RESTORE_H
#define MMHAND_SERVO_ABSOLUTE_RESTORE_H

#include "../../hal/servo/FeetechServoDriver.h"

namespace p4::servo {
enum class RestoreResult : uint8_t { Pending, Success, Failed };

class AbsoluteRestore {
public:
    RestoreResult step(FeetechServoDriver* driver, uint8_t id,
                       uint32_t nowMs)
    {
        if (!active_) {
            if (!driver || !driver->isOnline(id) ||
                !driver->isMultiTurnConfigured(id)) return RestoreResult::Failed;
            driver_ = driver;
            id_ = id;
            sequence_ = driver->getFeedback(id).feedbackSequence;
            session_ = driver->getFeedback(id).onlineSessionGeneration;
            deadline_ = nowMs + 750;
            calibratedAt_ = nowMs;
            turnsCleared_ = false;
            if (!driver->beginAbsoluteRestore(id)) return RestoreResult::Failed;
            active_ = true;
            return RestoreResult::Pending;
        }
        if (driver != driver_ || id != id_ || !driver || !driver->isOnline(id) ||
            driver->getFeedback(id).onlineSessionGeneration != session_) {
            active_ = false;
            return RestoreResult::Failed;
        }
        if (!turnsCleared_) {
            if (int32_t(nowMs - calibratedAt_) < 250 ||
                driver->getFeedback(id).feedbackSequence == sequence_) {
                if (int32_t(nowMs - deadline_) >= 0) { active_ = false; return RestoreResult::Failed; }
                return RestoreResult::Pending;
            }
            if (!driver->clearAbsoluteTurns(id)) { active_ = false; return RestoreResult::Failed; }
            sequence_ = driver->getFeedback(id).feedbackSequence;
            turnsCleared_ = true;
            deadline_ = nowMs + 500;
            return RestoreResult::Pending;
        }
        if (driver->confirmAbsoluteRestore(id, sequence_)) {
            active_ = false;
            return RestoreResult::Success;
        }
        if (int32_t(nowMs - deadline_) >= 0) {
            active_ = false;
            return RestoreResult::Failed;
        }
        return RestoreResult::Pending;
    }
    const char* phase() const { return turnsCleared_ ? "verify_zero" : "calibrate"; }
    bool active() const { return active_; }
    void cancel() { active_ = false; }
private:
    FeetechServoDriver* driver_ = nullptr;
    uint32_t calibratedAt_ = 0;
    bool turnsCleared_ = false;
    uint32_t sequence_ = 0;
    uint32_t session_ = 0;
    uint32_t deadline_ = 0;
    uint8_t id_ = 0;
    bool active_ = false;
};
} // namespace p4::servo
#endif
