/**
 * @file ServoAbsoluteRestore.h
 * @brief 用0x0B建立原生ABS基准，并用新反馈确认。
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
            reason_ = "precheck";
            startedAt_ = nowMs;
            sequence_ = 0;
            if (!driver || !driver->isOnline(id) ||
                !driver->isMultiTurnConfigured(id)) return RestoreResult::Failed;
            driver_ = driver;
            id_ = id;
            sequence_ = driver->getFeedback(id).feedbackSequence;
            session_ = driver->getFeedback(id).onlineSessionGeneration;
            deadline_ = nowMs + 750;
            calibratedAt_ = nowMs;
            reason_ = "correction_rejected";
            if (!driver->beginAbsoluteRestore(id)) return RestoreResult::Failed;
            reason_ = "waiting_correction_feedback";
            active_ = true;
            return RestoreResult::Pending;
        }
        if (driver != driver_ || id != id_ || !driver || !driver->isOnline(id) ||
            driver->getFeedback(id).onlineSessionGeneration != session_) {
            active_ = false;
            reason_ = "session_changed";
            return RestoreResult::Failed;
        }
        if (int32_t(nowMs - calibratedAt_) >= 250 &&
            driver->confirmAbsoluteRestore(id, sequence_)) {
            active_ = false;
            reason_ = "verified";
            return RestoreResult::Success;
        }
        if (int32_t(nowMs - deadline_) >= 0) {
            active_ = false;
            reason_ = driver->getFeedback(id).feedbackSequence == sequence_
                ? "correction_feedback_timeout" : "position_outside_tolerance";
            return RestoreResult::Failed;
        }
        return RestoreResult::Pending;
    }
    const char* phase() const { return "verify_position"; }
    const char* reason() const { return reason_; }
    uint32_t startedAt() const { return startedAt_; }
    uint32_t baselineSequence() const { return sequence_; }
    bool active() const { return active_; }
    void cancel() { active_ = false; }
private:
    FeetechServoDriver* driver_ = nullptr;
    uint32_t calibratedAt_ = 0;
    uint32_t startedAt_ = 0;
    const char* reason_ = "not_started";
    uint32_t sequence_ = 0;
    uint32_t session_ = 0;
    uint32_t deadline_ = 0;
    uint8_t id_ = 0;
    bool active_ = false;
};
} // namespace p4::servo
#endif
