/**
 * @file ServoAbsoluteRestore.h
 * @brief JSON恢复、热插拔和显式ABS归零共用的单次坐标写入与原生反馈确认。
 */
#ifndef MMHAND_SERVO_ABSOLUTE_RESTORE_H
#define MMHAND_SERVO_ABSOLUTE_RESTORE_H

#include "../../hal/servo/FeetechServoDriver.h"

namespace p4::servo {
enum class RestoreResult : uint8_t { Pending, Success, Failed };

class AbsoluteRestore {
public:
    RestoreResult step(FeetechServoDriver* driver, uint8_t id,
                       int32_t target, uint32_t nowMs)
    {
        if (!active_) {
            if (target < kServoHardwareCommandMin || target > kServoHardwareCommandMax ||
                !driver || !driver->isOnline(id) ||
                !driver->isMultiTurnConfigured(id)) return RestoreResult::Failed;
            driver_ = driver;
            id_ = id;
            sequence_ = driver->getFeedback(id).feedbackSequence;
            session_ = driver->getFeedback(id).onlineSessionGeneration;
            deadline_ = nowMs + 500;
            if (!driver->beginAbsoluteRestore(id, target)) return RestoreResult::Failed;
            active_ = true;
            return RestoreResult::Pending;
        }
        if (driver != driver_ || id != id_ || !driver || !driver->isOnline(id) ||
            driver->getFeedback(id).onlineSessionGeneration != session_) {
            active_ = false;
            return RestoreResult::Failed;
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
    bool active() const { return active_; }
    void cancel() { active_ = false; }
private:
    FeetechServoDriver* driver_ = nullptr;
    uint32_t sequence_ = 0;
    uint32_t session_ = 0;
    uint32_t deadline_ = 0;
    uint8_t id_ = 0;
    bool active_ = false;
};
} // namespace p4::servo
#endif
