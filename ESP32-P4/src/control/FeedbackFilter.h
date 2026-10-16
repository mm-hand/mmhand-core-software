/** @file FeedbackFilter.h
 * @brief 声明关节反馈滑动窗口的状态与估计结果。
 */
#pragma once
#include <stdint.h>
#include "ControlParameters.h"

namespace mmhand::control {
struct FeedbackEstimate {
    float fast = 0;
    float slow = 0;
    bool ready = false;
    bool fresh = false;
    float sampleDt = 0;
};
class FeedbackFilter {
public:
    FeedbackEstimate update(uint32_t timestamp, float angle, bool valid, const Parameters& parameters);
    void reset();
private:
    struct Sample { uint32_t time; float angle; } samples[256] = {};
    uint16_t write = 0, count = 0;
    uint32_t first = 0, last = 0, revision = 0;
    FeedbackEstimate estimate;
};
}
