/** @file FeedbackFilter.cpp
 * @brief 按有效磁编时间戳计算快慢滑动窗口与速度估计。
 */
#include "FeedbackFilter.h"
#include <math.h>

namespace mmhand::control {
void FeedbackFilter::reset() {
    write = count = 0;
    first = last = 0;
    estimate = {};
}
FeedbackEstimate FeedbackFilter::update(uint32_t timestamp, float angle, bool valid, const Parameters& p) {
    estimate.fresh = false;
    if (!valid || !timestamp || !isfinite(angle) || !p.configured) { reset(); return estimate; }
    if (revision != p.revision) { reset(); revision = p.revision; }
    if (count && timestamp == last) return estimate;
    if (count && (int32_t(timestamp - last) <= 0 || timestamp - last > 600)) reset();
    if (!count) first = timestamp;
    estimate.sampleDt = last && timestamp - last <= 100 ? (timestamp - last) * 0.001f : 0;
    last = timestamp;
    samples[write] = {timestamp, angle};
    write = (write + 1) % 256;
    if (count < 256) ++count;
    float fastSum = 0, slowSum = 0;
    unsigned fastCount = 0, slowCount = 0;
    for (uint16_t offset = 1; offset <= count; ++offset) {
        const auto& sample = samples[(write + 256 - offset) % 256];
        const auto age = timestamp - sample.time;
        if (age < p[FeedbackFastWindowMs]) { fastSum += sample.angle; ++fastCount; }
        if (age < p[FeedbackSlowWindowMs]) { slowSum += sample.angle; ++slowCount; }
    }
    estimate.fast = fastCount ? fastSum / fastCount : angle;
    estimate.slow = slowCount ? slowSum / slowCount : angle;
    estimate.ready = fastCount && slowCount && timestamp - first >= p[FeedbackSlowWindowMs];
    estimate.fresh = true;
    return estimate;
}
}
