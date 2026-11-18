/**
 * Require measured metric-depth support before stereo initialization.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace observation {
inline bool has_stereo_initialization_support(const std::vector<float> &depths) {
    // Match the minimum geometric support required by the image-only pose seed.
    return std::count_if(depths.begin(), depths.end(), [](float depth) {
        return std::isfinite(depth) && depth > 0;
    }) >= 30;
}
} // namespace observation
