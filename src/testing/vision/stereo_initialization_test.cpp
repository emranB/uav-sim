/**
 * Check stereo initialization against valid, missing and nonfinite depth samples.
 */
#include "stereo_initialization.h"
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    std::vector<float> depths(4000, -1);
    if (observation::has_stereo_initialization_support(depths)) {
        throw std::runtime_error("Left-image features without depth initialized a metric map");
    }
    std::fill_n(depths.begin(), 29, 4.f);
    depths[29] = std::numeric_limits<float>::infinity();
    depths[30] = std::numeric_limits<float>::quiet_NaN();
    if (observation::has_stereo_initialization_support(depths)) {
        throw std::runtime_error("Invalid depths counted as geometric initialization support");
    }
    depths[29] = 4.f;
    if (!observation::has_stereo_initialization_support(depths)) {
        throw std::runtime_error("Supported metric stereo initialization was rejected");
    }
    std::cout << "Stereo initialization depth-support boundaries passed\n";
}
