#pragma once

#include <cmath>

namespace erl::geometry::logodd {

    inline float
    LogOdd(const float p) {
        return std::log(p / (1.0f - p));
    }

    inline float
    Probability(const float logodd) {
        return 1.0f / (1.0f + std::exp(-logodd));
    }
}  // namespace erl::geometry::logodd
