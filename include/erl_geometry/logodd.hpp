#pragma once

#include <cmath>

namespace erl::geometry::logodd {

    inline float
    LogOdd(const float p) {
        return std::log(p / (1 - p));
    }

    inline float
    Probability(const float logodd) {
        return 1.0 / (1.0 + std::exp(-logodd));
    }
}  // namespace erl::geometry::logodd
