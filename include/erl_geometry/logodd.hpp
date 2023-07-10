#pragma once

#include <cmath>

namespace erl::geometry::logodd {

    inline float
    LogOdd(double p) {
        return (float) std::log(p / (1 - p));
    }

    inline double
    Probability(float logodd) {
        return 1.0 / (1.0 + std::exp(-logodd));
    }
}  // namespace erl::geometry::logodd
