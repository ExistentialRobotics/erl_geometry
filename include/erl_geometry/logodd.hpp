#pragma once

#include <cmath>

namespace erl::geometry::logodd {

    template<typename Dtype>
    inline Dtype
    LogOdd(const Dtype p) {
        return std::log(p / (1.0f - p));
    }

    template<typename Dtype>
    inline Dtype
    Probability(const Dtype logodd) {
        return 1.0f / (1.0f + std::exp(-logodd));
    }
}  // namespace erl::geometry::logodd
