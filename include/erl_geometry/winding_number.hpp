#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {
    template<typename Dtype>
    int
    WindingNumber(
        const Eigen::Ref<const Eigen::Vector2<Dtype>> &p,
        const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &vertices);

    extern template int
    WindingNumber<double>(
        const Eigen::Ref<const Eigen::Vector2<double>> &p,
        const Eigen::Ref<const Eigen::Matrix2X<double>> &vertices);

    extern template int
    WindingNumber<float>(
        const Eigen::Ref<const Eigen::Vector2<float>> &p,
        const Eigen::Ref<const Eigen::Matrix2X<float>> &vertices);
}  // namespace erl::geometry
