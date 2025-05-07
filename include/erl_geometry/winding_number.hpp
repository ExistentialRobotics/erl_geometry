#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {
    template<typename Dtype>
    int
    WindingNumber(
        const Eigen::Ref<const Eigen::Vector2<Dtype>> &p,
        const Eigen::Ref<const Eigen::Matrix2X<Dtype>> &vertices);
}

#include "winding_number.tpp"
