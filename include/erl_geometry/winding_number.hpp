#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    int
    WindingNumber(const Eigen::Ref<const Eigen::Vector2d> &p, const Eigen::Ref<const Eigen::Matrix2Xd> &vertices);
}
