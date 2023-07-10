#pragma once

#include "erl_common/eigen.hpp"
#include <vector>

namespace erl::geometry {

    int
    WindingNumber(const Eigen::Ref<const Eigen::Vector2d> &p, const Eigen::Ref<const Eigen::Matrix2Xd> &vertices);
}
