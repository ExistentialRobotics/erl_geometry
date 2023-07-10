#pragma once

#include "erl_common/assert.hpp"
#include "erl_common/eigen.hpp"

namespace erl::geometry {

    enum class EulerAngleOrder {
        // bit0-1, bit2-3, bit4-5: 00->x, 01->y, 10->z
        // bit6: 0->s, 1->r
        // SXYZ <--> RZYX
        kSxyz = 0b0000110,
        kSxyx = 0b0000100,
        kSxzy = 0b0001001,
        kSxzx = 0b0001000,

        kSyzx = 0b0011000,
        kSyzy = 0b0011001,
        kSyxz = 0b0010010,
        kSyxy = 0b0010001,

        kSzxy = 0b0100001,
        kSzxz = 0b0100010,
        kSzyx = 0b0100100,
        kSzyz = 0b0100110,

        kRxyz = 0b1000110,
        kRxyx = 0b1000100,
        kRxzy = 0b1001001,
        kRxzx = 0b1001000,

        kRyzx = 0b1011000,
        kRyzy = 0b1011001,
        kRyxz = 0b1010010,
        kRyxy = 0b1010001,

        kRzxy = 0b1100001,
        kRzxz = 0b1100010,
        kRzyx = 0b1100100,
        kRzyz = 0b1100110,
    };

    EulerAngleOrder
    GetEulerAngleOrder(const std::string& euler_order);

    Eigen::Matrix3d
    EulerToRotation3D(double a, double b, double c, EulerAngleOrder euler_angle_order);

}  // namespace erl::geometry
