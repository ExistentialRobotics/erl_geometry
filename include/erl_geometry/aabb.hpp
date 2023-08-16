#pragma once

#include <utility>
#include <vector>

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    template<typename ScalarType, int Dim>
    struct AABB : public Eigen::AlignedBox<ScalarType, Dim> {

        typedef ScalarType Scalar;
        typedef Eigen::Vector<Scalar, Dim> Point;

        Point center;
        Point half_sizes;

        AABB(Point center, Scalar half_size)
            : Eigen::AlignedBox<Scalar, Dim>(center.array() - half_size, center.array() + half_size),
              center(std::move(center)),
              half_sizes(Point::Constant(half_size)) {}

        AABB(const Point &min, const Point &max)
            : Eigen::AlignedBox<Scalar, Dim>(min, max),
              center((min + max) / 2),
              half_sizes((max - min) / 2) {}

    };

    typedef AABB<double, 2> Aabb2D;
    typedef AABB<double, 3> Aabb3D;
}  // namespace erl::geometry
