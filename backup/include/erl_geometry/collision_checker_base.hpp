#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    class CollisionCheckerBase {

    public:
        virtual ~CollisionCheckerBase() = default;

        [[nodiscard]] virtual bool
        IsCollided(const Eigen::Ref<const Eigen::VectorXi> &grid_coords) const = 0;
    };

}  // namespace erl::geometry
