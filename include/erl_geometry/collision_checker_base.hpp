#pragma once

#include <memory>

#include "erl_common/grid_map.hpp"

namespace erl::geometry {

    class CollisionCheckerBase {

    public:
        virtual ~CollisionCheckerBase() = default;

        [[nodiscard]] virtual bool
        IsCollided(const Eigen::Ref<const Eigen::VectorXi> &grid_coords) const = 0;
    };

}  // namespace erl::geometry
