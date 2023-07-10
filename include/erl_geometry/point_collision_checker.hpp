#pragma once

#include "collision_checker_base.hpp"

namespace erl::geometry {

    template<int Dim>
    class PointCollisionChecker : public CollisionCheckerBase {

        std::shared_ptr<common::GridMap<uint8_t, Dim>> m_grid_map_;

    public:
        explicit PointCollisionChecker(std::shared_ptr<common::GridMap<uint8_t, Dim>> grid_map)
            : m_grid_map_(std::move(grid_map)) {}

        [[nodiscard]] inline bool
        IsCollided(const Eigen::Ref<const Eigen::VectorXi> &coords) const override {
            return m_grid_map_->data[coords.head<Dim>()] > 0;  // the first d elements are grid coords while the remains could be orientation coords
        }
    };

    using PointCollisionChecker2D = PointCollisionChecker<2>;
    using PointCollisionChecker3D = PointCollisionChecker<3>;

}  // namespace erl::geometry
