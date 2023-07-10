#pragma once

#include <map>

#include "collision_checker_base.hpp"
#include "erl_common/assert.hpp"

namespace erl::geometry {

    class GridsCollisionCheckerSe2 : public CollisionCheckerBase {

        std::shared_ptr<common::GridMapUnsigned2D> m_grid_map_;
        Eigen::Matrix2Xd m_metric_shapes_;
        std::map<int, Eigen::Matrix2Xi> m_oriented_shapes_;

    public:
        GridsCollisionCheckerSe2(
            std::shared_ptr<common::GridMapUnsigned2D> grid_map,
            const std::shared_ptr<common::GridMapInfo3D> &grid_map_info,
            Eigen::Matrix2Xd metric_shape)
            : m_grid_map_(std::move(grid_map)),
              m_metric_shapes_(std::move(metric_shape)) {

            auto theta_c_min = int(std::round(grid_map_info->MeterToGridForValue(-M_PI, 2)));
            auto theta_c_max = int(std::round(grid_map_info->MeterToGridForValue(M_PI, 2)));

            for (int theta_c = theta_c_min; theta_c <= theta_c_max; ++theta_c) {
                double theta = grid_map_info->GridToMeterForValue(theta_c, 2);
                Eigen::Rotation2D<double> rotation(theta);
                Eigen::Matrix2Xd metric_coords = rotation.matrix() * m_metric_shapes_;
                m_oriented_shapes_[theta_c] = m_grid_map_->info->MeterToGridForPoints(metric_coords);
            }
        }

        [[nodiscard]] inline bool
        IsCollided(const Eigen::Ref<const Eigen::Matrix3d> &pose) const {
            auto num_cells = m_metric_shapes_.cols();
            for (int i = 0; i < num_cells; ++i) {
                Eigen::Vector2d metric_coords = pose.topLeftCorner<2, 2>() * m_metric_shapes_.col(i) + pose.topRightCorner<2, 1>();
                if (!m_grid_map_->info->InMap(metric_coords)) { return true; }
                Eigen::Vector2i grid_coords = m_grid_map_->info->MeterToGridForPoints(metric_coords);
                if (m_grid_map_->data[grid_coords] > 0) { return true; }
            }
            return false;
        }

        [[nodiscard]] inline bool
        IsCollided(const Eigen::Ref<const Eigen::VectorXi> &coords) const override {
            const Eigen::Matrix2Xi &kOrientedShapes = m_oriented_shapes_.at(coords[2]);
            auto num_cells = int(kOrientedShapes.cols());
            for (int i = 0; i < num_cells; ++i) {
                Eigen::Vector2i grid_coords = kOrientedShapes.col(i) + coords.head(2);
                if (!m_grid_map_->info->InGrids(grid_coords) || m_grid_map_->data[grid_coords] > 0) { return true; }
            }
            return false;
        }
    };

}  // namespace erl::geometry
