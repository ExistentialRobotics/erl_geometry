#pragma once

#include "collision_checker_base.hpp"

#include <map>

namespace erl::geometry {

    class GridCollisionCheckerSe2 : public CollisionCheckerBase {

        std::shared_ptr<common::GridMapUnsigned2D> m_grid_map_ = nullptr;
        Eigen::Matrix2Xd m_metric_shapes_;
        std::map<int, Eigen::Matrix2Xi> m_oriented_shapes_;

    public:
        GridCollisionCheckerSe2(
            std::shared_ptr<common::GridMapUnsigned2D> grid_map,
            const std::shared_ptr<common::GridMapInfo3D> &grid_map_info,
            Eigen::Matrix2Xd metric_shape)
            : m_grid_map_(std::move(grid_map)),
              m_metric_shapes_(std::move(metric_shape)) {

            const auto theta_c_min = static_cast<int>(std::round(grid_map_info->MeterToGridForValue(-M_PI, 2)));
            const auto theta_c_max = static_cast<int>(std::round(grid_map_info->MeterToGridForValue(M_PI, 2)));

            for (int theta_c = theta_c_min; theta_c <= theta_c_max; ++theta_c) {
                double theta = grid_map_info->GridToMeterForValue(theta_c, 2);
                Eigen::Rotation2D<double> rotation(theta);
                Eigen::Matrix2Xd metric_coords = rotation.matrix() * m_metric_shapes_;
                m_oriented_shapes_[theta_c] = m_grid_map_->info->MeterToGridForPoints(metric_coords);
            }
        }

        [[nodiscard]] bool
        IsCollided(const Eigen::Ref<const Eigen::Matrix3d> &pose) const {
            const long num_cells = m_metric_shapes_.cols();
            for (long i = 0; i < num_cells; ++i) {
                Eigen::Vector2d metric_coords = pose.topLeftCorner<2, 2>() * m_metric_shapes_.col(i) + pose.topRightCorner<2, 1>();
                if (!m_grid_map_->info->InMap(metric_coords)) { return true; }
                Eigen::Vector2i grid_coords = m_grid_map_->info->MeterToGridForPoints(metric_coords);
                if (m_grid_map_->data[grid_coords] > 0) { return true; }
            }
            return false;
        }

        [[nodiscard]] bool
        IsCollided(const Eigen::Ref<const Eigen::VectorXi> &coords) const override {
            const Eigen::Matrix2Xi &oriented_shapes = m_oriented_shapes_.at(coords[2]);
            const auto num_cells = static_cast<int>(oriented_shapes.cols());
            for (int i = 0; i < num_cells; ++i) {
                Eigen::Vector2i grid_coords = oriented_shapes.col(i) + coords.head(2);
                if (!m_grid_map_->info->InGrids(grid_coords) || m_grid_map_->data[grid_coords] > 0) { return true; }
            }
            return false;
        }
    };

}  // namespace erl::geometry
