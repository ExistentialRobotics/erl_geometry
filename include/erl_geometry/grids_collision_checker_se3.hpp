#pragma once

#include "collision_checker_base.hpp"

namespace erl::geometry {

    class GridCollisionChecker3D : public CollisionCheckerBase {

        std::shared_ptr<common::GridMapUnsigned3D> m_grid_map_;
        Eigen::Matrix3Xd m_metric_voxels_;

    public:
        GridCollisionChecker3D(std::shared_ptr<common::GridMapUnsigned3D> grid_map, Eigen::Matrix3Xd metric_voxels)
            : m_grid_map_(std::move(grid_map)),
              m_metric_voxels_(std::move(metric_voxels)) {}

        [[nodiscard]] inline bool
        IsCollided(const Eigen::Ref<const Eigen::Matrix4d> &pose) const {
            Eigen::Matrix3Xd metric_coords = (pose.topLeftCorner<3, 3>() * m_metric_voxels_).colwise() + pose.topRightCorner<3, 1>();
            auto num_cells = int(metric_coords.cols());
            for (int i = 0; i < num_cells; ++i) {
                if (!m_grid_map_->info->InMap(metric_coords.col(i))) { return false; }
                Eigen::VectorXi grid_coords = m_grid_map_->info->MeterToGridForPoints(metric_coords.col(i)).array().round().cast<int>();
                if (m_grid_map_->data[grid_coords] > 0) { return false; }
            }
            return true;
        }

        [[nodiscard]] inline bool
        IsCollided(const Eigen::Ref<const Eigen::VectorXi> &coords) const override {
            return m_grid_map_->data[Eigen::Vector3i(coords.head(3))] > 0;
        }
    };

}  // namespace erl::geometry
