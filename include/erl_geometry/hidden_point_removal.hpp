#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * HiddenPointRemoval removes the hidden points from the point cloud.
     * @param points
     * @param camera_position
     * @param radius
     * @param visible_point_indices variable to store the indices of the visible points.
     * @return
     */
    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &camera_position,
        double radius,
        std::vector<long> &visible_point_indices,
        bool joggle_inputs = false
    );

    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &camera_position,
        double radius,
        Eigen::Matrix3Xl &mesh_triangles,
        Eigen::Matrix3Xd &mesh_vertices,
        std::vector<long> &visible_point_indices,
        bool joggle_inputs = false
    );
}  // namespace erl::geometry
