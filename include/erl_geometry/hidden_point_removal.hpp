#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * HiddenPointRemoval removes the hidden points from the point cloud.
     * @param points
     * @param view_position
     * @param radius
     * @param visible_point_indices variable to store the indices of the visible points.
     * @param fast if true, will run QHull with `Q3 Q5 Q8`.
     * @param joggle_inputs if true, will run QHull with `QJ`.
     * @return
     */
    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &view_position,
        double radius,
        std::vector<long> &visible_point_indices,
        bool fast = false,
        bool joggle_inputs = false);

    /**
     * HiddenPointRemoval removes the hidden points from the point cloud and returns the mesh of the visible points.
     * @param points
     * @param view_position
     * @param radius
     * @param mesh_triangles variable to store the triangles of the mesh, each column is triangle vertex indices.
     * @param mesh_vertices variable to store the vertices of the mesh.
     * @param visible_point_indices variable to store the indices of the visible points.
     * @param fast if true, will run QHull with `Q3 Q5 Q8`.
     * @param joggle_inputs if true, will run QHull with `QJ`.
     * @return
     */
    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &view_position,
        double radius,
        Eigen::Matrix3Xl &mesh_triangles,
        Eigen::Matrix3Xd &mesh_vertices,
        std::vector<long> &visible_point_indices,
        bool fast = false,
        bool joggle_inputs = false);

    inline void
    HiddenPointRemoval(
        const std::vector<Eigen::Vector3d> &points,
        const Eigen::Ref<const Eigen::Vector3d> &view_position,
        const double radius,
        std::vector<long> &visible_point_indices,
        const bool fast,
        const bool joggle_inputs) {
        const Eigen::Map<const Eigen::Matrix3Xd> points_mat(points[0].data(), 3, static_cast<long>(points.size()));
        HiddenPointRemoval(points_mat, view_position, radius, visible_point_indices, fast, joggle_inputs);
    }

    inline void
    ParallelHiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Matrix3Xd> &view_positions,
        const double radius,
        std::vector<std::vector<long>> &visible_point_indices,
        const bool fast = false,
        const bool joggle_inputs = false) {

        const long num_view_positions = view_positions.cols();
        visible_point_indices.resize(num_view_positions);
#pragma omp parallel for default(none) shared(num_view_positions, points, view_positions, radius, visible_point_indices, fast, joggle_inputs)
        for (long i = 0; i < num_view_positions; ++i) {
            HiddenPointRemoval(points, view_positions.col(i), radius, visible_point_indices[i], fast, joggle_inputs);
        }
    }

    inline void
    ParallelHiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Matrix3Xd> &view_positions,
        const double radius,
        std::vector<Eigen::Matrix3Xl> &mesh_triangles,
        std::vector<Eigen::Matrix3Xd> &mesh_vertices,
        std::vector<std::vector<long>> &visible_point_indices,
        const bool fast = false,
        const bool joggle_inputs = false) {

        const long num_view_positions = view_positions.cols();
        mesh_triangles.resize(num_view_positions);
        mesh_vertices.resize(num_view_positions);
        visible_point_indices.resize(num_view_positions);
#pragma omp parallel for default(none) \
    shared(num_view_positions, points, view_positions, radius, mesh_triangles, mesh_vertices, visible_point_indices, fast, joggle_inputs)
        for (long i = 0; i < num_view_positions; ++i) {
            HiddenPointRemoval(points, view_positions.col(i), radius, mesh_triangles[i], mesh_vertices[i], visible_point_indices[i], fast, joggle_inputs);
        }
    }
}  // namespace erl::geometry
