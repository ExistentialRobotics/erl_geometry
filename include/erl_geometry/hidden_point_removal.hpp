#pragma once

#include "convex_hull.hpp"

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
    template<typename Dtype, int Dim>
    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix<Dtype, Dim, Eigen::Dynamic>> &points,
        const Eigen::Vector<Dtype, Dim> &view_position,
        Dtype radius,
        std::vector<long> &visible_point_indices,
        bool fast = false,
        bool joggle_inputs = false) {

        ERL_DEBUG_ASSERT_GT(radius, 0.0);

        // perform spherical projection
        const long num_points = points.cols();
        ERL_DEBUG_ASSERT_GT(num_points, 0);

        // qhullcpp uses double
        Eigen::Matrix<double, Dim, Eigen::Dynamic> projected_points(Dim, num_points + 1);
        Eigen::VectorXd norms(num_points);

        // spherical projection
        for (long i = 0; i < num_points; ++i) {
            auto point = points.col(i);
            auto projected_point = projected_points.col(i);
            double &norm = norms[i];

            projected_point << (point - view_position).template cast<double>();
            norm = projected_point.norm();
            ERL_DEBUG_ASSERT_LT(norm, radius);
            projected_point << projected_point + (2 * (radius - norm) / norm) * projected_point;
        }

        // add origin, which may be outside the point cloud
        projected_points.col(num_points).setZero();
        const long origin_index = num_points;

        // calculate convex hull of the projected points
        std::string qhull_options;
        if (fast) {
            qhull_options = "Q3 Q5 Q8";
            if (joggle_inputs) { qhull_options += " QJ"; }
        } else {
            if (joggle_inputs) {
                qhull_options = "QJ";
            } else {
                qhull_options = "Qt";
            }
        }
        ConvexHull(
            projected_points.data(),
            Dim,
            static_cast<int>(projected_points.cols()),
            visible_point_indices,
            qhull_options);

        // remove the index of the origin
        for (std::size_t i = 0; i < visible_point_indices.size(); ++i) {
            if (visible_point_indices[i] == origin_index) {
                visible_point_indices.erase(visible_point_indices.begin() + static_cast<long>(i));
                break;
            }
        }
    }

    /**
     * HiddenPointRemoval removes the hidden points from the point cloud and returns the mesh of the
     * visible points.
     * @param points
     * @param view_position
     * @param radius
     * @param mesh_triangles variable to store the triangles of the mesh, each column is triangle
     * vertex indices.
     * @param mesh_vertices variable to store the vertices of the mesh.
     * @param visible_point_indices variable to store the indices of the visible points.
     * @param fast if true, will run QHull with `Q3 Q5 Q8`.
     * @param joggle_inputs if true, will run QHull with `QJ`.
     * @return
     */
    template<typename Dtype>
    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &points,
        const Eigen::Vector3<Dtype> &view_position,
        Dtype radius,
        Eigen::Matrix3Xl &mesh_triangles,
        Eigen::Matrix3Xd &mesh_vertices,
        std::vector<long> &visible_point_indices,
        bool fast = false,
        bool joggle_inputs = false) {
        ERL_DEBUG_ASSERT_GT(radius, 0.0);

        // perform spherical projection
        const long num_points = points.cols();
        ERL_DEBUG_ASSERT_GT(num_points, 0);
        Eigen::Matrix3Xd projected_points(3, num_points + 1);
        Eigen::VectorXd norms(num_points);

        // spherical projection
        for (long i = 0; i < num_points; ++i) {
            auto point = points.col(i);
            auto projected_point = projected_points.col(i);
            double &norm = norms[i];

            projected_point << (point - view_position).template cast<double>();
            norm = projected_point.norm();
            ERL_DEBUG_ASSERT(norm < radius, "norm ({}) should be < radius ({}).", norm, radius);
            projected_point << projected_point + (2 * (radius - norm) / norm) * projected_point;
        }

        // add origin, which may be outside the point cloud
        projected_points.col(num_points).setZero();
        long origin_index = num_points;

        // calculate convex hull of the projected points
        std::string qhull_options;
        if (fast) {
            qhull_options = "Q3 Q5 Q8";
            if (joggle_inputs) { qhull_options += " QJ"; }
        } else {
            if (joggle_inputs) {
                qhull_options = "QJ";
            } else {
                qhull_options = "Qt";
            }
        }
        ConvexHull(
            projected_points.data(),
            projected_points.cols(),
            mesh_vertices,
            mesh_triangles,
            visible_point_indices,
            qhull_options);

        // restore original points
        bool origin_is_visible = false;
        long mesh_vertices_cnt = 0;
        for (std::size_t i = 0; i < visible_point_indices.size(); ++i) {
            if (const long idx = visible_point_indices[i]; idx == origin_index) {
                origin_index = static_cast<long>(i);
                origin_is_visible = true;
            } else {
                mesh_vertices.col(mesh_vertices_cnt++) = points.col(idx).template cast<double>();
            }
        }

        // erase origin
        if (!origin_is_visible) { return; }
        visible_point_indices.erase(visible_point_indices.begin() + origin_index);
        long triangle_cnt = 0;
        for (long i = 0; i < mesh_triangles.cols(); ++i) {
            if (mesh_triangles(0, i) == origin_index || mesh_triangles(1, i) == origin_index ||
                mesh_triangles(2, i) == origin_index) {
                continue;
            }
            auto triangle = mesh_triangles.col(triangle_cnt);
            if (const long idx0 = mesh_triangles(0, i); idx0 > origin_index) {
                triangle[0] = idx0 - 1;
            }
            if (const long idx1 = mesh_triangles(1, i); idx1 > origin_index) {
                triangle[1] = idx1 - 1;
            }
            if (const long idx2 = mesh_triangles(2, i); idx2 > origin_index) {
                triangle[2] = idx2 - 1;
            }
            ++triangle_cnt;
        }
    }

    template<typename Dtype>
    void
    HiddenPointRemoval(
        const std::vector<Eigen::Vector3<Dtype>> &points,
        const Eigen::Vector3<Dtype> &view_position,
        const Dtype radius,
        std::vector<long> &visible_point_indices,
        const bool fast,
        const bool joggle_inputs) {
        using Points = Eigen::Matrix3X<Dtype>;
        Eigen::Map<const Points> points_mat(points[0].data(), 3, static_cast<long>(points.size()));
        HiddenPointRemoval<Dtype, 3>(
            points_mat,
            view_position,
            radius,
            visible_point_indices,
            fast,
            joggle_inputs);
    }

    template<typename Dtype>
    void
    ParallelHiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &points,
        const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &view_positions,
        const Eigen::VectorXd &radii,
        std::vector<std::vector<long>> &visible_point_indices,
        const bool fast = false,
        const bool joggle_inputs = false) {

        const long num_view_positions = view_positions.cols();
        visible_point_indices.resize(num_view_positions);
#pragma omp parallel for default(none) \
    shared(num_view_positions,         \
               points,                 \
               view_positions,         \
               radii,                  \
               visible_point_indices,  \
               fast,                   \
               joggle_inputs)
        for (long i = 0; i < num_view_positions; ++i) {
            HiddenPointRemoval<Dtype, 3>(
                points,
                view_positions.col(i),
                radii[i],
                visible_point_indices[i],
                fast,
                joggle_inputs);
        }
    }

    template<typename Dtype>
    void
    ParallelHiddenPointRemoval(
        const Eigen::Matrix3X<Dtype> &points,
        const Eigen::Matrix3X<Dtype> &view_positions,
        const Eigen::VectorXd &radii,
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
    shared(num_view_positions,         \
               points,                 \
               view_positions,         \
               radii,                  \
               mesh_triangles,         \
               mesh_vertices,          \
               visible_point_indices,  \
               fast,                   \
               joggle_inputs)
        for (long i = 0; i < num_view_positions; ++i) {
            HiddenPointRemoval<double>(
                points,
                view_positions.col(i),
                radii[i],
                mesh_triangles[i],
                mesh_vertices[i],
                visible_point_indices[i],
                fast,
                joggle_inputs);
        }
    }
}  // namespace erl::geometry
