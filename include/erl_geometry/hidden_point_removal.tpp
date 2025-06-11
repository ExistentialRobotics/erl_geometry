#pragma once

// Reference: https://dl.acm.org/doi/10.1145/1276377.1276407
#include "hidden_point_removal.hpp"

#include "erl_common/logging.hpp"
#include "erl_geometry/convex_hull.hpp"

namespace erl::geometry {

    template<typename Dtype>
    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &points,
        const Eigen::Ref<const Eigen::Vector3<Dtype>> &view_position,
        const double radius,
        std::vector<long> &visible_point_indices,
        const bool fast,
        const bool joggle_inputs) {
        ERL_DEBUG_ASSERT(radius > 0.0, "radius ({}) should be positive.", radius);

        // perform spherical projection
        const long num_points = points.cols();
        ERL_DEBUG_ASSERT(num_points > 0, "num_points = {}, it should be > 0.", num_points);
        Eigen::Matrix3Xd projected_points(3, num_points + 1);  // qhullcpp uses double
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

    template<typename Dtype>
    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3X<Dtype>> &points,
        const Eigen::Ref<const Eigen::Vector3<Dtype>> &view_position,
        const double radius,
        Eigen::Matrix3Xl &mesh_triangles,
        Eigen::Matrix3Xd &mesh_vertices,
        std::vector<long> &visible_point_indices,
        const bool fast,
        const bool joggle_inputs) {
        ERL_DEBUG_ASSERT(radius > 0.0, "radius ({}) should be positive.", radius);

        // perform spherical projection
        const long num_points = points.cols();
        ERL_DEBUG_ASSERT(num_points > 0, "num_points = {}, it should be > 0.", num_points);
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
                mesh_vertices.col(mesh_vertices_cnt++) = points.col(idx);
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
            triangle_cnt++;
        }
    }

}  // namespace erl::geometry
