// Reference: https://dl.acm.org/doi/10.1145/1276377.1276407

#include "erl_geometry/hidden_point_removal.hpp"
#include "erl_geometry/convex_hull.hpp"
#include "erl_common/assert.hpp"

namespace erl::geometry {

    static inline void
    SphericalProjection(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &camera_position,
        double radius,
        Eigen::Matrix3Xd &projected_points,
        Eigen::VectorXd &norms) {
        long num_points = points.cols();
        for (long i = 0; i < num_points; ++i) {
            auto point = points.col(i);
            auto projected_point = projected_points.col(i);
            double &norm = norms[i];

            projected_point << point - camera_position;
            norm = projected_point.norm();
            ERL_ASSERTM(norm < radius, "norm (%f) should be < radius (%f).", norm, radius);
            projected_point << projected_point + 2 * (radius - norm) * (projected_point / norm);
        }
    }

    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &camera_position,
        double radius,
        std::vector<long> &visible_point_indices,
        bool joggle_inputs) {
        ERL_ASSERTM(radius > 0.0, "radius (%f) should be positive.", radius);

        // perform spherical projection
        long num_points = points.cols();
        ERL_ASSERTM(num_points > 0, "num_points = %ld, it should be > 0.", num_points);
        Eigen::Matrix3Xd projected_points(3, num_points + 1);
        Eigen::VectorXd norms(num_points);
        SphericalProjection(points, camera_position, radius, projected_points, norms);

        // add origin, which may be outside the point cloud
        projected_points.col(num_points).setZero();
        long origin_index = num_points;

        // calculate convex hull of the projected points
        ConvexHull(projected_points, visible_point_indices, joggle_inputs);

        // remove the index of the origin
        for (std::size_t i = 0; i < visible_point_indices.size(); ++i) {
            if (visible_point_indices[i] == origin_index) {
                visible_point_indices.erase(visible_point_indices.begin() + long(i));
                break;
            }
        }
    }

    void
    HiddenPointRemoval(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        const Eigen::Ref<const Eigen::Vector3d> &camera_position,
        double radius,
        Eigen::Matrix3Xl &mesh_triangles,
        Eigen::Matrix3Xd &mesh_vertices,
        std::vector<long> &visible_point_indices,
        bool joggle_inputs) {
        ERL_ASSERTM(radius > 0.0, "radius (%f) should be positive.", radius);

        // perform spherical projection
        long num_points = points.cols();
        ERL_ASSERTM(num_points > 0, "num_points = %ld, it should be > 0.", num_points);
        Eigen::Matrix3Xd projected_points(3, num_points + 1);
        Eigen::VectorXd norms(num_points);
        SphericalProjection(points, camera_position, radius, projected_points, norms);

        // add origin, which may be outside the point cloud
        projected_points.col(num_points).setZero();
        long origin_index = num_points;

        // calculate convex hull of the projected points
        ConvexHull(projected_points, mesh_triangles, mesh_vertices, visible_point_indices, joggle_inputs);

        // restore original points
        bool origin_is_visible = false;
        long mesh_vertices_cnt = 0;
        for (std::size_t i = 0; i < visible_point_indices.size(); ++i) {
            long idx = visible_point_indices[i];
            if (idx == origin_index) {
                origin_index = long(i);
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
            if (mesh_triangles(0, i) == origin_index || mesh_triangles(1, i) == origin_index || mesh_triangles(2, i) == origin_index) { continue; }
            long &idx0 = mesh_triangles(0, i);
            auto triangle = mesh_triangles.col(triangle_cnt);
            if (idx0 > origin_index) { triangle[0] = idx0 - 1; }
            long &idx1 = mesh_triangles(1, i);
            if (idx1 > origin_index) { triangle[1] = idx1 - 1; }
            long &idx2 = mesh_triangles(2, i);
            if (idx2 > origin_index) { triangle[2] = idx2 - 1; }
            triangle_cnt++;
        }
    }

}  // namespace erl::geometry
