#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * Get the indices of the points on the convex hull.
     * @param points
     * @param hull_pt_map variable to store the indices of the points on the convex hull.
     * @param joggle_inputs If True allows the algorithm to add random noise to the points to work around degenerate inputs.
     * @return
     */
    void
    ConvexHull(const Eigen::Ref<const Eigen::Matrix3Xd> &points, std::vector<long> &hull_pt_map, bool joggle_inputs = false);

    /**
     * Get the mesh of the convex hull.
     * @param points
     * @param mesh_triangles variable to store the triangles of the mesh, each column is triangle vertex indices.
     * @param mesh_vertices variable to store the vertices of the mesh.
     * @param hull_pt_map variable to store the indices of the points on the convex hull.
     * @param joggle_inputs
     */
    void
    ConvexHull(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        Eigen::Matrix3Xl &mesh_triangles,
        Eigen::Matrix3Xd &mesh_vertices,
        std::vector<long> &hull_pt_map,
        bool joggle_inputs = false
    );
}  // namespace erl::geometry
