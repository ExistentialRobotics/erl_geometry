#pragma once

#include "erl_common/eigen.hpp"
#include "erl_common/logging.hpp"

namespace erl::geometry {

    /**
     * Get the indices of the points on the convex hull.
     * @tparam Container A container class that uses contiguous per-point ordering storage and
     * provides .data() methods.
     * @param points
     * @param num_points
     * @param hull_pt_map variable to store the indices of the points on the convex hull.
     * @param options Options for qhull, e.g. "QJ" to joggle inputs. "QgGn" to generate a partial
     * convex hull that is visible to the nth point. See http://www.qhull.org/html/qh-optq.htm for
     * more details.
     * @return
     */
    void
    ConvexHull(
        const double *points,
        int num_points,
        std::vector<long> &hull_pt_map,
        const std::string &options = "Qt");

    /**
     * Get the mesh of the convex hull.
     * @tparam Container A container class that uses contiguous per-point ordering storage and
     * provides .data() methods.
     * @param points
     * @param num_points
     * @param mesh_triangles variable to store the triangles of the mesh, each column is triangle
     * vertex indices.
     * @param mesh_vertices variable to store the vertices of the mesh.
     * @param hull_pt_map variable to store the indices of the points on the convex hull.
     * @param options Options for qhull, e.g. "QJ" to joggle inputs. "QgGn" to generate a partial
     * convex hull that is visible to the nth point. See http://www.qhull.org/html/qh-optq.htm for
     * more details.
     */
    void
    ConvexHull(
        const double *points,
        int num_points,
        Eigen::Matrix3Xd &mesh_vertices,
        Eigen::Matrix3Xl &mesh_triangles,
        std::vector<long> &hull_pt_map,
        const std::string &options = "Qt");
}  // namespace erl::geometry
