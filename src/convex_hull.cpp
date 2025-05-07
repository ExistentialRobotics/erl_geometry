#include "erl_geometry/convex_hull.hpp"

// #include <libqhullcpp/PointCoordinates.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertexSet.h>

namespace erl::geometry {
    void
    ConvexHull(
        const double *points,
        int num_points,
        std::vector<long> &hull_pt_map,
        const std::string &options) {
        ERL_ASSERTM(num_points > 0, "num_points = {}, it should be > 0.", num_points);

        // calculate convex hull
        orgQhull::Qhull qhull("", 3, num_points, points, options.c_str());
        ERL_DEBUG_ASSERT(qhull.initialized(), "qhull is not initialized.");

        if (qhull.vertexCount() == 0) {
            ERL_WARN("No points on the convex hull.");
            hull_pt_map.clear();
            return;
        }
        hull_pt_map.resize(qhull.vertexCount());
        std::transform(
            qhull.vertexList().begin(),
            qhull.vertexList().end(),
            hull_pt_map.begin(),
            [](const auto &vertex) { return vertex.point().id(); });
    }

    void
    ConvexHull(
        const double *points,
        int num_points,
        Eigen::Matrix3Xd &mesh_vertices,
        Eigen::Matrix3Xl &mesh_triangles,
        std::vector<long> &hull_pt_map,
        const std::string &options) {
        ERL_ASSERTM(num_points > 0, "num_points = {}, it should be > 0.", num_points);

        // convert Eigen::Matrix3Xd to orgQhull::PointCoordinates
        // orgQhull::PointCoordinates qhull_points(3, "");
        // auto point_data = reinterpret_cast<const double *>(points.data());
        // qhull_points.append(static_cast<int>(num_points * 3), point_data);

        // calculate convex hull
        orgQhull::Qhull qhull("", 3, num_points, points, options.c_str());
        ERL_DEBUG_ASSERT(qhull.initialized(), "qhull is not initialized.");

        // load result from qhull
        orgQhull::QhullFacetList facets = qhull.facetList();
        mesh_triangles.resize(3, static_cast<long>(facets.size()));
        mesh_vertices.resize(3, static_cast<long>(facets.size() * 3));
        std::unordered_map<long, long> indices_map;  // original_id -> new_id
        long triangle_index = 0;
        Eigen::Vector3d hull_center(0.0, 0.0, 0.0);
        hull_pt_map.clear();
        hull_pt_map.reserve(num_points);
        Eigen::Map<const Eigen::Matrix3Xd> points_mat(points, 3, num_points);
        for (orgQhull::QhullFacet &facet: facets) {
            if (!facet.isGood()) { continue; }  // skip degenerate facets
            orgQhull::QhullVertexSet vertices = facet.vertices();

            long triangle_subscript = 0;
            for (const orgQhull::QhullVertex &vertex: vertices) {
                // index of the point in the original point cloud
                long point_id = vertex.point().id();
                mesh_triangles(triangle_subscript, triangle_index) = point_id;
                if (auto new_id = static_cast<long>(indices_map.size());
                    indices_map.try_emplace(point_id, new_id).second) {
                    // the point is first seen
                    mesh_vertices.col(new_id) = points_mat.col(point_id);
                    hull_pt_map.push_back(point_id);          // store the index of the point
                    hull_center += points_mat.col(point_id);  // update the center
                }
                triangle_subscript++;
            }

            triangle_index++;
        }
        auto num_hull_points = static_cast<long>(indices_map.size());
        hull_center /= static_cast<double>(num_hull_points);
        hull_pt_map.shrink_to_fit();
        mesh_triangles.conservativeResize(3, triangle_index);
        mesh_vertices.conservativeResize(3, num_hull_points);

        // adjust triangle vertex order
        long num_triangles = mesh_triangles.cols();
        for (long i = 0; i < num_triangles; ++i) {
            auto triangle = mesh_triangles.col(i);
            triangle[0] = indices_map[triangle[0]];
            triangle[1] = indices_map[triangle[1]];
            triangle[2] = indices_map[triangle[2]];

            const auto p0 = mesh_vertices.col(triangle[0]);
            const auto p1 = mesh_vertices.col(triangle[1]);
            const auto p2 = mesh_vertices.col(triangle[2]);
            // make sure the normal points outward
            if ((p1 - p0).cross(p2 - p0).dot((p0 + p1 + p2) * (1.0 / 3.0) - hull_center) < 0.0) {
                std::swap(triangle[0], triangle[1]);
            }
        }
    }
}  // namespace erl::geometry
