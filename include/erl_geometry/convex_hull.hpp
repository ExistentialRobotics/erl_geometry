#pragma once

#include "erl_common/eigen.hpp"
#include "erl_common/logging.hpp"

#include <libqhullcpp/PointCoordinates.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullVertexSet.h>

namespace erl::geometry {

    /**
     * Get the indices of the points on the convex hull.
     * @tparam Container A container class that uses contiguous per-point ordering storage and provides .data() methods.
     * @param points
     * @param num_points
     * @param hull_pt_map variable to store the indices of the points on the convex hull.
     * @param options options for qhull, e.g. "QJ" to joggle inputs. "QgGn" to generate partial convex hull that is visible to the nth point.
     * See http://www.qhull.org/html/qh-optq.htm for more details.
     * @return
     */
    template<typename Container = Eigen::Ref<const Eigen::Matrix3Xd>>
    void
    ConvexHull(const Container &points, std::size_t num_points, std::vector<long> &hull_pt_map, const std::string &options = "Qt") {
        ERL_ASSERTM(num_points > 0, "num_points = {}, it should be > 0.", num_points);

        // convert e.g. Eigen::Matrix3Xd to orgQhull::PointCoordinates
        orgQhull::PointCoordinates qhull_points(3, "");
        qhull_points.append(static_cast<int>(num_points * 3), reinterpret_cast<const double *>(points.data()));

        // calculate convex hull
        orgQhull::Qhull qhull;
        qhull.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(), qhull_points.count(), qhull_points.coordinates(), options.c_str());

        hull_pt_map.resize(qhull.vertexCount());
        std::transform(qhull.vertexList().begin(), qhull.vertexList().end(), hull_pt_map.begin(), [](const auto &vertex) { return vertex.point().id(); });
    }

    /**
     * Get the mesh of the convex hull.
     * @tparam Container A container class that uses contiguous per-point ordering storage and provides .data() methods.
     * @param points
     * @param num_points
     * @param mesh_triangles variable to store the triangles of the mesh, each column is triangle vertex indices.
     * @param mesh_vertices variable to store the vertices of the mesh.
     * @param hull_pt_map variable to store the indices of the points on the convex hull.
     * @param options options for qhull, e.g. "QJ" to joggle inputs. "QgGn" to generate partial convex hull that is visible to the nth point.
     * See http://www.qhull.org/html/qh-optq.htm for more details.
     */
    template<typename Container = Eigen::Ref<const Eigen::Matrix3Xd>>
    void
    ConvexHull(
        const Container &points,
        std::size_t num_points,
        Eigen::Matrix3Xd &mesh_vertices,
        Eigen::Matrix3Xl &mesh_triangles,
        std::vector<long> &hull_pt_map,
        const std::string &options = "Qt") {
        ERL_ASSERTM(num_points > 0, "num_points = {}, it should be > 0.", num_points);

        // convert Eigen::Matrix3Xd to orgQhull::PointCoordinates
        orgQhull::PointCoordinates qhull_points(3, "");
        auto point_data = reinterpret_cast<const double *>(points.data());
        qhull_points.append(static_cast<int>(num_points * 3), point_data);

        // calculate convex hull
        orgQhull::Qhull qhull;
        qhull.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(), qhull_points.count(), qhull_points.coordinates(), options.c_str());

        // load result from qhull
        orgQhull::QhullFacetList facets = qhull.facetList();
        mesh_triangles.resize(3, static_cast<long>(facets.size()));
        mesh_vertices.resize(3, static_cast<long>(facets.size() * 3));
        std::unordered_map<long, long> indices_map;
        long triangle_index = 0;
        Eigen::Vector3d hull_center(0.0, 0.0, 0.0);
        hull_pt_map.clear();
        hull_pt_map.reserve(num_points);
        auto point3_data = reinterpret_cast<const Eigen::Vector3d *>(point_data);
        for (orgQhull::QhullFacet &facet: facets) {
            if (!facet.isGood()) { continue; }  // skip degenerate facets
            orgQhull::QhullVertexSet vertices = facet.vertices();

            long triangle_subscript = 0;
            for (const orgQhull::QhullVertex &vertex: vertices) {
                long point_id = vertex.point().id();  // index of the point in the original point cloud
                mesh_triangles(triangle_subscript, triangle_index) = point_id;
                if (indices_map.try_emplace(point_id, static_cast<long>(indices_map.size())).second) {  // the point is first seen
                    mesh_vertices.col(static_cast<long>(indices_map.size() - 1)) = point3_data[point_id];
                    hull_pt_map.push_back(point_id);       // store the index of the point
                    hull_center += point3_data[point_id];  // update the center of the convex hull
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
            if ((p1 - p0).cross(p2 - p0).dot((p0 + p1 + p2) * (1.0 / 3.0) - hull_center) < 0.0) { std::swap(triangle[0], triangle[1]); }
        }
    }
}  // namespace erl::geometry
