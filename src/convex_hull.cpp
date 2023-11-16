#include "erl_geometry/convex_hull.hpp"
#include "erl_common/assert.hpp"
#include "libqhullcpp/PointCoordinates.h"
#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullVertexSet.h"

#include <set>

namespace erl::geometry {

    void
    ConvexHull(const Eigen::Ref<const Eigen::Matrix3Xd> &points, std::vector<long> &hull_pt_map, bool joggle_inputs) {
        long num_points = points.cols();
        ERL_ASSERTM(num_points > 0, "num_points = %ld, it should be > 0.", num_points);

        // convert Eigen::Matrix3Xd to orgQhull::PointCoordinates
        orgQhull::PointCoordinates qhull_points(3, "");
        qhull_points.append(int(points.cols() * 3), points.data());

        // calculate convex hull
        orgQhull::Qhull qhull;
        std::string options = "Qt";
        if (joggle_inputs) { options = "QJ"; }
        qhull.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(), qhull_points.count(), qhull_points.coordinates(), options.c_str());

        hull_pt_map.clear();
        hull_pt_map.reserve(qhull.vertexCount());
        for (const orgQhull::QhullVertex &kVertex: qhull.vertexList()) {
            hull_pt_map.push_back(kVertex.point().id());
        }
    }

    void
    ConvexHull(
        const Eigen::Ref<const Eigen::Matrix3Xd> &points,
        Eigen::Matrix3Xl &mesh_triangles,
        Eigen::Matrix3Xd &mesh_vertices,
        std::vector<long> &hull_pt_map,
        bool joggle_inputs
    ) {
        long num_points = points.cols();
        ERL_ASSERTM(num_points > 0, "num_points = %ld, it should be > 0.", num_points);

        // convert Eigen::Matrix3Xd to orgQhull::PointCoordinates
        orgQhull::PointCoordinates qhull_points(3, "");
        qhull_points.append(int(points.cols() * 3), points.data());

        // calculate convex hull
        orgQhull::Qhull qhull;
        std::string options = "Qt";
        if (joggle_inputs) { options = "QJ"; }
        qhull.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(), qhull_points.count(), qhull_points.coordinates(), options.c_str());

        // load result from qhull
        orgQhull::QhullFacetList facets = qhull.facetList();
        mesh_triangles.resize(3, long(facets.size()));
        mesh_vertices.resize(3, long(facets.size() * 3));
        std::unordered_map<long, long> indices_map;
        long triangle_index = 0;
        Eigen::Vector3d hull_center(0.0, 0.0, 0.0);
        hull_pt_map.clear();
        hull_pt_map.reserve(points.cols());
        for (orgQhull::QhullFacet &facet: facets) {
            if (!facet.isGood()) { continue; }  // skip degenerate facets
            orgQhull::QhullVertexSet vertices = facet.vertices();

            long triangle_subscript = 0;
            for (const orgQhull::QhullVertex &vertex: vertices) {
                long point_id = vertex.point().id();  // index of the point in the original point cloud
                mesh_triangles(triangle_subscript, triangle_index) = point_id;
                if (indices_map.try_emplace(point_id, long(indices_map.size())).second) {    // the point is first seen
                    mesh_vertices.col(long(indices_map.size() - 1)) = points.col(point_id);  // store the point
                    hull_pt_map.push_back(point_id);                                         // store the index of the point
                    hull_center += points.col(point_id);                                     // update the center of the convex hull
                }
                triangle_subscript++;
            }

            triangle_index++;
        }
        auto num_hull_points = long(indices_map.size());
        hull_center /= double(num_hull_points);
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

            auto p0 = mesh_vertices.col(triangle[0]);
            auto p1 = mesh_vertices.col(triangle[1]);
            auto p2 = mesh_vertices.col(triangle[2]);

            Eigen::Vector3d e1 = p1 - p0;
            Eigen::Vector3d e2 = p2 - p0;
            Eigen::Vector3d normal = e1.cross(e2);
            Eigen::Vector3d triangle_center = (p0 + p1 + p2) * (1.0 / 3.0);
            if (normal.dot(triangle_center - hull_center) < 0.0) { std::swap(triangle[0], triangle[1]); }  // make sure the normal points outward
        }
    }
}  // namespace erl::geometry
