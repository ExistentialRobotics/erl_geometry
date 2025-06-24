#include "erl_geometry/polygon_to_mesh.hpp"

#include "erl_common/logging.hpp"

#include <open3d/io/TriangleMeshIO.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include <numeric>
#include <vector>

namespace erl::geometry {

    std::shared_ptr<open3d::geometry::TriangleMesh>
    PolygonToMesh(
        const std::vector<std::vector<Eigen::Vector2d>> &polygon,
        double z,
        const bool upward_normal) {
        auto triangles_vertex_indices = RunEarcut(polygon);
        ERL_ASSERTM(
            triangles_vertex_indices.size() % 3 == 0,
            "Wrong earcut result: the number of triangles vertex indices is not a multiple of 3.");

        std::vector<std::size_t> num_vertices_per_polygon;
        num_vertices_per_polygon.reserve(polygon.size());
        std::transform(
            polygon.begin(),
            polygon.end(),
            std::back_inserter(num_vertices_per_polygon),
            [](const auto &polygon_i) { return polygon_i.size(); });
        std::vector<std::size_t> cum_sum_num_vertices_per_polygon(num_vertices_per_polygon.size());
        std::partial_sum(
            num_vertices_per_polygon.begin(),
            num_vertices_per_polygon.end(),
            cum_sum_num_vertices_per_polygon.begin());

        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        mesh->vertices_.reserve(cum_sum_num_vertices_per_polygon.back());
        mesh->triangles_.reserve(triangles_vertex_indices.size() / 3);

        for (const auto &polygon_i: polygon) {
            for (const auto &vertex: polygon_i) {
                mesh->vertices_.emplace_back(vertex[0], vertex[1], z);
            }
        }
        const double sign = upward_normal ? 1 : -1;
        for (std::size_t i = 0; i < triangles_vertex_indices.size(); i += 3) {
            Eigen::Vector3d v01 = mesh->vertices_[triangles_vertex_indices[i + 1]] -
                                  mesh->vertices_[triangles_vertex_indices[i]];
            Eigen::Vector3d v02 = mesh->vertices_[triangles_vertex_indices[i + 2]] -
                                  mesh->vertices_[triangles_vertex_indices[i]];
            if (Eigen::Vector3d normal = v01.cross(v02); normal[2] * sign < 0) {
                mesh->triangles_.emplace_back(
                    triangles_vertex_indices[i],
                    triangles_vertex_indices[i + 2],
                    triangles_vertex_indices[i + 1]);
            } else {
                mesh->triangles_.emplace_back(
                    triangles_vertex_indices[i],
                    triangles_vertex_indices[i + 1],
                    triangles_vertex_indices[i + 2]);
            }
        }
        mesh->ComputeTriangleNormals();
        mesh->ComputeVertexNormals();
        return mesh;
    }

}  // namespace erl::geometry
