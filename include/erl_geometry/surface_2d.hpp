#pragma once

#include "erl_common//eigen.hpp"
#include "erl_common/assert.hpp"

namespace erl::geometry {

    struct Surface2D {
        Eigen::Matrix2Xd vertices;                         // each column is a vertex
        Eigen::Matrix2Xd normals;                          // each column is the normal of the corresponding vertex in vertices
        Eigen::Matrix2Xi lines_to_vertices;                // each column is a pair of vertex indices of a line segment
        Eigen::Matrix2Xi objects_to_lines;                 // each column is a pair of start & end line indices of a polygon object
        Eigen::VectorXb outside_flags;                     // if true, the outside of polygon is free space
        Eigen::VectorXi vertices_to_objects;               // the i-th element is the index of the object that the i-th vertex belongs to
        std::vector<Eigen::VectorXi> objects_to_vertices;  // each item is a list of vertex indices of an object

    public:
        Surface2D() = delete;

        Surface2D(
            Eigen::Matrix2Xd vertices,
            Eigen::Matrix2Xd normals,
            Eigen::Matrix2Xi lines_to_vertices,
            Eigen::Matrix2Xi objects_to_lines,
            Eigen::VectorXb outside_flags)
            : vertices(std::move(vertices)),
              normals(std::move(normals)),
              lines_to_vertices(std::move(lines_to_vertices)),
              objects_to_lines(std::move(objects_to_lines)),
              outside_flags(std::move(outside_flags)) {

            ComputeObjectsToVertices();
        }

        Surface2D(const Surface2D &surface) = default;

        [[nodiscard]] inline long
        GetNumVertices() const {
            return vertices.cols();
        }

        [[nodiscard]] inline long
        GetNumLines() const {
            return lines_to_vertices.cols();
        }

        [[nodiscard]] inline long
        GetNumObjects() const {
            return objects_to_lines.cols();
        }

        [[nodiscard]] inline long
        GetNumVerticesOfObject(int idx_object) const {
            return objects_to_vertices[idx_object].size();
        }

        [[nodiscard]] inline bool
        NormalsAvailable() const {
            return normals.cols() == vertices.cols();
        }

        [[nodiscard]] inline bool
        OutsideFlagsAvailable() const {
            return outside_flags.size() == objects_to_lines.cols();
        }

        [[nodiscard]] inline Eigen::Matrix2Xd
        GetObjectVertices(int idx_object) const {
            return vertices(Eigen::indexing::all, objects_to_vertices[idx_object]);
        }

        [[nodiscard]] inline Eigen::Matrix2Xd
        GetObjectNormals(int idx_object) const {
            return normals(Eigen::indexing::all, objects_to_vertices[idx_object]);
        }

        [[nodiscard]] inline std::pair<int, int>
        GetVertexNeighbors(int idx_vertex_0) const {

            const auto &object_to_vertices = objects_to_vertices[vertices_to_objects[idx_vertex_0]];
            auto n_obj_vtx = object_to_vertices.size();
            auto last_vtx_idx = n_obj_vtx - 1;
            auto itr = std::find(object_to_vertices.begin(), object_to_vertices.end(), idx_vertex_0);
            if (itr != object_to_vertices.end()) {
                idx_vertex_0 = static_cast<int>(std::distance(object_to_vertices.begin(), itr));
                auto idx_vertex_1 = (idx_vertex_0 == 0 ? object_to_vertices[last_vtx_idx] : object_to_vertices[idx_vertex_0 - 1]);
                auto idx_vertex_2 = (idx_vertex_0 == last_vtx_idx ? object_to_vertices[0] : object_to_vertices[idx_vertex_0 + 1]);
                return {idx_vertex_1, idx_vertex_2};
            } else {
                return {-1, -1};
            }
        }

    private:
        void
        ComputeObjectsToVertices() {
            auto n_vtx = GetNumVertices();
            auto n_obj = GetNumObjects();

            objects_to_vertices.clear();
            objects_to_vertices.reserve(n_obj);

            vertices_to_objects.setConstant(n_vtx, -1);

            for (ssize_t i = 0; i < n_obj; ++i) {
                auto &idx_l_0 = objects_to_lines(0, i);
                auto &idx_l_1 = objects_to_lines(1, i);

                Eigen::VectorXi object_to_vertices = lines_to_vertices.row(0).segment(idx_l_0, idx_l_1 - idx_l_0);
                vertices_to_objects(object_to_vertices).setConstant((int) i);
                objects_to_vertices.push_back(std::move(object_to_vertices));
            }
        }
    };

}  // namespace erl::geometry
