#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    struct Surface2D {
        // each column is a vertex
        Eigen::Matrix2Xd vertices;
        // each column is the normal of the corresponding vertex in vertices
        Eigen::Matrix2Xd normals;
        // each column is a pair of vertex indices of a line segment
        Eigen::Matrix2Xi lines_to_vertices;
        // each column is a pair of start and end line indices of a polygon object
        Eigen::Matrix2Xi objects_to_lines;
        // if true, the outside of polygon is free space
        Eigen::VectorXb outside_flags;
        // the i-th element is the index of the object that the i-th vertex belongs to
        Eigen::VectorXi vertices_to_objects;
        // each item is a list of vertex indices for an object
        std::vector<Eigen::VectorXi> objects_to_vertices;

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

        [[nodiscard]] long
        GetNumVertices() const {
            return vertices.cols();
        }

        [[nodiscard]] long
        GetNumLines() const {
            return lines_to_vertices.cols();
        }

        [[nodiscard]] long
        GetNumObjects() const {
            return objects_to_lines.cols();
        }

        [[nodiscard]] long
        GetNumVerticesOfObject(const int idx_object) const {
            return objects_to_vertices[idx_object].size();
        }

        [[nodiscard]] bool
        NormalsAvailable() const {
            return normals.cols() == vertices.cols();
        }

        [[nodiscard]] bool
        OutsideFlagsAvailable() const {
            return outside_flags.size() == objects_to_lines.cols();
        }

        [[nodiscard]] Eigen::Matrix2Xd
        GetObjectVertices(const int idx_object) const {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
            return vertices(Eigen::indexing::all, objects_to_vertices[idx_object]);
#else
            Eigen::Matrix2Xd object_vertices;
            const auto &indices = objects_to_vertices[idx_object];
            object_vertices.resize(2, indices.size());
            for (long i = 0; i < object_vertices.cols(); ++i) {
                object_vertices.col(i) = vertices.col(indices[i]);
            }
            return object_vertices;
#endif
        }

        [[nodiscard]] Eigen::Matrix2Xd
        GetObjectNormals(const int idx_object) const {
#if EIGEN_VERSION_AT_LEAST(3, 4, 0)
            return normals(Eigen::indexing::all, objects_to_vertices[idx_object]);
#else
            Eigen::Matrix2Xd object_normals;
            const auto &indices = objects_to_vertices[idx_object];
            object_normals.resize(2, indices.size());
            for (long i = 0; i < object_normals.cols(); ++i) {
                object_normals.col(i) = normals.col(indices[i]);
            }
            return object_normals;
#endif
        }

        [[nodiscard]] std::pair<int, int>
        GetVertexNeighbors(int idx_vertex_0) const {

            const auto &object_to_vertices = objects_to_vertices[vertices_to_objects[idx_vertex_0]];
            const long n_obj_vtx = object_to_vertices.size();
            for (long i = 0; i < n_obj_vtx; ++i) {
                if (object_to_vertices[i] != idx_vertex_0) { continue; }
                int idx_vertex_1, idx_vertex_2;
                if (i == 0) {
                    idx_vertex_1 = object_to_vertices[n_obj_vtx - 1];
                    idx_vertex_2 = object_to_vertices[1];
                } else if (i == n_obj_vtx - 1) {
                    idx_vertex_1 = object_to_vertices[i - 1];
                    idx_vertex_2 = object_to_vertices[0];
                } else {
                    idx_vertex_1 = object_to_vertices[i - 1];
                    idx_vertex_2 = object_to_vertices[i + 1];
                }
                return {idx_vertex_1, idx_vertex_2};
            }
            return {-1, -1};
        }

    private:
        void
        ComputeObjectsToVertices() {
            const long n_vtx = GetNumVertices();
            const long n_obj = GetNumObjects();

            objects_to_vertices.clear();
            objects_to_vertices.reserve(n_obj);

            vertices_to_objects.setConstant(n_vtx, -1);

            for (long i = 0; i < n_obj; ++i) {
                const int &idx_l_0 = objects_to_lines(0, i);
                const int &idx_l_1 = objects_to_lines(1, i);

                Eigen::VectorXi object_to_vertices(idx_l_1 - idx_l_0 + 1);
                for (int j = idx_l_0; j < idx_l_1; ++j) {
                    vertices_to_objects(lines_to_vertices(0, j)) = static_cast<int>(i);
                    object_to_vertices(j - idx_l_0) = lines_to_vertices(0, j);
                }
                vertices_to_objects(lines_to_vertices(1, idx_l_1 - 1)) = static_cast<int>(i);
                object_to_vertices(idx_l_1 - idx_l_0) = lines_to_vertices(1, idx_l_1 - 1);

                objects_to_vertices.push_back(std::move(object_to_vertices));
            }
        }
    };

}  // namespace erl::geometry
