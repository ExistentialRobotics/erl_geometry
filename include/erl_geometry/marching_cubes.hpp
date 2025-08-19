#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {
    struct MarchingCubes {
        const static int kTriangleEdgeIndexTable[256][16];
        const static int kTriangleCountTable[256];
        const static int kTriangleUniqueEdgeIndexTable[256][14];
        const static int kTriangleVertexIndexTable[256][16];
        const static int kEdgeVertexIndexTable[12][2];
        const static int kCubeVertexCodes[8][3];
        const static int kCubeEdgeCodes[12][4];

        // generate kTriangleUniqueEdgeIndexTable
        static void
        ComputeTriangleUniqueEdgeIndexTable();

        // generate kTriangleVertexIndexTable
        static void
        ComputeTriangleVertexIndexTable();

        static const int *
        GetEdgeIndices(int config) {
            if (config <= 0 || config >= 255) { return nullptr; }
            return kTriangleEdgeIndexTable[config];
        }

        static const int *
        GetUniqueEdgeIndices(int config) {
            if (config <= 0 || config >= 255) { return nullptr; }
            return kTriangleUniqueEdgeIndexTable[config];
        }

        static const int *
        GetEdgeVertexIndices(int edge_index) {
            if (edge_index < 0 || edge_index >= 12) { return nullptr; }
            return kEdgeVertexIndexTable[edge_index];
        }

        static const int *
        GetVertexCode(int vertex_index) {
            if (vertex_index < 0 || vertex_index >= 8) { return nullptr; }
            return kCubeVertexCodes[vertex_index];
        }

        static const int *
        GetEdgeCode(int edge_index) {
            if (edge_index < 0 || edge_index >= 12) { return nullptr; }
            return kCubeEdgeCodes[edge_index];
        }

        static int
        CalculateVertexConfigIndex(const double *vertex_values, double iso_value);

        static int
        CalculateVertexConfigIndex(const float *vertex_values, float iso_value);

        /**
         * Single cube marching cube algorithm (double precision).
         * @param vertex_coords the matrix of vertex coordinates, each column is a vertex.
         * @param grid_values the signed distance function values at the vertices.
         * @param iso_value the isosurface value.
         * @param vertices the vector of vertices.
         * @param triangles the vector of triangles.
         * @param face_normals the vector of vertex normals.
         */
        static void
        SingleCube(
            const Eigen::Ref<const Eigen::Matrix<double, 3, 8>> &vertex_coords,
            const Eigen::Ref<const Eigen::Vector<double, 8>> &grid_values,
            double iso_value,
            std::vector<Eigen::Vector3d> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3d> &face_normals);

        /**
         * Single cube marching cube algorithm (single precision).
         * @param vertex_coords the matrix of vertex coordinates, each column is a vertex.
         * @param grid_values the signed distance function values at the vertices.
         * @param iso_value the isosurface value.
         * @param vertices the vector of vertices.
         * @param triangles the vector of triangles.
         * @param face_normals the vector of vertex normals.
         */
        static void
        SingleCube(
            const Eigen::Ref<const Eigen::Matrix<float, 3, 8>> &vertex_coords,
            const Eigen::Ref<const Eigen::Vector<float, 8>> &grid_values,
            float iso_value,
            std::vector<Eigen::Vector3f> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3f> &face_normals);

        struct ValidCube {
            Eigen::Vector3i coords{};
            int cfg_index = 0;
            std::vector<Eigen::Vector3i> edges{};  // global indices of edge, vertex1 and vertex2
        };

        static std::vector<std::vector<ValidCube>>
        CollectValidCubes(
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXd> &grid_values,
            double iso_value,
            bool row_major,
            bool parallel);

        static std::vector<std::vector<ValidCube>>
        CollectValidCubes(
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXf> &grid_values,
            float iso_value,
            bool row_major,
            bool parallel);

        static void
        ProcessValidCubes(
            const std::vector<std::vector<ValidCube>> &valid_cubes,
            const Eigen::Ref<const Eigen::Vector3d> &coords_min,
            const Eigen::Ref<const Eigen::Vector3d> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXd> &grid_values,
            double iso_value,
            bool row_major,
            bool parallel,
            std::vector<Eigen::Vector3d> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3d> &face_normals);

        static void
        ProcessValidCubes(
            const std::vector<std::vector<ValidCube>> &valid_cubes,
            const Eigen::Ref<const Eigen::Vector3f> &coords_min,
            const Eigen::Ref<const Eigen::Vector3f> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXf> &grid_values,
            float iso_value,
            bool row_major,
            bool parallel,
            std::vector<Eigen::Vector3f> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3f> &face_normals);

        static void
        Run(const Eigen::Ref<const Eigen::Vector3d> &coords_min,
            const Eigen::Ref<const Eigen::Vector3d> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXd> &grid_values,
            double iso_value,
            bool row_major,
            bool parallel,
            std::vector<Eigen::Vector3d> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3d> &face_normals);

        static void
        Run(const Eigen::Ref<const Eigen::Vector3f> &coords_min,
            const Eigen::Ref<const Eigen::Vector3f> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXf> &grid_values,
            float iso_value,
            bool row_major,
            bool parallel,
            std::vector<Eigen::Vector3f> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3f> &face_normals);
    };
}  // namespace erl::geometry
