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

        /**
         * Single cube marching cube algorithm (double precision).
         * @param vertex_coords the matrix of vertex coordinates, each column is a vertex.
         * @param sdf_values the signed distance function values at the vertices.
         * @param vertices the vector of vertices.
         * @param triangles the vector of triangles.
         * @param face_normals the vector of vertex normals.
         */
        static void
        SingleCube(
            const Eigen::Ref<const Eigen::Matrix<double, 3, 8>> &vertex_coords,
            const Eigen::Ref<const Eigen::Vector<double, 8>> &sdf_values,
            std::vector<Eigen::Vector3d> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3d> &face_normals);

        /**
         * Single cube marching cube algorithm (single precision).
         * @param vertex_coords the matrix of vertex coordinates, each column is a vertex.
         * @param sdf_values the signed distance function values at the vertices.
         * @param vertices the vector of vertices.
         * @param triangles the vector of triangles.
         * @param face_normals the vector of vertex normals.
         */
        static void
        SingleCube(
            const Eigen::Ref<const Eigen::Matrix<float, 3, 8>> &vertex_coords,
            const Eigen::Ref<const Eigen::Vector<float, 8>> &sdf_values,
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
            const Eigen::Ref<const Eigen::VectorXd> &sdf_values,
            bool row_major,
            bool parallel);

        static std::vector<std::vector<ValidCube>>
        CollectValidCubes(
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXf> &sdf_values,
            bool row_major,
            bool parallel);

        static void
        ProcessValidCubes(
            const std::vector<std::vector<ValidCube>> &valid_cubes,
            const Eigen::Ref<const Eigen::Vector3d> &coords_min,
            const Eigen::Ref<const Eigen::Vector3d> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXd> &sdf_values,
            bool row_major,
            std::vector<Eigen::Vector3d> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3d> &face_normals,
            bool parallel);

        static void
        ProcessValidCubes(
            const std::vector<std::vector<ValidCube>> &valid_cubes,
            const Eigen::Ref<const Eigen::Vector3f> &coords_min,
            const Eigen::Ref<const Eigen::Vector3f> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXf> &sdf_values,
            bool row_major,
            std::vector<Eigen::Vector3f> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3f> &face_normals,
            bool parallel);

        static void
        Run(const Eigen::Ref<const Eigen::Vector3d> &coords_min,
            const Eigen::Ref<const Eigen::Vector3d> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXd> &sdf_values,
            bool row_major,
            std::vector<Eigen::Vector3d> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3d> &face_normals,
            bool parallel);

        static void
        Run(const Eigen::Ref<const Eigen::Vector3f> &coords_min,
            const Eigen::Ref<const Eigen::Vector3f> &grid_res,
            const Eigen::Ref<const Eigen::Vector3i> &grid_shape,
            const Eigen::Ref<const Eigen::VectorXf> &sdf_values,
            bool row_major,
            std::vector<Eigen::Vector3f> &vertices,
            std::vector<Eigen::Vector3i> &triangles,
            std::vector<Eigen::Vector3f> &face_normals,
            bool parallel);
    };
}  // namespace erl::geometry
