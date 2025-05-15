#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    struct MarchingSquares {
        struct Edge {
            long v1x, v1y, v2x, v2y;

            bool
            operator==(const Edge &other) const {
                return v1x == other.v1x && v1y == other.v1y && v2x == other.v2x && v2y == other.v2y;
            }
        };

        struct HashEdge {
            std::size_t
            operator()(const Edge &e) const noexcept {
                constexpr std::hash<long> long_hash;
                std::size_t &&h_1 = long_hash(e.v1x);
                std::size_t &&h_2 = long_hash(e.v1y);
                std::size_t &&h_3 = long_hash(e.v2x);
                std::size_t &&h_4 = long_hash(e.v2y);
                return h_1 ^ h_2 << 1 ^ h_3 << 1 ^ h_4 << 1;
            }
        };

        static const std::array<Edge, 5> kBaseEdgeTable;
        static const std::array<std::array<Edge, 4>, 16> kEdgePairTable;

        static void
        Run(const Eigen::Ref<const Eigen::MatrixXd> &img,
            double iso_value,
            Eigen::Matrix2Xd &vertices,
            Eigen::Matrix2Xi &lines_to_vertices,
            Eigen::Matrix2Xi &objects_to_lines);

        static void
        Run(const Eigen::Ref<const Eigen::MatrixXf> &img,
            float iso_value,
            Eigen::Matrix2Xf &vertices,
            Eigen::Matrix2Xi &lines_to_vertices,
            Eigen::Matrix2Xi &objects_to_lines);
    };
}  // namespace erl::geometry
