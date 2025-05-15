#include "erl_geometry/marching_squares.hpp"

#include <unordered_map>

/**
 * @brief Implementation of Marching Square Algorithm.
 * @see https://ieeexplore.ieee.org/document/1219671
 * @see https://en.wikipedia.org/wiki/Marching_squares
 *
 * The cell number v is based on the clockwise sum of the following pattern:
 *  8 --- 4
 *  |     |
 *  1 --- 2
 *
 * The coordinate system is (0, 0) at top left, x-axis downwards, y-axis rightwards.
 *
 * The edge of a cell is
 * -------------------------> U
 * | (0, 0)   1    (1, 0)
 * |       #######
 * |     0 #     # 2
 * |       #######
 * | (0, 1)   3    (1, 1)
 * V
 *
 * There are 16 kinds of contour lines. For each case, we need to consider different edges to do
 * sub-pixel computation.
 * |  v  |    pattern   | edges to consider |
 * |  0  | [0, 0; 0, 0] | None              |
 * |  1  | [0, 0; 1, 0] | 0, 3              |
 * |  2  | [0, 0; 0, 1] | 2, 3              |
 * |  3  | [0, 0; 1, 1] | 0, 2              |
 * |  4  | [0, 1; 0, 0] | 1, 2              |
 * |  5  | [0, 1; 1, 0] | 0, 3 and 1, 2     |
 * |  6  | [0, 1; 0, 1] | 1, 3              |
 * |  7  | [0, 1; 1, 1] | 0, 1              |
 * |  8  | [1, 0; 0, 0] | 0, 1              |
 * |  9  | [1, 0; 1, 0] | 1, 3              |
 * |  10 | [1, 0; 0, 1] | 0, 1 and 2, 3     |
 * |  11 | [1, 0; 1, 1] | 1, 2              |
 * |  12 | [1, 1; 0, 0] | 0, 2              |
 * |  13 | [1, 1; 1, 0] | 2, 3              |
 * |  14 | [1, 1; 0, 1] | 0, 3              |
 * |  15 | [1, 1; 1, 1] | None              |
 */

namespace erl::geometry {

    const std::array<MarchingSquares::Edge, 5> MarchingSquares::kBaseEdgeTable = {
        0, 0, 0, 1,  // edge 0
        0, 0, 1, 0,  // edge 1
        1, 0, 1, 1,  // edge 2
        0, 1, 1, 1,  // edge 3
        4, 4, 4, 4   // None
    };

    const std::array<std::array<MarchingSquares::Edge, 4>, 16> MarchingSquares::kEdgePairTable = {
        kBaseEdgeTable[4], kBaseEdgeTable[4], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[3], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[2], kBaseEdgeTable[3], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[2], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[1], kBaseEdgeTable[2], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[3], kBaseEdgeTable[1], kBaseEdgeTable[2],  // and 1, 2
        kBaseEdgeTable[1], kBaseEdgeTable[3], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[1], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[1], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[1], kBaseEdgeTable[3], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[1], kBaseEdgeTable[2], kBaseEdgeTable[3],  // and 2, 3
        kBaseEdgeTable[1], kBaseEdgeTable[2], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[2], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[2], kBaseEdgeTable[3], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[0], kBaseEdgeTable[3], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
        kBaseEdgeTable[4], kBaseEdgeTable[4], kBaseEdgeTable[4], kBaseEdgeTable[4],  //
    };  // 16 x 4 Edge, 16 x 16 long

    template<typename Dtype>
    void
    MarchingSquareImpl(
        const Eigen::Ref<const Eigen::MatrixX<Dtype>> &img,
        const Dtype iso_value,
        Eigen::Matrix2X<Dtype> &vertices,
        Eigen::Matrix2Xi &lines_to_vertices,
        Eigen::Matrix2Xi &objects_to_lines) {

        static auto sort_lines_to_objects = [](Eigen::Matrix2Xi &lines_to_vertices_,
                                               Eigen::Matrix2Xi &objects_to_lines_) {
            const long num_lines = lines_to_vertices_.cols();
            // estimated maximum number of objects
            objects_to_lines_.setConstant(2, num_lines + 1, -1);
            objects_to_lines_(0, 0) = 0;

            bool reverse = false;
            int num_objects = 0;
            for (int line_idx = 0; line_idx < num_lines; ++line_idx) {
                int vertex_idx = lines_to_vertices_(1, line_idx);
                const int next_line_idx = line_idx + 1;

                int next_connected_line_idx = next_line_idx;
                for (; next_connected_line_idx < num_lines; ++next_connected_line_idx) {
                    if (lines_to_vertices_(0, next_connected_line_idx) == vertex_idx ||
                        lines_to_vertices_(1, next_connected_line_idx) == vertex_idx) {
                        break;
                    }
                }

                if (next_connected_line_idx != num_lines) {
                    lines_to_vertices_.col(next_line_idx)
                        .swap(lines_to_vertices_.col(next_connected_line_idx));
                    if (lines_to_vertices_(1, next_line_idx) == vertex_idx) {
                        std::swap(
                            lines_to_vertices_(0, next_line_idx),
                            lines_to_vertices_(1, next_line_idx));
                    }
                } else {
                    // reverse the line sequence of the current object, try to extend it from the
                    // other end
                    const int obj_begin_line_idx = objects_to_lines_(0, num_objects);
                    vertex_idx = lines_to_vertices_(0, obj_begin_line_idx);

                    next_connected_line_idx = next_line_idx;
                    for (; next_connected_line_idx < num_lines; ++next_connected_line_idx) {
                        if (lines_to_vertices_(0, next_connected_line_idx) == vertex_idx ||
                            lines_to_vertices_(1, next_connected_line_idx) == vertex_idx) {
                            break;
                        }
                    }

                    if (next_connected_line_idx != num_lines) {
                        // reverse the sequence from beginIdx to i (included)
                        reverse = true;
                        auto block = lines_to_vertices_.block(
                            0,
                            obj_begin_line_idx,
                            2,
                            next_line_idx - obj_begin_line_idx);
                        // reverse each column: swap line start and end
                        block.colwise().reverseInPlace();
                        block.rowwise().reverseInPlace();  // reverse each row: reverse line order

                        lines_to_vertices_.col(next_line_idx)
                            .swap(lines_to_vertices_.col(next_connected_line_idx));
                        if (lines_to_vertices_(1, next_line_idx) == vertex_idx) {
                            std::swap(
                                lines_to_vertices_(0, next_line_idx),
                                lines_to_vertices_(1, next_line_idx));
                        }
                    } else {  // both ends cannot be extended anymore
                        if (reverse) {
                            auto block = lines_to_vertices_.block(
                                0,
                                obj_begin_line_idx,
                                2,
                                next_line_idx - obj_begin_line_idx);
                            // reverse each column: swap line start and end
                            block.colwise().reverseInPlace();
                            // reverse each row: reverse line order
                            block.rowwise().reverseInPlace();
                            reverse = false;
                        }
                        objects_to_lines_(1, num_objects++) = next_line_idx;
                        objects_to_lines_(0, num_objects) = next_line_idx;
                    }
                }
            }

            if (objects_to_lines_(0, num_objects) == -1) {
                objects_to_lines_(1, num_objects++) = static_cast<int>(num_lines);
            }
            objects_to_lines_.conservativeResize(2, num_objects);
        };

        const long img_height = img.rows();
        const long img_width = img.cols();
        // binary mGoalMask of img <= iso_value
        auto b_mat = Eigen::MatrixX<bool>(img_height, img_width);

        std::vector<MarchingSquares::Edge> edges;
        std::unordered_map<MarchingSquares::Edge, int, MarchingSquares::HashEdge> unique_edges;
        edges.reserve(img_height * img_width);
        unique_edges.reserve(img_height * img_width);

        // 1. compute the first row of b_mat
        for (long y = 0; y < img_width; y++) { b_mat(0, y) = img(0, y) <= iso_value; }

        // 2. compute vMat
        //      a. compute x+1 row of b_mat
        //      b. compute v, Update edges, unique_edges and lines_to_vertices
        int idx_3, idx_4;
        auto get_edge_index = [&](const MarchingSquares::Edge &e) -> int {
            // assign value to `e` only when it is a new key
            auto [map_pair, is_new_edge] = unique_edges.try_emplace(e, edges.size());
            if (is_new_edge) { edges.push_back(e); }
            auto &[edge, edge_index] = *map_pair;
            return edge_index;
        };

        auto &edge_pair_table = MarchingSquares::kEdgePairTable;

        int num_lines = 0;
        for (long v = 0; v < img_height - 1; v++) {
            b_mat(v + 1, 0) = img(v + 1, 0) <= iso_value;
            for (long u = 0; u < img_width - 1; u++) {
                b_mat(v + 1, u + 1) = img(v + 1, u + 1) <= iso_value;

                if (const int val = b_mat(v, u) << 3 | b_mat(v, u + 1) << 2 |
                                    b_mat(v + 1, u + 1) << 1 | b_mat(v + 1, u);
                    val > 0 && val < 15) {
                    const auto &[e1v1x, e1v1y, e1v2x, e1v2y] = edge_pair_table[val][0];
                    int idx_1 = get_edge_index({u + e1v1x, v + e1v1y, u + e1v2x, v + e1v2y});

                    const auto &[e2v1x, e2v1y, e2v2x, e2v2y] = edge_pair_table[val][1];
                    int idx_2 = get_edge_index({u + e2v1x, v + e2v1y, u + e2v2x, v + e2v2y});

                    if (lines_to_vertices.cols() == num_lines) {
                        lines_to_vertices.conservativeResize(2, 2 * num_lines + 1);
                    }
                    lines_to_vertices.col(num_lines++) << idx_1, idx_2;

                    if (val == 5) {
                        const auto &[e3v1x, e3v1y, e3v2x, e3v2y] = edge_pair_table[val][2];
                        idx_3 = get_edge_index({u + e3v1x, v + e3v1y, u + e3v2x, v + e3v2y});

                        const auto &[e4v1x, e4v1y, e4v2x, e4v2y] = edge_pair_table[val][3];
                        idx_4 = get_edge_index({u + e4v1x, v + e4v1y, u + e4v2x, v + e4v2y});

                        if (lines_to_vertices.cols() == num_lines) {
                            lines_to_vertices.conservativeResize(2, 2 * num_lines + 1);
                        }
                        lines_to_vertices.col(num_lines++) << idx_3, idx_4;

                    } else if (val == 10) {
                        const auto &[e3v1x, e3v1y, e3v2x, e3v2y] = edge_pair_table[val][2];
                        idx_3 = get_edge_index({u + e3v1x, v + e3v1y, u + e3v2x, v + e3v2y});

                        const auto &[e4v1x, e4v1y, e4v2x, e4v2y] = edge_pair_table[val][3];
                        idx_4 = get_edge_index({u + e4v1x, v + e4v1y, u + e4v2x, v + e4v2y});

                        if (lines_to_vertices.cols() == num_lines) {
                            lines_to_vertices.conservativeResize(2, 2 * num_lines + 1);
                        }
                        lines_to_vertices.col(num_lines++) << idx_3, idx_4;
                    }
                }
            }
        }
        lines_to_vertices.conservativeResize(2, num_lines);

        // 3. compute sub-pixel vertex coordinate by interpolation
        vertices.resize(2, static_cast<ssize_t>(edges.size()));
        for (ssize_t i = 0; i < static_cast<ssize_t>(edges.size()); ++i) {
            const auto &[v1x, v1y, v2x, v2y] = edges[i];
            const Dtype w1 = std::abs(img(v1y, v1x) - iso_value);
            const Dtype w2 = std::abs(img(v2y, v2x) - iso_value);
            const Dtype a = w2 / (w1 + w2);
            // clang-format off
            vertices.col(i) << static_cast<Dtype>(v1x) * a + static_cast<Dtype>(v2x) * (1.0f - a),
                               static_cast<Dtype>(v1y) * a + static_cast<Dtype>(v2y) * (1.0f - a);
            // clang-format on
        }

        // 4. find objects
        sort_lines_to_objects(lines_to_vertices, objects_to_lines);
    }

    void
    MarchingSquares::Run(
        const Eigen::Ref<const Eigen::MatrixXd> &img,
        const double iso_value,
        Eigen::Matrix2Xd &vertices,
        Eigen::Matrix2Xi &lines_to_vertices,
        Eigen::Matrix2Xi &objects_to_lines) {
        MarchingSquareImpl<double>(img, iso_value, vertices, lines_to_vertices, objects_to_lines);
    }

    void
    MarchingSquares::Run(
        const Eigen::Ref<const Eigen::MatrixXf> &img,
        const float iso_value,
        Eigen::Matrix2Xf &vertices,
        Eigen::Matrix2Xi &lines_to_vertices,
        Eigen::Matrix2Xi &objects_to_lines) {
        MarchingSquareImpl<float>(img, iso_value, vertices, lines_to_vertices, objects_to_lines);
    }

}  // namespace erl::geometry
