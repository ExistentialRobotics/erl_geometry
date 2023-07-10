#include "erl_geometry/marching_square.hpp"
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
 * There are 16 kinds of contour lines. For each case, we need to consider different edges to do sub-pixel computation.
 * |  v  |    pattern   | edges to consider |
 * |  0  | [0, 0; 0, 0] | kNone              |
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
 * |  15 | [1, 1; 1, 1] | kNone              |
 */

namespace erl::geometry {

    void
    MarchingSquare(
        const Eigen::Ref<const Eigen::MatrixXd> &img,
        double iso_value,
        Eigen::Matrix2Xd &vertices,
        Eigen::Matrix2Xi &lines_to_vertices,
        Eigen::Matrix2Xi &objects_to_lines) {

        typedef struct Edge {
            Eigen::Vector2i v_1, v_2;  // vertices of an edge

            bool
            operator==(const Edge &other) const {
                return other.v_1 == v_1 && other.v_2 == v_2;
            }
        } Edge;

        struct HashEdge {
            std::size_t
            operator()(const Edge &e) const noexcept {
                std::hash<long> long_hash;
                std::size_t h_1 = long_hash(e.v_1.x());
                std::size_t h_2 = long_hash(e.v_1.y());
                std::size_t h_3 = long_hash(e.v_2.x());
                std::size_t h_4 = long_hash(e.v_2.y());
                return ((h_1 ^ (h_2 << 1)) ^ (h_3 << 1)) ^ (h_4 << 1);
            }
        };

        static std::vector<Edge> base_edge_table{
            {{0, 0}, {0, 1}},  // edge 0
            {{0, 0}, {1, 0}},  // edge 1
            {{1, 0}, {1, 1}},  // edge 2
            {{0, 1}, {1, 1}},  // edge 3
            {{4, 4}, {4, 4}}   // kNone
        };

        static std::vector<std::vector<Edge>> edge_pair_table = {
            {base_edge_table[4], base_edge_table[4]},
            {base_edge_table[0], base_edge_table[3]},
            {base_edge_table[2], base_edge_table[3]},
            {base_edge_table[0], base_edge_table[2]},
            {base_edge_table[1], base_edge_table[2]},
            {base_edge_table[0], base_edge_table[3], base_edge_table[1], base_edge_table[2]},  // val = 5 and 1, 2
            {base_edge_table[1], base_edge_table[3]},
            {base_edge_table[0], base_edge_table[1]},
            {base_edge_table[0], base_edge_table[1]},
            {base_edge_table[1], base_edge_table[3]},
            {base_edge_table[0], base_edge_table[1], base_edge_table[2], base_edge_table[3]},  // and 2, 3
            {base_edge_table[1], base_edge_table[2]},
            {base_edge_table[0], base_edge_table[2]},
            {base_edge_table[2], base_edge_table[3]},
            {base_edge_table[0], base_edge_table[3]},
            {base_edge_table[4], base_edge_table[4]}};  // 16 x 2 Edge, 16 x 8 long

        static auto sort_lines_to_objects = [](Eigen::Matrix2Xi &lines_to_vertices, Eigen::Matrix2Xi &objects_to_lines) {
            auto num_lines = lines_to_vertices.cols();
            objects_to_lines.setConstant(2, num_lines / 3 + 1, -1);  // estimated maximum number of objects
            objects_to_lines(0, 0) = 0;

            bool reverse = false;
            int num_objects = 0;
            for (int line_idx = 0; line_idx < num_lines; ++line_idx) {
                auto vertex_idx = lines_to_vertices(1, line_idx);
                auto next_line_idx = line_idx + 1;
                auto next_connected_line_idx = next_line_idx;

                for (; next_connected_line_idx < num_lines; ++next_connected_line_idx) {
                    if (lines_to_vertices(0, next_connected_line_idx) == vertex_idx || lines_to_vertices(1, next_connected_line_idx) == vertex_idx) { break; }
                }

                if (next_connected_line_idx != num_lines) {
                    lines_to_vertices.col(next_line_idx).swap(lines_to_vertices.col(next_connected_line_idx));
                    if (lines_to_vertices(1, next_line_idx) == vertex_idx) {
                        std::swap(lines_to_vertices(0, next_line_idx), lines_to_vertices(1, next_line_idx));
                    }
                } else {  // reverse the line sequence of the current object, try to extend it from the other end
                    auto obj_begin_line_idx = objects_to_lines(0, num_objects);
                    vertex_idx = lines_to_vertices(0, obj_begin_line_idx);

                    next_connected_line_idx = next_line_idx;
                    for (; next_connected_line_idx < num_lines; ++next_connected_line_idx) {
                        if (lines_to_vertices(0, next_connected_line_idx) == vertex_idx || lines_to_vertices(1, next_connected_line_idx) == vertex_idx) {
                            break;
                        }
                    }

                    if (next_connected_line_idx != num_lines) {
                        // reverse the sequence from beginIdx to i (included)
                        reverse = true;
                        auto block = lines_to_vertices.block(0, obj_begin_line_idx, 2, next_line_idx - obj_begin_line_idx);
                        block.colwise().reverseInPlace();  // reverse each column: swap line start & end
                        block.rowwise().reverseInPlace();  // reverse each row: reverse line order

                        lines_to_vertices.col(next_line_idx).swap(lines_to_vertices.col(next_connected_line_idx));
                        if (lines_to_vertices(1, next_line_idx) == vertex_idx) {
                            std::swap(lines_to_vertices(0, next_line_idx), lines_to_vertices(1, next_line_idx));
                        }
                    } else {  // both ends cannot be extended anymore
                        if (reverse) {
                            auto block = lines_to_vertices.block(0, obj_begin_line_idx, 2, next_line_idx - obj_begin_line_idx);
                            block.colwise().reverseInPlace();  // reverse each column: swap line start & end
                            block.rowwise().reverseInPlace();  // reverse each row: reverse line order
                            reverse = false;
                        }
                        objects_to_lines(1, num_objects++) = next_line_idx;
                        objects_to_lines(0, num_objects) = next_line_idx;
                    }
                }
            }

            if (objects_to_lines(0, num_objects) == -1) { objects_to_lines(1, num_objects++) = (int) num_lines; }
            objects_to_lines.conservativeResize(2, num_objects);
        };

        auto img_height = img.rows();
        auto img_width = img.cols();

        auto b_mat = Eigen::MatrixX<bool>(img_height, img_width);  // binary mGoalMask of img <= iso_value

        std::vector<Edge> edges;
        std::unordered_map<Edge, int, HashEdge> unique_edges;

        // 1. compute first row of b_mat
        for (long y = 0; y < img_width; y++) { b_mat(0, y) = img(0, y) <= iso_value; }

        // 2. compute vMat
        //      a. compute x+1 row of b_mat
        //      b. compute v, Update edges, unique_edges and lines_to_vertices
        int idx_1, idx_2, idx_3, idx_4;
        auto get_edge_index = [&](const Edge &e) -> int {
            auto [map_pair, is_new_edge] = unique_edges.try_emplace(e, edges.size());  // assign value to `e` only when it is a new key
            if (is_new_edge) { edges.push_back(e); }
            auto &[edge, edge_index] = *map_pair;
            return edge_index;
        };

        int num_lines = 0;
        // auto &edge_pair_table = getEdgePairTable();
        for (long v = 0; v < img_height - 1; v++) {
            b_mat(v + 1, 0) = img(v + 1, 0) <= iso_value;
            for (long u = 0; u < img_width - 1; u++) {
                b_mat(v + 1, u + 1) = img(v + 1, u + 1) <= iso_value;

                auto val = (b_mat(v, u) << 3) | (b_mat(v, u + 1) << 2) | (b_mat(v + 1, u + 1) << 1) | b_mat(v + 1, u);
                if ((val > 0) && (val < 15)) {
                    auto &e_1 = edge_pair_table[val][0];
                    idx_1 = get_edge_index({{u + e_1.v_1.x(), v + e_1.v_1.y()}, {u + e_1.v_2.x(), v + e_1.v_2.y()}});

                    auto &e_2 = edge_pair_table[val][1];
                    idx_2 = get_edge_index({{u + e_2.v_1.x(), v + e_2.v_1.y()}, {u + e_2.v_2.x(), v + e_2.v_2.y()}});

                    if (lines_to_vertices.cols() == num_lines) { lines_to_vertices.conservativeResize(2, 2 * num_lines + 1); }
                    lines_to_vertices.col(num_lines++) << idx_1, idx_2;

                    if (val == 5) {
                        auto &e_3 = edge_pair_table[val][2];
                        idx_3 = get_edge_index({{u + e_3.v_1.x(), v + e_3.v_1.y()}, {u + e_3.v_2.x(), v + e_3.v_2.y()}});

                        auto &e_4 = edge_pair_table[val][3];
                        idx_4 = get_edge_index({{u + e_4.v_1.x(), v + e_4.v_1.y()}, {u + e_4.v_2.x(), v + e_4.v_2.y()}});

                        if (lines_to_vertices.cols() == num_lines) { lines_to_vertices.conservativeResize(2, 2 * num_lines + 1); }
                        lines_to_vertices.col(num_lines++) << idx_3, idx_4;

                    } else if (val == 10) {
                        auto &e_3 = edge_pair_table[val][2];
                        idx_3 = get_edge_index({{u + e_3.v_1.x(), v + e_3.v_1.y()}, {u + e_3.v_2.x(), v + e_3.v_2.y()}});

                        auto &e_4 = edge_pair_table[val][3];
                        idx_4 = get_edge_index({{u + e_4.v_1.x(), v + e_4.v_1.y()}, {u + e_4.v_2.x(), v + e_4.v_2.y()}});

                        if (lines_to_vertices.cols() == num_lines) { lines_to_vertices.conservativeResize(2, 2 * num_lines + 1); }
                        lines_to_vertices.col(num_lines++) << idx_3, idx_4;
                    }
                }
            }
        }
        lines_to_vertices.conservativeResize(2, num_lines);

        // 3. compute sub-pixel vertex coordinate by interpolation
        vertices.resize(2, ssize_t(edges.size()));
        double w_1, w_2, a;
        for (ssize_t i = 0; i < ssize_t(edges.size()); ++i) {
            auto &p = edges[i];
            w_1 = std::abs(img(p.v_1.y(), p.v_1.x()) - iso_value);
            w_2 = std::abs(img(p.v_2.y(), p.v_2.x()) - iso_value);
            a = w_2 / (w_1 + w_2);
            vertices.col(i) << double(p.v_1.x()) * a + double(p.v_2.x()) * (1. - a), double(p.v_1.y()) * a + double(p.v_2.y()) * (1. - a);
        }

        // 4. find objects
        sort_lines_to_objects(lines_to_vertices, objects_to_lines);
    }
}  // namespace erl::geometry
