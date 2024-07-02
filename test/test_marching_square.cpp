#include "erl_common/assert.hpp"
#include "erl_geometry/marching_square.hpp"

#include <iostream>

/** Output:
img:
1 1 1 1 1
1 2 3 2 1
1 3 3 3 1
1 2 3 2 1
1 1 1 1 1
linesOutBuf:
 0  1  5  7  9 10 11  8  6  4  3  2
 1  5  7  9 10 11  8  6  4  3  2  0
verticesOutBuf:
1 0 2 3 4 0 4 0 4 1 2 3
0 1 0 0 1 2 2 3 3 4 4 4
objectOutBuf:
 0
12
 */

int
main() {
    // clang-format off
    Eigen::MatrixXd img = Eigen::MatrixXd::Zero(5, 5);
    img << 1, 1, 1, 1, 1,
           1, 2, 3, 2, 1,
           1, 3, 3, 3, 1,
           1, 2, 3, 2, 1,
           1, 1, 1, 1, 1;

    Eigen::Matrix2Xd vertices_gt(2, 12);
    vertices_gt << 1, 0, 2, 3, 4, 0, 4, 0, 4, 1, 2, 3,
                   0, 1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 4;

    Eigen::Matrix2Xi lines_to_vertices_gt(2, 12);
    lines_to_vertices_gt << 0, 1, 5, 7,  9, 10, 11, 8, 6, 4, 3, 2,
                            1, 5, 7, 9, 10, 11,  8, 6, 4, 3, 2, 0;

    Eigen::Matrix2Xi  objects_to_lines_gt(2, 1);
    objects_to_lines_gt << 0, 12;
    // clang-format on

    Eigen::Matrix2Xd vertices;
    Eigen::Matrix2Xi lines_to_vertices;
    Eigen::Matrix2Xi objects_to_lines;
    double iso_value = 1;
    erl::geometry::MarchingSquare(img, iso_value, vertices, lines_to_vertices, objects_to_lines);

    std::cout << "img:" << std::endl << img << std::endl;
    std::cout << "lines_to_vertices:" << std::endl << lines_to_vertices << std::endl;
    std::cout << "vertices:" << std::endl << vertices << std::endl;
    std::cout << "objectOutBuf:" << std::endl << objects_to_lines << std::endl;

    ERL_ASSERT((vertices.array() == vertices_gt.array()).all());
    ERL_ASSERT((lines_to_vertices.array() == lines_to_vertices_gt.array()).all());
    ERL_ASSERT((objects_to_lines.array() == objects_to_lines_gt.array()).all());

    return 0;
}
