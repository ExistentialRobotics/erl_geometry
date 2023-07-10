#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {
    void
    MarchingSquare(
        const Eigen::Ref<const Eigen::MatrixXd> &img,
        double iso_value,
        Eigen::Matrix2Xd &vertices,
        Eigen::Matrix2Xi &lines_to_vertices,
        Eigen::Matrix2Xi &objects_to_lines);
}
