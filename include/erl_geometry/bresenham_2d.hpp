#pragma once

#include "erl_common/eigen.hpp"

#include <functional>

namespace erl::geometry {
    Eigen::Matrix2Xi
    Bresenham2D(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::Vector2i> &end);

    /**
     * Compute the vertices of the line to plot. May stop in advance when stop returns true.
     * @param start
     * @param end
     * @param stop
     * @return
     */
    Eigen::Matrix2Xi
    Bresenham2D(
        const Eigen::Ref<const Eigen::Vector2i> &start,
        const Eigen::Ref<const Eigen::Vector2i> &end,
        const std::function<bool(int, int)> &stop);

    Eigen::Matrix2Xi
    ComputePixelsOfPolygonContour(const Eigen::Ref<const Eigen::Matrix2Xi> &polygon_vertices);
}  // namespace erl::geometry
