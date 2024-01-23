#pragma once

#include <open3d/geometry/TriangleMesh.h>
#include "earcut.hpp"

namespace erl::geometry {

    std::shared_ptr<open3d::geometry::TriangleMesh>
    PolygonToMesh(const std::vector<std::vector<Eigen::Vector2d>> &polygon, double z, bool upward_normal = true);
}
