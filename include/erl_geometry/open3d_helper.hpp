#pragma once

#include <open3d/geometry/TriangleMesh.h>

#include <memory>

namespace erl::geometry {

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateEllipsoidMesh(double a, double b, double c, long num_azimuths = 360, long num_elevations = 180);

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateAxisMesh(const Eigen::Matrix4d &pose, double axis_length);
}  // namespace erl::geometry
