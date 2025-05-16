#pragma once

#include <open3d/geometry/TriangleMesh.h>

#include <memory>

namespace erl::geometry {

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateEllipsoidMesh(
        double a,
        double b,
        double c,
        long num_azimuths = 360,
        long num_elevations = 180);

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateUnitBoxFrameMesh(double edge_radius);

}  // namespace erl::geometry
