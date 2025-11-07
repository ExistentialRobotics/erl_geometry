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

    void
    GetMinimalOrientedBoundingBox(
        const open3d::geometry::TriangleMesh &mesh,
        bool z_up,
        Eigen::Vector3d &box_center,
        Eigen::Matrix3d &box_rotation,
        Eigen::Vector3d &box_extent);

}  // namespace erl::geometry
