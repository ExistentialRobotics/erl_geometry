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

    /**
     * Rotates the oriented bounding box such that the specified axis is aligned with the up
     * direction.
     * @param obb The input oriented bounding box
     * @param up_axis_idx The index of the up axis (0 for x, 1 for y, 2 for z)
     * @param box_center The output box center
     * @param box_rotation The output box rotation
     * @param box_extent The output box extent
     */
    void
    GetOrientedBoundingBoxWithAxisUp(
        const open3d::geometry::OrientedBoundingBox &obb,
        int up_axis_idx,
        Eigen::Vector3d &box_center,
        Eigen::Matrix3d &box_rotation,
        Eigen::Vector3d &box_extent);

}  // namespace erl::geometry
