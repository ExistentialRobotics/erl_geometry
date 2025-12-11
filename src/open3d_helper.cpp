#include "erl_geometry/open3d_helper.hpp"

#include "erl_common/angle_utils.hpp"

#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/visualization/visualizer/VisualizerWithKeyCallback.h>

namespace erl::geometry {

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateEllipsoidMesh(
        const double a,
        const double b,
        const double c,
        const long num_azimuths,
        const long num_elevations) {

        const double azimuth_step = 2 * M_PI / static_cast<double>(num_azimuths);
        const double elevation_step = M_PI / static_cast<double>(num_elevations - 1);

        open3d::geometry::PointCloud point_cloud;
        point_cloud.points_.resize(num_azimuths * num_elevations);

#pragma omp parallel for default(none) \
    shared(point_cloud, a, b, c, num_azimuths, num_elevations, azimuth_step, elevation_step)
        for (long i = 0; i < num_azimuths; ++i) {
            const double azimuth = azimuth_step * static_cast<double>(i);
            for (long j = 0; j < num_elevations; ++j) {
                const double elevation = elevation_step * static_cast<double>(j) - M_PI / 2.0;
                const double x = a * std::cos(azimuth) * std::cos(elevation);
                const double y = b * std::sin(azimuth) * std::cos(elevation);
                const double z = c * std::sin(elevation);
                point_cloud.points_[i * num_elevations + j] = Eigen::Vector3d(x, y, z);
            }
        }

        point_cloud.EstimateNormals();
        for (std::size_t i = 0; i < point_cloud.normals_.size(); ++i) {
            if (point_cloud.points_[i].dot(point_cloud.normals_[i]) < 0) {
                point_cloud.normals_[i] *= -1;
            }
        }
        auto mesh = std::get<0>(
            open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(point_cloud, 8));
        return mesh;
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateUnitBoxFrameMesh(const double edge_radius) {
        // clang-format off
        //       v7_______e6_____________v6
        //        /|                    /|
        //       / |                   / |
        //    e7/  |                e5/  |
        //     /___|______e4_________/   |
        //  v4|    |                 |v5 |e10
        //    |    |                 |   |
        //    |    |e11              |e9 |
        //  e8|    |                 |   |
        //    |    |_________________|___|
        //    |   / v3      e2       |   /v2
        //    |  /                   |  /
        //    | /e3                  | /e1
        //    |/_____________________|/
        //    v0         e0          v1
        // clang-format on

        auto box = std::make_shared<open3d::geometry::TriangleMesh>();
        auto edge = open3d::geometry::TriangleMesh::CreateCylinder(edge_radius, 1.0);

        open3d::geometry::TriangleMesh z_axis = *edge;
        z_axis.Translate({0.0, 0.0, 0.5});
        // *box += edge8;

        open3d::geometry::TriangleMesh edge0 = z_axis;
        edge0.Rotate(
            Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix(),
            {0.0, 0.0, 0.0});
        *box += edge0;

        open3d::geometry::TriangleMesh edge3 = z_axis;
        edge3.Rotate(
            Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix(),
            {0.0, 0.0, 0.0});
        *box += edge3;

        open3d::geometry::TriangleMesh edge1 = edge3;
        edge1.Translate({1.0, 0.0, 0.0});
        *box += edge1;

        open3d::geometry::TriangleMesh edge2 = edge0;
        edge2.Translate({0.0, 1.0, 0.0});
        *box += edge2;

        open3d::geometry::TriangleMesh frame = *box;
        frame.Translate({0.0, 0.0, 1.0});
        *box += frame;

        *box += z_axis;
        z_axis.Translate({1.0, 0.0, 0.0});
        *box += z_axis;
        z_axis.Translate({0.0, 1.0, 0.0});
        *box += z_axis;
        z_axis.Translate({-1.0, 0.0, 0.0});
        *box += z_axis;
        return box;
    }

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateCameraMesh(
        const double img_width,
        const double img_height,
        const double focal_length,
        const double scale,
        const double frame_thickness,
        const Eigen::Vector3d &frame_color,
        const bool with_axis) {

        using namespace open3d::geometry;

        const double width = img_width * scale;
        const double height = img_height * scale;
        const double len = focal_length * scale;
        const double half_w = 0.5 * width;
        const double half_h = 0.5 * height;
        const double edge_f_len = std::sqrt(len * len + half_w * half_w + half_h * half_h);

        auto mesh = std::make_shared<TriangleMesh>();
        auto edge_w = *TriangleMesh::CreateCylinder(frame_thickness, width + 2.0 * frame_thickness);
        auto edge_h = *TriangleMesh::CreateCylinder(frame_thickness, height + frame_thickness);
        auto edge_f = *TriangleMesh::CreateCylinder(frame_thickness, edge_f_len);

        edge_w
            .Rotate(
                Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix(),
                {0, 0, 0})
            .Translate({0, half_h, len});
        *mesh += edge_w;
        edge_w.Translate({0, -height, 0});
        *mesh += edge_w;

        edge_h
            .Rotate(
                Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix(),
                {0, 0, 0})
            .Translate({half_w, 0, len});
        *mesh += edge_h;
        edge_h.Translate({-width, 0, 0});
        *mesh += edge_h;

        edge_f.Translate({0.0, 0.0, 0.5 * edge_f_len});
        for (auto dir: {
                 Eigen::Vector3d(half_w, half_h, len),
                 Eigen::Vector3d(-half_w, half_h, len),
                 Eigen::Vector3d(-half_w, -half_h, len),
                 Eigen::Vector3d(half_w, -half_h, len),
             }) {

            dir.normalize();
            Eigen::Vector3d axis = dir.cross(Eigen::Vector3d::UnitZ());
            const double norm = axis.norm();
            double angle = 0.0;
            if (norm < 1.e-8) {
                axis = Eigen::Vector3d::UnitX();
            } else {
                axis /= norm;
                angle = std::asin(norm);
            }
            auto edge_f_rotated = edge_f;
            edge_f_rotated.Rotate(Eigen::AngleAxisd(angle, axis).toRotationMatrix(), {0, 0, 0});
            *mesh += edge_f_rotated;
        }

        mesh->PaintUniformColor(frame_color);

        if (with_axis) { *mesh += *TriangleMesh::CreateCoordinateFrame(half_w); }
        return mesh;
    }

    void
    GetOrientedBoundingBoxWithAxisUp(
        const open3d::geometry::OrientedBoundingBox &obb,
        int up_axis_idx,
        Eigen::Vector3d &box_center,
        Eigen::Matrix3d &box_rotation,
        Eigen::Vector3d &box_extent) {

        box_center = obb.center_;
        box_rotation = obb.R_;
        box_extent = obb.extent_;

        Eigen::Vector3d up_axis = Eigen::Vector3d::Zero();
        up_axis[up_axis_idx] = 1.0;

        Eigen::Vector3d score = obb.R_.transpose() * up_axis;
        long axis_idx = 0;
        if (score[axis_idx] < score[1]) { axis_idx = 1; }
        if (score[axis_idx] < score[2]) { axis_idx = 2; }

        if (axis_idx != up_axis_idx || score[axis_idx] < 0) {
            Eigen::Matrix3d rotation2;
            Eigen::Vector3d new_up_axis = Eigen::Matrix3d::Identity().col(axis_idx);
            if (score[axis_idx] < 0) { new_up_axis = -new_up_axis; }
            const Eigen::Vector3d v = new_up_axis.cross(up_axis);
            const double c = new_up_axis.dot(up_axis);
            const double s = v.norm();

            if (s < 1.e-5) {
                if (c > 0.0) {  // same direction
                    rotation2 = Eigen::Matrix3d::Identity();
                } else {  // rotate to make the axis up
                    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
                    axis[(up_axis_idx + 1) % 3] = 1.0;
                    rotation2 = Eigen::AngleAxisd(M_PI, axis);
                }
            } else {
                rotation2 = Eigen::AngleAxisd(std::atan2(s, c), v / s);
            }
            box_rotation = box_rotation * rotation2.transpose();
        }

        box_extent = box_rotation.transpose() * obb.R_ * box_extent;
        box_extent = box_extent.cwiseAbs();
    }
}  // namespace erl::geometry
