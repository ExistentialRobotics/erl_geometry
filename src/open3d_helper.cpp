#include "erl_geometry/open3d_helper.hpp"

#include "erl_common/angle_utils.hpp"

#include <open3d/geometry/PointCloud.h>

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
}  // namespace erl::geometry
