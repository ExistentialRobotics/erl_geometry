#include "erl_geometry/open3d_helper.hpp"

#include "erl_common/angle_utils.hpp"

#include <open3d/geometry/PointCloud.h>

namespace erl::geometry {

    std::shared_ptr<open3d::geometry::TriangleMesh>
    CreateEllipsoidMesh(const double a, const double b, const double c, const long num_azimuths, const long num_elevations) {

        const double azimuth_step = 2 * M_PI / static_cast<double>(num_azimuths);
        const double elevation_step = M_PI / static_cast<double>(num_elevations - 1);

        open3d::geometry::PointCloud point_cloud;
        point_cloud.points_.resize(num_azimuths * num_elevations);

#pragma omp parallel for default(none) shared(point_cloud, a, b, c, num_azimuths, num_elevations, azimuth_step, elevation_step)
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
            if (point_cloud.points_[i].dot(point_cloud.normals_[i]) < 0) { point_cloud.normals_[i] *= -1; }
        }
        auto mesh = std::get<0>(open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(point_cloud, 8));
        return mesh;
    }
}  // namespace erl::geometry
