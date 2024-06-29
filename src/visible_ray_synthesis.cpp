#include "erl_geometry/visible_ray_synthesis.hpp"

#include "erl_common/logging.hpp"
#include "erl_common/random.hpp"
#include "erl_common/angle_utils.hpp"

#include <absl/container/flat_hash_map.h>
#include <Eigen/Geometry>

namespace erl::geometry {

    void
    VisibleRaySynthesis(
        const long point_index,
        const Eigen::Ref<const Eigen::Matrix3Xd> &hit_points,
        const Eigen::Ref<const Eigen::Vector3d> &sensor_position,
        const long num_azimuth_segments,
        const long num_samples_per_azimuth_segment,
        const uint64_t seed,
        std::vector<Eigen::Vector3d> &sampled_ray_origins) {

        std::mt19937 random_engine(seed);

        Eigen::Vector3d viewing_direction = hit_points.col(point_index) - sensor_position;
        const double hit_distance = viewing_direction.norm();
        viewing_direction.normalize();

        const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d axis = -viewing_direction.cross(z_axis).normalized();
        const double angle = std::acos(-viewing_direction.dot(z_axis));
        const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        const double azimuth_resolution = 2 * M_PI / static_cast<double>(num_azimuth_segments);

        struct RayInfo {
            double ray_azimuth = 0.0;
            double ray_elevation = 0.0;
            double end_point_elevation = 0.0;
            double range = 0.0;
            Eigen::Vector3d dir_world = {};
        };

        // 1. transform the point cloud to the hit point's frame, and partition the rays into azimuth segments
        absl::flat_hash_map<long, std::vector<RayInfo>> azimuth_rays;
        // 2. remove hit rays behind the viewing position, compute spherical coordinates, and partition the points into azimuth segments
        std::vector<std::pair<double, double>> spherical_coords;  // azimuth, elevation
        spherical_coords.reserve(hit_points.cols());
        // 3. calculate max end_point_elevation in each azimuth segment
        Eigen::VectorXd max_elevations = Eigen::VectorXd::Constant(num_azimuth_segments, -M_PI_2);
        for (long i = 0; i < hit_points.cols(); ++i) {
            if (i == point_index) { continue; }  // skip the hit point
            RayInfo ray_info;
            ray_info.dir_world = hit_points.col(i) - sensor_position;
            ray_info.range = ray_info.dir_world.norm();
            ERL_DEBUG_ASSERT(
                ray_info.range > 0.0,
                "Zero range detected. hit_points.col(i) = {}, sensor_position = {}",
                common::EigenToNumPyFmtString(hit_points.col(i).transpose()),
                common::EigenToNumPyFmtString(sensor_position.transpose()));
            ray_info.dir_world /= ray_info.range;
            if (ray_info.dir_world.dot(viewing_direction) <= 0.0) { continue; }  // skip points behind the viewing position
            // 1.
            Eigen::Vector3d dir_local = rotation * ray_info.dir_world;
            common::DirectionToAzimuthElevation(dir_local, ray_info.ray_azimuth, ray_info.ray_elevation);
            auto azimuth_index = static_cast<long>((ray_info.ray_azimuth + M_PI) / azimuth_resolution) % num_azimuth_segments;
            // 2.
            Eigen::Vector3d point_local = rotation * (hit_points.col(i) - hit_points.col(point_index));
            ray_info.end_point_elevation = std::asin(point_local.z() / point_local.norm());
            spherical_coords.emplace_back(ray_info.ray_azimuth, ray_info.end_point_elevation);
            // 3.
            if (double &max_elevation = max_elevations[azimuth_index]; ray_info.end_point_elevation > max_elevation) {
                max_elevation = ray_info.end_point_elevation;
            }
            azimuth_rays[azimuth_index].push_back(std::move(ray_info));
        }

        // 4. sample along rays in each azimuth segment
        std::uniform_real_distribution<double> uniform_range_ratio(0.1, 0.9);
        sampled_ray_origins.clear();
        sampled_ray_origins.reserve(num_azimuth_segments * num_samples_per_azimuth_segment);
        for (auto &[azimuth_index, rays]: azimuth_rays) {
            const double &max_elevation = max_elevations[azimuth_index];
            const double cos_max_elevation = std::cos(max_elevation) * hit_distance;
            std::uniform_int_distribution<std::size_t> uniform_ray_index(0, rays.size() - 1);
            for (long cnt_samples = 0; cnt_samples < num_samples_per_azimuth_segment; ++cnt_samples) {
                const std::size_t ray_index = uniform_ray_index(random_engine);
                const auto &[ray_azimuth, ray_elevation, end_point_elevation, range, dir_world] = rays[ray_index];
                const double r = uniform_range_ratio(random_engine);
                const double elevation_diff = max_elevation - ray_elevation;
                const double max_range = std::min(range, cos_max_elevation / std::sin(elevation_diff));  // calculate max sampling range along the kRay
                sampled_ray_origins.emplace_back(sensor_position + r * max_range * dir_world);
            }
        }
    }

}  // namespace erl::geometry
