#include "erl_common/assert.hpp"
#include "erl_geometry/visible_ray_synthesis.hpp"
#include "erl_geometry/azimuth_elevation.hpp"
#include <absl/container/flat_hash_map.h>

namespace erl::geometry {

    Eigen::Matrix3Xd
    VisibleRaySynthesis(
        long point_index,
        const Eigen::Ref<const Eigen::Matrix3Xd> &hit_points,
        const Eigen::Ref<const Eigen::Vector3d> &sensor_position,
        long num_azimuth_segments,
        long num_samples_per_azimuth_segment,
        uint64_t seed) {

        std::mt19937 random_engine(seed);

        Eigen::Vector3d viewing_direction = hit_points.col(point_index) - sensor_position;
        double hit_distance = viewing_direction.norm();
        viewing_direction.normalize();

        Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d axis = -viewing_direction.cross(z_axis).normalized();
        double angle = std::acos(-viewing_direction.dot(z_axis));
        Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        double azimuth_resolution = 2 * M_PI / double(num_azimuth_segments);

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
            ERL_DEBUG_ASSERT(ray_info.range > 0.0, "Zero range detected. hit_points.col(i) = %s, sensor_position = %s",
                common::EigenToNumPyFmtString(hit_points.col(i).transpose()).c_str(),
                common::EigenToNumPyFmtString(sensor_position.transpose()).c_str()
            );
            ray_info.dir_world /= ray_info.range;
            if (ray_info.dir_world.dot(viewing_direction) <= 0.0) { continue; }  // skip points behind the viewing position
            // 1.
            Eigen::Vector3d dir_local = rotation * ray_info.dir_world;
            DirectionToAzimuthElevation(dir_local, ray_info.ray_azimuth, ray_info.ray_elevation);
            auto azimuth_index = long((ray_info.ray_azimuth + M_PI) / azimuth_resolution) % num_azimuth_segments;
            // 2.
            Eigen::Vector3d point_local = rotation * (hit_points.col(i) - hit_points.col(point_index));
            ray_info.end_point_elevation = std::asin(point_local.z() / point_local.norm());
            spherical_coords.emplace_back(ray_info.ray_azimuth, ray_info.end_point_elevation);
            // 3.
            double &max_elevation = max_elevations[azimuth_index];
            if (ray_info.end_point_elevation > max_elevation) { max_elevation = ray_info.end_point_elevation; }
            azimuth_rays[azimuth_index].push_back(std::move(ray_info));
        }

        // 4. sample along rays in each azimuth segment
        std::uniform_real_distribution<double> uniform_range_ratio(0.1, 0.9);
        Eigen::Matrix3Xd positions_samples(3, num_azimuth_segments * num_samples_per_azimuth_segment);
        long sample_idx = 0;
        for (auto &[azimuth_index, rays]: azimuth_rays) {
            double &max_elevation = max_elevations[azimuth_index];
            double cos_max_elevation = std::cos(max_elevation) * hit_distance;
            std::uniform_int_distribution<std::size_t> uniform_ray_index(0, rays.size() - 1);
            for (long cnt_samples = 0; cnt_samples < num_samples_per_azimuth_segment; ++cnt_samples) {
                std::size_t ray_index = uniform_ray_index(random_engine);
                RayInfo &ray = rays[ray_index];
                double r = uniform_range_ratio(random_engine);
                double elevation_diff = max_elevation - ray.ray_elevation;
                double max_range = std::min(ray.range, cos_max_elevation / std::sin(elevation_diff));  // calculate max sampling range along the ray
                Eigen::Vector3d position = sensor_position + r * max_range * ray.dir_world;
                positions_samples.col(sample_idx++) << position;
            }
        }
        positions_samples.conservativeResize(3, sample_idx);

        return positions_samples;
    }

}  // namespace erl::geometry
