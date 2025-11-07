#pragma once

#include "aabb.hpp"

#include "erl_common/eigen.hpp"

#include <random>

namespace erl::geometry {
    template<typename Dtype, int Dim>
    class OccupancyMap {
    public:
        using VectorD = Eigen::Vector<Dtype, Dim>;
        using VectorX = Eigen::VectorX<Dtype>;
        using MatrixDX = Eigen::Matrix<Dtype, Dim, Eigen::Dynamic>;
        using AabbD = Aabb<Dtype, Dim>;

        struct RayInfo {
            VectorD origin = VectorD::Zero();  // origin point of the ray
            VectorD end = VectorD::Zero();     // end point of the ray
            bool hit_flag = false;             // true if the ray hits a point inside the map
            long num_free_points = 0;          // number of free points
            Dtype d1 = 0.0f;                   // left ratio limit
            Dtype d2 = 0.0f;                   // right ratio limit

            RayInfo() = default;

            RayInfo(
                const VectorD origin,
                const VectorD end,
                const bool hit_flag,
                const long num_free_points,
                const Dtype d1,
                const Dtype d2)
                : origin(origin),
                  end(end),
                  hit_flag(hit_flag),
                  num_free_points(num_free_points),
                  d1(d1),
                  d2(d2) {}

            [[nodiscard]] bool
            operator==(const RayInfo &other) const {
                return origin == other.origin && end == other.end && hit_flag == other.hit_flag &&
                       num_free_points == other.num_free_points && d1 == other.d1 && d2 == other.d2;
            }

            [[nodiscard]] bool
            operator!=(const RayInfo &other) const {
                return !(*this == other);
            }
        };

        /**
         *
         * @param sensor_position the position of the sensor in the world frame.
         * @param points the point cloud in the world frame of the sensor measurement.
         * @param point_indices the indices of the points in the point cloud to sample.
         * @param map_boundary map boundary in the world frame.
         * @param min_dist minimum distance to collect samples from the sensor.
         * @param max_dist maximum distance to collect samples from the sensor.
         * @param free_sampling_margin margin between free samples and hit samples.
         * @param free_points_per_meter number of free points to sample per meter.
         * @param hit_indices indices of the points that are occupied (hit) in the point cloud.
         * @param rays information of rays to sample.
         */
        static void
        CollectRays(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            const std::vector<long> &point_indices,
            const AabbD &map_boundary,
            Dtype min_dist,
            Dtype max_dist,
            Dtype free_sampling_margin,
            Dtype free_points_per_meter,
            std::vector<long> &hit_indices,
            std::vector<RayInfo> &rays);

        /**
         *
         * @param rays information of rays to sample.
         * @param generator random number generator.
         * @param random if true, randomly sample the rays. Used rays will be moved to the back of
         * the ray list.
         * @param num_hit_to_sample number of hit points to sample.
         * @param num_free_to_sample number of free points to sample.
         * @param num_hit number of hit points generated.
         * @param num_samples number of samples generated.
         * @param dataset_points points in the dataset.
         * @param dataset_labels labels of the points in the dataset.
         * @return Number of rays used to generate samples.
         */
        [[nodiscard]] static std::size_t
        GenerateSamples(
            std::vector<RayInfo> &rays,
            std::mt19937_64 &generator,
            bool random,
            long num_hit_to_sample,
            long num_free_to_sample,
            long &num_hit,
            long &num_samples,
            MatrixDX &dataset_points,
            VectorX &dataset_labels);

        /**
         * Generate a dataset of {x, y} where x is the position and y is the occupancy label (1 for
         * occupied, 0 for free).
         * @param sensor_position the position of the sensor in the world frame.
         * @param points point cloud in the world frame of the sensor measurement.
         * @param point_indices indices of the points in the point cloud that are valid for dataset.
         * If empty, all points will be used.
         * @param map_boundary the boundary of the map in the world frame.
         * @param generator random number generator.
         * @param min_distance minimum distance to collect samples from the sensor.
         * @param max_distance maximum distance to collect samples from the sensor.
         * @param free_sampling_margin margin between free samples and hit samples.
         * @param free_points_per_meter number of free points to sample per meter.
         * @param max_dataset_size maximum number of points in the dataset. -1 means no limit.
         * @param num_samples number of points in the dataset.
         * @param dataset_points points in the dataset.
         * @param dataset_labels labels of the points in the dataset.
         * @param hit_indices indices of the points that are occupied.
         * @return
         */
        static void
        GenerateDataset(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            const std::vector<long> &point_indices,
            const AabbD &map_boundary,
            std::mt19937_64 &generator,
            Dtype min_distance,
            Dtype max_distance,
            Dtype free_sampling_margin,
            Dtype free_points_per_meter,
            long max_dataset_size,
            long &num_samples,
            MatrixDX &dataset_points,
            VectorX &dataset_labels,
            std::vector<long> &hit_indices);
    };

    extern template class OccupancyMap<double, 3>;
    extern template class OccupancyMap<double, 2>;
    extern template class OccupancyMap<float, 3>;
    extern template class OccupancyMap<float, 2>;
}  // namespace erl::geometry
