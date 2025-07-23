#pragma once

#include "aabb.hpp"

#include "erl_common/eigen.hpp"

#include <random>

namespace erl::geometry {
    template<typename Dtype, int Dim>
    struct OccupancyMap {
        struct RayInfo {
            long point_index = -1;
            bool hit_flag = false;
            long num_free_points = 0;
            Dtype d1 = 0.0f;
            Dtype d2 = 0.0f;

            RayInfo(
                const long point_index,
                const bool hit_flag,
                const long num_free_points,
                const Dtype d1,
                const Dtype d2)
                : point_index(point_index),
                  hit_flag(hit_flag),
                  num_free_points(num_free_points),
                  d1(d1),
                  d2(d2) {}
        };

        using VectorD = Eigen::Vector<Dtype, Dim>;
        using VectorX = Eigen::VectorX<Dtype>;
        using MatrixDX = Eigen::Matrix<Dtype, Dim, Eigen::Dynamic>;
        using AabbD = Aabb<Dtype, Dim>;

        /**
         *
         * @param sensor_position the position of the sensor in the world frame.
         * @param points the point cloud in the world frame of the sensor measurement.
         * @param point_indices the indices of the points in the point cloud to sample.
         * @param map_boundary map boundary in the world frame.
         * @param min_distance minimum distance to collect samples from the sensor.
         * @param max_distance maximum distance to collect samples from the sensor.
         * @param free_sampling_margin margin between free samples and hit samples.
         * @param free_points_per_meter number of free points to sample per meter.
         * @param infos information of rays to sample.
         * @param total_num_free_points total number of free points available to sample.
         * @param total_num_hit_points total number of hit points available to sample.
         */
        static void
        GenerateRayInfos(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            const std::vector<long> &point_indices,
            const AabbD &map_boundary,
            Dtype min_distance,
            Dtype max_distance,
            Dtype free_sampling_margin,
            Dtype free_points_per_meter,
            std::vector<RayInfo> &infos,
            long &total_num_free_points,
            long &total_num_hit_points);

        /**
         *
         * @param sensor_position the position of the sensor in the world frame.
         * @param points the point cloud in the world frame of the sensor measurement.
         * @param infos information of rays to sample.
         * @param generator random number generator.
         * @param random_infos if true, shuffle the infos before sampling.
         * @param num_hit_to_sample number of hit points to sample.
         * @param num_free_to_sample number of free points to sample.
         * @param num_samples number of samples generated.
         * @param dataset_points points in the dataset.
         * @param dataset_labels labels of the points in the dataset.
         * @param hit_indices indices of the points that are occupied (hit) in the point cloud.
         */
        static void
        GenerateSamples(
            const Eigen::Ref<const VectorD> &sensor_position,
            const Eigen::Ref<const MatrixDX> &points,
            const std::vector<RayInfo> &infos,
            std::mt19937_64 &generator,
            bool random_infos,
            long num_hit_to_sample,
            long num_free_to_sample,
            long &num_samples,
            MatrixDX &dataset_points,
            VectorX &dataset_labels,
            std::vector<long> &hit_indices);

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
