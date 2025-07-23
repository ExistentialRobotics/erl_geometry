#include "erl_geometry/occupancy_map.hpp"

#include "erl_geometry/intersection.hpp"

namespace erl::geometry {
    template<typename Dtype, int Dim>
    void
    OccupancyMap<Dtype, Dim>::GenerateRayInfos(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        const AabbD &map_boundary,
        const Dtype min_distance,
        const Dtype max_distance,
        const Dtype free_sampling_margin,
        const Dtype free_points_per_meter,
        std::vector<RayInfo> &infos,
        long &total_num_free_points,
        long &total_num_hit_points) {

        infos.clear();
        infos.reserve(points.cols() / 10);  // reserve space for the infos
        total_num_free_points = 0;
        total_num_hit_points = 0;

        auto npts = point_indices.empty() ? points.cols() : static_cast<long>(point_indices.size());

        for (long i = 0; i < npts; ++i) {
            long idx = point_indices.empty() ? i : point_indices[i];
            VectorD point = points.col(idx);
            VectorD v = point - sensor_position;
            Dtype v_norm = v.norm();
            v /= v_norm;  // normalize the vector
            Dtype d1 = 0;
            Dtype d2 = 0;
            bool hit_flag = false;
            bool intersected = false;
            bool is_inside = false;
            // compute intersection between the ray (point -> sensor_position) and the map boundary
            geometry::ComputeIntersectionBetweenRayAndAabb<Dtype, Dim>(
                sensor_position,
                v.cwiseInverse(),
                map_boundary.min(),
                map_boundary.max(),
                d1,
                d2,
                intersected,
                is_inside);
            // the ray does not intersect with the map boundary, or
            // hits a point outside the map, and v points away from the map; or
            // the ray hits a point outside the map, v points toward the map.
            if (!intersected || (d1 < 0 && d2 < 0) || (v_norm <= d1 && d1 <= d2)) { continue; }
            // check if the point is inside the map
            hit_flag =
                map_boundary.contains(point) && (v_norm < max_distance) && (v_norm > min_distance);
            if (is_inside) {  // the ray hits a point inside the map, d2 < 0 is useless
                d2 = std::min((1.0f - free_sampling_margin) * v_norm, d1);
                d1 = free_sampling_margin * v_norm;
            } else {
                d1 = std::max(free_sampling_margin * v_norm, d1);
                d2 = std::min((1.0f - free_sampling_margin) * v_norm, d2);
            }
            // number of free points to sample
            auto n = std::max(0l, static_cast<long>(std::ceil((d2 - d1) * free_points_per_meter)));
            if (n == 0 && !hit_flag) { continue; }  // no free points and the point is not hit
            total_num_free_points += n;             // count the number of free points to sample
            total_num_hit_points += static_cast<long>(hit_flag);  // count the number of hit points
            d1 = std::min(std::max(d1, min_distance), max_distance);
            d2 = std::min(std::max(d2, min_distance), max_distance);
            d1 /= v_norm;
            d2 /= v_norm;
            infos.emplace_back(idx, hit_flag, n, d1, d2);
        }
    }

    template<typename Dtype, int Dim>
    void
    OccupancyMap<Dtype, Dim>::GenerateSamples(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<RayInfo> &infos,
        std::mt19937_64 &generator,
        const bool random_infos,
        const long num_hit_to_sample,
        const long num_free_to_sample,
        long &num_samples,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {

        ERL_DEBUG_ASSERT(!infos.empty(), "infos is empty.");

        const long n_to_sample = num_hit_to_sample + num_free_to_sample;
        if (dataset_points.cols() < n_to_sample) { dataset_points.resize(Dim, n_to_sample); }
        if (dataset_labels.size() < n_to_sample) { dataset_labels.resize(n_to_sample); }

        std::vector<std::size_t> info_indices(infos.size());
        std::iota(info_indices.begin(), info_indices.end(), 0);
        if (random_infos) { std::shuffle(info_indices.begin(), info_indices.end(), generator); }

        Dtype *points_ptr = dataset_points.data();
        Dtype *labels_ptr = dataset_labels.data();
        num_samples = 0;
        long n_hit = 0, n_free = 0;
        hit_indices.clear();

        for (std::size_t i = 0; i < infos.size(); ++i) {
            if (num_samples >= n_to_sample) { break; }  // already sampled enough points

            std::size_t idx = i;
            if (random_infos) { idx = info_indices[i]; }
            const auto &[point_index, hit_flag, num_free_points, d1, d2] = infos[idx];
            const Dtype *point_ptr = points.col(point_index).data();

            if (hit_flag && n_hit < num_hit_to_sample) {
                std::memcpy(points_ptr, point_ptr, sizeof(Dtype) * Dim);  // save the hit point
                *labels_ptr++ = 1.0f;                                     // label as occupied
                points_ptr += Dim;  // move to the next position
                ++n_hit;
                ++num_samples;
                hit_indices.push_back(point_index);  // add the index to the list
            }

            const long n = std::min(num_free_points, num_free_to_sample - n_free);
            if (n <= 0) { continue; }  // no free points to sample

            n_free += n;
            num_samples += n;
            std::uniform_real_distribution<Dtype> distribution(d1, d2);
            for (long j = 0; j < n; ++j) {
                // sample a random distance within the range [d1, d2]
                Dtype r = distribution(generator);
                Dtype s = 1 - r;
                for (long k = 0; k < Dim; ++k) {  // compute the free point position
                    *points_ptr++ = sensor_position[k] * s + point_ptr[k] * r;
                }
                *labels_ptr++ = 0.0f;  // label as free
            }
        }

        ERL_DEBUG("Sampled {} points, {} hit points, {} free points.", num_samples, n_hit, n_free);
    }

    template<typename Dtype, int Dim>
    void
    OccupancyMap<Dtype, Dim>::GenerateDataset(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        const AabbD &map_boundary,
        std::mt19937_64 &generator,
        const Dtype min_distance,
        const Dtype max_distance,
        const Dtype free_sampling_margin,
        const Dtype free_points_per_meter,
        const long max_dataset_size,
        long &num_samples,
        MatrixDX &dataset_points,
        VectorX &dataset_labels,
        std::vector<long> &hit_indices) {

        // 1. check if the ray intersects with the map boundary.
        // 2. compute the range to sample free points and the number of points to sample.
        // 3. sample the free points uniformly within the range.
        // 4. return the result.

        // tuple of (point_index, hit_flag, num_free_points, d1, d2)
        std::vector<RayInfo> infos;
        long max_num_free_points = 0, max_num_hit_points = 0;
        GenerateRayInfos(
            sensor_position,
            points,
            point_indices,
            map_boundary,
            min_distance,
            max_distance,
            free_sampling_margin,
            free_points_per_meter,
            infos,
            max_num_free_points,
            max_num_hit_points);
        if (infos.empty()) { return; }

        // check if the dataset size limit is exceeded.
        // if exceeded, adjust the number of points to sample.
        const long max_num_points = max_num_free_points + max_num_hit_points;
        const bool limit_exceeded = max_dataset_size > 0 && max_num_points > max_dataset_size;
        long num_hit_to_sample, num_free_to_sample;
        if (limit_exceeded) {
            num_hit_to_sample = max_dataset_size * max_num_hit_points / max_num_points;
            num_free_to_sample = max_dataset_size * max_num_free_points / max_num_points;
        } else {
            num_hit_to_sample = max_num_hit_points;
            num_free_to_sample = max_num_free_points;
        }

        GenerateSamples(
            sensor_position,
            points,
            infos,
            generator,
            limit_exceeded /*random_infos*/,
            num_hit_to_sample,
            num_free_to_sample,
            num_samples,
            dataset_points,
            dataset_labels,
            hit_indices);
    }

    template class OccupancyMap<double, 3>;
    template class OccupancyMap<double, 2>;
    template class OccupancyMap<float, 3>;
    template class OccupancyMap<float, 2>;
}  // namespace erl::geometry
