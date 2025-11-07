#include "erl_geometry/occupancy_map.hpp"

#include "erl_geometry/intersection.hpp"

namespace erl::geometry {
    template<typename Dtype, int Dim>
    void
    OccupancyMap<Dtype, Dim>::CollectRays(
        const Eigen::Ref<const VectorD> &sensor_position,
        const Eigen::Ref<const MatrixDX> &points,
        const std::vector<long> &point_indices,
        const AabbD &map_boundary,
        const Dtype min_dist,
        const Dtype max_dist,
        const Dtype free_sampling_margin,
        const Dtype free_points_per_meter,
        std::vector<long> &hit_indices,
        std::vector<RayInfo> &rays) {

        hit_indices.clear();

        auto npts = point_indices.empty() ? points.cols() : static_cast<long>(point_indices.size());

        for (long i = 0; i < npts; ++i) {
            long idx = point_indices.empty() ? i : point_indices[i];
            VectorD point = points.col(idx);
            VectorD v = point - sensor_position;
            Dtype dist = v.norm();
            v /= dist;  // normalize the vector
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
            if (!intersected || (d1 < 0 && d2 < 0) || (dist <= d1 && d1 <= d2)) { continue; }
            // check if the point is inside the map
            hit_flag = map_boundary.contains(point) && (dist <= max_dist) && (dist >= min_dist);
            if (is_inside) {  // the sensor_position is inside the map, d2 < 0 is useless
                d2 = std::min((1.0f - free_sampling_margin) * dist, d1);
                d1 = free_sampling_margin * dist;
            } else {
                d1 = std::max(free_sampling_margin * dist, d1);
                d2 = std::min((1.0f - free_sampling_margin) * dist, d2);
            }
            // number of free points to sample
            auto n = std::max(0l, static_cast<long>(std::round((d2 - d1) * free_points_per_meter)));
            d1 = std::min(std::max(d1, min_dist), max_dist);
            d2 = std::min(std::max(d2, min_dist), max_dist);
            d1 /= dist;
            d2 /= dist;
            if (hit_flag) { hit_indices.push_back(idx); }
            rays.emplace_back(sensor_position, point, hit_flag, n, d1, d2);
        }
    }

    template<typename Dtype, int Dim>
    std::size_t
    OccupancyMap<Dtype, Dim>::GenerateSamples(
        std::vector<RayInfo> &rays,
        std::mt19937_64 &generator,
        const bool random,
        const long num_hit_to_sample,
        const long num_free_to_sample,
        long &num_hit,
        long &num_samples,
        MatrixDX &dataset_points,
        VectorX &dataset_labels) {

        const long n_to_sample = num_hit_to_sample + num_free_to_sample;
        if (dataset_points.cols() < n_to_sample) { dataset_points.resize(Dim, n_to_sample); }
        if (dataset_labels.size() < n_to_sample) { dataset_labels.resize(n_to_sample); }

        if (n_to_sample == 0 || rays.empty()) { return 0; }

        Dtype *points_ptr = dataset_points.data();
        Dtype *labels_ptr = dataset_labels.data();
        num_samples = 0;
        num_hit = 0;
        long n_free = 0;

        std::uniform_int_distribution<std::size_t> ray_distribution(0, rays.size() - 1);

        std::size_t i = 0;
        for (; i < rays.size(); ++i) {
            if (num_samples >= n_to_sample) { break; }  // already sampled enough points

            std::size_t idx1 = i;
            std::size_t idx2 = rays.size() - 1 - i;
            // move the used ray to the back of the list
            if (random) {
                idx1 = ray_distribution(generator) % (rays.size() - i);
                std::swap(rays[idx1], rays[idx2]);
            } else {
                std::swap(rays[idx1], rays[idx2]);
            }
            const auto &[p1, p2, hit_flag, num_free_points, d1, d2] = rays[idx2];

            if (hit_flag && num_hit < num_hit_to_sample) {
                std::memcpy(points_ptr, p2.data(), sizeof(Dtype) * Dim);  // save the hit point
                *labels_ptr++ = 1.0f;                                     // label as occupied
                points_ptr += Dim;  // move to the next position
                ++num_hit;
                ++num_samples;
            }

            const long n = std::min(num_free_points, num_free_to_sample - n_free);
            if (n <= 0) { continue; }  // no free points to sample

            n_free += n;
            num_samples += n;
            std::uniform_real_distribution<Dtype> distribution(d1, d2);
            for (long j = 0; j < n; ++j) {
                // sample a random distance within the range [d1, d2]
                Dtype r = distribution(generator);
                Dtype s = 1.0f - r;
                // compute the free point position
                for (long k = 0; k < Dim; ++k) { *points_ptr++ = p1[k] * s + p2[k] * r; }
                *labels_ptr++ = 0.0f;  // label as free
            }
        }

        ERL_DEBUG("Sampled {} points, {} hit points, {} free points", num_samples, num_hit, n_free);

        return i;
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

        // (origin, end, hit_flag, num_free_points, d1, d2)
        std::vector<RayInfo> rays;
        rays.reserve(point_indices.size());
        CollectRays(
            sensor_position,
            points,
            point_indices,
            map_boundary,
            min_distance,
            max_distance,
            free_sampling_margin,
            free_points_per_meter,
            hit_indices,
            rays);
        if (rays.empty()) { return; }

        long total_num_free_points = 0, total_num_hit_points = 0;
        for (const auto &ray: rays) {
            if (ray.hit_flag) { ++total_num_hit_points; }
            total_num_free_points += ray.num_free_points;
        }

        // check if the dataset size limit is exceeded.
        // if exceeded, adjust the number of points to sample.
        const long max_num_points = total_num_free_points + total_num_hit_points;
        const bool limit_exceeded = max_dataset_size > 0 && max_num_points > max_dataset_size;
        long num_hit_to_sample, num_free_to_sample;
        if (limit_exceeded) {
            num_hit_to_sample = max_dataset_size * total_num_hit_points / max_num_points;
            num_free_to_sample = max_dataset_size * total_num_free_points / max_num_points;
        } else {
            num_hit_to_sample = total_num_hit_points;
            num_free_to_sample = total_num_free_points;
        }

        (void) GenerateSamples(
            rays,
            generator,
            limit_exceeded,
            num_hit_to_sample,
            num_free_to_sample,
            num_hit_to_sample,
            num_samples,
            dataset_points,
            dataset_labels);
    }

    template class OccupancyMap<double, 3>;
    template class OccupancyMap<double, 2>;
    template class OccupancyMap<float, 3>;
    template class OccupancyMap<float, 2>;
}  // namespace erl::geometry
