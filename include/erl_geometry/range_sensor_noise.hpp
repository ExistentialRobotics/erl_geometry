#pragma once

#include "erl_common/random.hpp"
#include <thread>
#include <vector>
#include <random>

namespace erl::geometry {

    /**
     * @brief Add noise to range sensor data. For LiDAR data, gaussian noise is recommended
     * (noise_is_axial = false). For depth camera data, axial noise is recommended (noise_is_axial =
     * true).
     * @param range_data Pointer to the range data array.
     * @param n Number of elements in the range data.
     * @param noise_is_axial If true, use axial noise model; otherwise, use gaussian noise model.
     * @param noise_std Standard deviation of the gaussian noise (used when noise_is_axial = false).
     * @param noise_axial_scale Scale factor for the axial noise (used when noise_is_axial = true).
     */
    template<typename Dtype>
    void
    AddRangeSensorNoise(
        Dtype *range_data,
        const long n,
        const bool noise_is_axial = true,
        const Dtype noise_std = 0.01,
        const Dtype noise_axial_scale = 0.0025) {

        const std::size_t n_threads = std::thread::hardware_concurrency();
        std::vector<uint64_t> random_seeds(n_threads);
        for (std::size_t i = 0; i < n_threads; ++i) { random_seeds[i] = common::g_random_engine(); }

#pragma omp parallel for default(none) \
    shared(range_data, n, n_threads, random_seeds, noise_is_axial, noise_std, noise_axial_scale)
        for (std::size_t t = 0; t < n_threads; ++t) {
            const long bs = static_cast<long>(n / n_threads);
            const long start = bs * static_cast<long>(t);
            const long end = (t == n_threads - 1) ? n : start + bs;
            std::mt19937_64 generator(random_seeds[t]);
            if (noise_is_axial) {
                std::normal_distribution<Dtype> dist(0.0, 1.0);
                for (long i = start; i < end; ++i) {
                    Dtype &r = range_data[i];
                    if (r <= 0) { continue; }
                    r += dist(generator) * noise_axial_scale * r * r;
                    if (r < 0) { r = 0; }
                }
            } else {
                std::normal_distribution<Dtype> dist(0.0, noise_std);
                for (long i = start; i < end; ++i) {
                    Dtype &r = range_data[i];
                    if (r <= 0) { continue; }
                    r += dist(generator);
                    if (r < 0) { r = 0; }
                }
            }
        }
    }

}  // namespace erl::geometry
