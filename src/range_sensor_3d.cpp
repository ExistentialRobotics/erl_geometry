#include "erl_geometry/range_sensor_3d.hpp"

#include "erl_common/random.hpp"

namespace erl::geometry {
    Eigen::MatrixXd
    RangeSensor3D::Scan(
        const Eigen::Ref<const Eigen::Matrix3d> &orientation,
        const Eigen::Ref<const Eigen::Vector3d> &translation,
        const bool add_noise,
        const double noise_stddev) const {
        Eigen::MatrixX<Eigen::Vector3d> directions = GetRayDirectionsInFrame();
        const long n_rows = directions.rows();
        const long n_cols = directions.cols();
        Eigen::MatrixXd scales(n_rows, n_cols);
        // ReSharper disable once CppDFAUnusedValue, CppDFAUnreadVariable
        Eigen::Vector3f ray_start = translation.cast<float>();
        // column major
        open3d::core::Tensor rays({n_rows, n_cols, 6}, open3d::core::Dtype::Float32);
        // ReSharper disable once CppDFAUnusedValue, CppDFAUnreadVariable
        auto *rays_ptr = rays.GetDataPtr<float>();
#pragma omp parallel for default(none) shared(n_rows, n_cols, orientation, directions, scales, ray_start, rays_ptr)
        for (long v = 0; v < n_cols; ++v) {
            const long base0 = v * n_rows;
            for (long u = 0; u < n_rows; ++u) {
                long base1 = base0 + u;

                Eigen::Vector3d direction = orientation * directions.data()[base1];
                double &scale = scales.data()[base1];
                scale = direction.norm();  // keep scale for later use

                base1 *= 6;
                rays_ptr[base1 + 0] = ray_start[0];
                rays_ptr[base1 + 1] = ray_start[1];
                rays_ptr[base1 + 2] = ray_start[2];

                // may not be normalized, e.g. depth camera uses camera normalized coordinates.
                rays_ptr[base1 + 3] = static_cast<float>(direction[0] / scale);
                rays_ptr[base1 + 4] = static_cast<float>(direction[1] / scale);
                rays_ptr[base1 + 5] = static_cast<float>(direction[2] / scale);
            }
        }

        const std::unordered_map<std::string, open3d::core::Tensor> cast_results = m_scene_->CastRays(rays);
        std::vector<float> ranges = cast_results.at("t_hit").ToFlatVector<float>();
        const std::vector<uint32_t> geometry_ids = cast_results.at("geometry_ids").ToFlatVector<uint32_t>();
        const uint32_t invalid_id = open3d::t::geometry::RaycastingScene::INVALID_ID();
        Eigen::MatrixXd ranges_mat_d(n_rows, n_cols);
        std::vector<uint64_t> random_seeds(n_cols);
        for (long i = 0; i < n_cols; ++i) { random_seeds[i] = common::g_random_engine(); }
#pragma omp parallel for default(none) shared(n_rows, n_cols, ranges, ranges_mat_d, geometry_ids, scales, invalid_id, random_seeds, add_noise, noise_stddev)
        for (long v = 0; v < n_cols; ++v) {
            const long base0 = v * n_rows;
            std::mt19937_64 generator(random_seeds[v]);
            std::normal_distribution<double> distribution(0, noise_stddev);
            for (long u = 0; u < n_rows; ++u) {
                if (const long i = base0 + u; geometry_ids[i] == invalid_id) {
                    ranges_mat_d(u, v) = std::numeric_limits<double>::infinity();
                } else {
                    double &range = ranges_mat_d(u, v);
                    range = static_cast<double>(ranges[i]) / scales.data()[i];
                    if (add_noise) { range += distribution(generator); }
                }
            }
        }

        return ranges_mat_d;
    }
}  // namespace erl::geometry
