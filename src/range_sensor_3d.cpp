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
        long m = directions.rows();
        long n = directions.cols();
        Eigen::MatrixXd scales(m, n);
        Eigen::Vector3f ray_start = translation.cast<float>();
        // column major
        open3d::core::Tensor rays({m, n, 6}, open3d::core::Dtype::Float32);
        auto *rays_ptr = rays.GetDataPtr<float>();
        for (long v = 0; v < n; ++v) {
            const long base0 = v * m;
            for (long u = 0; u < m; ++u) {
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
        for (std::size_t i = 0; i < geometry_ids.size(); ++i) {
            if (geometry_ids[i] == invalid_id) {
                ranges[i] = std::numeric_limits<float>::infinity();
            } else {
                ranges[i] /= static_cast<float>(scales.data()[i]);  // scale back
            }
        }

        const Eigen::Map<Eigen::MatrixXf> ranges_mat(ranges.data(), m, n);
        Eigen::MatrixXd ranges_mat_d = ranges_mat.cast<double>();

        if (add_noise) {
            std::normal_distribution<double> distribution(0, noise_stddev);
            for (int i = 0; i < ranges_mat_d.size(); ++i) { ranges_mat_d.data()[i] += distribution(common::g_random_engine); }
        }

        return ranges_mat_d;
    }
}  // namespace erl::geometry
