#include "erl_geometry/lidar_3d.hpp"

namespace erl::geometry {
    Eigen::MatrixXd
    Lidar3D::Scan(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) const {

        Eigen::MatrixX<Eigen::Vector3d> directions = GetRayDirectionsInFrame();
        open3d::core::Tensor rays({m_setting_->num_azimuth_lines, m_setting_->num_elevation_lines, 6}, open3d::core::Dtype::Float32);
        auto *rays_ptr = rays.GetDataPtr<float>();

        Eigen::Vector3f ray_start = translation.cast<float>();
        // column major
        for (int elevation_idx = 0; elevation_idx < m_setting_->num_elevation_lines; ++elevation_idx) {
            int base0 = elevation_idx * m_setting_->num_azimuth_lines * 6;
            for (int azimuth_idx = 0; azimuth_idx < m_setting_->num_azimuth_lines; ++azimuth_idx) {
                int base1 = base0 + azimuth_idx * 6;
                Eigen::Vector3f direction = (orientation * directions(azimuth_idx, elevation_idx)).cast<float>();
                rays_ptr[base1 + 0] = ray_start[0];
                rays_ptr[base1 + 1] = ray_start[1];
                rays_ptr[base1 + 2] = ray_start[2];
                rays_ptr[base1 + 3] = direction[0];
                rays_ptr[base1 + 4] = direction[1];
                rays_ptr[base1 + 5] = direction[2];
            }
        }

        std::unordered_map<std::string, open3d::core::Tensor> cast_results = m_scene_->CastRays(rays);
        std::vector<float> ranges = cast_results.at("t_hit").ToFlatVector<float>();
        std::vector<uint32_t> geometry_ids = cast_results.at("geometry_ids").ToFlatVector<uint32_t>();
        uint32_t invalid_id = open3d::t::geometry::RaycastingScene::INVALID_ID();
        for (std::size_t i = 0; i < geometry_ids.size(); ++i) {
            if (geometry_ids[i] == invalid_id) { ranges[i] = std::numeric_limits<float>::infinity(); }
        }

        Eigen::Map<Eigen::MatrixXf> ranges_mat(ranges.data(), m_setting_->num_azimuth_lines, m_setting_->num_elevation_lines);
        Eigen::MatrixXd ranges_mat_d = ranges_mat.cast<double>();

        if (m_setting_->add_noise) {
            std::normal_distribution<double> distribution(0, m_setting_->range_stddev);
            for (int i = 0; i < ranges_mat_d.size(); ++i) { ranges_mat_d.data()[i] += distribution(common::g_random_engine); }
        }
        return ranges_mat_d;
    }
}  // namespace erl::geometry
