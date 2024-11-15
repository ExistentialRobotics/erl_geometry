#include "erl_geometry/rgbd_frame_3d.hpp"

namespace erl::geometry {

    void
    RgbdFrame3D::ConvertToPointCloud(const bool in_world_frame, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) const {
        points.resize(m_hit_ray_indices_.size());
        colors.resize(m_hit_ray_indices_.size());
        for (std::size_t k = 0; k < m_hit_ray_indices_.size(); ++k) {
            const auto [i, j] = m_hit_ray_indices_[k];
            if (in_world_frame) {
                points[k] = m_hit_points_world_[k];
            } else {
                points[k] = m_end_pts_frame_(i, j);
            }
            const auto &color = m_rgb_.at<cv::Vec3b>(static_cast<int>(i), static_cast<int>(j));
            colors[k][0] = static_cast<double>(color[0]) / 255.0;
            colors[k][1] = static_cast<double>(color[1]) / 255.0;
            colors[k][2] = static_cast<double>(color[2]) / 255.0;
        }
    }

}  // namespace erl::geometry
