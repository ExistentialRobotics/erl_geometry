#include "erl_geometry/rgbd_frame_3d.hpp"

namespace erl::geometry {

    template<typename Dtype>
    RgbdFrame3D<Dtype>::RgbdFrame3D(std::shared_ptr<Setting> setting)
        : Super(std::move(setting)) {}

    template<typename Dtype>
    void
    RgbdFrame3D<Dtype>::UpdateRgbd(
        const Eigen::Ref<const Matrix3> &rotation,
        const Eigen::Ref<const Vector3> &translation,
        const MatrixX &depth,
        const cv::Mat &rgb) {
        Super::UpdateRanges(rotation, translation, depth);
        rgb.convertTo(m_rgb_, CV_8UC3);
    }

    template<typename Dtype>
    void
    RgbdFrame3D<Dtype>::ConvertToPointCloud(
        const bool in_world_frame,
        std::vector<Vector3> &points,
        std::vector<Vector3> &colors) const {
        points.resize(Super::m_hit_ray_indices_.size());
        colors.resize(Super::m_hit_ray_indices_.size());
        for (std::size_t k = 0; k < Super::m_hit_ray_indices_.size(); ++k) {
            const auto [i, j] = Super::m_hit_ray_indices_[k];
            if (in_world_frame) {
                points[k] = Super::m_hit_points_world_[k];
            } else {
                points[k] = Super::m_hit_points_frame_[k];
            }
            const auto &color = m_rgb_.at<cv::Vec3b>(static_cast<int>(i), static_cast<int>(j));
            colors[k][0] = static_cast<Dtype>(color[0]) / 255.0f;
            colors[k][1] = static_cast<Dtype>(color[1]) / 255.0f;
            colors[k][2] = static_cast<Dtype>(color[2]) / 255.0f;
        }
    }

    template class RgbdFrame3D<double>;
    template class RgbdFrame3D<float>;
}  // namespace erl::geometry
