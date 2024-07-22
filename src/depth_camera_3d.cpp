#include "erl_geometry/depth_camera_3d.hpp"

#include "erl_common/random.hpp"

namespace erl::geometry {
    Eigen::MatrixX<Eigen::Vector3d>
    DepthCamera3D::GetRayDirectionsInFrame() const {
        Eigen::MatrixX<Eigen::Vector3d> directions(m_setting_->image_height, m_setting_->image_width);

#pragma omp parallel for default(none) shared(directions, Eigen::Dynamic)
        for (int u = 0; u < m_setting_->image_width; ++u) {
            const double xu = -(static_cast<double>(u) - m_setting_->camera_cx) / m_setting_->camera_fx;
            for (int v = 0; v < m_setting_->image_height; ++v) {
                Eigen::Vector3d &dir_frame = directions(v, u);
                // camera normalized coordinates, i.e. depth = 1.0
                // dir_frame << (double(u) - m_setting_->camera_cx) / m_setting_->camera_fx,
                //              (double(v) - m_setting_->camera_cy) / m_setting_->camera_fy,
                //              1.0;
                // transform from optical frame to camera frame
                // cRo = [0, 0, 1; -1, 0, 0; 0, -1, 0];
                // clang-format off
                dir_frame << 1.0,
                             xu,
                             -(static_cast<double>(v) - m_setting_->camera_cy) / m_setting_->camera_fy;
                // clang-format on
            }
        }

        return directions;
    }

    std::tuple<Eigen::Matrix3d, Eigen::Vector3d>
    DepthCamera3D::GetExtrinsicMatrix(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) const {
        Eigen::Vector3d t = orientation * kCameraToOptical.topRightCorner<3, 1>() + translation;
        Eigen::Matrix3d r = orientation * kCameraToOptical.topLeftCorner<3, 3>();
        return {std::move(r), std::move(t)};
    }

}  // namespace erl::geometry
