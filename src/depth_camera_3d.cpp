#include "erl_geometry/depth_camera_3d.hpp"
#include "erl_common/random.hpp"

namespace erl::geometry {
    Eigen::MatrixX<Eigen::Vector3d>
    DepthCamera3D::GetRayDirectionsInFrame() const {
        Eigen::MatrixX<Eigen::Vector3d> directions(m_setting_->image_height, m_setting_->image_width);

#pragma omp parallel for collapse(2) default(none) shared(directions, Eigen::Dynamic)
        for (int v = 0; v < m_setting_->image_height; ++v) {
            for (int u = 0; u < m_setting_->image_width; ++u) {
                Eigen::Vector3d &dir_frame = directions(v, u);
                // camera normalized coordinates, i.e. depth = 1.0
                // dir_frame << (double(u) - m_setting_->camera_cx) / m_setting_->camera_fx,
                //              (double(v) - m_setting_->camera_cy) / m_setting_->camera_fy,
                //              1.0;
                // transform from optical frame to camera frame
                // rRo = [0, 0, 1; -1, 0, 0; 0, -1, 0];
                // clang-format off
                dir_frame << 1.0,
                             -(double(u) - m_setting_->camera_cx) / m_setting_->camera_fx,
                             -(double(v) - m_setting_->camera_cy) / m_setting_->camera_fy;
                // clang-format on
            }
        }

        return directions;
    }
}  // namespace erl::geometry
