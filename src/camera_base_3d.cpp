#include "erl_geometry/camera_base_3d.hpp"

namespace erl::geometry {
    Eigen::MatrixX<Eigen::Vector3d>
    CameraBase3D::ComputeRayDirectionsInFrame(
        const long image_height,
        const long image_width,
        const double camera_fx,
        const double camera_fy,
        const double camera_cx,
        const double camera_cy) {
        Eigen::MatrixX<Eigen::Vector3d> directions(image_height, image_width);

#pragma omp parallel for default(none) shared(image_height, image_width, camera_fx, camera_fy, camera_cx, camera_cy, directions, Eigen::Dynamic)
        for (int u = 0; u < image_width; ++u) {
            const double xu = -(static_cast<double>(u) - camera_cx) / camera_fx;
            for (int v = 0; v < image_height; ++v) {
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
                             -(static_cast<double>(v) - camera_cy) / camera_fy;
                // clang-format on
            }
        }

        return directions;
    }

}  // namespace erl::geometry
