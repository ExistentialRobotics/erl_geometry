#include "erl_geometry/lidar_3d.hpp"
#include "erl_common/random.hpp"

namespace erl::geometry {

    Eigen::MatrixX<Eigen::Vector3d>
    Lidar3D::GetRayDirectionsInFrame() const {
        Eigen::VectorXd azimuth_angles = GetAzimuthAngles();
        Eigen::VectorXd elevation_angles = GetElevationAngles();
        Eigen::MatrixX<Eigen::Vector3d> directions(m_setting_->num_azimuth_lines, m_setting_->num_elevation_lines);

#pragma omp parallel for collapse(2) default(none) shared(azimuth_angles, elevation_angles, directions, Eigen::Dynamic)
        for (int azimuth_idx = 0; azimuth_idx < m_setting_->num_azimuth_lines; ++azimuth_idx) {
            for (int elevation_idx = 0; elevation_idx < m_setting_->num_elevation_lines; ++elevation_idx) {
                directions(azimuth_idx, elevation_idx) = AzimuthElevationToDirection(azimuth_angles[azimuth_idx], elevation_angles[elevation_idx]);
            }
        }

        return directions;
    }
}  // namespace erl::geometry
