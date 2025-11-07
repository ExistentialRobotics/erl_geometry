#pragma once

#include "erl_common/angle_utils.hpp"

namespace erl::geometry {

    template<typename Dtype>
    typename Lidar3D<Dtype>::VectorX
    Lidar3D<Dtype>::GetAzimuthAngles() const {
        if (m_setting_->azimuth_max - m_setting_->azimuth_min == 2.0 * M_PI) {
            const Dtype d = 2.0 * M_PI / static_cast<Dtype>(m_setting_->num_azimuth_lines);
            return VectorX::LinSpaced(
                m_setting_->num_azimuth_lines,
                m_setting_->azimuth_min,
                m_setting_->azimuth_max - d);
        }
        return VectorX::LinSpaced(
            m_setting_->num_azimuth_lines,
            m_setting_->azimuth_min,
            m_setting_->azimuth_max);
    }

    template<typename Dtype>
    typename Lidar3D<Dtype>::VectorX
    Lidar3D<Dtype>::GetElevationAngles() const {
        return VectorX::LinSpaced(
            m_setting_->num_elevation_lines,
            m_setting_->elevation_min,
            m_setting_->elevation_max);
    }

    template<typename Dtype>
    Eigen::MatrixX<typename Lidar3D<Dtype>::Vector3>
    Lidar3D<Dtype>::GetRayDirectionsInFrame() const {
        VectorX azimuth_angles = GetAzimuthAngles();
        VectorX elevation_angles = GetElevationAngles();
        Eigen::MatrixX<Vector3> directions(
            m_setting_->num_azimuth_lines,
            m_setting_->num_elevation_lines);

        using namespace erl::common;

#pragma omp parallel for default(none) \
    shared(azimuth_angles, elevation_angles, directions, Eigen::Dynamic)
        for (int elevation_idx = 0; elevation_idx < m_setting_->num_elevation_lines;
             ++elevation_idx) {
            for (int azimuth_idx = 0; azimuth_idx < m_setting_->num_azimuth_lines; ++azimuth_idx) {
                directions(azimuth_idx, elevation_idx) = AzimuthElevationToDirection<Dtype>(
                    azimuth_angles[azimuth_idx],
                    elevation_angles[elevation_idx]);
            }
        }

        return directions;
    }

    template<typename Dtype>
    std::tuple<typename Lidar3D<Dtype>::Matrix3, typename Lidar3D<Dtype>::Vector3>
    Lidar3D<Dtype>::GetOpticalPose(
        const Eigen::Ref<const Matrix3> &orientation,
        const Eigen::Ref<const Vector3> &translation) const {
        return {orientation, translation};
    }
}  // namespace erl::geometry
