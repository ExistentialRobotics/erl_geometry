#pragma once

#include "erl_common/angle_utils.hpp"

namespace erl::geometry {
    template<typename Dtype>
    YAML::Node
    Lidar3D<Dtype>::Setting::YamlConvertImpl::encode(const Setting &setting) {
        YAML::Node node;
        node["azimuth_min"] = setting.azimuth_min;
        node["azimuth_max"] = setting.azimuth_max;
        node["elevation_min"] = setting.elevation_min;
        node["elevation_max"] = setting.elevation_max;
        node["num_azimuth_lines"] = setting.num_azimuth_lines;
        node["num_elevation_lines"] = setting.num_elevation_lines;
        return node;
    }

    template<typename Dtype>
    bool
    Lidar3D<Dtype>::Setting::YamlConvertImpl::decode(const YAML::Node &node, Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.azimuth_min = node["azimuth_min"].as<Dtype>();
        setting.azimuth_max = node["azimuth_max"].as<Dtype>();
        setting.elevation_min = node["elevation_min"].as<Dtype>();
        setting.elevation_max = node["elevation_max"].as<Dtype>();
        setting.num_azimuth_lines = node["num_azimuth_lines"].as<long>();
        setting.num_elevation_lines = node["num_elevation_lines"].as<long>();
        return true;
    }

    template<typename Dtype>
    typename Lidar3D<Dtype>::Vector
    Lidar3D<Dtype>::GetAzimuthAngles() const {
        if (m_setting_->azimuth_max - m_setting_->azimuth_min == 2.0 * M_PI) {
            const Dtype d = 2.0 * M_PI / static_cast<Dtype>(m_setting_->num_azimuth_lines);
            return Vector::LinSpaced(m_setting_->num_azimuth_lines, m_setting_->azimuth_min, m_setting_->azimuth_max - d);
        }
        return Vector::LinSpaced(m_setting_->num_azimuth_lines, m_setting_->azimuth_min, m_setting_->azimuth_max);
    }

    template<typename Dtype>
    typename Lidar3D<Dtype>::Vector
    Lidar3D<Dtype>::GetElevationAngles() const {
        return Vector::LinSpaced(m_setting_->num_elevation_lines, m_setting_->elevation_min, m_setting_->elevation_max);
    }

    template<typename Dtype>
    Eigen::MatrixX<typename Lidar3D<Dtype>::Vector3>
    Lidar3D<Dtype>::GetRayDirectionsInFrame() const {
        Vector azimuth_angles = GetAzimuthAngles();
        Vector elevation_angles = GetElevationAngles();
        Eigen::MatrixX<Vector3> directions(m_setting_->num_azimuth_lines, m_setting_->num_elevation_lines);

        using namespace erl::common;

#pragma omp parallel for default(none) shared(azimuth_angles, elevation_angles, directions, Eigen::Dynamic)
        for (int elevation_idx = 0; elevation_idx < m_setting_->num_elevation_lines; ++elevation_idx) {
            for (int azimuth_idx = 0; azimuth_idx < m_setting_->num_azimuth_lines; ++azimuth_idx) {
                directions(azimuth_idx, elevation_idx) = AzimuthElevationToDirection<Dtype>(azimuth_angles[azimuth_idx], elevation_angles[elevation_idx]);
            }
        }

        return directions;
    }

    template<typename Dtype>
    std::tuple<typename Lidar3D<Dtype>::Matrix3, typename Lidar3D<Dtype>::Vector3>
    Lidar3D<Dtype>::GetOpticalPose(const Eigen::Ref<const Matrix3> &orientation, const Eigen::Ref<const Vector3> &translation) const {
        return {orientation, translation};
    }
}  // namespace erl::geometry
