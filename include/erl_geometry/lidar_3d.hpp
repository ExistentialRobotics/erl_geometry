#pragma once

#include "range_sensor_3d.hpp"

#include "erl_common/yaml.hpp"

#include <open3d/t/geometry/RaycastingScene.h>

namespace erl::geometry {

    class Lidar3D : public RangeSensor3D {

    public:
        struct Setting : common::Yamlable<Setting> {

            // default setting is from: https://velodynelidar.com/wp-content/uploads/2019/12/63-9229_Rev-K_Puck-_Datasheet_Web.pdf

            double azimuth_min = -M_PI;
            double azimuth_max = M_PI;
            double elevation_min = -M_PI / 12;  // 15 degree
            double elevation_max = M_PI / 12;   // 15 degree
            int num_azimuth_lines = 900;        // angular resolution: 0.4 degree
            int num_elevation_lines = 16;       // angular resolution: 2 degree
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        Lidar3D() = delete;

        Lidar3D(std::shared_ptr<Setting> setting, const Eigen::Ref<const Eigen::Matrix3Xd> &vertices, const Eigen::Ref<const Eigen::Matrix3Xi> &triangles)
            : RangeSensor3D(vertices, triangles),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        Lidar3D(std::shared_ptr<Setting> setting, const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles)
            : RangeSensor3D(vertices, triangles),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        Lidar3D(std::shared_ptr<Setting> setting, const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : RangeSensor3D(o3d_scene),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Eigen::VectorXd
        GetAzimuthAngles() const {
            if (m_setting_->azimuth_max - m_setting_->azimuth_min == 2.0 * M_PI) {
                const double d = 2.0 * M_PI / m_setting_->num_azimuth_lines;
                return Eigen::VectorXd::LinSpaced(m_setting_->num_azimuth_lines, m_setting_->azimuth_min, m_setting_->azimuth_max - d);
            }
            return Eigen::VectorXd::LinSpaced(m_setting_->num_azimuth_lines, m_setting_->azimuth_min, m_setting_->azimuth_max);
        }

        [[nodiscard]] Eigen::VectorXd
        GetElevationAngles() const {
            return Eigen::VectorXd::LinSpaced(m_setting_->num_elevation_lines, m_setting_->elevation_min, m_setting_->elevation_max);
        }

        [[nodiscard]] Eigen::MatrixX<Eigen::Vector3d>
        GetRayDirectionsInFrame() const override;
    };
}  // namespace erl::geometry

// ReSharper disable CppInconsistentNaming
template<>
struct YAML::convert<erl::geometry::Lidar3D::Setting> {

    static Node
    encode(const erl::geometry::Lidar3D::Setting &rhs) {
        Node node;
        node["azimuth_min"] = rhs.azimuth_min;
        node["azimuth_max"] = rhs.azimuth_max;
        node["elevation_min"] = rhs.elevation_min;
        node["elevation_max"] = rhs.elevation_max;
        node["num_azimuth_lines"] = rhs.num_azimuth_lines;
        node["num_elevation_lines"] = rhs.num_elevation_lines;
        return node;
    }

    static bool
    decode(const Node &node, erl::geometry::Lidar3D::Setting &rhs) {
        if (!node.IsMap()) { return false; }
        rhs.azimuth_min = node["azimuth_min"].as<double>();
        rhs.azimuth_max = node["azimuth_max"].as<double>();
        rhs.elevation_min = node["elevation_min"].as<double>();
        rhs.elevation_max = node["elevation_max"].as<double>();
        rhs.num_azimuth_lines = node["num_azimuth_lines"].as<int>();
        rhs.num_elevation_lines = node["num_elevation_lines"].as<int>();
        return true;
    }
};  // namespace YAML

// ReSharper restore CppInconsistentNaming
