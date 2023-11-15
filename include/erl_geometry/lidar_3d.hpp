#pragma once

#include "erl_common/yaml.hpp"
#include "erl_common/random.hpp"
#include "erl_geometry/azimuth_elevation.hpp"

#include <open3d/t/geometry/RaycastingScene.h>

namespace erl::geometry {

    class Lidar3D {

    public:
        struct Setting : common::Yamlable<Setting> {

            // default setting is from: https://velodynelidar.com/wp-content/uploads/2019/12/63-9229_Rev-K_Puck-_Datasheet_Web.pdf

            double azimuth_min = -M_PI;
            double azimuth_max = M_PI;
            double elevation_min = -M_PI / 12;  // 15 degree
            double elevation_max = M_PI / 12;   // 15 degree
            int num_azimuth_lines = 900;        // angular resolution: 0.4 degree
            int num_elevation_lines = 16;       // angular resolution: 2 degree
            bool add_noise = false;             // add noise to range measurements
            double range_stddev = 0.03;         // typical range error: 3cm
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        open3d::t::geometry::RaycastingScene *m_scene_ = nullptr;

    public:
        Lidar3D() = delete;

        Lidar3D(std::shared_ptr<Setting> setting, const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : m_setting_(std::move(setting)),
              m_scene_(o3d_scene.get()) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        Lidar3D(std::shared_ptr<Setting> setting, open3d::t::geometry::RaycastingScene *o3d_scene)
            : m_setting_(std::move(setting)),
              m_scene_(o3d_scene) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        [[nodiscard]] inline std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] inline Eigen::VectorXd
        GetAzimuthAngles() const {
            return Eigen::VectorXd::LinSpaced(m_setting_->num_azimuth_lines, m_setting_->azimuth_min, m_setting_->azimuth_max);
        }

        [[nodiscard]] inline Eigen::VectorXd
        GetElevationAngles() const {
            return Eigen::VectorXd::LinSpaced(m_setting_->num_elevation_lines, m_setting_->elevation_min, m_setting_->elevation_max);
        }

        [[nodiscard]] inline Eigen::MatrixX<Eigen::Vector3d>
        GetRayDirectionsInFrame() const {
            Eigen::VectorXd azimuth_angles = GetAzimuthAngles();
            Eigen::VectorXd elevation_angles = GetElevationAngles();

            Eigen::MatrixX<Eigen::Vector3d> directions(m_setting_->num_azimuth_lines, m_setting_->num_elevation_lines);
            for (int azimuth_idx = 0; azimuth_idx < m_setting_->num_azimuth_lines; ++azimuth_idx) {
                double azimuth = azimuth_angles[azimuth_idx];
                for (int elevation_idx = 0; elevation_idx < m_setting_->num_elevation_lines; ++elevation_idx) {
                    directions(azimuth_idx, elevation_idx) = AzimuthElevationToDirection(azimuth, elevation_angles[elevation_idx]);
                }
            }

            return directions;
        }

        [[nodiscard]] inline const open3d::t::geometry::RaycastingScene &
        GetScene() const {
            return *m_scene_;
        }

        [[nodiscard]] Eigen::MatrixXd
        Scan(const Eigen::Ref<const Eigen::Matrix3d> &orientation, const Eigen::Ref<const Eigen::Vector3d> &translation) const;
    };
}  // namespace erl::geometry

namespace YAML {

    template<>
    struct convert<erl::geometry::Lidar3D::Setting> {

        static Node
        encode(const erl::geometry::Lidar3D::Setting &rhs) {
            Node node;
            node["azimuth_min"] = rhs.azimuth_min;
            node["azimuth_max"] = rhs.azimuth_max;
            node["elevation_min"] = rhs.elevation_min;
            node["elevation_max"] = rhs.elevation_max;
            node["num_azimuth_lines"] = rhs.num_azimuth_lines;
            node["num_elevation_lines"] = rhs.num_elevation_lines;
            node["add_noise"] = rhs.add_noise;
            node["range_stddev"] = rhs.range_stddev;
            return node;
        }

        static bool
        decode(const Node &node, erl::geometry::Lidar3D::Setting &rhs) {
            rhs.azimuth_min = node["azimuth_min"].as<double>();
            rhs.azimuth_max = node["azimuth_max"].as<double>();
            rhs.elevation_min = node["elevation_min"].as<double>();
            rhs.elevation_max = node["elevation_max"].as<double>();
            rhs.num_azimuth_lines = node["num_azimuth_lines"].as<int>();
            rhs.num_elevation_lines = node["num_elevation_lines"].as<int>();
            rhs.add_noise = node["add_noise"].as<bool>();
            rhs.range_stddev = node["range_stddev"].as<double>();
            return true;
        }
    };

    inline Emitter &
    operator<<(Emitter &out, const erl::geometry::Lidar3D::Setting &rhs) {
        out << BeginMap;
        out << Key << "azimuth_min" << Value << rhs.azimuth_min;
        out << Key << "azimuth_max" << Value << rhs.azimuth_max;
        out << Key << "elevation_min" << Value << rhs.elevation_min;
        out << Key << "elevation_max" << Value << rhs.elevation_max;
        out << Key << "num_azimuth_lines" << Value << rhs.num_azimuth_lines;
        out << Key << "num_elevation_lines" << Value << rhs.num_elevation_lines;
        out << Key << "add_noise" << Value << rhs.add_noise;
        out << Key << "range_stddev" << Value << rhs.range_stddev;
        out << EndMap;
        return out;
    }

}  // namespace YAML
