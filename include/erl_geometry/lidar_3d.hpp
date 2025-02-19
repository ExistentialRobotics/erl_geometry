#pragma once

#include "range_sensor_3d.hpp"

#include "erl_common/random.hpp"
#include "erl_common/yaml.hpp"

namespace erl::geometry {

    template<typename Dtype>
    class Lidar3D : public RangeSensor3D<Dtype> {
    public:
        using Super = RangeSensor3D<Dtype>;
        using Matrix = Eigen::MatrixX<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Vector = Eigen::VectorX<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;

        struct Setting : common::Yamlable<Setting> {

            // default setting is from: https://velodynelidar.com/wp-content/uploads/2019/12/63-9229_Rev-K_Puck-_Datasheet_Web.pdf

            Dtype azimuth_min = -M_PI;
            Dtype azimuth_max = M_PI;
            Dtype elevation_min = -M_PI / 12;  // 15 degree
            Dtype elevation_max = M_PI / 12;   // 15 degree
            long num_azimuth_lines = 900;      // angular resolution: 0.4 degree
            long num_elevation_lines = 16;     // angular resolution: 2 degree

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
        };

    private:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        explicit Lidar3D(std::shared_ptr<Setting> setting)
            : m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        Lidar3D(std::shared_ptr<Setting> setting, const std::shared_ptr<open3d::t::geometry::RaycastingScene> &o3d_scene)
            : Super(o3d_scene),
              m_setting_(std::move(setting)) {
            ERL_ASSERTM(m_setting_ != nullptr, "setting is nullptr.");
        }

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] Vector
        GetAzimuthAngles() const;

        [[nodiscard]] Vector
        GetElevationAngles() const;

        [[nodiscard]] Eigen::MatrixX<Vector3>
        GetRayDirectionsInFrame() const override;

        [[nodiscard]] std::tuple<Matrix3, Vector3>
        GetOpticalPose(const Eigen::Ref<const Matrix3> &orientation, const Eigen::Ref<const Vector3> &translation) const override;
    };

    using Lidar3Dd = Lidar3D<double>;
    using Lidar3Df = Lidar3D<float>;
}  // namespace erl::geometry

#include "lidar_3d.tpp"

template<>
struct YAML::convert<erl::geometry::Lidar3Dd::Setting> : erl::geometry::Lidar3Dd::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::Lidar3Df::Setting> : erl::geometry::Lidar3Df::Setting::YamlConvertImpl {};
