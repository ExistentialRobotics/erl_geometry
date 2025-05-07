#pragma once

#include "range_sensor_frame_3d.hpp"

#include "erl_common/logging.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>

namespace erl::geometry {

    template<typename Dtype>
    class LidarFrame3D : public RangeSensorFrame3D<Dtype> {
        friend class LidarFramePartition3D;

    public:
        using Super = RangeSensorFrame3D<Dtype>;
        using MatrixX = Eigen::MatrixX<Dtype>;
        using Matrix3 = Eigen::Matrix3<Dtype>;
        using Matrix2X = Eigen::Matrix2X<Dtype>;
        using VectorX = Eigen::VectorX<Dtype>;
        using Vector3 = Eigen::Vector3<Dtype>;
        using Vector2 = Eigen::Vector2<Dtype>;

        struct Setting : public common::Yamlable<Setting, typename Super::Setting> {
            Dtype azimuth_min = -M_PI;
            Dtype azimuth_max = M_PI;
            Dtype elevation_min = -M_PI * 0.5f;
            Dtype elevation_max = M_PI * 0.5f;
            long num_azimuth_lines = 360;
            long num_elevation_lines = 180;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
        };

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;

    public:
        explicit LidarFrame3D(std::shared_ptr<Setting> setting);

        void
        Reset() {
            this->m_max_valid_range_ = std::numeric_limits<Dtype>::min();
        }

        [[nodiscard]] bool
        PointIsInFrame(const Vector3 &xyz_frame) const override;

        [[nodiscard]] Vector2
        ComputeFrameCoords(const Vector3 &dir_frame) const override;

        void
        UpdateRanges(
            const Eigen::Ref<const Matrix3> &rotation,
            const Eigen::Ref<const Vector3> &translation,
            MatrixX ranges) override;

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] long
        GetNumAzimuthLines() const {
            return this->m_frame_coords_.rows();
        }

        [[nodiscard]] long
        GetNumElevationLines() const {
            return this->m_frame_coords_.cols();
        }

        [[nodiscard]] bool
        operator==(const Super &other) const override;

        [[nodiscard]] bool
        Write(std::ostream &s) const override;

        [[nodiscard]] bool
        Read(std::istream &s) override;
    };

    using LidarFrame3Dd = LidarFrame3D<double>;
    using LidarFrame3Df = LidarFrame3D<float>;

}  // namespace erl::geometry

#include "lidar_frame_3d.tpp"

template<>
struct YAML::convert<erl::geometry::LidarFrame3Dd::Setting>
    : erl::geometry::LidarFrame3Dd::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::LidarFrame3Df::Setting>
    : erl::geometry::LidarFrame3Df::Setting::YamlConvertImpl {};
