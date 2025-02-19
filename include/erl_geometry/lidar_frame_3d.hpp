#pragma once

#include "range_sensor_frame_3d.hpp"

#include "erl_common/angle_utils.hpp"
#include "erl_common/logging.hpp"
#include "erl_common/yaml.hpp"

#include <absl/container/flat_hash_map.h>

#include <utility>

namespace erl::geometry {

    template<typename Dtype>
    class LidarFrame3D : public RangeSensorFrame3D<Dtype> {
        friend class LidarFramePartition3D;

    public:
        using Super = RangeSensorFrame3D<Dtype>;
        using Matrix = typename Super::Matrix;
        using Matrix3 = typename Super::Matrix3;
        using Matrix2X = Eigen::Matrix2X<Dtype>;
        using Vector = typename Super::Vector;
        using Vector3 = typename Super::Vector3;
        using Vector2 = typename Super::Vector2;

        struct Setting : common::Yamlable<Setting, typename Super::Setting> {
            Dtype azimuth_min = -M_PI;
            Dtype azimuth_max = M_PI;
            Dtype elevation_min = -M_PI / 2;
            Dtype elevation_max = M_PI / 2;
            long num_azimuth_lines = 360;
            long num_elevation_lines = 180;

            struct YamlConvertImpl {
                static YAML::Node
                encode(const Setting &setting);

                static bool
                decode(const YAML::Node &node, Setting &setting);
            };
        };

        class Partition {
            LidarFrame3D *m_lidar_frame_ = nullptr;
            Matrix2X m_ray_indices_ = {};
        };

        // inline static const volatile bool kSettingRegistered = common::YamlableBase::Register<Setting>();

    private:
        inline static const std::string kFileHeader = fmt::format("# erl::geometry::LidarFrame3D<{}>", type_name<Dtype>());

    protected:
        std::shared_ptr<Setting> m_setting_ = nullptr;
        std::vector<Partition> m_partitions_ = {};
        bool m_partitioned_ = false;

    public:
        explicit LidarFrame3D(std::shared_ptr<Setting> setting);

        void
        Reset() {
            Super::m_max_valid_range_ = std::numeric_limits<Dtype>::min();
            m_partitioned_ = false;
        }

        [[nodiscard]] bool
        PointIsInFrame(const Vector3 &xyz_frame) const override {
            if (const Dtype range = xyz_frame.norm(); range < m_setting_->valid_range_min || range > m_setting_->valid_range_max) { return false; }
            return Super::CoordsIsInFrame(ComputeFrameCoords(xyz_frame.normalized()));
        }

        [[nodiscard]] Vector2
        ComputeFrameCoords(const Vector3 &dir_frame) const override {
            Vector2 frame_coords;
            common::DirectionToAzimuthElevation<Dtype>(dir_frame, frame_coords[0], frame_coords[1]);
            return frame_coords;
        }

        void
        UpdateRanges(const Eigen::Ref<const Matrix3> &rotation, const Eigen::Ref<const Vector3> &translation, Matrix ranges, bool partition_rays) override;

        [[nodiscard]] std::shared_ptr<const Setting>
        GetSetting() const {
            return m_setting_;
        }

        [[nodiscard]] long
        GetNumAzimuthLines() const {
            return Super::m_frame_coords_.rows();
        }

        [[nodiscard]] long
        GetNumElevationLines() const {
            return Super::m_frame_coords_.cols();
        }

        [[nodiscard]] bool
        IsPartitioned() const {
            return m_partitioned_;
        }

        [[nodiscard]] const std::vector<Partition> &
        GetPartitions() const {
            ERL_ASSERTM(m_partitioned_, "LidarFrame3D<Dtype>::GetPartitions() is called before partitioning.");
            return m_partitions_;
        }

        [[nodiscard]] bool
        operator==(const Super &other) const override;

        [[nodiscard]] bool
        Write(const std::string &filename) const override;

        [[nodiscard]] bool
        Write(std::ostream &s) const override;

        [[nodiscard]] bool
        Read(const std::string &filename) override;

        [[nodiscard]] bool
        Read(std::istream &s) override;

    protected:
        void
        PartitionRays();
    };

#include "lidar_frame_3d.tpp"

    using LidarFrame3Dd = LidarFrame3D<double>;
    using LidarFrame3Df = LidarFrame3D<float>;

    // ERL_REGISTER_RANGE_SENSOR_FRAME_3D(LidarFrame3Dd);
    // ERL_REGISTER_RANGE_SENSOR_FRAME_3D(LidarFrame3Df);

}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::LidarFrame3Dd::Setting> : erl::geometry::LidarFrame3Dd::Setting::YamlConvertImpl {};

template<>
struct YAML::convert<erl::geometry::LidarFrame3Df::Setting> : erl::geometry::LidarFrame3Df::Setting::YamlConvertImpl {};
